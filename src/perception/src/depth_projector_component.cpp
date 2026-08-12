#include "perception/depth_projector_component.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>   // FIXED: correct header for EuclideanClusterExtraction
#include <pcl/search/kdtree.h>                    // ADDED: needed for pcl::search::KdTree
#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/point_stamped.hpp>            // ADDED
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>        // ADDED: required for tf_buffer_->transform() to accept PointStamped

#include <algorithm>
#include <limits>
#include <functional>

namespace perception
{

DepthProjectorComponent::DepthProjectorComponent(const rclcpp::NodeOptions & options)
: Node("depth_projector_component", options)
{
  // load params from perception_params.yaml
  fx_ = declare_parameter<double>("camera.fx", 600.0);
  fy_ = declare_parameter<double>("camera.fy", 600.0);
  cx_ = declare_parameter<double>("camera.cx", 320.0);
  cy_ = declare_parameter<double>("camera.cy", 240.0);
  ground_dist_thresh_ = declare_parameter<double>("ground_removal.dist_thresh", 0.05);
  cluster_tolerance_ = declare_parameter<double>("clustering.tolerance", 0.3);
  min_cluster_size_ = declare_parameter<int>("clustering.min_size", 10);
  bbox_shrink_ratio_ = declare_parameter<double>("bbox.shrink_ratio", 0.15);
  camera_frame_ = declare_parameter<std::string>("frames.camera", "camera_link");
  lidar_frame_ = declare_parameter<std::string>("frames.lidar", "velodyne");
  base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
  grid_resolution_ = declare_parameter<double>("bev.grid_resolution", 0.5);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // FIXED: explicitly set SensorData QoS (Best Effort) to match the Velodyne
  // driver, avoiding the case where it builds/runs fine but the callback is
  // never called because of a QoS mismatch.
  cloud_sub_.subscribe(this, "/velodyne_points", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  tracks_sub_.subscribe(this, "/tracked_persons", rclcpp::QoS(10).get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), cloud_sub_, tracks_sub_);
  sync_->registerCallback(
    std::bind(&DepthProjectorComponent::syncCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  objects_pub_ = create_publisher<perception::msg::TrackedObject3DArray>(
    "/tracked_persons_depth", 10);

  RCLCPP_INFO(get_logger(), "DepthProjectorComponent Ready. Sub cloud=/velodyne_points, Sub tracks=/tracked_persons");
}

void DepthProjectorComponent::syncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & tracks_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // 0. Ground removal
  auto cloud_no_ground = removeGround(cloud);

  // 1. Transform lidar -> camera frame
  auto cloud_cam = transformCloud(
    cloud_no_ground, camera_frame_, lidar_frame_, cloud_msg->header.stamp);

  perception::msg::TrackedObject3DArray out_msg;
  out_msg.header = cloud_msg->header;
  out_msg.header.frame_id = base_frame_;
  out_msg.grid_resolution = static_cast<float>(grid_resolution_);
  out_msg.grid_origin.x = 0.0;
  out_msg.grid_origin.y = 0.0;
  out_msg.grid_origin.z = 0.0;

  for (const auto & det : tracks_msg->detections) {
    if (det.results.empty()) {
      continue;  // safety guard, avoid a crash if the hypothesis is missing
    }

    int track_id;
    try {
      track_id = std::stoi(det.results[0].hypothesis.class_id);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not parse track_id: %s", e.what());
      continue;
    }

    geometry_msgs::msg::Point pos_cam;
    if (!estimateDistanceForBbox(cloud_cam, det, pos_cam)) {
      continue;  // this bbox doesn't have enough valid lidar points
    }

    // Transform the object's position from the camera frame to base_frame
    geometry_msgs::msg::PointStamped pt_cam, pt_base;
    pt_cam.point = pos_cam;
    pt_cam.header.frame_id = camera_frame_;
    pt_cam.header.stamp = cloud_msg->header.stamp;

    try {
      tf_buffer_->transform(pt_cam, pt_base, base_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "TF transform to '%s' failed for track_id %d: %s",
        base_frame_.c_str(), track_id, ex.what());
      continue;
    }

    // FIXED: compute distance AFTER transforming to base_frame, using x,y
    // (the horizontal plane) — this is the correct meaning of "distance to follow".
    float distance = std::sqrt(
      pt_base.point.x * pt_base.point.x +
      pt_base.point.y * pt_base.point.y);

    perception::msg::TrackedObject3D obj;
    obj.track_id = track_id;
    obj.position = pt_base.point;
    obj.distance = distance;
    obj.grid_n = static_cast<int32_t>(std::floor(pt_base.point.x / grid_resolution_));
    obj.grid_m = static_cast<int32_t>(std::floor(pt_base.point.y / grid_resolution_));
    out_msg.objects.push_back(obj);
  }

  objects_pub_->publish(out_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthProjectorComponent::removeGround(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

  if (cloud->points.empty()) {
    return filtered;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(15.0 * M_PI / 180.0);
  seg.setDistanceThreshold(ground_dist_thresh_);
  seg.setMaxIterations(100);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coeff);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(get_logger(), "Ground removal: no ground plane found, returning the cloud unchanged.");
    *filtered = *cloud;
    return filtered;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true = keep the points that do NOT belong to the ground plane
  extract.filter(*filtered);
  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthProjectorComponent::transformCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & stamp)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup '%s' <- '%s' failed: %s",
      target_frame.c_str(), source_frame.c_str(), ex.what());
    return out;
  }
  Eigen::Affine3d transform = tf2::transformToEigen(tf_stamped);
  pcl::transformPointCloud(*cloud, *out, transform.matrix().cast<float>());
  return out;
}

bool DepthProjectorComponent::estimateDistanceForBbox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_cam,
  const vision_msgs::msg::Detection2D & det,
  geometry_msgs::msg::Point & out_position)
{
  // Shrink the bbox before filtering points, to avoid picking up
  // background/neighboring-object points near the bbox edges
  double bw = det.bbox.size_x * (1.0 - bbox_shrink_ratio_ * 2);
  double bh = det.bbox.size_y * (1.0 - bbox_shrink_ratio_ * 2);
  double cx_box = det.bbox.center.position.x;
  double cy_box = det.bbox.center.position.y;
  double x0 = cx_box - bw / 2, x1 = cx_box + bw / 2;
  double y0 = cy_box - bh / 2, y1 = cy_box + bh / 2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_bbox(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & pt : cloud_cam->points) {
    // camera_frame_ (camera_link) follows the ROS REP-103 convention
    // (x-forward, y-left, z-up). Real depth = pt.x. Convert to the optical
    // image axes (x-right, y-down, z-forward):
    if (pt.x <= 0.1) {
      continue;
    }
    double u = fx_ * (-pt.y) / pt.x + cx_;
    double v = fy_ * (-pt.z) / pt.x + cy_;
    if (u >= x0 && u <= x1 && v >= y0 && v <= y1) {
      cloud_in_bbox->points.push_back(pt);
    }
  }
  if (cloud_in_bbox->points.size() < 5) {
    return false;
  }

  // Statistical Outlier Removal — remove scattered noise
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in_bbox);
  sor.setMeanK(std::min<int>(15, static_cast<int>(cloud_in_bbox->points.size()) - 1));
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);
  if (cloud_filtered->points.empty()) {
    return false;
  }

  // Euclidean Clustering — split into clusters in case another object
  // bleeds into the bbox
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  if (cluster_indices.empty()) {
    return false;
  }

  // Pick the cluster closest to the camera along the REAL DEPTH axis
  // (pt.x in camera_link, not pt.z)
  float min_mean_x = std::numeric_limits<float>::max();
  pcl::PointIndices best;
  for (const auto & indices : cluster_indices) {
    float mean_x = 0;
    for (int idx : indices.indices) {
      mean_x += cloud_filtered->points[idx].x;
    }
    mean_x /= static_cast<float>(indices.indices.size());
    if (mean_x < min_mean_x) {
      min_mean_x = mean_x;
      best = indices;
    }
  }

  // Use the median for x, y, z — more robust against outliers than the mean
  auto median_of = [&](const std::function<float(const pcl::PointXYZ &)> & f) {
      std::vector<float> vals;
      vals.reserve(best.indices.size());
      for (int idx : best.indices) {
        vals.push_back(f(cloud_filtered->points[idx]));
      }
      std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
      return vals[vals.size() / 2];
    };

  out_position.x = median_of([](const pcl::PointXYZ & p) {return p.x;});
  out_position.y = median_of([](const pcl::PointXYZ & p) {return p.y;});
  out_position.z = median_of([](const pcl::PointXYZ & p) {return p.z;});
  return true;
}

}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception::DepthProjectorComponent)
