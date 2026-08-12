#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <perception/msg/tracked_object3_d.hpp>
#include <perception/msg/tracked_object3_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

namespace perception
{

class DepthProjectorComponent : public rclcpp::Node
{
public:
  explicit DepthProjectorComponent(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & tracks_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & stamp);

  // Đã bỏ tham số out_distance — distance được tính SAU khi transform sang base_frame,
  // vì tính ở camera frame (như bản cũ) sẽ sai ý nghĩa "khoảng cách phẳng x,y".
  bool estimateDistanceForBbox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_cam,
    const vision_msgs::msg::Detection2D & det,
    geometry_msgs::msg::Point & out_position);

  // params
  double fx_, fy_, cx_, cy_;
  double ground_dist_thresh_;
  double cluster_tolerance_;
  int min_cluster_size_;
  double bbox_shrink_ratio_;
  std::string camera_frame_, lidar_frame_, base_frame_;
  double grid_resolution_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> tracks_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, vision_msgs::msg::Detection2DArray>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<perception::msg::TrackedObject3DArray>::SharedPtr objects_pub_;
};

}  // namespace perception
