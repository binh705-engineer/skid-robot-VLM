// cbf_safety_filter_node.cpp
//
// Nhận:
//   - Local costmap (nav_msgs/OccupancyGrid) từ Nav2 (vd /local_costmap/costmap)
//   - cmd_vel_nav (geometry_msgs/Twist) là output của DWB controller_server
// Xuất:
//   - /cmd_vel (geometry_msgs/Twist) đã qua CBF-QP safety filter
//
// Ý tưởng: mỗi khi costmap update -> tính lại Euclidean Distance Transform (EDT)
// một lần, cache lại. Mỗi khi nhận cmd_vel_nav -> lấy pose robot qua TF, nội suy
// D(x,y) và gradient tại offset-point phía trước robot, giải QP nhỏ (OSQP) để
// tìm (v, w) gần với (v_ref, w_ref) nhất nhưng vẫn thỏa CBF constraint.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <OsqpEigen/OsqpEigen.h>

#include <mutex>
#include <cmath>
#include <optional>

using std::placeholders::_1;

class CbfSafetyFilterNode : public rclcpp::Node
{
public:
  CbfSafetyFilterNode() : Node("cbf_safety_filter_node")
  {
    // ---- Parameters ----
    declare_parameter("d_safe", 0.35);          // khoảng cách an toàn tối thiểu (m)
    declare_parameter("alpha", 2.0);            // hệ số class-K của CBF
    declare_parameter("l_offset", 0.2);         // offset-point phía trước robot (m)
    declare_parameter("v_max", 1.0);
    declare_parameter("v_min", -0.3);
    declare_parameter("w_max", 1.5);
    declare_parameter("costmap_topic", std::string("/local_costmap/costmap"));
    declare_parameter("cmd_vel_nav_topic", std::string("/cmd_vel_smoothed"));
    declare_parameter("cmd_vel_out_topic", std::string("/cmd_vel"));
    declare_parameter("robot_base_frame", std::string("base_link"));
    declare_parameter("obstacle_lethal_threshold", 90); // cost >= giá trị này coi là vật cản

    d_safe_ = get_parameter("d_safe").as_double();
    alpha_ = get_parameter("alpha").as_double();
    l_offset_ = get_parameter("l_offset").as_double();
    v_max_ = get_parameter("v_max").as_double();
    v_min_ = get_parameter("v_min").as_double();
    w_max_ = get_parameter("w_max").as_double();
    lethal_threshold_ = get_parameter("obstacle_lethal_threshold").as_int();
    base_frame_ = get_parameter("robot_base_frame").as_string();

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Sub/Pub ----
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("costmap_topic").as_string(), rclcpp::QoS(1),
        std::bind(&CbfSafetyFilterNode::costmapCallback, this, _1));

    cmd_vel_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_nav_topic").as_string(), rclcpp::QoS(10),
        std::bind(&CbfSafetyFilterNode::cmdVelNavCallback, this, _1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_out_topic").as_string(), rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "CBF safety filter node started. d_safe=%.2f alpha=%.2f",
                d_safe_, alpha_);
  }

private:
  // ---------- Costmap -> Euclidean Distance Transform (mét) ----------
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int w = msg->info.width;
    const int h = msg->info.height;
    const double res = msg->info.resolution;

    cv::Mat obstacle_mask(h, w, CV_8UC1, cv::Scalar(255)); // 255 = free, 0 = obstacle
    for (int y = 0; y < h; ++y)
    {
      for (int x = 0; x < w; ++x)
      {
        int8_t cost = msg->data[y * w + x];
        // -1 = unknown -> coi như free để không khóa robot vô cớ
        if (cost >= lethal_threshold_)
        {
          obstacle_mask.at<uint8_t>(y, x) = 0;
        }
      }
    }

    cv::Mat dist_px;
    // distanceTransform: khoảng cách (pixel) từ mỗi free-cell tới obstacle-cell gần nhất
    cv::distanceTransform(obstacle_mask, dist_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    std::lock_guard<std::mutex> lock(map_mutex_);
    dist_m_ = dist_px * res; // đổi sang mét
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    resolution_ = res;
    map_frame_ = msg->header.frame_id;
    have_map_ = true;
  }

  // Nội suy song tuyến D(x,y) tại tọa độ world (map_frame_), trả về std::nullopt
  // nếu ngoài biên bản đồ.
  std::optional<double> interpDistance(double wx, double wy, double &dDdx, double &dDdy)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!have_map_) return std::nullopt;

    double fx = (wx - origin_x_) / resolution_;
    double fy = (wy - origin_y_) / resolution_;

    int x0 = static_cast<int>(std::floor(fx));
    int y0 = static_cast<int>(std::floor(fy));
    int x1 = x0 + 1, y1 = y0 + 1;

    if (x0 < 0 || y0 < 0 || x1 >= dist_m_.cols || y1 >= dist_m_.rows)
      return std::nullopt;

    double tx = fx - x0, ty = fy - y0;

    double d00 = dist_m_.at<float>(y0, x0);
    double d10 = dist_m_.at<float>(y0, x1);
    double d01 = dist_m_.at<float>(y1, x0);
    double d11 = dist_m_.at<float>(y1, x1);

    double D = d00 * (1 - tx) * (1 - ty) + d10 * tx * (1 - ty) +
               d01 * (1 - tx) * ty + d11 * tx * ty;

    // gradient xấp xỉ bằng central-ish finite difference trên ô lưới song tuyến
    dDdx = ((d10 - d00) * (1 - ty) + (d11 - d01) * ty) / resolution_;
    dDdy = ((d01 - d00) * (1 - tx) + (d11 - d10) * tx) / resolution_;

    return D;
  }

  // ---------- cmd_vel_nav -> QP -> cmd_vel ----------
  void cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v_ref = msg->linear.x;
    double w_ref = msg->angular.z;

    if (!have_map_)
    {
      // Chưa có costmap -> publish thẳng ref (fail-open); có thể đổi thành
      // fail-closed (publish 0) tuỳ yêu cầu an toàn của bạn.
      cmd_vel_pub_->publish(*msg);
      return;
    }

    // Lấy pose robot trong map_frame_ qua TF
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed (%s), publish v=0 for safety", ex.what());
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      return;
    }

    double rx = tf_stamped.transform.translation.x;
    double ry = tf_stamped.transform.translation.y;
    double theta = tf2::getYaw(tf_stamped.transform.rotation);

    // Offset-point phía trước robot để hạ relative-degree về 1
    double px = rx + l_offset_ * std::cos(theta);
    double py = ry + l_offset_ * std::sin(theta);

    double dDdx = 0, dDdy = 0;
    auto D_opt = interpDistance(px, py, dDdx, dDdy);
    if (!D_opt.has_value())
    {
      // Ngoài costmap -> không đủ thông tin, đi thẳng ref (giữ nguyên hành vi DWB)
      cmd_vel_pub_->publish(*msg);
      return;
    }
    double h = D_opt.value() - d_safe_;

    // p_dot = J(theta) * [v; w], J = [[cosθ - l sinθ], [sinθ + l cosθ]] cho unicycle+offset
    // ḣ = ∇D · p_dot = a_v * v + a_w * w
    double a_v = dDdx * std::cos(theta) + dDdy * std::sin(theta);
    double a_w = dDdx * (-l_offset_ * std::sin(theta)) + dDdy * (l_offset_ * std::cos(theta));

    // CBF constraint: a_v*v + a_w*w + alpha*h >= 0
    //   =>  -a_v*v - a_w*w <= alpha*h
    solveQP(v_ref, w_ref, a_v, a_w, alpha_ * h);
  }

  // QP: min 0.5*||u - u_ref||^2  s.t.  A u <= b  (1 CBF constraint) và box [v_min,v_max]x[-w_max,w_max]
  void solveQP(double v_ref, double w_ref, double a_v, double a_w, double b)
  {
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3); // 1 CBF + v box + w box (mỗi box 1 dòng, dùng min/max qua 2 dòng nếu cần chặt hơn)

    Eigen::SparseMatrix<double> H(2, 2);
    H.insert(0, 0) = 1.0;
    H.insert(1, 1) = 1.0;
    Eigen::VectorXd g(2);
    g << -v_ref, -w_ref;

    Eigen::SparseMatrix<double> A(3, 2);
    A.insert(0, 0) = -a_v; A.insert(0, 1) = -a_w; // CBF: -a_v*v - a_w*w <= b
    A.insert(1, 0) = 1.0;                          // v <= v_max, v >= v_min
    A.insert(2, 1) = 1.0;                          // w <= w_max, w >= -w_max

    Eigen::VectorXd lower(3), upper(3);
    lower << -OsqpEigen::INFTY, v_min_, -w_max_;
    upper << b, v_max_, w_max_;

    if (!solver.data()->setHessianMatrix(H) ||
        !solver.data()->setGradient(g) ||
        !solver.data()->setLinearConstraintsMatrix(A) ||
        !solver.data()->setLowerBound(lower) ||
        !solver.data()->setUpperBound(upper))
    {
      RCLCPP_ERROR(get_logger(), "QP setup failed, publishing zero velocity");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    if (!solver.initSolver())
    {
      RCLCPP_ERROR(get_logger(), "OSQP init failed, publishing zero velocity");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      RCLCPP_WARN(get_logger(), "QP infeasible/solve error, publishing zero velocity (fail-safe)");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    Eigen::VectorXd sol = solver.getSolution();

    geometry_msgs::msg::Twist out;
    out.linear.x = sol(0);
    out.angular.z = sol(1);
    cmd_vel_pub_->publish(out);
  }

  // ---- members ----
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex map_mutex_;
  cv::Mat dist_m_;
  double origin_x_{0}, origin_y_{0}, resolution_{0.05};
  std::string map_frame_{"map"};
  bool have_map_{false};

  double d_safe_, alpha_, l_offset_, v_max_, v_min_, w_max_;
  int lethal_threshold_;
  std::string base_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CbfSafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
