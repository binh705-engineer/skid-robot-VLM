#include "perception/usb_cam_component.hpp"
#include <cv_bridge/cv_bridge.h>
namespace perception
{
UsbCamComponent::UsbCamComponent(const rclcpp::NodeOptions & options)
: Node("usb_cam_component", options)
{
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("image_width", 640);
  this->declare_parameter("image_height", 480);
  this->declare_parameter("framerate", 30.0);
  std::string device = this->get_parameter("video_device").as_string();
  width_ = static_cast<int>(this->get_parameter("image_width").as_int());
  height_ = static_cast<int>(this->get_parameter("image_height").as_int());
  double fps = this->get_parameter("framerate").as_double();
  cap_.open(device, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Could not open camera at: %s", device.c_str());
    throw std::runtime_error("Cannot open camera device: " + device);
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS, fps);
  // Match QoS with the consumer node (yolov8_trt_component uses
  // SensorDataQoS): best-effort + small depth -> no ACK overhead, no
  // backlog of stale frames, always prioritizing the newest frame for low
  // latency instead of guaranteeing every frame arrives (not needed for a
  // realtime video stream).
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS().keep_last(1));
  auto period = std::chrono::duration<double>(1.0 / fps);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UsbCamComponent::timerCallback, this));
  RCLCPP_INFO(this->get_logger(), "USB Camera Component (C++) has started!");
}
void UsbCamComponent::timerCallback()
{
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) return;
  // NO cvtColor here anymore. OpenCV V4L2 returns the raw BGR image, so we
  // publish it directly as "bgr8" (zero color conversions on the full-res
  // image). The BGR->RGB conversion is moved to yolov8_trt_component,
  // performed AFTER resizing down to the model's input size (e.g.
  // 640x640) -> converting on an image tens of times smaller, much
  // cheaper than converting on the full-res image and then resizing.
  sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "camera_link";
  cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*msg);
  pub_image_->publish(std::move(msg));
}
UsbCamComponent::~UsbCamComponent()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}
}  // namespace perception
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  perception::UsbCamComponent
)
