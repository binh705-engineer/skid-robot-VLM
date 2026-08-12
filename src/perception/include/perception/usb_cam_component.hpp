#ifndef PERCEPTION__USB_CAM_COMPONENT_HPP_
#define PERCEPTION__USB_CAM_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace perception
{
class UsbCamComponent : public rclcpp::Node
{
public:
  explicit UsbCamComponent(const rclcpp::NodeOptions & options);
  ~UsbCamComponent();

private:
  void timerCallback();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  cv::VideoCapture cap_;
  int width_;
  int height_;
};
} // namespace perception

#endif // PERCEPTION__USB_CAM_COMPONENT_HPP_
