#ifndef PERCEPTION__BYTETRACK_COMPONENT_HPP_
#define PERCEPTION__BYTETRACK_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <memory>
#include <vector>

// Include header của bộ lõi ByteTrack mà bạn đã thêm vào dự án
#include "perception/bytetrack/BYTETracker.h"

namespace perception
{

class ByteTrackComponent : public rclcpp::Node
{
public:
    explicit ByteTrackComponent(const rclcpp::NodeOptions & options);
    ~ByteTrackComponent() override;

private:
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detections_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_tracked_detections_;

    // Con trỏ quản lý đối tượng tracker
    std::unique_ptr<byte_track::BYTETracker> tracker_;

    // Các tham số cấu hình của ByteTrack
    int frame_rate_;
    int track_buffer_;
    float track_thresh_;
    float high_thresh_;
    float match_thresh_;
};

}  // namespace perception

#endif  // PERCEPTION__BYTETRACK_COMPONENT_HPP_
