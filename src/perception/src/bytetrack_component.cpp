#include "perception/bytetrack_component.hpp"
#include <string>
namespace perception
{
ByteTrackComponent::ByteTrackComponent(const rclcpp::NodeOptions & options)
: Node("bytetrack_component", options)
{
    // 1. Declare and get parameters
    declare_parameter<int>("frame_rate", 30);
    declare_parameter<int>("track_buffer", 30);      // Number of frames an ID is kept alive during occlusion
    declare_parameter<float>("track_thresh", 0.5f);  // Confidence threshold for tracking
    declare_parameter<float>("high_thresh", 0.6f);
    declare_parameter<float>("match_thresh", 0.65f);  // IoU threshold for ID matching
    frame_rate_ = get_parameter("frame_rate").as_int();
    track_buffer_ = get_parameter("track_buffer").as_int();
    track_thresh_ = get_parameter("track_thresh").as_double();
    high_thresh_ = get_parameter("high_thresh").as_double();
    match_thresh_ = get_parameter("match_thresh").as_double();
    // Initialize the BYTETracker object (depends on the constructor of the core library used)
    tracker_ = std::make_unique<byte_track::BYTETracker>(
        frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_
    );
    // 2. Setup Sub & Pub
    sub_detections_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections_output",
        10,
        std::bind(&ByteTrackComponent::detectionCallback, this, std::placeholders::_1));
    pub_tracked_detections_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "/tracked_persons",
        10);
    RCLCPP_INFO(get_logger(), "ByteTrack Component Ready.");
}
ByteTrackComponent::~ByteTrackComponent() = default;
void ByteTrackComponent::detectionCallback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::vector<byte_track::Object> objects;
    
    for (const auto & det : msg->detections)
    {
        if (det.results.empty()) continue;
        // Get coordinates from the ROS 2 msg
        float cx = det.bbox.center.position.x;
        float cy = det.bbox.center.position.y;
        float w = det.bbox.size_x;
        float h = det.bbox.size_y;
        // 1. FIX OBJECT CREATION: Use the constructor instead of direct assignment.
        // The library requires constructing a Rect first, then wrapping it in an Object.
        byte_track::Rect<float> rect(cx - w / 2.0, cy - h / 2.0, w, h);
        byte_track::Object obj(rect, 0, det.results[0].hypothesis.score);
        objects.push_back(obj);
    }
    // 2. FIX POINTER RETURN: the update function returns an array of pointers (shared_ptr)
    std::vector<std::shared_ptr<byte_track::STrack>> output_stracks = tracker_->update(objects);
    vision_msgs::msg::Detection2DArray tracked_msg;
    tracked_msg.header = msg->header; // Preserve the original timestamp
    for (const auto & track_ptr : output_stracks)
    {
        vision_msgs::msg::Detection2D out_det;
        
        // 3. FIX ENCAPSULATION: use getRect(), and x()/y() accessors instead of static members
        auto rect = track_ptr->getRect(); 
        
        out_det.bbox.size_x = rect.width();
        out_det.bbox.size_y = rect.height();
        out_det.bbox.center.position.x = rect.x() + rect.width() / 2.0;
        out_det.bbox.center.position.y = rect.y() + rect.height() / 2.0;
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        
        // 4. FIX GETTERS: use getTrackId() and getScore() to access private data
        hyp.hypothesis.class_id = std::to_string(track_ptr->getTrackId()); 
        hyp.hypothesis.score = track_ptr->getScore();
        out_det.results.push_back(hyp);
        tracked_msg.detections.push_back(out_det);
    }
    pub_tracked_detections_->publish(tracked_msg);
}
}  // namespace perception
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception::ByteTrackComponent)
