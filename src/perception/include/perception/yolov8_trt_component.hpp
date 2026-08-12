#ifndef PERCEPTION__YOLOV8_TRT_COMPONENT_HPP_
#define PERCEPTION__YOLOV8_TRT_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <NvInfer.h>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace perception
{

class Yolov8TrtComponent : public rclcpp::Node
{
public:
    explicit Yolov8TrtComponent(
        const rclcpp::NodeOptions & options);

    ~Yolov8TrtComponent();

private:
    //---------------------------------------
    // TensorRT Logger
    //---------------------------------------
    class Logger : public nvinfer1::ILogger
    {
    public:
        void log(
            Severity severity,
            const char * msg) noexcept override;
    };

    Logger logger_;

    //---------------------------------------
    // TensorRT
    //---------------------------------------
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    //---------------------------------------
    // CUDA
    //---------------------------------------
    cudaStream_t stream_ = nullptr;
    void * input_buffer_ = nullptr;
    void * output_buffer_ = nullptr;

    //---------------------------------------
    // Tensor Information
    //---------------------------------------
    std::string input_tensor_name_;
    std::string output_tensor_name_;
    int input_index_ = -1;
    int output_index_ = -1;
    int input_width_ = 640;
    int input_height_ = 640;
    size_t input_bytes_ = 0;
    size_t output_bytes_ = 0;

    // Lấy động từ shape output engine, KHÔNG hardcode 80 / 8400
    int num_classes_ = 0;
    int num_anchors_ = 0;

    //---------------------------------------
    // Host Memory (pinned/page-locked để cudaMemcpyAsync
    // D2H nhanh hơn so với std::vector thường - pageable memory
    // buộc driver phải stage qua buffer trung gian, làm chậm băng
    // thông PCIe/NVLink một cách âm thầm, không báo lỗi gì cả)
    //---------------------------------------
    float * host_output_ = nullptr;
    size_t host_output_count_ = 0;

    //---------------------------------------
    // Trạng thái xử lý 1 frame (dùng chung giữa
    // preprocess -> inference -> postprocess)
    //---------------------------------------
    cv::Mat frame_;
    float x_scale_ = 1.0f;
    float y_scale_ = 1.0f;

    std::vector<cv::Rect> boxes_;
    std::vector<float> scores_;
    std::vector<int> class_ids_;
    std::vector<int> nms_indices_;

    //---------------------------------------
    // Tham số có thể chỉnh qua YAML
    //---------------------------------------
    float conf_threshold_ = 0.25f;
    float nms_threshold_ = 0.45f;

    //---------------------------------------
    // ROS2
    //---------------------------------------
    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<
        vision_msgs::msg::Detection2DArray>::SharedPtr pub_detection_;

    //---------------------------------------
    // TensorRT
    //---------------------------------------
    void loadEngine(
        const std::string & engine_path);
    void allocateBuffers();

    //---------------------------------------
    // ROS Callback
    //---------------------------------------
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg);

    //---------------------------------------
    // Pipeline
    //---------------------------------------
    void preprocess(
        const sensor_msgs::msg::Image::SharedPtr msg);
    void inference();
    void postprocess(
        vision_msgs::msg::Detection2DArray & detections);
};

}  // namespace perception

#endif  // PERCEPTION__YOLOV8_TRT_COMPONENT_HPP_
