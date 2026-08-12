#include "perception/yolov8_trt_component.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

#include <cuda_runtime_api.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <cv_bridge/cv_bridge.h>

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace perception
{

void Yolov8TrtComponent::Logger::log(
    Severity severity,
    const char * msg) noexcept
{
    if (severity <= Severity::kWARNING)
    {
        std::cout << "[TensorRT] " << msg << std::endl;
    }
}

Yolov8TrtComponent::Yolov8TrtComponent(
    const rclcpp::NodeOptions & options)
: Node("yolov8_trt_component", options),
  stream_(nullptr),
  input_buffer_(nullptr),
  output_buffer_(nullptr),
  input_bytes_(0),
  output_bytes_(0)
{
    declare_parameter<std::string>("engine_path", "");
    declare_parameter<float>("conf_threshold", conf_threshold_);
    declare_parameter<float>("nms_threshold", nms_threshold_);

    std::string engine_path = get_parameter("engine_path").as_string();
    conf_threshold_ = get_parameter("conf_threshold").as_double();
    nms_threshold_ = get_parameter("nms_threshold").as_double();

    if (engine_path.empty())
    {
        RCLCPP_FATAL(get_logger(), "engine_path is empty.");
        throw std::runtime_error("engine path empty");
    }

    loadEngine(engine_path);

    cudaStreamCreate(&stream_);

    allocateBuffers();

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(
            &Yolov8TrtComponent::imageCallback,
            this,
            std::placeholders::_1));

    pub_detection_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "/detections_output",
        10);

    RCLCPP_INFO(get_logger(), "YOLOv8 TensorRT10 Component Ready.");
}

Yolov8TrtComponent::~Yolov8TrtComponent()
{
    if (input_buffer_)
        cudaFree(input_buffer_);

    if (output_buffer_)
        cudaFree(output_buffer_);

    if (stream_)
        cudaStreamDestroy(stream_);

    if (host_output_)
        cudaFreeHost(host_output_);
}

void Yolov8TrtComponent::loadEngine(
    const std::string & engine_path)
{
    std::ifstream file(engine_path, std::ios::binary);

    if (!file)
    {
        throw std::runtime_error("Cannot open engine.");
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engine(size);
    file.read(engine.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));

    if (!runtime_)
        throw std::runtime_error("runtime");

    engine_.reset(runtime_->deserializeCudaEngine(engine.data(), size));

    if (!engine_)
        throw std::runtime_error("engine");

    context_.reset(engine_->createExecutionContext());

    if (!context_)
        throw std::runtime_error("context");

    // ============================
    // TensorRT 10: Query IO tensors
    // ============================
    const int nb_io = engine_->getNbIOTensors();

    RCLCPP_INFO(get_logger(), "Engine has %d IO tensors.", nb_io);

    for (int i = 0; i < nb_io; ++i)
    {
        const char * tensor_name = engine_->getIOTensorName(i);
        auto mode = engine_->getTensorIOMode(tensor_name);
        auto shape = engine_->getTensorShape(tensor_name);

        std::stringstream ss;
        ss << "[";
        for (int d = 0; d < shape.nbDims; ++d)
        {
            ss << shape.d[d];
            if (d != shape.nbDims - 1)
                ss << ",";
        }
        ss << "]";

        RCLCPP_INFO(
            get_logger(),
            "Tensor %d : %s  shape=%s",
            i, tensor_name, ss.str().c_str());

        if (mode == nvinfer1::TensorIOMode::kINPUT)
        {
            input_tensor_name_ = tensor_name;
            input_index_ = i;
        }
        else
        {
            output_tensor_name_ = tensor_name;
            output_index_ = i;
        }
    }

    RCLCPP_INFO(get_logger(), "Input Tensor : %s", input_tensor_name_.c_str());
    RCLCPP_INFO(get_logger(), "Output Tensor: %s", output_tensor_name_.c_str());

    auto input_dims = engine_->getTensorShape(input_tensor_name_.c_str());
    auto output_dims = engine_->getTensorShape(output_tensor_name_.c_str());

    // input: [1, 3, H, W]
    input_height_ = input_dims.d[2];
    input_width_ = input_dims.d[3];

    input_bytes_ = sizeof(float);
    for (int i = 0; i < input_dims.nbDims; ++i)
        input_bytes_ *= input_dims.d[i];

    output_bytes_ = sizeof(float);
    for (int i = 0; i < output_dims.nbDims; ++i)
        output_bytes_ *= output_dims.d[i];

    // output YOLOv8 mặc định: [1, 4 + num_classes, num_anchors]
    // Lấy động thay vì hardcode 80 / 8400 để không vỡ khi đổi model.
    if (output_dims.nbDims != 3 || output_dims.d[1] <= 4)
    {
        throw std::runtime_error(
            "Output shape không đúng định dạng YOLOv8 [1, 4+C, N] đã kỳ vọng.");
    }

    num_classes_ = static_cast<int>(output_dims.d[1] - 4);
    num_anchors_ = static_cast<int>(output_dims.d[2]);

    RCLCPP_INFO(get_logger(), "Input bytes : %zu", input_bytes_);
    RCLCPP_INFO(get_logger(), "Output bytes: %zu", output_bytes_);
    RCLCPP_INFO(
        get_logger(),
        "num_classes=%d num_anchors=%d",
        num_classes_, num_anchors_);
}

void Yolov8TrtComponent::allocateBuffers()
{
    cudaError_t err = cudaMalloc(&input_buffer_, input_bytes_);
    if (err != cudaSuccess)
    {
        throw std::runtime_error("Cannot allocate input buffer.");
    }

    err = cudaMalloc(&output_buffer_, output_bytes_);
    if (err != cudaSuccess)
    {
        throw std::runtime_error("Cannot allocate output buffer.");
    }

    host_output_count_ = output_bytes_ / sizeof(float);

    err = cudaMallocHost(
        reinterpret_cast<void **>(&host_output_),
        output_bytes_);

    if (err != cudaSuccess)
    {
        throw std::runtime_error("Cannot allocate pinned host output buffer.");
    }

    RCLCPP_INFO(get_logger(), "GPU Buffers allocated successfully.");
    RCLCPP_INFO(get_logger(), "Input Buffer : %p", input_buffer_);
    RCLCPP_INFO(get_logger(), "Output Buffer: %p", output_buffer_);
}

void Yolov8TrtComponent::preprocess(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    // toCvShare: không copy dữ liệu ảnh, chỉ wrap con trỏ buffer của msg.
    // An toàn ở đây vì msg (SharedPtr) được giữ sống suốt cả imageCallback
    // (preprocess -> inference -> postprocess đều nằm trong 1 lần gọi).
    // encoding khớp đúng "bgr8" với những gì usb_cam_component publish
    // (không convert màu) -> zero-copy thật sự, không rơi vào nhánh
    // convert ngầm bên trong cv_bridge.
    try
    {
        frame_ = cv_bridge::toCvShare(
            msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (const cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        frame_.release();
        return;
    }

    x_scale_ = static_cast<float>(frame_.cols) / static_cast<float>(input_width_);
    y_scale_ = static_cast<float>(frame_.rows) / static_cast<float>(input_height_);

    // frame_ giờ là BGR nguyên gốc từ camera (usb_cam_component không
    // còn cvtColor trên full-res nữa). blobFromImage resize xuống
    // input_width x input_height TRƯỚC, rồi mới swapRB trên ảnh đã nhỏ
    // -> rẻ hơn nhiều so với cvtColor trên ảnh full-res như bản cũ.
    cv::Mat blob = cv::dnn::blobFromImage(
        frame_,
        1.0 / 255.0,
        cv::Size(input_width_, input_height_),
        cv::Scalar(),
        /*swapRB=*/true,
        /*crop=*/false,
        CV_32F);

    cudaMemcpyAsync(
        input_buffer_,
        blob.ptr<float>(),
        input_bytes_,
        cudaMemcpyHostToDevice,
        stream_);
}

void Yolov8TrtComponent::inference()
{
    if (frame_.empty())
        return;

    context_->setTensorAddress(input_tensor_name_.c_str(), input_buffer_);
    context_->setTensorAddress(output_tensor_name_.c_str(), output_buffer_);

    if (!context_->enqueueV3(stream_))
    {
        RCLCPP_ERROR(get_logger(), "enqueueV3 failed.");
        return;
    }

    cudaMemcpyAsync(
        host_output_,
        output_buffer_,
        output_bytes_,
        cudaMemcpyDeviceToHost,
        stream_);

    cudaStreamSynchronize(stream_);
}

/*void Yolov8TrtComponent::postprocess(
    vision_msgs::msg::Detection2DArray & detections)
{
    boxes_.clear();
    scores_.clear();
    class_ids_.clear();
    nms_indices_.clear();

    if (frame_.empty())
        return;

    const float * output = host_output_;

    for (int i = 0; i < num_anchors_; ++i)
    {
        float best_score = 0.0f;
        int best_class = -1;

        for (int c = 0; c < num_classes_; ++c)
        {
            float score = output[(4 + c) * num_anchors_ + i];
            if (score > best_score)
            {
                best_score = score;
                best_class = c;
            }
        }

        if (best_score < conf_threshold_)
            continue;

        float cx = output[0 * num_anchors_ + i];
        float cy = output[1 * num_anchors_ + i];
        float w = output[2 * num_anchors_ + i];
        float h = output[3 * num_anchors_ + i];

        int left = static_cast<int>((cx - w * 0.5f) * x_scale_);
        int top = static_cast<int>((cy - h * 0.5f) * y_scale_);
        int width = static_cast<int>(w * x_scale_);
        int height = static_cast<int>(h * y_scale_);

        boxes_.emplace_back(left, top, width, height);
        scores_.emplace_back(best_score);
        class_ids_.emplace_back(best_class);
    }

    cv::dnn::NMSBoxes(
        boxes_,
        scores_,
        conf_threshold_,
        nms_threshold_,
        nms_indices_);

    for (int idx : nms_indices_)
    {
        vision_msgs::msg::Detection2D det;

        det.bbox.center.position.x = boxes_[idx].x + boxes_[idx].width * 0.5;
        det.bbox.center.position.y = boxes_[idx].y + boxes_[idx].height * 0.5;
        det.bbox.size_x = boxes_[idx].width;
        det.bbox.size_y = boxes_[idx].height;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(class_ids_[idx]);
        hyp.hypothesis.score = scores_[idx];

        det.results.push_back(hyp);
        detections.detections.push_back(det);
    }
}
*/
void Yolov8TrtComponent::postprocess(
    vision_msgs::msg::Detection2DArray & detections)
{
    boxes_.clear();
    scores_.clear();
    class_ids_.clear();
    nms_indices_.clear();

    if (frame_.empty())
        return;

    const float * output = host_output_;

    for (int i = 0; i < num_anchors_; ++i)
    {
        // TRÍCH XUẤT TRỰC TIẾP CLASS 0 (PERSON)
        // Bỏ qua vòng lặp for (int c = 0; c < num_classes_; ++c)
        int person_class_id = 0;
        float score = output[(4 + person_class_id) * num_anchors_ + i];

        // Nếu điểm số của class người thấp hơn ngưỡng, bỏ qua anchor này luôn
        if (score < conf_threshold_)
            continue;

        float cx = output[0 * num_anchors_ + i];
        float cy = output[1 * num_anchors_ + i];
        float w = output[2 * num_anchors_ + i];
        float h = output[3 * num_anchors_ + i];

        int left = static_cast<int>((cx - w * 0.5f) * x_scale_);
        int top = static_cast<int>((cy - h * 0.5f) * y_scale_);
        int width = static_cast<int>(w * x_scale_);
        int height = static_cast<int>(h * y_scale_);

        boxes_.emplace_back(left, top, width, height);
        scores_.emplace_back(score);
        class_ids_.emplace_back(person_class_id); // Cố định luôn là 0
    }

    cv::dnn::NMSBoxes(
        boxes_,
        scores_,
        conf_threshold_,
        nms_threshold_,
        nms_indices_);

    for (int idx : nms_indices_)
    {
        vision_msgs::msg::Detection2D det;

        det.bbox.center.position.x = boxes_[idx].x + boxes_[idx].width * 0.5;
        det.bbox.center.position.y = boxes_[idx].y + boxes_[idx].height * 0.5;
        det.bbox.size_x = boxes_[idx].width;
        det.bbox.size_y = boxes_[idx].height;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(class_ids_[idx]);
        hyp.hypothesis.score = scores_[idx];

        det.results.push_back(hyp);
        detections.detections.push_back(det);
    }
}
void Yolov8TrtComponent::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    preprocess(msg);
    inference();

    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;

    postprocess(detections);

    pub_detection_->publish(detections);
}

}  // namespace perception

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(
  perception::Yolov8TrtComponent
)
