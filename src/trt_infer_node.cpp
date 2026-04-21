#include "trt_infer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>

namespace avp{
    void TrtLogger::log(Severity severity, const char* msg) noexcept
    {
        if (severity <= Severity::kINFO) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }

    TrtInfer::TrtInfer() = default;

    TrtInfer::~TrtInfer(){
        destroyBuffers();
        if(stream_ != nullptr){
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
        if (context_ != nullptr) {
            delete context_;
            context_ = nullptr;
        }

        if (engine_ != nullptr) {
            delete engine_;
            engine_ = nullptr;
        }

        if (runtime_ != nullptr) {
            delete runtime_;
            runtime_ = nullptr;
        }
    }
    
    bool TrtInfer::createStream(){
        return cudaStreamCreate(&stream_) == cudaSuccess;
    }

    void TrtInfer::destroyBuffers(){
        if(d_input_ != nullptr){
            cudaFree(d_input_);
            d_input_ = nullptr;
        }
        if(d_output_ != nullptr){
            cudaFree(d_output_);
            d_output_ = nullptr;
        }
    }

    size_t TrtInfer::calcNumElements(const std::vector<int64_t>& dims) const
    {
        size_t numel = 1;
        for(const auto d : dims){
            if(d <= 0){
                return 0;
            }
            numel *= static_cast<size_t>(d);
        }
        return numel;
    }

    bool TrtInfer::loadEngine(const std::string& engine_path){
        // engine file binary read
        std::ifstream ifs(engine_path, std::ios::binary);
        if(!ifs){
            std::cerr << "[ERROR] failed to open engine file: " << engine_path << std::endl;
            return false;
        }

        ifs.seekg(0, std::ios::end);
        const std::streamsize file_size = ifs.tellg();
        if(file_size <= 0){
            std::cerr << "[ERROR] invalid engine file size" << std::endl;
            return false;
        }

        ifs.seekg(0, std::ios::beg);
        engine_data_.resize(static_cast<size_t>(file_size));
        if(!ifs.read(engine_data_.data(), file_size)){
            std::cerr << "[ERROR] failed to read engine file" << std::endl;
            return false;
        }

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if(!runtime_){
            std::cerr << "[ERROR] failed to create TensorRT runtime" << std::endl;
            return false;
        }

        engine_ = runtime_->deserializeCudaEngine(engine_data_.data(), engine_data_.size());
        if(!engine_){
            std::cerr << "[ERROR] failed to deserialize TensorRT engine" << std::endl;
            return false;
        }

        // create execution context
        context_ = engine_->createExecutionContext();
        if (!context_) {
            std::cerr << "[ERROR] failed to create execution context" << std::endl;
            return false;
        }

        const int32_t nb_io = engine_->getNbIOTensors();
        if (nb_io < 2) {
            std::cerr << "[ERROR] invalid number of IO tensors: " << nb_io << std::endl;
            return false;
        }

        for(int32_t i=0; i<nb_io; ++i){
            const char* tensor_name = engine_->getIOTensorName(i);
            if(!tensor_name){
                continue;
            }
            
            const auto mode = engine_->getTensorIOMode(tensor_name);
            const auto dims = engine_->getTensorShape(tensor_name);
            
            std::vector<int64_t> shape;
            for(int32_t d =0; d < dims.nbDims; ++d){
                shape.push_back(static_cast<int64_t>(dims.d[d]));
            }
            if(mode == nvinfer1::TensorIOMode::kINPUT){
                input_tensor_name_ = tensor_name;
                input_dims_ = shape;
            }
            else if(mode == nvinfer1::TensorIOMode::kOUTPUT){
                output_tensor_name_ = tensor_name;
                output_dims_ = shape;
            }
        }

        if(input_tensor_name_.empty() || output_tensor_name_.empty()){
            std::cerr << "[ERROR] failed to find input/output tensor names" << std::endl;
            return false;
        }

        return true;   
    }

    bool TrtInfer::allocateBuffers(){
        // create cuda stream 
        if (!createStream()) {
            std::cerr << "[ERROR] failed to create CUDA stream" << std::endl;
            return false;
        }

        // calculate the total number of elements based on tensor shape
        input_numel_ = calcNumElements(input_dims_);
        output_numel_ = calcNumElements(output_dims_);

        if(input_numel_ == 0 || output_numel_ == 0){
            std::cerr << "[ERROR] invalid tensor numel" << std::endl;
            return false;
        }

        h_input_.resize(input_numel_);
        h_output_.resize(output_numel_);

        const size_t input_bytes = input_numel_ * sizeof(float);
        const size_t output_bytes = output_numel_ * sizeof(float);

        // allocate the GPU input buffer
        if(cudaMalloc(&d_input_, input_bytes) != cudaSuccess){
            std::cerr << "[ERROR] cudaMalloc failed for input buffer" << std::endl;
            return false;
        }

        // allocate the GPU output buffer
        if (cudaMalloc(&d_output_, output_bytes) != cudaSuccess) {
            std::cerr << "[ERROR] cudaMalloc failed for output buffer" << std::endl;
            return false;
        }

        return true;
    }

    void TrtInfer::printTensorInfo() const
    {
        std::cout << "[INFO] input tensor: " << input_tensor_name_ << std::endl;
        std::cout << "[INFO] output tensor: " << output_tensor_name_ << std::endl;

        std::cout << "[INFO] input dims: ";
        for (auto d : input_dims_) std::cout << d << " ";
        std::cout << std::endl;

        std::cout << "[INFO] output dims: ";
        for (auto d : output_dims_) std::cout << d << " ";
        std::cout << std::endl;
    }

    bool TrtInfer::runInference(const std::vector<float>& input_data)
    {
        if(input_data.size() != input_numel_){
            std::cerr << "[ERROR] input_data size mismatch. expected="
                      << input_numel_ << " actual=" << input_data.size() << std::endl;
            return false;
        }

        const size_t input_bytes = input_numel_ * sizeof(float);
        const size_t output_bytes = output_numel_ * sizeof(float);

        if(cudaMemcpyAsync(
            d_input_,
            input_data.data(),
            input_bytes,
            cudaMemcpyHostToDevice,
            stream_) != cudaSuccess){
                std::cerr << "[ERROR] H2D memcpy failed" << std::endl;
                return false;
            }
        if (!context_->setTensorAddress(input_tensor_name_.c_str(), d_input_)){
            std::cerr << "[ERROR] failed to set input tensor address" << std::endl;
            return false;
        }

        if (!context_->setTensorAddress(output_tensor_name_.c_str(), d_output_)) {
            std::cerr << "[ERROR] failed to set output tensor address" << std::endl;
            return false;
        }

        if (!context_->enqueueV3(stream_)) {
            std::cerr << "[ERROR] enqueueV3 failed" << std::endl;
            return false;
        }

        if(cudaMemcpyAsync(
            h_output_.data(),
            d_output_,
            output_bytes,
            cudaMemcpyDeviceToHost,
            stream_) != cudaSuccess){
                std::cerr << "[ERROR] D2H memcpy failed" << std::endl;
                return false;
            }

        if(cudaStreamSynchronize(stream_) != cudaSuccess){
            std::cerr << "[ERROR] cudaStreamSynchronize failed" << std::endl;
            return false;
        }

        std::cout << "[INFO] output sample: ";
        const size_t print_count = std::min<size_t>(10, h_output_.size());
        for(size_t i = 0; i < print_count; ++i){
            std::cout << h_output_[i] << " ";
        }
        std::cout << std::endl;

        return true;
    }

    class TrtStreamInferNode : public rclcpp::Node
    {
    public:
    TrtStreamInferNode() : Node("trt_stream_infer_node"){
        this->declare_parameter<std::string>(
            "engine_path",
            "models/trt/yolov5n_fp16.engine"
        );
        this->declare_parameter<std::string>(
            "image_topic",
            "/avp/camera/front"
        );
        this->declare_parameter<std::string>(
            "overlay_topic",
            "/avp/infer/overlay"
        );
        const auto engine_path = 
            this->get_parameter("engine_path").as_string();
        const auto image_topic = 
            this->get_parameter("image_topic").as_string();
        const auto overlay_topic =
            this->get_parameter("overlay_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Engine path: %s", engine_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "overlay topic: %s", overlay_topic.c_str());

        // to ready engine before images are comming 
        if(!infer_.loadEngine(engine_path)){
            RCLCPP_ERROR(this->get_logger(), "Failed to load TensorRT Engine");
            rclcpp::shutdown();
            return;
        }

        if(!infer_.allocateBuffers()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate TensorRT buffers");
            rclcpp::shutdown();
            return;
        }

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&TrtStreamInferNode::onImage, this, std::placeholders::_1)
        );

        overlayPub_ = this->create_publisher<sensor_msgs::msg::Image>(
            overlay_topic,
            10
        );

        start_time_ = std::chrono::steady_clock::now();
    }


    private:
        void onImage(const sensor_msgs::msg::Image::SharedPtr msg){
            //TODO1:
            frameCnt++;

            //TODO2:
            cv_bridge::CvImageConstPtr cv_ptr;

            try{
                cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            } catch(const cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
                rclcpp::shutdown();
                return;
            }

            //TODO3:
            cv::Mat resized_bgr;
            cv::resize(cv_ptr->image, resized_bgr, cv::Size(640, 640));
            //TODO4:
            cv::Mat rgb;
            cv::cvtColor(resized_bgr, rgb, cv::COLOR_BGR2RGB);

            //TODO5:
            std::vector<float> input_data(3*640*640, 0.0f);
            for (int y = 0; y < 640; ++y) {
                for (int x = 0; x < 640; ++x) {
                    const auto& pixel = rgb.at<cv::Vec3b>(y, x);

                    input_data[0 * 640 * 640 + y * 640 + x] = static_cast<float>(pixel[0]) / 255.0f;
                    input_data[1 * 640 * 640 + y * 640 + x] = static_cast<float>(pixel[1]) / 255.0f;
                    input_data[2 * 640 * 640 + y * 640 + x] = static_cast<float>(pixel[2]) / 255.0f;
                }
            }


            //TODO6:
            const auto t0 = std::chrono::steady_clock::now();
            if(!infer_.runInference(input_data)){
                RCLCPP_ERROR(this->get_logger(), "Image inference failed");
                rclcpp::shutdown();
                return;
            }
            const auto t1 = std::chrono::steady_clock::now();
            const auto elapsed_ms = 
            std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

            //TODO7: 
            //copy the original image to keep the original image
            cv::Mat overlay = cv_ptr->image.clone();

            const std::string line1 = "infer=ok";
            const std::string line2 = "frame=" + std::to_string(frameCnt);
            const std::string line3 = "infer_ms=" + std::to_string(elapsed_ms);
            
            //write text on the image
            cv::putText(overlay, line1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(overlay, line2, cv::Point(30, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(overlay, line3, cv::Point(30, 120), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            //TODO8:
            // convert the OpenCV image to ROS2 image message 
            auto overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg();
            overlayPub_->publish(*overlay_msg);

            //TODO9:
            if((frameCnt % 10) == 0)
            {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time_).count() / 1000.0;
                const double fps = (elapsed_sec > 0.0) ? (static_cast<double>(frameCnt) / elapsed_sec) : 0.0;
                RCLCPP_INFO(this->get_logger(), 
                "Total Frame = %zu, Elapsed Time = %.2f, average FPS = %.2f", 
                frameCnt, elapsed_sec, fps);
            }
            
        }
        TrtInfer infer_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub_; 
        uint32_t frameCnt = 0;
        std::chrono::steady_clock::time_point start_time_;
    };
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<avp::TrtStreamInferNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}