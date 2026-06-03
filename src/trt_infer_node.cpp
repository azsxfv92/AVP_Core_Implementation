#include "trt_infer.hpp"
#include "preprocess.hpp"
#include "video_encoder.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <filesystem>
#include <iomanip>

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
        this->declare_parameter<std::string>(
            "csv_path",
            "results/week12/GStreamer/compressed_on.csv"
        );
        this->declare_parameter<std::string>(
            "preprocess_backend", 
            "cpu"
        );
        this->declare_parameter<bool>(
            "enable_encode", 
            false
        );
        output_video_path_ = this->declare_parameter<std::string>(
            "output_video_path", 
            "results/week12/output/week12_overlay.mp4"
        );
        this->declare_parameter<std::string>(
            "input_transport",
            "raw"
        );
        this->declare_parameter<std::string>(
            "compressed_image_topic",
            "/avp/camera/front/compressed"
        );
        encode_fps_ = this->declare_parameter<int>("encode_fps", 30);
        encode_width_ = this->declare_parameter<int>("encode_width", 1280);
        encode_height_ = this->declare_parameter<int>("encode_height", 720);
        encode_queue_size_ = this->declare_parameter<int>("encode_queue_size", 4);        
        
        
        const auto engine_path = 
            this->get_parameter("engine_path").as_string();
        const auto image_topic = 
            this->get_parameter("image_topic").as_string();
        const auto overlay_topic =
            this->get_parameter("overlay_topic").as_string();
        const auto csv_path =
            this->get_parameter("csv_path").as_string();
        preprocess_backend_ = 
            this->get_parameter("preprocess_backend").as_string();
        enable_encode_ =
            this->get_parameter("enable_encode").as_bool();
        input_transport_ = this->get_parameter("input_transport").as_string();
        compressed_image_topic_ = this->get_parameter("compressed_image_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "Engine path: %s", engine_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "overlay topic: %s", overlay_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "csv path: %s", csv_path.c_str());
        RCLCPP_INFO(this->get_logger(), "preprocess_backend: %s", preprocess_backend_.c_str());
        RCLCPP_INFO(this->get_logger(), "encode enabled: %s", enable_encode_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "output video path: %s", output_video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "encode fps: %d", encode_fps_);
        RCLCPP_INFO(this->get_logger(), "encode size: %dx%d", encode_width_, encode_height_);
        RCLCPP_INFO(this->get_logger(), "encode queue size: %d", encode_queue_size_);
        RCLCPP_INFO(this->get_logger(), "input_transport: %s", input_transport_.c_str());
        RCLCPP_INFO(this->get_logger(), "compressed_image_topic: %s", compressed_image_topic_.c_str());

        if(enable_encode_){
            video_encoder_ = std::make_unique<avp_core_implementation::VideoEncoder>();

            RCLCPP_INFO(this->get_logger(), "Week12 encoder is enabled. output=%s size=%dx%d fps=%d",
                        output_video_path_.c_str(), encode_width_, encode_height_, encode_fps_);
        }
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

        // sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     image_topic,
        //     rclcpp::SensorDataQoS(),
        //     std::bind(&TrtStreamInferNode::onImage, this, std::placeholders::_1)
        // );
        auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        if(input_transport_ == "compressed"){
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                compressed_image_topic_,
                camera_qos,
                std::bind(&TrtStreamInferNode::onCompressedImage, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed compressed image topic: %s", compressed_image_topic_.c_str());
        }
        else{
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic, camera_qos, std::bind(&TrtStreamInferNode::onImage, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Subscribed raw image topic: %s", image_topic.c_str());
        }

        

        overlayPub_ = this->create_publisher<sensor_msgs::msg::Image>(
            overlay_topic,
            10
        );

        csv_path_ = csv_path;
        std::filesystem::path csv_fs_path(csv_path_);
        std::filesystem::create_directories(csv_fs_path.parent_path());

        csv_.open(csv_path_, std::ios::out | std::ios::app);
        if(!csv_.is_open()){
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        if(std::filesystem::exists(csv_fs_path) && std::filesystem::file_size(csv_fs_path) == 0){
            csv_ << "frame_id,"
                 << "transport,"
                 << "preprocess_backend,"
                 << "pre_ms,"
                 << "pre_h2d_ms,"
                 << "pre_kernel_ms,"
                 << "pre_d2h_ms,"
                 << "infer_ms,"
                 << "post_ms,"
                 << "compression_ratio,"
                 << "compressed_size_byte,"
                 << "raw_size_bytes,"
                 << "decode_ms,"
                 << "encode_ms,"
                 << "encode_queue_depth,"
                 << "drop_count,"
                 << "drop_reason,"
                 << "total_ms\n";
                 csv_.flush();
            csv_header_written_ = true;
        }
        else{
            csv_header_written_ = true;
        }

        preprocess_cfg_.output_width = 640;
        preprocess_cfg_.output_height = 640;
        preprocess_cfg_.output_channels = 3;
        preprocess_cfg_.input_color_order = avp::ColorOrder::BGR;
        preprocess_cfg_.model_color_order = avp::ColorOrder::RGB;
        preprocess_cfg_.output_layout = avp::TensorLayout::CHW;
        preprocess_cfg_.normalize_to_unit = true;
        preprocess_cfg_.use_mean_std = false;

        const size_t preprocess_tensor_count = 
            static_cast<size_t>(preprocess_cfg_.output_width) *
            static_cast<size_t>(preprocess_cfg_.output_height) *
            static_cast<size_t>(preprocess_cfg_.output_channels);

        preprocess_buffer_.resize(preprocess_tensor_count, 0.0f);

        start_time_ = std::chrono::steady_clock::now();
    }
    ~TrtStreamInferNode()
    {
        if(video_encoder_){
            if(video_encoder_){
                video_encoder_->close();
            }
        }
        avp::release_cuda_preprocess_context(cuda_pre_ctx_);
    }

private:
    void onImage(const sensor_msgs::msg::Image::SharedPtr msg){
        frameCnt++;
        const auto t_total_start = std::chrono::steady_clock::now();

        if (!current_frame_from_compressed_) {
            current_transport_ = "raw";
            decode_ms_ = 0.0;
            compressed_size_bytes_ = 0;
            raw_size_byte_est_ = msg->data.size();
            compression_ratio_ = 1.0;
        }

        const auto t_pre_start = std::chrono::steady_clock::now();

        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch(const cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        preprocess_cfg_.input_width = cv_ptr->image.cols;
        preprocess_cfg_.input_height = cv_ptr->image.rows;
        preprocess_cfg_.input_channels = cv_ptr->image.channels();
        
        avp::ImageView input_view;
        input_view.data = cv_ptr->image.data;
        input_view.width = cv_ptr->image.cols;
        input_view.height = cv_ptr->image.rows;
        input_view.channels = cv_ptr->image.channels();
        input_view.step = static_cast<int>(cv_ptr->image.step);

        avp::TensorView tensor_view;
        tensor_view.data = preprocess_buffer_.data();
        tensor_view.n = 1;
        tensor_view.c = preprocess_cfg_.output_channels;
        tensor_view.h = preprocess_cfg_.output_height;
        tensor_view.w = preprocess_cfg_.output_width;
        
        avp::PreprocessTimingBreakdown pre_breakdown{};
        bool ok = false;

        if(preprocess_backend_ == "cuda"){
            if (!avp::ensure_cuda_preprocess_context(
                    cuda_pre_ctx_,
                    input_view.width,
                    input_view.height,
                    input_view.channels,
                    input_view.step,
                    preprocess_cfg_))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "ensure_cuda_preprocess_context failed. frame=%u",
                             frameCnt);
                return;
            }

            ok = avp::preprocess_cuda_reuse(
                input_view,
                preprocess_cfg_,
                tensor_view,
                cuda_pre_ctx_,
                pre_breakdown);
        }
        else{
            const auto cpu_pre_start = std::chrono::steady_clock::now();
            ok = avp::preprocess_cpu(input_view, preprocess_cfg_, tensor_view);
            const auto cpu_pre_end = std::chrono::steady_clock::now();
            pre_breakdown.total_ms =
                std::chrono::duration_cast<std::chrono::microseconds>(cpu_pre_end - cpu_pre_start).count() / 1000.0;

            pre_breakdown.h2d_ms = 0.0;
            pre_breakdown.kernel_ms = 0.0;
            pre_breakdown.d2h_ms = 0.0;
        }

        if(!ok){
            RCLCPP_ERROR(this->get_logger(), "preprocess failed. frame=%u backend=%s",
                        frameCnt, preprocess_backend_.c_str());
            return;
        }

        const auto t_pre_end = std::chrono::steady_clock::now();

        const auto t_infer_start = std::chrono::steady_clock::now();
        if(!infer_.runInference(preprocess_buffer_)){
            RCLCPP_ERROR(this->get_logger(), "Image inference failed");
            rclcpp::shutdown();
            return;
        }
        const auto t_infer_end = std::chrono::steady_clock::now();

        const auto t_post_start = std::chrono::steady_clock::now();
        //copy the original image to keep the original image
        cv::Mat overlay = cv_ptr->image.clone();

        const auto infer_ms_for_text =
            std::chrono::duration_cast<std::chrono::milliseconds>(t_infer_end - t_infer_start).count();
        const std::string line1 = "infer=ok";
        const std::string line2 = "frame=" + std::to_string(frameCnt);
        const std::string line3 = "infer_ms=" + std::to_string(infer_ms_for_text);
        
        //write text on the image
        cv::putText(overlay, line1, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        cv::putText(overlay, line2, cv::Point(30, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        cv::putText(overlay, line3, cv::Point(30, 120), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // convert the OpenCV image to ROS2 image message 
        auto overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg();
        overlayPub_->publish(*overlay_msg);

        const auto t_post_end = std::chrono::steady_clock::now();
        const auto t_total_end = std::chrono::steady_clock::now();

        const double pre_ms = (preprocess_backend_ == "cuda")
                ? pre_breakdown.total_ms : std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_pre_start).count() / 1000.0;

        const double infer_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_infer_end - t_infer_start).count() / 1000.0;
        const double post_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_post_end - t_post_start).count() / 1000.0;

        encode_ms_ = 0.0;
        encode_queue_depth_ = 0;
        drop_reason_ = "NONE";
        bool encode_ok = false; 

        // encoding process
        if (enable_encode_) {
            const auto encode_start = std::chrono::steady_clock::now();
            
            // check if encoder open 
            if (!encoder_opened_) {
                // open encoder
                encoder_opened_ = video_encoder_->open(output_video_path_, encode_width_, encode_height_, encode_fps_);

                if (!encoder_opened_) {
                    drop_reason_ = "ENCODER_OPEN_FAILED";
                    RCLCPP_ERROR(this->get_logger(), "Failed to open video encoder: %s",output_video_path_.c_str());
                }
            }

            if (encoder_opened_) {
                cv::Mat encode_frame_bgr;
                if(overlay.cols != encode_width_ || overlay.rows != encode_height_){
                    cv::resize(overlay, encode_frame_bgr, cv::Size(encode_width_, encode_height_));
                }
                else{
                    encode_frame_bgr = overlay;
                }

                encode_ok = video_encoder_->write_bgr(encode_frame_bgr);
                
                if (!encode_ok) {
                    drop_reason_ = "ENCODE_WRITE_FAILED";
                    drop_count_++;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Video encoder write failed");
                }
            }

            const auto encode_end = std::chrono::steady_clock::now();
            encode_ms_ = std::chrono::duration<double, std::milli>(encode_end - encode_start).count();
        }

        
        const double total_ms_pre = std::chrono::duration_cast<std::chrono::microseconds>(t_total_end - t_total_start).count() / 1000.0;
        const double total_ms = decode_ms_ + total_ms_pre + encode_ms_;


        if (csv_.is_open())
        {
            csv_ << frameCnt << ","
                 << current_transport_ << ","
                 << preprocess_backend_ << ","
                 << std::fixed << std::setprecision(3)
                 << pre_ms << ","
                 << pre_breakdown.h2d_ms << ","
                 << pre_breakdown.kernel_ms << ","
                 << pre_breakdown.d2h_ms << ","
                 << infer_ms << ","
                 << post_ms << ","
                 << compression_ratio_ << ","
                 << compressed_size_bytes_<< ","
                 << raw_size_byte_est_ << ","
                 << decode_ms_ << ","
                 << encode_ms_ << ","
                 << encode_queue_depth_ << ","
                 << drop_count_ << ","
                 << drop_reason_ << ","
                 << total_ms << "\n";
            csv_.flush();
        }

        if ((frameCnt % 10) == 0)
        {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() / 1000.0;
            const double fps = (elapsed_sec > 0.0) ? (static_cast<double>(frameCnt) / elapsed_sec) : 0.0;

            RCLCPP_INFO(
                this->get_logger(),
                "frames=%u elapsed=%.2f sec avg_fps=%.2f backend=%s pre=%.3f ms (h2d=%.3f kernel=%.3f d2h=%.3f) \
                infer=%.3f ms post=%.3f ms encode_ms = %.3f ms encode_queue_depth = %d drop_count = %d drop_reason = %s total=%.3f ms",
                frameCnt,
                elapsed_sec,
                fps,
                preprocess_backend_.c_str(),
                pre_ms,
                pre_breakdown.h2d_ms,
                pre_breakdown.kernel_ms,
                pre_breakdown.d2h_ms,
                infer_ms,
                post_ms,
                encode_ms_,
                encode_queue_depth_,
                drop_count_,
                drop_reason_.c_str(),
                total_ms);
        }

    }

    void onCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
        const auto decode_start = std::chrono::steady_clock::now();

        if(msg->data.empty()){
            drop_reason_ = "EMPTY_COMPRESSED_IMAGE";
            drop_count_++;

            RCLCPP_WARN(this->get_logger(), "Received empty compressed image");
            return;
        }
        
        // wrapp the JPEG image
        cv::Mat encoded_mat(
            1, static_cast<int>(msg->data.size()),
            CV_8UC1, static_cast<unsigned char *>(msg->data.data())
        );
        // release the compression 
        cv::Mat decoded_bgr = cv::imdecode(encoded_mat, cv::IMREAD_COLOR);
        
        const auto decode_end = std::chrono::steady_clock::now();
        decode_ms_ = std::chrono::duration<double, std::milli>(decode_end - decode_start).count();

        if(decoded_bgr.empty()){
            drop_reason_ = "JPEG_DECODE_FAILED";
            drop_count_++;
            RCLCPP_ERROR(this->get_logger(), "JPEG decode failed");
            return;
        }

        current_frame_from_compressed_ = true;
        current_transport_ = "compressed";
        compressed_size_bytes_ = msg->data.size();
        raw_size_byte_est_ = decoded_bgr.total() * decoded_bgr.elemSize();

        if(compressed_size_bytes_ > 0){
            compression_ratio_ = static_cast<double>(raw_size_byte_est_) / static_cast<double>(compressed_size_bytes_);
        }
        else{
            compression_ratio_ = 0.0;
        }

        auto raw_msg = cv_bridge::CvImage(msg->header, "bgr8", decoded_bgr).toImageMsg();
        onImage(raw_msg);
        current_frame_from_compressed_ = false;        
    }

    TrtInfer infer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub_; 
    uint32_t frameCnt = 0;
    std::chrono::steady_clock::time_point start_time_;
    std::ofstream csv_;
    bool csv_header_written_{false};
    std::string csv_path_;
    std::string preprocess_backend_{"cpu"};
    avp::PreprocessConfig preprocess_cfg_;
    std::vector<float> preprocess_buffer_;
    avp::CudaPreprocessContext cuda_pre_ctx_;
    // switch for encode/save
    bool enable_encode_{false};
    // record video path
    std::string output_video_path_{"results/week12/output/week12_overlay.mp4"};
    int encode_fps_{30};
    int encode_width_{1280};
    int encode_height_{720};
    // the number of frame queued when encode/save delay
    int encode_queue_size_{4};
    double encode_ms_ = 0.0;
    int encode_queue_depth_ = 0;
    int drop_count_ = 0;
    std::string drop_reason_ = "NONE";
    std::unique_ptr<avp_core_implementation::VideoEncoder> video_encoder_;
    bool encoder_opened_{false};
    std::string input_transport_{"raw"};
    std::string compressed_image_topic_{"/avp/camera/front/compressed"};
    bool current_frame_from_compressed_{false};
    std::string current_transport_{"raw"};
    double decode_ms_{0.0};
    std::size_t compressed_size_bytes_{0};
    std::size_t raw_size_byte_est_{0};
    double compression_ratio_{1.0};
    
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
