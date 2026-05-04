#pragma once

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>


namespace avp{
class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severrity, const char* msg) noexcept override;
};

class TrtInfer
{
public:
    TrtInfer();
    ~TrtInfer();

    bool loadEngine(const std::string& engine_path);
    bool allocateBuffers();
    bool runInference(const std::vector<float>& input_data);
    void printTensorInfo() const;

private:
    
    TrtLogger logger_;
    // a buffer including .engine file as binary format
    std::vector<char> engine_data_;
    
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};

    cudaStream_t stream_{};

    std::string input_tensor_name_;
    std::string output_tensor_name_;

    std::vector<int64_t> input_dims_;
    std::vector<int64_t> output_dims_;

    size_t input_numel_{0};
    size_t output_numel_{0};

    void* d_input_{nullptr};
    void* d_output_{nullptr};

    std::vector<float> h_input_;
    std::vector<float> h_output_;
  
    bool createStream();
    void destroyBuffers();
    size_t calcNumElements(const std::vector<int64_t>& dims) const;
};
}