#include "preprocess.hpp"

#include <cuda_runtime.h>

#include <iostream>
#include <vector>

namespace avp
{
namespace
{

__device__ inline float normalize_value_device(float v, const PreprocessConfig& cfg, int c){
    if(cfg.normalize_to_unit){
        v /= 255.0f;
    }

    if(cfg.use_mean_std){
        v = (v - cfg.mean[c]) / cfg.std[c];
    }
    return v;
}

__device__ inline int chw_index_device(int c, int h, int w, int H, int W){
    return c*H*W + h*W+w;
}

__global__ void preprocess_kernel(const uint8_t* input, int input_width, int input_height,
    int input_channels, int input_step, float* output, PreprocessConfig cfg){
        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        const int y = blockIdx.y * blockDim.y + threadIdx.y;

        if(x >= cfg.output_width || y >= cfg.output_height){
            return;
        }

        const float src_x = static_cast<float>(x) * static_cast<float>(input_width) /
                        static_cast<float>(cfg.output_width);
        const float src_y = static_cast<float>(y) * static_cast<float>(input_height) /
                            static_cast<float>(cfg.output_height);

        const int ix = static_cast<int>(src_x);
        const int iy = static_cast<int>(src_y);

        if (ix < 0 || ix >= input_width || iy < 0 || iy >= input_height)
        {
            return;
        }
        
        const uint8_t* pixel = input + iy * input_step + ix * input_channels;

        uint8_t c0 = pixel[0];
        uint8_t c1 = pixel[1];
        uint8_t c2 = pixel[2];

        float out0 = 0.0f;
        float out1 = 0.0f;
        float out2 = 0.0f;

        const bool need_bgr_to_rgb = ((cfg.input_color_order == ColorOrder::BGR) &&
                                    (cfg.model_color_order == ColorOrder::RGB));
        const bool need_rgb_to_bgr = ((cfg.input_color_order == ColorOrder::RGB) &&
                                    (cfg.model_color_order == ColorOrder::BGR));
        
        if(need_bgr_to_rgb || need_rgb_to_bgr){
            out0 = normalize_value_device(static_cast<float>(c2), cfg, 0);
            out1 = normalize_value_device(static_cast<float>(c1), cfg, 1);
            out2 = normalize_value_device(static_cast<float>(c0), cfg, 2);
        }
        else{
            out0 = normalize_value_device(static_cast<float>(c0), cfg, 0);
            out1 = normalize_value_device(static_cast<float>(c1), cfg, 1);
            out2 = normalize_value_device(static_cast<float>(c2), cfg, 2);
        }

        if(cfg.output_layout == TensorLayout::CHW){
            output[chw_index_device(0,  y, x, cfg.output_height, cfg.output_width)] = out0;
            output[chw_index_device(1,  y, x, cfg.output_height, cfg.output_width)] = out1;
            output[chw_index_device(2,  y, x, cfg.output_height, cfg.output_width)] = out2;
        }
        else{
            const int idx = (y * cfg.output_width + x) * 3;
            output[idx + 0] = out0;
            output[idx + 1] = out1;
            output[idx + 2] = out2;
        }
    }

    inline bool check_cuda(cudaError_t err, const char* msg){
        if(err != cudaSuccess){
            std::cerr << "[CUDA ERROR] " << msg << " : "
                    << cudaGetErrorString(err) << std::endl;
            return false;
        }
        return true;
    }
}

bool preprocess_cuda(const ImageView& input, const PreprocessConfig& cfg, TensorView& output){
    if(!input.data || !output.data){
        return false;
    }

    if(input.channels != 3 || cfg.output_channels != 3){
        return false;
    }

    const size_t input_bytes = 
        static_cast<size_t>(input.step) * static_cast<size_t>(input.height);
    

    const size_t output_count = static_cast<size_t>(cfg.output_width) *
                                static_cast<size_t>(cfg.output_height) *
                                static_cast<size_t>(cfg.output_channels);
    
    const size_t output_bytes = output_count * sizeof(float);

    uint8_t* d_input = nullptr;
    float* d_output = nullptr;

    if(!check_cuda(cudaMalloc(&d_input, input_bytes), "cudaMalloc d_input")){
        cudaFree(d_input);
        return false;
    }

    if(!check_cuda(cudaMalloc(&d_output, output_bytes), "cudaMalloc d_output")){
        cudaFree(d_input);
        cudaFree(d_output);
        return false;
    }

    bool ok = true;

    do{
        if(!check_cuda(cudaMemcpy(d_input, input.data, input_bytes, cudaMemcpyHostToDevice),"cudaMemcpy HtoD")){
            ok = false;
            break;
        }

        dim3 block(16,16);
        dim3 grid((cfg.output_width + block.x - 1) / block.x,
                (cfg.output_height + block.y -1) / block.y);

        preprocess_kernel<<<grid, block>>>(d_input, input.width, input.height, input.channels,
                                            input.step, d_output, cfg);

        if(!check_cuda(cudaGetLastError(), "kernel launch")){
            ok = false;
            break;
        }

        if(!check_cuda(cudaDeviceSynchronize(), "kernel sync")){
            ok = false;
            break;
        }

        if(!check_cuda(cudaMemcpy(output.data, d_output, output_bytes, cudaMemcpyDeviceToHost),
                                    "cudaMemcpy DtoH")){
            ok = false;
            break;
        }       
    }while(false);

    cudaFree(d_input);
    cudaFree(d_output);

    return ok;
}
}
