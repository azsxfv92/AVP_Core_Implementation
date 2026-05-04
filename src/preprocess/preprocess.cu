#include "preprocess.hpp"

#include <cuda_runtime.h> 

#include <iostream>
#include <vector>
#include <chrono>

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
    int input_channels, int input_step, float* output, PreprocessConfig cfg)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= cfg.output_width || y >= cfg.output_height){
        return;
    }

    const float src_x = static_cast<float>(x) * static_cast<float>(input_width) /
                        static_cast<float>(cfg.output_width);
    const float src_y = static_cast<float>(y) * static_cast<float>(input_height) /
                        static_cast<float>(cfg.output_height);
    
    const int ix = min(max(static_cast<int>(src_x), 0), input_width - 1);
    const int iy = min(max(static_cast<int>(src_y), 0), input_height - 1);

    const uint8_t* pixel = input + iy * input_step + ix * input_channels;

    const uint8_t c0 = pixel[0];
    const uint8_t c1 = pixel[1];
    const uint8_t c2 = pixel[2];

    float out0 = 0.0f;
    float out1 = 0.0f;
    float out2 = 0.0f;

    const bool need_bgr_to_rgb =
        (cfg.input_color_order == ColorOrder::BGR &&
         cfg.model_color_order == ColorOrder::RGB);

    const bool need_rgb_to_bgr =
        (cfg.input_color_order == ColorOrder::RGB &&
         cfg.model_color_order == ColorOrder::BGR);

    
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
        output[chw_index_device(0, y, x, cfg.output_height, cfg.output_width)] = out0;
        output[chw_index_device(1, y, x, cfg.output_height, cfg.output_width)] = out1;
        output[chw_index_device(2, y, x, cfg.output_height, cfg.output_width)] = out2;
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

inline size_t calc_input_bytes(int input_h, int input_step){
    return static_cast<size_t>(input_h) * static_cast<size_t>(input_step);
}

inline size_t calc_output_bytes(const PreprocessConfig& cfg){
    const size_t output_count = static_cast<size_t>(cfg.output_width) *
                                static_cast<size_t>(cfg.output_height) *
                                static_cast<size_t>(cfg.output_channels);
    return output_count * sizeof(float);
}

inline void reset_timing(PreprocessTimingBreakdown& timing){
    timing.h2d_ms = 0.0;
    timing.kernel_ms = 0.0;
    timing.d2h_ms = 0.0;
    timing.total_ms = 0.0;
}
}

bool init_cuda_preprocess_context(
    CudaPreprocessContext& ctx,
    int input_w,
    int input_h,
    int input_c,
    const PreprocessConfig& cfg
)
{
    release_cuda_preprocess_context(ctx);

    if(input_w <= 0 || input_h <= 0 || input_c != 3){
        return false;
    }

    const int input_step = input_w * input_c;
    
    ctx.input_bytes = calc_input_bytes(input_h, input_step);
    ctx.output_bytes = calc_output_bytes(cfg);

    if(!check_cuda(cudaMalloc(&ctx.d_input, ctx.input_bytes), "cudaMalloc ctx.d_input")){
        release_cuda_preprocess_context(ctx);
        return false;
    }
    if(!check_cuda(cudaMalloc(&ctx.d_output, ctx.output_bytes), "cudaMalloc ctx.d_output")){
        release_cuda_preprocess_context(ctx);
        return false;
    }

    ctx.allocated_input_w = input_w;
    ctx.allocated_input_h = input_h;
    ctx.allocated_input_c = input_c;
    ctx.initialized = true;

    return true;
}

bool ensure_cuda_preprocess_context(
    CudaPreprocessContext& ctx,
    int input_w,
    int input_h,
    int input_c,
    int input_step,
    const PreprocessConfig& cfg
)
{
    if(!ctx.initialized){
        return init_cuda_preprocess_context(ctx, input_w, input_h, input_c, cfg);
    }

    size_t current_input_bytes = calc_input_bytes(input_h, input_step);
    if(current_input_bytes != ctx.input_bytes){
        release_cuda_preprocess_context(ctx);
        return init_cuda_preprocess_context(ctx, input_w, input_h, input_c, cfg);
    }

    return true;
}

bool preprocess_cuda_reuse(
    const ImageView& input,
    const PreprocessConfig& cfg,
    TensorView& output,
    CudaPreprocessContext& ctx, 
    PreprocessTimingBreakdown& timing)
{
    reset_timing(timing);

    if(!input.data || !output.data){
        return false;
    }
    if(!ctx.initialized){
        return false;
    }

    const auto total_start = std::chrono::steady_clock::now();

    //create cuda event
    cudaEvent_t ev_start = nullptr;
    cudaEvent_t ev_stop = nullptr;

    if(!check_cuda(cudaEventCreate(&ev_start), "cudaEventCreate start")){
        return false;
    }
    if(!check_cuda(cudaEventCreate(&ev_stop), "cudaEventCreate stop")){
        cudaEventDestroy(ev_start);
        return false;
    }

    const size_t input_bytes = static_cast<size_t>(input.height) * static_cast<size_t>(input.step);
    const size_t output_bytes = calc_output_bytes(cfg);

    bool ok = true;
    float ms = 0.0f;

    do{
        if(!check_cuda(cudaEventRecord(ev_start), "event record h2d start")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaMemcpy(ctx.d_input, input.data, input_bytes, cudaMemcpyHostToDevice), 
                                   "cudaMemcpy HtoD")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventRecord(ev_stop), "event record h2d stop")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventSynchronize(ev_stop), "event sync h2d")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventElapsedTime(&ms, ev_start, ev_stop), "elapsed h2d")){
            ok = false;
            break;
        }
        timing.h2d_ms = static_cast<double>(ms);

        dim3 block(16, 16);
        dim3 grid((cfg.output_width + block.x - 1) / block.x,
                  (cfg.output_height + block.y - 1) / block.y);

        if(!check_cuda(cudaEventRecord(ev_start), "event record kernel start")){
            ok = false;
            break;
        }

        preprocess_kernel<<<grid, block>>>(
            ctx.d_input,
            input.width,
            input.height,
            input.channels,
            input.step,
            ctx.d_output,
            cfg);
        
        if(!check_cuda(cudaGetLastError(), "kernel launch")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventRecord(ev_stop), "event record kernel stop")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventSynchronize(ev_stop), "event sync kernel")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventElapsedTime(&ms, ev_start, ev_stop), "elapsed kernel")){
            ok = false;
            break;
        }
        timing.kernel_ms = static_cast<double>(ms);
        
        if(!check_cuda(cudaEventRecord(ev_start), "event record d2h start")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaMemcpy(output.data, ctx.d_output, output_bytes, cudaMemcpyDeviceToHost), "cudaMemcpu DtoH")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventRecord(ev_stop), "event record d2h stop")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventSynchronize(ev_stop), "event sync d2h stop")){
            ok = false;
            break;
        }
        if(!check_cuda(cudaEventElapsedTime(&ms, ev_start, ev_stop), "elapsed d2h")){
            ok = false;
            break;
        }
        timing.d2h_ms = static_cast<double>(ms);
    }while(false);

    cudaEventDestroy(ev_start);
    cudaEventDestroy(ev_stop);
    
    const auto total_end = std::chrono::steady_clock::now();
    timing.total_ms = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() / 1000.0;

    return ok;    
}

void release_cuda_preprocess_context(CudaPreprocessContext& ctx){
    // ctx.d_input and ctx.d_output are not nullptr
    if(ctx.d_input){
        cudaFree(ctx.d_input);
    }
    if(ctx.d_output){
        cudaFree(ctx.d_output);
    }

    ctx.d_input = nullptr;
    ctx.d_output = nullptr;
    ctx.input_bytes = 0;
    ctx.output_bytes = 0;
    ctx.allocated_input_w = 0;
    ctx.allocated_input_h = 0;
    ctx.allocated_input_c = 0;
    ctx.initialized = false;
}



__global__ void preprocess_kernel(const uint8_t* input, int input_width, int input_height,
    int input_channels, int input_step, float* output, PreprocessConfig cfg){

        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        const int y = blockIdx.y * blockDim.y + threadIdx.y;

        if(input_width <= 0 || input_height <= 0 || input_channels != 3){
            return;
        }
        if(input == nullptr || output == nullptr){
            return;
        }
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

bool preprocess_cuda(const ImageView& input, const PreprocessConfig& cfg, TensorView& output){
    if(!input.data || !output.data){
        return false;
    }

    if(input.channels != 3 || cfg.output_channels != 3){
        return false;
    }

    CudaPreprocessContext ctx;
    if(!init_cuda_preprocess_context(ctx, input.width, input.height, input.channels, cfg)){
        return false;
    }

    PreprocessTimingBreakdown timing;
    const bool ok = preprocess_cuda_reuse(input, cfg, output, ctx, timing);
    release_cuda_preprocess_context(ctx);
    return ok;
}
}
