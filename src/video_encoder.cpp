#include "video_encoder.hpp"

#include <cstring>
#include <iostream>
#include <sstream>

namespace avp_core_implementation
{

VideoEncoder::VideoEncoder()
{
    static bool gst_initialized = false;
    if(!gst_initialized){
        gst_init(nullptr, nullptr);
        gst_initialized = true;
    }
}

VideoEncoder::~VideoEncoder()
{
    close();
}

bool VideoEncoder::open(
  const std::string & output_path,
  int width,
  int height,
  int fps
)
{
  if (opened_) {
    return true;
  }

  if (width <= 0 || height <= 0 || fps <= 0) {
    std::cerr << "[VideoEncoder] Invalid encoder config: "
              << width << "x" << height << " fps=" << fps << std::endl;
    return false;
  }

  width_ = width;
  height_ = height;
  fps_ = fps;
  frame_index_ = 0;

  std::ostringstream pipeline_desc;

  pipeline_desc
    << "appsrc name=src is-live=true format=time do-timestamp=true "
    << "caps=video/x-raw,format=BGR,width=" << width_
    << ",height=" << height_
    << ",framerate=" << fps_ << "/1 "
    << "! queue "
    << "! videoconvert "
    << "! video/x-raw,format=I420 "
    << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 "
    << "! h264parse "
    << "! mp4mux faststart=true "
    << "! filesink location=" << output_path << " sync=false async=false";

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_desc.str().c_str(), &error);

  if (error) {
    std::cerr << "[VideoEncoder] Failed to create pipeline: "
              << error->message << std::endl;
    g_error_free(error);
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    return false;
  }

  if (!pipeline_) {
    std::cerr << "[VideoEncoder] Failed to create pipeline" << std::endl;
    return false;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
  if (!appsrc_) {
    std::cerr << "[VideoEncoder] Failed to get appsrc" << std::endl;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
  gst_app_src_set_max_bytes(GST_APP_SRC(appsrc_), static_cast<guint64>(width_ * height_ * 3 * 4));
  gst_app_src_set_latency(GST_APP_SRC(appsrc_), 0, GST_CLOCK_TIME_NONE);

  const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "[VideoEncoder] Failed to set pipeline to PLAYING" << std::endl;
    close();
    return false;
  }

  opened_ = true;

  std::cout << "[VideoEncoder] Opened: " << output_path
            << " (" << width_ << "x" << height_
            << " @" << fps_ << "fps)" << std::endl;

  return true;
}

bool VideoEncoder::write_bgr(const cv::Mat & bgr_frame)
{
  if(!opened_ || !pipeline_ || !appsrc_){
    return false;
  } 

  if(bgr_frame.empty()){
    std::cerr << "[VideoEncoder] Empty frame" << std::endl;
    return false;
  }

  // frame size is wrong 
  if(bgr_frame.cols != width_ || bgr_frame.rows != height_){
    std::cerr << "[VideoEncoder] Frame size mismatch. expected="
              << width_ << "x" << height_
              << " actual=" << bgr_frame.cols << "x" << bgr_frame.rows
              << std::endl;
    return false;
  }

  // typre check 
  if(bgr_frame.type() != CV_8UC3){
    std::cerr << "[VideoEncoder] Frame type must be CV_8UC3(BGR)" << std::endl;
    return false;
  }

  const std::size_t frame_size = 
    static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_) * 3;
  
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, frame_size, nullptr);
  if (!buffer) {
    std::cerr << "[VideoEncoder] Failed to allocate GstBuffer" << std::endl;
    return false;
  }

  GstMapInfo map;
  if(!gst_buffer_map(buffer, &map, GST_MAP_WRITE)){
    std::cerr << "[VideoEncoder] Failed to map GstBuffer" << std::endl;
    gst_buffer_unref(buffer);
    return false;
  }

  if(bgr_frame.isContinuous()){
    std::memcpy(map.data, bgr_frame.data, frame_size);
  }
  else{
    const std::size_t row_size = static_cast<std::size_t>(width_) * 3;
    for(int r = 0; r < height_; ++r){
      std::memcpy(map.data + static_cast<std::size_t>(r) * row_size,
                  bgr_frame.ptr(r), row_size);
    }
  }

  gst_buffer_unmap(buffer, &map);

  const GstClockTime duration = static_cast<GstClockTime>(GST_SECOND / static_cast<guint64>(fps_));

  GST_BUFFER_PTS(buffer) = frame_index_ * duration;
  GST_BUFFER_DTS(buffer) = frame_index_ * duration;
  GST_BUFFER_DURATION(buffer) = duration;
  frame_index_++;

  const GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if(ret != GST_FLOW_OK){
    std::cerr << "[VideoEncoder] gst_app_src_push_buffer failed: " << ret << std::endl;
    return false;
  }

  return true;
}

void VideoEncoder::close()
{
  if(!pipeline_){
    opened_ = false;
    appsrc_ = nullptr;
    return;
  }

  if(appsrc_){
    gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
  }

  gst_element_set_state(pipeline_, GST_STATE_NULL);

  if(appsrc_){
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  gst_object_unref(pipeline_);
  pipeline_ = nullptr;
  opened_ = false;
}



bool VideoEncoder::is_open() const
{
  return opened_;
}
}
