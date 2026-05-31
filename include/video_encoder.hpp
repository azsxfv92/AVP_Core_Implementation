#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

namespace avp_core_implementation
{
// class for saving videos
class VideoEncoder
{
public:
    VideoEncoder();
    ~VideoEncoder();

    bool open(
        const std::string & output_path,
        int width,
        int height,
        int fps
    );

    bool write_bgr(const cv::Mat & bgr_frame);
    void close();
    bool is_open() const;

private:
    GstElement * pipeline_{nullptr};
    GstElement * appsrc_{nullptr};

    int width_{0};
    int height_{0};
    int fps_{30};
    uint64_t frame_index_{0};
    bool opened_{false};
};
}