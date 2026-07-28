#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

  namespace avp {

  struct Detection {
      cv::Rect2f box;      // original image coordinate (x, y, w, h)
      int   class_id;
      float score;
  };

  std::vector<Detection> decodeYolov5(
      const std::vector<float>& output,   // [1, num_boxes, num_attrs] normalization
      int num_boxes,                      // 25200
      int num_attrs,                      // 85 (=4 bbox + 1 obj + 80 cls)
      float conf_thresh,
      float nms_thresh,
      int orig_w,
      int orig_h,
      int input_size = 640);

  }  // namespace avp