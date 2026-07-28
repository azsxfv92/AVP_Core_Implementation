#include "yolo_postprocess.hpp"
#include <opencv2/dnn.hpp>
#include <algorithm>

namespace avp {

std::vector<Detection> decodeYolov5(
    const std::vector<float>& output,
    int num_boxes,
    int num_attrs,
    float conf_thresh,
    float nms_thresh,
    int orig_w,
    int orig_h,
    int input_size)
{
    std::vector<cv::Rect> nms_boxes;
    std::vector<float>    scores;
    std::vector<int>      class_ids;

    const int num_classes = num_attrs - 5;

    // Step 1: iterate 25200 frames
    for (int i = 0; i < num_boxes; ++i) {
        const float* data = output.data() + (i*num_attrs);
        float obj_conf = data[4];
        if(obj_conf < conf_thresh) continue;
        
        float best_prob = 0.0f; 
        int   best_cls  = 0;
        
        for(int j=5; j < num_attrs; j++){
        if(best_prob < data[j]){
            best_prob = data[j];
            best_cls = j-5;
        }
        }

        float final_score = obj_conf * best_prob;
        if (final_score < conf_thresh) continue;

        float cx = data[0];
        float cy = data[1];
        float w = data[2];
        float h = data[3];

        nms_boxes.emplace_back(
            static_cast<int>(cx - w*0.5f),
            static_cast<int>(cy - h*0.5f),
            static_cast<int>(w),
            static_cast<int>(h));
        scores.push_back(final_score);
        class_ids.push_back(best_cls);
    }

    // Step 4 : NMS
    std::vector<int> keep;
    cv::dnn::NMSBoxes(nms_boxes, scores, conf_thresh, nms_thresh, keep);

    // Step 5 : 640 coordinate → restore original coordinate
    const float scale_x = static_cast<float>(orig_w) / static_cast<float>(input_size);
    const float scale_y = static_cast<float>(orig_h) / static_cast<float>(input_size);

    std::vector<Detection> results;
    results.reserve(keep.size());

    for (int idx : keep) {
        const cv::Rect& b = nms_boxes[idx];
        const float bx = b.x * scale_x;
        const float by = b.y * scale_y;
        const float bw = b.width * scale_x;
        const float bh = b.height * scale_y;

        const float x1 = std::max(0.0f, bx);
        const float y1 = std::max(0.0f, by);
        const float x2 = std::min(static_cast<float>(orig_w), bx + bw);
        const float y2 = std::min(static_cast<float>(orig_h), by + bh);

        Detection d;
        d.box = cv::Rect2f(x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1));
        d.class_id = class_ids[idx];
        d.score = scores[idx];
        results.push_back(d);
    }

    return results;
}

} 
