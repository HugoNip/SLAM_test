#include "myslam/frame.h"

namespace myslam {

Frame::Frame(long id, double time_stamp, const SE3 &pose,
        const cv::Mat &left, const cv::Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose),
        left_img_(left), right_img_(right) {}

Frame::Ptr Frame::CreateFrame() { // Static functions in a class
        static long factory_id = 0; // static variable in a function
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }

void Frame::SetKeyFrame() {
        static long keyframe_factory_id = 0; // static variable in a function
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }
}