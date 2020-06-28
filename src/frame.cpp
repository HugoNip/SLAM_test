#include "myslam/frame.h"

namespace myslam {

    // constructor
    Frame::Frame(long id, double time_stamp, const SE3 &pose,
            const cv::Mat &left, const cv::Mat &right)
            : id_(id), time_stamp_(time_stamp), pose_(pose),
            left_img_(left), right_img_(right) {}

    // create the frame, may be not the keyframe
    Frame::Ptr Frame::CreateFrame() {
            static long factory_id = 0; // static variable in a function
            Frame::Ptr new_frame(new Frame);
            new_frame->id_ = factory_id++; // pointer
            return new_frame;
    }

    // set keyframe, selected from existed frames
    void Frame::SetKeyFrame() {
        /**
         * static variable in a function,
         * it gets allocated for the lifetime of the program,
         * the space for it is allocated only once
         */
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
        /**
         * keyframes_.find(frame->keyframe_id_) == keyframes_.end()
         */
    }
}