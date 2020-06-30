#pragma once

#ifndef FRAME_H
#define FRAME_H

#include "camera.h"
#include "common_include.h"

namespace myslam {
// forward declare
struct MapPoint;
struct Feature;

/**
 * each frame has ID
 * each keyframe has keyID
 */
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double time_stamp_;
    SE3 pose_; // Tcw
    std::mutex pose_mutex_; // pose data lock
    cv::Mat left_img_, right_img_; // stereo images
    // extract features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

public:
    Frame() {}

    /**
     * constructor
     * @param id
     * @param time_stamp
     * @param pose
     * @param left
     * @param right
     */
    Frame(long id, double time_stamp, const SE3 &pose,
            const cv::Mat &left, const cv::Mat &right);

    /**
     * set and get pose, Tcw, thread safe
     * @return
     */
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    /**
     * @param pose
     * frame->pose_ is set
     * but got by frame.Pose(), e.g., current_frame_->Pose()
     */
    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /**
     * set keyframe and id
     * keyframes_.find(frame->keyframe_id_) == keyframes_.end() in map.cpp
     */
    void SetKeyFrame();

    /**
     * set factory mode and id
     * @return frame
     */
    static std::shared_ptr<Frame> CreateFrame(); // Static functions in a class/struct

};

} // namespace myslam

#endif //FRAME_H
