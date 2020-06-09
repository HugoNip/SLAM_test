#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include "opencv2/features2d.hpp"
#include "common_include.h"

namespace myslam {

struct Frame;
struct MapPoint;

/**
 * 2D keypoint
 */
struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_; // corresponding map point

    bool is_outlier_ = false;
    bool is_on_left_image_ = true;

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
} // namespace myslam

#endif // FEATURE_H
