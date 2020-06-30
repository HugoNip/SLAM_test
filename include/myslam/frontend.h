#pragma once

#ifndef FRONTEND_H
#define FRONTEND_H

#include <opencv2/features2d.hpp>
#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam {
    class Backend;
    class Viewer;

    // three status in the frontend
    enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

    /**
     * @details frondend
     * @details compute pose of current frame, add the frame to map
     * @details when it satisfies the condition of being a keyframe
     */
    class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        bool AddFrame(Frame::Ptr frame);

        void SetMap(Map::Ptr map) { map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        FrontendStatus GetStatus() const { return status_; }

        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        /**
         * @details Track in normal mode
         * @details bad status
         * @return true if success
         */
        bool Track();

        /**
         * @details Reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * @details Track with last frame
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * @details 3D-2D PnP, only for camera pose
         * @details It only use left camera to estimate current frame's pose
         * @details this problem can be regarded as mono VO,
         * @details so the pose need to be initialized firstly.
         * @details we know:
         * @details 1. initialized_pose: current_frame_->Pose()
         * @details 2. left_camera_intrinsic: K
         * @details 3. 2D_features: current_frame_->features_left_
         * @details 4. 3D_MapPoint/landmark_position: mp->pos_
         * @return num of inliers/tracking features
         */
        int EstimateCurrentPose();

        /**
         * @details set current frame as a keyframe and insert it to backend
         * @return true if success
         */
        bool InsertKeyframe();

        /**
         * @details Try initialize the frontend with stereo images saved in current_frame_
         * @return true if success
         */
        bool StereoInit();

        /**
         * @details Detect features in left image in current_frame_
         * @details keypoints will be saved in current_frame_
         * @return num of features found
         */
        int DetectFeatures();

        /**
         * @details Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * @details Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * @details Triangulate the 2D points in current frame
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * @details Set the features in keyframe as new observation of the map points
         */
        void SetObservationsForKeyFrame();

        // data
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;
        Frame::Ptr last_frame_ = nullptr;
        Camera::Ptr camera_left_ = nullptr;
        Camera::Ptr camera_right_ = nullptr;

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        // the relative motion between current frame and the last frame,
        // this variable is used to calculate the initial pose of current frame
        // relative_motion_ * last_frame_->Pose()
        SE3 relative_motion_;

        int tracking_inliers_ = 0; // inliers, used for testing new keyframes

        // params
        int num_features_ = 150;
        int num_features_init_ = 50;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv

    };
} // namespace myslam

#endif // FRONTEND_H
