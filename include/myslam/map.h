#pragma once

#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam{
    /**
     * interact with map
     * for frontend, use InsertKeyframe and InsertMapPoint to insert new keyframe and MapPoint
     * for backend, improve the map, recognize the outlier and remove it
     */
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        void InsertKeyFrame(Frame::Ptr frame);
        void InsertMapPoint(MapPoint::Ptr map_point);

        // get all MapPoints
        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }

        // get all keyframes
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        // get active MapPoints
        LandmarksType GetActiveMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        // get active keyframes
        KeyframesType GetActiveKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        // clear the point in the map which has 0 observation
        void CleanMap();

    private:
        // Set old keyframe to inactive status
        void RemoveOldKeyframe();

        std::mutex data_mutex_; // lock
        LandmarksType landmarks_; // all landmarks
        LandmarksType active_landmarks_; // active landmarks
        KeyframesType keyframes_; // all keyframes
        KeyframesType active_keyframes_; // active keyframes

        Frame::Ptr current_frame_ = nullptr;

        // settings
        int num_active_keyframes_ = 7;
    };
} // namespace myslam

#endif // MAP_H
