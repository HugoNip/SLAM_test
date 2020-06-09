#pragma once

#ifndef BACKEND_H
#define BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
    class Map;

    /**
     * Backend
     * has independent optimized thread, it will start when the map updates
     * the upating of map is activated by frondend
     */
    class Backend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        Backend();

        // set left and right cameras and get camera intrinsics
        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            cam_left_ = left;
            cam_right_ = right;
        }

        void Setmap(std::shared_ptr<Map> map) { map_ = map; }

        void UpdateMap();

        void Stop();

    private:
        void BackendLoop();

        // optimize the keyframe and landmarks
        void Optimize(Map::KeyframeType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

    };
} // namespace myslam

#endif // BACKEND_H
