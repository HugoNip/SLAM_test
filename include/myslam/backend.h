#ifndef BACKEND_H
#define BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
    class Map;

    /**
     * @details Backend
     * @details it has independent optimized thread,
     * @details it will start when the map updates
     * @details the updating of map is activated by frondend.cpp
     * @details backend_->UpdateMap()
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

        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        void UpdateMap();

        void Stop();

    private:
        void BackendLoop();

        // optimize the keyframe and landmarks
        void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

    };
} // namespace myslam

#endif // BACKEND_H
