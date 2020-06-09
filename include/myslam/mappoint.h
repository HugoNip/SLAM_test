#pragma once

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"

namespace myslam {
    struct Frame;
    struct Feature;

    /**
     * landmark
     * feature converts to landmark after trianglation
     */
    struct MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_ = 0; // ID
        bool is_outlier_ = false;
        Vec3 pos_ = Vec3::Zero(); // position in the world
        std::mutex data_mutex_;
        int observed_times_ = 0; // being observed by feature matching algorithm
        std::list<std::weak_ptr<Feature>> observations_;

        MapPoint() {};

        MapPoint(long id, Vec3 position);

        Vec3 Pos() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        void SetPos(const Vec3 &pos) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        }

        void AddObservation(std::shared_ptr<Feature> feature) {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_++;
        }

        void RemoveObservation(std::shared_ptr<Feature> feat);

        std::list<std::weak_ptr<Feature>> GetObs() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }

        // factory function
        static MapPoint::Ptr CreateNewMappoint(); // Static functions in a class
    };
} // namespace myslam

#endif // MAPPOINT_H
