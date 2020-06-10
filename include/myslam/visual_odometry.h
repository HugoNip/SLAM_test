#pragma once

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "backend.h"
#include "common_include.h"
#include "dataset.h"
#include "frontend.h"
#include "viewer.h"

namespace myslam{
    class VisualOdometry{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        // constructor with config file
        VisualOdometry(std::string &config_path);

        bool Init();

        void Run();

        bool Step();

        //
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        // dataset
        Dataset::Ptr dataset_ = nullptr;
    };

} // namespace myslam

#endif //VISUAL_ODOMETRY_H
