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

    private:
        bool inited_ = false;
        std::string config_file_path_;


    };
}




#endif //VISUAL_ODOMETRY_H
