#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "yaml file locates in " << FLAGS_config_file << std::endl; // ../config/default.yaml
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}