#ifndef DATASET_H
#define DATASET_H

#include "camera.h"
#include "common_include.h"
#include "frame.h"

namespace myslam {
    /**
     * @details read data from dataset.
     * @details After Initialization, it can get camera and the next frame.
     */
    class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string& dataset_path);

        // initialization
        bool Init();

        // Create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        // get camera by id
        Camera::Ptr GetCamera(int camera_id) const {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };
} // namespace myslam

#endif // DATASET_H
