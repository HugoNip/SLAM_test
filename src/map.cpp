#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

    void Map::InsertKeyFrame(Frame::Ptr frame) {
        current_frame_ = frame;

        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            /**
             * if current frame is not included in keyframes,
             * insert it into the keyframe, and activate it.
             * To activate it, this frame needs to be inserted
             * into the active_keyframes_ variable
             */
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        } else {
            /**
             * if the current is include in existed keyframes,
             * find it according to its id, and activate it
             */
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if (active_keyframes_.size() > num_active_keyframes_) {
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point) {
        if (landmarks_.find(map_point->id_) == landmarks_.end()) {
            // if there is a new landmark, insert to existed landmarks_ map with format of (id, MapPoint)
            // and then activate it
            landmarks_.insert(std::make_pair(map_point->id_, map_point));
            active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
        } else {
            // if this landmark has already been in existed landmarks_ map
            // find and activate it
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    // only 7 frames are keyframes
    void Map::RemoveOldKeyframe() {
        if (current_frame_ == nullptr) return;
        // find the frames with the minimum and maximum distance away from current frame
        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse(); // SE3, cv::Mat Twc =Tcw.inv()
        for (auto& kf : active_keyframes_) {
            // map.first, map.second
            if (kf.second == current_frame_) continue;
            auto dis = (kf.second->Pose() * Twc).log().norm();
            if (dis > max_dis) {
                max_dis = dis;
                max_kf_id = kf.first;
            }

            if (dis < min_dis) {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2; // minimum distance threshold
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th) {
            // if one keyframe have small distance with current frame, remove it
            // the current will replace this keyframe as a new keyframe
            frame_to_remove = keyframes_.at(min_kf_id);
        } else {
            // remove the farest keyframe
            frame_to_remove = keyframes_.at(max_kf_id);
        }

        LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
        // remove keyframe and its corresponding landmarks/MapPoints/observations
        // by detecting feature points in the frame
        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        // left frame
        for (auto feat : frame_to_remove->features_left_) {
            // std::vector<std::shared_ptr<Feature>> features_left_;
            auto mp = feat->map_point_.lock();
            // std::weak_ptr<MapPoint> map_point_;
            if (mp) {
                mp->RemoveObservation(feat);
            }
        }
        // right frame
        for (auto feat : frame_to_remove->features_right_) {
            if (feat == nullptr) continue;
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(feat);
            }
        }

        CleanMap();
    }

    // clean active MapPoints/landmarks
    void Map::CleanMap() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
            if (iter->second->observed_times_ == 0) {
                iter = active_landmarks_.erase(iter);
                cnt_landmark_removed++;
            } else
                ++iter;
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }

} // namespace myslam