#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

    // create a new MapPoint/landmark and set a new id
    // landmarks_.find(map_point->id_) == landmarks_.end() in map.cpp
    MapPoint::Ptr MapPoint::CreateNewMappoint() { // Static functions in a class
        static long factory_id = 0; // static variable in a function, lifetime
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
            if (iter->lock() == feat) {

                observations_.erase(iter); // remove this feature
                /**
                 * // Erasing elements in between the list using
                 * // erase(position) member function. Let's iterate to 3rd position
                 * it = listOfNumbers.begin();
                 * it++;
                 * it++;
                 * // Iterator 'it' is at 3rd position. Now erase this element.
                 * listOfNumbers.erase(it);
                 */

                feat->map_point_.reset();
                observed_times_--;
                break;
            }
        }
    }
} // namespace myslam