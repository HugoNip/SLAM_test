#pragma once

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "common_include.h"

namespace myslam {
    /**
     * linear triangulation with SVD
     * @param poses         poses,
     * @param points        points in normalized plane
     * @param pt_world      triangulated point in the world
     * @return true if success
     */
    inline bool triangulation(const std::vector<SE3> &poses,
            const std::vector<Vec3> points, Vec3 &pt_world) {
        MatXX D(2 * poses.size(), 4); // 2n x 4
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i) {
            Mat34 m = poses[i].matrix3x4();
            D.block<1, 4>(2 * i, 0) =
                    points[i][0] * m.row(2) - m.row(0); // [u_i P_i,3^T - P_i,1^T]
            D.block<1, 4>(2 * i + 1, 0) =
                    points[i][1] * m.row(2) - m.row(1); // [v_i P_i,3^T - P_i,2_T]
        }
        auto svd = D.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeFullV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>(); // y = u_4

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
            // bad results, give up
            return true;
        }
        return false;
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

} // namespace myslam

#endif // ALGORITHM_H
