/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Christiane Sommer, Yumin Sun.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "PairDetector.h"
// standard library
#include <random>
//libraries
#include <opencv2/highgui/highgui.hpp>
#include <nanoflann/nanoflann.hpp>

struct PointCloudAdaptor {

    const std::vector<Eigen::Vector3f>& points_;
    
    const size_t num_points;

    PointCloudAdaptor(const std::vector<Eigen::Vector3f>& points) :
        points_(points),
        num_points(points.size())
    {}

    inline size_t kdtree_get_point_count() const {
        return num_points;
    }
    
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        return points_[idx][dim];
    }
    
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }

};

/*
 * plane pair detection from std::vector of points - ref and pair are the same
 */
void ppf::PairDetector::detect_ortho_pairs(const std::vector<Vec6> &points, PlaneGraph &planeMap) {

    // number of points
    const size_t num_points = points.size();
    const size_t num_points_threshold = num_points - 2*vote_threshold_;

    for (size_t fir = 0; fir < num_points_threshold; ++fir) { // reference point

        Eigen::Matrix<uint16_t, 2*num_rho_+1, num_theta_> acc;
        acc.setZero();
        int count_plane = 0;

        Vec3 pr = points[fir].segment<3>(0);
        Vec3 nr = points[fir].segment<3>(3);
        // nr.normalize();
        Sophus::SO3<double> rot = align_so3(nr);

        //! Vote
        for (size_t sec = fir+1; sec < num_points; ++sec) { // pair points
            Vec3 pi = points[sec].segment<3>(0);
            Vec3 ni = points[sec].segment<3>(3);
            // ni.normalize();
            Vec4 cppf = pair_feature(nr, ni, pr - pi);
            if (cppf[0] > para_thresh_ && std::abs(cppf[1]) < ortho_thresh_*cppf[3] && std::abs(cppf[2]) < ortho_thresh_*cppf[3]) {
                ++count_plane;
            }
            if (cppf[3] < dmax_ && cppf[3] > dmin_ && std::abs(cppf[0]) < ortho_thresh_) {
                int theta_i = theta2idx(compute_theta(rot, ni));
                int rho_i = rho2idx(cppf[2]);
                ++acc(rho_i, theta_i); // actual voting
            }   // norm_d in range {min, max} & orthogonality
        }     // second pt

        //! Threshold votes
        if (count_plane > vote_threshold_) {
//            std::cout << "Plane votes:\t" << count_plane << std::endl;
            orthogonal_vote_single(rot, pr, nr, acc, planeMap); // only take one maximum per reference point
//            orthogonal_vote_multi(rot, pr, nr, acc, planeMap); // take all pairs above threshold
        }
    } // first pt

}

/*
 * helper to generate a set of random indices
 */
void random_indices(int num_points, int N, std::vector<int>& idx) {

    idx.clear();
    for (int k=N; k>0; --k) {
        int out = std::rand() % num_points;
        idx.push_back(out);
    }
    
    return;
}

/*
 * plane pair detection from two std::vectors (points and normals) - neighborhood sampling
 */
void ppf::PairDetector::detect_ortho_pairs(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, PlaneGraph &planeMap) {

    // number of points
    const size_t num_points = points.size();

    std::vector<int> idx;
    const size_t N = std::max(points.size() / 500, (size_t)(500));
    random_indices(num_points, N, idx);

    // neighborhood search: Kd-Tree    
    const PointCloudAdaptor pc2kd(points);
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3>;    
    KDTree index(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10)); // arguments: dimensionality, point cloud adaptor, max leafs
    index.buildIndex();
	const float search_radius_sq = dmax_ * dmax_;
	std::vector<std::pair<size_t, float>> ret_matches;
	nanoflann::SearchParams params;
	params.sorted = false; // do not sort matches by distance to query point
    
    for (size_t fir = 0; fir < N; ++fir) { // reference point

        Eigen::Matrix<uint16_t, 2*num_rho_+1, num_theta_> acc;
        acc.setZero();
        int count_plane = 0;

        Vec3 pr = points[idx[fir]].cast<double>();
        Vec3 nr = normals[idx[fir]].cast<double>();
        // nr.normalize();
        Sophus::SO3<double> rot = align_so3(nr);
       
        ret_matches.clear();
	    size_t num_neighbors = index.radiusSearch(points[idx[fir]].data(), search_radius_sq, ret_matches, params);

        //! Vote
        num_neighbors = std::min(size_t(250), num_neighbors);
        for (size_t sec = 0; sec < num_neighbors; ++sec) { // pair points
            Vec3 pi = points[ret_matches[sec].first].cast<double>();
            Vec3 ni = normals[ret_matches[sec].first].cast<double>();
            // ni.normalize();
            Vec4 cppf = pair_feature(nr, ni, pr - pi);
            if (cppf[0] > para_thresh_ && std::abs(cppf[1]) < ortho_thresh_*cppf[3] && std::abs(cppf[2]) < ortho_thresh_*cppf[3]) {
                ++count_plane;
            }
            if (cppf[3] > dmin_ && std::abs(cppf[0]) < ortho_thresh_) {
                int theta_i = theta2idx(compute_theta(rot, ni));
                int rho_i = rho2idx(cppf[2]);
                ++acc(rho_i, theta_i); // actual voting
            }   // norm_d in range {min, max} & orthogonality
        }     // second pt

        //! Threshold votes
//        std::cout << "number of plane inliers: " << count_plane << std::endl; // TODO: remove debug information
        if (count_plane > vote_threshold_) {
//            std::cout << "Plane votes:\t" << count_plane << std::endl;
//            orthogonal_vote_single(rot, pr, nr, acc, planeMap); // only take one maximum per reference point
            orthogonal_vote_multi(rot, pr, nr, acc, planeMap); // take all pairs above threshold
        }
    } // first pt

}
