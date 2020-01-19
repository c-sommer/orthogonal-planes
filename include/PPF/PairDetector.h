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

#ifndef PAIR_DETECTOR_H_
#define PAIR_DETECTOR_H_

// includes
#include <iostream>
#include "definitions.h"
// libraries
#include <Eigen/Dense>
#include <sophus/so3.hpp>
// classes
#include "Plane.h"
#include "graph/PlaneGraph.h"

namespace ppf {

class PairDetector {

// variables

    static constexpr int num_theta_ = 36; // corresponds to 360 / num_theta degrees
    static constexpr int num_rho_ = 12; // directly related to diameter or dmax, depending on implementation
    
    // helper variables
    static constexpr int num_theta_half_ = num_theta_ / 2;
    static constexpr double theta_prefactor_ = .5 * num_theta_ / M_PI;
    static constexpr double theta_prefactor_inv_ = 2 * M_PI / num_theta_;
    
    const double dmax_;
    const double dmin_;
    const double rho_prefactor_;
    const double rho_prefactor_inv_;
    
    const double ortho_thresh_;
    const double para_thresh_;
    
    const int vote_threshold_;

// experimental stuff, test whether decompose the vote_threshold_ will be more better for orthogonality
    const int plane_vote_thresh_;

    const int orth_vote_thresh_;


// member functions

    // pair feature computation (scalar product PPFs)
    Vec4 pair_feature(const Vec3 &nr, const Vec3 &ni, const Vec3 &d) {
        return Vec4(nr.dot(ni), nr.dot(d), ni.dot(d), d.norm());
    }
    
    static double compute_theta(const Sophus::SO3<double>& rot, const Vec3& n2) {
        Vec3 R_n2 = rot * n2;
        return std::atan2(R_n2[1], R_n2[0]);
    }
    
    int theta2idx(double theta) {
        int idx = static_cast<int>(theta * theta_prefactor_ + num_theta_half_ + .5);
        return idx % num_theta_;
    }
    
    double idx2theta(int idx) {
        return (idx - .5) * theta_prefactor_inv_ - M_PI;
    }
    
    int rho2idx(double rho) {
        return static_cast<int>(rho * rho_prefactor_ + num_rho_ + .5);
    }
    
    double idx2rho(int idx) {
        return (idx - .5) * rho_prefactor_inv_ - dmax_;
    }
    
    static Sophus::SO3<double> align_so3(Vec3 n) {
    
        n.normalize();
        if (n[2] < 1e-4-1) {
            return Sophus::SO3<double>(Eigen::Quaternion<double>(.0, .0, 1., .0));
        }
        double s = std::sqrt(.5*(1+n[2]));
        double inv_2s = .5 / s;
        return Sophus::SO3<double>(Eigen::Quaternion<double>(s, inv_2s * n[1], -inv_2s * n[0], 0));   
    }

public:

// constructors / destructors

    PairDetector() :
        dmax_(1.0),
        dmin_(0.05),
        rho_prefactor_(num_rho_ / dmax_),
        rho_prefactor_inv_(dmax_ / num_rho_),
        ortho_thresh_(std::sin(M_PI / num_theta_half_)),
        para_thresh_(std::cos(M_PI / num_theta_half_)),
        vote_threshold_(4),
        plane_vote_thresh_(4),
        orth_vote_thresh_(4)
    {}
    
    PairDetector(double dmax, double dmin) :
        dmax_(dmax),
        dmin_(dmin),
        rho_prefactor_(num_rho_ / dmax_),
        rho_prefactor_inv_(dmax_ / num_rho_),
        ortho_thresh_(std::sin(M_PI / num_theta_half_)),
        para_thresh_(std::cos(M_PI / num_theta_half_)),
        vote_threshold_(4),
        plane_vote_thresh_(4),
        orth_vote_thresh_(4)
    {}
    
    PairDetector(int min_votes) :
        dmax_(1.0),
        dmin_(0.05),
        rho_prefactor_(num_rho_ / dmax_),
        rho_prefactor_inv_(dmax_ / num_rho_),
        ortho_thresh_(std::sin(M_PI / num_theta_half_)),
        para_thresh_(std::cos(M_PI / num_theta_half_)),
        vote_threshold_(min_votes-1),
        plane_vote_thresh_(min_votes-1),
        orth_vote_thresh_(min_votes-1)
    {}
    
    PairDetector(double dmax, double dmin, int min_votes) :
        dmax_(dmax),
        dmin_(dmin),
        rho_prefactor_(num_rho_ / dmax_),
        rho_prefactor_inv_(dmax_ / num_rho_),
        ortho_thresh_(std::sin(M_PI / num_theta_half_)),
        para_thresh_(std::cos(M_PI / num_theta_half_)),
        vote_threshold_(min_votes-1),
        plane_vote_thresh_(min_votes-1),
        orth_vote_thresh_(min_votes-1)
    {}

    PairDetector(double dmax, double dmin, int min_plane_votes, int min_orth_vote)
        : dmax_(dmax)
        , dmin_(dmin)
        , rho_prefactor_(num_rho_ / dmax_)
        , rho_prefactor_inv_(dmax_ / num_rho_)
        , ortho_thresh_(std::sin(M_PI / num_theta_half_))
        , para_thresh_(std::cos(M_PI / num_theta_half_))
        , vote_threshold_(min_plane_votes),
          plane_vote_thresh_(min_plane_votes),
        orth_vote_thresh_ (min_orth_vote)
    {}

    // getters

    double ortho_thresh() {
        return ortho_thresh_;
    }
    
    double para_thresh() {
        return para_thresh_;
    }
    
    double distance_bin() {
        return rho_prefactor_inv_;
    }

// other member functions

    void orthogonal_vote_single(const Sophus::SO3<double>& rot, const Vec3& pr, const Vec3& nr, Eigen::Matrix<uint16_t, 2*num_rho_+1, num_theta_>& acc, PlaneGraph& planeMap) {
        int max_rho_idx, max_theta_idx;
        uint16_t max_vote = acc.maxCoeff(&max_rho_idx, &max_theta_idx);
//        std::cout<<"orth max vote:"<<max_vote<<std::endl;
        if (max_vote > vote_threshold_) {
            double theta = idx2theta(max_theta_idx); //! Recover theta
            double rho = idx2rho(max_rho_idx); //! Recover rho
            Vec3 n2 = rot.inverse() * Vec3(std::cos(theta), std::sin(theta), 0); //! Rotate n2 from XY-Plane back to original frame
            double d2 = rho - n2.dot(pr); //! Compute d2
//            planeMap.insert_pair(Plane(nr, -nr.dot(pr)), Plane(n2, d2)); //! Insert to orthogonal plane list of ref plane
            planeMap.insert_pair(Plane(nr, -nr.dot(pr), pr), Plane(n2, d2, pr - rho*n2)); //! Insert to orthogonal plane list of ref plane
        }
    }

    void orthogonal_vote_multi(const Sophus::SO3<double>& rot, const Vec3& pr, const Vec3& nr, Eigen::Matrix<uint16_t, 2*num_rho_+1, num_theta_>& acc, PlaneGraph& planeMap) {
        int max_rho_idx, max_theta_idx;
        uint16_t max_vote = acc.maxCoeff(&max_rho_idx, &max_theta_idx);
        std::vector<Plane> vs;
        while (max_vote > vote_threshold_) {
            double theta = idx2theta(max_theta_idx); //! Recover theta
            double rho = idx2rho(max_rho_idx); //! Recover rho
            Vec3 n2 = rot.inverse() * Vec3(std::cos(theta), std::sin(theta), 0); //! Rotate n2 from XY-Plane back to original frame
            double d2 = rho - n2.dot(pr); //! Compute d2
            vs.push_back(Plane(n2, d2, pr - rho*n2)); //! Insert to orthogonal plane list of ref plane
            acc(max_rho_idx, max_theta_idx) = 0; //! Supress and update next max
            max_vote = acc.maxCoeff(&max_rho_idx, &max_theta_idx);
        }
        if (vs.size()>0) {
            planeMap.insert_pairs(Plane(nr, -nr.dot(pr), pr), vs);
        }
    }

    void detect_ortho_pairs(const std::vector<Vec6> &points, PlaneGraph &planeMap);
    
    void detect_ortho_pairs(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, PlaneGraph &planeMap);
    
};

} // namespace ppf

#endif // PAIR_DETECTOR_H_
