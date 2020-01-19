/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Christiane Sommer.
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

#ifndef CORNER_H_
#define CORNER_H_

// includes
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
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

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 3>;


/**
  * class Corner
  */
class Corner {

    using Vec3 = Eigen::Vector3d;

// variables

    Vec3 distances_;
    Sophus::SO3<double> rot_;
    
    std::vector<size_t> indices_;
    
    static constexpr double search_radius_ = 0.12;
    static constexpr double search_radius_sq_ = search_radius_ * search_radius_;
    
    static constexpr double normal_thresh_ = std::cos(20. * M_PI / 180.); // 20 deg
    static constexpr double view_thresh_ = std::sin(3. * M_PI / 180.) * std::sin(3. * M_PI / 180.); // 3 deg
    static constexpr double dist_thresh_ = 0.01; // 1cm

public:

// constructors / destructors

    Corner() :
        distances_(Vec3::Zero()),
        rot_(Sophus::SO3<double>())
    {}
    
    Corner(Vec3 n1, double d1, Vec3 n2, double d2, Vec3 n3, double d3) {
        // set up R matrix, R = (n1'; n2'; n3')
        Eigen::Matrix3d R;
        R.row(0) = n1;
        R.row(1) = n2;
        Vec3 n1xn2 = n1.cross(n2);
        if (n1xn2.dot(n3) > 0) {
            R.row(2) = n3;
            distances_ = Vec3(d1, d2, d3);
        }
        else {
            R.row(2) = -n3;
            distances_ = Vec3(d1, d2, -d3);
        }
        // do SVD on R to orthogonalize: R <- U * V^T
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::NoQRPreconditioner> svd(R, Eigen::ComputeFullV | Eigen::ComputeFullU );
        R = svd.matrixU() * svd.matrixV().transpose();
        // init Sophus rotation by R
        rot_ = Sophus::SO3<double>(R);
    }
    
    Corner(Eigen::Matrix3d R, Vec3 p) {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullV | Eigen::ComputeFullU);
        R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            Vec3 make_rot(1., 1., -1.);
            R = svd.matrixU() * make_rot.asDiagonal() * svd.matrixV().transpose();
        }
        distances_ = -R * p;
        rot_ = Sophus::SO3<double>(R);
    }

// variables
    
    static Eigen::Matrix3f K;

// methods

    Vec3 distances() const {
        return distances_;
    }
    
    Sophus::SO3<double> rot() const {
        return rot_;
    }

    Vec3 position() const {
        return -(rot_.inverse() * distances_);
    }
    
    double dist(const Corner& other) {
        Vec3 diff = position() - other.position();
        return diff.norm();
    }
    
    static void set_K(Eigen::Matrix<float, 3, 3> Kf) {
        K = Kf;
    }

    static void set_K(Eigen::Matrix<double, 3, 3> Kd) {
        K = Kd.cast<float>();
    }
    
    void set_indices(const std::vector<size_t>& indices) {
        indices_ = indices;
    }
    
    void print()
    {
        std::cout << "-- a corner --"<< std::endl;
        std::cout << rot_.params() << std::endl;
        std::cout << distances_ << std::endl;
        std::cout << "-- -------- --"<< std::endl;
    }
    
    static void set_index(const std::vector<Eigen::Vector3f>& points);
    
    static std::vector<size_t> eval_candidate(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals,
                                              Vec3 n1, double d1, Vec3 n2, double d2, Vec3 n3, double d3, KDTree& index);
    
    bool refine(const std::vector<Eigen::Vector3f>& points);
    
// friend functions

    friend std::ostream& operator<<(std::ostream& os, const Corner& C);

};

/**
 * ostream definition
 */
inline std::ostream& operator<<(std::ostream& os, const Corner& C) {

    os << C.position().transpose() << std::endl << C.rot_.matrix();  
    return os;
    
}

#endif // CORNER_H_
