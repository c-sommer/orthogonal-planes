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

#include "Corner.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include "CornerError.h"

Eigen::Matrix3f Corner::K = Eigen::Matrix3f::Identity();

/*
 * check if candidate really is on a corner, or if it's rather a virtual candidate - ply (unstructured) data
 */
std::vector<size_t> Corner::eval_candidate(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals,
                                           Vec3 n1, double d1, Vec3 n2, double d2, Vec3 n3, double d3, KDTree& index) {

    const size_t threshold = 5;

    Vec3 p = -(d1 * n1 + d2 * n2 + d3 * n3);
    Eigen::Vector3f pf = p.cast<float>();
    std::vector<size_t> indices;

	const float search_radius_sq = search_radius_ * search_radius_;
	std::vector<std::pair<size_t, float>> ret_matches;
	nanoflann::SearchParams params;
	params.sorted = false; // do not sort matches by distance to query point
    const size_t num_neighbors = index.radiusSearch(pf.data(), search_radius_sq, ret_matches, params);
    
    
    if (num_neighbors < threshold) {
        std::cout << num_neighbors << " inliers." << std::endl;
        return std::vector<size_t>();
    }
    
    size_t count1 = 0, count2 = 0, count3 = 0;
    
    for (const auto& i : ret_matches) {
        indices.push_back(i.first);
        Vec3 ni = normals[i.first].cast<double>();
        if (std::abs(n1.dot(ni)) > normal_thresh_) {
            ++count1;
        } else if (std::abs(n2.dot(ni)) > normal_thresh_) {
            ++count2;
        } else if (std::abs(n3.dot(ni)) > normal_thresh_) {
            ++count3;
        }
    }
    
//    // TODO: possibly replace direction check by plane check?!
//    for (const auto& i : indices) {
//        Vec3 pi = points[i].cast<double>()S;
//        if (std::abs(n1.dot(pi)+d1) < dist_thresh_) {
//            ++count1;
//        } else if (std::abs(n2.dot(pi)+d2) < dist_thresh_) {
//            ++count2;
//        } else if (std::abs(n3.dot(pi)+d3) < dist_thresh_) {
//            ++count3;
//        }
//    }    
    
    const size_t threshold_10 = threshold / 10;
    if (count1 > threshold_10 && count2 > threshold_10 && count3 > threshold_10) {
        return indices;
    }
    
    std::cout << num_neighbors << "\t" << count1 << "\t" << count2 << "\t" << count3 << std::endl;
    return std::vector<size_t>();

}

/*
 * refine corner parameters based on neighborhood points using Ceres
 */
bool Corner::refine(const std::vector<Eigen::Vector3f>& points) {
      
    // set up problem
    ceres::Problem problem;
    for (const auto i : indices_) {
        ceres::CostFunction* cost_function = new CornerError(points[i][0], points[i][1], points[i][2]);
        problem.AddResidualBlock(cost_function, nullptr, distances_.data(), rot_.data()); // robust: replace nullptr e.g. by new ceres::CauchyLoss(1.0), new ceres::ArctanLoss(1.0), new ceres::TukeyLoss(1.0)
    }
    problem.SetParameterization(rot_.data(), new Sophus::LocalParameterizationSO3);
    
    // set up solver
    ceres::Solver::Options options;
    options.max_num_iterations = 8;
    options.minimizer_progress_to_stdout = false;//true;
    ceres::Solver::Summary summary;
    
    // solve problem
    ceres::Solve(options, &problem, &summary);
    
//    std::cout << summary.BriefReport() << std::endl; // BriefReport or FullReport
    if (summary.termination_type == ceres::CONVERGENCE) {
        return true;
    }
    
    return false;
}
