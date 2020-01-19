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

#include "ParallelPlaneGraph.h"
// libraries
#include <ceres/ceres.h>
// custom functions
#include "ceres/LocalParamS2.h"
#include "ceres/WeightedPlaneConfigurationCost.h"
#include "ceres/OrthogonalityConstraint.h"

int WeightedPlaneConfigurationCost::num_ns = 0;
std::vector<int> WeightedPlaneConfigurationCost::num_ds = std::vector<int>();

/*
 * refine normals and distances of planes using CERES with a point cloud as obtained from reading a ply file
 */
void ParallelPlaneGraph::refine(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double lambda, double truncation, int sampling) {

    std::vector<double*> parameter_blocks;
    
    add_parameter_blocks(parameter_blocks, WeightedPlaneConfigurationCost::num_ns, WeightedPlaneConfigurationCost::num_ds);
    
    ceres::Problem problem;
    
    // data term
    const int num_points = points.size();
    for (int i=0; i<num_points; i+=sampling) {
        WeightedPlaneConfigurationCost* cost_function = new WeightedPlaneConfigurationCost(points[i][0], points[i][1], points[i][2], normals[i][0], normals[i][1], normals[i][2], truncation);
        problem.AddResidualBlock(cost_function, nullptr, parameter_blocks); // add to problem
    }
    
    // orthogonality regularizer
    for (const auto &e : edges_) {
        double* n1 = get_ni(e.first);
        double* n2 = get_ni(e.second);
        OrthogonalityConstraint::Cost* cost_function = new OrthogonalityConstraint::Cost(new OrthogonalityConstraint());
        problem.AddResidualBlock(cost_function, new ceres::ScaledLoss(nullptr, lambda, ceres::TAKE_OWNERSHIP), n1, n2);
    }
    
    // unit norm parameterization
    for (int in = 0; in < num_vertices_; ++in) {
        double* n = get_ni(in);
        problem.SetParameterization(n, new ceres::LocalParamS2);
    }
    
    ceres::Solver::Options options;
    options.function_tolerance = 1e-4; // default: 1e-6, increase to speed up convergence
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    
    return;
}
