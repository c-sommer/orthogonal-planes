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

#ifndef PLANE_CONFIGURATION_COST_H_
#define PLANE_CONFIGURATION_COST_H_

#include <vector>
//libraries
#include <ceres/ceres.h>

/*
 * cost function containing data term residual (minimal distance from any of the planes) and its derivatives
 */
class PlaneConfigurationCost : public ceres::CostFunction {

    double x_, y_, z_;
    double truncation_;

public:

    PlaneConfigurationCost(double x, double y, double z, double truncation = 0.05) :
        x_(x),
        y_(y),
        z_(z),
        truncation_(truncation)
    {
        set_num_residuals(1);
        mutable_parameter_block_sizes()->clear();
        for (auto const &i : num_ds) {
            mutable_parameter_block_sizes()->push_back(3); // 3 parameters for normal
            mutable_parameter_block_sizes()->push_back(i); // i parameters for number of distances
        }
    }
    
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {

        double res_min = 1./0.;
        int i_min = 0, j_min = 0;
        std::vector<int> j_min_vec;
        for (int i=0; i<num_ns; ++i) { // loop over all normal vectors
            int num_d = num_ds[i];
            double nTp = parameters[2*i][0] * x_ + parameters[2*i][1] * y_ + parameters[2*i][2] * z_;
            for (int j=0; j<num_d; ++j) { // loop over all distances with normal vector ni
                double res_ij = nTp + parameters[2*i+1][j];
                if (std::abs(res_ij) < std::abs(res_min)) {
                    res_min = res_ij;
                    i_min = i;
                    j_min = j;
                }
            }
            j_min_vec.push_back(j_min);
        }
        
        bool below_threshold = true;
        if (std::abs(res_min) < truncation_) {
            residuals[0] = res_min;
        } else {
            residuals[0] = truncation_;
            below_threshold = false;
        }
        
        if (jacobians) {
            // set whole Jacobian zero first
            for (int i=0; i<num_ns; ++i) {
                if (jacobians[2*i]) {
                    jacobians[2*i][0] = 0;
                    jacobians[2*i][1] = 0;
                    jacobians[2*i][2] = 0;
                }
                if (jacobians[2*i+1]) {
                    for (int j=0; j<num_ds[i]; ++j) {
                        jacobians[2*i+1][j] = 0;
                    }
                }
            }
            // set Jacobian for plane closest to point (x, y, z, 1)
            if (below_threshold && jacobians[2*i_min]) {
                jacobians[2*i_min][0] = x_;
                jacobians[2*i_min][1] = y_;
                jacobians[2*i_min][2] = z_;            
            }
            if (below_threshold && jacobians[2*i_min+1]) {
                jacobians[2*i_min+1][j_min_vec[i_min]] = 1.;
            }
        }
        
        return true;
    }
    
    static std::vector<int> num_ds; // number of ds per normal vector
    static int num_ns; // number of different normal vectors

};

#endif // PLANE_CONFIGURATION_COST_H_
