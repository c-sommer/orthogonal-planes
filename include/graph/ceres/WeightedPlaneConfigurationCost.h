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

#ifndef WEIGHTED_PLANE_CONFIGURATION_COST_H_
#define WEIGHTED_PLANE_CONFIGURATION_COST_H_

//libraries
#include <ceres/ceres.h>

/*
 * cost function containing weighted data term residual (minimal distance from any of the planes) and its derivatives
 * this cost function also uses lin-lin search instead of multiplied search
 */
class WeightedPlaneConfigurationCost : public ceres::CostFunction {

    double x_, y_, z_;
    double nx_, ny_, nz_;
    double truncation_;
    static constexpr double para_thresh_ = std::cos(M_PI * 20/180);

public:

    WeightedPlaneConfigurationCost(double x, double y, double z, double nx, double ny, double nz, double truncation = 0.05) :
        x_(x),
        y_(y),
        z_(z),
        nx_(nx),
        ny_(ny),
        nz_(nz),
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

        int i_min = 0, j_min = 0;
        
        double res_min, res_min_abs = truncation_;;
        for (int i=0; i<num_ns; ++i) { // loop over all normal vectors
            double nTn = parameters[2*i][0] * nx_ + parameters[2*i][1] * ny_ + parameters[2*i][2] * nz_;
            if (std::abs(nTn) > para_thresh_) { // exclude points with wrong normals early, to not have to search whole space of planes
                int num_d = num_ds[i];
                double nTp = parameters[2*i][0] * x_ + parameters[2*i][1] * y_ + parameters[2*i][2] * z_;
                double res_old, res_new, res_old_abs, res_new_abs = 1./0.;
                int j = 0;
                bool old_larger;
                do { // for sorted d, only search until residual becomes larger again
                    res_old = res_new;
                    res_old_abs = res_new_abs;
                    res_new = nTp + parameters[2*i+1][j++];
                    res_new_abs = std::abs(res_new);
                    old_larger = res_old_abs > res_new_abs;
                } while (old_larger && j < num_d);
                if (res_old_abs < res_min_abs || res_new_abs < res_min_abs) { // check if smallest residual < res_min
                    i_min = i;
                    if (old_larger) {
                        res_min = res_new;
                        res_min_abs = res_new_abs;
                        j_min = j-1; // j-1 because j was already incremented in j++
                    } else {
                        res_min = res_old;
                        res_min_abs = res_old_abs;
                        j_min = j-2; // j-2 because j was already incremented in j++ and res_old
                    }
                }
            }
        }
        
        bool below_threshold = true;
        if (res_min_abs < truncation_) {
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
                jacobians[2*i_min+1][j_min] = 1.;
            }
        }
        
        return true;
    }
    
    static std::vector<int> num_ds; // number of ds per normal vector
    static int num_ns; // number of different normal vectors

};

#endif // WEIGHTED_PLANE_CONFIGURATION_COST_H_
