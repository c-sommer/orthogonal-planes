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

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "LocalParamSO3.h"

/*
 * CornerError
 */
class CornerError: public ceres::SizedCostFunction<1, 3, 4> { // sizes: <residuals, parameters[0], parameters[1]>

    using Vec3 = Eigen::Vector3d;

    Vec3  p_;

public:
    
    CornerError(double x, double y, double z) : p_(Vec3(x, y, z)) { }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        
        Eigen::Map<const Sophus::SO3d> rot(parameters[1]);
        Vec3 Rp = rot * p_;
        Vec3 q = Rp + Vec3(parameters[0][0], parameters[0][1], parameters[0][2]);
        
        int i_min = 0;
        if (std::abs(q[1]) < std::abs(q[0]))     i_min = 1;
        if (std::abs(q[2]) < std::abs(q[i_min])) i_min = 2;
        
        residuals[0] = q[i_min];
 
        if (jacobians!=nullptr && jacobians[0]!=nullptr) {       
            switch(i_min) {
                case 0: { // (e_x, Rp x e_x)
                    jacobians[0][0] = 1;
                    jacobians[0][1] = 0;
                    jacobians[0][2] = 0;
                    jacobians[1][0] = 0;
                    jacobians[1][1] = Rp[2];
                    jacobians[1][2] = -Rp[1];
                } break;
                case 1: { // (e_y, Rp x e_y)
                    jacobians[0][0] = 0;
                    jacobians[0][1] = 1;
                    jacobians[0][2] = 0;
                    jacobians[1][0] = -Rp[2];
                    jacobians[1][1] = 0;
                    jacobians[1][2] = Rp[0];
                } break;
                case 2: { // (e_z, Rp x e_z)
                    jacobians[0][0] = 0;
                    jacobians[0][1] = 0;
                    jacobians[0][2] = 1;
                    jacobians[1][0] = Rp[1];
                    jacobians[1][1] = -Rp[0];
                    jacobians[1][2] = 0;
                } break;
            }
            jacobians[1][3] = 0; // dimensionality
        }

        return true;
    }

};
