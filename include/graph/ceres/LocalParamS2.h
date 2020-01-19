/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2018, Christiane Sommer.
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

#ifndef LOCAL_PARAM_S2_H_
#define LOCAL_PARAM_S2_H_

#include <ceres/local_parameterization.h>

namespace ceres {

class LocalParamS2 : public ceres::LocalParameterization {

public:

    LocalParamS2() {}
    
    virtual ~LocalParamS2() {}

    // S2 plus operation for Ceres
    //
    virtual bool Plus(double const* n, double const* delta,
                      double* n_plus_delta) const {
        
        const double norm_inv = 1. / std::sqrt(1. + delta[0]*delta[0] + delta[1]*delta[1]);
        
        if (n[2] > 0.) {
            const double nxdelta_nz1 = (n[1] * delta[0] - n[0] * delta[1]) / (1 + n[2]);
            n_plus_delta[0] = ( nxdelta_nz1 * n[1] + n[2] * delta[0] + n[0]) * norm_inv;
            n_plus_delta[1] = (-nxdelta_nz1 * n[0] + n[2] * delta[1] + n[1]) * norm_inv;
            n_plus_delta[2] = (n[2] - n[0] * delta[0] - n[1] * delta[1]) * norm_inv;
        }
        else {
            const double nxdelta_nz1 = (n[1] * delta[0] - n[0] * delta[1]) / (1. - n[2]);
            n_plus_delta[0] = (-nxdelta_nz1 * n[1] + n[2] * delta[0] + n[0]) * norm_inv;
            n_plus_delta[1] = ( nxdelta_nz1 * n[0] + n[2] * delta[1] + n[1]) * norm_inv;
            n_plus_delta[2] = (n[2] - n[0] * delta[0] - n[1] * delta[1]) * norm_inv;
        }

        return true;

    }

    // Jacobian of S2 plus operation for Ceres
    //
    virtual bool ComputeJacobian(double const* n,
                                 double* J) const {
        
        if (n[2] > 0.) {
            const double nz1_inv = 1. / (1 + n[2]);
            // Jacobian has size 3x2
            J[0] =  n[1] * n[1] * nz1_inv + n[2];
            J[1] = -n[0] * n[1] * nz1_inv;
            J[2] =  J[1];
            J[3] =  n[0] * n[0] * nz1_inv + n[2];
            J[4] = -n[0];
            J[5] = -n[1];
        }
        else {
            const double nz1_inv = 1. / (1 - n[2]);
            // Jacobian has size 3x2
            J[0] = -n[1] * n[1] * nz1_inv + n[2];
            J[1] =  n[0] * n[1] * nz1_inv;
            J[2] =  J[1];
            J[3] = -n[0] * n[0] * nz1_inv + n[2];
            J[4] = -n[0];
            J[5] = -n[1];
        }
        
        return true;

    }

    virtual int GlobalSize() const { return 3; }

    virtual int LocalSize() const { return 2; }
};

} // namespace ceres

#endif // LOCAL_PARAM_S2_H_
