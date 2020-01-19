/**
Copyright 2011-2017 Hauke Strasdat           
          2012-2017 Steven Lovegrove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*/

#ifndef SOPHUS_LOCAL_PARAM_SO3_H_
#define SOPHUS_LOCAL_PARAM_SO3_H_

#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>

namespace Sophus {

class LocalParameterizationSO3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSO3() {}

  // SO3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    
    Eigen::Map<SO3d const> const T(T_raw);
    Eigen::Map<Vector3d const> const delta(delta_raw);
    Eigen::Map<SO3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = SO3d::exp(delta) * T; // TODO: document usage
    
    return true;
    
  }

  // Jacobian of SO3 plus operation for Ceres
  //
  // not real Jacobian w.r.t. quaternions, implement Jacobian w.r.t. xi
  // for each cost function separately!
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
                               
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobian(jacobian_raw);
    jacobian.setZero();
    jacobian.block<3,3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    
    return true;
    
  }

  virtual int GlobalSize() const { return 4; }

  virtual int LocalSize() const { return 3; }
};

}  // namespace Sophus

#endif // SOPHUS_LOCAL_PARAM_SO3_H_
