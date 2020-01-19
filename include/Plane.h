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


#ifndef PLANE_H_
#define PLANE_H_

// includes
#include <iostream>
#include <Eigen/Dense>

/**
  * class Plane
  */
class Plane {

    using Vec3 = Eigen::Matrix<double, 3, 1>;

// variables

    Vec3 n_;
    
    double d_;
    
    Vec3 p_;

public:

// constructors / destructors

    Plane() {}
    
    Plane(Vec3 n, double d) :
        n_(n.normalized()),
        d_(d),
        p_(-d_*n_)
    {}
    
    Plane(Vec3 n, Vec3 p) :
        n_(n.normalized()),
        d_(-n_.dot(p)),
        p_(p)
    {}
    
    Plane(Vec3 n, double d, Vec3 p) :
        n_(n.normalized()),
        d_(d)
    {
        p_ = p - (p.dot(n_)  + d_) * n_;
    }
    
    Plane(double nx, double ny, double nz, double d) :
        n_(Vec3(nx, ny, nz)),
        d_(d),
        p_(-d_*n_.normalized())
    {
        n_.normalize();
    }
    
    Plane(double nx, double ny, double nz, double px, double py, double pz) :
        n_(Vec3(nx, ny, nz)),
        d_(-n_.normalized().dot(Vec3(px, py, pz))),
        p_(Vec3(px, py, pz))
    {
        n_.normalize();
    }
    
    Plane(double nx, double ny, double nz, double d, double px, double py, double pz) :
        n_(Vec3(nx, ny, nz)),
        d_(d)
    {
        n_.normalize();
        Vec3 p(px, py, pz);
        p_ = p - (p.dot(n_)  + d_) * n_;
    }
    
    
    ~Plane() {}
    
// methods

    Vec3 n() const { return n_; }
    
    double d() const { return d_; }
    
    Vec3 p() const { return p_; }

    void set_n(Vec3 n) {
        n_ = n.normalized();
    }
    
    void set_n(double nx, double ny, double nz) {
        n_ = Vec3(nx, ny ,nz);
        n_.normalize();
    }
    
    void set_d(double d) { d_ = d; }
    
    void set_d(Vec3 p) { d_ = -n_.dot(p); }
    
    double dist(Vec3 p) const { return p.dot(n_) + d_; }
    
    Vec3 project(Vec3 p) const { return p - dist(p) * n_; }
    
// friend functions

    friend std::ostream& operator<<(std::ostream& os, const Plane& P);

};

/**
 * ostream definition
 */
inline std::ostream& operator<<(std::ostream& os, const Plane& P) {

    os << "[" << P.n_(0) << " " << P.n_(1) << " " << P.n_(2) << "," << P.d_ << "]";  
    return os;
    
}

#endif // PLANE_H_
