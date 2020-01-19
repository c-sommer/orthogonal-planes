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

#ifndef PARALLEL_PLANE_SET_H_
#define PARALLEL_PLANE_SET_H_

// includes
#include <iostream>
#include "definitions.h"

/**
  * class ParallelPlaneSet
  */
class ParallelPlaneSet {

// variables

    Vec3 n_;
    std::vector<double> ds_;
//    
//// friend classes
//    
//    friend class ParallelPlaneGraph;
    
// friend functions

    friend std::ostream& operator<<(std::ostream& os, const ParallelPlaneSet& pps);

public:

// constructors / destructors

    ParallelPlaneSet() {}
    
    ParallelPlaneSet(Vec3 n) :
        n_(n.normalized()),
        ds_(std::vector<double>())
    {}
    
    ParallelPlaneSet(double nx, double ny, double nz) :
        n_(Vec3(nx, ny, nz)),
        ds_(std::vector<double>())
    {
        n_.normalize();
    }
    
    ParallelPlaneSet(Vec3 n, std::vector<double> ds) :
        n_(n.normalized()),
        ds_(ds)
    {}
    
    ParallelPlaneSet(double nx, double ny, double nz, std::vector<double> ds) :
        n_(Vec3(nx, ny, nz)),
        ds_(ds)
    {
        n_.normalize();
    }
    
    ~ParallelPlaneSet() {}
    
// methods

    Vec3 n() const { return n_; }
    
    const std::vector<double>& ds() const { return ds_; }
    
    int num_d() const { return ds_.size(); }
    
    double* n_data() { return n_.data(); }
    
    double* ds_data() { return ds_.data(); }
   
    void set_n(Vec3 n) {
        n_ = n.normalized();
    }
    
    void set_n(double nx, double ny, double nz) {
        n_ = Vec3(nx, ny ,nz);
        n_.normalize();
    }
    
    void add_d(double d) {
        ds_.push_back(d);
    }
    
    void set_d(int i, double d) {
        ds_.at(i) = d;
    }
    
    void reset_ds(std::vector<double> ds) {
        ds_ = ds;
    }
    
    void normalize() {
        n_.normalize();
    }
    
    void sort_d() {
        std::sort(ds_.begin(), ds_.end());
    }
    
    void cluster_d(double dist_threshold = 0.05) {
        // important: this assumes sorted ds!! does not work for ds of random order
        size_t num_d = ds_.size();
        std::vector<size_t> d_sizes;
        std::vector<double> new_ds;
        double dist = 1./0.;
        d_sizes.push_back(1);
        new_ds.push_back(ds_[0]);
        for (int i=1; i<num_d; ++i) {
            if (ds_[i]-ds_[i-1] < dist_threshold) { // neighboring ds are below threshold
                new_ds.back() += ds_[i];
                ++d_sizes.back();
            } else { // neighboring ds are far enough apart
                new_ds.push_back(ds_[i]);
                d_sizes.push_back(1);
            }
        }
        ds_ = new_ds;
        num_d = ds_.size();
        for (int i=0; i<num_d; ++i) {
            ds_[i] /= d_sizes[i];
        }
    }

};

/**
 * ostream definition
 */
inline std::ostream& operator<<(std::ostream& os, const ParallelPlaneSet& pps) {

    os << "[" << pps.n_(0) << " " << pps.n_(1) << " " << pps.n_(2) << "]";  
    return os;
    
}

#endif // PARALLEL_PLANE_SET_H_
