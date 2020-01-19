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

/*
 * outlier filtering with normals and removal of planes with too few inliers - ply data
 */
void ParallelPlaneGraph::filter_outliers(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double truncation, int sampling) {

    // create counter for inliers
    std::vector<std::vector<size_t>> inliers;
    for (const auto& pps : vertices_) {
        inliers.push_back(std::vector<size_t>(pps.num_d(), 0));
    }
    
    const double para_thresh = std::cos(M_PI * 20/180);
    const int num_points = points.size();
    
    for (int k=0; k<num_points; k+=sampling) {
        
        int i_min = 0, j_min = 0;
        
        double res_min, res_min_abs = truncation;
        for (int i=0; i<num_vertices_; ++i) { // loop over all normal vectors
            double nTn = vertices_[i].n()[0] * normals[k][0] + vertices_[i].n()[1] * normals[k][1] + vertices_[i].n()[2] * normals[k][2];
            if (std::abs(nTn) < para_thresh) {
                continue;
            }
            double nTp = vertices_[i].n()[0] * points[k][0] + vertices_[i].n()[1] * points[k][1] + vertices_[i].n()[2] * points[k][2];
            double res_old, res_new, res_old_abs, res_new_abs = 1./0.;
            int j = 0;
            bool old_larger;
            int num_d = vertices_[i].num_d();
            do { // for sorted d, only search until residual becomes larger again
                res_old = res_new;
                res_old_abs = res_new_abs;
                res_new = nTp + vertices_[i].ds()[j++];
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
        if (res_min_abs < truncation) {
            ++inliers[i_min][j_min];
        }
    }
    
    std::vector<ParallelPlaneSet> new_vertices;
    std::vector<int> new_index(num_vertices_, num_vertices_);
    int counter = 0;
    for (int i=0; i<num_vertices_; ++i) {
        int num_d = vertices_[i].num_d();
        std::vector<double> ds;
        for (int j=0; j<num_d; ++j) {
            if (inliers[i][j]>0) {
                ds.push_back(vertices_[i].ds()[j]);
            }
        }
        if (ds.size()>0) {
            ParallelPlaneSet pps(vertices_[i].n(), ds);
            new_vertices.push_back(pps);
            new_index[i] = counter;
            ++counter;
        }
    }
    
    vertices_ = new_vertices;
    num_vertices_ = vertices_.size();
    
    std::set<Edge> new_edges;
    for (auto const &e : edges_) {
        int i1 = new_index[e.first];
        int i2 = new_index[e.second];
        if (i1<i2 && i2<num_vertices_) new_edges.emplace(i1, i2);
        else if (i2<i1 && i1<num_vertices_) new_edges.emplace(i2, i1);
    }
    
    edges_ = new_edges;
    num_edges_ = edges_.size();
}
