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

#ifndef PARALLEL_PLANE_GRAPH_H_
#define PARALLEL_PLANE_GRAPH_H_

// classes
#include "ParallelPlaneSet.h"
#include "Graph.h"

/*
 * ParallelPlaneGraph
 */
class ParallelPlaneGraph : public Graph<ParallelPlaneSet> {

    double dist_threshold_;
    double para_threshold_;

public:
    
// constructors

    ParallelPlaneGraph() :
        Graph<ParallelPlaneSet>()
    {}
    
    ParallelPlaneGraph(std::vector<ParallelPlaneSet> vertices, std::set<Edge> edges) :
        Graph<ParallelPlaneSet>(vertices, edges)
    {
        sort_ds();
    }

    ParallelPlaneGraph(std::vector<ParallelPlaneSet> vertices, std::set<Edge> edges, double dist_threshold, double para_threshold) :
        Graph<ParallelPlaneSet>(vertices, edges),
        dist_threshold_(dist_threshold),
        para_threshold_(para_threshold)
    {
        sort_ds();
    }
    
// member functions
    
    // use carefully! TODO: remove once no longer needed
    const std::vector<ParallelPlaneSet>& vertices() const {
        return vertices_;
    }

    // use carefully! TODO: remove once no longer needed
    const std::set<Edge>& edges() const {
        return edges_;
    }
    
    void set_edges(std::set<Edge> edges) {
        edges_ = edges;
    }
       
    double* get_ni(int i) {
        return vertices_[i].n_data();
    }

    void add_parameter_blocks(std::vector<double*> &parameter_blocks, int &num_ns, std::vector<int> &num_ds) {
        parameter_blocks.clear();
        num_ns = num_vertices_;
        num_ds.clear();
        for (auto &pps : vertices_) {
            parameter_blocks.push_back(pps.n_data());
            parameter_blocks.push_back(pps.ds_data());
            num_ds.push_back(pps.num_d());
        }
    }
    
    void renormalize() {
        for (auto &pps : vertices_) {
            pps.normalize();
        }
    }
    
    void sort_ds() {
        for (auto &pps : vertices_) {
            pps.sort_d();
        }
    }
    
    void print_parameters() {
        for (auto &pps : vertices_) {
            std::cout << "\t[" << pps.n().transpose() << "],\t[ ";
            for (auto &d : pps.ds()) {
                std::cout << d << " ";
            }
            std::cout << "]" << std::endl;
        }
    }
    
    // filtering
    void filter_outliers(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double truncation, int sampling);
    
    // clustering
    void cluster_graph_vertices();    
    void triangle_reduce();
    
    // optimization
    void refine(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double lambda, double truncation, int sampling);
    void refine_coarse_fine(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double lambda, double truncation, int sampling) {
        // coarse
        refine(points, normals, lambda, truncation, sampling);
        cluster_graph_vertices();
        // fine
        refine(points, normals, lambda, truncation, 25);   
    }

};

#endif // PARALLEL_PLANE_GRAPH_H_
