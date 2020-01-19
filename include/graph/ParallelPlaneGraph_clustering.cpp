/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019 Yumin Sun, Christiane Sommer.
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
 * custom graph vertex clustering, differs from original Graph<Vertex> one in the way vertices are merged
 */
void ParallelPlaneGraph::cluster_graph_vertices() { // TODO: maybe reduce nr of loops

    UnionFind UF;
    UF.INIT(num_vertices_);
    for (size_t i1 = 0; i1 < num_vertices_; ++i1) {
        for (size_t i2 = i1+1; i2 < num_vertices_; ++i2) {
            if (std::abs(vertices_[i1].n().dot(vertices_[i2].n())) > para_threshold_) {
                UF.UNION(i1, i2);
            }
        }
    }
    
    std::vector<int> new_index(num_vertices_, num_vertices_);
    size_t counter = 0;
    std::vector<ParallelPlaneSet> new_vertices;
    for (size_t i=0; i < num_vertices_; ++i) {
        if (UF.parents[i] == i) {
            new_index[i] = counter;
            new_vertices.push_back(vertices_[i]);
            ++counter;
        }
        else {
            size_t rep = UF.FIND_SET(i);
            new_index[i] = new_index[rep];
            for (const auto& d : vertices_[i].ds())
                new_vertices[new_index[rep]].add_d(d);
        }
    }
    
    vertices_ = new_vertices;
    num_vertices_ = vertices_.size();
    
    sort_ds();
    for (auto& pps : vertices_) {
        pps.cluster_d(dist_threshold_);
    }
    
    std::set<Edge> new_edges;
    for (auto const &e : edges_) {
        int i1 = new_index[e.first];
        int i2 = new_index[e.second];
        if (i1<i2) new_edges.emplace(i1, i2);
        else if (i2<i1) new_edges.emplace(i2, i1);
    }
    edges_ = new_edges;
    num_edges_ = edges_.size();
    reduced_ = true;
}

/*
 * graph reduction from triangles: if two triangles in the graph are adjacent, i.e. share an edge, they are collapsed to one
 */
void ParallelPlaneGraph::triangle_reduce() {

    std::vector<Triangle> triangles;
    find_triangles(triangles);

    UnionFind UF;
    UF.INIT(num_vertices_);
    for (size_t i1 = 0; i1 < triangles.size(); ++i1) {
        for (size_t i2 = i1+1; i2 < triangles.size(); ++i2) {
            std::pair<int, int> ij = triangles[i1].is_adjacent(triangles[i2]);
            if (ij.first < ij.second) UF.UNION(ij.first, ij.second);
            else if (ij.second < ij.first) UF.UNION(ij.second, ij.first);
        }
    }
    
    std::vector<int> new_index(num_vertices_, num_vertices_);
    size_t counter = 0;
    std::vector<ParallelPlaneSet> new_vertices;
    for (size_t i=0; i < num_vertices_; ++i) {
        if (UF.parents[i] == i) {
            new_index[i] = counter;
            new_vertices.push_back(vertices_[i]);
            ++counter;
        }
        else {
            uint16_t rep = UF.FIND_SET(i);
            new_index[i] = new_index[rep];
            Vec3 n_old = vertices_[rep].n();
            Vec3 n_new = vertices_[i].n();
            if (n_old.dot(n_new) > 0) {
                for (auto const& d : vertices_[i].ds())
                    new_vertices[new_index[i]].add_d(d);
            } else {
                for (auto const& d : vertices_[i].ds())
                    new_vertices[new_index[i]].add_d(-d);
            }
        }
    }
    
    vertices_ = new_vertices;
    num_vertices_ = vertices_.size();
    
    std::set<Edge> new_edges;
    for (auto const &e : edges_) {
        int i1 = new_index[e.first];
        int i2 = new_index[e.second];
        if (i1<i2) new_edges.emplace(i1, i2);
        else if (i2<i1) new_edges.emplace(i2, i1);
    }
    
    edges_ = new_edges;
    num_edges_ = edges_.size();
    
    sort_ds();

}
