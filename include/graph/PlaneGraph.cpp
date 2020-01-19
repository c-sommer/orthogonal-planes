/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Yumin Sun, Christiane Sommer.
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

#include "PlaneGraph.h"
// libraries
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * helper struct for cluster average computation
 */
struct ClusterAvg {
    Mat3 C = Mat3::Zero();
    Vec3 pm = Vec3::Zero();
    Vec3 nm = Vec3::Zero();
    int sum = 0;
    ClusterAvg() {}
    void update(Vec3 n, Vec3 p) {
        C += p * p.transpose();
        pm += p;
        nm += n;
        ++sum;
    }
};

/*
 * clustering for the particular case of a PlaneGraph: averaging of respective planes
 */
void PlaneGraph::cluster_graph_vertices() { // TODO: maybe reduce nr of loops
    UnionFind UF;
    UF.INIT(num_vertices_);
    for (size_t i1 = 0; i1 < num_vertices_; ++i1) {
        for (size_t i2 = i1+1; i2 < num_vertices_; ++i2) {
            if(UF.FIND_SET(i1) == UF.FIND_SET(i2)) // avoid multiple clustering to speed up things!
                continue;
            if (are_similar(i1, i2)) {
                UF.UNION(i1, i2);
//                std::cout << "Cluster planes" << std::endl
//                          << "\t" << vertices_[i1] << "\t[" << vertices_[i1].p().transpose() << "]" << std::endl
//                          << "\t" << vertices_[i2] << "\t[" << vertices_[i2].p().transpose() << "]" << std::endl;
            }
        }
    }
    
    // TODO: std::vector of struct containing N, num, D, push back to that one
    std::vector<int> new_index(num_vertices_, num_vertices_);
    size_t counter = 0;
    std::vector<Plane> new_vertices;
    std::vector<ClusterAvg> avg_vec;
    for (size_t i=0; i < num_vertices_; ++i) {
        if (UF.parents[i] == i) {
            new_index[i] = counter;
            avg_vec.push_back(ClusterAvg());
//            new_vertices.push_back(vertices_[i]);
            ++counter;
        }
        else {
            new_index[i] = new_index[UF.FIND_SET(i)];
        }
        avg_vec[new_index[i]].update(vertices_[i].n(), vertices_[i].p());
    }
    
    // compute cluster average based on representative points
    // TODO: check!
    for (int i=0; i<avg_vec.size(); ++i) {
        if (avg_vec[i].sum < 2) {
            new_vertices.push_back(Plane(avg_vec[i].nm, -avg_vec[i].nm.dot(avg_vec[i].pm), avg_vec[i].pm));
        } else {
            Vec3 p = avg_vec[i].pm / avg_vec[i].sum; // mean representative point
            Eigen::SelfAdjointEigenSolver<Mat3> eig(avg_vec[i].C - avg_vec[i].sum * p * p.transpose());
            Vec3 l = eig.eigenvectors().col(2);
            Vec3 n = avg_vec[i].nm;
            n -= l.dot(n) * l;
            n.normalize();
            new_vertices.push_back(Plane(n, -n.dot(p), p));
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
    reduced_ = true;
}

/*
 * graph reduction from vertices (n,d) to vertices (n, list of ds)
 */
ParallelPlaneGraph PlaneGraph::reduce_graph() {

    UnionFind UF;
    UF.INIT(num_vertices_);
    for (size_t i1 = 0; i1 < num_vertices_; ++i1) {
        for (size_t i2 = i1+1; i2 < num_vertices_; ++i2) {
            Vec3 n1 = vertices_[UF.FIND_SET(i1)].n(); // could also just take i1 instead of parent[i1]
            Vec3 n2 = vertices_[i2].n();
            if (n1.dot(n2) > normal_threshold_ || n1.dot(n2) < -normal_threshold_) { // parallel or anti-parallel
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
            ParallelPlaneSet pps(vertices_[i].n());
            pps.add_d(vertices_[i].d());
            new_vertices.push_back(pps);
            ++counter;
        }
        else {
            uint16_t rep = UF.FIND_SET(i);
            new_index[i] = new_index[rep];
            Vec3 n_old = vertices_[rep].n();
            Vec3 n_new = vertices_[i].n();
            new_vertices[new_index[i]].add_d(-n_old.dot(vertices_[i].p()));
        }
    }
    
    std::set<Edge> new_edges;
    for (auto const &e : edges_) {
        int i1 = new_index[e.first];
        int i2 = new_index[e.second];
        if (i1<i2) new_edges.emplace(i1, i2);
        else if (i2<i1) new_edges.emplace(i2, i1);
    }

    return ParallelPlaneGraph(new_vertices, new_edges, dist_threshold_, normal_threshold_);

}

/*
 * determine line of intersection in data from unstructured ply data
 */
bool PlaneGraph::get_line(Vec3 n1, double d1, Vec3 n2, double d2, const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, Vec6& line) {
    
    double dist_threshold_sq = dist_threshold_ * dist_threshold_;
    double prefactor = 1. / (1 - n1.dot(n2) * n1.dot(n2));
    Vec3 l = n1.cross(n2).normalized();
    Vec3 p = (-d1*n1 - d2*n2 + n1.dot(n2) * (d2*n1 + d1*n2)) * prefactor;

    std::vector<double> hs1, hs2;
    for (int i=0; i<points.size(); i+=5) {
        Vec3 pi = points[i].cast<double>();
        Vec3 ni = normals[i].cast<double>();
        double dist1 = n1.dot(pi) + d1;
        double dist2 = n2.dot(pi) + d2;
        double dist_sq = dist1 * dist1 + dist2 * dist2;
        if (dist_sq < dist_threshold_sq) {
            if (std::abs(n1.dot(ni)) > normal_threshold_)
                hs1.push_back(l.dot(pi));
            if (std::abs(n2.dot(ni)) > normal_threshold_)
                hs2.push_back(l.dot(pi));
        }
    }
    if (hs1.size() < 1 || hs2.size() < 1) {
        return false;
    }
    std::sort(hs1.begin(), hs1.end());
    std::sort(hs2.begin(), hs2.end());
    double hmin = std::max(hs1[0], hs2[0]);
    double hmax = std::min(hs1[hs1.size()-1], hs2[hs2.size()-1]);
    // TODO: replace hs[0] to hs[end] by clustered
    line.segment<3>(0) = p + hmin * l;
    line.segment<3>(3) = p + hmax * l;
    return true;     
}


/*
 * extract lines of intersection of orthogonal plane pairs (ply unstructured data)
 */
std::vector<Vec6> PlaneGraph::extract_lines(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals) {

    std::vector<Vec6> lines;
    Vec6 line;
    
    if (edges_.size() > 0) {
        for (const auto& e : edges_) {
            Vec3 n1 = vertices_[e.first].n();
            double d1 = vertices_[e.first].d();
            Vec3 n2 = vertices_[e.second].n();
            double d2 = vertices_[e.second].d();
            
            if (get_line(n1, d1, n2, d2, points, normals, line)) {
                lines.push_back(line);
            }
        }
    } else {
        double ortho_threshold = std::sqrt(1 - normal_threshold_ * normal_threshold_);
        for (const auto& pl1 : vertices_) {
            Vec3 n1 = pl1.n();
            double d1 = pl1.d();
            for (const auto& pl2 : vertices_) {
                Vec3 n2 = pl2.n();
                if (std::abs(n1.dot(n2)) < ortho_threshold) {
                    double d2 = pl2.d();
                    if (get_line(n1, d1, n2, d2, points, normals, line)) {
                        lines.push_back(line);
                    }
                }
            }
        }
    }

    return lines;
}


/*
 * helper for sorting without using complicated data structures
 * https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
 */
template <typename T>
std::vector<size_t> sort_indices(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

void PlaneGraph::go_thr_pcl(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, std::vector<Eigen::Matrix<float, 6, 1>>& labeled) {

    size_t counter = 0;        // point on closest plane
    size_t counter_line = 0;   // intersection on line
    size_t counter_corner = 0; // intersection on corner
    
    const size_t num_points = points.size();
    labeled = std::vector<Eigen::Matrix<float, 6, 1>>(num_points);

    for (size_t k = 0; k < num_points; ++k) {

        //! For each point in the scene
        Vec3 p = points[k].cast<double>();
        Vec3 n = normals[k].cast<double>();

        std::vector<double> distances;
        for (auto const &pl : vertices_) {
            distances.push_back(std::abs(pl.dist(p)));
        }
        std::vector<size_t> idx = sort_indices<double>(distances);
        bool plane = false;
        double inv_threshold = 1. / dist_threshold_;
        if (distances[idx[0]] < dist_threshold_) {
            Vec3 n_plane = vertices_[idx[0]].n();
            if (std::abs(n.dot(n_plane)) > normal_threshold_) {
                // accept point as belonging to closest plane
                double t = 1. - distances[idx[0]] * inv_threshold; // 0 <= t <= 1 TODO: make more efficient
                labeled[k].segment<3>(0) = points[k];
                labeled[k].segment<3>(3) = t * n_plane.cast<float>();
                plane = true;
                ++counter;
            }
            else if (distances[idx[1]] < dist_threshold_) {
                n_plane = vertices_[idx[1]].n();
                if (std::abs(n.dot(n_plane)) > normal_threshold_) {
                    // accept point as belonging to second closest plane
                    double t = 1. - distances[idx[1]] * inv_threshold; // 0 <= t <= 1 TODO: make more efficient
                    labeled[k].segment<3>(0) = points[k];
                    labeled[k].segment<3>(3) = t * n_plane.cast<float>();
                    plane = true;
                    ++counter_line;
                }
                else if (distances[idx[2]] < dist_threshold_ && std::abs(n.dot(vertices_[idx[2]].n())) > normal_threshold_) {
                    n_plane = vertices_[idx[2]].n();
                    // accept point as belonging to third closest plane
                    double t = 1. - distances[idx[2]] * inv_threshold; // 0 <= t <= 1 TODO: make more efficient
                    labeled[k].segment<3>(0) = points[k];
                    labeled[k].segment<3>(3) = t * n_plane.cast<float>();
                    plane = true;
                    ++counter_corner;
                }
            }
        }
        if (!plane) {
            // point is not part of any (meaningful) plane
            labeled[k].segment<3>(0) = points[k];
            labeled[k].segment<3>(3) = Eigen::Vector3f::Zero();
        }
    }        //! for each point

    std::cout << counter << "\tpoints on closest plane," << std::endl
    << counter_line << "\tpoints on line," << std::endl
    << counter_corner << "\tpoints on corner." << std::endl;

}

void PlaneGraph::remove_vertex_without_edge(const std::vector<bool>& hasEdge){
    std::vector<int> new_index(num_vertices_, num_vertices_);
    size_t counter = 0;
    std::vector<Plane> new_vertices;
    for(int i = 0 ; i < num_vertices_; ++i) {
        if (hasEdge[i]){
            new_index[i] = counter;
            new_vertices.push_back(vertices_[i]);
            ++counter;
        }
    }
    vertices_ = new_vertices;
    num_vertices_ = vertices_.size();

    std::set<Edge> new_edges;
    for (auto const &e : edges_) {
        int i1 = new_index[e.first];
        int i2 = new_index[e.second];
        if (i1 < num_vertices_ && i2 < num_vertices_) {
            new_edges.emplace(i1, i2);
        }
    }

    edges_ = new_edges;
    num_edges_ = edges_.size();

}
