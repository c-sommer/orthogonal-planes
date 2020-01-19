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

#ifndef PLANE_GRAPH_H_
#define PLANE_GRAPH_H_

// libraries
#include <opencv2/core/core.hpp>
#include <sophus/so3.hpp>
// classes
#include "Plane.h"
#include "Graph.h"
#include "ParallelPlaneGraph.h"

class PlaneGraph : public Graph<Plane> {

// variables

    const double normal_threshold_;
    const double dist_threshold_;

// member functions
    
    bool ptUsable(const Vec3 &pt, const Vec3 &normal) const {
        bool point_ok = std::isfinite(pt[0]) && std::isfinite(pt[1]) && std::isfinite(pt[2]);
        bool normal_ok = std::isfinite(normal[0]) && std::isfinite(normal[1]) && std::isfinite(normal[2]);
        return point_ok && normal_ok;
    }

    //! Discuss: when use static?
    static bool cmpPairSmaller(const std::pair<double, int> &lhs, const std::pair<double, int> &rhs){
      return lhs.first < rhs.first;
    }
    
    bool are_similar(int v1, int v2) {
        Plane& P1 = vertices_[v1];
        Plane& P2 = vertices_[v2];
        return P1.n().dot(P2.n()) > normal_threshold_ && std::abs(P1.dist(P2.p())) < dist_threshold_ && std::abs(P2.dist(P1.p())) < dist_threshold_;
    }

    bool get_line(Vec3 n1, double d1, Vec3 n2, double d2, const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, Vec6& line);

// friend functions

    friend void generate_points_normals(const PlaneGraph &G, int num_points, std::vector<Vec6> &points, double sigma, double outliers);

public:

// constructors and destructors

    PlaneGraph() :
        Graph(),
        normal_threshold_(std::cos(M_PI / 10.)),
        dist_threshold_(0.1)
    {
    }
    
    PlaneGraph(double normal_threshold, double dist_threshold) :
        Graph(),
        normal_threshold_(normal_threshold),
        dist_threshold_(dist_threshold)
    {}

//     ~PlaneGraph() {}

// getters

    std::vector<Plane> vertices() const {
        return vertices_;
    }

    std::set<Edge> edges() const {
        return edges_;
    }
   //TODO: ADD const? 
    size_t numVertices() {
        return num_vertices_;
    }
   
    //! Given a plane v_ref, and a list of planes vs, add edges between v_ref and each element in vs.
    //! Add v_ref and all elments in vs to PlaneGraph.
    //! Assume incrementing indexing to new vertices, and use the current number of vertices as the next index to assign.
    void insert_pairs(const Plane& v_ref, const std::vector<Plane>& vs) {
        int idx_ref = num_vertices_;
        insert_vertex(v_ref);
        int sz = vs.size();
        for (int k=0; k<sz; ++k) {
            insert_edge(idx_ref, num_vertices_);
            insert_vertex(vs[k]);
        }
    }

    //! Extract lines from point cloud with points: x,y,z, and normals: nx,ny,nz;
    //! The returned lines is a 6D vector, use a point and a line direction to represent.
    //! Provide opencv and eigen interfaces
    std::vector<Vec6> extract_lines(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals);

    void remove_vertex_without_edge(const std::vector<bool>& hasEdge);

    ParallelPlaneGraph reduce_graph();

    /*
     * Label point cloud to different plane segments by checking the point plane distance. A very rough technology, used just for filter out outliers or visualization.
     */
    void go_thr_pcl(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, std::vector<Eigen::Matrix<float, 6, 1>>& labeled);

    /*
     * if Label = num_vertices_ : this vertex is non-plane
     */
    cv::Mat label_pcl(const cv::Mat &scene, double normal_threshold, double dist_threshold);

    /*
     * From ParallelPlaneGraph, we can initialize a normal PlaneGraph contains only vertices, connect_orth_intersect_planes() needs to be called to complete the normal PlaneGraph.
     */
    static PlaneGraph from_ppg(ParallelPlaneGraph& PPG, double normal_threshold = std::cos(M_PI / 10), double dist_threshold = 0.1) {
        PlaneGraph PG(normal_threshold, dist_threshold);
        for (auto const &pps : PPG.vertices()) {
            Vec3 n = pps.n();
            for (auto const &d : pps.ds()) {
                PG.insert_vertex(Plane(n, d));
            }
        }
        return PG;
    }

    void cluster_graph_vertices();
    
    void print_parameters() {
        for (auto const &pl : vertices_) {
            std::cout << pl << std::endl;
        }
    }
    
    void print_vertex(int i) const {
        if (i<num_vertices_) {
            std::cout << vertices_[i] << std::endl;
        }
    }

    void print_thresh(std::string text){
        std::cout<<"["<<text<<"] normal thresh: "<<normal_threshold_<<", dist thresh: "<<dist_threshold_<<std::endl;
    }


    // Given a gt plane and a detected plane, compare their normal and d-component diviation with some threshold defined in constructors.
    bool are_similar(Plane gt, Plane detected){
        Vec3 n1 = gt.n();
        Vec3 n2 = detected.n();
        double d1 = gt.d();
        double d2 = detected.d();
        bool temp = (std::abs(d1 - d2) < dist_threshold_) ||(std::abs(d1 + d2) < dist_threshold_);
        return std::abs(n1.dot(n2)) > normal_threshold_ && temp;
    }

    // Compare planes similaity with specified thresholds from outside
    bool are_similar(Plane gt, Plane detected,double normal_thresh, double dist_thresh){
        Vec3 n1 = gt.n();
        Vec3 n2 = detected.n();
        double d1 = gt.d();
        double d2 = detected.d();

       bool temp = (std::abs(d1 - d2) < dist_thresh) ||(std::abs(d1 + d2) < dist_thresh);

        return std::abs(n1.dot(n2)) > normal_thresh && temp;
    }

};

#endif // PLANE_GRAPH_H_
