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

// includes
#include <iostream>
#include <fstream>
#include "definitions.h"
// libraries
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <sophus/so3.hpp>
#include <CLI/CLI.hpp>
// classes
#include "Timer.h"
#include "Plane.h"
#include "graph/PlaneGraph.h"
#include "graph/ParallelPlaneGraph.h"
#include "PPF/PairDetector.h"
#include "corners/Corner.h"
// function includes
#include "io/load_ply_cloud.h"
#include "visualize/pcshow.h"

/*
 * main
 */
int main(int argc, char** argv) {

    Timer T;
    
    // parse setttings from command line
    std::string ply_file;
    int min_votes = 5;
    double d_min = .1, d_max = 1.;
    int sampling = 50;

    CLI::App app{"Detect and refine orthogonal plane configurations"};
    app.add_option("--img", ply_file, "ply file path, from data/")->required();
    app.add_option("--min-votes", min_votes, "minimal number of votes to accept configuration");
    app.add_option("--dmin", d_min, "minimal distance between two points");
    app.add_option("--dmax", d_max, "maximal distance between two points");
    app.add_option("--sampling", sampling, "sampling of points");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }
    
    // load data
    std::string filepath = "../data/";
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;
    
    T.tic();
    read_ply_file(filepath+ply_file, points, normals);
    T.toc("read data");
    std::cout << points.size() << " points loaded." << std::endl;
    std::cout << normals.size() << " normals loaded." << std::endl;
    
    pcshow(points);
              
    // pairing
    ppf::PairDetector pairDet(d_max, d_min, min_votes);
    PlaneGraph planeMap(pairDet.para_thresh(), pairDet.distance_bin());
    T.tic();
    pairDet.detect_ortho_pairs(points, normals, planeMap);
    T.toc("time pairing PPF ");
    planeMap.print_info();
    
    std::cout << std::endl << "Thresholds:\tAngle:\t" << pairDet.para_thresh() << "\tDistance:\t" << pairDet.distance_bin() << std::endl << std::endl;

    // clustering & filtering
    T.tic();
    planeMap.cluster_graph_vertices();
    T.toc("clustering planes");
    planeMap.print_info();
    planeMap.print_parameters();
    planeMap.print_edges();
    
    // reduce graph to ParallelPlaneGraph
    T.tic();
    ParallelPlaneGraph redMap = planeMap.reduce_graph();
    T.toc("graph reduction");
    redMap.print_info();
    redMap.print_edges();
    T.tic();
    redMap.triangle_reduce();
    T.toc("triangle reduction");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();
    
    T.tic();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    T.toc("outlier filtering");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();
    
    // coarse refinement
    double lambda = .01 * points.size();
    T.tic();
    redMap.refine(points, normals, lambda, pairDet.distance_bin(), sampling);
    redMap.cluster_graph_vertices();
    T.toc("coarse refine and clustering");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();     

    T.tic();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    T.toc("outlier filtering");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();
      
    // find triangles
    std::vector<Graph<ParallelPlaneSet>::Triangle> triangles;
    T.tic();
    redMap.find_triangles(triangles);
    T.toc("find triangles in reduced graph");
    redMap.print_triangles(triangles);
    // if no triangles found: stop here
    if (triangles.size() < 1) {
        std::cout << "No corners found, return." << std::endl;
        return 0;
    }
    
    // neighborhood search: Kd-Tree    
    const PointCloudAdaptor pc2kd(points);
    KDTree index(3, pc2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10)); // arguments: dimensionality, point cloud adaptor, max leafs
    index.buildIndex();
    std::vector<Corner> corners;
    std::vector<size_t> indices;
    
    // find good corners
    T.tic();
    for (const auto& t : triangles) {
        Vec3 n1 = redMap.vertices()[t.first].n();
        Vec3 n2 = redMap.vertices()[t.second].n();
        Vec3 n3 = redMap.vertices()[t.third].n();
        for (const auto d1 : redMap.vertices()[t.first].ds()) for (const auto d2 : redMap.vertices()[t.second].ds()) for (const auto d3 : redMap.vertices()[t.third].ds()) {
            indices = Corner::eval_candidate(points, normals, n1, d1, n2, d2, n3, d3, index);
            std::cout << (-d1*n1 - d2*n2 - d3*n3).transpose() << std::endl;
            if (indices.size() > 0) {
                Corner c(n1, d1, n2, d2, n3, d3);
                c.set_indices(indices);
                if (c.refine(points)) { // only accept if refinement converges
                    corners.push_back(c);
                }
            }
        }
    }
    T.toc("evaluated corner candidates");
    
    for (const auto& c : corners) {
        std::cout << c << std::endl << std::endl;
    }
    
    pcshow_corners(points, corners);
    
}
