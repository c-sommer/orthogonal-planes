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
#include <CLI/CLI.hpp>
// classes
#include "Timer.h"
#include "Plane.h"
#include "graph/PlaneGraph.h"
#include "graph/ParallelPlaneGraph.h"
#include "PPF/PairDetector.h"
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
    
    // find triangles
    std::vector<Graph<Plane>::Triangle> triangles;
    T.tic();
    planeMap.find_triangles(triangles);
    T.toc("Finding triangles");
    planeMap.print_triangles(triangles);
    
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
 
    // CERES problem setup and solving
    double lambda = 0.01 * points.size();
    T.tic();
    redMap.refine_coarse_fine(points, normals, lambda, pairDet.distance_bin(), sampling);
    T.toc("Multi-plane parameter optimization");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();
    
    T.tic();
    redMap.filter_outliers(points, normals, pairDet.distance_bin(), .5*sampling);
    T.toc("outlier filtering");
    redMap.print_info();
    redMap.print_parameters();
    redMap.print_edges();
    
    // From here on visualization / debugging

//    // get points located on planes and not-planes
    PlaneGraph finalMap = PlaneGraph::from_ppg(redMap, 2*pairDet.para_thresh()*pairDet.para_thresh()-1, pairDet.distance_bin()); // double angle
    std::vector<Eigen::Matrix<float, 6, 1>> labeled;
    T.tic();
    finalMap.go_thr_pcl(points, normals, labeled);
    T.toc("labeling planes");

    // visualize
    pcshow(labeled, "Result");
    
//    std::vector<Vec6> lines;
//    // line extraction and visualization
//    T.tic();
//    lines = finalMap.extract_lines(points, normals);
//    T.toc("get lines");
//    std::cout << lines.size() << " lines found." << std::endl;
//    
//    pcshow_lines(points, lines, "Orth Plane Intersections");
    
}
