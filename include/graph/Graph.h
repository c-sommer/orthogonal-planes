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

#ifndef GRAPH_H_
#define GRAPH_H_

// includes
#include <iostream>
#include <fstream>  // write
#include <stdlib.h> // system(cmd)
#include <set>
#include <vector>
#include <map>
#include "UnionFind/UnionFindForestIndexAndPathHalving.h"

template <class Vertex>
class Graph {

protected:

// variables

    std::set<std::pair<int, int>> edges_;
    std::vector<Vertex> vertices_;

    size_t num_vertices_;
    size_t num_edges_;

    bool reduced_;
    
// member function

    virtual bool are_similar(int v1, int v2) { // TODO: static would be better, but what about virtual then?
        return false;
    }

public:

    using Edge = std::pair<int, int>;

// constructors and destructors

    Graph() :
        vertices_(std::vector<Vertex>()),
        edges_(std::set<Edge>()),
        num_vertices_(0),
        num_edges_(0),
        reduced_(false)
    {}

    //! Intialialize with label number, replace vertex later
    Graph(size_t num_vertices) :
            vertices_(std::vector<Vertex>(num_vertices)),
            edges_(std::set<Edge>()),
            num_vertices_(num_vertices),
            num_edges_(0),
            reduced_(false)
    {}
    
    Graph(std::vector<Vertex> vertices, std::set<Edge> edges) :
        vertices_(vertices),
        edges_(edges),
        num_vertices_(vertices.size()),
        num_edges_(edges.size()),
        reduced_(false)
    {}
    
    virtual ~Graph() {} // use virtual to avoid memory leaks

// structs

    struct Triangle {
        int first, second, third;
        
        Triangle(int i1, int i2, int i3) :
            first(i1),
            second(i2),
            third(i3)
        {}
        
        std::pair<int, int> is_adjacent(Triangle& other) {
            // Note: this implementation only works for other > this!
            using int_pair = std::pair<int, int>;
            if (first == other.first) {
                if (second == other.second) {
                    return int_pair(third, other.third);
                } else if (third  == other.second) {
                    return int_pair(second, other.third);
                } else if (third  == other.third) {
                    return int_pair(second, other.second);
                }
            } else if (second == other.first) {
                if (third == other.second) {
                    return int_pair(first, other.third);
                } else if (third == other.third) {
                    return int_pair(first, other.second);
                }
            } else if (second == other.second && third == other.third) {
                return int_pair(first, other.first);
            }
            return int_pair(0, 0);
        }
    };
    
// member functions

// basic insertion (deletion is not allowed!)

    // Use with care! This is only for match GT, or build a GT graph
    virtual void replace(const Vertex& v, int idx) {
        vertices_[idx] = v;
    }

    virtual void insert_vertex(const Vertex& v) {
        vertices_.push_back(v);
        num_vertices_++;
    }
    
    virtual void insert_vertices(std::vector<Vertex> vs) {
        int sz = vs.size();
        for (int k=0; k<sz; ++k) {
            vertices_.push_back(vs[k]);
        }
        num_vertices_ += sz;
    }

    virtual void insert_edge(int i1, int i2) {
        (i1<i2) ? edges_.emplace(i1, i2) : edges_.emplace(i2, i1);
        num_edges_ = edges_.size();
    }
    
    virtual void insert_edges(std::vector<Edge> es) {
        edges_.insert(es.begin(), es.end());
        num_edges_ = edges_.size();
    }
    
    virtual void insert_pair(const Vertex& v1, const Vertex& v2) {
        vertices_.push_back(v1);
        vertices_.push_back(v2);
        insert_edge(num_vertices_, num_vertices_+1);
        ++(++num_vertices_);
    }
    
    virtual void cluster_graph_vertices() { // TODO: maybe reduce nr of loops
        UnionFind UF;
        UF.INIT(num_vertices_);
        for (size_t i1 = 0; i1 < num_vertices_; ++i1) {
            for (size_t i2 = i1+1; i2 < num_vertices_; ++i2) {
//                if (are_similar(UF.FIND_SET(i1), i2)) { // two options: (1) compare i1, i2 (2) compare parent[i1], i2
                if (are_similar(i1, i2)) {
                    UF.UNION(i1, i2);
                }
            }
        }
        
        std::vector<int> new_index(num_vertices_, num_vertices_);
        size_t counter = 0;
        std::vector<Vertex> new_vertices;
        for (size_t i=0; i < num_vertices_; ++i) {
            if (UF.parents[i] == i) {
                new_index[i] = counter;
                new_vertices.push_back(vertices_[i]);
                ++counter;
            }
            else {
                new_index[i] = new_index[UF.FIND_SET(i)];
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

// other graph operations
    
    // only operates on edges, i.e. vertex data structure doesn't matter
    virtual void find_triangles(std::vector<Triangle> &triangles) const {
        triangles.clear();
        for (auto &e: edges_) {
            int i1 = e.first;
            int i2 = e.second;
            for (int iv=e.second; iv<num_vertices_; ++iv) {
                if (edges_.find(Edge(i1, iv)) != edges_.end() && edges_.find(Edge(i2, iv)) != edges_.end()) {
                    triangles.push_back(Triangle(i1, i2, iv));
                }
            }
        }
        std::cout << triangles.size() << "\ttriangles found." << std::endl;
    }

// print
    
    virtual void print_info() const {
        std::cout << "***** Information about Graph:" << std::endl
                  << "Number of vertices:\t" << num_vertices_ << std::endl
                  << "Number of edges:\t"    << num_edges_ << std::endl;
    }
    
    virtual void print_edges() const {
        std::cout << "Edges:" << std::endl;
        for (auto const &e : edges_) {
            std::cout << "\t" << e.first << "\t" << e.second << std::endl;
        }
    }
//TODO: finish this!
//    virtual void print_edges(const std::vector<std::vector<int>>& labelmap) const {
//        std::cout << "Edges: map gt vertex to our results" << std::endl;
//        for (auto const &e : edges_) {
//            std::cout << "\t" << e.first << "\t" << e.second << std::endl;
//        }
//    }


    virtual void print_edges(const std::map<int,int>& labelmap) const {
        std::cout << "Edges(map to orig):" << std::endl;
        for (auto const &e : edges_) {
            std::cout << "\t" << labelmap.at(e.first) << "\t" << labelmap.at(e.second) << std::endl;
        }
    }

    static void print_triangles(std::vector<Triangle> &triangles) {
        for (auto &t: triangles) {
            std::cout << "(" << t.first << ",\t" << t.second << ",\t" << t.third << ")" << std::endl;
        }
    }
    
    virtual void print_vertex(int i) const {}

// write
    virtual void write_info(std::string outputDir, std::string file_name) const {
        {//create outputDir
#ifdef _WIN32
            std::string cmd="mkdir "+outputDir;
#else
            std::string cmd="mkdir -p "+outputDir;
#endif
            system(cmd.c_str());
        }

        std::ofstream myfile;
        myfile.open(outputDir +"/"+ file_name);
        myfile<< "# Information about Graph:\n"
              << "# Number of vertices (for non-plane, holes keep in the graph):\n"
              << num_vertices_ << "\n"
              << "# Number of edges:\n"
              << num_edges_ << "\n";

        for (auto const &e : edges_) {
            myfile << e.first << ","
                   << e.second << "\n";
        }

        myfile.close();
    }
};

#endif // GRAPH_H_
