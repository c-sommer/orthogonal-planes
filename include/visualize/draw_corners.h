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

#ifndef DRAW_CORNERS_H_
#define DRAW_CORNERS_H_

#include "corners/Corner.h"

inline void draw_corners(std::vector<Corner> corners) {

    using Vec3 = Eigen::Vector3d;
    
    double scale = 0.08;

    glColor3d(0., 0., 1.);

    for (const auto& c : corners) {
    
        Sophus::SO3<double> rot_inv = c.rot().inverse();
        Vec3 neg_distances = -c.distances();
        const Vec3 p = rot_inv * neg_distances;
        const Vec3 n1 = rot_inv * Vec3(scale, .0, .0);
        const Vec3 n2 = rot_inv * Vec3(.0, scale, .0);
        const Vec3 n3 = rot_inv * Vec3(.0, .0, scale);

        const GLdouble verts[] = { p[0]-n1[0], p[1]-n1[1], p[2]-n1[2],
                                   p[0]+n1[0], p[1]+n1[1], p[2]+n1[2],
                                   p[0]-n2[0], p[1]-n2[1], p[2]-n2[2],
                                   p[0]+n2[0], p[1]+n2[1], p[2]+n2[2],
                                   p[0]-n3[0], p[1]-n3[1], p[2]-n3[2],
                                   p[0]+n3[0], p[1]+n3[1], p[2]+n3[2] };
        glLineWidth(5.);
        pangolin::glDrawVertices<double>(6, verts, GL_LINES, 3);
        
        const int num_vertices = 36;
        GLdouble verts_circ[3*num_vertices];
        
        for (int i=0; i<num_vertices; ++i) {
            const double alpha = i * 2 * M_PI / num_vertices;
            const double cos_a = std::cos(alpha);
            const double sin_a = std::sin(alpha);
            verts_circ[3*i]   = p[0] + cos_a * n2[0] + sin_a * n3[0];
            verts_circ[3*i+1] = p[1] + cos_a * n2[1] + sin_a * n3[1];
            verts_circ[3*i+2] = p[2] + cos_a * n2[2] + sin_a * n3[2];
        }
        
        glLineWidth(2.);
        pangolin::glDrawVertices<double>(num_vertices, verts_circ, GL_LINE_LOOP, 3);
        
        for (int i=0; i<num_vertices; ++i) {
            const double alpha = i * 2 * M_PI / num_vertices;
            const double cos_a = std::cos(alpha);
            const double sin_a = std::sin(alpha);
            verts_circ[3*i]   = p[0] + cos_a * n3[0] + sin_a * n1[0];
            verts_circ[3*i+1] = p[1] + cos_a * n3[1] + sin_a * n1[1];
            verts_circ[3*i+2] = p[2] + cos_a * n3[2] + sin_a * n1[2];
        }
        
        pangolin::glDrawVertices<double>(num_vertices, verts_circ, GL_LINE_LOOP, 3);
        
        for (int i=0; i<num_vertices; ++i) {
            const double alpha = i * 2 * M_PI / num_vertices;
            const double cos_a = std::cos(alpha);
            const double sin_a = std::sin(alpha);
            verts_circ[3*i]   = p[0] + cos_a * n1[0] + sin_a * n2[0];
            verts_circ[3*i+1] = p[1] + cos_a * n1[1] + sin_a * n2[1];
            verts_circ[3*i+2] = p[2] + cos_a * n1[2] + sin_a * n2[2];
        }
        
        pangolin::glDrawVertices<double>(num_vertices, verts_circ, GL_LINE_LOOP, 3);
    }
    
    glColor3d(1., 1., 1.);
}

#endif // DRAW_CORNERS_H_
