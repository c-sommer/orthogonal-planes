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

#include "pcshow.h"
// libraries
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <sophus/so3.hpp>
#include "draw_corners.h"

/**
 * class for 3D point viewing
 */
class PointViewer3D {

    const int stride;
    const int num_points;
    GLuint vbo;
    
public:
    PointViewer3D(std::vector<Vec3> *points) : num_points(points->size()), stride(sizeof(Vec3)) {
        void* ptr_to_data = &(*points).front();
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, num_points * stride, ptr_to_data, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    virtual ~PointViewer3D() {
        glDeleteBuffers(1, &vbo);
    }

    void draw() {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(3, GL_DOUBLE, stride, 0); // GL_FLOAT
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, num_points);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

};

class PointViewer3Df {

    using Vec3f = Eigen::Vector3f;

    const int stride;
    const int num_points;
    GLuint vbo;
    
public:
    PointViewer3Df(std::vector<Vec3f> *points) : num_points(points->size()), stride(sizeof(Vec3f)) {
        void* ptr_to_data = &(*points).front();
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, num_points * stride, ptr_to_data, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    virtual ~PointViewer3Df() {
        glDeleteBuffers(1, &vbo);
    }

    void draw() {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(3, GL_FLOAT, stride, 0); // GL_FLOAT
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, num_points);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

};

/**
 * actual point cloud viewing function
 */
void pcshow_corners(std::vector<Vec3> &points, const std::vector<Corner>& corners, std::string win_name)
{

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind(win_name, 640, 480);
    const int UI_WIDTH = 180;

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState camera(
                pangolin::ProjectionMatrix(640, 480, 480, 480, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(0, 0.5, 1.0, 0, 0, 0, pangolin::AxisY)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& display = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(camera));
    
    while( !pangolin::ShouldQuit() )
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Activate efficiently by object
        display.Activate(camera);

        PointViewer3D cloud(&points);
        cloud.draw();
        draw_corners(corners);
        
        pangolin::FinishFrame();
    }
}

void pcshow_corners(std::vector<Eigen::Vector3f> &points, const std::vector<Corner>& corners, std::string win_name)
{

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind(win_name, 640, 480);
    const int UI_WIDTH = 180;

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState camera(
                pangolin::ProjectionMatrix(640, 480, 530, 530, 320, 240, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -2.5, 12.5, 0, 0, 0, pangolin::AxisY)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& display = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(camera));
    
    while( !pangolin::ShouldQuit() )
    {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Activate efficiently by object
        display.Activate(camera);

        PointViewer3Df cloud(&points);
        cloud.draw();
        draw_corners(corners);
        
        pangolin::FinishFrame();
    }
}
