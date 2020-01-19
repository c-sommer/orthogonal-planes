//============================================================================
// Name        : pcshow.cpp
// Author      : adapted from Pangolin by S. Lovegrove and PangoCloud by J. Liu
// Date        : 10/2018
// License     : MIT License (Pangolin) / GNU General Public License
// Description : display 3D point cloud using Pangolin viewer
// Github      : https://github.com/stevenlovegrove/Pangolin#include
//               https://github.com/HTLife/PangoCloud
//============================================================================

#include "pcshow.h"
// libraries
#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

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

class PointViewer3DN {

    const int stride;
    const int num_points;
    GLuint vbo;
    
public:
    PointViewer3DN(std::vector<Vec6> *points) : num_points(points->size()), stride(sizeof(Vec6)) {
        void* ptr_to_data = &(*points).front();
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, num_points * stride, ptr_to_data, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    virtual ~PointViewer3DN() {
        glDeleteBuffers(1, &vbo);
    }

    void draw() {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(3, GL_DOUBLE, stride, 0); // GL_FLOAT
        glColorPointer(3, GL_DOUBLE, stride, (void*)(3 * sizeof(double)));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glDrawArrays(GL_POINTS, 0, num_points);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

};

class PointViewer3DNf {

    using Vec6f = Eigen::Matrix<float, 6, 1>;

    const int stride;
    const int num_points;
    GLuint vbo;
    
public:
    PointViewer3DNf(std::vector<Vec6f> *points) : num_points(points->size()), stride(sizeof(Vec6f)) {
        void* ptr_to_data = &(*points).front();
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, num_points * stride, ptr_to_data, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    virtual ~PointViewer3DNf() {
        glDeleteBuffers(1, &vbo);
    }

    void draw() {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexPointer(3, GL_FLOAT, stride, 0); // GL_FLOAT
        glColorPointer(3, GL_FLOAT, stride, (void*)(3 * sizeof(float)));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glDrawArrays(GL_POINTS, 0, num_points);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

};

/**
 * actual point cloud viewing function
 */
void pcshow(std::vector<Vec3> &points, std::string win_name)
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
        
        pangolin::FinishFrame();
    }
}

void pcshow(std::vector<Eigen::Vector3f> &points, std::string win_name)
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
        
        pangolin::FinishFrame();
    }
}

void pcshow(std::vector<Vec6> &points, std::string win_name)
{

    std::vector<Vec6> points_new;

    for (auto p: points) {
        p(3) = 0.5 * (p(3)+1);
        p(4) = 0.5 * (p(4)+1);
        p(5) = 0.5 * (p(5)+1);
        points_new.push_back(p);
    }

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind(win_name, 640, 480);
    const int UI_WIDTH = 180;

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState camera(
                pangolin::ProjectionMatrix(640, 480, 530, 530, 320, 240, 0.1, 1000),
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

        PointViewer3DN cloud(&points_new);
        cloud.draw();
        
        pangolin::FinishFrame();
    }
}

void pcshow(std::vector<Eigen::Matrix<float, 6, 1>> &points, std::string win_name)
{

    std::vector<Eigen::Matrix<float, 6, 1>> points_new;

    for (auto p: points) {
        p(3) = 0.5 * (p(3)+1);
        p(4) = 0.5 * (p(4)+1);
        p(5) = 0.5 * (p(5)+1);
        points_new.push_back(p);
    }

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

        PointViewer3DNf cloud(&points_new);
        cloud.draw();
        
        pangolin::FinishFrame();
    }
}
