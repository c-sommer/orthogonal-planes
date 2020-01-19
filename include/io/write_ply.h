/**
BSD 3-Clause License

This file is part of the code accompanying the paper
From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
accepted for Publication in IEEE Robotics and Automation Letters (RA-L) 2020.

Copyright (c) 2019, Yumin Sun.
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>

/*
 * height by width point cloud xyz cv::Mat, type double precision.
 * height by width color R,G,B cv::Mat, type int.
 */
void writePLY(const cv::Mat& x, const cv::Mat& y,const cv::Mat& z,const cv::Mat& R, const cv::Mat& G, const cv::Mat& B, const char* FileName)
{
  std::ofstream outFile( FileName );

  if ( !outFile.is_open() ) {
    std::cout<<"not open...\n";
    return;
  }

  const int pointNum = ( int ) x.rows*( int ) x.cols;

  outFile << "ply" << std::endl;
  outFile << "format ascii 1.0" << std::endl;
  outFile << "element vertex " << pointNum << std::endl;
  outFile << "property float x" << std::endl;
  outFile << "property float y" << std::endl;
  outFile << "property float z" << std::endl;
  outFile << "property float r" << std::endl;
  outFile << "property float g" << std::endl;
  outFile << "property float b" << std::endl;
  outFile << "end_header" << std::endl;

  for ( int r = 0; r < ( int ) x.rows; ++r )
  {
    for ( int c = 0; c < ( int ) x.cols; ++c )
    {
      outFile << x.at<double>(r,c) << " " << y.at<double>(r,c) << " " << z.at<double>(r,c);

      outFile<<" " << R.at<int>(r,c) << " " << G.at<int>(r,c) << " " << B.at<int>(r,c);

      outFile << std::endl;
    }
  }

  return;
}



void writePLY_normals(const cv::Mat& x, const cv::Mat& y,const cv::Mat& z,const cv::Mat& nx, const cv::Mat& ny, const cv::Mat& nz, const char* FileName)
{
  std::ofstream outFile( FileName );

  if ( !outFile.is_open() ) {
    std::cout<<"not open...\n";
    return;
  }

  const int pointNum = ( int ) x.rows*( int ) x.cols;

  int valid = 0;
  for ( int r = 0; r < ( int ) x.rows; ++r )
  {
    for ( int c = 0; c < ( int ) x.cols; ++c )
    {
      bool point_ok = std::isfinite(x.at<double>(r,c)) && std::isfinite(y.at<double>(r,c)) && std::isfinite(z.at<double>(r,c));
      bool normal_ok = std::isfinite(nx.at<double>(r,c)) && std::isfinite( ny.at<double>(r,c)) && std::isfinite( nz.at<double>(r,c));

      if(point_ok && normal_ok){
        valid++;
      }

    }
  }

  outFile << "ply" << std::endl;
  outFile << "format ascii 1.0" << std::endl;
  outFile << "element vertex " << valid << std::endl;
  outFile << "property float x" << std::endl;
  outFile << "property float y" << std::endl;
  outFile << "property float z" << std::endl;
  outFile << "property float nx" << std::endl;
  outFile << "property float ny" << std::endl;
  outFile << "property float nz" << std::endl;
  outFile << "end_header" << std::endl;


  for ( int r = 0; r < ( int ) x.rows; ++r )
  {
    for ( int c = 0; c < ( int ) x.cols; ++c )
    {
      bool point_ok = std::isfinite(x.at<double>(r,c)) && std::isfinite(y.at<double>(r,c)) && std::isfinite(z.at<double>(r,c));
      bool normal_ok = std::isfinite(nx.at<double>(r,c)) && std::isfinite( ny.at<double>(r,c)) && std::isfinite( nz.at<double>(r,c));

      if(point_ok && normal_ok){
        outFile << x.at<double>(r,c) << " " << y.at<double>(r,c) << " " << z.at<double>(r,c);

        outFile<<" " << nx.at<double>(r,c) << " " << ny.at<double>(r,c) << " " << nz.at<double>(r,c);

        outFile << std::endl;
      }

    }
  }

  return;
}
