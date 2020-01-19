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

#ifndef PCSHOW_H_
#define PCSHOW_H_

#include "definitions.h"

class Corner;

void pcshow(std::vector<Vec3> &points, std::string win_name = "Point Cloud"); // points only

void pcshow(std::vector<Eigen::Vector3f> &points, std::string win_name = "Point Cloud");

void pcshow(std::vector<Vec6> &points, std::string win_name = "Colored Point Cloud"); // points and normals, use normals as colors

void pcshow(std::vector<Eigen::Matrix<float, 6, 1>> &points, std::string win_name = "Colored Point Cloud");

void pcshow_corners(std::vector<Vec3> &points, const std::vector<Corner>& corners, std::string win_name = "Point Cloud with Corners");

void pcshow_corners(std::vector<Eigen::Vector3f> &points, const std::vector<Corner>& corners, std::string win_name = "Point Cloud with Corners");

void pcshow_lines(std::vector<Vec3> &points, const std::vector<Eigen::Matrix<double, 6, 1>>& lines, std::string win_name = "Point Cloud with Lines");

void pcshow_lines(std::vector<Eigen::Vector3f> &points, const std::vector<Eigen::Matrix<double, 6, 1>>& lines, std::string win_name = "Point Cloud with Lines");

#endif // PCSHOW_H_
