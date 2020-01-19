//
// Copyright 2014 Mitsubishi Electric Research Laboratories All
// Rights Reserved.
//
// Permission to use, copy and modify this software and its
// documentation without fee for educational, research and non-profit
// purposes, is hereby granted, provided that the above copyright
// notice, this paragraph, and the following three paragraphs appear
// in all copies.
//
// To request permission to incorporate this software into commercial
// products contact: Director; Mitsubishi Electric Research
// Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.
//
// IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
// INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
// LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
// DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGES.
//
// MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
// SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <limits>

namespace LA {
	inline static bool eig33sym(double K[3][3], double s[3], double V[3][3])
	{
		//below we did not specify row major since it does not matter, K==K'
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
			Eigen::Map<Eigen::Matrix3d>(K[0], 3, 3) );
		Eigen::Map<Eigen::Vector3d>(s,3,1)=es.eigenvalues();
		//below we need to specify row major since V!=V'
		Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(V[0],3,3)=es.eigenvectors();
		return true;
	}
}//end of namespace LA