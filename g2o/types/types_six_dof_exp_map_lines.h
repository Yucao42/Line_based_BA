// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)
// Modified by Raúl Mur Artal (2016)
// Added EdgeStereoSE3ProjectXYZ (project using focal_length in x,y directions)
// Added EdgeSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)
// Added EdgeStereoSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include "line3d.h"
#include <Eigen/Geometry>
#include "../core/g2o_core_api.h"
#include <iostream>
namespace g2o {
namespace types_six_dof_expmap {
void init();
}
Vector2d project2d(const Vector3d& v);


using namespace Eigen;

class G2O_CORE_API VertexSE3Expmap : public BaseVertex<6, SE3Quat> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};


class G2O_CORE_API EdgeSE3ProjectLinesOnlyPose : public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectLinesOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement),res;
    res = project2d(Proj * ((v1->estimate().map(Lw)).w()));
    _error = obs- res;
  }

  bool isDepthPositive() {
    return true;
  }

  virtual void linearizeOplus();

  void set_camera(){
    m.fill(0.);
    m(0,0) = fx;
    m(1,1) = fy;
    m(0,2) = cx;
    m(1,2) = cy; 
    m(2,2) = 1 ;
    Proj = m.inverse().transpose();
  }

  Line3D Lw; ///
  double fx, fy, cx, cy; ///
  Matrix3d m, Proj;
};


} // end namespace

#endif
