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

#include "types_six_dof_exp_map_lines.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;


Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

//Only Pose

bool EdgeSE3ProjectLinesOnlyPose::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectLinesOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


/**
 * @brief Linearization
 * Jacobian of line_2d over pose(6 DoF vector)
 */
void EdgeSE3ProjectLinesOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Line3D line_tran = vi->estimate().map(Lw);
  Vector3d u = line_tran.w(), v = line_tran.d();
  Vector3d line_2d = Proj * u;
  double x = line_2d[0];
  double y = line_2d[1];
  double invz = 1.0/line_2d[2];

  // Camera projection Jacobian
  Eigen::Matrix<double, 2, 3> J_proj;
  J_proj <<  1.0 ,  0.0  , -x*invz  ,
             0.0 ,  1.0  , -y*invz ;
  J_proj = invz * J_proj;

  //Coordinates transformation Jacobian
  Eigen::Matrix<double, 3, 6> J_tran;

  //ATTENTION: The update of epsilon is in order of rotation -> translation. 
  //So it doesn't coincide with the paper.
  J_tran << 
            0 ,   u(2),  -u(1),     0 ,   v(2),  -v(1),   
         -u(2),     0 ,   u(0),  -v(2),     0 ,   v(0),      
          u(1),  -u(0),    0  ,   v(1),  -v(0),    0   ;

  //Put them together
  _jacobianOplusXi = -J_proj * Proj * J_tran;
}

} // end namespace
