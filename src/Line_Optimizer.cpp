/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include<iostream>
#include "../include/Optimizer.h"
#include "../g2o/types/line3d.h"
#include<vector>
#include "../g2o/core/block_solver.h"
#include "../g2o/core/optimization_algorithm_levenberg.h"
#include "../g2o/core/optimization_algorithm_gauss_newton.h"
#include "../g2o/core/optimization_algorithm_factory.h"
#include "../g2o/core/robust_kernel_impl.h"
#include "../g2o/solvers/linear_solver_eigen.h"
#include "../g2o/solvers/linear_solver_dense.h"

#include<Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/Converter.h"
#include "../include/Optimizer.h"
#include "../g2o/core/block_solver.h"
#include "../g2o/core/optimization_algorithm_levenberg.h"
#include "../g2o/core/robust_kernel_impl.h"
#include "../g2o/solvers/linear_solver_eigen.h"
#include "../g2o/solvers/linear_solver_dense.h"
using namespace std;
using namespace cv;
using namespace g2o;

/*/g2o practice
  SparseOptimizer* g = new SparseOptimizer();
  ParameterSE3Offset* odomOffset = new ParameterSE3Offset();
  odomOffset->setId(0);
  g->addParameter(odomOffset);

  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  OptimizationAlgorithm* solver = solverFactory->construct(strSolver, solverProperty);
  g->setAlgorithm(solver);
*/


int PoseOptimizationLines(const cv::Mat& P2, std::vector<g2o::Line3D>& L_3D, std::vector<g2o::Vector2d>& L_2D, cv::Mat init)
{
    // Construct g2o optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
  // SparseOptimizer optimizer;
  // OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  // OptimizationAlgorithmProperty solverProperty;
  // string strSolver = "lm_var";

  //   solverFactory->listSolvers(std::cout);
  // OptimizationAlgorithm* solver = solverFactory->construct(strSolver, solverProperty);
  // optimizer.setAlgorithm(solver);
    

    int nInitialCorrespondences=0;

    // Set Frame vertex Tcw, transformation from the world coordinates to the current frame camera coordinate.
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(init));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = L_3D.size();
    std::vector<bool> mvbOutlier;
    mvbOutlier.reserve(N);

    // for Monocular
    std::vector<g2o::EdgeSE3ProjectLinesOnlyPose*> vpEdgesMono;
    std::vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);
   
    // Add unary edge
    {
    for(int i=0; i<N; i++)
    {
        //MapPoint* pMP = pFrame->mvpMapPoints[i];
         {
            // Monocular observation
            // 单目情况
                nInitialCorrespondences++;
                mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
//                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << L_2D[i][0], L_2D[i][1];
// cout<<"Point "<<obs<<endl;
                g2o::EdgeSE3ProjectLinesOnlyPose* e = new g2o::EdgeSE3ProjectLinesOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
//                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity());

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = P2.at<double>(0,0);
                e->fy = P2.at<double>(1,1);
                e->cx = P2.at<double>(0,2);
                e->cy = P2.at<double>(1,2);
                e->set_camera();
                // cout<<e->Proj<<endl;//No problem


                e->Lw = L_3D[i];
                // e->Xw[0] = (float)( (i%7)* 8);//8mm each with 7 circles a row
                // e->Xw[1] = (float)(i%7?row:++row)*8;
                // e->Xw[2] = (float)(0);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
          }
    }
    }
    cout<<"Correspondences "<<nInitialCorrespondences<<endl;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

    //Threshold from the Chi2 distribution
    // const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    // const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={40,20,5,50};

    int nBad=0;
    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(init));
        //cout<<"Optimization Initial guess \n"<<vSE3->estimate()<<endl;

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        //cout<<"Optimization guess \n"<<vSE3->estimate()<<endl;

        nBad=0;
        double sum =0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectLinesOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(mvbOutlier[idx])
            {
                e->computeError(); // NOTE add errors from outliers
            }

            const float chi2 = e->chi2();
            sum+=chi2;
//            if(chi2>chi2Mono[it])
//            {
//                mvbOutlier[idx]=true;
//                e->setLevel(1);                 // 设置为outlier
//                nBad++;
//            }
//            else
//            {
//                mvbOutlier[idx]=false;
//                e->setLevel(0);                 // 设置为inlier
//            }
//cout<<"Error of node "<<i<<": "<<chi2<<endl;
            if(it==2)
                e->setRobustKernel(0); // 除了前两次优化需要RobustKernel以外, 其余的优化都不需要
        }
    }
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    if(pose.empty())cout <<"Result Not Available!"<<endl;
    else
    cout<<"Optimized Pose: \n"<<pose<<"\n"<<endl;
//    cout<<"cross product: "<<pose.colRange(1,1)<<endl;
//    cout<<"cross product: "<<pose.colRange(2,3).dot(pose.colRange(2,3))<<endl;
//    cout<<"Optimized Pose: "<<pose<<endl;
    //pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

