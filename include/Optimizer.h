/**
* Pose optimization using lines
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include "../g2o/types/types_six_dof_exp_map_lines.h"
#include "../g2o/types/line3d.h"

//Parameters
//	 P2  ---Camera intrinsic matrix, 
//	L_3D ---3D lines' Pl\"ucker coordinates, 
//	L_2D ---Normalized coordinates on 2D image plane, 
//	init ---Initial Guess of Pose.
int PoseOptimizationLines(const cv::Mat& P2, std::vector<g2o::Line3D>& L_3D, std::vector<g2o::Vector2d>& L_2D, cv::Mat init=cv::Mat::eye(4,4,CV_64F));


#endif // OPTIMIZER_H
