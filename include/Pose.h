#ifndef _POSE_H_
#define _POSE_H_

#include <Eigen/Dense>

#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


//for SE3 tranlation is before rotation;
Sophus::SE3d Data2Pose(double * data);

Sophus::SE3d Data2Pose(float * data);

cv::Mat Se2cvf(Sophus::SE3d pose);

#endif