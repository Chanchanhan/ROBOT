#ifndef _POSE_H_
#define _POSE_H_

#include <Eigen/Dense>
#include <sophus/se3.hpp>
class Pose{
    explicit Pose(double data[6]);

private:
  Sophus::SE3d m_pose ;
};

#endif