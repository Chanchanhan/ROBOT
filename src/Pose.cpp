#include "Pose.h"

Pose::Pose(double data[6])
{
  m_pose= Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(data[0],data[1],data[2])),Eigen::Vector3d(data[3],data[4],data[5]));
}
