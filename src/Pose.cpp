#include "Pose.h"

using namespace std;

Sophus::SE3d Data2Pose(double * data)
{
    Sophus::Vector6d se3_pose;
    se3_pose<<data[3],data[4],data[5],
            data[0],data[1],data[2];
    return Sophus::SE3d::exp(se3_pose);
}

Sophus::SE3d Data2Pose(float * data)
{
    Sophus::Vector6d se3_pose;
    se3_pose<<(double)data[3],(double)data[4],(double)data[5],
            (double)data[0],(double)data[1],(double)data[2];

    auto m_pose = Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d((double) data[0], (double) data[1], (double) data[2])),
                          Eigen::Vector3d((double) data[3], (double) data[4], (double) data[5]));

    return m_pose;
}

cv::Mat Se2cvf(Sophus::SE3d pose)
{
    cv::Mat pose_cv(4,4,CV_32F);
    cv::Mat temp;
    cv::eigen2cv(pose.matrix(),temp);
    temp.convertTo(pose_cv,CV_32F);
    return pose_cv;
}
