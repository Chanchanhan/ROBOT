#ifndef _POSE_H_
#define _POSE_H_

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
class Pose{
public:
    Pose();
    Pose(double data[6]);
    Pose(float data[6]);

    void getExtrinsicMat(cv::Mat &extrinsic);
    ~Pose();

private:
  Sophus::SE3d m_pose ;
};

#endif