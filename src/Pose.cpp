#include "Pose.h"
Pose::Pose() {
}


Pose::Pose(double data[6])
{
  m_pose= Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(data[0],data[1],data[2])),Eigen::Vector3d(data[3],data[4],data[5]));
}
Pose::Pose(float data[6])
{
    m_pose= Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d((double)data[0],(double)data[1],(double)data[2])),
                         Eigen::Vector3d((double)data[3],(double)data[4],(double)data[5]));
}
void Pose::getExtrinsicMat(cv::Mat &extrinsic) {
    Eigen::Matrix< double, int(4), int(4) > T = m_pose.matrix();

   for(int i=0;i<4;i++){
       for(int j=0;j<4;j++){
           extrinsic.at<float>(i,j)=(float)T(i,j);
       }
   }
}
Pose::~Pose(){
}