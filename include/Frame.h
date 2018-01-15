#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H
#include <opencv2/opencv.hpp>
#include "Pose.h"
#include <memory>

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	cv::Mat img;
	//SE3D dpose;
	//SE3D grondTruthPose;
	Pose m_pose;
	Pose gt_Pose;
	cv::Mat segmentation;
    cv::Mat dt;
    cv::Mat dtLocation;
	std::vector<cv::Point2d> VerticesNear2ContourX2D;
    std::vector<cv::Point3d> VerticesNear2ContourX3D;
	std::vector<cv::Point> contourX2D;

    double fw_prior;
    double bg_prior;

	unsigned int index;
	void Segment();
	void DTMap();
    cv::Point nearstContourP(const cv::Point &p);
    void ComputePrior();

};

typedef std::shared_ptr<Frame> FramePtr;

#endif