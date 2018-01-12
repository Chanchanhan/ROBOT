#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H
#include <opencv2/opencv.hpp>
#include "Pose.h"

class Frame
{
public:
	cv::Mat img;
	//SE3D dpose;
	//SE3D grondTruthPose;
	Pose m_pose;
	Pose gt_Pose;
	cv::Mat segmentation;
	std::vector<cv::Point2i> contour;

	unsigned int index;

	void segment();
};



#endif