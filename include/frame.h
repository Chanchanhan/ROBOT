#include <opencv2/opencv.hpp>


class Frame
{
public:
	cv::Mat img;
	//SE3D dpose;
	//SE3D grondTruthPose;
	cv::Mat segmentation;

	unsigned int index;
};

