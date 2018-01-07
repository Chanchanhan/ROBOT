#include <opencv2/opencv.hpp>


class Frame
{
public:
	cv::Mat *img;
	//SE3D dpose;
	//SE3D grondTruthPose;
	unsigned int index;
};

