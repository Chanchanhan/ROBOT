#include <opencv2/opencv.hpp>


class Frame
{
public:
	cv::Mat img;
	//SE3D dpose;
	//SE3D grondTruthPose;
    cv::Mat segmentation;
	std::vector<cv::Point2i> contour;

	unsigned int index;

	void segment();
};

void Frame::segment() {

}

