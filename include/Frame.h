#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H
#include <opencv2/opencv.hpp>
#include "Pose.h"
#include <memory>

class Region;
class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	cv::Mat img;
    //data of py
    std::vector<cv::Mat> imgPyramid;
    std::vector<cv::Mat> fw_posteriorPyramid;
    std::vector<cv::Mat> bg_posteriorPyramid;
    cv::Mat fw_posterior;
    cv::Mat bg_posterior;
	//SE3D dpose;
	//SE3D grondTruthPose;
	Pose m_pose;
	Pose gt_Pose;
    cv::Mat segmentation;
    cv::Mat bound_map;
    cv::Mat dt;
    cv::Mat dtLocation;
	std::vector<cv::Point2d> VerticesNear2ContourX2D;
    std::vector<cv::Point3d> VerticesNear2ContourX3D;
	std::vector<cv::Point> contourX2D;

    double fw_prior;
    double bg_prior;

	unsigned int index;
public:
	void Segment();
	void Segment(const cv::Mat &inPut,cv::Mat &segmentation,cv::Mat &bound_map,std::vector<cv::Point> &contourX2D);

	void DTMap();
    void DTMap(const cv::Mat &inPut,cv::Mat &dt,cv::Mat &dtLocation);
    void GetPyraid(const int &nPyraid);
    cv::Point nearstContourP(const cv::Point &p);
    void ComputePosterior(const std::vector<Region>& rg);
	void ComputePosterior(const cv::Mat &inPutImg,const std::vector<Region>& rg);
};

typedef std::shared_ptr<Frame> FramePtr;

#endif