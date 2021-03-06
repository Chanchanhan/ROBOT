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
	Sophus::SE3d m_pose;
	Sophus::SE3d gt_Pose;
    cv::Mat segmentation;
    cv::Mat bound_map;
    cv::Mat dt;
    cv::Mat dtLocationInside;
	cv::Mat dtLocationOutside;

	std::vector<cv::Point2d> VerticesNear2ContourX2D;
    std::vector<cv::Point3d> VerticesNear2ContourX3D;
	std::vector<cv::Point> contourX2D;
	std::vector<cv::Point> gt_contourX2D;

    double fw_prior;
    double bg_prior;

	unsigned int index;
public:
	void Segment();
	void Segment(const cv::Mat &inPutImg,const std::vector<cv::Point> &contourX2D,cv::Mat &segmentation,cv::Mat &bound_map);

	void UpdateDTMap();
	void UpdateDTMap(const std::vector<cv::Point> &contourX2D);
    void DTMap(const cv::Mat &inPut,cv::Mat &dt,cv::Mat &dtLocation);
    void GetPyraid(const int &nPyraid);
    cv::Point GetNearstContourP(const cv::Point &p, const cv::Mat &dtLocation,const int iLevel=0);
    void ComputePosterior(const cv::Mat &inputImg,const std::vector<Region>& rg);

	void ComputePosterior(const cv::Mat &inPutImg,const std::vector<Region>& rg,cv::Mat &fw_posterior,cv::Mat &bg_posterior);
};

typedef std::shared_ptr<Frame> FramePtr;

#endif