#ifndef ROBOT_BRESENHAM
#define ROBOT_BRESENHAM

#include <opencv2/opencv.hpp>

void BresenhamCircle(cv::Point2i veterx,int radius,std::vector<cv::Point2i>& sampleset);


#endif
