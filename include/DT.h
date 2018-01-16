//
// Created by qqh on 18-1-15.
//

#ifndef ROBOT_DT_H
#define ROBOT_DT_H



#include <opencv2/opencv.hpp>

// Calculates the distance trasnform as described in Distance Transforms of Sampled Functions
void distanceTransfqorm(const cv::Mat &inputMatrix, cv::Mat &outputMatrix,
                       cv::Mat &locations,std::vector<float> weights = std::vector<float>());


#endif