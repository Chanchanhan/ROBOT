#ifndef DATA_PASER_H
#define DATA_PASER_H
#include <iostream>
#include<fstream>
#include <opencv2/highgui.hpp>
#include "OcvYamlConfig.h"
#include "Model.h"
#include "frame.h"


class DataPaser{
public:
    explicit DataPaser(const OcvYamlConfig& config);
    bool doTraking();
    bool doDetecting();
    bool parseAFrame(FramePtr f);
    ~DataPaser();
private:
//   std::ifstream gtData();
//   void doTraking_Frame(const  std::make_shared<OD::Traker> traker, const cv::Mat &frame);
  
  int starframeId;
  float prePose[6];
    Model model;
  std::ifstream  gtData;
private:
  void getNextGTData(float *newPose);
  void doTrakingWithVideo();
  void doTrakingWithPictures();
    cv::VideoCapture videoCapture;

};
#endif