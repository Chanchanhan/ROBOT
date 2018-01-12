#ifndef DATA_PASER_H
#define DATA_PASER_H
#include <iostream>
#include<fstream>
#include <opencv2/highgui.hpp>
#include "OcvYamlConfig.h"
class DataPaser{
public:
    explicit DataPaser(const int &argc,char **argv,const OcvYamlConfig& config);
    bool doTraking();
    bool doDetecting();
    ~DataPaser();
private:
//   std::ifstream gtData();
//   void doTraking_Frame(const  std::make_shared<OD::Traker> traker, const cv::Mat &frame);
  
  int starframeId;
  float prePose[6];
  std::ifstream  gtData;
private:
  void getNextGTData(float *newPose);
  void doTrakingWithVideo();
  void doTrakingWithPictures();

};
#endif