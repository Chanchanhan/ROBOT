#include <opencv2/imgproc.hpp>

#include "DataPaser.h"
#include "GlobalConfig.h"
#include "Pose.h"
#include "Frame.h"

using namespace std;

DataPaser::DataPaser(const OcvYamlConfig& config)
{
    Config::LoadConfig(config);

  FLAGS_log_dir=config.text("Output.Directory.LOG_DIR");     
  FLAGS_stderrthreshold = std::lround(config.value_f("LOG_Threshold"));  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3

  gtData= std::ifstream(Config::ConfigInstance().gtFile);
      /*** load first frame to init***/
    starframeId= Config::ConfigInstance().START_INDEX;

    int k=0;
    while(k<starframeId){
      std::string str;
      std::getline(gtData,str) ;
      k++;
    }

    videoCapture.open(Config::ConfigInstance().videoPath);
}
bool DataPaser::doTraking()
{
    if(Config::ConfigInstance().USE_VIDEO){
        doTrakingWithVideo();
    }
    else{
        doTrakingWithPictures();
    }
    return true;
}

void DataPaser::doTrakingWithPictures() {

}
void DataPaser::doTrakingWithVideo()
{
  int frameId=starframeId; 
  cv::VideoCapture videoCapture;
  Config &gConfig = Config::ConfigInstance();
  videoCapture.open(gConfig.videoPath);
  if (!videoCapture.isOpened())
  {
    LOG(ERROR)<<("Cannot open camera\n");
    return ;    
  }
    model.LoadObj(gConfig.objFile);
  cv::Mat img;
  while (videoCapture.read(img))
  {
      cv::Mat frameDrawing = img.clone();
      Sophus::SE3d pose = Data2Pose(prePose);
      if(!gConfig.USE_GT){
	 
	    
      }

      Frame curFrame;
      curFrame.img=img.clone();
      model.DisplayCV(pose, cv::Scalar(255, 255, 0), frameDrawing);
      model.GetContourPointsAndIts3DPoints(pose, curFrame.VerticesNear2ContourX3D, curFrame.VerticesNear2ContourX2D,
                                           curFrame.contourX2D);
      if(gConfig.CV_DRAW_FRAME){
	    cv::imshow("frameDrawing",frameDrawing);
	    cv::waitKey(1);
      }
      if(Config::ConfigInstance().USE_GT){
	    getNextGTData(prePose);
      }
    }
}

void DataPaser::getNextGTData(float *newPose)
{
	float gtPose[6]={0};
	std::string str,filename;
	std::getline(gtData,str) ;
	std::istringstream gt_line(str);
	gt_line>>filename;
	int i=0;
	for (float pos; gt_line >> pos; ++i) {        
	  if(std::isnan(pos)){
	    break;
	  }
	  gtPose[i] = pos;
	}     
	if(!std::isnan(gtPose[0])){
	  memcpy(newPose,gtPose,sizeof(float)*6);
	}
}
DataPaser::~DataPaser()
{

}

bool DataPaser::parseAFrame(FramePtr frame) {
    int frameId=starframeId;
    if (!videoCapture.isOpened())
    {
        LOG(ERROR)<<("Cannot open camera\n");
        return false;
    }
    cv::Mat img;
    if (!videoCapture.read(img))
        return false;

    cv::Mat frameDrawing = img.clone();
    float gtPose[6]={0};
    std::string str,temp;
    std::getline(gtData,str) ;

    std::istringstream gt_line(str);

    gt_line>>temp;
    for (int j = 0; j < 6; ++j) {
        float pos;
        gt_line >> pos;
        gtPose[j] = pos;
    }
    memcpy(prePose,gtPose,sizeof(float)*6);

    frame->img=img.clone();
    frame->gt_Pose = Data2Pose(gtPose);
    return true;
}


