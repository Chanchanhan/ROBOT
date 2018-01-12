#include <opencv2/imgproc.hpp>

#include "DataPaser.h"
#include "GlobalConfig.h"
#include "Pose.h"
#include "frame.h"
DataPaser::DataPaser(const int &argc,char **argv,const OcvYamlConfig& config)
{
  Config::loadConfig(config);
  FLAGS_log_dir=config.text("Output.Directory.LOG_DIR");     
  FLAGS_stderrthreshold = std::lround(config.value_f("LOG_Threshold"));  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3

  gtData= std::ifstream(Config::configInstance().gtFile);      
  /*** load first frame to init***/  
  starframeId=Config::configInstance().START_INDEX;
  int k=0;
  {  
    while(k<starframeId){
      std::string str;
      std::getline(gtData,str) ;
      k++;
    } 
    float gtPose[6]={0};
    std::string str,filename;
    std::getline(gtData,str) ;
    
    std::istringstream gt_line(str);
    gt_line>>filename;
    int i=0;
    for (float pos; gt_line >> pos; ++i) {   
	  gtPose[i] = pos;   
    }
//    LOG(INFO)<<" getPose"<<" " << gtPose[0]<<" " << gtPose[1]<<" " << gtPose[2]<<" " << gtPose[3]<<" " << gtPose[4]<<" " << gtPose[5];
    memcpy(prePose,gtPose,sizeof(float)*6);
  }
    model.loadObj(Config::configInstance().objFile);
}
bool DataPaser::doTraking()
{
    if(Config::configInstance().USE_VIDEO){
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
  Config &gConfig = Config::configInstance();
  videoCapture.open(gConfig.videoPath);
  if (!videoCapture.isOpened())
  {
    LOG(ERROR)<<("Cannot open camera\n");
    return ;    
  }
//    gConfig.VIDEO_WIDTH = (int)videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
//    gConfig.VIDEO_HEIGHT = (int)videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
//   auto traker = std::make_shared<OD::Traker>(prePose,true); 
  cv::Mat img;
  while (videoCapture.read(img))
  {
      cv::Mat frameDrawing = img.clone();
      Pose pose(prePose);
      if(!gConfig.USE_GT){
	 
	    
      }

      Frame curFrame;
      curFrame.img=img.clone();
      model.displayCV(pose,cv::Scalar(255,255,0),frameDrawing);
      model.getContourPointsAndIts3DPoints(pose,curFrame.VerticesNear2ContourX3D,curFrame.VerticesNear2ContourX2D,curFrame.contourX2D);
      if(gConfig.CV_DRAW_FRAME){
	    cv::imshow("frameDrawing",frameDrawing);
	    cv::waitKey(1);
      }
      if(Config::configInstance().USE_GT){
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

