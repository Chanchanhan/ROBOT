#include "dataPaser.h"
#include "GlobalConfig.h"
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
    LOG(WARNING)<<" getPose"<<" " << gtPose[0]<<" " << gtPose[1]<<" " << gtPose[2]<<" " << gtPose[3]<<" " << gtPose[4]<<" " << gtPose[5];
    memcpy(prePose,gtPose,sizeof(float)*6);
  }
}

DataPaser::~DataPaser()
{

}


