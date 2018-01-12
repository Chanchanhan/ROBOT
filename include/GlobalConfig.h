#ifndef _CONFIG_H
#define _CONFIG_H

#include <iostream>
#include <string>
#include <fstream>
#include <glog/logging.h>
#include "OcvYamlConfig.h"
  class Config
  {
  public:
    int IMG_PYR_NUMBER;
    int VIDEO_WIDTH;
    int VIDEO_HEIGHT;
    bool USE_VIDEO;
    bool USE_GT;
    int START_INDEX;
    float distortions[5];
    std::string videoPath;
    std::string objFile;  
    std::string gtFile;  
    std::string DISTORTIONS; 
  public:
    
    static void loadConfig(const OcvYamlConfig &ocvYamlConfig){
      //parameters for init
      {	
		Config::configInstance().USE_VIDEO = std::lround(ocvYamlConfig.value_f("USE_VIDEO"))== 1;
		Config::configInstance().objFile = ocvYamlConfig.text("Input.Directory.Obj");;
		Config::configInstance().videoPath =ocvYamlConfig.text("Input.Directory.Video");
		Config::configInstance().gtFile= ocvYamlConfig.text("Input.Directory.GroudTruth");
		Config::configInstance().DISTORTIONS= ocvYamlConfig.text("Input.Directory.DISTORTIONS");
		Config::configInstance().START_INDEX=std::lround(ocvYamlConfig.value_f("Init_Frame_Index"));
		Config::configInstance().USE_GT=std::lround(ocvYamlConfig.value_f("USE_GT_DATA"))==1;
		Config::configInstance().VIDEO_HEIGHT=std::lround(ocvYamlConfig.value_f("VIDEO_HEIGHT"));
		Config::configInstance().VIDEO_WIDTH=std::lround(ocvYamlConfig.value_f("VIDEO_WIDTH"));
      }
      //parameters for Detect
      {
	Config::configInstance().IMG_PYR_NUMBER=std::lround(ocvYamlConfig.value_f("IMG_PYR_NUMBER"));
      }
      //Load Files
      {
	loadFiles();
      }
    }
    
    static Config& configInstance() {
      static Config G_CONFIG;
      return G_CONFIG;
    }
    private:
      static void loadFiles(){
	std::string str;
	std::ifstream _DISTORTIONS=std::ifstream(Config::configInstance().DISTORTIONS);
	if(_DISTORTIONS.is_open()){
	  std::getline(_DISTORTIONS,str) ;    
	  std::istringstream gt_line(str);
	  int i=0;
	  for (float pos; gt_line >> pos; ++i) {   
		configInstance().distortions[i] = pos;   
	  }
	}else{
	  memset(configInstance().distortions,0,5*sizeof(float));
	}
    }
  };
    


#endif