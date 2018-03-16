#ifndef _CONFIG_H
#define _CONFIG_H

#include <iostream>
#include <string>
#include <fstream>
#include <glog/logging.h>
#include "OcvYamlConfig.h"

#define PROJECT_WITH_GT

class Config
{
public:
    int IMG_PYR_NUMBER;
    int VIDEO_WIDTH;
    int VIDEO_HEIGHT;
    int IMG_DT_WEIGHT;
    int TK_VER_NUMBER;
    float FX,FY;
    float CX,CY;
    bool USE_VIDEO;
    bool USE_GT;
    int START_INDEX;
    float distortions[5];
    std::string videoPath;
    std::string objFile;
    std::string gtFile;
    std::string DISTORTIONS;
    bool CV_DRAW_FRAME;


    //solver
    int SV_MAX_WRONG_POINT;
    int SV_MAX_ITERATIONS ;
    float SV_E_TOO_SMALL_SIZE ;
    float SV_E_OK ;
    float  SV_HE_b;
    float  SV_LAMDA_b;
    //debug
    std::vector<unsigned char> pointState;

public:

    static void loadConfig(const OcvYamlConfig &ocvYamlConfig){
        Config &configInstance = Config::configInstance() ;

        //parameters for init
        {
            configInstance.USE_VIDEO = std::lround(ocvYamlConfig.value_f("USE_VIDEO"))== 1;
            configInstance.objFile = ocvYamlConfig.text("Input.Directory.Obj");;
            configInstance.videoPath =ocvYamlConfig.text("Input.Directory.Video");
            configInstance.gtFile= ocvYamlConfig.text("Input.Directory.GroudTruth");
            configInstance.DISTORTIONS= ocvYamlConfig.text("Input.Directory.DISTORTIONS");
            configInstance.START_INDEX=std::lround(ocvYamlConfig.value_f("Init_Frame_Index"));
            configInstance.USE_GT=std::lround(ocvYamlConfig.value_f("USE_GT_DATA"))==1;
            configInstance.VIDEO_HEIGHT=std::lround(ocvYamlConfig.value_f("VIDEO_HEIGHT"));
            configInstance.VIDEO_WIDTH=std::lround(ocvYamlConfig.value_f("VIDEO_WIDTH"));
            configInstance.FX= ocvYamlConfig.value_f("Calib_FX");
            configInstance.FY= ocvYamlConfig.value_f("Calib_FY");
            configInstance.CX = ocvYamlConfig.value_f("Calib_CX");
            configInstance.CY= ocvYamlConfig.value_f("Calib_CY");

        }
        //parameters for CV
        {
            configInstance.CV_DRAW_FRAME=std::lround(ocvYamlConfig.value_f("CV_DRAW_FRAME"))== 1;;

        }
        //parameters for Detect
        {
            configInstance.TK_VER_NUMBER=std::lround(ocvYamlConfig.value_f("TK_VER_NUMBER"));
            configInstance.IMG_DT_WEIGHT=ocvYamlConfig.value_f("IMG_DT_WEIGHT");
            configInstance.IMG_PYR_NUMBER=std::lround(ocvYamlConfig.value_f("IMG_PYR_NUMBER"));

        }
        //parameters for solvers
        {
            configInstance.SV_HE_b= ocvYamlConfig.value_f("SV_HE_b");
            configInstance.SV_LAMDA_b= ocvYamlConfig.value_f("SV_LAMDA_b");

            configInstance.SV_MAX_WRONG_POINT=std::lround(ocvYamlConfig.value_f("SV_MAX_WRONG_POINT"));
            configInstance.SV_MAX_ITERATIONS=std::lround(ocvYamlConfig.value_f("SV_MAX_ITERATIONS"));
            configInstance.SV_E_TOO_SMALL_SIZE= ocvYamlConfig.value_f("SV_E_TOO_SMALL_SIZE");
            configInstance.SV_E_OK= ocvYamlConfig.value_f("SV_E_OK");


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
        std::ifstream _DISTORTIONS=std::ifstream(configInstance().DISTORTIONS);
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