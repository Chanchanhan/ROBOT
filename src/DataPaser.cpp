
#include<iostream>
#include <string.h>
#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <dirent.h>
#endif
#include <opencv2/imgproc.hpp>

#include "DataPaser.h"
#include "GlobalConfig.h"
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


DataPaser::~DataPaser()
{

}
std::vector<std::string> DataPaser::GetFiles(const std::string path, const string pattern)
{
    std:: vector<std::string> files;//存放文件名

#ifdef WIN32
    _finddata_t file;
    long lf;
    //输入文件夹路径
    if ((lf=_findfirst(path.c_str(), &file)) == -1) {
        std::cout<<path<<" not found!!!"<<endl;
    } else {
        while(_findnext(lf, &file) == 0) {
            //输出文件名
            //cout<<file.name<<endl;
            if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
                continue;
            files.push_back(file.name);
        }
    }
    _findclose(lf);

#else
//#ifdef linux
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(path.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            int size = strlen(ptr->d_name);

            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            if (strcmp((ptr->d_name + (size- pattern.size())), pattern.c_str())!=0)
                continue;
            files.push_back(ptr->d_name);

        }
        else if(ptr->d_type == 10)    ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if(ptr->d_type == 4)    ///dir
        {
            files.push_back(ptr->d_name);
            /*
                memset(base,'\0',sizeof(base));
                strcpy(base,basePath);
                strcat(base,"/");
                strcat(base,ptr->d_nSame);
                readFileList(base);
            */
        }
    }
    closedir(dir);
#endif
    //排序，按从小到大排序
    sort(files.begin(), files.end());
    return files;
}
bool DataPaser::parseAFrame(FramePtr frame,const int frameID) {
    cv::Mat img;
    if(Config::ConfigInstance().USE_VIDEO){
        if (!videoCapture.isOpened())
        {
            LOG(ERROR)<<("Cannot open camera\n");
            return false;
        }
        if (!videoCapture.read(img))
            return false;
    }
    else{
        if(filesName.empty()){
            filesName = GetFiles(Config::ConfigInstance().videoPath, ".jpg");
        }
        auto frameFile=filesName[frameID];
        img = cv::imread(Config::ConfigInstance().videoPath+frameFile);
    }

    float gtPose[6]={0};
    std::string str,temp;
    std::getline(gtData,str);
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


