//
// Created by flamming on 2018/1/12.
//
#include <Frame.h>
#include <Region.h>

#include "GlobalConfig.h"
#include "DT.h"

#include "Region.h"

using namespace cv;
using namespace std;

void Frame::Segment(const cv::Mat &inPutImg,const std::vector<cv::Point> &contourX2D,cv::Mat &segmentation,cv::Mat &bound_map){
    vector<vector<Point>> contours;
    contours.push_back(contourX2D);
    segmentation = Mat::zeros(inPutImg.rows,inPutImg.cols,CV_8U);
    bound_map = Mat::zeros(inPutImg.rows,inPutImg.cols,CV_32FC1);
    cv::drawContours(segmentation, contours,-1, Scalar(255), CV_FILLED);
    cv::drawContours(bound_map, contours,-1, Scalar(255),CV_FILLED);
}

void Frame::GetPyraid(const int &nPyraid) {
    imgPyramid.resize(nPyraid);
    Config &config= Config::ConfigInstance();
    imgPyramid[0]=img.clone();
    for(int i =1;i<nPyraid;i++){
        cv::Mat dst;
        pyrDown(imgPyramid[i-1], dst,  Size(imgPyramid[i-1].cols*0.5,imgPyramid[i-1].rows*0.5) );
        imgPyramid[i]=dst.clone();
    }
}

void Frame::UpdateDTMap() {

    vector<float> weights(2, Config::ConfigInstance().IMG_DT_WEIGHT);
    cv::Mat dt1(this->bound_map.size(),CV_32FC1),dt2(this->bound_map.size(),CV_32FC1);
    this->dt= cv::Mat(this->bound_map.size(),CV_32FC1);
//    cv::distanceTransform(this->bound_map,dt1,CV_DIST_L2,3);//inside
//    cv::distanceTransform(255-this->bound_map,dt2,CV_DIST_L2,3);//outside

    distanceTransform(bound_map,dt1,dtLocationInside,weights);
    distanceTransform(255-bound_map,dt2,dtLocationOutside,weights);
    this->dt= dt2-dt1;


}

void Frame::UpdateDTMap(const std::vector<cv::Point> &contourX2D) {
    if(contourX2D.empty())
        return;
    vector<float> weights(2, Config::ConfigInstance().IMG_DT_WEIGHT);

    vector<vector<Point>> contours;
    contours.push_back(contourX2D);
    cv::drawContours(bound_map, contours,-1, Scalar(255), CV_FILLED);

    cv::Mat dt1(this->bound_map.size(),CV_32FC1),dt2(this->bound_map.size(),CV_32FC1);
    this->dt= cv::Mat(this->bound_map.size(),CV_32FC1);
    distanceTransform(bound_map,dt1,dtLocationInside,weights);
    distanceTransform(255-bound_map,dt2,dtLocationOutside,weights);

//    cv::distanceTransform(this->bound_map,dt1,CV_DIST_L2,3);//inside
//    cv::distanceTransform(255-this->bound_map,dt2,CV_DIST_L2,3);//outside


    this->dt= dt2-dt1;
   // cout<<"dt"<<dt<<endl;

}
cv::Point Frame::GetNearstContourP(const cv::Point &point, const cv::Mat &dtLocation,const int iLevel) {
    int x= point.y;
    int y= point.x;

    int *_locations=(int *)dtLocation.data;
    Config &g_Config = Config::ConfigInstance();
    int weight = g_Config.VIDEO_WIDTH /std:: pow(2,iLevel),
            height = g_Config.VIDEO_HEIGHT /std:: pow(2,iLevel);
    while(_locations[y + weight * x]!=x||_locations[height *weight+y + weight * x]!=y){
        x=_locations[y + weight * x];
        y=_locations[height *weight+y + weight* x];
    }
    return Point(y,x);
}



void Frame::ComputePosterior(const Mat &inputImg,const std::vector<Region>& rg)
{



    fw_posterior = Mat::zeros(inputImg.size(),CV_64F) ;
    bg_posterior = Mat::zeros(inputImg.size(),CV_64F) ;
#ifdef  PROJECT_WITH_GT

    for(int i=0;i<segmentation.rows;i++){
        for (int j=0;j<segmentation.cols;j++){
            fw_posterior.at<double>(i,j) = segmentation.at<unsigned char>(i,j)>0?1:0;
        }
    }

    bg_posterior=1-fw_posterior;
    return;
#endif

    Mat occur = Mat::zeros(inputImg.size(),CV_16S);
    Histogram overall_fw;
    Histogram overall_bg;
    double fw_pixels = 0;
    double bg_pixels = 0;

    for(auto s:rg)
    {
        s.UpdateHistorgram(this);
        for (int i = 0; i < s.circle_bound_.size(); i++) {
            int left = s.circle_bound_[i].x;
            while(s.circle_bound_[i+1].y == s.circle_bound_[i].y)
                i++;
            auto y = s.circle_bound_[i].y;
            for (int x = left; x < s.circle_bound_[i].x; ++x) {
                auto& p = inputImg.at<Vec3b>(y,x);
                occur.at<short>(y,x)++;
                auto ar = s.aera;
                auto fw_likelihood = (s.fwd.B[p[0]]/s.n_fw
                                      * s.fwd.G[p[1]]/s.n_fw
                                      *s.fwd.R[p[2]]/s.n_fw);
                auto bg_likelihood = (s.bg.B[p[0]]/s.n_bg
                                      * s.bg.G[p[1]]/s.n_bg
                                      *s.bg.R[p[2]]/s.n_bg );
                auto btm = s.n_fw/ar * fw_likelihood + s.n_bg/ar * bg_likelihood;
                fw_posterior.at<double>(y,x) += fw_likelihood / btm;
                bg_posterior.at<double>(y,x) += bg_likelihood / btm ;
            }
        }
        overall_fw = overall_fw + s.fwd;
        overall_bg = overall_bg + s.bg;
        fw_pixels += s.n_fw;
        bg_pixels += s.n_bg;
    }
    auto p1 = occur.ptr<short>(0);
    auto f = fw_posterior.ptr<double>(0);
    auto b = bg_posterior.ptr<double>(0);
    auto imgp = inputImg.ptr<Vec3b>(0);
    auto oc = occur.ptr<short>(0);
    double temp;
    auto all_pixel = fw_pixels + bg_pixels;
    cout<<bg_pixels<<endl;
    for (int j = 0; j < occur.size().area(); ++j) {
        if(*oc==0)
        {
            *f = (overall_fw.B[(*imgp)[0]]/fw_pixels *
                  overall_fw.G[(*imgp)[1]]/fw_pixels *
                  overall_fw.R[(*imgp)[2]]/fw_pixels);
            *b = (overall_bg.B[(*imgp)[0]]/bg_pixels *
                  overall_bg.G[(*imgp)[1]]/bg_pixels *
                  overall_bg.R[(*imgp)[2]]/bg_pixels);
//            auto btm = fw_pixels/all_pixel * (*f) + bg_pixels/all_pixel * (*b);
//            *f/=btm;
//            *b/=btm;
        }
        else
        {
            *f /= *p1;
            *b /= *p1;
        }
        temp = *f + *b;
        *f = *f/temp;
        *b = *b/temp;
        p1++;
        f++;
        b++;
        imgp++;
        oc++;
    }
}


