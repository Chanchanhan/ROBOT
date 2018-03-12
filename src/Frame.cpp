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
void Frame::Segment()
{
    vector<vector<Point> > contours;
    contours.push_back(this->contourX2D);
    this->segmentation = Mat::zeros(this->img.rows,this->img.cols,CV_8U);
    this->bound_map = Mat::zeros(this->img.rows,this->img.cols,CV_32F);
    cv::drawContours(this->segmentation, contours,-1, CV_RGB(255, 255, 255), CV_FILLED);
    cv::drawContours(this->bound_map, contours,-1, CV_RGB(255, 255, 255),CV_FILLED);
}
void Frame::Segment(const cv::Mat &inPutImg,const std::vector<cv::Point> &contourX2D,cv::Mat &segmentation,cv::Mat &bound_map){
    vector<vector<Point>> contours;
    contours.push_back(contourX2D);
    segmentation = Mat::zeros(inPutImg.rows,inPutImg.cols,CV_8U);
    bound_map = Mat::zeros(inPutImg.rows,inPutImg.cols,CV_32F);
    cv::drawContours(segmentation, contours,-1, Scalar(255), CV_FILLED);
    cv::drawContours(bound_map, contours,-1, Scalar(255),CV_FILLED);
  //  bound_map/=255;

//    cv::cvtColor( bound_map, bound_map, cv::COLOR_RGB2GRAY );
//    cv::blur( bound_map, bound_map, cv::Size(3,3) );
//    cv::Canny( bound_map, bound_map, 20, 60, 3 );
//    imshow("bound_map",bound_map);
}

void Frame::GetPyraid(const int &nPyraid) {
    imgPyramid.resize(nPyraid);
    Config &config= Config::configInstance();
    imgPyramid[0]=img.clone();
    for(int i =1;i<nPyraid;i++){
        cv::Mat dst;
        pyrDown(imgPyramid[i-1], dst,  Size(imgPyramid[i-1].cols*0.5,imgPyramid[i-1].rows*0.5) );
        imgPyramid[i]=dst.clone();
    }
}

void Frame::DTMap() {

    vector<float> weights(2,Config::configInstance().IMG_DT_WEIGHT);
    cv::Mat dt1,dt2;
    int k=100;
    this->bound_map*=k;
    distanceTransform(this->bound_map,dt1,this->dtLocation,weights);/// dt1 is outside
    distanceTransform(k*255-this->bound_map,dt2,this->dtLocation,weights);///dt2 is insied
    normalize(dt1,dt1,1.0,0.0,NORM_MINMAX);
    normalize(dt2,dt2,1.0,0.0,NORM_MINMAX);

    //LOG(INFO)<<"bound_map"<<this->bound_map;
   // LOG(INFO)<<"dt"<<this->dt;
    this->dt= dt1-dt2;
   // imshow("dt",this->dt);


}
void Frame::DTMap(const cv::Mat &inPut,cv::Mat &dt,cv::Mat &dtLocation) {
    vector<float> weights(2,Config::configInstance().IMG_DT_WEIGHT);
    //distanceTransform(this->bound_map,dt,dtLocation,weights);/// it's wired
    distanceTransform(inPut,dt,dtLocation,weights);/// it's wired

    //  imshow("dt",dt);
}
cv::Point Frame::nearstContourP(const cv::Point &point) {
    int x= point.y;
    int y= point.x;

    int *_locations=(int *)dtLocation.data;
    Config &gConfig = Config::configInstance();

    while(_locations[y + gConfig.VIDEO_WIDTH * x]!=x||_locations[gConfig.VIDEO_HEIGHT *gConfig.VIDEO_WIDTH+y + gConfig.VIDEO_WIDTH * x]!=y){
        x=_locations[y + gConfig.VIDEO_WIDTH * x];
        y=_locations[gConfig.VIDEO_HEIGHT *gConfig.VIDEO_WIDTH+y + gConfig.VIDEO_WIDTH* x];
    }
    return Point(y,x);
}

void Frame::ComputePosterior(const cv::Mat &inPutImg,const std::vector<Region>& rg,cv::Mat &fw_posterior,cv::Mat &bg_posterior)
{
    cv::Mat fw_posterior_tmp = Mat::zeros(inPutImg.size(),CV_64F) ;
    cv::Mat bg_posterior_tmp = Mat::zeros(inPutImg.size(),CV_64F) ;
    Mat occur = Mat::zeros(inPutImg.size(),CV_16S);
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
                auto& p = inPutImg.at<Vec3b>(y,x);
                occur.at<short>(y,x)++;
                auto ar = s.aera;
                auto fw_likelihood = (s.fwd.B[p[0]]/s.n_fw
                                      * s.fwd.G[p[1]]/s.n_fw
                                      *s.fwd.R[p[2]]/s.n_fw);
                auto bg_likelihood = (s.bg.B[p[0]]/s.n_bg
                                      * s.bg.G[p[1]]/s.n_bg
                                      *s.bg.R[p[2]]/s.n_bg );
                auto btm = s.n_fw/ar * fw_likelihood + s.n_bg/ar * bg_likelihood;
                fw_posterior_tmp.at<double>(y,x) += fw_likelihood / btm;
                bg_posterior_tmp.at<double>(y,x) += bg_likelihood / btm ;
            }
        }
        overall_fw = overall_fw + s.fwd;
        overall_bg = overall_bg + s.bg;
        fw_pixels += s.n_fw;
        bg_pixels += s.n_bg;
    }
    auto p1 = occur.ptr<short>(0);
    auto f = fw_posterior_tmp.ptr<double>(0);
    auto b = bg_posterior_tmp.ptr<double>(0);
    auto imgp = inPutImg.ptr<Vec3b>(0);
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
    fw_posteriorPyramid.push_back(fw_posterior_tmp);
    bg_posteriorPyramid.push_back(bg_posterior_tmp);

}


void Frame::ComputePosterior(const std::vector<Region>& rg)
{
    fw_posterior = Mat::zeros(img.size(),CV_64F) ;
    bg_posterior = Mat::zeros(img.size(),CV_64F) ;
    Mat occur = Mat::zeros(img.size(),CV_16S);
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
                auto& p = img.at<Vec3b>(y,x);
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
    auto imgp = img.ptr<Vec3b>(0);
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


