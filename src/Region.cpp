//
// Created by flamming on 2018/1/14.
//

#include "Region.h"
#include <vector>
#include <iostream>
#include <set>


using namespace std;
using namespace cv;

struct cmp {
    bool operator()(const Point2i &a, const Point2i &b) {
        if (a.y != b.y)
            return a.y < b.y;
        return a.x < b.x;
    }
};
// get sample set in circle with given radius
void BresenhamCircle(Point2i veterx,int radius,vector<Point2i>& sampleset)
{

    int c_x = 0;
    int c_y = radius;
    double d = 1 - radius;

    set<Point2i,cmp> point_set;

    while(c_x<c_y)
    {
        point_set.insert({c_x,c_y});
        point_set.insert({c_y,c_x});
        point_set.insert({-c_x,c_y});
        point_set.insert({-c_y,c_x});

        point_set.insert({-c_x,-c_y});
        point_set.insert({-c_y,-c_x});
        point_set.insert({c_x,-c_y});
        point_set.insert({c_y,-c_x});

        if(d<0)
        {
            d = d+2*c_x+3;
        }
        else
        {
            d=d+2*(c_x-c_y)+5;
            c_y--;
        }
        c_x++;
    }


    for(auto& p:point_set)
    {
        sampleset.emplace_back(veterx.x + p.x,veterx.y + p.y);
    }

}


//void UpdateHistorgram(const Frame& curFrame,vector<cv::Point2i> sampleVertices)
//{
//    vector<Mat> channels;
//    split(curFrame.img,channels);//分离色彩通道
//
//    for (auto v:sampleVertices) {
//        vector<Point2i> sampleset;
//        BresenhamCircle(v,5,sampleset);
//        for(auto p:sampleset)
//        {
//            if(curFrame.segmentation.at<unsigned char>(p))
//            {
//                fwd.B[channels[0].at<unsigned char>(p)]++;
//                fwd.G[channels[1].at<unsigned char>(p)]++;
//                fwd.R[channels[2].at<unsigned char>(p)]++;
//            }
//            else
//            {
//                bg.B[channels[0].at<unsigned char>(p)]++;
//                bg.G[channels[1].at<unsigned char>(p)]++;
//                bg.R[channels[2].at<unsigned char>(p)]++;
//            }
//        }
//    }
//}



void Region::VizHistImg(const Histogram& img)
{
    int mval = 0;
    for (int j = 0; j < 255; ++j) {
        mval = mval > img.G[j]? mval : img.G[j];
    }
    Mat histImg1 = Mat::zeros(256,400,CV_8U);
    int height = 299;
    for (int i = 0; i < 255; ++i) {
        line(histImg1,{i,height},{i,height-img.G[i]*height/mval},{255,255,255});
    }
    imshow("hist",histImg1);
    waitKey(0);
}

Region::Region(cv::Point center,int r) {
    center_ = center;
    radius_ = r;
    BresenhamCircle(center,r,circle_bound_);
}

void Region::UpdateHistorgram(FramePtr curFrame)
{
    Mat img = curFrame->img;
    fill(fwd.B,fwd.B+255,1);
    fill(fwd.G,fwd.G+255,1);
    fill(fwd.R,fwd.R+255,1);
    fill(bg.B,bg.B+255,1);
    fill(bg.G,bg.G+255,1);
    fill(bg.R,bg.R+255,1);
    int i = 1;

    for (; i < circle_bound_.size()-1; i++) {
        int left = circle_bound_[i].x;
        while(circle_bound_[i+1].y == circle_bound_[i].y)
            i++;
        auto y = circle_bound_[i].y;
        for (int x = left; x <= circle_bound_[i].x; ++x)
        {
            if(curFrame->segmentation.at<unsigned char>(y,x))
            {
                n_fw++;
                fwd.B[img.at<Vec3b>(y,x)[0]]++;
                fwd.G[img.at<Vec3b>(y,x)[1]]++;
                fwd.R[img.at<Vec3b>(y,x)[2]]++;
            }
            else
            {
                n_bg++;
                bg.B[img.at<Vec3b>(y,x)[0]]++;
                bg.G[img.at<Vec3b>(y,x)[1]]++;
                bg.R[img.at<Vec3b>(y,x)[2]]++;
            }
        }
    }
    aera = n_fw+n_bg;
    n_fw/=aera;
    n_bg/=aera;
}

void Region::UpdateHistorgram(Frame* curFrame)
{
    Mat img = curFrame->img;
    fill(fwd.B,fwd.B+255,1);
    fill(fwd.G,fwd.G+255,1);
    fill(fwd.R,fwd.R+255,1);
    fill(bg.B,bg.B+255,1);
    fill(bg.G,bg.G+255,1);
    fill(bg.R,bg.R+255,1);




    for (int i = 0; i < circle_bound_.size()-1; i++) {
        int left = circle_bound_[i].x;
        while(circle_bound_[i+1].y == circle_bound_[i].y)
            i++;
        auto y = circle_bound_[i].y;
        for (int x = left; x <= circle_bound_[i].x; ++x)
        {
            if(curFrame->segmentation.at<unsigned char>(y,x))
            {
                n_fw++;
                fwd.B[img.at<Vec3b>(y,x)[0]]++;
                fwd.G[img.at<Vec3b>(y,x)[1]]++;
                fwd.R[img.at<Vec3b>(y,x)[2]]++;
            }
            else
            {
                n_bg++;
                bg.B[img.at<Vec3b>(y,x)[0]]++;
                bg.G[img.at<Vec3b>(y,x)[1]]++;
                bg.R[img.at<Vec3b>(y,x)[2]]++;
            }
        }
    }
    aera = n_fw+n_bg;
//    prior_fw/=aera;
//    n_bg/=aera;
}
