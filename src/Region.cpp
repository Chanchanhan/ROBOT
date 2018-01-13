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


//void UpdatingHistorgram(const Frame& curFrame,vector<cv::Point2i> sampleVertices)
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

Region::Region(int r, cv::Point center) {
    center_ = center;
    radius_ = r;
    BresenhamCircle(center,r,circle_bound_);
}

void Region::UpdatingHistorgram(const Frame& curFrame)
{
    int y_max = circle_bound_.back();

    vector<Mat> channels;
    split(curFrame.img,channels);//分离色彩通道

    int i = 1;
    if(circle_bound_[0].y==circle_bound_[1].y)
        i = 0;
    for (; i < channels.size()-1; i+=2) {
        auto y = circle_bound_[i].y;
        for (int x = circle_bound_[i].x; x < circle_bound_[i+1].x; ++x) {
            if(curFrame.segmentation.at<unsigned char>())
            {
                fwd.B[channels[0].at<unsigned char>(x,y)]++;
                fwd.G[channels[1].at<unsigned char>(x,y)]++;
                fwd.R[channels[2].at<unsigned char>(x,y)]++;
            }
            else
            {
                bg.B[channels[0].at<unsigned char>(x,y)]++;
                bg.G[channels[1].at<unsigned char>(x,y)]++;
                bg.R[channels[2].at<unsigned char>(x,y)]++;
            }
        }
    }
}
