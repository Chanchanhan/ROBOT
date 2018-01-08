#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

void BresenhamCircle(Point2d veterx,int radius)
{
    int c_x = 0;
    int c_y = radius;
    double d = 1 - radius;

    vector<Point2d> point_set;

    while(c_x<c_y)
    {
        point_set.emplace_back(c_x,c_y);
        point_set.emplace_back(c_y,c_x);
        point_set.emplace_back(-c_x,c_y);
        point_set.emplace_back(-c_y,c_x);

        point_set.emplace_back(-c_x,-c_y);
        point_set.emplace_back(-c_y,-c_x);
        point_set.emplace_back(c_x,-c_y);
        point_set.emplace_back(c_y,-c_x);

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


    Mat draw = Mat::zeros(300,300,CV_8U);
    for(auto& p:point_set)
    {
        draw.at<unsigned char>(veterx.x + p.x,veterx.y + p.y) = 255;
    }
    imshow("result",draw);
    waitKey(0);
    cout<<point_set<<endl;

}


