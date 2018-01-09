#include <vector>
#include <iostream>
#include <set>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

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


