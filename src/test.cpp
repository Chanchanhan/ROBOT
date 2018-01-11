#include <iostream>
#include "bresenhams_circle.h"
#include "histogram.h"

using namespace std;
using namespace cv;
int main()
{
    vector<cv::Point2i> region;
    BresenhamCircle({150,150},55,region);
    Mat draw = imread("/Users/flamming/Downloads/MarkerAR550/Assets/Resources/marker3.jpg");
//    for(auto& p : region)
//    {
//        draw.at<Vec3b>(p.x,p.y)[0] = 0;
//        draw.at<Vec3b>(p.x,p.y)[1] = 255;
//        draw.at<Vec3b>(p.x,p.y)[2] = 0;
//    }
    Frame f;
    f.img = draw;
    f.segmentation = Mat::zeros(draw.rows,draw.cols,CV_8U);
    Histogram fore,bg;
    UpdatingHistorgram(f,region,fore,bg);
    imshow("result",draw);

    VizHistImg(bg);



    cout << "ROBOT:region based object tracking" << endl;
	return 0;
}

