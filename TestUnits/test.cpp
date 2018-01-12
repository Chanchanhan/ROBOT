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
    Mat out = draw.clone();
    for(auto& p : region)
    {
        out.at<Vec3b>(p.x,p.y)[0] = 0;
        out.at<Vec3b>(p.x,p.y)[1] = 255;
        out.at<Vec3b>(p.x,p.y)[2] = 0;
    }
    Frame f;
    f.img = draw;
    f.segmentation = Mat::zeros(draw.rows,draw.cols,CV_8U);

    drawContours(f.segmentation,region,0,cv::Scalar(255,255,255),1,CV_FILLED);
    Histogram fore,bg;
    UpdatingHistorgram(f,region,fore,bg);
    imshow("result",draw);
    imshow("result",f.segmentation);

    VizHistImg(bg);



    cout << "ROBOT:region based glm tracking" << endl;
	return 0;
}

