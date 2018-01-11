#include <iostream>
#include "bresenhams_circle.h"

using namespace std;
using namespace cv;
int main()
{
    vector<cv::Point2i> region;
    BresenhamCircle({150,150},55,region);
    Mat draw = Mat::zeros(300,300,CV_8U);
    for(auto& p : region)
        draw.at<unsigned char>(p.x,p.y) = 255;

    imshow("result",draw);
    waitKey(0);

    cout << "ROBOT:region based glm tracking" << endl;
	return 0;
}

