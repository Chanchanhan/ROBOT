#include "histogram.h"
#include "bresenhams_circle.h"

using namespace std;
using namespace cv;
void UpdatingHistorgram(const Frame& curFrame,vector<cv::Point2i> sampleVertices,Histogram& foreth,Histogram& bg)
{
    vector<Mat> channels;
    split(curFrame.img,channels);//分离色彩通道

    for (auto v:sampleVertices) {
        vector<Point2i> sampleset;
        BresenhamCircle(v,5,sampleset);
        for(auto p:sampleset)
        {
            if(curFrame.segmentation.at<unsigned char>(p))
            {
                foreth.B[channels[0].at<unsigned char>(p)]++;
                foreth.G[channels[1].at<unsigned char>(p)]++;
                foreth.R[channels[2].at<unsigned char>(p)]++;
            }
            else
            {
                bg.B[channels[0].at<unsigned char>(p)]++;
                bg.G[channels[1].at<unsigned char>(p)]++;
                bg.R[channels[2].at<unsigned char>(p)]++;
            }
        }
    }
}

Mat VizHistImg(const Histogram& img)
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
    return {};
}
