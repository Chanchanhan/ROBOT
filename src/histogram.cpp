#include "histogram.h"
#include "bresenhams_circle.h"

using namespace std;
using namespace cv;
void UpdatingHistorgram(const Frame& curFrame,vector<cv::Point2d> sampleVertices,Histogram& foreth,Histogram& bg)
{
//     vector<Mat> channels;
//     split(curFrame.img,channels);//分离色彩通道
// 
//     for (auto v:sampleVertices) {
//         vector<Point2i> sampleset;
//         BresenhamCircle(v,5,sampleset);
//         for(auto p:sampleset)
//         {
//             if(curFrame.segmentation.at<unsigned char>(p))
//             {
//                 foreth.B[channels[0].at(p)]++;
//                 foreth.G[channels[1].at(p)]++;
//                 foreth.R[channels[2].at(p)]++;
//             }
//             else
//             {
//                 bg.B[channels[0].at(p)]++;
//                 bg.G[channels[1].at(p)]++;
//                 bg.R[channels[2].at(p)]++;
//             }
//         }
//     }
}


