#include <vector>
#include "frame.h"

struct Histogram
{
    int R[255]{};
    int G[255]{};
    int B[255]{};
};

void UpdatingHistorgram(const Frame& curFrame,std::vector<cv::Point2d> sampleVertices,Histogram& foreth,Histogram& bg);
//UpdatingHistorgramYUV(Frame curFrame,VerticesNearToContour);
