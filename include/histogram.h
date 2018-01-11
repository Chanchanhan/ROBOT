#include <vector>
#include "frame.h"

struct Histogram
{
    int R[255]{};
    int G[255]{};
    int B[255]{};
};

void UpdatingHistorgram(const Frame& curFrame,std::vector<cv::Point2i> sampleVertices,Histogram& foreth,Histogram& bg);
cv::Mat VizHistImg(const Histogram& img);


//UpdatingHistorgramYUV(Frame curFrame,VerticesNearToContour);
