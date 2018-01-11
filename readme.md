## ROBOT:RegiOn Based Object Tracking

### DataSet:

### Algorithm:

1. Projecting vertices to image plane
2. Finding contour of the projected in current frame.
3. Segment the current Frame
3. Sample vertices near to contour.
4. ~~Statistic the histogram of the region around vertices~~
5. Compute posterior and draw statistic img.
6. Compute and optimize the enery fuction.

### Libs:
- Eigen
- Sophus: pose processing
- Opencv: img processing
- glog
 
### Interfaces:
```
1. LoadImg(File *file,Type type); 
2. GettingContour(Model model,Frame frame); 
3. GettingVerticesNearToContour(Model model,Contour contour); 
4. UpdatingHistorgram(Frame curFrame,VerticesNearToContour) 
5. ComputePosterior(Histogram historgram);
6. ....(Energy Optimization)
```

### Datastructure
```
class Frame
{ 
    cv::Mat *img; 
    SE3D dpose; 
    SE3D grondTruthPose; 
    unsigned int index;
}
```
