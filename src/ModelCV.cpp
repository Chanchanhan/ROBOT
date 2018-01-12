//
// Created by qqh on 18-1-12.
//
#include "Pose.h"
#include "Model.h"
#include "GlobalConfig.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
void Model::getContourPointsAndIts3DPoints(Pose &pose, std::vector<cv::Point3d> &verticesContour_Xs,
                                           std::vector<cv::Point2d> &verticesContour_xs,std::vector<cv::Point> &resContour) {
    cv::Mat visible_Xs,visible_xs;
    getVisualableVertices(pose,visible_Xs);
    project3D_2D(pose, visible_Xs, visible_xs);
    Config &g_Config = Config::configInstance();
    cv::Mat img1=cv::Mat::zeros(g_Config.VIDEO_HEIGHT, g_Config.VIDEO_WIDTH, CV_32SC1);
    for (int i = 0; i < visible_xs.cols; ++i) {
        cv::Point pt(visible_xs.at<float>(0, i), visible_xs.at<float>(1, i));
        //     LOG(INFO)<<i + 1<<" pt : "<<pt;
        if (pointInFrame(pt)){
            img1.at<int>(pt) = i + 1;
        }
    }

    std::vector<std::vector<cv::Point> > contours;

    cv::Mat line_img = cv::Mat::zeros(g_Config.VIDEO_HEIGHT, g_Config.VIDEO_WIDTH , CV_8UC1);
    displayCV(pose, cv::Scalar(255, 255, 255),line_img);
    cv::findContours(line_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

   cv::Mat mask_img = cv::Mat::zeros(Config::configInstance().VIDEO_HEIGHT,Config::configInstance().VIDEO_WIDTH, CV_8UC1);
   cv::drawContours(mask_img, contours, -1, CV_RGB(255, 255, 255), CV_FILLED);
   // cv::imshow("line_img",line_img);

    cv::imshow("mask_img",mask_img);
    cv::waitKey(1);
    /***to map X-x***/
    if(contours.size()==0){
        return;
    }
    std::vector<cv::Point> &contour=contours[0];
    resContour=contours[0];
    int near[9][2]={{0,0},{0,-1},{0,1},{-1,0},{1,0},{1,1},{1,-1},{-1,1},{-1,-1}};
    cv::Mat extrinsic(4, 4, CV_32FC1);
    pose.getExtrinsicMat(extrinsic);
    for (int i = 0; i < contour.size(); ++i){
        for(int j=0;j<9;j++){
            int value = img1.at<int>(contour[i].y+near[j][0],contour[i].x+near[j][1]);
            if (value > 0) {
//              img1.at<int>(contour[i].y+near[j][0],contour[i].x+near[j][1])=0;
                cv::Point3d pt3d( visible_Xs.at<float>(0, value - 1), visible_Xs.at<float>(1, value - 1),visible_Xs.at<float>(2, value - 1));
                verticesContour_Xs.push_back(pt3d);
                verticesContour_xs.push_back(X_to_x(pt3d,extrinsic));
                break;
            }
        }
    }
    LOG(INFO)<<"verticesContour_xs.size() : "<<verticesContour_xs.size();

}

cv::Point Model::X_to_x(const cv::Point3f &X,const cv::Mat &extrisic)
{
  cv::Mat P(4,1,CV_32FC1);
  cv::Mat res(3,1,CV_32FC1);
  P.at<float>(0,0)=X.x;
  P.at<float>(1,0)=X.y;
  P.at<float>(2,0)=X.z;
  P.at<float>(3,0)=1;
  res=intrinsic*extrisic*P;
  
  return cv::Point(res.at<float>(0,0)/res.at<float>(2,0),res.at<float>(1,0)/res.at<float>(2,0));
  
}


void Model::displayCV( Pose &pose,const cv::Scalar &color, cv::Mat& frame)
{
	cv::Mat visualable_model_points;
	getVisualableVertices(pose, visualable_model_points);

	cv::Mat image_points;
	project3D_2D(pose, visualable_model_points, image_points);

	int size = image_points.cols/2;
	for (int i = 0; i < size; ++i) {
		cv::Point pt1(image_points.at<float>(0, 2*i), image_points.at<float>(1, 2*i));
		cv::Point pt2(image_points.at<float>(0, 2*i+1), image_points.at<float>(1, 2*i+1));
			
		if (pointInFrame(pt1) && pointInFrame(pt2)) {
			cv::line(frame, pt1, pt2, color);
		}
	}
}
bool Model::pointInFrame(const cv::Point &p){
 Config &gConfig = Config::configInstance();
 return(p.x>=0&&p.y>=0&&p.x<gConfig.VIDEO_WIDTH&&p.y<gConfig.VIDEO_HEIGHT);
}
void Model::getVisualableVertices(Pose &pose, cv::Mat& vis_vertices) {
    cv::Mat pt_in_cam(3, VerticesCount()+1, CV_32FC1);

    cv::Mat extrinsic(4, 4, CV_32FC1);
    pose.getExtrinsicMat(extrinsic);
//    LOG(INFO)<<"extrinsic: "<<extrinsic;
    pt_in_cam = extrinsic * vertices_hom_;

    float u[3], v[3], n[3], c[3];
    for (size_t i = 0; i < model_->numtriangles; i++) {
        //compute the norm of the triangles
        u[0] = pt_in_cam.at<float>(0, model_->triangles[i].vindices[1]) - pt_in_cam.at<float>(0, model_->triangles[i].vindices[0]);
        u[1] = pt_in_cam.at<float>(1, model_->triangles[i].vindices[1]) - pt_in_cam.at<float>(1, model_->triangles[i].vindices[0]);
        u[2] = pt_in_cam.at<float>(2, model_->triangles[i].vindices[1]) - pt_in_cam.at<float>(2, model_->triangles[i].vindices[0]);

        v[0] = pt_in_cam.at<float>(0, model_->triangles[i].vindices[2]) - pt_in_cam.at<float>(0, model_->triangles[i].vindices[0]);
        v[1] = pt_in_cam.at<float>(1, model_->triangles[i].vindices[2]) - pt_in_cam.at<float>(1, model_->triangles[i].vindices[0]);
        v[2] = pt_in_cam.at<float>(2, model_->triangles[i].vindices[2]) - pt_in_cam.at<float>(2, model_->triangles[i].vindices[0]);

        Cross(u, v, n);
        Normalize(n);

        //center of triangle
        c[0] = 0.25f*pt_in_cam.at<float>(0, model_->triangles[i].vindices[0]) + 0.25f*pt_in_cam.at<float>(0, model_->triangles[i].vindices[1]) + 0.5f*pt_in_cam.at<float>(0, model_->triangles[i].vindices[2]);
        c[1] = 0.25f*pt_in_cam.at<float>(1, model_->triangles[i].vindices[0]) + 0.25f*pt_in_cam.at<float>(1, model_->triangles[i].vindices[1]) + 0.5f*pt_in_cam.at<float>(1, model_->triangles[i].vindices[2]);
        c[2] = 0.25f*pt_in_cam.at<float>(2, model_->triangles[i].vindices[0]) + 0.25f*pt_in_cam.at<float>(2, model_->triangles[i].vindices[1]) + 0.5f*pt_in_cam.at<float>(2, model_->triangles[i].vindices[2]);

        Normalize(c);

        //judge the whether the line is visible or not
        float cross = n[0] * c[0] + n[1] * c[1] + n[2] * c[2];
        if (cross < 0.0f) {
            if (model_->lines[model_->triangles[i].lindices[0]].e1 == 1)
                model_->lines[model_->triangles[i].lindices[0]].e2 = 1;
            else
                model_->lines[model_->triangles[i].lindices[0]].e1 = 1;

            if (model_->lines[model_->triangles[i].lindices[1]].e1 == 1)
                model_->lines[model_->triangles[i].lindices[1]].e2 = 1;
            else
                model_->lines[model_->triangles[i].lindices[1]].e1 = 1;

            if (model_->lines[model_->triangles[i].lindices[2]].e1 == 1)
                model_->lines[model_->triangles[i].lindices[2]].e2 = 1;
            else
                model_->lines[model_->triangles[i].lindices[2]].e1 = 1;


        }
    }

    int visualable_line_count = 0;
    for (size_t i = 0; i<model_->numLines; i++) {
        if ((model_->lines[i].e1 == 1 && model_->lines[i].e2 == 0) || (model_->lines[i].e1 == 0 && model_->lines[i].e2 == 1) || (model_->lines[i].e1 == 1 && model_->lines[i].e2 == 1)) {
            visualable_line_count++;
        }
    }

    cv::Mat pos(4, visualable_line_count*2, CV_32FC1);
    int vis_line_index = 0;
    for (size_t i = 0; i<model_->numLines; i++) {
        if ((model_->lines[i].e1 == 1 && model_->lines[i].e2 == 0) || (model_->lines[i].e1 == 0 && model_->lines[i].e2 == 1) || (model_->lines[i].e1 == 1 && model_->lines[i].e2 == 1)) {
            GLuint v0 = model_->lines[i].vindices[0];
            GLuint v1 = model_->lines[i].vindices[1];

            pos.at<float>(0, 2*vis_line_index) = model_->vertices[3 * v0];
            pos.at<float>(1, 2*vis_line_index) = model_->vertices[3 * v0 + 1];
            pos.at<float>(2, 2*vis_line_index) = model_->vertices[3 * v0 + 2];
            pos.at<float>(3, 2*vis_line_index) = 1;

            pos.at<float>(0, 2*vis_line_index+1) = model_->vertices[3 * v1];
            pos.at<float>(1, 2*vis_line_index+1) = model_->vertices[3 * v1 + 1];
            pos.at<float>(2, 2*vis_line_index+1) = model_->vertices[3 * v1 + 2];
            pos.at<float>(3, 2*vis_line_index+1) = 1;

            vis_line_index++;
        }
        model_->lines[i].e1 = 0; model_->lines[i].e2 = 0;
    }

    vis_vertices = pos;
}


void Model::project3D_2D( Pose &pose, const cv::Mat& visible_Xs,  cv::Mat &visible_xs)
{
    cv::Mat extrinsic(4, 4, CV_32FC1) ;
    pose.getExtrinsicMat(extrinsic);
    visible_xs=intrinsic*extrinsic*visible_Xs;
    for (int i = 0; i < visible_xs.cols; ++i) {
        float dz = 1.0f/visible_xs.at<float>(2, i);
        visible_xs.at<float>(0, i) *= dz;
        visible_xs.at<float>(1, i) *= dz;
    }
}