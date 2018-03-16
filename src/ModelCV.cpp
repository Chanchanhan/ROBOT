//
// Created by qqh on 18-1-12.
//
#include "Pose.h"
#include "Model.h"
#include "GlobalConfig.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace cv;
void Model::GetContour(const Sophus::SE3d &pose,std::vector<cv::Point> &resContour, const int iLevel) {
    resContour.resize(0);
    std::vector<std::vector<cv::Point>> contours;
    int weight = Config::configInstance().VIDEO_WIDTH /std:: pow(2,iLevel),
            height = Config::configInstance().VIDEO_HEIGHT /std:: pow(2,iLevel);
    cv::Mat line_img = cv::Mat::zeros(height,weight, CV_8UC1);
    DisplayCV(pose, cv::Scalar(255, 255, 255), line_img, iLevel);
    cv::findContours(line_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if (contours.size() == 0) {
        return;
    }
    resContour = contours[0];

}

void Model::GetContourPointsAndIts3DPoints(const Sophus::SE3d &pose, std::vector<cv::Point3d> &verticesContour_Xs,
                                           std::vector<cv::Point2d> &verticesContour_xs,
                                           std::vector<cv::Point> &resContour, const int iLevel) {
    verticesContour_Xs.resize(0);
    verticesContour_xs.resize(0);
    resContour.resize(0);
    cv::Mat visible_Xs, visible_xs;
    GetVisualableVertices(pose, visible_Xs);

    Project3D_2D(pose, visible_Xs, visible_xs, iLevel);
    Config &g_Config = Config::configInstance();
    int weight = g_Config.VIDEO_WIDTH /std:: pow(2,iLevel),
        height = g_Config.VIDEO_HEIGHT /std:: pow(2,iLevel);

    cv::Mat img1 = cv::Mat::zeros(height,weight, CV_32SC1);
    for (int i = 0; i < visible_xs.cols; ++i) {
        cv::Point pt(visible_xs.at<float>(0, i), visible_xs.at<float>(1, i));
        //     LOG(INFO)<<i + 1<<" pt : "<<pt;
        if (PointInFrame(pt, iLevel)) {
            img1.at<int>(pt) = i + 1;
        }
    }
    GetContour(pose,resContour,iLevel);

    if(resContour.empty()) return;

   // cv::waitKey(0);
    /***to map X-x***/

    int near[9][2] = {{0,  0},
                      {0,  -1},
                      {0,  1},
                      {-1, 0},
                      {1,  0},
                      {1,  1},
                      {1,  -1},
                      {-1, 1},
                      {-1, -1}};
    cv::Mat extrinsic(4, 4, CV_32FC1);
    extrinsic = Se2cvf(pose);
//    LOG(INFO)<<"pose "<<pose.log();
    for (int i = 0; i < resContour.size(); ++i) {
        for (int j = 0; j < 9; j++) {
            int value = img1.at<int>(resContour[i].y + near[j][0], resContour[i].x + near[j][1]);
            if (value > 0) {
//              img1.at<int>(contour[i].y+near[j][0],contour[i].x+near[j][1])=0;
                cv::Point3d pt3d(visible_Xs.at<float>(0, value - 1), visible_Xs.at<float>(1, value - 1),
                                 visible_Xs.at<float>(2, value - 1));

                cv::Point2d pt2d=X_to_x(pt3d, extrinsic,iLevel);
                if(PointInFrame(pt2d, iLevel)){
                    verticesContour_Xs.push_back(pt3d);
                    verticesContour_xs.push_back(pt2d);
//                    LOG(INFO)<<"pt3d : "<<pt3d;
//
//                    LOG(INFO)<<"pt2d : "<<pt2d;
                }
                break;
            }
        }
    }
    int totalN = verticesContour_xs.size();
    if (totalN <= g_Config.TK_VER_NUMBER) {
        LOG(WARNING)<<"totalN <= g_Config.TK_VER_NUMBER";
        return;
    }
    int base = totalN/g_Config.TK_VER_NUMBER;
    int *randoms = new int[totalN];
    for (int i = 0; i < totalN; i++) {
        randoms[i] = i;
    }
    for (int i = 0; i < totalN; i++) {
        std::swap(randoms[i], randoms[i + rand() % (totalN - i)]);
    }
    std::vector<cv::Point3d> resVerticesContour_Xs(g_Config.TK_VER_NUMBER);
    std::vector<cv::Point2d> resVerticesContour_xs(g_Config.TK_VER_NUMBER);

    for (int i = 0; i < g_Config.TK_VER_NUMBER; i++) {
//        resVerticesContour_Xs[i] = verticesContour_Xs[i*base+randoms[i]%base];
//        resVerticesContour_xs[i] = verticesContour_xs[i*base+randoms[i]%base];
        resVerticesContour_Xs[i] = verticesContour_Xs[randoms[i]];
        resVerticesContour_xs[i] = verticesContour_xs[randoms[i]];
    }
    verticesContour_Xs = resVerticesContour_Xs;
    verticesContour_xs = resVerticesContour_xs;
    delete[] randoms;
//    LOG(INFO)<<"verticesContour_Xs : "<<verticesContour_Xs.size();
}

cv::Point Model::X_to_x(const cv::Point3f &X,const cv::Mat &extrisic,const int &iLevel)
{
    cv::Mat P(4,1,CV_32FC1);
    cv::Mat res(3,1,CV_32FC1);
    P.at<float>(0,0)=X.x;
    P.at<float>(1,0)=X.y;
    P.at<float>(2,0)=X.z;
    P.at<float>(3,0)=1;
    res=intrinsics[iLevel]*extrisic*P;

    return cv::Point(res.at<float>(0,0)/res.at<float>(2,0),res.at<float>(1,0)/res.at<float>(2,0));

}
void Model::DrawPoints(const Sophus::SE3d &pose, const std::vector<cv::Point3d> &Xs, cv::Mat &frame,
                       const int iLevel) {
    cv::Mat extrinsic(4, 4, CV_32FC1);
    extrinsic = Se2cvf(pose);
    cv::Mat out = frame.clone();

    for(int i=0;i<Xs.size();i++){
        if(Config::configInstance().pointState[i]){
            DrawOnePoint(extrinsic,Xs[i],frame,cv::Scalar(0,0,255), iLevel);//inside

        }else {
            DrawOnePoint(extrinsic,Xs[i],frame,cv::Scalar(255,0,0), iLevel);//outside

        }
    }
    imshow("verify",out);

}
void Model::DrawPoints(const Sophus::SE3d &pose, const std::vector<cv::Point3d> &Xs, Frame &frame,
                       const std::string owner,const int iLevel) {
    cv::Mat extrinsic(4, 4, CV_32FC1);
    extrinsic = Se2cvf(pose);
    cv::Mat pt_status = frame.fw_posterior > frame.bg_posterior;
    cv::Mat out = frame.img.clone();
   // DisplayCV(pose, cv::Scalar(0,0,0),out,iLevel);

    int wrong=0;
    for(int i=0;i<Xs.size();i++){
        auto p2d= X_to_x(Xs[i],extrinsic,iLevel);
        switch (Config::configInstance().pointState[i]){
            case 0:
                wrong++;
                circle(out,p2d,3,cv::Scalar(255,0,0));//b->f ,蓝色
                break;
            case 1:
                wrong++;
                circle(out,p2d,3,cv::Scalar(0,255,0));//f->b ,绿色
                break;
            case 2:
                circle(out,p2d,3,cv::Scalar(0,0,255));//判断正确，内点，红色点
                break;
            case 3:
                circle(out,p2d,3,cv::Scalar(255,255,255));//判断正确，外点，白色点
                break;
        }

    }
    LOG(INFO)<<"owner "<<owner<<"'s wrong point Num = "<<wrong;
    imshow(owner,out);
}
void Model::DrawOnePoint(const cv::Mat &extrinsic, const cv::Point3d &X,cv::Mat &frame,const cv::Scalar scalar, const int iLevel) {
    auto p2d= X_to_x(X,extrinsic,iLevel);
    circle(frame,p2d,3,scalar);
}

void Model::DisplayCV(const Sophus::SE3d &pose, const cv::Scalar &color, cv::Mat &frame, const int iLevel)
{
    cv::Mat visualable_model_points;
    GetVisualableVertices(pose, visualable_model_points);

    cv::Mat image_points;
    Project3D_2D(pose, visualable_model_points, image_points, iLevel);

    int size = image_points.cols/2;
    for (int i = 0; i < size; ++i) {
        cv::Point pt1(image_points.at<float>(0, 2*i), image_points.at<float>(1, 2*i));
        cv::Point pt2(image_points.at<float>(0, 2*i+1), image_points.at<float>(1, 2*i+1));
        if (PointInFrame(pt1, iLevel) && PointInFrame(pt2, iLevel)) {
            cv::line(frame, pt1, pt2, color);
        }
    }
}

bool Model::PointInFrame(const cv::Point &p, const int iLevel){
    Config &gConfig = Config::configInstance();
    return(p.x>=0&&p.y>=0&&p.x*pow(2,iLevel)<gConfig.VIDEO_WIDTH&&p.y*pow(2,iLevel)<gConfig.VIDEO_HEIGHT);
}
void Model::GetVisualableVertices(const Sophus::SE3d &pose, cv::Mat &vis_vertices) {
    cv::Mat pt_in_cam(3, VerticesCount()+1, CV_32FC1);

    cv::Mat extrinsic(4, 4, CV_32FC1);
    extrinsic = Se2cvf(pose);
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



void Model::Project3D_2D(const Sophus::SE3d &pose, const cv::Mat &visible_Xs, cv::Mat &visible_xs, const int iLevel)
{
    cv::Mat extrinsic = Se2cvf(pose);
    visible_xs=intrinsics[iLevel]*extrinsic*visible_Xs;
//    LOG(INFO)<<"new Intrinsic: "<<intrinsics[iLevel];
    for (int i = 0; i < visible_xs.cols; ++i) {
        float dz = 1.0f/visible_xs.at<float>(2, i);
        visible_xs.at<float>(0, i) *= dz;
        visible_xs.at<float>(1, i) *= dz;
    }
}
std::vector<cv::Point> Model::GetContourAt(Sophus::SE3d &pose) {
    cv::Mat visible_Xs,visible_xs;
    GetVisualableVertices(pose, visible_Xs);
    Project3D_2D(pose, visible_Xs, visible_xs);
    Config &g_Config = Config::configInstance();

    std::vector<std::vector<cv::Point> > contours;

    cv::Mat line_img = Mat::zeros(g_Config.VIDEO_HEIGHT, g_Config.VIDEO_WIDTH , CV_8UC1);
    DisplayCV(pose, cv::Scalar(255, 255, 255), line_img);
    cv::findContours(line_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(contours.empty())
        return  {};
    return move(contours[0]);
}

