//
// Created by qqh on 18-3-14.
//

#include "MySolver.h"
void MySolver::Solve(FramePtr cur_frame, const int &iLevel) {
    double lamda=1;
    for(int k=0;k<5;k++){
        Sophus::Vector6d jacobians=Sophus::Vector6d::Zero();
        Sophus::Matrix6d jtjs= Sophus::Matrix6d::Zero();
        Sophus::SE3d initialPose=cur_frame->m_pose;
        double e_inital=0;
        for (int i = 0; i < cur_frame->VerticesNear2ContourX3D.size(); i++) {
            auto Xi = cur_frame->VerticesNear2ContourX3D[i];
            double energy=0;
            Sophus::Vector6d jac;

            if(Evaluate(cur_frame,Xi,energy,jac)){
                if(std::isnan(energy)) {
                    LOG(WARNING)<<" nan energy";
                    continue;
                    //  Evaluate(cur_frame->m_pose.log(),Xi,energy,jac);
                }
                e_inital+=energy;
                jacobians+=jac;
                jtjs+=jac*jac.transpose();
            }

        }
        LOG(INFO)<<" initialPose: \n "<< initialPose.log();

        cur_frame->m_pose = Sophus::SE3d::exp(lamda*jtjs.inverse()*jacobians)*cur_frame->m_pose;
        double e_final=0;
        LOG(INFO)<<" cur_frame->m_pose: \n "<< cur_frame->m_pose.log();


        ComputeEnergy(cur_frame,cur_frame->VerticesNear2ContourX3D,e_final);
        if(e_inital<e_final){
            lamda/=10;
            cur_frame->m_pose = initialPose;
            LOG(INFO)<<"energy become bigger:  inital = "<<e_inital<<" ,final = "<<e_final;

            // break;
        }
        else {
            lamda=1;
            LOG(INFO)<<"energy become smaller:  inital = "<<e_inital<<" ,final = "<<e_final;
        }
    }
}
void MySolver::ComputeEnergy(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs, double &energySum) {
    energySum=0;
    for (auto Xi: Xs) {
        ComputeEnergy(cur_frame,Xi,energySum);
    }
}
void MySolver::ComputeEnergy(const FramePtr cur_frame,const cv::Point3d &X_, double &energy) {
    auto m_pose = cur_frame->m_pose;

    Sophus::Vector3d Xis;
    // LOG(INFO)<<"in ComputeEnergy \n";
    Xis[0] = X_.x;
    Xis[1] = X_.y;
    Xis[2] = X_.z;
    Sophus::Vector3d X_Camera_coord = m_pose * Xis;
    Sophus::Vector3d x3 = K_ * X_Camera_coord;
    cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
    auto Theta_x = (double) (cur_frame->dt.at<float>(x_plane));

    const int b=Config::configInstance().SV_HE_b;
    double He = M_1_PI*(-atan(b*Theta_x)+M_PI_2);
    if(std::isnan(energy))
        return;
    energy += (-log(He * cur_frame->fw_posterior.at<double>(x_plane) + (1 - He) * cur_frame->bg_posterior.at<double>(x_plane))) ;

}
bool MySolver::Evaluate(const FramePtr cur_frame,const cv::Point3d &X_,
                        double &energy,
                        Sophus::Vector6d &jac) const {


    auto m_pose =cur_frame->m_pose;

    Sophus::Vector3d Xis;
    Xis[0] = X_.x;
    Xis[1] = X_.y;
    Xis[2] = X_.z;

    Sophus::Vector3d X_Camera_coord = m_pose * Xis;
    Sophus::Vector3d x3 = K_ * X_Camera_coord;
    cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
    auto Theta_x = (double) (cur_frame->dt.at<float>(x_plane));

    const int b=Config::configInstance().SV_HE_b;
    double He = M_1_PI*(-atan(b*Theta_x)+M_PI_2);
    double phi = -M_1_PI*b/(1+b*b*Theta_x*Theta_x);
    double left =  phi* (cur_frame->fw_posterior.at<double>(x_plane) - cur_frame->bg_posterior.at<double>(x_plane)) /
                   (He * cur_frame->fw_posterior.at<double>(x_plane) + (1 - He) * cur_frame->bg_posterior.at<double>(x_plane));


    Eigen::MatrixXd j_X_Lie(2, 6);
    Eigen::MatrixXd j_Phi_x(1, 2);

    Config &gConfig = Config::configInstance();
    double _x_in_Camera = X_Camera_coord[0];
    double _y_in_Camera = X_Camera_coord[1];
    double _z_in_Camera = X_Camera_coord[2];

    j_X_Lie(0, 0) = -gConfig.FX / _z_in_Camera;
    j_X_Lie(0, 1) = 0;
    j_X_Lie(0, 2) = gConfig.FX * _x_in_Camera / (_z_in_Camera * _z_in_Camera);
    j_X_Lie(0, 3) = gConfig.FX * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
    j_X_Lie(0, 4) = -gConfig.FX * (1 + _x_in_Camera * _x_in_Camera / (_z_in_Camera * _z_in_Camera));
    j_X_Lie(0, 5) = gConfig.FX * _y_in_Camera / _z_in_Camera;

    j_X_Lie(1, 0) = 0;
    j_X_Lie(1, 1) = -gConfig.FY / _z_in_Camera;
    j_X_Lie(1, 2) = gConfig.FY * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
    j_X_Lie(1, 3) = gConfig.FY * (1 + _y_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera));
    j_X_Lie(1, 4) = -gConfig.FY * _x_in_Camera * _y_in_Camera / (_z_in_Camera * _z_in_Camera);
    j_X_Lie(1, 5) = -gConfig.FY * _x_in_Camera / _z_in_Camera;

    j_Phi_x(0, 0) = 0.5f * (cur_frame->dt.at<float>(cv::Point(x_plane.x + 1, x_plane.y)) -
            cur_frame->dt.at<float>(cv::Point(x_plane.x - 1, x_plane.y)));
    j_Phi_x(0, 1) = 0.5f * (cur_frame->dt.at<float>(cv::Point(x_plane.x, x_plane.y + 1)) -
            cur_frame->dt.at<float>(cv::Point(x_plane.x, x_plane.y - 1)));
    Eigen::MatrixXd jacTmp = left * j_Phi_x * j_X_Lie;

    for(int i=0;i<6;i++)
    {
        jac[i]=jacTmp(0,i);
    }
//    LOG(INFO)<<"left = "<<left;
//    LOG(INFO)<<"j_Phi_x = "<<j_Phi_x;
//    LOG(INFO)<<"j_X_Lie = "<<j_X_Lie;
    if(std::isnan(energy)){
        LOG(INFO)<<"energy is nan";
    }

    energy = (-log(He * cur_frame->fw_posterior.at<double>(x_plane) + (1 - He) * cur_frame->bg_posterior.at<double>(x_plane))) ;

    return true;
}

