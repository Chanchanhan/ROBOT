//
// Created by qqh on 18-3-14.
//

#include "MySolver.h"

MySolver::MySolver(cv::Mat &intrinsic){
    option.max_iterations=Config::configInstance().SV_MAX_ITERATIONS;

    option.energyOK=Config::configInstance().SV_E_OK;
    option.energyTooSmallSize=Config::configInstance().SV_E_TOO_SMALL_SIZE;
    option.He_b=Config::configInstance().SV_HE_b;
    option.max_wrong_point=Config::configInstance().SV_MAX_WRONG_POINT;
    option.lamda=1;
    option.energyLittle=0.1;
    option.lamdaSmaller=0.1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_(i,j) = intrinsic.at<float>(i,j);
        }
    }
}
void MySolver::Solve(FramePtr cur_frame, const int &iLevel) {


    for(int k=0;k<option.max_iterations;k++){
//        LOG(INFO)<<cur_frame->m_pose.log();

        Sophus::Vector6d jacobians=Sophus::Vector6d::Zero();
        Sophus::Matrix6d jtjs= Sophus::Matrix6d::Zero();
        Sophus::SE3d initialPose=cur_frame->m_pose;
        int initWrongJudgeCnt=0;
        double e_inital=Config::configInstance().TK_VER_NUMBER-cur_frame->VerticesNear2ContourX3D.size();
        for (int i = 0; i < cur_frame->VerticesNear2ContourX3D.size(); i++) {
            auto Xi = cur_frame->VerticesNear2ContourX3D[i];
            double energy=0;
            Sophus::Vector6d jac;
            bool judge;
            if(Evaluate(cur_frame,Xi,energy,jac,judge)){
                initWrongJudgeCnt+=(!judge)?1:0;
                if(std::isnan(energy)) {
                    LOG(WARNING)<<" nan energy";
//                    Evaluate(cur_frame,Xi,energy,jac);
                    continue;
                }
                e_inital+=energy;
                jacobians+=jac;
                jtjs+=jac*jac.transpose();
            }

        }
//        LOG(INFO)<<" initialPose: \n "<< initialPose.log();
        if(e_inital<option.energyOK) {
            LOG(INFO)<<"energyOK with : "<<e_inital;
            return;
        }
        Sophus::Vector6d update= option.lamda*jtjs.inverse()*jacobians;
        if(std::isnan(update[0])){
            LOG(WARNING)<<" nan update";
        }
        cur_frame->m_pose = Sophus::SE3d::exp(update)*cur_frame->m_pose;
        if(std::isnan(cur_frame->m_pose.log()[0])){
            LOG(WARNING)<<" nan m_pose";
        }
        double e_final=0;
//        LOG(INFO)<<" cur_frame->m_pose: \n "<< cur_frame->m_pose.log();

        int finalWrongJudgeCnt=0;
        pointStateTmp.resize(0);
        ComputeEnergy(cur_frame,cur_frame->VerticesNear2ContourX3D,e_final,finalWrongJudgeCnt);

        if(fabs(e_final-e_inital)<option.energyLittle) break;
        if(e_inital<e_final||e_final<option.energyTooSmallSize*e_inital||finalWrongJudgeCnt>initWrongJudgeCnt){
            option.lamda*=option.lamdaSmaller;
            cur_frame->m_pose = initialPose;
            if(e_inital<e_final)LOG(INFO)<<"energy become bigger:  inital = "<<e_inital<<" ,final = "<<e_final;
            else  if(e_final<option.energyTooSmallSize*e_inital)
                LOG(INFO)<<"energy become too small :  inital = "<<e_inital<<" ,final = "<<e_final;
            else
                LOG(INFO)<<"finalWrongJudgeCnt too much :"<<finalWrongJudgeCnt<<" , initial WrongJudgeCnt = "
                         <<initWrongJudgeCnt<<" , inital = "<<e_inital<<" ,final = "<<e_final;

            // break;
        }
        else {
            //  option.lamda=1;
            Config::configInstance().pointState=pointStateTmp;
            LOG(INFO)<<"energy become smaller:  inital = "<<e_inital<<" ,final = "<<e_final<<
                     " , initial finalWrongJudgeCnt = "<<initWrongJudgeCnt<<", final finalWrongJudgeCnt = "<<finalWrongJudgeCnt ;
        }
    }
}
void MySolver::ComputeEnergy(const FramePtr cur_frame,const std::vector<cv::Point3d> &Xs, double &energySum,int &wrongPointCnt) {
    energySum=0;
    k_th_tmp=0;
    pointStateTmp.resize(Xs.size());

    for (auto Xi: Xs) {
        if(!ComputeEnergy(cur_frame,Xi,energySum)){
            wrongPointCnt++;
        }


    }
}

bool MySolver::ComputeEnergy(const FramePtr cur_frame,const cv::Point3d &X_, double &energy) {
    auto m_pose = cur_frame->m_pose;

    Sophus::Vector3d Xis;
    // LOG(INFO)<<"in ComputeEnergy \n";
    Xis[0] = X_.x;
    Xis[1] = X_.y;
    Xis[2] = X_.z;
    Sophus::Vector3d X_Camera_coord = m_pose * Xis;
    Sophus::Vector3d x3 = K_ * X_Camera_coord;
    cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
//    LOG(INFO)<<"X_(Solver): "<<X_;
//    LOG(INFO)<<"x_plane(Solver): "<<x_plane;
    if(x_plane.x<0||x_plane.y<0||x_plane.x>=cur_frame->fw_posterior.rows||x_plane.y>=cur_frame->fw_posterior.cols){
        energy+=100;
        return false;
    }
    auto Theta_x = (double) (cur_frame->dt.at<float>(x_plane));

    double He = M_1_PI*(-atan(option.He_b*Theta_x)+M_PI_2);



    energy += (-log(He * cur_frame->fw_posterior.at<double>(x_plane) + (1 - He) * cur_frame->bg_posterior.at<double>(x_plane))) ;
    if(std::isnan(energy)){
        energy=1000;
        return false;
    }
    if( cur_frame->bg_posterior.at<double>(x_plane)>0.5 &&He > 0.5){
        // LOG(WARNING)<<"Wrong judge :  b->f ,  this x energy = "<<energy;
        return false;
    }
    else if( cur_frame->fw_posterior.at<double>(x_plane)>0.5 &&He < 0.5){
        // LOG(WARNING)<<"Wrong judge :  f->b , this x energy = "<<energy;
        return false;
    }
    if(cur_frame->bg_posterior.at<double>(x_plane)>cur_frame->fw_posterior.at<double>(x_plane)){
        pointStateTmp[k_th_tmp++]= false;
    }else
        pointStateTmp[k_th_tmp++]= true;


    return  true;
}
bool MySolver::Evaluate(const FramePtr cur_frame,const cv::Point3d &X_,
                        double &energy,
                        Sophus::Vector6d &jac,bool &judge) const {


    auto m_pose =cur_frame->m_pose;

    Sophus::Vector3d Xis;
    Xis[0] = X_.x;
    Xis[1] = X_.y;
    Xis[2] = X_.z;

    Sophus::Vector3d X_Camera_coord = m_pose * Xis;
    Sophus::Vector3d x3 = K_ * X_Camera_coord;
    cv::Point x_plane(x3(0) / x3(2), x3(1) / x3(2));
    auto Theta_x = (double) (cur_frame->dt.at<float>(x_plane));

    double He = M_1_PI*(-atan(option.He_b*Theta_x)+M_PI_2);
    double phi = -M_1_PI*option.He_b/(1+option.He_b*option.He_b*Theta_x*Theta_x);

    double left =  phi* (cur_frame->fw_posterior.at<double>(x_plane) - cur_frame->bg_posterior.at<double>(x_plane)) /
                   (He * cur_frame->fw_posterior.at<double>(x_plane) + (1 - He) * cur_frame->bg_posterior.at<double>(x_plane));
    energy = -log(
            He * cur_frame->fw_posterior.at<double>(x_plane) +
            (1 - He) * cur_frame->bg_posterior.at<double>(x_plane)) ;

    if(std::isnan(energy)){
        LOG(INFO)<<"energy is nan";
        return false;
    }
    double  fwp=cur_frame->bg_posterior.at<double>(x_plane);

    if( cur_frame->bg_posterior.at<double>(x_plane)>0.5 &&He > 0.5){
        // LOG(WARNING)<<"Wrong judge :  b->f ,  this x energy = "<<energy;
        judge= false;
    }
    else if( cur_frame->fw_posterior.at<double>(x_plane)>0.5 &&He < 0.5){
        judge= false;

        // LOG(WARNING)<<"Wrong judge :  f->b , this x energy = "<<energy;
    }else{
        judge= true;

        // LOG(WARNING)<<"Right judge  , this x energy = "<<energy;
    }
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
        if(std::isnan(jacTmp(0,i))){
            return  false;
        }
        jac[i]=jacTmp(0,i);
    }
//    LOG(INFO)<<"left = "<<left;
//    LOG(INFO)<<"j_Phi_x = "<<j_Phi_x;
//    LOG(INFO)<<"j_X_Lie = "<<j_X_Lie;


//    double He = M_1_PI*(-atan(b*Theta_x)+M_PI_2);


    return true;
}

