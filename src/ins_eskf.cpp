#include <ins_eskf/ins_eskf.h>



namespace ins_eskf{

Ins_eskf::Ins_eskf(YAML::Node& _node){
    dataset = _node["dataset"].as<std::string>();

    std::vector<float> state_std_vec = _node["init_state_std"].as<std::vector<float>>();
    
    state_std.setZero();
    state_std.diagonal() = Eigen::Map<Eigen::Matrix<float,18,1>>(state_std_vec.data());
    if(use_gao_xiang_q_matrix){
        Q_v = _node["Q_gao_xiang/Q_v"].as<float>();
        Q_q = _node["Q_gao_xiang/Q_q"].as<float>();
        Q_bg = _node["Q_gao_xiang/Q_bg"].as<float>();
        Q_ba = _node["Q_gao_xiang/Q_ba"].as<float>();
    }
    else{
        //TODO 使用fast-lio中的方式来计算Q
        exit(1);
        LOG(ERROR) << "Invalid Q matrix variables.\nMake sure Ins_eskf::use_gao_xiang_q_matrix = True\nUse Gao xiang ESKF Q matrix instead.";
    }

    v_x = _node["v_x"].as<float>();
    v_y = _node["v_y"].as<float>();
    v_z = _node["v_z"].as<float>();

}

//abandoned : use recieve_measure instead  
void Ins_eskf::recieve_gps(const GPS_data _gps_data){
    if(!initialized){
        return;
    }
    
    // LOG(INFO) << "gps data recieved after intialization.";

}

//abandoned ： use recieve_measure instead 
void Ins_eskf::recieve_imu(const IMU_data _imu_data){

    if(!initialized){
        return;
    }


    // LOG(INFO) << "imu-data recieved after initialization.";
    //使用IMU惯性结算 获得预测的nominal state 
    //对于初始化之后的第一帧imu数据 ，initialization_stamp != -1
    double time_inverval_ = _imu_data.stamp - state_stamp;
    CHECK(time_inverval_ > 0) << "imu data time ealier than state stamp,time seq error.";

}

void Ins_eskf::specify_init_state(const State& _init_state,Ins_eskf::GPS_data _init_gps_data){
    mtx.lock();
    state = _init_state;
    initialized = true;
    mtx.unlock();

    // initialization_stamp = _init_gps_data.stamp;
    state_stamp = _init_gps_data.stamp;
    
    CHECK(state_stamp != -1) << " Havn't specify initialization_stamp in ROS_wrapper.";

    geo_converter.Reset(_init_gps_data.lla[0],_init_gps_data.lla[1],_init_gps_data.lla[2]);

}


/*
此函数相当于measure的回调函数

算法的核心部分在此回调函数中执行
0.检查当前的输入的传感器数据的时序是否错乱 或者是否时间相差太远 有丢包的情况
1.在初始状态的基础上进行imu惯性解算 获得nominal state 
2.进行ESKF的 误差状态的先验方差的计算(不用显示的计算预测的误差状态)
3.进行卡尔曼增益 以及 计算后验的误差状态  并叠加到nominal state 上
*/
void Ins_eskf::recieve_measure(Measure _measure){
    current_measure = _measure;

    //TODO DEBUG
    CHECK(!_measure.imu_buf.empty()) << "No IMU Data in coming measure.";
    CHECK(_measure.gps_data.stamp - state_stamp > 0) << "Coming gps data stamp earlier than current state.Time Seq Error.";
    // LOG(INFO)<< std::setprecision(13)<< "_measure.gps_data.stamp = " << _measure.gps_data.stamp;
    // LOG(INFO)<< std::setprecision(13)<< "state_stamp = " << state_stamp;
    // LOG(INFO) << "state_stamp = " << state_stamp;
    if(initialized){
        // LOG(INFO) << "_measure.gps_data.stamp - state_stamp = " << _measure.gps_data.stamp - state_stamp;
        
        CHECK(_measure.gps_data.stamp - state_stamp < 1.1) << "Too large time gap between current state and coming gps data.";
    }

    prediction();

    double observation_x,observation_y,observation_z;
    geo_converter.Forward(_measure.gps_data.lla[0],_measure.gps_data.lla[1],_measure.gps_data.lla[2],observation_x,observation_y,observation_z);
    Eigen::Vector3f observation_gps_cartesian_(observation_x,observation_y,observation_z);
    correction(observation_gps_cartesian_);


}
//TODO 
void Ins_eskf::correction(Eigen::Vector3f _observation_gps_cartesian){
    /*
    0.计算先验误差的方差 
    1.计算卡尔曼增益    
    2.计算后验的期望
    3.计算后验的方差
    */
    
    //观测矩阵的维度为 18*3
    Eigen::Matrix<float,3,18> H_matrix_ = Eigen::Matrix<float,3,18>::Zero();
    H_matrix_.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f V_matrix_;
    V_matrix_ << v_x,0,0,
                 0,v_y,0,
                 0 ,0,v_z;
    Eigen::Matrix<float,18,3> kalman_gain_ = state_std * H_matrix_.transpose() * ((H_matrix_ * state_std * (H_matrix_.transpose()) + V_matrix_).inverse());
    Eigen::Matrix<float,18,1> delta_x_ = kalman_gain_ * (_observation_gps_cartesian - state.p);
    LOG(INFO) << "observation error pose = " << (_observation_gps_cartesian - state.p).transpose();
    LOG(INFO) << "delta_x_ = " << delta_x_.transpose(); 
    state_std = (Eigen::Matrix<float,18,18>::Identity() - kalman_gain_ * H_matrix_) * state_std;


    //进行状态的更新 将误差状态叠加到nominal state上
    //TODO 这个结果还是有点问题  显示四元数没有 normalize
    Eigen::Vector3f error_state_rot = delta_x_.block<3,1>(6,0);
    float error_state_rot_norm = error_state_rot.norm();
    if(error_state_rot_norm > 0.0000001){
        error_state_rot /= error_state_rot_norm;
        error_state_rot *= std::sin(error_state_rot_norm/2);
        Eigen::Quaternionf error_state_q_(std::cos(error_state_rot_norm/2),error_state_rot[0],error_state_rot[1],error_state_rot[2]);
        state.q = state.q * error_state_q_;
    }
    state.q.normalize();
    LOG(INFO) << "error_state_rot_norm = " << error_state_rot_norm;
    // state.q = (state.q * Exp(delta_x_.block<3,1>(6,0),1.)).normalized();
    state.p += delta_x_.block<3,1>(0,0);
    state.v += delta_x_.block<3,1>(3,0);
    state.bg += delta_x_.block<3,1>(9,0);
    state.ba += delta_x_.block<3,1>(12,0);
    state.g +=  delta_x_.block<3,1>(15,0);
    // LOG(INFO) << "state_std = \n" << state_std;
    LOG(INFO) << "---------------------------------------------"; 
}




void Ins_eskf::prediction(){
    /*
    根据 当前最新的measure 以及 当前的状态  进行IMU惯性结算 获得nominal state 
    ! 先直接使用IMU惯性解算看看 IMU惯性结算的结果大体是否正确
    ! 这个initial_state的旋转不是很正确。。。    
    */
    
    for(int i = 0;i <= current_measure.imu_buf.size();i++){
        /*
        当前状态与measure之间的时序的关系：
            state.stamp
                        .  . .. .. . . . IMU DATA.. . . . . . .. . . GPS DATA
        中间大部分的用于惯性结算的数据由插值产生
        头尾两端不用插值
        其中_imu_data 的stamp 表示 本次惯性结算的终点的时间戳
        */
        IMU_data imu_data_;
        if(i == 0){
            imu_data_ = current_measure.imu_buf[0];
        }
        else if(i == current_measure.imu_buf.size()){
            imu_data_ = current_measure.imu_buf.back();
            imu_data_.stamp = current_measure.gps_data.stamp;
        }
        else{
            imu_data_.linear_acc = (current_measure.imu_buf[i - 1].linear_acc + current_measure.imu_buf[i].linear_acc)/2;
            imu_data_.angular_velo = (current_measure.imu_buf[i - 1].angular_velo + current_measure.imu_buf[i].angular_velo)/2;
            imu_data_.stamp = current_measure.imu_buf[i].stamp;
        }
        // LOG(INFO) << "current integration imu data stamp = " <<  imu_data_.stamp;
        forward_propagation(imu_data_);
    }


}

/*


!fast-lio中使用的imu惯性解算的方法。 R' = R * Exp(gyr,dt);
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm; //!旋转轴
        Eigen::Matrix<T, 3, 3> K; //旋转轴的反对称矩阵
        K << SKEW_SYM_MATRX(r_axis);   //!获得角速度的单位向量对应的反对称矩阵
        T r_ang = ang_vel_norm * dt; //!这个相当于获得了
        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}
*/

/*
在此处的 Fx = 


*/
void Ins_eskf::forward_propagation(IMU_data _imu_data){
    /*
    将当前状态 state 上根据_imu_data进行惯性解算 
    !其中需要进行状态递推的时间gap = _imu_data.stamp - state_stamp;
    */
    _imu_data.angular_velo -= state.bg;
    _imu_data.linear_acc -= state.ba;
    double time_gap = _imu_data.stamp - state_stamp;

    state.p += state.v * time_gap;
    state.v += (state.q * _imu_data.linear_acc + state.g) * time_gap;
    state.q = state.q * Exp(_imu_data.angular_velo,time_gap);

    state_stamp = _imu_data.stamp;

    // 在此处完成状态方差的递推
    Eigen::Matrix<float,18,18> fx_ = make_Fx_matrix_from_imu_and_delta_t(_imu_data,time_gap);
    Eigen::Matrix<float,18,18> Q_ = make_Q_matrix_from_imu_and_delta_t(_imu_data,time_gap);
    state_std = fx_ * state_std * fx_.transpose() + Q_;
    // LOG(INFO) << "prediction std = \n" << state_std;
    // LOG(INFO) << "state_std.diagonal() = \n" << state_std.diagonal().transpose();
}

//TODO 先使用高博的写法试试 Q是一个固定的矩阵  ，如果这种写法有问题在使用fast-lio的写法
Eigen::Matrix<float,18,18> Ins_eskf::make_Q_matrix_from_imu_and_delta_t(IMU_data _imu_data,float _delta_t){

    //q以及g的噪声为0 ， 只需要对其余的项进行设置   
    Eigen::Matrix<float,18,18> Q_matrix_ = Eigen::Matrix<float,18,18>::Zero();
    
    Q_matrix_.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * Q_v;
    Q_matrix_.block<3,3>(6,6) = Eigen::Matrix3f::Identity() * Q_q;
    Q_matrix_.block<3,3>(9,9) = Eigen::Matrix3f::Identity() * Q_bg;
    Q_matrix_.block<3,3>(12,12) = Eigen::Matrix3f::Identity() * Q_ba;

    return Q_matrix_;

}

//Fx的构造方式参考高博的ESKF知乎帖子
//! 高博的Fx矩阵block<3,3>(6,9)这个位置应该是有点问题 bg  ba 的项搞混了 
Eigen::Matrix<float,18,18> Ins_eskf::make_Fx_matrix_from_imu_and_delta_t(IMU_data _imu_data,float _delta_t){
    Eigen::Matrix<float,18,18> Fx_ = Eigen::Matrix<float,18,18>::Identity();
    Fx_.block<3,3>(0,3) = Eigen::Matrix3f::Identity() * _delta_t;
    Fx_.block<3,3>(3,6) = -state.q.toRotationMatrix() * BuildSkewMatrix(_imu_data.linear_acc) * _delta_t;
    Fx_.block<3,3>(3,12) = -state.q.toRotationMatrix() * _delta_t;
    Fx_.block<3,3>(3,15) = state.q.toRotationMatrix() * _delta_t;
    Fx_.block<3,3>(6,6) = Exp(-_imu_data.angular_velo,_delta_t);
    Fx_.block<3,3>(6,9) = -Eigen::Matrix3f::Identity() * _delta_t;
    return Fx_;
}


Eigen::Matrix3f Ins_eskf::BuildSkewMatrix(const Eigen::Vector3f& vec){
    Eigen::Matrix3f matrix;
    matrix << 0.0,     -vec[2],   vec[1],
              vec[2],    0.0,     -vec[0],
              -vec[1],   vec[0],    0.0;

    return matrix;
}

//I copied it from FAST-LIO lol. Thanks to HKU.
Eigen::Matrix3f Ins_eskf::Exp(const Eigen::Vector3f &ang_vel, const float &dt){
    float ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3f Eye3 = Eigen::Matrix3f::Identity();
    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Vector3f r_axis = ang_vel / ang_vel_norm; //!旋转轴
        Eigen::Matrix3f K = BuildSkewMatrix(r_axis); //旋转轴的反对称矩阵
        float r_ang = ang_vel_norm * dt; //!这个相当 于获得了
        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

}