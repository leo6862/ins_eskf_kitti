#include <ins_eskf/ins_eskf.h>



namespace ins_eskf{

Ins_eskf::Ins_eskf(YAML::Node& _node){
    dataset = _node["dataset"].as<std::string>();


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


    
    correction();


}
//TODO 
void Ins_eskf::correction(){
    /*
    0.计算先验误差的方差
    1.计算卡尔曼增益
    2.计算后验的期望
    3.计算后验的方差
    */
    Eigen::Matrix<>   Fx;

    
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
void Ins_eskf::forward_propagation(IMU_data _imu_data){
    /*
    将当前状态 state 上根据_imu_data进行惯性解算 
    !其中需要进行状态递推的时间gap = _imu_data.stamp - state_stamp;
    */
    _imu_data.angular_velo -= state.bg;
    _imu_data.linear_acc -= state.ba;
    double time_gap = _imu_data.stamp - state_stamp;
    Eigen::Quaternionf q_incre(

    );
    state.p += state.v * time_gap;
    state.v += (state.q * _imu_data.linear_acc + state.g) * time_gap;
    state.q = state.q * Exp(_imu_data.angular_velo,time_gap);

    state_stamp = _imu_data.stamp;


}

//I copied it from FAST-LIO lol. Thanks to HKU.
Eigen::Matrix3f Ins_eskf::Exp(const Eigen::Vector3f &ang_vel, const float &dt){
    float ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3f Eye3 = Eigen::Matrix3f::Identity();
    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Vector3f r_axis = ang_vel / ang_vel_norm; //!旋转轴
        Eigen::Matrix3f K; //旋转轴的反对称矩阵
        K << 0.,-r_axis[2],r_axis[1],
             r_axis[2],0.0,-r_axis[0],
             -r_axis[1],r_axis[0],0.0;   //!获得角速度的单位向量对应的反对称矩阵
        float r_ang = ang_vel_norm * dt; //!这个相当于获得了
        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}




}