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

void Ins_eskf::specify_init_state(const State& _init_state,double _initialization_stamp){
    mtx.lock();
    state = _init_state;
    initialized = true;
    mtx.unlock();

    initialization_stamp = _initialization_stamp;
    state_stamp = _initialization_stamp;
    CHECK(initialization_stamp != -1) << " Havn't specify initialization_stamp in ROS_wrapper.";
    
}




}