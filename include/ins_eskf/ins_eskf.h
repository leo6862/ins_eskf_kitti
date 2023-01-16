#ifndef INS_ESKF_INS_ESKF_H
#define INS_ESKF_INS_ESKF_H
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <mutex>
#include <glog/logging.h>
#include <iomanip>
#include <stdlib.h>
#include <GeographicLib/LocalCartesian.hpp>
namespace ins_eskf{
class Ins_eskf{
public:
    struct IMU_data{
        double stamp;
        Eigen::Vector3f linear_acc;
        Eigen::Vector3f angular_velo;
    };

    struct GPS_data{
        double stamp;
        Eigen::Vector3f lla;
    };

    struct State{
        Eigen::Quaternionf q;
        Eigen::Vector3f v;
        Eigen::Vector3f p;
        Eigen::Vector3f bg;
        Eigen::Vector3f ba;
        Eigen::Vector3f g;
    };

    struct Measure{
        std::vector<IMU_data> imu_buf;
        GPS_data gps_data;
    };

    Ins_eskf(YAML::Node &_node);
    void recieve_imu(const IMU_data _imu_data);
    void recieve_gps(const GPS_data _gps_data);
    void recieve_measure(Measure _measure);
    void specify_init_state(const State& _init_state,GPS_data _init_gps_data);
    inline State get_state() {return state;}
    inline double get_state_stamp() {return state_stamp;}


private:
    // void initialization_kitti();
    void prediction(); //imu 惯性解算获得nomial state.
    void forward_propagation(IMU_data _imu_data);
    void correction();

    Eigen::Matrix3f Exp(const Eigen::Vector3f &ang_vel, const float &dt);
    
private:
    GeographicLib::LocalCartesian geo_converter;
    Measure current_measure;
    bool initialized = false;
    std::string dataset = "kitti";
    // double initialization_stamp;
    double state_stamp = -1;
    State state;
    std::mutex mtx;
}; 

}


#endif