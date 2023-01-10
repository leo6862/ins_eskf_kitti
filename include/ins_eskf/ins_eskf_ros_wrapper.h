#ifndef INS_ESKF_INS_ESKF_ROS_WRAPPER_H
#define INS_ESKF_INS_ESKF_ROS_WRAPPER_H


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <ins_eskf/ins_eskf.h>

#include <glog/logging.h>
#include <string>
#include <mutex>
namespace ins_eskf{

class Ins_eskf_ROS_Wrapper{

public:
    Ins_eskf_ROS_Wrapper(ros::NodeHandle &_nh,YAML::Node& _node);

private:
    void imu_cb(const sensor_msgs::ImuConstPtr& imu_in);
    void gps_cb(const sensor_msgs::NavSatFixConstPtr& gps_in);
    void kitti_vel_cb(const geometry_msgs::TwistStampedConstPtr& twist_in);
    void register_sub_pub();
    void initialization_kitti();
    void synce_measure();
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_imu;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_vel;
    ros::Publisher pub_pos;
    std::shared_ptr<Ins_eskf> p_ins_eskf;


    std::string dataset = "kitti";


    std::string imu_topic,gps_topic;
    std::string kitti_vel_topic;
    bool initialized = false;
    std::mutex mtx;
    std::vector<sensor_msgs::NavSatFix> gps_buf;
    std::vector<sensor_msgs::Imu> imu_buf;
    std::vector<geometry_msgs::TwistStamped> init_twist_buf;
    Ins_eskf::State init_state;
    double initialization_stamp = -1;
};


}



#endif