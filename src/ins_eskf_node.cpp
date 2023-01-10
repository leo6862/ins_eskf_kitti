#include <ins_eskf/ins_eskf.h>
#include <ins_eskf/ins_eskf_ros_wrapper.h>
#include "global_definition.h"
#include <ros/ros.h>


#include <glog/logging.h>

using namespace ins_eskf;


int main(int argc,char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = PROJECT_PATH + "/log";

    ros::init(argc,argv,"ins_eskf_node");
    ros::NodeHandle nh;

    YAML::Node node = YAML::LoadFile(PROJECT_PATH +"/config/ins_eskf.yaml");

    LOG(INFO) << "INS ESKF STARTED.";
    Ins_eskf_ROS_Wrapper ins_eskf_ros(nh,node);


    ros::spin();


}