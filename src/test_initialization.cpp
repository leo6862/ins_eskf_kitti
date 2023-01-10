#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "global_definition.h"
#include <glog/logging.h>


using namespace ins_eskf;

#define INITIALIZATION_IMU_NUM 1000

std::vector<sensor_msgs::Imu> imu_buf;

void imu_cb(const sensor_msgs::ImuConstPtr& imu_msg){
    if(imu_buf.size() < INITIALIZATION_IMU_NUM){
        sensor_msgs::Imu imu_data = sensor_msgs::Imu(*imu_msg);
        imu_buf.push_back(imu_data);
        return;
    }



    if(imu_buf.size() == INITIALIZATION_IMU_NUM){
        //TODO计算b系的平均加速度、平均角速度、以及重力×自转
        Eigen::Vector3f ave_acc = Eigen::Vector3f::Zero();
        Eigen::Vector3f ave_gyr = Eigen::Vector3f::Zero();
        Eigen::Vector3f b_g_c_w;
        for(size_t i = 0;i < imu_buf.size();i++){
            ave_acc[0] += imu_buf[i].linear_acceleration.x;
            ave_acc[1] += imu_buf[i].linear_acceleration.y;
            ave_acc[2] += imu_buf[i].linear_acceleration.z;
        
            ave_gyr[0] += imu_buf[i].angular_velocity.x;
            ave_gyr[1] += imu_buf[i].angular_velocity.y;
            ave_gyr[2] += imu_buf[i].angular_velocity.z;
        }
        ave_acc /= imu_buf.size();
        ave_acc *= 9.81;
        ave_gyr /= imu_buf.size();
        b_g_c_w = ave_acc.cross(ave_gyr);
        // LOG(INFO) << "ave_acc = \n" << ave_acc;
        // LOG(INFO) << "ave_gyr = \n" << ave_gyr;
        // LOG(INFO) << "b_g_c_w = \n" << b_g_c_w;







        //TODO获得n系的地球自转角速度，重力，重力×自转。
        double w = 7.27220521664304e-05;    // 地球自转速度
        Eigen::Vector3f w_ie_n( w * std::cos(43.8 * M_PI / 180),0,
                           -w * std::sin(43.8 * M_PI / 180));
        Eigen::Vector3f g_n(0,0,-9.81);
        Eigen::Vector3f n_g_c_w =  g_n.cross(w_ie_n);

        Eigen::Matrix3f mat1,mat2;
        mat1.col(0) = ave_acc;
        mat1.col(1) = ave_gyr;
        mat1.col(2) = b_g_c_w;
        mat2.col(0) = g_n;
        mat2.col(1) = w_ie_n;
        mat2.col(2) = n_g_c_w;

        Eigen::Matrix3f c_n_b = (mat1*mat2.inverse()).transpose();

        float tan_yaw = c_n_b(1,0)/c_n_b(0,0);
        float yaw = atan(tan_yaw)/3.1415 * 180;
        // LOG(INFO) << "tan_yaw = " << tan_yaw; 

        float roll = atan(-c_n_b(2,0)/sqrt(c_n_b(2,1)*c_n_b(2,1) + c_n_b(2,2) * c_n_b(2,2)))/3.1415 * 180;
        float pitch = atan(c_n_b(2,1)/c_n_b(2,2))/3.14 * 180;

        LOG(INFO) << "yaw = " << yaw;
        LOG(INFO) << "roll = " << roll;
        LOG(INFO) << "pitch = " << pitch; 
        LOG(INFO) << "------------------------------" ;
        imu_buf.clear();
        // ros::shutdown();
    }

}


int main(int argc,char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = PROJECT_PATH + "/log/test_initialization";



    //使用IMU进行姿态的初始化
    ros::init(argc,argv,"test_initialization_node");
    ros::NodeHandle nh;


    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/livox/imu",10000,&imu_cb);

    ros::spin();
}