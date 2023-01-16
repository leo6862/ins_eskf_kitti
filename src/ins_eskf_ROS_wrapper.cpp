#include <ins_eskf/ins_eskf_ros_wrapper.h>


namespace ins_eskf{

Ins_eskf_ROS_Wrapper::Ins_eskf_ROS_Wrapper(ros::NodeHandle &_nh,YAML::Node& _node){
    nh = _nh;

    //TODO YAML
    imu_topic = _node["imu_topic"].as<std::string>();
    gps_topic = _node["gps_topic"].as<std::string>();
    dataset   = _node["dataset"  ].as<std::string>();


    LOG(INFO) << "imu topic : " << imu_topic;
    LOG(INFO) << "gps topic : " << gps_topic;
    LOG(INFO) << "using dataset: " << dataset;

    if(dataset == "kitti"){
        LOG(INFO) << "Using imu and vel in kitti for initialization.";
        kitti_vel_topic = _node["kitti/vel_topic"].as<std::string>();
    }


    p_ins_eskf = std::make_shared<Ins_eskf>(_node);

    register_sub_pub();
}

void Ins_eskf_ROS_Wrapper::register_sub_pub(){

    sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic,1000,&Ins_eskf_ROS_Wrapper::imu_cb,this);
    sub_gps = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic,100,&Ins_eskf_ROS_Wrapper::gps_cb,this);


    pub_imu_odometrty_ = nh.advertise<nav_msgs::Odometry>("imu_odometry",10);
    pub_kitti_gps_odometrty_ = nh.advertise<nav_msgs::Odometry>("kitti_gps_odometry",10);

    if(dataset == "kitti"){
        sub_vel = nh.subscribe<geometry_msgs::TwistStamped>(kitti_vel_topic,1000,&Ins_eskf_ROS_Wrapper::kitti_vel_cb,this);
    }
    
}

void Ins_eskf_ROS_Wrapper::DEBUG_check_synce_measure(){
/*
    !  DEBUG
    1. 在此处测试一下打包的measure 中的imu队列以及gps的时间戳
    2. 以及剩余的imu_buf gps_buf中的数据的情况
*/
    LOG(INFO) << std::setprecision(14) << "measure.imu_buf.size() = " << measure.imu_buf.size();
    for(int i = 0;i < measure.imu_buf.size();i++){
        LOG(INFO) << std::setprecision(14) << "No. " << i << " imu data stamp = " << measure.imu_buf[i].stamp; 
    }
    LOG(INFO) << std::setprecision(14) << "measure.gps_data.stamp = " << measure.gps_data.stamp;
    LOG(INFO) << "-------------------------------------------------";


}

void Ins_eskf_ROS_Wrapper::gps_cb(const sensor_msgs::NavSatFixConstPtr& gps_in){

    mtx.lock();
    sensor_msgs::NavSatFix gps_data_ros(*gps_in);
    // if(!initialized){
    //     gps_buf.push_back(gps_data_ros);
        
    // }
    gps_buf.push_back(gps_data_ros);
    mtx.unlock();



    if(synce_measure()){
        p_ins_eskf->recieve_measure(measure);

        //! 主要用于将纯imu惯性解算的结果进行可视化的验证  与kittti数据集中的GPS+磁力计的结果进行比较
        DEBUG_visualize_imu_propagation_res_and_kitti_gps_magnetormeter();
    }


    // DEBUG_check_synce_measure();

    // Ins_eskf::GPS_data gps_data;
    // gps_data.stamp = gps_in->header.stamp.toSec();
    // gps_data.lla << gps_in->latitude , gps_in->longitude,gps_in->altitude;

    // LOG(INFO) << "DEBUG current gps_buf.size() = " << gps_buf.size();
    // LOG(INFO) << "DEBUG current imu_buf.size() = " << imu_buf.size();
    // LOG(INFO) << "gps msg recieved";
    // p_ins_eskf->recieve_gps(gps_data);
}

void Ins_eskf_ROS_Wrapper::imu_cb(const sensor_msgs::ImuConstPtr& imu_in){
    mtx.lock();
    sensor_msgs::Imu imu_data_ros(*imu_in);
    if(!initialized){
        
        imu_buf.push_back(imu_data_ros);
        //打印一下各个init容器是否正常接收到了数据
        // LOG(INFO) << "init_gps_buf.size() = " << init_gps_buf.size();
        // LOG(INFO) << "init_imu_buf.size() = " << init_imu_buf.size();
        // LOG(INFO) << "init_twist_buf.size() = " << init_twist_buf.size();

        // 检查init容器中的数据的数量是否已经达到了初始化的要求
        if(dataset == "kitti"){
            if(gps_buf.size() > 0 && imu_buf.size() > 10 && init_twist_buf.size() > 0){
                initialization_kitti();
                initialized = true;
                p_ins_eskf->specify_init_state(init_state,gps_msg_2_data(gps_buf.back()));
                

                //TODO DEBUG
                geo_converter_.Reset(gps_buf.back().latitude,gps_buf.back().longitude,gps_buf.back().altitude);

                LOG(INFO) << "initialization completed.";
                imu_buf.clear();
                gps_buf.clear();
            }
        }
        else{
            // 对于其他数据集额外指定其初始化的方式
            LOG(INFO) << "Please specify an initialization method for dataset: " << dataset;
        }

         mtx.unlock();
         
    }
    
    imu_buf.push_back(imu_data_ros);
    mtx.unlock();


    Ins_eskf::IMU_data imu_data = imu_msg_2_data(*imu_in);

}

void Ins_eskf_ROS_Wrapper::kitti_vel_cb(const geometry_msgs::TwistStampedConstPtr& twist_in){
    mtx.lock();
    if(!initialized){
        geometry_msgs::TwistStamped twist_in_ros(*twist_in);
        init_twist_buf.push_back(twist_in_ros);
    }
    mtx.unlock();
}



void Ins_eskf_ROS_Wrapper::initialization_kitti(){
    /*
    !kitti中vel与fix已经做了时间同步 
    使用init容器中的数据对   init_state进行初始化
    速度的值直接使用vel的值
    位置直接设为原点
    旋转使用磁力计
    重力直接设为(0,0,-9.81)
    */
    CHECK(init_twist_buf.back().header.stamp == gps_buf.back().header.stamp) << "GPS fix and vel time not synchronized.";
    init_state.v  = Eigen::Vector3f(init_twist_buf.back().twist.linear.x,
                                    init_twist_buf.back().twist.linear.y,
                                    init_twist_buf.back().twist.linear.z);
    init_state.p = Eigen::Vector3f::Zero();
    double gps_data_stamp = gps_buf.back().header.stamp.toSec();
    //遍历imu队列，寻找与gps时间相同的imu数据
    sensor_msgs::Imu imu_data_;
    int imu_data_index = 0;
    for(; imu_data_index < imu_buf.size();imu_data_index++){
        if(imu_buf[imu_data_index].header.stamp.toSec() >= gps_data_stamp){
            break;
        }
    }
    if(imu_data_index == imu_buf.size() || imu_data_index == 0){
        LOG(INFO) << "Do not have imu data later than gps data.Initialization failed.";
        ros::shutdown();
    }
    CHECK(imu_buf[imu_data_index - 1].header.stamp.toSec() < gps_data_stamp) << "imu_buf[imu_data_index - 1].header.stamp.toSec() >= gps_data_stamp,initialization failed.";
    float imu_data_inverval_ = imu_buf[imu_data_index].header.stamp.toSec() - imu_buf[imu_data_index-1].header.stamp.toSec();
    float front_propotion = (gps_data_stamp - imu_buf[imu_data_index - 1].header.stamp.toSec())/imu_data_inverval_;
    float back_propotion = 1 - front_propotion;
    Eigen::Quaternionf q_front_imu( imu_buf[imu_data_index-1].orientation.w,
                                    imu_buf[imu_data_index-1].orientation.x,
                                    imu_buf[imu_data_index-1].orientation.y,
                                    imu_buf[imu_data_index-1].orientation.z);
    Eigen::Quaternionf q_back_imu(  imu_buf[imu_data_index].orientation.w,
                                    imu_buf[imu_data_index].orientation.x,
                                    imu_buf[imu_data_index].orientation.y,
                                    imu_buf[imu_data_index].orientation.z);
    init_state.q = q_front_imu.slerp(back_propotion,q_back_imu);
    init_state.bg = Eigen::Vector3f::Zero();
    init_state.ba = Eigen::Vector3f::Zero();
    init_state.g = Eigen::Vector3f(0,0,-9.81);

    initialization_stamp = gps_data_stamp;

    //TODO print the init_state 
    // LOG(INFO) << "front_propotion = " << front_propotion;
    // LOG(INFO) << "init_state.p = " << init_state.p;
    // LOG(INFO) << "init_state.v = " << init_state.v;
    // LOG(INFO) << "init_state.q = " << init_state.q.w() << " " << init_state.q.x() << " " << init_state.q.y() << " " << init_state.q.z();
    // LOG(INFO) << "init_state.ba = " << init_state.ba; 
    // LOG(INFO) << "init_state.bg = " << init_state.bg; 
    // LOG(INFO) << "init_state.g = " << init_state.g;



}


bool Ins_eskf_ROS_Wrapper::synce_measure(){
    /*
    0.确保gps_buf以及 imu_buf都不为空
    1.如果最新的imu数据时间戳仍然早于gps数据的时间戳，则继续等待
    2.当前的imu数据已经覆盖了最老的gps数据的时间，将该帧GPS数据以及该帧GPS之前的所有IMU数据一起打包
    */


    //! 这个地方可以不用上mtx的锁了   因为其余的会操作imu_bug 以及gps_buf的地方都已经上锁了
    //! 所以这个地方不上锁问题也不大
    


    
    if(!initialized) return false;

    if(gps_buf.empty() || imu_buf.empty()){
        return false;
    }

    double recent_gps_stamp = gps_buf.front().header.stamp.toSec();
    if(imu_buf.back().header.stamp.toSec() < recent_gps_stamp){
        return false;
    }


    //TODO 在此处记录一下kitti_gps_odometry
    double kitti_gps_oodm_x,kitti_gps_oodm_y,kitti_gps_oodm_z;
    geo_converter_.Forward(gps_buf.back().latitude,gps_buf.back().longitude,gps_buf.back().altitude,kitti_gps_oodm_x,kitti_gps_oodm_y,kitti_gps_oodm_z);
    kitti_gps_odom_msg_.header.stamp = gps_buf.back().header.stamp;
    kitti_gps_odom_msg_.header.frame_id = "odom"; 
    kitti_gps_odom_msg_.child_frame_id = "imu";
    kitti_gps_odom_msg_.pose.pose.position.x = kitti_gps_oodm_x;
    kitti_gps_odom_msg_.pose.pose.position.y = kitti_gps_oodm_y;
    kitti_gps_odom_msg_.pose.pose.position.z = kitti_gps_oodm_z;




    measure.imu_buf.clear();

    sensor_msgs::Imu last_imu_msg;
    while(imu_buf.front().header.stamp.toSec() < recent_gps_stamp){
        measure.imu_buf.push_back(imu_msg_2_data(imu_buf.front()));
        last_imu_msg = imu_buf.front();
        imu_buf.pop_front();
    }

    //TODO DEBUG从measure中的最后一个imu数据获得姿态
    
    kitti_gps_odom_msg_.pose.pose.orientation = last_imu_msg.orientation;



    measure.gps_data = gps_msg_2_data(gps_buf.front());
    gps_buf.pop_front();


    return true;

}





Ins_eskf::IMU_data Ins_eskf_ROS_Wrapper::imu_msg_2_data(sensor_msgs::Imu _imu_msg){

    Ins_eskf::IMU_data imu_data_;


    imu_data_.stamp = _imu_msg.header.stamp.toSec();
    imu_data_.linear_acc   << _imu_msg.linear_acceleration.x ,
                             _imu_msg.linear_acceleration.y ,
                             _imu_msg.linear_acceleration.z ;

    imu_data_.angular_velo << _imu_msg.angular_velocity.x ,
                             _imu_msg.angular_velocity.y ,
                             _imu_msg.angular_velocity.z ;
    
    return imu_data_;


}
Ins_eskf::GPS_data Ins_eskf_ROS_Wrapper::gps_msg_2_data(sensor_msgs::NavSatFix _gps_msg){
    Ins_eskf::GPS_data gps_data_;
    gps_data_.stamp = _gps_msg.header.stamp.toSec();
    gps_data_.lla << _gps_msg.latitude , _gps_msg.longitude,_gps_msg.altitude;
    return gps_data_;
}

sensor_msgs::NavSatFix Ins_eskf_ROS_Wrapper::gps_data_2_msg(Ins_eskf::GPS_data& _gps_data){
    sensor_msgs::NavSatFix gps_msg_;
    gps_msg_.header.stamp = ros::Time(_gps_data.stamp);
    gps_msg_.latitude = _gps_data.lla[0];
    gps_msg_.longitude = _gps_data.lla[1];
    gps_msg_.altitude = _gps_data.lla[2];

    return gps_msg_;
}

void Ins_eskf_ROS_Wrapper::DEBUG_visualize_imu_propagation_res_and_kitti_gps_magnetormeter(){
    /*
    1.从p_ins_eskf中拿到最新的state
    2.根据当前measure的GPS以及IMU的数据获得kitti的GPS里程计
    3.将两者进行可视化的比较
    */
    //TODO DEBUG
    // LOG(INFO) << "gps buff size = " << gps_buf.size();  //0
    // LOG(INFO) << "imu buff size = " << imu_buf.size();
    // CHECK(!imu_buf.empty()) << " empty imu_buf";
    
    Ins_eskf::State current_state_ = p_ins_eskf->get_state();
    double state_stamp = p_ins_eskf->get_state_stamp();
    sensor_msgs::NavSatFix gps_msg_ = gps_data_2_msg(measure.gps_data);
    nav_msgs::Odometry imu_odom_ = state_to_odom_msg(current_state_,state_stamp);
    
    //将kitti_gps_odom_msg_以及 imu_odom_发布出去
    pub_imu_odometrty_.publish(imu_odom_);
    pub_kitti_gps_odometrty_.publish(kitti_gps_odom_msg_);
    LOG(INFO) << "kitti gps odom pos : " << kitti_gps_odom_msg_.pose.pose.position.x << " "
                                         << kitti_gps_odom_msg_.pose.pose.position.y << " "
                                         << kitti_gps_odom_msg_.pose.pose.position.z;
    LOG(INFO) << "imu odom pos : " << imu_odom_.pose.pose.position.x << " "
                                         << imu_odom_.pose.pose.position.y << " "
                                         << imu_odom_.pose.pose.position.z;
    LOG(INFO) << "DEBUG imu odom and kitti gps odom published.";

}


//TODO 
nav_msgs::Odometry Ins_eskf_ROS_Wrapper::state_to_odom_msg(Ins_eskf::State _state,double _stamp){
    nav_msgs::Odometry odom_msg_;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "imu";
    odom_msg_.header.stamp = ros::Time(_stamp);


    odom_msg_.pose.pose.position.x = _state.p[0];
    odom_msg_.pose.pose.position.y = _state.p[1];
    odom_msg_.pose.pose.position.z = _state.p[2];

    odom_msg_.pose.pose.orientation.w = _state.q.w();
    odom_msg_.pose.pose.orientation.x = _state.q.x();
    odom_msg_.pose.pose.orientation.y = _state.q.y();
    odom_msg_.pose.pose.orientation.z = _state.q.z();

    return odom_msg_;

}
}