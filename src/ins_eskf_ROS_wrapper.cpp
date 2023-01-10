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

    if(dataset == "kitti"){
        sub_vel = nh.subscribe<geometry_msgs::TwistStamped>(kitti_vel_topic,1000,&Ins_eskf_ROS_Wrapper::kitti_vel_cb,this);
    }
    
}



void Ins_eskf_ROS_Wrapper::gps_cb(const sensor_msgs::NavSatFixConstPtr& gps_in){

    mtx.lock();
    sensor_msgs::NavSatFix gps_data_ros(*gps_in);
    // if(!initialized){
    //     gps_buf.push_back(gps_data_ros);
        
    // }
    gps_buf.push_back(gps_data_ros);
    mtx.unlock();

    Ins_eskf::GPS_data gps_data;
    gps_data.stamp = gps_in->header.stamp.toSec();
    gps_data.lla << gps_in->latitude , gps_in->longitude,gps_in->altitude;

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
                p_ins_eskf->specify_init_state(init_state,initialization_stamp);
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


    Ins_eskf::IMU_data imu_data;


    imu_data.stamp = imu_in->header.stamp.toSec();
    imu_data.linear_acc   << imu_in->linear_acceleration.x ,
                             imu_in->linear_acceleration.y ,
                             imu_in->linear_acceleration.z ;

    imu_data.angular_velo << imu_in->angular_velocity.x ,
                             imu_in->angular_velocity.y ,
                             imu_in->angular_velocity.z ;


    // LOG(INFO) << "imu data recieved.";
    // p_ins_eskf->recieve_imu(imu_data);
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


void Ins_eskf_ROS_Wrapper::synce_measure(){
    /*
    在imu_buf 以及 gps_buff 中进行数据的时间的筛选
    将数据打包 ：
          。。。。。。。。。。。。。  。 IMU数据
                               ！   GPS数据
    按照以上的形式对数据进行打包
    */

    //TODO 已经将数据加入到了 imu_buf 以及 gps_buf了准备开干
}




}