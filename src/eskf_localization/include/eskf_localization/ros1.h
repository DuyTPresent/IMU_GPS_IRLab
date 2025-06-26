#ifndef __ROS1__
#define __ROS1__

#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "nav_msgs/Path.h"         //path
#include "sensor_msgs/Imu.h"       //imu
#include "sensor_msgs/NavSatFix.h" //gps
#include "geometry_msgs/PoseStamped.h"

#include "eskf.h"
#include "variable.h"
#include "geography.h"

using namespace std;

class ROS_interface
{
    private:
        // data //
        ros::NodeHandle nh;
        bool init;

        // PUBLISH
        ros::Publisher gps_path_pub;
        ros::Publisher fuse_path_pub;

        // publish data
        nav_msgs::Path gps_path;
        nav_msgs::Path fuse_path;

        //SUBSCRIBE
        ros::Subscriber gps_sub;
        ros::Subscriber imu_sub;

        // FILE_VARIABLE
        State state;
        IMU_Data imu_data;
        GPS_Data gps_data;
        map_projection_reference map_ref;
        double lat0;
        double lon0;
        double alt0;

        // FILE_ESKF
        ESKF eskf;

        // FILE_GEOGRAPHY
        GEOGRAPHY geography;

        // log
        std::ofstream file_state;
        std::ofstream file_gps;
        std::ofstream file_error;

    public:
        ROS_interface(ros::NodeHandle &n, double lat, double lon);
        ~ROS_interface();

        void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
        void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
        void data_convert_imu(const sensor_msgs::ImuConstPtr& imu_msg, IMU_Data& imu_data);
        void data_convert_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data);

        // //FILE_LOG
        // void LogGps(GPS_Data& gps_data);
        // void LogState(State& state);
        // void LogError(State& state);
};

////////////// FUNCTION

ROS_interface::ROS_interface(ros::NodeHandle &n, double lat, double lon)
{
    cout << "ROS_Interface Start!" << endl;
    nh = n;
    init = false;

    //PUBLISH
    gps_path_pub = nh.advertise <nav_msgs::Path> ("/gps_path", 100);
    fuse_path_pub = nh.advertise <nav_msgs::Path> ("/fused_path", 100);

    //SUBSCRIBE
    gps_sub = nh.subscribe("/fix", 100, &ROS_interface::gps_callback, this);
    imu_sub = nh.subscribe("/imu/data_newIMU", 100, &ROS_interface::imu_callback, this);   ///imu/data_newIMU   ///imu/data

    //init gps_path
    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    // init estimated_path
    fuse_path.header.frame_id = "map";
    fuse_path.header.stamp = ros::Time::now();
    fuse_path.header.seq = 0;

    //state.gravity = Eigen::Vector3d(0., 0., 9.81007); // ned frame
    state.gravity = Eigen::Vector3d(0., 0., -9.81007); // enu frame

    // init reference lat, lon projection
    geography.map_projection_init(&map_ref, lat, lon);
    lat0 = lat;
    lon0 = lon;
    alt0 = 0.0;   //42,3  //0.0
    //alt0 = 88.676;

    // // Log.
    // std::string log_folder = "/home/duy/ros_irl_cpp/src/eskf_localization";
    // ros::param::get("log_folder", log_folder);

    // file_state.open(log_folder + "/state_1.csv");
    // file_gps.open(log_folder +"/gps_1.csv");
    // file_error.open(log_folder +"/error_1.csv");
}

ROS_interface::~ROS_interface()
{
    file_state.close();
    file_gps.close();
    cout << "ROS_Interface Finish" << endl;
}

///////////////////////// IMU_CALL_BACK /////////////////////////////
void ROS_interface::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    data_convert_imu(imu_msg, imu_data);
    if(!init)
    {
        return;
    }
    eskf.Predict(imu_data, state);
}

void ROS_interface::data_convert_imu(const sensor_msgs::ImuConstPtr& imu_msg, IMU_Data& imu_data)
{
    imu_data.timestemp = imu_msg->header.stamp.toSec();

    imu_data.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                   imu_msg->linear_acceleration.y,
                                   imu_msg->linear_acceleration.z);
    
    imu_data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                    imu_msg->angular_velocity.y,
                                    imu_msg->angular_velocity.z);
}

///////////////////////// GPS_CALL_BACK /////////////////////////////
void ROS_interface::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    data_convert_gps(gps_msg, gps_data);
    if(!init)
    {
        eskf.Init(gps_data, state);
        init = true;
        return;
    }
    eskf.Correct(gps_data, state);
    eskf.State_update(state);
    //ADD FILE LOG POSITION AFTER FUSE
    //LogState(state);
    // add log error
    //LogError(state);
    ////////////// RESET_ERROR /////////////////
    eskf.Error_State_Reset(state);
    

    // publish gps_path
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.pose.position.x = gps_data.ned[0];
    point.pose.position.y = gps_data.ned[1];
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 0;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 1.0;
    gps_path.poses.push_back(point);
    gps_path_pub.publish(gps_path);

    // publish estimated_path
    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "map";
    estimated_point.header.stamp = ros::Time::now();
    estimated_point.pose.position.x = state.position[0];
    estimated_point.pose.position.y = state.position[1];
    estimated_point.pose.position.z = 0.0;
    estimated_point.pose.orientation.x = state.quaternion.x();
    estimated_point.pose.orientation.y = state.quaternion.y();
    estimated_point.pose.orientation.z = state.quaternion.z();
    estimated_point.pose.orientation.w = state.quaternion.w();

    fuse_path.poses.push_back(estimated_point);
    fuse_path_pub.publish(fuse_path);

}

void ROS_interface::data_convert_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data)
{
    gps_data.timestemp = gps_msg->header.stamp.toSec();

    gps_data.lla = Eigen::Vector3d(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    // CONVERT LLA --> ENU
    double enu[3];
    double lla[3] = {gps_msg->latitude, gps_msg->longitude, gps_msg->altitude};
    double ref[3] = {lat0, lon0, alt0};
    geography.lla2enu(enu, lla, ref);
    gps_data.ned = Eigen::Vector3d(enu[0], enu[1], enu[2]);
    // LogGps(gps_data);
}

// void ROS_interface::LogGps(GPS_Data& gps_data)
// {
//     file_gps << gps_data.timestemp << ","
//              << gps_data.ned[0] << "," << gps_data.ned[1] << "," << gps_data.ned[2] << "\n";
// }

// void ROS_interface::LogState(State& state)
// {
//     file_state << state.timestemp << ","
//                << state.position[0] << "," << state.position[1] << "," << state.position[2] << ","
//                << state.velocity[0] << "," << state.velocity[1] << "," << state.velocity[2] << ","
//                << state.quaternion.x() << "," << state.quaternion.y() << "," << state.quaternion.z() << "," << state.quaternion.w() << "\n";
// }

// void ROS_interface::LogState(State& state)
// {
//     file_state << state.timestemp << ","
//                << state.position[0] << "," << state.position[1] << "," << state.position[2] << ","
//                << state.velocity[0] << "," << state.velocity[1] << "," << state.velocity[2] << ","
//                << state.quaternion.x() << "," << state.quaternion.y() << "," << state.quaternion.z() << "," << state.quaternion.w() << ","
//                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
//                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] <<"\n";
// }


// void ROS_interface::LogError(State& state)
// {
//     Eigen::Vector3d error_pos = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(0, 0).transpose().data());
//     file_error << state.timestemp << ","
//                << error_pos[0] << "," << error_pos[1] << "," << error_pos[2] << "\n";
// }

#endif // ROS_INTERFACE
