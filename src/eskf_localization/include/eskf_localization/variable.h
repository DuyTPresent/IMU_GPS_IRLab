#ifndef __VARIABLE__
#define __VARIABLE__

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct IMU_Data
{
    double timestemp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro; 
};

struct GPS_Data
{
    double timestemp;

    Eigen::Vector3d lla;
    Eigen::Vector3d ned; 
};

struct State
{
    double timestemp;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d gravity;
    Eigen::Matrix <double, 18, 18> PEst; 
    Eigen::Matrix <double, 18, 1> error;

    // Eigen::Matrix <double, 9, 9> PEst; 
    // Eigen::Matrix <double, 9, 1> error;
};

struct map_projection_reference
{
    uint64_t timestemp;
    double lat_rad;
    double lon_rad;
    double sin_lat;
    double cos_lat;
    bool init_done;
};

#endif