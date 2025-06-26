#include <ros/ros.h>
#include "eskf_localization/ros1.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eskf_localization");
    ros::NodeHandle n;

    // double lat0 = 47.5115140833;
    // double lon0 = 6.79310693333;

    //data_1
    // double lat0 = 36.624675424;
    // double lon0 = 127.45634484116667;

    //data_2
    double lat0 = 36.462914;
    double lon0 = 127.086149;
    ROS_interface ros_interface (n, lat0, lon0);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}


