#include <ros/ros.h>

#include "satellite_manager/satellite_manager_ros.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "satellite_manager_node");
    volatile SatelliteManagerROS thing;
    ros::spin();
    return 0;
}
