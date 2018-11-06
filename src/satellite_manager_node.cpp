#include <ros/ros.h>

#include "satellite_manager/satellite_manager_ros.h"


int main(int argc, char** argv)
{
    volatile SatelliteManagerROS thing;
    ros::init(argc, argv, "satellite_manager_node");
    ros::spin();
    return 0;
}
