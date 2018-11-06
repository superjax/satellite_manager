#pragma once


#include <ros/ros.h>
#include <inertial_sense/GNSSEphemeris.h>
#include <inertial_sense/GlonassEphemeris.h>
#include <inertial_sense/GNSSObservation.h>

#include "satellite_manager/satellite_manager.h"
extern "C" {
#include "rtklib.h"
}

class SatelliteManagerROS
{
public:
    SatelliteManagerROS();

    void ephCallback(const inertial_sense::GNSSEphemerisConstPtr &msg);
    void gephCallback(const inertial_sense::GlonassEphemerisConstPtr& msg);
    void obsCallback(const inertial_sense::GNSSObservationConstPtr &msg);

private:
    void update();
    SatelliteManager sat_manager_;
};
