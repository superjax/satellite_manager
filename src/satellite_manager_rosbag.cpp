#include "satellite_manager/satellite_manager_ros.h"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

extern "C"
{
#include "rtklib.h"
}

using namespace std;

void print_progress(double progress, double rate)
{
  int total_width = 100;
  int barwidth = 70;
  cout << "[";
  int pos = barwidth * progress;
  for (int i = 0; i < barwidth; ++i)
    if (i < pos) std::cout << "=";
    else if (i == pos) std::cout << ">";
    else std::cout << " ";
  cout << "]  ";
  printf("%.3f %%\t", progress*100.0);
  printf("(%.3fx)       \r", rate);
  cout.flush();
}

int main(int argc, char * argv[])
{
  string bag_filename = "";
  bool realtime = false;
  bool verbose = false;
  double start_time = 0;
  double duration = INFINITY;
  for (int i = 0; i < argc; i++)
  {
    string arg = argv[i];
    if (arg == "-h" || argc == 1 || arg == "--help")
    {
      cout << "USAGE: vi_ekf_rosbag [options]" << "\n\n";
      cout << "Options:\n";
      cout << "\t -h, --help\tShow this help message and exit\n";
      cout << "\t -f FILENAME\tBagfile to parse\n";
      cout << "\t -s START_TIME\tstart time of bag (seconds)\n";
      cout << "\t -r \t\tRun bag in real time\n";
      cout << "\t -u DURATION\tduration to run bag (seconds)\n";
      cout << "\t -v Show Verbose Output\n";
      cout << endl;
      return 0;
    }
    else if (arg == "-f")
    {
      if (i + 1 >= argc)
      {
        cout << "Please supply bag filename" << endl;
        return 0;
      }
      bag_filename = argv[++i];
    }
    else if (arg == "-s")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify start time" << endl;
        return 0;
      }
      start_time = atof(argv[++i]);
    }
    else if (arg == "-r")
    {
      realtime = true;
    }
    else if (arg == "-u")
    {
      if (i + 1 >= argc)
      {
        cout << "Please specify duration" << endl;
        return 0;
      }
      duration = atof(argv[++i]);
    }
    else if (arg == "-v")
    {
      verbose = true;
    }
    else if (i == 0)
    {
      continue;
    }
  }

  if (bag_filename.empty())
  {
    cout << "Please Specify bag file" << endl;
  }

  ros::init(argc, argv, "satellite_manager_rosbag");
  SatelliteManagerROS node;

  rosbag::Bag bag;
  try
  {
    bag.open(bag_filename.c_str(), rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    ROS_ERROR("unable to load rosbag %s, %s", bag_filename.c_str(), e.what());
    return -1;
  }
  rosbag::View view(bag);

  // Get list of topics and print to screen - https://answers.ros.org/question/39345/rosbag-info-in-c/
  if (verbose)
  {
    vector<const rosbag::ConnectionInfo*> connections = view.getConnections();
    vector<string> topics;
    vector<string> types;
    cout << "\nloaded bagfile: " << bag_filename << "\n===================================\n";
    cout << "Topics\t\tTypes\n----------------------------\n\n" << endl;
    for(vector<const rosbag::ConnectionInfo*>::iterator info = connections.begin();
        info != connections.end(); info++)
    {
      topics.push_back((*info)->topic);
      types.push_back((*info)->datatype);
      cout << (*info)->topic << "\t\t" << (*info)->datatype << endl;
    }
  }

  // Figure out the end time of the bag
  double end_time = start_time + duration;
  end_time = (end_time < view.getEndTime().toSec() - view.getBeginTime().toSec()) ? end_time : view.getEndTime().toSec() - view.getBeginTime().toSec();
  if (verbose)
    cout << "Playing bag from: = " << start_time << "s to: " << end_time << "s" << endl;

  // Get some time variables
  ros::Time system_start = ros::Time::now();
  ros::Time last_print = ros::Time(0);
  ros::Time bag_start = view.getBeginTime() + ros::Duration(start_time);
  ros::Time bag_end = view.getBeginTime() + ros::Duration(end_time);
  cout << "\n";

  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++)
  {
    ros::spinOnce();
    rosbag::MessageInstance const m = *it;
    // break on Ctrl+C
    if (!ros::ok())
      break;

    // skip messages before start time
    if (m.getTime() < bag_start)
      continue;

    // End bag after duration has passed
    if (m.getTime() > bag_end)
      break;

    // Run at realtime
    if (realtime)
    {
      while (ros::Time::now() - system_start < (m.getTime() - bag_start))
      {
        usleep(1000);
      }
    }

    // Print status at 30 Hz
    ros::Time now = ros::Time::now();
    if (now - last_print > ros::Duration(0.03333))
    {
      double bag_elapsed = (m.getTime() - bag_start).toSec();
      double system_elapsed = (now - system_start).toSec();
      print_progress(bag_elapsed / (bag_end - bag_start).toSec(), bag_elapsed / system_elapsed);
      last_print = now;
    }


    /// Call all the callbacks

    // Cast datatype into proper format and call the appropriate callback
    string datatype = m.getDataType();

    if (datatype.compare("inertial_sense/GNSSObservation") == 0)
    {
        const inertial_sense::GNSSObservationConstPtr obs(m.instantiate<inertial_sense::GNSSObservation>());
        node.obsCallback(obs);
    }

    if (datatype.compare("inertial_sense/GlonassEphemeris") == 0)
    {
        const inertial_sense::GlonassEphemerisConstPtr geph(m.instantiate<inertial_sense::GlonassEphemeris>());
        node.gephCallback(geph);
    }

    if (datatype.compare("inertial_sense/GNSSEphemeris") == 0)
    {
        const inertial_sense::GNSSEphemerisConstPtr eph(m.instantiate<inertial_sense::GNSSEphemeris>());
        node.ephCallback(eph);
    }
  }
  bag.close();
}

