#include <ros/ros.h>
#include "mav_planner/path_planner.h"
#include <iostream>

static const std::string STREAM_PREFIX = "[Semi-Auto Pilot in control (Man-lead exploration)]: ";


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "mav_planner_node");
	ros::NodeHandle nh("~");

	// nut::Planner::Ptr mav_planner;
    // mav_planner.reset(new nut::Planner(nh));
	nut::Planner* mav_planner;
	mav_planner->getInstance(nh);

  	while (ros::ok()) 
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	} 

  	return 0;
}