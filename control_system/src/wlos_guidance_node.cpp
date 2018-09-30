#include "control_system/wlos_guidance.h"

#include <ros/ros.h>

int 
main(int argc, char **argv) 
{
	ros::init(argc, argv, "los_guidance");
	WlosGuidance wlos_guidance;

	ros::Rate update_rate(wlos_guidance.rate_); //Hz

	while(ros::ok()){
		ros::spinOnce();
		update_rate.sleep();
	}
	return 0;
}
