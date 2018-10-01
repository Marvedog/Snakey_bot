#include "planning/spline_generator.h"

#include <ros/ros.h>

int 
main(int argc, char **argv) 
{
	ros::init(argc, argv, "spline_generator");
	SplineGenerator spline_generator;

	ros::Rate update_rate(spline_generator.rate_); //Hz

	while(ros::ok()){
		ros::spinOnce();
		update_rate.sleep();
	}
	return 0;
}
