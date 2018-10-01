//
// Created by Marcus Engebretsen on 25.08.18.
//

#include <planning/waypoint_provider.h>
#include <ros/ros.h>

int 
main(int argc, char **argv) 
{
    ros::init(argc, argv, "waypoint_provider");
		WaypointProvider wp_provider;
		ros::Duration r = ros::Duration(0.1);
		
		while (ros::ok()) {
			wp_provider.wpPublisher();
			ros::spinOnce();
			r.sleep();
		}
    return 0;
}
