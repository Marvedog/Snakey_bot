#include <snake_msgs/Waypoints.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

class WaypointProvider 
{
 public:
  
	/* WaypointProvider
		 Reads from csv file and stores waypoint in containers wps_ and 		wps_viz.
	*/
	WaypointProvider();
  
	void wpPublisher();

 private:
  
	ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Publisher waypoint_viz_pub_;
	
	std::string csv_file_;

	/* Output data */
	snake_msgs::Waypoints wps_;  

	/* Visualization */
	visualization_msgs::MarkerArray wps_viz_;
};
