#include <ros/time.h>
#include <ros/package.h>
#include <ros/console.h>

#include <planning/waypoint_provider.h>
#include <planning/csv.h>

/* Utility messages */
#include <snake_msgs/Waypoint.h>
#include <visualization_msgs/Marker.h>

WaypointProvider::WaypointProvider()
	: waypoint_pub_(nh_.advertise<snake_msgs::Waypoints>("waypoints", 1))
	, waypoint_viz_pub_(nh_.advertise<visualization_msgs::MarkerArray>("viz/waypoints", 1))
	{
	
		if (nh_.getParam("test_data_csv", csv_file_)) {
			ROS_INFO_STREAM("--------------------------------------");
			ROS_INFO_STREAM("Reading test waypoints");
			ROS_INFO_STREAM("--------------------------------------");
			
			/* Read csv file */
			io::CSVReader<3> in(ros::package::getPath("planning") + "/test_data/" + csv_file_);
			in.read_header(io::ignore_extra_column, "x", "y", "z");

			/* Pre allocation */
			int i = 0;
			double x, y, z;
			
			/* Waypoints container */
			wps_.header.frame_id = "map_frame";
			wps_.loop = false;
			
			/* Iterate through file */
			while (in.read_row(x, y, z)) {
				
				/* Waypoint container */
				snake_msgs::Waypoint wp;
				wp.x = x;
				wp.y = y;
				wp.z = z;
				wps_.waypoints.push_back(wp);

				/* Visualization container */
				visualization_msgs::Marker wp_viz;
				wp_viz.header.stamp = ros::Time::now();
				wp_viz.header.frame_id = "map";
				wp_viz.id = i;
				wp_viz.type = 2;
				wp_viz.pose.position.x = x;
				wp_viz.pose.position.y = y;
				wp_viz.pose.position.z = z;
				wp_viz.scale.x = 0.1;
				wp_viz.scale.y = 0.1;
				wp_viz.scale.z = 0.1;
				wp_viz.color.r = 0;
				wp_viz.color.g = 0;
				wp_viz.color.b = 255;
				wp_viz.color.a = 1;
				wp_viz.lifetime = ros::Duration(0.1);
				wps_viz_.markers.push_back(wp_viz);
					
				i++;
			}
		} else
		{
			ROS_ERROR_STREAM("Could not read waypoints CSV");
		}
	} 

	void
	WaypointProvider::wpPublisher()
	{
		wps_.header.stamp = ros::Time::now();
    waypoint_pub_.publish(wps_);
    waypoint_viz_pub_.publish(wps_viz_);
  }
	
