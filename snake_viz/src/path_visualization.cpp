#include <snake_viz/path_visualization.h>

#include <pkg_utils/spline.h>
#include <pkg_utils/conversions.h>

#include <geometry_msgs/Point.h>

#include <Eigen/Dense>

#include <ros/console.h>

PathVisualization::PathVisualization() 
{
	pub_ = nh_.advertise<visualization_msgs::Marker>("planning/viz/path", 1);
	sub_ = nh_.subscribe("planning/path", 1, &PathVisualization::pathCb, this);
}

void 
PathVisualization::pathCb(const snake_msgs::Spline::ConstPtr msg) 
{
	visualization_msgs::Marker marker = markerSetup();
	utils::Spline spline;
	utils::msgToSpline(*msg, spline);

	for (double t = 0; t < spline.end(); t += 0.1f) { // NOLINT
		Eigen::VectorXd p = spline.evaluate(t, 0);

		geometry_msgs::Point point;
		point.x = p[0];
		point.y = p[1];
		point.z = p[2];

		marker.points.push_back(point);
	}
	pub_.publish(marker);
}

visualization_msgs::Marker 
PathVisualization::markerSetup() 
{
  visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	
	marker.scale.x = 0.1;

	return marker;
}

int 
main( int argc, char** argv ) 
{
	ros::init(argc, argv, "racetrack_visualization");

	PathVisualization racetrackVisualization;

  ros::spin();
}
