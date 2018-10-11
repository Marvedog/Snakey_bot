#include <snake_viz/guidance_visualization.h>

#include <geometry_msgs/Point.h>
#include <ros/console.h>

GuidanceVisualization::GuidanceVisualization() 
{
	this->vlospub = this->nh.advertise<visualization_msgs::Marker>("control/viz/vlos", 1);
	this->odometrypub = this->nh.advertise<visualization_msgs::Marker>("control/viz/odometry_ref", 1);
	this->vlossub = this->nh.subscribe("control/vlos", 1, &GuidanceVisualization::vlosCb, this);
	this->odometrysub = this->nh.subscribe("control/odometry_ref", 1, &GuidanceVisualization::odomRefCb, this);
}

void 
GuidanceVisualization::vlosCb(const snake_msgs::Vlos::ConstPtr msg) 
{

	/* Read los vector start point (my assumed position)*/
	geometry_msgs::Point p0;
	p0.x = msg->x0;
	p0.y = msg->y0;
	p0.z = msg->z0;
	
	/* Read los vector end point */
	geometry_msgs::Point p1;
	p1.x = msg->x1;
	p1.y = msg->y1;
	p1.z = msg->z1;
	
	visualization_msgs::Marker marker = this->lineMarkerSetup();
	marker.points.push_back(p0);
	marker.points.push_back(p1);
	
	this->vlospub.publish(marker);
}

void 
GuidanceVisualization::odomRefCb(const nav_msgs::Odometry::ConstPtr msg)
{
	this->odomref = *msg;
	visualization_msgs::Marker marker = arrowMarkerSetup(this->odomref);
	odometrypub.publish(marker);
}

visualization_msgs::Marker 
GuidanceVisualization::lineMarkerSetup() 
{
  visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	return marker;
}

visualization_msgs::Marker 
GuidanceVisualization::arrowMarkerSetup(const nav_msgs::Odometry &msg) 
{
  visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position = msg.pose.pose.position;
	marker.pose.orientation	= msg.pose.pose.orientation;

	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;

	return marker;
}

int 
main( int argc, char** argv ) 
{
	ros::init(argc, argv, "racetrack_visualization");

	GuidanceVisualization racetrackVisualization;

  ros::spin();
}
