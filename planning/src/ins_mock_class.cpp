#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int 
main(int argv, char** argc)
{
	ros::init(argv, argc, "ins_mock_class");
	ros::NodeHandle nh;
	ros::Rate r(10);

	nav_msgs::Odometry odom_msg;

	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);

	odom_msg.pose.pose.position.x = 1;
	odom_msg.pose.pose.position.y = 1;
	odom_msg.pose.pose.position.z = 1;
	odom_msg.pose.pose.orientation.x = 0;
	odom_msg.pose.pose.orientation.y = 0;
	odom_msg.pose.pose.orientation.z = 0;
	odom_msg.pose.pose.orientation.w = 1;

	while (ros::ok())
	{
		ros::spinOnce();
		pub.publish(odom_msg);
		r.sleep();
	}
	return 0;
}
