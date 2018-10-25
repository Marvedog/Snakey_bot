#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <snake_msgs/JointAngle.h>
#include <snake_msgs/JointAngles.h>

/// The goal of this function is to simulate the coordinate relations of a snake

/**
	* @brief Snake data is visualized
	*/


int 
main(int argv, char** argc)
{
	ros::init(argv, argc, "ins_mock_class");
	ros::NodeHandle nh;
	ros::Rate r(10);

	double PI = 3.14;

	nav_msgs::Odometry odom_msg;
	snake_msgs::JointAngle q0_msg;
	snake_msgs::JointAngle q1_msg;
	snake_msgs::JointAngle q2_msg;
	snake_msgs::JointAngle q3_msg;
	snake_msgs::JointAngle q4_msg;
	snake_msgs::JointAngles q_msg;

	ros::Publisher odompub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
	ros::Publisher jointpub = nh.advertise<snake_msgs::JointAngles>("joint_angles", 1);

	odom_msg.pose.pose.position.x = 5;
	odom_msg.pose.pose.position.y = -3;
	odom_msg.pose.pose.position.z = 1;
	odom_msg.pose.pose.orientation.x = 0;
	odom_msg.pose.pose.orientation.y = 0;
	odom_msg.pose.pose.orientation.z = 1;
	odom_msg.pose.pose.orientation.w = 0;

	// Assuming 2 joint modules 
	q0_msg.angle = 0;
	q1_msg.angle = PI/2;
	q2_msg.angle = PI/2;
	q3_msg.angle = 0;
	/*
	q2_msg.header.frame_id = "joint_1";
	q3_msg.header.frame_id = "joint_2";
	q4_msg.header.frame_id = "joint_3";
	q4_msg.angle = 0;
	*/
	q_msg.theta.push_back(q0_msg);
	q_msg.theta.push_back(q1_msg);
	q_msg.theta.push_back(q2_msg);
	q_msg.theta.push_back(q3_msg);
	/*
	q_msg.theta.push_back(q4_msg);
	*/

	while (ros::ok())
	{
		ros::spinOnce();

		/// Specify angle msg
		q_msg.theta[0].header.seq++;
		q_msg.theta[0].header.stamp = ros::Time::now();
		q_msg.theta[1].header.seq++;
		q_msg.theta[1].header.stamp = ros::Time::now();
		q_msg.theta[2].header.seq++;
		q_msg.theta[2].header.stamp = ros::Time::now();
		q_msg.theta[3].header.seq++;
		q_msg.theta[3].header.stamp = ros::Time::now();
		/*
		q_msg.theta[4].header.seq++;
		q_msg.theta[4].header.stamp = ros::Time::now();
		*/

		/// Specify odom msg
		odom_msg.header.seq++;
		odom_msg.header.stamp = ros::Time::now();
		odom_msg.header.frame_id = "map";

		odompub.publish(odom_msg);
		jointpub.publish(q_msg);
		r.sleep();
	}
	return 0;
}
