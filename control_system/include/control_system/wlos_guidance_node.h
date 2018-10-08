#ifndef WLOS_GUIDANCE_NODE_H
#define WLOS_GUIDANCE_NODE_H

#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <snake_msgs/Vlos.h>
#include <snake_msgs/Spline.h>

#include <control_system/wlos_guidance.h>

class WlosGuidanceNode
{
	public:
		WlosGuidanceNode(const ros::NodeHandle &nh);

	private:
		
		void trajectoryCb(const snake_msgs::Spline::ConstPtr trajectory_msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr odom);
		void timerCb(const ros::TimerEvent &e);

		void publishVlos();
		void publishOdomRef();

		WlosGuidance wlosguidance;

		Eigen::VectorXd posref;
		Eigen::VectorXd vlos;
		Eigen::Quaterniond qref; 
	
		snake_msgs::Vlos vlosmsg;
		nav_msgs::Odometry odomrefmsg;;

		ros::NodeHandle nh;
		ros::NodeHandle nhp;
		ros::Subscriber trajectorysub;
		ros::Subscriber odomsub;
		ros::Publisher vlospub;
		ros::Publisher odomrefpub;

		ros::Timer timer;

		double tstimer;

		ros::Time trajectorystamp;
		ros::Time odometrystamp;
};

#endif
