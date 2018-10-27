#include <control_system/wlos_guidance_node.h>
#include <pkg_utils/helper_functions.h>
#include <pkg_utils/conversions.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Quaternion.h>

/**
  * @brief Callback trajectory makes a deep copy of the input trajectory.
	* TODO: Add speed profile functionality
	*/
void
WlosGuidanceNode::trajectoryCb(const snake_msgs::Spline::ConstPtr trajectory_msg)
{
	utils::msgToSpline(*trajectory_msg, this->wlosguidance.trajectory);
	this->trajectorystamp = ros::Time::now();
}

/** 
	* @brief Callback for odometry makes a deep copy of odometry input 
 	*/
void 
WlosGuidanceNode::odomCb(const nav_msgs::Odometry::ConstPtr odom_msg)
{
	this->wlosguidance.msgTo3dPose(*odom_msg);
	this->odometrystamp = ros::Time::now();
}

/**
	* @brief Callback running at constant rate to feed dynamic and 
	* configuration controller with reference values	
	* @param e timing event with timing info about the callback
	*/
void
WlosGuidanceNode::timerCb(const ros::TimerEvent &e)
{
	
	double tin = ros::Time::now().toSec();
	this->vlos = this->wlosguidance.losVector();
	this->qref = this->wlosguidance.computeAngularReference();
	this->posref = this->wlosguidance.computeLinearReference();

	this->publishVlos();
	this->publishOdomRef();

	double tout = ros::Time::now().toSec();
	ROS_INFO_STREAM("WLOSGUIDANCE:: Profiling period is " << e.profile.last_duration);
	ROS_INFO_STREAM("WLOSGUIDANCE:: Runtime:: " << tout - tin);
}

WlosGuidanceNode::WlosGuidanceNode(const ros::NodeHandle &nh)
: nh(nh)
{
	double minlookahead, maxlookahead;
	this->nh.getParam("minlookahead", minlookahead);
	this->nh.getParam("maxlookahead", maxlookahead);
	this->nh.getParam("tstimer", tstimer);

	wlosguidance = WlosGuidance(minlookahead, maxlookahead);

	this->trajectorysub = this->nh.subscribe("/planning/path", 1, &WlosGuidanceNode::trajectoryCb, this);
	this->odomsub = this->nh.subscribe("/odometry", 1, &WlosGuidanceNode::odomCb, this);
	this->vlospub = this->nh.advertise<snake_msgs::Vlos>("vlos", 1);
	this->odomrefpub = this->nh.advertise<nav_msgs::Odometry>("odometry_ref", 1);

	while (this->wlosguidance.trajectory.m == 0)
	{
		ros::spinOnce();
	}
	ROS_INFO_STREAM("WlosGuidanceNode:: Got first trajectory");

	this->timer = this->nh.createTimer(ros::Duration(this->tstimer), &WlosGuidanceNode::timerCb, this);
}

void
WlosGuidanceNode::publishVlos()
{
	this->vlosmsg.x0 = this->posref(0) - this->vlos(0);
	this->vlosmsg.y0 = this->posref(1) - this->vlos(1);
	this->vlosmsg.z0 = this->posref(2) - this->vlos(2);
	this->vlosmsg.x1 = this->posref(0);
	this->vlosmsg.y1 = this->posref(1);
	this->vlosmsg.z1 = this->posref(2);
	this->vlospub.publish(this->vlosmsg);
}

void
WlosGuidanceNode::publishOdomRef()
{
	this->odomrefmsg.pose.pose.position.x = this->posref(0);
	this->odomrefmsg.pose.pose.position.y = this->posref(1);
	this->odomrefmsg.pose.pose.position.z = this->posref(2);

	geometry_msgs::Quaternion quat;	
	tf::quaternionEigenToMsg(this->qref, quat);
	this->odomrefmsg.pose.pose.orientation = quat;
	this->odomrefpub.publish(odomrefmsg);
}

int 
main(int argc, char **argv) 
{
	ros::init(argc, argv, "wlos_guidance");
	ros::NodeHandle nh;
	WlosGuidanceNode wlos_guidance_node(nh);
	ROS_INFO_STREAM("Initialized wlos guidance node");
	ros::spin();
	return 0;
}
