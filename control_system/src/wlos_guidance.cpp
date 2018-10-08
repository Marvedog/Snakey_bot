#include <control_system/wlos_guidance.h>
#include <pkg_utils/helper_functions.h>
#include <pkg_utils/conversions.h>

#include <ros/console.h>

#include <math.h>

WlosGuidance::WlosGuidance()
: WlosGuidance(0.1, 0.2)
{;}

WlosGuidance::WlosGuidance(double minlookahead, double maxlookahead)
: minlookahead(minlookahead)
, maxlookahead(maxlookahead)
, roll(0)
, pitch(0)
, yaw(0)
, splineprogress(0)
, PI(3.141592)
{
	pose = Eigen::VectorXd::Zero(3);
	vlos = Eigen::VectorXd::Zero(3);
	poslookahead = Eigen::VectorXd::Zero(3);
}

void 
WlosGuidance::msgTo3dPose(const nav_msgs::Odometry odometry)
{
	/* Position */
	this->pose[0] = odometry.pose.pose.position.x;
	this->pose[1] = odometry.pose.pose.position.y;
	this->pose[2] = odometry.pose.pose.position.z;
	
	/* Orientation */
	this->map_base = Eigen::Quaterniond(odometry.pose.pose.orientation.w
																			 ,odometry.pose.pose.orientation.x
																			 ,odometry.pose.pose.orientation.y
																			 ,odometry.pose.pose.orientation.z);
}

/**
  * @brief Computes current spatial progress along spline 
  */
void
WlosGuidance::progressPath()
{
	this->splineprogress = this->trajectory.project(this->pose, this->splineprogress);
}

/** 
	* @brief Compute vlos vector 
  */
Eigen::VectorXd 
WlosGuidance::losVector()
{
	this->progressPath();

	/* End point los vector */
	this->wplookahead = this->splineprogress + this->minlookahead;
	this->poslookahead = this->trajectory.evaluate(this->wplookahead, 0);	

	/* los vector */
	this->vlos = this->poslookahead - this->pose;
	return this->vlos;
}

/**
  * @brief	Computes orientation and angular velocity references.
	* TODO: Add angular velocity reference
  */
Eigen::Quaterniond
WlosGuidance::computeAngularReference()
{
	/* Orientation los frame */
	this->roll = acos(this->vlos[0]/this->vlos.norm());
	this->pitch = acos(this->vlos[1]/this->vlos.norm());
	this->yaw = acos(this->vlos[2]/this->vlos.norm());

	this->map_vlos =  Eigen::AngleAxisd(this->roll*PI, Eigen::Vector3d::UnitX())
									* Eigen::AngleAxisd(this->pitch*PI, Eigen::Vector3d::UnitY())
									* Eigen::AngleAxisd(this->yaw*PI, Eigen::Vector3d::UnitZ());
	return this->map_vlos;	
}

/**
  * @brief	Computes orientation and linear velocity references.
	* TODO: Add linear velocity reference
  */
Eigen::VectorXd
WlosGuidance::computeLinearReference()
{
	/* Position: return spline evaluated at lookahead waypoint */
	return  this->poslookahead;
}
