#include <control_system/wlos_guidance.h>
#include <pkg_utils/helper_functions.h>
#include <pkg_utils/conversions.h>

#include <ros/console.h>

#include <math.h>

WlosGuidance::WlosGuidance()
: WlosGuidance(0.4, 0.6)
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
	ux << 1, 0 , 0;
	uy << 0, 1 , 0;
	uz << 0, 0 , 1;
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
	this->wplookahead = this->splineprogress + this->maxlookahead;
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
	double norm = this->vlos.norm();
	
	/* Roll assumed constant */
	this->roll = 0;
	Eigen::Quaterniond qx = Eigen::AngleAxisd(this->roll, Eigen::Vector3d::UnitX())
												* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
												* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
	
	/* Correct for direction of z axis */
	Eigen::Vector3d vlosxy;
	Eigen::Vector3d vlos1 = qx.inverse() * this->vlos;

	vlosxy << vlos1[0], vlos1[1], 0;
	Eigen::Vector3d kyaw = this->ux.cross(vlosxy);
	
	if (kyaw.dot(uz) < 0 )
		this->yaw = -atan2((vlosxy.cross(this->ux)).norm(), vlosxy.dot(this->ux));
	else
		this->yaw = atan2((vlosxy.cross(this->ux)).norm(), vlosxy.dot(this->ux));
	
	Eigen::Quaterniond qz = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
												* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
												* Eigen::AngleAxisd(this->yaw, Eigen::Vector3d::UnitZ());

	//Eigen::Vector3d vlos2 = (qx.toRotationMatrix() * qz.toRotationMatrix()).transpose() * this->vlos;
	Eigen::Vector3d vlos2 = (qx*qz).inverse() * this->vlos;
	Eigen::Vector3d vlos2xy;
	vlos2xy << vlos2[0], vlos2[1], 0;
	Eigen::Vector3d kpitch = vlos2xy.cross(vlos2);

	if (kpitch.dot(this->uy) <= 0)
		this->pitch = -atan2((vlosxy.cross(vlos1)).norm(), vlosxy.dot(vlos1));
	else
		this->pitch = atan2((vlosxy.cross(vlos1)).norm(), vlosxy.dot(vlos1));
	
	Eigen::Quaterniond qy = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
												* Eigen::AngleAxisd(this->pitch, Eigen::Vector3d::UnitY())
												* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
	
	ROS_ERROR_STREAM("roll : " << this->roll );
	ROS_ERROR_STREAM("pitch : " << this->pitch );
	ROS_ERROR_STREAM("yaw : " << this->yaw );

	this->map_vlos = qx*qz*qy;
		
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

void 
WlosGuidance::getRPY(double &roll, double &pitch, double &yaw)
{
	roll = this->roll;
	pitch = this->pitch;
	yaw = this->yaw;
}
