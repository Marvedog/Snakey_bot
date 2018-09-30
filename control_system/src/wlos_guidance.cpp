#include "control_system/wlos_guidance.h"
#include "pkg_utils/helper_functions.h"
#include "pkg_utils/conversions.h"

#include <ros/console.h>

#include <math.h>

/* WlosGuidance implemented for Real time performance 
	 ---> Pre-allocation of all utilities
*/

WlosGuidance::WlosGuidance()
: rate_(50)
{
	bool read_success = true;
	std::string package_name = "control_system";
	
	/* Read parameters */
	if (!helper_functions::getRosParamAndPrintRosWarningIfFail("min_lookahead"
																														, min_lookahead_
																														, &nh_
																														, package_name
																														, 0.2))
		read_success = false;

	if (!helper_functions::getRosParamAndPrintRosWarningIfFail("max_lookahead"
																														, max_lookahead_
																														, &nh_
																														, package_name
																														, 0.4))
		read_success = false;
	
	if (!helper_functions::getRosParamAndPrintRosWarningIfFail("joints"
																														, joints_
																														, &nh_
																														, package_name
																														, 1))
		read_success = false;
	
	/* Verify all parameters read */ 
	if(!read_success){
		ROS_WARN("wlos_guidance:: Failed to read some parameters.");
	}

	PI = 3.141592;

	/* Initialize subscribers */ 
	odom_sub_ = nh_.subscribe("/odommetry", 1, &WlosGuidance::odomCb, this);
	trajectory_sub_ = nh_.subscribe("/planning/trajectory", 1, &WlosGuidance::trajectoryCb, this);

	/* Initialize publishers */
	desired_velocity_ = nh_.advertise<nav_msgs::Odometry>("base_velocity", 5);
	desired_end_effector_pose_ = nh_.advertise<nav_msgs::Odometry>("base_velocity", 5);

	/* Container pre-allocation ::trajectoryCb */
	time_traj_input_ = ros::Time::now();			

	/* Container pre-allocation ::odomCb */
	time_odom_input_ = ros::Time::now();

	/* Container pre-allocation ::progressPath */
	t_progress_ = 0;
	
	pose_progress_ = Eigen::VectorXd::Zero(3);

	/* Container pre-allocation ::losVector */
	double lookahead_waypoint_ = 0;
	
	pos_lookahead_ = Eigen::VectorXd::Zero(3);
	vlos_ = Eigen::VectorXd::Zero(3);

	/* Container pre-allocation ::computeVelocityReference */
	x_angle_ = 0;
	y_angle_ = 0;
	z_angle_ = 0;
	
	los_frame_ = Eigen::MatrixXd::Identity(3,3);
	base_frame_ = Eigen::MatrixXd::Identity(3,3);
	orientation_error_ = Eigen::MatrixXd::Identity(3,3);
}

/* Callback for trajectory 
	 ---> TODO: Edge cases ref. r18dv_los_guidance
*/
void
WlosGuidance::trajectoryCb(const snake_msgs::Trajectory::ConstPtr trajectory_msg)
{
	msgToTrajectory(*trajectory_msg, trajectory_);

	/* Timing */
	time_traj_input_ = ros::Time::now();
}

/* Callback for odometry */
void 
WlosGuidance::odomCb(const nav_msgs::Odometry::ConstPtr odom_msg)
{
	odometry_ = *odom_msg;
	
	/* Store orientation as euler angles */
	base_frame_quat_ = Eigen::Quaterniond(odometry_.pose.pose.orientation.w
																			 ,odometry_.pose.pose.orientation.x
																			 ,odometry_.pose.pose.orientation.y
																			 ,odometry_.pose.pose.orientation.z);

	/* Timing */
	time_odom_input_ = ros::Time::now();
}

/* Compute current spatial progress along spline */
void
WlosGuidance::progressPath()
{
	pose_progress_[0] = odometry_.pose.pose.position.x;
	pose_progress_[1] = odometry_.pose.pose.position.y;
	pose_progress_[2] = odometry_.pose.pose.position.z;

	t_progress_ = trajectory_.path.project(pose_progress_, t_progress_);
}

/* Compute vlos */
void 
WlosGuidance::losVector()
{
	/* End point los vector */
	lookahead_waypoint_ = t_progress_ + min_lookahead_;
	pos_lookahead_ = trajectory_.path.evaluate(lookahead_waypoint_, 0);	

	/* los vector */
	vlos_ = pos_lookahead_ - pose_progress_;
}

/* Compute references 
 	 ---> Assumes XYZ Euler angles
	 ---> Functionality:
	 ------> Currently finds reference angle which is passed to simulation.
	 ------> Attitude regulation is assumed to be good for our current purpose.

	 ---> TODO: Find an accurate velocity mapping => pass to dynamic controller
 */
void 
WlosGuidance::computeVelocityReference()
{
	/* Orientation los frame */
	x_angle_ = acos(vlos_[0]/vlos_.norm());
	y_angle_ = acos(vlos_[1]/vlos_.norm());
	z_angle_ = acos(vlos_[2]/vlos_.norm());

	los_frame_quat_ = Eigen::AngleAxisd(x_angle_*PI, Eigen::Vector3d::UnitX())
						  		 *Eigen::AngleAxisd(y_angle_*PI, Eigen::Vector3d::UnitY())
						  		 *Eigen::AngleAxisd(z_angle_*PI, Eigen::Vector3d::UnitZ());
	
	/* Oritentation rotation matrices */
	los_frame_ = los_frame_quat_.toRotationMatrix();
	base_frame_ = base_frame_quat_.toRotationMatrix();

	/* Find orientation error */
	orientation_error_.noalias() = base_frame_.transpose() * los_frame_;
}

/* Compute tail reference position 
 	 ---> TODO: Find general mapping for from end_frame to n'th link
 */ 
void
WlosGuidance::computeTailReference()
{
	if (joints_ > 1)
	{

	}	
}
