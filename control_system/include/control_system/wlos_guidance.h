#ifndef WLOS_GUIDANCE_H
#define WLOS_GUIDANCE_H

#include <nav_msgs/Odometry.h>
#include <pkg_utils/spline.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/** 
	* @brief WlosGuidance class computes a waypoint line-of-sight guidance
	* vector and compute position, orientation and velocity references.
	* @implicit Base frame: CoM end-effector (head)
	* @implicit Angle represenation: XYZ euler (computing los frame) 
*/

class WlosGuidance
{
	public:
		WlosGuidance();
		WlosGuidance(double minlookahead, double maxlookahead);

		void msgTo3dPose(const nav_msgs::Odometry odom);
		Eigen::VectorXd losVector();
		Eigen::VectorXd computeLinearReference();
		Eigen::Quaterniond computeAngularReference();

		utils::Spline trajectory; /* TODO: Add handling of speed profile */
		Eigen::VectorXd pose;
		Eigen::Quaterniond map_base; 

	private:
		
		/* Utilities */
		void progressPath();

		double splineprogress;
		
		/* losVector containers */
		double wplookahead;
		Eigen::VectorXd poslookahead;
		Eigen::VectorXd vlos;
	
		/* computeVelocityReference containers */
		double roll;
		double pitch;
		double yaw;
		Eigen::Quaterniond map_vlos; 
		
		/* Tuning parameters */
		double PI;
		double minlookahead;
		double maxlookahead;
};
#endif
