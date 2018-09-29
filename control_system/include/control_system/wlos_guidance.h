#ifndef WLOS_GUIDANCE
#define WLOS_GUIDANCE

#include <ros/ros.h>
#include <ros/Time.h>

/* Inputs */
#include <nav_msgs/Odometry.h>
#include <snake_msgs/Trajectory.h>

/* Waypoint LOS guiance utils */
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pkg_utils/Trajectory.h>

/* Outputs */


/* WlosGuidance 
	 ---> Functionality
	 ------> Computes reference angles for end effector
	 ------> Computes referene velocity for end effector
	 ------> Computes reference pose for tail relative to end effector
	 ---> Assumes
	 ------> Base frame: CoM end-effector
	 ------> Angle represenation: XYZ 
*/

class WlosGuidance
{
	public:
		WlosGuidance();

		ros::Publisher desired_velocity_;
		ros::Publisher desired_end_effector_pose_;
	private:

		/* Callbacks */
		void trajectoryCb(const r18dv_msgs::Trajectory::ConstPtr trajectory_msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr odom);

		/* Utilities */
		void progressPath();
		void losVector();
		void computeVelocityReference();
		void computeTailReference();

		/* Ros utils */
		ros::NodeHandle nh_;
		ros::Subscriber trajectory_sub_;
		ros::Subscriber odom_sub_;

		/* Input */
		nav_msgs::Odometry odometry_;	
		Trajectory trajectory_;

		/* trajectoryCb containers */
		ros::Time time_traj_input_;
		
		/* odomCb containers */
		Eigen::Quaternion base_frame_quat_; 
		ros::Time time_odom_input_;

		/* progressPath containers */
		double t_progress_;
		Eigen::VectorXd pose_progress_;
		
		/* losVector containers */
		double lookahead_waypoint_ = 0;
		Eigen::VectorXd pos_lookahead_;
		Eigen::VectorXd vlos_;
	
		/* computeVelocityReference containers */
		double x_angle_;
		double y_angle_;
		double z_angle_;
		Eigen::MatrixXf los_frame_;
		Eigen::MatrixXf base_frame_;
		Eigen::MatrixXf orientation_error_;

		/* Tuning parameters */
		double PI;
		double rate_;
		double min_lookahead_;
		double max_lookahead_;
		int joints_;
};

#endif