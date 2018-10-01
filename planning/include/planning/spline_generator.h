#ifndef WLOS_GUIDANCE
#define WLOS_GUIDANCE

#include <ros/ros.h>
#include <ros/time.h>

/* Inputs */
#include <snake_msgs/Waypoints.h>

/* Spline generation utils */
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pkg_utils/spline.h>
#include <pkg_utils/trajectory.h>

/* Outputs */
#include <snake_msgs/Spline.h>


/* WlosGuidance 
	 ---> Functionality
	 ------> Computes reference angles for end effector
	 ---> Assumes
*/

class SplineGenerator 
{
	public:
		SplineGenerator();

		double rate_;
	private:

		/* Callbacks */
		void waypointCb(const snake_msgs::Waypoints::ConstPtr waypoint_msg);

		/* Utilities */
		int dim_; // Default:3

		/* Ros utils */
		ros::NodeHandle nh_;
		ros::Subscriber waypoint_sub_;
		ros::Publisher spline_pub_;

		/* waypointCb containers */
		int number_waypoints_;
		Eigen::MatrixXd waypoints_;
		utils::Spline spline_;
		snake_msgs::Spline spline_msg_;

		/* Waypoint conversion timing */
		ros::Time time_wp_in_;
		ros::Time time_wp_out_;

		/* Spline computation timing */
		ros::Time time_spline_in_;
		ros::Time time_spline_out_;
		
		/* Spline conversion timing */
		ros::Time time_spline_conversion_in_;
		ros::Time time_spline_conversion_out_;
};

#endif
