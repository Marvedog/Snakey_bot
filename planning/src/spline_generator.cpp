#include "planning/spline_generator.h"
#include "pkg_utils/eigen_helpers.h"
#include "pkg_utils/helper_functions.h"
#include "pkg_utils/conversions.h"

#include <math.h>

#include <iostream>
/* Storage of matrix sizes.
	 TODO: Create storage file for preallocated sizes! 
*/
namespace 
{
	const int maximum_waypoints = 256;
}

/* SplineGenerator implemented for Real time performance 
	 ---> Pre-allocation of all utilities
*/

SplineGenerator::SplineGenerator()
: rate_(50)
, dim_(3) 
{
	/* Initialize subscribers */ 
	waypoint_sub_ = nh_.subscribe("waypoints", 1, &SplineGenerator::waypointCb, this);

	/* Initialize publishers */
	spline_pub_ = nh_.advertise<snake_msgs::Spline>("path", 5);

	/* Container pre-allocation ::waypointCb */
	number_waypoints_ = 0;
	waypoints_ = Eigen::MatrixXd::Zero(dim_, maximum_waypoints);
}

/* Callback for trajectory 
	 ---> TODO: Edge cases ref. r18dv_los_guidance
*/
void
SplineGenerator::waypointCb(const snake_msgs::Waypoints::ConstPtr waypoint_msg)
{
	/* Timing */
	time_wp_in_ = ros::Time::now();

	/* Store waypoint data in-house */
	number_waypoints_ = waypoint_msg->waypoints.size();	
	utils::waypointsMsgToPointMatrix(waypoint_msg, waypoints_);

	/* Timing */
	time_wp_out_ = ros::Time::now();
	time_spline_in_ = time_wp_out_;

	/* Compute spline */
	spline_ = utils::Spline( waypoints_.block(0, 0, dim_, number_waypoints_)
				 								 , utils::getChordBreaks(waypoints_.block(0, 0, dim_, number_waypoints_), false) 
												 , utils::Spline::CUBIC  
												 , false);

	/* Timing */
	time_spline_out_ = ros::Time::now();
	time_spline_conversion_in_ = time_spline_out_;
	
	/* Create spline msg */
	utils::splineToMsg(spline_, spline_msg_);

	/* Timing */
	time_spline_conversion_out_ = ros::Time::now();
	spline_msg_.start.data = time_wp_in_;	
	spline_msg_.end.data = time_spline_conversion_out_;	
	
	/* Publish spline */
	spline_pub_.publish(spline_msg_);
}
