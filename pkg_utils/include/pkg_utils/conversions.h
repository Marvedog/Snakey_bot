#ifndef CONVERSIONS_H
#define CONVERSIONS_H

/* Messages */
#include <snake_msgs/Spline.h>
#include <snake_msgs/Trajectory.h>
#include <snake_msgs/Waypoints.h>
#include <eigen_conversions/eigen_msg.h>

/* Eigen utils */
#include <Eigen/Core>
#include <Eigen/Dense>

/* Home-crafted utils */
#include <pkg_utils/track.h>
#include <pkg_utils/trajectory.h>

#include <ros/console.h>

namespace utils
{
	inline void
	splineToMsg(const Spline &spline, snake_msgs::Spline &msg)
	{
			tf::matrixEigenToMsg(spline.coefdata(), msg.coefs);
			msg.breaks = std::vector<double>(spline.breakdata().data(), spline.breakdata().data() + spline.m + 1);
			msg.dim = spline.dim;
			msg.m = spline.m;
			msg.deg = spline.deg;
			msg.open = !spline.closed;
	}

	inline void
	msgToSpline(const snake_msgs::Spline &msg, Spline &spline)
	{
			spline.coefs = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Map(msg.coefs.data.data(), msg.coefs.layout.dim[0].size, msg.coefs.layout.dim[1].size);
			spline.breaks = Eigen::VectorXd::Map(msg.breaks.data(), msg.breaks.size());
			spline.dim = msg.dim;
			spline.m = msg.m;
			spline.deg = msg.deg;
			spline.closed = !msg.open;
	}

	/*

	inline void
	msgToSpline2d(const snake_msgs::Spline &msg, Spline2d &spline) {
			spline.coefs = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Map(msg.coefs.data.data(), msg.coefs.layout.dim[0].size, msg.coefs.layout.dim[1].size);
			spline.breaks = Eigen::VectorXd::Map(msg.breaks.data(), msg.breaks.size());
			spline.dim = msg.dim;
			spline.m = msg.m;
			spline.deg = msg.deg;
			spline.closed = !msg.open;
	}

	inline void
	spline2dToMsg(const Spline2d &spline, snake_msgs::Spline &msg)
	{
			tf::matrixEigenToMsg(spline.coefdata(), msg.coefs);
			msg.breaks = std::vector<double>(spline.breakdata().data(), spline.breakdata().data() + spline.m + 1);
			msg.dim = spline.dim;
			msg.m = spline.m;
			msg.deg = spline.deg;
			msg.open = !spline.closed;
	}

	*/

	inline void
	msgToTrajectory(const snake_msgs::Trajectory &traj_msg, utils::Trajectory &trajectory)
	{
			msgToSpline(traj_msg.path, trajectory.path);
			msgToSpline(traj_msg.speed, trajectory.speed);
	}

	inline void
	trajectoryToMsg(const utils::Trajectory trajectory, snake_msgs::Trajectory &traj_msg)
	{
			splineToMsg(trajectory.path, traj_msg.path);
			splineToMsg(trajectory.speed, traj_msg.speed);
	}

	/*

	inline void
	racetrackToMsg(const Track &track, snake_msgs::Racetrack &msg)
	{
			splineToMsg(track.centerline, msg.centerline);
			msg.width = track.width;
	}

	inline void
	msgToRacetrack(const snake_msgs::Racetrack &msg, Track &track)
	{
		 msgToSpline(msg.centerline, track.centerline);
		 track.width = msg.width;
	}

	*/

	/* Stores data from waypoints message in place in the pts matrix 
		 ---> Inputs:
		 ------> wps: An array of three dimensional pts
		 ---> Outputs:
		 ------> pts: A Eigen matrix of dimensions 3xN. Provided for efficient storage of points from message
	*/
	inline void 
	waypointsMsgToPointMatrix(const snake_msgs::Waypoints::ConstPtr wps, Eigen::MatrixXd &pts)
	{
		int i = 0;
		for (auto& wp: wps->waypoints)
		{
			pts(0,i) = wp.x;
			pts(1,i) = wp.y;
			pts(2,i) = wp.z;
			i++;
		}
	}

} /* End namespace utils */

#endif
