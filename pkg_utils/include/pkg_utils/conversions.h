#ifndef CONVERSIONS_h
#define CONVERSIONS_H

#include <snake_msgs/Spline.h>
//#include <snake_msgs/Racetrack.h>
#include <snake_msgs/Trajectory.h>
//#include <snake_msgs/Waypoints.h>
//#include <snake_msgs/Cone.h>
#//include <snake_msgs/Cones.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <pkg_utils/track.h>
#include <pkg_utils/trajectory.h>
#include <pkg_utils/point2d.h>

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

inline void 
waypointMsgToPoint2dVector(const snake_msgs::Waypoints &wps, std::vector<r18dv_utils::Point2d> &pts, int size)
{
	for (int i = 0; i < size; i++)
		pts.push_back(pkg_utils::Point2d(wps.waypoints[i].x, wps.waypoints[i].y));
}

*/

#endif
