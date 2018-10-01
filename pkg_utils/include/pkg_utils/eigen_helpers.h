#ifndef _EIGEN_HELPERS_H
#define _EIGEN_HELPERS_H

#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

namespace utils
{
	Eigen::MatrixXd readWaypoints(std::string path);
	Eigen::MatrixXd cycleMatrix(const Eigen::MatrixXd &X, int drow, int dcol);
	Eigen::VectorXd cycleVector(const Eigen::VectorXd &X, int drow);

	/* Function getChordLengths
		 ---> In:
		 ------> Y: Matrix of n dimensional coordinates dim x points
		 ---> Out:
		 ------> t: Vector of the length between each chord of Y
		 TODO: Make this function RT friendly
	 */

	Eigen::VectorXd getChordLengths(const Eigen::MatrixXd &Y);

	/* Function getChordBreaks
		 ---> In:
		 ------> Y: Matrix of n dimensional coordinates dim x points
		 ------> closed: Boolean expressing whether this is closed or nah
		 ---> Out:
		 ------> t: Vectors expressing the length between chords between points in Y.
  	 TODO: Make this function RT friendly
	*/

	Eigen::VectorXd getChordBreaks(const Eigen::MatrixXd &Y, bool closed);
	Eigen::MatrixXd multiArrayToMatrix(std_msgs::Float64MultiArray msg);
	std_msgs::Float64MultiArray matrixToMultiArray(Eigen::MatrixXd m);

} /* End namespace utils */

#endif
