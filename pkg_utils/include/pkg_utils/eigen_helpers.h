#ifndef _EIGEN_HELPERS_
#define _EIGEN_HELPERS_

#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

Eigen::MatrixXd readWaypoints(std::string path);
Eigen::MatrixXd cycleMatrix(const Eigen::MatrixXd &X, int drow, int dcol);
Eigen::VectorXd cycleVector(const Eigen::VectorXd &X, int drow);
Eigen::VectorXd getChordLengths(const Eigen::MatrixXd &Y);
Eigen::VectorXd getChordBreaks(const Eigen::MatrixXd &Y, bool closed);
Eigen::MatrixXd multiArrayToMatrix(std_msgs::Float64MultiArray msg);
std_msgs::Float64MultiArray matrixToMultiArray(Eigen::MatrixXd m);

#endif
