#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <Eigen/Dense>

namespace utils
{
	void skew3D(const Eigen::Vector3d &p, Eigen::Matrix3d &skew);

	void computeAdjoint(const Eigen::Matrix4d &tf, Eigen::Matrix<double, 6, 6> &adjoint);
	
	void computeAdjointInverse(const Eigen::Matrix4d &tf, Eigen::Matrix<double, 6, 6> &adjoint);

	Eigen::MatrixXd geometricJacobian(const int joints, const Eigen::MatrixXd &homTransforms, const Eigen::MatrixXd bodyTwists);

} /* end namespace utils */
#endif
