#include <cmath>
#include <Eigen/Dense>

namespace utils
{
	
	/* 
		 3d rotation round i-axis
		 ---> Input
		 ------> rot_i: rotation i axis (rad)
		 ------> out: output rotation (possibly block), Eigen::Matrix3d
	*/

	template <typename Derived> void
	rot_x(const double rot_x, Eigen::MatrixBase<Derived> const &out) 
	{
		const_cast< Eigen::MatrixBase<Derived>& > (out) << 1,           0,           0,
																											 0,  cos(rot_x), -sin(rot_x),
																											 0,  sin(rot_x),  cos(rot_x);
	}

	template <typename Derived> void
	rot_y(const double rot_y, Eigen::MatrixBase<Derived> const &out) 
	{
		const_cast< Eigen::MatrixBase<Derived>& > (out) <<  cos(rot_y),  0,  sin(rot_y),
																											           0,  1,           0,
																											 -sin(rot_y),  0,  cos(rot_y);
	}
	
	template <typename Derived> void
	rot_z(const double rot_z, Eigen::MatrixBase<Derived> const &out) 
	{
		const_cast< Eigen::MatrixBase<Derived>& > (out) <<  cos(rot_z), -sin(rot_z), 0,
																											  sin(rot_z),  cos(rot_z), 0,
																											           0,           0, 1;
	}


	template <typename Derived> void
	attutude_error(const Eigen::MatrixXd& R_1, const Eigen::Matrix& R_2, Eigen::MatrixBase<Derived> const &out)
	{
		const< Eigen::MatrixBase<Derived>& > (out).noalias() = R_1*R_2.transpose();
	}

}
