#include <pkg_utils/jacobian.h>

namespace utils
{

	void
	skew3D(const Eigen::Vector3d &p, Eigen::Matrix3d &skew)
	{
		skew(1,0) = p(2);
		skew(2,0) = -p(1);
		skew(0,1) = -p(2);
		skew(2,1) = p(0);
		skew(0,2) = p(1);
		skew(1,2) = -p(0);
	}

	void 
	computeAdjoint(const Eigen::Matrix4d &tf, Eigen::Matrix<double, 6, 6> &adjoint)
	{
		Eigen::Matrix3d skew;
		skew3D(tf.block(3,0,3,1), skew);
		adjoint <<  tf.block(0,0,3,3)			 , skew*tf.block(0,0,3,3)
			        , Eigen::Matrix3d::Zero(), tf.block(0,0,3,3);
	}
	
	void 
	computeAdjointInverse(const Eigen::Matrix4d &tf, Eigen::Matrix<double, 6, 6> &adjoint)
	{	
		Eigen::Matrix3d skew;
		skew3D(tf.block(3,0,3,1), skew);
		adjoint <<  tf.block(0,0,3,3).transpose(), -tf.block(0,0,3,3).transpose()*skew
						  , Eigen::Matrix3d::Zero()				, tf.block(0,0,3,3).transpose();
	}

	Eigen::MatrixXd 
	geometricJacobian(const int joints, const Eigen::MatrixXd &homTransforms, const Eigen::MatrixXd bodyTwists)
	{
		Eigen::MatrixXd jacobian_manipulator = Eigen::MatrixXd::Zero(6,joints);
		for (int i = 0; i < joints; i++)
		{
			Eigen::Matrix<double, 6, 6> adjoint;
			computeAdjoint(homTransforms.block(i*4, i*4, 4, 4), adjoint);
			
			jacobian_manipulator.block(0,i,6,1) = adjoint * bodyTwists.block(0,i,6,1); 
		}
		Eigen::Matrix<double, 6, 6> adjoint_inverse;
		computeAdjointInverse(homTransforms.block(0,joints*4, 4, 4), adjoint_inverse);

		Eigen::MatrixXd jacobian_geometric;
		jacobian_geometric << adjoint_inverse, adjoint_inverse*jacobian_manipulator;
		return jacobian_geometric;
	}

} /* end namespace utils */
