#ifndef DH_CONVENTION_H
#define DH_CONVENTION_H

#include <pkg_utils/Pose3.h>

namespace geometry
{

class DHTransform
{
	public:
		DHTransform();
		DHTransform(const double &d, const double &r, const double &alpha, const double &theta);

		Pose3 pose;

		DHTransform& operator=(const DHTransform& tf)
		{
			pose = Pose3(tf.pose.t(), tf.pose.rot());
			return *this;
		}
};

DHTransform::DHTransform()
: pose(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity())
{}

DHTransform::DHTransform(const double &d, const double &r, const double &alpha, const double &theta)
: pose(  cos(theta)
			 , sin(theta)
			 ,-sin(theta)*cos(alpha)
			 , cos(theta)*cos(alpha)
		   , sin(alpha)
			 , sin(theta)*sin(alpha)
			 , -cos(theta)*sin(alpha)
			 , cos(alpha)
			 , r*cos(theta)
			 , r*sin(theta)
			 , d)
{}

} /// end namespace geometry

#endif
