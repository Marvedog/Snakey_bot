#include <pkg_utils/Pose3.h>

namespace geometry
{

Pose3::Pose3()
: Pose3(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity())
{}

Pose3::Pose3(Eigen::Vector3d t, Eigen::Matrix3d rot)
:  t_(t)
{
	 rot_ = Eigen::Quaterniond(rot);
}

Pose3::Pose3(Eigen::Vector3d t, Eigen::Quaterniond rot)
:  t_(t)
,  rot_(rot)
{}

Pose3::Pose3(double x, double y, double z, Eigen::Quaterniond rot)
:  rot_(rot)
{
	 t_ << x, y, z;
}

Pose3::Pose3(double R00, double R10, double R01, double R11, double R12, double R02, double R21, double R22, double t0, double t1, double t2)
{
	Eigen::Vector3d t;
	t << t0, t1, t2;
	t_ = t;

	Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
	rot(0,0) = R00;
	rot(0,1) = R01;
	rot(0,2) = R02;
	rot(1,0) = R10;
	rot(1,1) = R11;
	rot(1,2) = R12;
	rot(2,1) = R11;
	rot(2,2) = R12;
				 
	rot_ = Eigen::Quaterniond(rot);
}

double 
Pose3::x() const
{
	return  t_(0);
}

double 
Pose3::y() const
{
	return  t_(1);
}
		
double 
Pose3::z() const
{
	return  t_(2);
}

Eigen::Vector3d
Pose3::t() const
{
	return t_;
}

Eigen::Quaterniond 
Pose3::rot() const
{
	return rot_;
}

Pose3
Pose3::inverse() const
{
	return Pose3( -rot_.inverse().toRotationMatrix() * t_, rot_.inverse() );
}

Pose3
Pose3::compose(const Pose3& T) const
{
	return Pose3( t_ + rot_.toRotationMatrix()*T.t(), rot_ * T.rot());
}

void
Pose3::print(std::string s)
{
	std::cout << s << std::endl;
	Eigen::Matrix3d mat = this->rot_.toRotationMatrix();
	std::cout << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << this->t_.x() << std::endl;
	std::cout << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << this->t_.y() << std::endl;
	std::cout << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << this->t_.z() << std::endl;
}


} /// End namespace geometry
