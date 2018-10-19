#ifndef POSE_3_H
#define POSE_3_H

#include <Eigen/Dense>
#include <string>
#include <iostream>

namespace geometry
{

class Pose3
{
	public: 
		Pose3();
		Pose3(Eigen::Vector3d t, Eigen::Quaterniond rot);
		Pose3(Eigen::Vector3d t, Eigen::Matrix3d rot);
		Pose3(double x, double y, double z, Eigen::Quaterniond rot);
		Pose3(double R00, double R10, double R01, double R11, double R12, double R20, double R21, double R22, double t0, double t1, double t2);

		double x() const;
		double y() const;
		double z() const;
	 	Eigen::Vector3d t() const;
		Eigen::Quaterniond rot() const;

		Pose3 inverse() const;
		Pose3 compose(const Pose3& T) const;
		
		Pose3 operator*(const Pose3& T) const
		{
			return Pose3(t_ + rot_.toRotationMatrix()*T.t(), rot_ * T.rot());
		}

		Pose3 operator=(const Pose3& T)
		{
			t_ = T.t();
			rot_ = T.rot();
		}

		void print(std::string s);		
	private:

		Eigen::Vector3d t_;
		Eigen::Quaterniond rot_;

};

} /// End namespace geometry

#endif
