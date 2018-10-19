#ifndef MODULES_H
#define MODULES_H

#include <pkg_utils/dh_convention.h>

class JointModule
{
	public:
		JointModule();
		JointModule(double a0, double a1, double a2, double q1, double q2, double alpha0);
		JointModule(const JointModule &obj);

		geometry::Pose3 updateTransform(double q1, double q2);
		geometry::Pose3 getTransform() const;
		geometry::Pose3 getTransform(int i) const;
		geometry::Pose3 inverse() const;
		geometry::Pose3 inverse(int i) const;

	private:

		double a0;
		double a1;
		double a2;
		double alpha0;

		geometry::DHTransform T_front_1;
		geometry::DHTransform T_1_2;
		geometry::DHTransform T_2_aft;
		geometry::DHTransform T_front_aft;
};


class LinkModule
{
	public:
		LinkModule();
		LinkModule(double l);

		geometry::Pose3 getTransform() const;
		geometry::Pose3 inverse() const;
	private:

		double l;

		geometry::DHTransform T_front_aft;
};

#endif 
