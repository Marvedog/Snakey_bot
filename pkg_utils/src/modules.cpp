#include <pkg_utils/modules.h>

JointModule::JointModule()
: JointModule(0.1, 0.1, 0.1, 0, 0, 0)
{;}

JointModule::JointModule(double a0, double a1, double a2, double q1, double q2, double alpha0)
: a0(a0)
, a1(a1)
, a2(a2)
, alpha0(alpha0)
{
	this->T_front_1 = geometry::DHTransform(0, this->a0, this->alpha0, 0);
	this->T_1_2 = geometry::DHTransform(0, this->a1, 0, q1);
	this->T_2_aft = geometry::DHTransform(0, this->a2, 0, q2);

	this->T_front_aft.pose = (this->T_front_1.pose * this->T_1_2.pose * this->T_2_aft.pose);
}

JointModule::JointModule(const JointModule &obj)
{
	this->a0 = obj.a0;
	this->a1 = obj.a1;
	this->a2 = obj.a2;
	this->alpha0 = obj.alpha0;
	this->T_front_1 = obj.T_front_1;
	this->T_1_2 = obj.T_1_2;
	this->T_2_aft = obj.T_2_aft;
	this->T_front_aft = obj.T_front_aft;
}

geometry::Pose3
JointModule::updateTransform(double q1, double q2)
{
	this->T_1_2 = geometry::DHTransform(0, this->a1, 0, q1);
	this->T_2_aft = geometry::DHTransform(0, this->a2,0, q2);
	
	this->T_front_aft.pose = this->T_front_1.pose * this->T_1_2.pose * this->T_2_aft.pose;

	return this->T_front_aft.pose;
}


geometry::Pose3
JointModule::getTransform() const
{
	return this->T_front_aft.pose;
}

geometry::Pose3
JointModule::getTransform(int i) const
{
	if (i == 0)
		return this->T_front_1.pose;
	if (i == 1)
		return this->T_1_2.pose;
	if (i == 2)
		return this->T_2_aft.pose;

	return geometry::Pose3();
}

geometry::Pose3
JointModule::inverse() const
{
	return this->T_front_aft.pose.inverse();
}

geometry::Pose3
JointModule::inverse(int i) const
{
	geometry::Pose3 inv;
	if (i == 0)
	{
		inv = this->T_front_1.pose.inverse();
	}
	if (i == 1)
	{
		inv = this->T_1_2.pose.inverse();
	}
	if (i == 2)
	{
		inv = this->T_2_aft.pose.inverse();
	}
	return inv;
}

LinkModule::LinkModule()
: LinkModule(0.4)
{}


LinkModule::LinkModule(double l)
: l(l)
, T_front_aft(geometry::DHTransform(0, this->l, 0, 0))
{}

geometry::Pose3
LinkModule::getTransform() const
{
	return this->T_front_aft.pose;
}

geometry::Pose3
LinkModule::inverse() const
{
	return this->T_front_aft.pose.inverse();
}
