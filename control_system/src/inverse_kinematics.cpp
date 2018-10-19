#include <control_system/inverse_kinematics.h>
#include <iostream>

InverseKinematics::InverseKinematics() {;}

InverseKinematics::InverseKinematics( int links
		 																, int joints
																		, std::vector<double> a0_joints
																		, std::vector<double> a1_joints
																		, std::vector<double> a2_joints
																		, std::vector<double> q1_joints
																		, std::vector<double> q2_joints
																		, std::vector<double> alpha0_joints
																		, std::vector<double> l_links
																		, std::vector<int> snake_config) 
: snake_config(snake_config)
{
	/*
	/// Parameter checking
	if (!(links + joints == snake_config.size()))
		std::cerr << "snake config and number of links and joints not compatible!" << std::endl;

	/// Allocate container sizes
	this->link_modules.resize(links);
	this->joint_modules.resize(joints);
	
	/// Number of joints. 3 joints per joint module
	this->base_to_joints.resize(3*joints);
	this->base_to_joints_zero.resize(3*joints+2); /// includes end-effectors 

	/// Construct initial link transformations
	for (int i = 0; i < links; i++)
	{
		this->link_modules[i] = LinkModule(l_links[i]);
	}
	
	/// Construct initial joint module transformations 
	for (int i = 0; i < joints; i++)
	{
		this->joint_modules[i] = JointModule(a0_joints[i], a1_joints[i], a2_joints[i], q1_joints[i], q2_joints[i], alpha0_joints[i]);
	}

	int it = 0;
	int joint = 0;
	int link = 1; /// TODO: ADD more intuitive fix for starting at CoM head

	/// Construct initial base to joint transformations
	for (int i = 0; i < this->joint_modules.size(); i++)
	{
		Eigen::Matrix4d tmp = Eigen::Matrix4d::Zero();
		for (it; it < this->snake_config.size(); it++)
		{
			if (this->snake_config[it] == 1) /// link
			{	
				tmp *= this->link_modules[link].getTransform();
				link++;
			}
			else if (this->snake_config[it] == 0) /// Found the next joint
			{
				this->base_to_joints[i*3 + 0] = tmp * this->joint_modules[joint].getTransform(0);
				this->base_to_joints[i*3 + 1] = this->base_to_joints[i*3+0] * this->joint_modules[joint].getTransform(1);
				this->base_to_joints[i*3 + 2] = this->base_to_joints[i*3+1] * this->joint_modules[joint].getTransform(2);
				joint++;
				break;
			}
		}
	}

	it = 0;
	joint = 0;
	link = 1; /// TODO: ADD more intuitive fix for starting at CoM head
	
	/// Construct initial base to joint transformations
	for (int i = 0; i < links; i++)
	{
		Eigen::Matrix4d tmp = Eigen::Matrix4d::Zero();
		for (it; it < this->snake_config.size(); it++)
		{
			if (this->snake_config[it] == 1) /// link
			{	
				tmp *= this->link_modules[link].getTransform();
				link++;
			}
			else if (this->snake_config[it] == 0) /// Found the next joint
			{
				this->base_to_joints_zero[i*3 + 0] = tmp * this->joint_modules[joint].getTransform(0);
				this->base_to_joints_zero[i*3 + 1] = this->base_to_joints_zero[i*3+0] * this->joint_modules[joint].getTransform(1);
				this->base_to_joints_zero[i*3 + 2] = this->base_to_joints_zero[i*3+1] * this->joint_modules[joint].getTransform(2);
				joint++;
				break;
			}
		}
	}
	// TODO: Find a better way to include this in the loop
	this->base_to_joints_zero[3*joints+1] = this->link_modules[links-1].getTransform();
	*/
}

Eigen::MatrixXd
InverseKinematics::computeGeometricJacobian( const std::vector<double> &q1
																					 , const std::vector<double> &q2)
{
	
}

Eigen::Matrix4d
InverseKinematics::baseToTail()
{	
	/*
	int link = 0;
	int joint = 0;
	Eigen::Matrix4d T_base_tail;
	T_base_tail << 1, 0, 0, 0,
								 0, 1, 0, 0,
								 0, 0, 1, 0,
								 0, 0, 0, 1;

 	for (auto& i: this->snake_config)
	{
		if (i == 0) /// joint	
		{
		//	T_base_tail *= this->joint_modules[joint].getTransform();
			joint++;
		}
		else if (i == 1) /// link	
		{
		//	T_base_tail *= this->link_modules[link].getTransform();
		 	link++;	
		}
	}
	return T_base_tail;
	*/
}

Eigen::Matrix4d
InverseKinematics::tailToBase()
{	
	/*
	//int link = this->link_modules.size();
	//int joint = this->joint_modules.size();
	Eigen::Matrix4d T_tail_base;
	T_tail_base << 1, 0, 0, 0,
								 0, 1, 0, 0,
								 0, 0, 1, 0,
								 0, 0, 0, 1;

 	for (int i = this->snake_config.size() - 1; i >= 0; i--)
	{
		if (this->snake_config[i] == 0) /// joint	
		{
			//T_tail_base *= this->joint_modules[joint].inverse();
			joint--;
		}
		else if (this->snake_config[i] == 1) /// link	
		{
			//T_tail_base *= this->link_modules[link].inverse();
		 	link--;	
		}
	}
	return T_tail_base;
	*/
}

Eigen::Matrix4d
InverseKinematics::transformBetween(int a, int b)
{
	/*
	Eigen::Matrix4d tf;
	tf << 1, 0, 0, 0,
		  	0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

	if (a < b)
	{
		int joint = 0;
		int link = 0;
		for (int i = 0; i < a; i++)
		{
			if (this->snake_config[i] == 0) ///joints
				joint++;
			else if (this->snake_config[i] == 1) ///links
				link++;
		}
		for (int i = a; i < this->snake_config.size(); i++)
		{	
			if (this->snake_config[i] == 0) ///joints
			{
				//tf *= this->joint_modules[joint].getTransform();
				joint++;
			}
			else if (this->snake_config[i] == 1) ///links
			{
				//tf *= this->link_modules[joint].getTransform();
				link++;
			}
		}
	}
	else if (a > b)
	{
		//int joint = this->joint_modules.size() - 1;
		//int link = this->link_modules.size() - 1;
		for (int i = this->snake_config.size(); i > b; i--)
		{
			if (this->snake_config[i] == 0) ///joints
				joint--;
			else if (this->snake_config[i] == 1) ///links
				link--;
		}
		for (int i = b; i >= 0; i--)
		{	
			if (this->snake_config[i] == 0) ///joints
			{
				//tf *= this->joint_modules[joint].inverse();
				joint--;
			}
			else if (this->snake_config[i] == 1) ///links
			{
				//tf *= this->link_modules[joint].inverse();
				link--;
			}
		}
	}
	return tf;
	*/
}

int 
main(int argv, char** argc)
{
	return 0;
}
