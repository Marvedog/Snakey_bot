#include <pkg_utils/snake.h>
#include <iostream>

Snake::Snake(  std::vector<std::string> snake_config
						 , std::vector<double> d
						 , std::vector<double> a
						 , std::vector<double> alpha
						 , std::vector<double> theta
						 , int base)
: snake_config(snake_config)
, d(d)
, a(a)
, alpha(alpha)
, theta(theta)
{
	/// Count number of frames necessary to store in memory 
	this->frames = 0;
	for (auto& module: this->snake_config)		
	{
		if (module == eelume::joint_module)
		{
			this->frames += eelume::JM_frames;
		}
		if (module == eelume::link_module)
		{
			this->frames += eelume::L_frames;
		}
	}
	
	/// Error check all sizes
	if (this->d.size() != this->frames)
	{
		std::cerr << "Snake constructor:: d is to small!" << std::endl;
		return;
	}
	
	if (this->a.size() != this->frames)
	{
		std::cerr << "Snake constructor:: a is to small!" << std::endl;
		return;
	}
	
	if (this->alpha.size() != this->frames)
	{
		std::cerr << "Snake constructor:: alpha is to small!" << std::endl;
		return;
	}
		
	if (this->theta.size() != this->frames)
	{
		std::cerr << "Snake constructor:: theta is to small!" << std::endl;
		return;
	}

	/// Error check base within range
		std::cout << base << std::endl;
	if (base < 0 || base > this->frames)
	{
		std::cerr << "Snake constructor:: base is out of range!" << std::endl;
		return;
	}
	this->base_frame = base;

	/// Pre-allocate joint transformation containers
	this->joint_tf.resize(this->frames);
	this->joint_tf_home.resize(this->frames);

	/// Initialize base tf
	this->joint_tf[this->base_frame] = geometry::Dhtf().tf;
}

void
Snake::computeJointTransformations(std::vector<double> theta)
{
	/// Forward iteration ()
	/// TODO: Add home transformations
	std::cout << "her" << std::endl;
	for (int i = this->base_frame; i >= 0; i--)
	{
		std::cout << i << std::endl;
		tf::Transform T_prev_i = geometry::Dhtf( this->d[i-1], this->a[i-1], this->alpha[i-1], theta[i-1]).tf;
		std::cout << i << std::endl;
		this->joint_tf[i-1] = joint_tf[i] * T_prev_i.inverse();
	}
	
	/// Backward iteration
	/// TODO: Add home transformations
	for (int i = this->base_frame; i < this->frames-1; i++)
	{
		std::cout << i << std::endl;
		std::cout << "d" << d[i] << std::endl;
		std::cout << "a" << a[i] << std::endl;
		std::cout << "alpha" << alpha[i] << std::endl;
		std::cout << "theta" << theta[i] << std::endl;
		tf::Transform T_prev_i = geometry::Dhtf( this->d[i+1], this->a[i+1], this->alpha[i+1], theta[i+1]).tf;
		this->joint_tf[i+1] = joint_tf[i] * T_prev_i;
	}
}
