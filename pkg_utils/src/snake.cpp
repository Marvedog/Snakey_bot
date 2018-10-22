#include <pkg_utils/snake.h>
#include <iostream>
#include <sstream>

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
	this->dynamic_frames = 0;
	for (auto& module: this->snake_config)		
	{
		if (module == eelume::joint_module)
		{
			this->frames += eelume::JM_frames;
			this->dynamic_frames += eelume::JM_frames-1;
		}
		else if (module == eelume::link_module)
		{
			this->frames += eelume::L_frames;
		}
		else if (module == eelume::connector)
		{
			this->frames += eelume::C_frames;
		}
	}

	std::cout << "Number of frames: " << this->frames << std::endl;
	
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
	this->frame_class.resize(this->frames);
	this->frame_names.resize(this->frames);

	/// Initialize base tf
	this->joint_tf[this->base_frame] = geometry::Dhtf().tf;
	
	/// Initiate all transformations
	int i = 0;
	int dynamic_f = 0;
	int static_f = 0;
	for (auto& module: this->snake_config)
	{
		if (module == eelume::joint_module)
		{
			this->joint_tf[i] = geometry::Dhtf(d[i], a[i], alpha[i], theta[i]).tf;
			this->frame_names[i] = this->frameName("link_", static_f);
			this->frame_class[i] = "static";
			static_f++;
			i++;
			
			this->joint_tf[i] = geometry::Dhtf(d[i], a[i], alpha[i], theta[i]).tf;
			this->frame_names[i] = this->frameName("joint_", dynamic_f);
			this->frame_class[i] = "dynamic";
			dynamic_f++;
			i++;
			
			this->joint_tf[i] = geometry::Dhtf(d[i], a[i], alpha[i], theta[i]).tf;
			this->frame_names[i] = this->frameName("joint_", dynamic_f);
			this->frame_class[i] = "dynamic";
			dynamic_f++;
			i++;
		}
		else if (module == eelume::link_module)
		{ 
			std::cout << i << std::endl;	
			this->joint_tf[i] = geometry::Dhtf(d[i], a[i], alpha[i], theta[i]).tf;
			this->frame_names[i] = frameName("link_", static_f);
			this->frame_class[i] = "static";
			static_f++;
			i++;
		}

		/// Add last frame for visualization
		if (i == this->frames-1)
		{
			std::cout << i << std::endl;	
			this->joint_tf[i] = geometry::Dhtf(0, 0, 0, 0).tf;
			this->frame_names[i] = frameName("link_", static_f);
			this->frame_class[i] = "static";
			static_f++;
			i++;
		}
	}
	this->printFrames();
}

std::string
Snake::frameName(const std::string &type, int &i)
{
	std::stringstream ss;
	std::string tmp_s;
	ss << i;
	ss >> tmp_s;
	return type + tmp_s;
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

void 
Snake::updateTransformations(const std::vector<double> &theta)
{
	int i = 0;
	for (int j = 0; j < this->frame_class.size(); j++)
	{
		if (this->frame_class[j] == eelume::joint)
		{
			this->joint_tf[j] = geometry::Dhtf(this->d[j], this->a[j], this->alpha[j], theta[i]).tf;
			this->theta[j] = theta[i];
			i++;
		}
	}
}

void
Snake::printFrames() const
{
	std::cout << "----------- Frame names from front to back -----------" << std::endl;
 	for (int i = 0; i < this->frames; i++)
 	{
		tf::Matrix3x3 mat = this->joint_tf[i].getBasis();

		std::cout << this->frame_names[i] << std::endl;
		std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << std::endl;
		std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << std::endl;
		std::cout << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << std::endl;
	}
	std::cout << "------------------------------------------------------" << std::endl;
}

void
Snake::getFrames(std::vector<tf::Transform> &tf, std::vector<std::string> &names) const
{
	tf = this->joint_tf;
	std::vector<std::string> tmp = this->frame_names;
	tmp[base_frame] = "base_link";
	names = tmp;
}

int
Snake::getJoints() const
{
	return this->dynamic_frames;
}
