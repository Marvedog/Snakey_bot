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
	/// -----------------------------------------------------------	
	/// Configures the number of frames existing on the snake
	/// -----------------------------------------------------------	

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
		else if (module == eelume::end_effector_module)
		{
			this->frames += eelume::EE_frames;
			this->dynamic_frames += eelume::EE_frames;
		}
		else
		{
			std::cerr << "Snake constructor:: <" << module << "> is not a module!" << std::endl;
		}
	}

	std::cout << "Number of frames: " << this->frames << std::endl;

	/// -----------------------------------------------------------	
	/// Error check all input sizes
	/// -----------------------------------------------------------	

	if (this->d.size() != this->frames)
	{
		std::cerr << "Snake constructor:: d is to small!" << std::endl;
		std::cerr << "Snake constructor:: d is: " << this->d.size() << std::endl;
		std::cerr << "Snake constructor:: d should be: " << this->frames << std::endl;
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

	/// -----------------------------------------------------------	
	/// Error check base within range
	/// -----------------------------------------------------------	
	
	std::cout << base << std::endl;
	if (base < 0 || base > this->frames)
	{
		std::cerr << "Snake constructor:: base is out of range!" << std::endl;
		return;
	}
	this->base_frame = base;

	/// -----------------------------------------------------------	
	/// Pre-allocate joint transformation containers
	/// -----------------------------------------------------------	

	this->_T_joint.resize(this->frames);
	this->_T_b_i.resize(this->frames);
	this->_T_b_i_h.resize(this->frames);

	/// -----------------------------------------------------------	
	/// Initialize base tf
	/// -----------------------------------------------------------	
	
	this->_T_joint[this->base_frame].tf = geometry::Dhtf().tf;
	
	/// -----------------------------------------------------------	
	/// Initiate all transformations
	/** 
		* This part loads the initial dh parameters specified in the snake configuration
		* file and stores all frame definitions.
		* 
		* It is for now assumed that there are no end-effector joints
		* TODO: Add end-effectors
		* TODO: Add thruster modules
		* TODO: Add sensor modules
		*/
	/// -----------------------------------------------------------	

	int i = 0;
	int dynamic_count = 0;
	int static_count = 0;
	
	for (auto& module: this->snake_config)
	{
		if (module == eelume::joint_module)
		{
			this->addJointModule(i, static_count, dynamic_count);
		}
		else if (module == eelume::link_module)
		{ 
			this->addLinkModule(i, static_count);
		}
		else if (module == eelume::end_effector_module)
		{
			if (i == 0)
				this->addEndEffectorModule(i, dynamic_count, true);
			else if (i == this->frames-1) 
				this->addEndEffectorModule(i, dynamic_count, false);
			else
			{
				std::cerr << "End_effector module cannot be placed her! " << std::endl;
				return;
			}
		}
		
		if (i > this->frames)
		{
			ROS_ERROR_STREAM("Too many modules for configuration");
			return;
		}
	}

	/// -----------------------------------------------------------	
	/// Dynamic joint distribution
	/// -----------------------------------------------------------	
	this->dynamic_frames_front = 0;
	this->dynamic_frames_rear = 0;
	for (int i = 0; i < this->frames; i++)
	{
		if (i < this->base_frame && this->_T_joint[i].type == eelume::joint)
			this->dynamic_frames_front++;
		if (i > this->base_frame && this->_T_joint[i].type == eelume::joint)
			this->dynamic_frames_rear++;
	}
	
	if (this->dynamic_frames_front + this->dynamic_frames_rear != this->dynamic_frames)
	{
		std::cerr << "Sum of dynamic frames front and rear doesn't match all dynamic frames" << std::endl;
		std::cerr << "Dynamic frames front: " << this->dynamic_frames_front << std::endl;
		std::cerr << "Dynamic frames rear: " << this->dynamic_frames_rear << std::endl;
		std::cerr << "Dynamic frames total: " << this->dynamic_frames << std::endl;
		return;
	}	


	/// -----------------------------------------------------------	
	/// Set base link
	/// -----------------------------------------------------------	

	this->_T_joint[this->base_frame].frame_id = "base_link";
	this->_T_joint[this->base_frame-1].child_id = "base_link";
	
	for (int i = this->base_frame-1; i >= 0; i--)
	{
		this->_T_joint[i].tf = this->_T_joint[i].tf.inverse();
		std::string tmp = this->_T_joint[i].frame_id;
		this->_T_joint[i].frame_id = this->_T_joint[i].child_id;
		this->_T_joint[i].child_id = tmp;
	}

	/// -----------------------------------------------------------	
	/// Init transforms
	/// -----------------------------------------------------------	
	
	for(int i = 0; i < this->frames; i++)
	{
		this->_T_b_i[i].tf.setIdentity();
		this->_T_b_i_h[i].tf.setIdentity();
	}

	this->updateBaseToFront();
	this->updateBaseToRear();
	this->printFrames(-1);
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
Snake::updateTransformations(const std::vector<double> &theta)
{
	int i = 0;
	for (int j = 0; j < this->_T_joint.size(); j++)
	{
		if (this->_T_joint[j].type == eelume::joint)
		{
			if (j < this->base_frame)
				this->_T_joint[j].tf = geometry::Dhtf(this->d[j], this->a[j], this->alpha[j], theta[i]).tf.inverse();
			else
				this->_T_joint[j].tf = geometry::Dhtf(this->d[j], this->a[j], this->alpha[j], theta[i]).tf;
			this->theta[j] = theta[i];
			i++;
		}
	}
	
	/// ----------------------------------------------------
	/// Initialize base link transformations
	/// ----------------------------------------------------
	
	/// ----------------------------------------------------
	/// Add rotation only of joint base fram transform to base frame
	/** 
		* T_b_i is the accumulated transforms from base link to each coordinate frame
		* with the rotation of the dynamic joints accounted for. Base to front will
		* therefore be straight-forward, while the base to back will require adding the
		* rotation of the next joint transformation to the current T_b_i transform.
		*
		* Note again that base link of T_b_i is rotated with the joint if its revolute
		* or has a static rotation
		*/
	/// ----------------------------------------------------
	
	tf::Transform tf_init_b_i( this->_T_joint[this->base_frame].tf.getBasis()
											 		 , tf::Vector3(0.0,0.0,0.0));
	this->_T_b_i[this->base_frame].tf = tf_init_b_i;
	
	
	/// ----------------------------------------------------
	/// Add translation only of joint base frame transform
	/**
		* Only translation is added because base link is assumed static
		* and the snake moves relative to the base. Therefore all
		* kinematic movement is accumulated into the front and rear expressions.
		*/
	/// ----------------------------------------------------
	tf::Transform tf_init_b_i_h;
	tf_init_b_i_h.setBasis(this->_T_joint[this->base_frame].tf.getBasis());
	//tf_init_b_i_h.setOrigin(this->_T_joint[this->base_frame-1].tf.getOrigin());
	//this->_T_b_i_h[this->base_frame-1].tf = tf_init_b_i_h;

	
	/// Update front starting from the frame coming before base link (again base i static)	
	if (this->base_frame > 0)
	{
		this->updateBaseToFront();
	}

	this->updateBaseToRear();
	this->printFrames(-1);
}

void
Snake::updateBaseToFront()
{
	for (int i = this->base_frame-1; i >= 0; i--)
	{
		/// Define translation
		tf::Transform T_i_tr;
		T_i_tr.setBasis(geometry::Dhtf(0, 0, this->alpha[i], 0).tf.inverse().getBasis());
		T_i_tr.setOrigin(this->_T_joint[i].tf.getOrigin());
	
		/// T_b_i_h
		/**
		 	* We want the rotation from everything before this frame and the translation
			* to this frame, aka the transformation upto the origin of this frame.
			* Since we work our way backwards, the upto now is encoded in R_i,b and the translation
			* to this frame is encoded in t_i,i+1.
		 	* T_b,i_home = (T_i,i+1_home * T_i+1,b_home)^(-1)
			*						 = (T_i+1,b_home)^(-1) * (T_i,i+1_home)^(-1) 
			*/
		this->_T_b_i_h[i].tf = this->_T_b_i[i+1].tf * T_i_tr;
			
		/// T_b_i
		/**
			* We want the rotation of the previous frames and the translation to the previous frame
			* T_b-1,i = (T_i,i+1 * T_i+1,b)^(-1)
			* 			  = (T_i+1,b)^(-1) * (T_i,i+1)^(-1)
			*/
		this->_T_b_i[i].tf = this->_T_b_i_h[i].tf * geometry::Dhtf(0, 0, 0, this->theta[i]).tf.inverse();
	}
}

void
Snake::updateBaseToRear()
{
	if (this->base_frame >= this->frames)
	{
		std::cout << "No transformations from base to rear to consider" << std::endl;
		return;
	}

	for (int i = this->base_frame + 1; i < this->frames; i++)
	{
		/// Define rotation only
		tf::Transform T_i_rot;
		tf::Matrix3x3 rot_i = geometry::Dhtf(0, 0, 0, this->theta[i]).tf.getBasis();

		/// T_b_i_h
		/**
			* We want to deploy the same principle as for the base to front.
			* The difference now is that we don't want to invert forward transformations
			* T_b,i_home = T_b,i-1_home * T_i-1,i
			*/
		this->_T_b_i_h[i].tf = this->_T_b_i_h[i-1].tf * this->_T_joint[i-1].tf;

		/// T_b_i
		/**
			* We want to rotate the current home transformation to obtain the 
			* full transformation to this frame 
			* T_b,i = T_b,i-1_home * T_b,i
			*/
		this->_T_b_i[i].tf = tf::Transform( this->_T_b_i_h[i].tf.getBasis()*rot_i
				                              , this->_T_b_i_h[i].tf.getOrigin());
	}
}

void
Snake::printFrames(int viz_level) const
{
	if (viz_level == -1)
		return;
	std::cout << "----------- Frame names from base to back -----------" << std::endl; 

	for (int i = this->base_frame; i < this->frames; i++)
 	{
		if (viz_level == 0)
		{
			tf::Matrix3x3 mat = this->_T_joint[i].tf.getBasis();
			tf::Vector3 vec = this->_T_joint[i].tf.getOrigin();
			std::cout << "Joint transforms!" << std::endl;	
			std::cout << "frame_id: " << this->_T_joint[i].frame_id << std::endl;
			std::cout << "child_id: " << this->_T_joint[i].child_id << std::endl;
			std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " " << vec[0] << std::endl;
			std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << " " << vec[1] << std::endl;
			std::cout << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << " " << vec[2] << std::endl;
		}

		if (viz_level == 1)
		{
			tf::Matrix3x3 mat = this->_T_b_i_h[i].tf.getBasis();
			tf::Vector3 vec = this->_T_b_i_h[i].tf.getOrigin();
			std::cout << "Full home transforms transforms!" << std::endl;	
			std::cout << "frame_id: " << this->_T_b_i_h[i].frame_id << std::endl;
			std::cout << "child_id: " << this->_T_b_i_h[i].child_id << std::endl;
			std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " " << vec[0] << std::endl;
			std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << " " << vec[1] << std::endl;
			std::cout << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << " " << vec[2] << std::endl;
		}
	}
	
	std::cout << "------------------------------------------------------" << std::endl;
	std::cout << "----------- Frame names from base to front -----------" << std::endl;

 	for (int i = 0; i < this->base_frame; i++)
 	{
		if (viz_level == 0)
		{
			tf::Matrix3x3 mat = this->_T_joint[i].tf.getBasis();
			tf::Vector3 vec = this->_T_joint[i].tf.getOrigin();
			std::cout << "Joint transforms!" << std::endl;	
			std::cout << "frame_id: " << this->_T_joint[i].frame_id << std::endl;
			std::cout << "child_id: " << this->_T_joint[i].child_id << std::endl;
			std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " " << vec[0] << std::endl;
			std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << " " << vec[1] << std::endl;
			std::cout << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << " " << vec[2] << std::endl;
		}	
		
		if (viz_level == 1)
		{
			tf::Matrix3x3 mat = this->_T_b_i_h[i].tf.getBasis();
			tf::Vector3 vec = this->_T_b_i_h[i].tf.getOrigin();
			std::cout << "Full home transforms transforms!" << std::endl;	
			std::cout << "frame_id: " << this->_T_b_i_h[i].frame_id << std::endl;
			std::cout << "child_id: " << this->_T_b_i_h[i].child_id << std::endl;
			std::cout << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << " " << vec[0] << std::endl;
			std::cout << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << " " << vec[1] << std::endl;
			std::cout << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << " " << vec[2] << std::endl;
		}
	}
	
	std::cout << "------------------------------------------------------" << std::endl;
}

void 
Snake::getFrames(std::vector<eelume::Tf> &T_joint, std::vector<eelume::Tf> &T_b_i, std::vector<eelume::Tf> &T_b_i_h) const
{
	T_joint = this->_T_joint;
	T_b_i = this->_T_b_i;
	T_b_i_h = this->_T_b_i_h;
}

int
Snake::getJoints() const
{
	return this->dynamic_frames;
}

int
Snake::getFrames() const
{
	return this->frames;
}

int
Snake::getDynamicFramesFront() const
{
	return this->dynamic_frames_front;
}

int
Snake::getDynamicFramesRear() const
{
	return this->dynamic_frames_rear;
}

///------------------------------------------------
/// Initialization utilities
///------------------------------------------------

void
Snake::addJointModule(int &i, int &static_count, int &dynamic_count)
{
	/// First static link of joint
	this->_T_joint[i].tf = geometry::Dhtf(this->d[i], this->a[i], this->alpha[i], this->theta[i]).tf;
	this->_T_joint[i].frame_id = this->frameName("link_", static_count);
	this->_T_joint[i].type = "static";
	
	/// Help previous frame
	this->_T_joint[i-1].child_id = this->_T_joint[i].frame_id;
	
	this->_T_b_i[i].frame_id = "base_link";
	this->_T_b_i[i].child_id = this->frameName("link_base_", static_count);
	
	this->_T_b_i_h[i].frame_id = "base_link";
	this->_T_b_i_h[i].child_id = this->frameName("link_home_", static_count);

	static_count++;
	i++;

	/// First revolute joint
	this->_T_joint[i].tf = geometry::Dhtf(this->d[i], this->a[i], this->alpha[i], this->theta[i]).tf;
	this->_T_joint[i].frame_id = this->frameName("joint_", dynamic_count);
	this->_T_joint[i].type = "dynamic";
	
	/// Help previous frame
	this->_T_joint[i-1].child_id = this->_T_joint[i].frame_id;
	
	this->_T_b_i[i].frame_id = "base_link";
	this->_T_b_i[i].child_id = this->frameName("joint_base_", dynamic_count);
	
	this->_T_b_i_h[i].frame_id = "base_link";
	this->_T_b_i_h[i].child_id = this->frameName("joint_home_", dynamic_count);
	
	dynamic_count++;
	i++;
	
	/// Second revolute joint
	this->_T_joint[i].tf = geometry::Dhtf(this->d[i], this->a[i], this->alpha[i], this->theta[i]).tf;
	this->_T_joint[i].frame_id = this->frameName("joint_", dynamic_count);
	this->_T_joint[i].type = "dynamic";
	
	/// Help previous frame
	this->_T_joint[i-1].child_id = this->_T_joint[i].frame_id;
			
	this->_T_b_i[i].frame_id = "base_link";
	this->_T_b_i[i].child_id = this->frameName("joint_base_", dynamic_count);
	
	this->_T_b_i_h[i].frame_id = "base_link";
	this->_T_b_i_h[i].child_id = this->frameName("joint_home_", dynamic_count);
	
	dynamic_count++;
	i++;
}


void
Snake::addLinkModule(int &i, int &static_count)
{
	this->_T_joint[i].tf = geometry::Dhtf(this->d[i], this->a[i], this->alpha[i], this->theta[i]).tf;
	this->_T_joint[i].frame_id = this->frameName("link_", static_count);
	this->_T_joint[i-1].child_id = this->_T_joint[i].frame_id;
	this->_T_joint[i].type = "static";
	
	this->_T_b_i[i].frame_id = "base_link";
	this->_T_b_i[i].child_id = this->frameName("link_base_", static_count);
	
	this->_T_b_i_h[i].frame_id = "base_link";
	this->_T_b_i_h[i].child_id = this->frameName("link_home_", static_count);
	
	static_count++;
	i++;
}

void
Snake::addEndEffectorModule(int &i, int &dynamic_count, bool first_frame)
{
	this->_T_joint[i].tf = geometry::Dhtf(this->d[i], this->a[i], this->alpha[i], this->theta[i]).tf;
	this->_T_joint[i].type = "dynamic";

	if (first_frame)
	{
		this->_T_joint[i].frame_id = this->frameName("end_effector_", dynamic_count);
	}
	else
	{
		this->_T_joint[i].frame_id = this->frameName("end_effector_", dynamic_count);
		this->_T_joint[i-1].child_id = this->_T_joint[i].frame_id;
	}

	this->_T_b_i[i].frame_id = "base_link";
	this->_T_b_i[i].child_id = this->frameName("end_effector_base_", dynamic_count);
	
	this->_T_b_i_h[i].frame_id = "base_link";
	this->_T_b_i_h[i].child_id = this->frameName("end_effector_home_", dynamic_count);
	
	dynamic_count++;
	i++;
}
