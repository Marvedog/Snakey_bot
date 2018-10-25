#ifndef EELUME_DEFINITIONS_H
#define EELUME_DEFINITIONS_H

#include <string>
#include <tf/tf.h>

namespace eelume
{
	
	/// Joint module joints
	const int JM_frames = 3;

	/// Link module frames 
	const int L_frames = 1;
	
	/// Connector module frames`
	const int C_frames = 1;

	/// End-effector module frames`
	const int EE_frames = 1;
	
	const std::string joint_module = "joint_module";
	
	const std::string link_module = "link_module";
	
	const std::string connector = "connector";
	
	const std::string joint = "dynamic";
	
	const std::string link = "static";

	const std::string thruster_module = "thruster";

	const std::string sensor_module = "sensor";

	const std::string end_effector_module = "end_effector";

	struct Tf
	{
		tf::Transform tf;
		std::string child_id;
		std::string frame_id;
		std::string type;
	};

} /* end namespace eelume */
#endif
