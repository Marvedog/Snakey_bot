#ifndef SNAKE_H
#define SNAKE_H

#include <pkg_utils/dh_convention.h>
#include <pkg_utils/eelume_definitions.h>

#include <string>
#include <vector>

/// Ros dependency
#include <geometry_msgs/TransformStamped.h>

/// Representation of snake and all transformations
/**
	* Note: Both ends have to be end-effector modules
	*/
class Snake
{
	public:
		Snake( std::vector<std::string> snake_config
				 , std::vector<double> d
				 , std::vector<double> a
				 , std::vector<double> alpha
				 , std::vector<double> theta
				 , int base);
		
		/// Updates joint transformations and based on those T_b_i and T_b_i_h
		void updateTransformations(const std::vector<double> &theta);

		/// Debugging functionality
		/**
			* @param vis_level: 
			* 	0 ==> T_joint prints
			* 	0 ==> T_b_i_h prints
			*/
		void printFrames(int viz_level) const;
		
		/// Deep copies of T_joints, T_b_i and T_b_i_h
		void getFrames(std::vector<eelume::Tf> &T_joint, std::vector<eelume::Tf> &T_b_i, std::vector<eelume::Tf> &T_b_i_h) const;
		
		/// Deep copy of all dynamic joint data theta
		int getJoints() const;

	private:
	
		std::string frameName(const std::string &type, int &i);
	
		/// These functions are utilities for configuration of the snake
		/**
			* addJointModule --> Adds three frames. Types: static->dynamic->dynamic
			* addLinkModule --> adds one frame. Type: static
			*/

		/// Adds a joint module during initialization
		void addJointModule(int &frame, int &static_count, int &dynamic_count);
		
		/// Adds a link module during initialization
		void addLinkModule(int &frame, int &static_count);

		/// Adds enf-effector module during initialization
		void addEndEffectorModule(int &frame, int &dynamic_count, bool first_frame);

		/// Computes transforms for T_b_i and T_b_i_home from base to front
		/**
			* TODO: Add end-effector mapping
			*/
		void updateBaseToFront();
		
		/// Computes transforms for T_b_i and T_b_i_home from base to rear
		/**
			* TODO: Add end-effector mapping
			*/
		void updateBaseToRear();

		/// Frame data
		int base_frame;
		int frames;	
		int dynamic_frames;

		/// Dh convention parameters
		std::vector<double> d;
		std::vector<double> a;
		std::vector<double> alpha;
		std::vector<double> theta;

		/// Keeps track of dynamic (revolute joints) and static coordinate frames
		std::vector<std::string> snake_config;
	
		/// Structures for maintaing transforms
		/**
			* Note: All transforms are store in the following sequence
			* 0 <-- 1 <-- .. <-- base_link --> b+1 --> .. --> n-2 --> n-1
			*/
		std::vector<eelume::Tf> _T_joint; //Non-moving base
		std::vector<eelume::Tf> _T_b_i; //Non-moving base
		std::vector<eelume::Tf> _T_b_i_h; //Non-moving base
};

#endif
