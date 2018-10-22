#ifndef SNAKE_H
#define SNAKE_H

#include <pkg_utils/dh_convention.h>
#include <pkg_utils/eelume_definitions.h>

#include <string>
#include <vector>

class Snake
{
	public:
		Snake( std::vector<std::string> snake_config
				 , std::vector<double> d
				 , std::vector<double> a
				 , std::vector<double> alpha
				 , std::vector<double> theta
				 , int base);

		void computeJointTransformations(std::vector<double> theta);
		void updateTransformations(const std::vector<double> &theta);
		//void getFrameAssociations(const std::vector<std::string> &frame, std::vector<std::string> &child_frame) const;
		void printFrames() const;
		void getFrames(std::vector<tf::Transform> &tf, std::vector<std::string> &names) const;
		int getJoints() const;

	private:
	
		std::string frameName(const std::string &type, int &i);

		/// Zero indexed
		int base_frame;
		int dynamic_frames;

		/// One indexed since its a count 	
		int frames;	
		std::vector<double> d;
		std::vector<double> a;
		std::vector<double> alpha;
		std::vector<double> theta;

		tf::Transform T_I_base; // Moving base
		std::vector<std::string> snake_config;
		std::vector<tf::Transform> joint_tf; //Non-moving base
		std::vector<std::string> frame_class;
		std::vector<std::string> frame_names;
};

#endif
