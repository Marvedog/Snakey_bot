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

		std::vector<std::string> snake_config;
		std::vector<tf::Transform> joint_tf;
		std::vector<tf::Transform> joint_tf_home;

		void computeJointTransformations(std::vector<double> theta);

	private:
	
		/// Zero indexed
		int base_frame;

		/// One indexed since its a count 	
		int frames;	
		std::vector<double> d;
		std::vector<double> a;
		std::vector<double> alpha;
		std::vector<double> theta;

		tf::Transform T_I_base;
};

#endif
