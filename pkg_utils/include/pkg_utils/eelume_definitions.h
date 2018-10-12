#ifndef EELUME_DEFINITIONS_H
#define EELUME_DEFINITIONS_H

#include <Eigen/Dense>
#include <math.h>

namespace eelume
{
	
	/// Transformation with DH convention
	inline Eigen::Matrix4d
	homTransform(const double &d, const double &r, const double &alpha, const double &theta)
	{
		Eigen::Matrix4d transform;
		transform(0,0) = cos(theta);
		transform(1,0) = sin(theta);
		transform(0,1) = -sin(theta)*cos(alpha);
		transform(1,1) = cos(theta)*cos(alpha);
		transform(2,1) = sin(alpha);
		transform(0,2) = sin(theta)*sin(alpha);
		transform(1,2) = -cos(theta)*sin(alpha);
		transform(2,2) = cos(alpha);
		transform(0,3) = r*cos(theta);
		transform(1,3) = r*sin(theta);
		transform(2,3) = d;
		transform(3,3) = 1;
	}	

	namespace snake_1
	{
		int number_links = 2;

		/// Two joints per joint module
		int number_joints = 2;

		/// Length of links
		/** Starting from head */
		Eigen::VectorXd link_lengths;
		link_lengths << 0.20,
										0.80;

		Eigen::Matrix4d T_0_1 = homTransform(0, 20, 0, 0);
		Eigen::Matrix4d T_2_B = homTransform(0, 80, 0, 0);
		
		/// Find position of head relative to tail
		/** Given tail position and transform for head to tail find head pos and orientation
			*/
		inline void
		transformPos( const Eigen::Vector4d &pos_in, const Eigen::Matrix4d &tf, Eigen::Vector4d &pos_out)
		{
			pos_out.noalias() = tf*pos_in;
		}
	
	} /* end namespace snake_1 */

} /* end namespace eelume */
#endif
