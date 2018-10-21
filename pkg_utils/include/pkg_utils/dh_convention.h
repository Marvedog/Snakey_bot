#ifndef DH_CONVENTION_H
#define DH_CONVENTION_H

//#include <pkg_utils/Pose3.h>
#include <tf/tf.h>

namespace geometry
{

class Dhtf
{
	public:
		Dhtf();
		Dhtf(const double &d, const double &r, const double &alpha, const double &theta);

		tf::Transform tf;

};

Dhtf::Dhtf()
: tf(tf::Matrix3x3(1,0,0,0,1,0,0,0,1), tf::Vector3(0,0,0))
{}

Dhtf::Dhtf(const double &d, const double &r, const double &alpha, const double &theta)
	: tf(  tf::Matrix3x3( cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha)
											, sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha)
											, 0, sin(alpha), cos(alpha)
											)
			,  tf::Vector3( r*cos(theta), r*sin(theta), d )
			)
					
{}

} /// end namespace geometry

#endif
