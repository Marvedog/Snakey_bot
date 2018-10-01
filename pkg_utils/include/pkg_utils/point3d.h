#ifndef Point3D_H
#define Point3D_H

#include <math.h>

namespace utils
{
	class Point3d
	{
		public: 
			Point3d()
			: x(0)
			, y(0)	
			, z(0)	
			{;}

			Point3d(double x, double y, double z)
			: x(x)
			, y(y)
			, z(z)
			{;}

			/* Distance between two points */
			inline double 
			distanceTo(Point3d p3) 
			{
				return sqrt(  (this->x-p3.x)*(this->x-p3.x) 
										+ (this->y-p3.y)*(this->y-p3.y) 
										+ (this->z-p3.z)*(this->z-p3.z));
			}

			double x;
			double y;
			double z;
	};
} /* end namespace Utils */

#endif
