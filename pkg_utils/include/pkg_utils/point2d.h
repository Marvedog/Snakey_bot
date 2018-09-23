#ifndef Point2D_H
#define Point2D_H

#include <math.h>

namespace Utils
{
	class Point2d
	{
		public: 
			Point2d()
			: x(0)
			, y(0)	
			{;}

			Point2d(double x, double y)
			: x(x)
			, y(y)
			{;}

			/* Distance between two points */
			inline double 
			distanceTo(Point2d p2) 
			{
				return sqrt((this->x-p2.x)*(this->x-p2.x) + (this->y-p2.y)*(this->y-p2.y));}

			double x;
			double y;
	};
} /* end namespace Utils */

#endif
