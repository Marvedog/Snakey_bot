#include <pkg_utils/trajectory.h>
#include <pkg_utils/eigen_helpers.h>

namespace utils
{
	Trajectory::Trajectory()
	{}
	
	Trajectory::Trajectory(Spline path, Spline speed)
	: path(path)
	, speed(speed)
	{}
}
