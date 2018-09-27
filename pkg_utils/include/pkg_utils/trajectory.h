#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H
#include <pkg_utils/spline.h>

class Trajectory {
public:
    Trajectory();
    Trajectory(Spline path, Spline speed);

    Spline path;
    Spline speed;
};
#endif
