// Fardin Abdi
// 4-2016

// if this file is included in geometry.h, the controlled pendulum dynamics will be compiled

#ifndef DYNAMICS_PENDULUM_H_
#define DYNAMICS_PENDULUM_H_

#define DYNAMICS_PENDULUM

#include "geometry.h"

int fill_in_the_critical_points_3dof(double (*points)[6], HyperRectangle *rect);
double get_derivative_bounds(HyperRectangle *rect, int faceIndex, int controller_type);
double find_F_max(HyperRectangle* rect);


#endif
