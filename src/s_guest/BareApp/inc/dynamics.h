// Stanley Bak
// 4-2014
// Dynamics header file for real-time reachability

#ifndef DYNAMICS_H_

#define DYNAMICS_H_

#include "geometry.h"

/**
 * Get the bounds on the derivative in a region of space at a range of times
 */

double get_derivative_bounds(HyperRectangle *rect, int faceIndex, int controller_type);
int fill_in_the_critical_points_3dof(double (*points)[6], HyperRectangle *rect);

#endif
