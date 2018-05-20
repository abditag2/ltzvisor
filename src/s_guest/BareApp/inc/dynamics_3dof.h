// Stanley Bak
// 4-2014
// Controlled Pendulum header

// if this file is included in geometry.h, the controlled pendulum dynamics will be compiled

#ifndef DYNAMICS_PENDULUM_H_
#define DYNAMICS_PENDULUM_H_

#define DYNAMICS_PENDULUM

#define NUM_DIMS (6)

//#define U_MAX (0.3)
//#define U_MIN (-0.3)

#define U_MAX (2.0)
#define U_MIN (-2.0)


#define COMPLEX_CONTROLLER (0)
#define SIMPLE_CONTROLLER (1)


//#include "face_lift.h"
#include "geometry.h"


#endif
