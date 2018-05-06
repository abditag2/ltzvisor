// Stanley Bak
// Pendulum Real-time reach check header

#ifndef PENDULUM_H_
#define PENDULUM_H_

#include "geometry.h"
#include "dynamics_3dof.h"
#include "face_lift.h"
#include "util.h"

// return the potential of the lmi-outputted function for a given state
double potential(double e, double p, double y, double de, double dp, double dy);

// check if the passed-in state is provably safe
bool isSafe(double state[NUM_DIMS], double reachTimeCC, double reachTimeSC, double* simTime);
double findMaxRestartTime(double state[NUM_DIMS], double reachTimeSC);
bool shouldStop(double state[NUM_DIMS], double simTime, void* p);
bool shouldStopWithSafety(double state[NUM_DIMS], double simTime, void *p, double maxTime);

#endif
