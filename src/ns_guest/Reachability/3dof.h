// Stanley Bak
// Pendulum Real-time reach check header

#ifndef PENDULUM_H_
#define PENDULUM_H_

#include <stdbool.h>
#include "geometry.h"
#include "dynamics_3dof.h"

// return the potential of the lmi-outputted function for a given state
double potential(double e, double p, double y, double de, double dp, double dy);

// check if the passed-in state is provably safe
bool isSafe(double state[NUM_DIMS], double reachTimeCC, double reachTimeSC, double* simTime, double stressSoFar, double * maxOR);
double findMaxRestartTime(double state[NUM_DIMS]);
double getSimulatedSafeTime(double start[NUM_DIMS], double reachTimeCC);
bool shouldStop(double state[NUM_DIMS], double simTime, void* p);
bool shouldStopWithSafety(double state[NUM_DIMS], double simTime, void *p);
void prvSeiRestartTask(void *pvParameters);

#endif
