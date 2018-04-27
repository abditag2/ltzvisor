// Stanley Bak
// 4-2014
// Simulate Header

#ifndef SIMULATE_H_
#define SIMULATE_H_

#include "dynamics.h"
#include "geometry.h"
#include <stdbool.h>

// simulateSC dynamics using Eurler's method
void simulateSC(double point[NUM_DIMS],
				double stepSize,
				bool (*shouldStop)(double state[NUM_DIMS], double simTime, void *p),
				void *param);

void simualteCC(double startPoint[NUM_DIMS], double stepSize, double length, double * finalPoints);

#endif
