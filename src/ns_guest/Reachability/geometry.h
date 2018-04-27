// Stanley Bak
// 4-2014
// Geometry header for real-time reach

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <stdbool.h>

// pick the dynamics header to compile (this defines NUM_DIMS)
#include "dynamics_3dof.h"

#define NUM_FACES (2 * NUM_DIMS)

typedef struct Interval
{
	double min;
	double max;
} Interval;

// for a HyperPoint use a double[]
typedef struct HyperPoint
{
	double dims[NUM_DIMS];
} HyperPoint;

// for a HyperRectangle use an Interval[]
typedef struct HyperRectangle
{
	Interval dims[NUM_DIMS];
} HyperRectangle;

double interval_width(Interval* i);

bool hyperrectangle_contains(HyperRectangle* outside, HyperRectangle* inside, bool printErrors);
void hyperrectangle_grow_to_convex_hull(HyperRectangle* grower, HyperRectangle* contained);
double hyperrectange_max_width(HyperRectangle* rect);
void hyperrectangle_bloat(HyperRectangle* out, double from[NUM_DIMS], double width);

//void print(HyperRectangle* r);
//void println(HyperRectangle* r);

#endif
