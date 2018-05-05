// Stanley Bak
// 4-2014
// Real-time face lifting main algorithm header

#ifndef FACE_LIFT_H_
#define FACE_LIFT_H_

#include <stdbool.h>

#include "geometry.h"
#include "dynamics.h"
#include "util.h"

typedef struct LiftingSettings
{
	HyperRectangle init;

	double reachTimeCC; // total reach time
	double reachTimeSC;


	double initialStepSizeSC; // the initial size of the steps to use
	double initialStepSizeCC; // the initial size of the steps to use

	double maxRectWidthBeforeError; // maximum allowed rectangle size

	int maxRuntimeMilliseconds; // maximum runtime in milliseconds

	// called at the intermediate times
	// return true if this rectangle is satisfactory (for safety or whatever)
	bool (*reachedAtIntermediateTime)(HyperRectangle* r);

	// called at the final time
	// return true if the system is satisfactory (for liveness or whatever)
	bool (*reachedAtFinalTime)(HyperRectangle* r);

	bool (*checkStabilizabilityAfterCCperiod)(HyperRectangle* r, double reachTimeSC);

	// called whenever we restart the computation after refining
	void (*restartedComputation)();

} LiftingSettings;

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)
bool face_lifting_iterative_improvement(LiftingSettings* settings);

// do a face lifting operation with the given settings. This one does NOT do iterative improvement
// or attempt to run within a given time
// instead, it tried to do everything in a single step (so it's best for very short reach times).
// it will return true if the set of states is
// satisfactory according to the function you provide in LiftingSettings
// (reachedAtIntermediateTime, reachedAtFinalTime)
//bool face_lifting_quick_nosplit(LiftingSettings* settings);

#endif
