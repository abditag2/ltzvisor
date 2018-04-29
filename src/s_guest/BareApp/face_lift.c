// Stanley Bak
// 4-2014

#include <stdio.h>

#include "inc/face_lift.h"


double fabs(double x){
	if (x < 0){
		x = -x;
	}

	return x;
}

// make a face's neighborhood of a given width
void make_neighborhood_rect(HyperRectangle* out, int f,
		HyperRectangle* bloatedRect, HyperRectangle* originalRect, double nebWidth)
{
	*out = *bloatedRect;
	bool isMin = (f % 2) == 0;
	int d = f / 2;

	// flatten
	if (isMin)
	{
		out->dims[d].min = originalRect->dims[d].min;
		out->dims[d].max = originalRect->dims[d].min;
	}
	else
	{
		out->dims[d].min = originalRect->dims[d].max;
		out->dims[d].max = originalRect->dims[d].max;
	}

	// swap if nebWidth was negative
	if (nebWidth < 0)
		out->dims[d].min += nebWidth;
	else
		out->dims[d].max += nebWidth;
}

// do a single face lifting operation
// et (error tracker) is set if you want to track the sources of errors, can be null
// returns time elapsed




double lift_single_rect(HyperRectangle* rect, double stepSize, double timeRemaining, int controller_type)
{
	static int debugNumCalls = 0;
	++debugNumCalls;
	//HyperRectangle debug_initialRect = *rect;

	// necessary to guarantee loop termination
	double MAX_DER = 99999;
	double MIN_DER = -99999;

	////////////////////////////////////////////////
	// estimate the widths of the neighborhoods   //
	// construct bloated rect (for neighborhoods) //

	HyperRectangle bloatedRect = *rect;
	double nebWidth[NUM_FACES];

	// initially, estimate nebWidth based on the derivative in the center of the rectangle we care about

	for (int f = 0; f < NUM_FACES; ++f)
		nebWidth[f] = 0;

	bool needRecompute = true;
	double minNebCrossTime;
	double ders[NUM_FACES];

	while (needRecompute)
	{
		needRecompute = false;
		minNebCrossTime = 1000000.0;

		for (int f = 0; f < NUM_FACES; ++f)
		{


			int dim = f / 2;
			bool isMin = (f % 2) == 0;

			HyperRectangle faceNebRect;

			// make candidate neighborhood
			make_neighborhood_rect(&faceNebRect, f, &bloatedRect, rect, nebWidth[f]);

			// test derivative inside neighborhood
			double der = get_derivative_bounds(&faceNebRect, f, controller_type);

			if (der > MAX_DER)
				der = MAX_DER;
			else if (der < MIN_DER)
				der = MIN_DER;

			double prevNebWidth = nebWidth[f];
			double newNebWidth = der * stepSize;

			bool grewOutward = (isMin && newNebWidth < 0) || (!isMin && newNebWidth > 0);
			bool prevGrewOutward = (isMin && prevNebWidth < 0) || (!isMin && prevNebWidth > 0);

			// prevent flipping from outward face to inward face
			if (!grewOutward && prevGrewOutward)
			{
				newNebWidth = 0;
				der = 0;
			}

			// if flipping from inward to outward
			if (!prevGrewOutward && grewOutward)
				needRecompute = true;

			// 2nd condition to recompute, der doubled (which means neb width is twice what it was before)
			if (fabs(newNebWidth) > 2 * fabs(prevNebWidth))
				needRecompute = true;

			// adjust bloated rect only if we are requiring a later recomputation
			if (needRecompute)
			{
				nebWidth[f] = newNebWidth;

				if (isMin && nebWidth[f] < 0)
					bloatedRect.dims[dim].min = rect->dims[dim].min + nebWidth[f];
				else if (!isMin && nebWidth[f] > 0)
					bloatedRect.dims[dim].max = rect->dims[dim].max + nebWidth[f];
			}
			else
			{
				// might be the last iteration, compute min time to cross face

				// clamp derivative if it changed direction
				// this means along the face it's inward, but in the neighborhood it's outward
				if (der < 0 && prevNebWidth > 0)
					der = 0;
				else if (der > 0 && prevNebWidth < 0)
					der = 0;

				if (der != 0)
				{
					double crossTime = prevNebWidth / der;

					if (crossTime < minNebCrossTime)
						minNebCrossTime = crossTime;
				}

				ders[f] = der;
			}
		}
	}

	if (minNebCrossTime * 2 < stepSize)
	{
		//printf(": minNebCrossTime = %f, stepSize = %f\n", minNebCrossTime, stepSize);
		//printf(": debugNumCalls = %i\n", debugNumCalls);
#ifdef DEBUG_FARDIN
		printf("minNebCrossTime is less than half of step size.");
#endif
        return -10000;
	}

	//printf("\n");

	////////////////////////////////////////
	// lift each face by the minimum time //

	double timeToElapse = minNebCrossTime;

	// subtract a tiny amount time due to multiplication / division rounding
	timeToElapse = timeToElapse * 99999 / 100000;

	if (timeRemaining < timeToElapse)
		timeToElapse = timeRemaining;

	// do the lifting
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		rect->dims[d].min += ders[2*d] * timeToElapse;
		rect->dims[d].max += ders[2*d+1] * timeToElapse;
	}

	if (!hyperrectangle_contains(&bloatedRect, rect, true))
	{
		//printf("error occurred when debugNumCalls = %d\n", debugNumCalls);
		//printf("rect = ");
		//println(&debug_initialRect);
#ifdef DEBUG_FARDIN
		printf("lifted rect is outside of bloated rect");
#endif
		return -10000;
	}

	return timeToElapse;
}

// generate a split rectangle
void generateSplitRectangle(HyperRectangle* rectToSplit, HyperRectangle* out,
		int iteratorVal, int splitDimensions[NUM_DIMS])
{
	for (int dimIndex = 0; dimIndex < NUM_DIMS; ++dimIndex)
	{
		int mask = splitDimensions[dimIndex];
//		int splitNum = iteratorVal % mask;
		int splitNum = iteratorVal - (int) ((double) iteratorVal/ (double) mask) * mask;
		iteratorVal = (int) (iteratorVal / (double) mask);

		double width = interval_width(&rectToSplit->dims[dimIndex]) / splitDimensions[dimIndex];

		// assign the current dimension
		out->dims[dimIndex].min = rectToSplit->dims[dimIndex].min + splitNum * width;
		out->dims[dimIndex].max = out->dims[dimIndex].min + width;
	}
}

///////////////// below is from header file ///////////////

bool face_lifting_iterative_improvement(LiftingSettings* settings) {

	double stepSize = settings->initialStepSizeCC;

	bool safe = true; // until proven otherwise

	double timeRemaining = settings->reachTimeCC;
	HyperRectangle trackedRect = settings->init;
	HyperRectangle hull;


	// compute reachability up to split time
	while (safe && timeRemaining > 0)
	{
		if (settings->reachedAtIntermediateTime)
			hull = trackedRect;

		// debug changed so error tracker is always passed in (see note)
		double timeElapsed = lift_single_rect(&trackedRect, stepSize, timeRemaining, COMPLEX_CONTROLLER);

		// if we're not even close to the desired step size
		if (hyperrectange_max_width(&trackedRect) > settings->maxRectWidthBeforeError)
		{
#ifdef DEBUG_FARDIN
			printf("maxRectWidthBeforeError exceeded at time %f, rect = ",
				   settings->reachTimeCC - timeRemaining);
#endif

			// step size is too large, make it smaller and recompute
			safe = false;
		}
		else if (settings->reachedAtIntermediateTime)
		{
			hyperrectangle_grow_to_convex_hull(&hull, &trackedRect);

			safe = safe && settings->reachedAtIntermediateTime(&hull);
		}

		timeRemaining -= timeElapsed;
#ifdef DEBUG_FARDIN
		printf("time timeRemaining %f\n", timeRemaining);
		printf("CC: %f \t %f \t %f \t %f \t %f \t %f Safe: %s \n", trackedRect.dims[0].min, trackedRect.dims[1].min, trackedRect.dims[2].min, trackedRect.dims[3].min,trackedRect.dims[4].min, trackedRect.dims[5].min, safe?"true":"false");
		printf("  : %f \t %f \t %f \t %f \t %f \t %f Safe: %s \n", trackedRect.dims[0].max, trackedRect.dims[1].max, trackedRect.dims[2].max, trackedRect.dims[3].max,trackedRect.dims[4].max, trackedRect.dims[5].max, safe?"true":"false");
#endif

	}
	if (safe){
		settings->reachedAtFinalTime(&trackedRect);
		return settings->checkStabilizabilityAfterCCperiod(&trackedRect, settings->reachTimeSC);
	}
	else
		return false;

}
