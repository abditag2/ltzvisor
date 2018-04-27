#include "simulate.h"




void simualteCC(double startPoint[NUM_DIMS], double stepSize, double length, double * finalPoints) {

	double point[NUM_DIMS];

	for (int d = 0; d < NUM_DIMS; ++d)
		point[d] = startPoint[d];

	HyperRectangle rect;
	double time = 0;

	while (time < length)
	{

		for (int d = 0; d < NUM_DIMS; ++d)
			rect.dims[d].min = rect.dims[d].max = point[d];

		// euler's method

		for (int d = 0; d < NUM_DIMS; ++d)
		{

			double der = get_derivative_bounds(&rect, 2 * d, COMPLEX_CONTROLLER);

			point[d] += stepSize * der;
		}
		time += stepSize;
	}

	for(int j = 0; j < NUM_DIMS; j++)
		finalPoints[j] = point[j];
	return ;
}

void simulateSC(double startPoint[NUM_DIMS],
				double stepSize,
				bool (*shouldStop)(double state[NUM_DIMS], double simTime, void *p),
				void *param)
{
	double point[NUM_DIMS];

	for (int d = 0; d < NUM_DIMS; ++d)
		point[d] = startPoint[d];

	HyperRectangle rect;
	double time = 0;

	while (true)
	{
		if (shouldStop(point, time, param))
			break;

		for (int d = 0; d < NUM_DIMS; ++d)
			rect.dims[d].min = rect.dims[d].max = point[d];

		// euler's method

		for (int d = 0; d < NUM_DIMS; ++d)
		{

			double der = get_derivative_bounds(&rect, 2 * d, SIMPLE_CONTROLLER);

			point[d] += stepSize * der;
		}
		time += stepSize;
	}

}
