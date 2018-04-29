#define DEBUG_FARDIN
#include "inc/3dof.h"
#include <stdio.h>

int main(int argc, char** argv)
{
	int rv = 0;
    double reachTimeSC = 20;

	double startState[6] = {-0.15, -0.3, 0, 0, 0, 0};
//	double startState[6] = {-0.15, 0.3, 0, 0, 0, 0};


#ifdef DEBUG_FARDIN
	printf("from state = [%f, %f, %f, %f]\n", startState[0], startState[1], startState[2], startState[3]);
#endif
    double res = findMaxRestartTime(startState, reachTimeSC);
#ifdef DEBUG_FARDIN
	printf("maxRestartTime: %f\n", res);
#endif

	return rv;
}