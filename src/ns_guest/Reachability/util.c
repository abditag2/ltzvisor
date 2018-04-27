#include "util.h"
#include "face_lift.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef WIN32
#include "windows.h" // for GetTickCount
#else
	#ifdef ZEDBOARD

	#else
		#include <sys/time.h> // for gettimeofday
	#endif
#endif

// just for debug
static LiftingSettings errorPrintParams;
static bool errorParamsAssigned = false;

void set_error_print_params(LiftingSettings* set)
{
	errorParamsAssigned = true;
	errorPrintParams = *set;
}

void error_exit(const char* str)
{
	printf("Error: %s\n", str);

	// print the params that caused the error
	if (errorParamsAssigned)
	{
		printf("\nSettings:\n");
		printf("Reach TimeCC = %f\n", errorPrintParams.reachTimeCC);
		printf("Runtime = %i ms\n", errorPrintParams.maxRuntimeMilliseconds);
		printf("Init = ");
		println(&errorPrintParams.init);
	}
	else
		printf("Error print params were not assigned.\n");

	fflush(stdout);

	exit(1);
}


int milliseconds()
{
#ifdef WIN32
	return GetTickCount();
#else
	#ifdef ZEDBOARD
		return 0;
	#else
		static bool initialized = false;
		static time_t startSec = 0;

		struct timeval now;
		gettimeofday(&now, NULL);

		if (!initialized)
		{
			initialized = true;
			startSec = now.tv_sec;
		}

		int difSec = now.tv_sec - startSec;
		int ms = now.tv_usec / 1000;

		return difSec * 1000 + ms;
	#endif
#endif
}


