#include "inc/util.h"

#ifdef WIN32
#include "windows.h" // for GetTickCount
#else
	#ifdef ZEDBOARD

	#else
		#include <sys/time.h> // for gettimeofday
	#endif
#endif

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


