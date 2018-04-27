// Stanley Bak
// 4-2014
// Real-time Reachability utilities

#ifndef UTIL_H_
#define UTIL_H_

#include "face_lift.h"

#define ZEDBOARD

// print a string and exit
void error_exit(const char* str);

// sets the lifting params to print in case error_exit is called
void set_error_print_params(LiftingSettings* set);

// milliseconds timer
int milliseconds();

//fills in the critical points in the points array


#endif
