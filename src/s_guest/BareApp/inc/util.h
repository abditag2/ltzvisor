//
// Created by fardin on 5/3/18.
//

#ifndef LTZVISOR_UTIL_H
#define LTZVISOR_UTIL_H

//extern int safe_call_count = 0;

void double2string(double num);
void safe_controller();
void write_to_serial(int float_volts[]);
void read_from_serial(int *sensor_readings);

#endif //LTZVISOR_UTIL_H
