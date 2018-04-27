/*
 * FlightControl.h
 *
 *  Created on: Mar 15, 2017
 *      Author: cy
 */

#ifndef SRC_FLIGHTCONTROL_FLIGHT_CONTROL_H_
#define SRC_FLIGHTCONTROL_FLIGHT_CONTROL_H_

void vFlightControlInit( void );
void prvFlightControlTask(void *pvParameters);
void prvSeiFlightControlTask(void *pvParameters);
void prvMaliciousFlightControlTask(void *pvParameters);

#define CMD_READ_SENSORS 0
#define CMD_WRITE_VOLTAGES 1

struct controller_storage {
    double int_elevation;
    double int_pitch;
    double int_travel;

    double elevation1;
    double pitch1;
    double travel1;

    double elevation2;
    double pitch2;
    double travel2;
} controller_storage;

struct state {
    double elevation;
    double pitch;
    double travel;
    double d_elevation;
    double d_pitch;
    double d_travel;
    int safe;
} state;

struct command {
    double u1;
    double u2;
} command;

#endif /* SRC_FLIGHTCONTROL_FLIGHT_CONTROL_H_ */
