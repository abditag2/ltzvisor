//
// Created by fardin on 4/26/18.
//

#ifndef LTZVISOR_BLINK_H
#define LTZVISOR_BLINK_H

#define SIM_STEP        0.02
#define PERIOD          0.02
#define RESTART_TIME    0.

#define MAX_VOLTAGE     4

#define HX_SIZE         52




#define CMD_READ_SENSORS 0
#define CMD_WRITE_VOLTAGES 1

struct controller_storage {
    float int_elevation;
    float int_pitch;
    float int_travel;

    float elevation1;
    float pitch1;
    float travel1;

    float elevation2;
    float pitch2;
    float travel2;
} controller_storage;

struct state {
    float elevation;
    float pitch;
    float travel;
    float d_elevation;
    float d_pitch;
    float d_travel;
    int safe;
} state;

struct command {
    float u1;
    float u2;
} command;

#endif //LTZVISOR_BLINK_H
