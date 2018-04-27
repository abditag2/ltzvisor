/*
 * LTZVisor, a Lightweight TrustZone-assisted Hypervisor
 *
 * Copyright (c) TZVisor Project (www.tzvisor.org), 2017-
 *
 * Authors:
 *  Sandro Pinto <sandro@tzvisor.org>
 *
 * This file is part of LTZVisor.
 *
 * LTZVisor is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, with a special   
 * exception described below.
 * 
 * Linking this code statically or dynamically with other modules 
 * is making a combined work based on this code. Thus, the terms 
 * and conditions of the GNU General Public License V2 cover the 
 * whole combination.
 *
 * As a special exception, the copyright holders of LTZVisor give  
 * you permission to link LTZVisor with independent modules to  
 * produce a statically linked executable, regardless of the license 
 * terms of these independent modules, and to copy and distribute  
 * the resulting executable under terms of your choice, provided that 
 * you also meet, for each linked independent module, the terms and 
 * conditions of the license of that module. An independent module  
 * is a module which is not derived from or based on LTZVisor.
 *
 * LTZVisor is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 *
 * [blink.c]
 *
 * This file contains a bare-metal Blink application.
 * 
 * (#) $id: blink.c 27-09-2017 s_pinto$
*/

#include <hw_zynq.h>
#include <printk.h>
#include <zynq_uart.h>


#include "blink.h"

void led_blink(void *pvParameters);


float vol_right = 0;
float vol_left = 0;


float add_left = 0;
float add_right = 0;

float C[HX_SIZE][1];
struct state sps;


// Loop <delay> times in a way that the compiler won't optimize away
static inline void wait() {
    int32_t count = 1000;
    asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
    : "=r"(count): [count]"0"(count) : "cc");
}

void write_to_serial(int float_volts[]) {

    int voltages[2];
    unsigned char Txbuffer[10];
    char i = 0;
    Txbuffer[0] = 0xCC; // End Byte
    Txbuffer[9] = 0xFF; // Start Byte

    voltages[0] = float_volts[0];
    voltages[1] = float_volts[1];

    for (i = 0; i < 2; i++) {

        Txbuffer[4 * i + 1] = (char) (voltages[i] & 0xff); /* first byte */
        Txbuffer[4 * i + 2] = (char) (voltages[i] >> 8 & 0xff); /* second byte */
        Txbuffer[4 * i + 3] = (char) (voltages[i] >> 16 & 0xff); /* third byte */
        Txbuffer[4 * i + 4] = (char) (voltages[i] >> 24 & 0xff); /* fourth byte */

    }

    for (i = 0; i < 10; i++) {
//        printk(" c %d is %x ", i, Txbuffer[i]);
        uart_putc(1, Txbuffer[i]);
        wait();
    }

    return;
}


void matrix_mult(double A[HX_SIZE][6], double B[6][1], int m, int n, int k) {

    int r = 0;
    int c = 0;
    int kk = 0;
    for (r = 0; r < m; r++) {
        for (c = 0; c < n; c++) {
            C[r][c] = 0;
            for (kk = 0; kk < k; kk++) {
                C[r][c] += A[r][kk] * B[kk][c];
            }
        }
    }

}


double voltage_max_min(double voltage) {

    if (voltage > MAX_VOLTAGE)
        voltage = MAX_VOLTAGE;
        //else if (voltage < 0)
        //	voltage = 0;
    else if (voltage < -MAX_VOLTAGE)
        voltage = -MAX_VOLTAGE;
    return voltage;
}


void read_from_serial(int *sensor_readings) {

    char rxChar1[12];
    char rxChar;

    while (1) {

//        rxChar = 's';
//        uart_putc(1, rxChar);
//        wait();

        while (1) {
            char cc = uart_getc(1);
            printk("cc is %c\n", cc);
            if (cc == 's')
                break;
        }

        int i = 0;
        for (i = 0; i < 12; i++) {
            rxChar1[i] = uart_getc(1);
            printk("c: %c\n", rxChar1[i]);
            wait();
        }

        rxChar = uart_getc(1);
        if (rxChar == 'e') {
            break;
        }

    }

    sensor_readings[0] = *(unsigned int *) &rxChar1[0];
    sensor_readings[1] = *(unsigned int *) &rxChar1[4];
    sensor_readings[2] = *(unsigned int *) &rxChar1[8];
    printk("sensor1: %d\n", sensor_readings[0]);
    printk("sensor2: %d\n", sensor_readings[1]);
    printk("sensor3: %d\n", sensor_readings[2]);


    return;
}

float P[10] = {-.500f, -2.4000f, 0.00f, 0.1200f, 0.1200f, -2.5000f, -0.0200f, 0.200f, 2.1000f, 10.0000f};

float Hx[HX_SIZE][6] = {
        {-0.937749f, -0.347314f, 0, 0.000000f,  0.000000f,  0},
        {1.000000f,  0.000000f,  0, 0.000000f,  0.000000f,  0},
        {0.000000f,  0.000000f,  0, 1.000000f,  0.000000f,  0},
        {0.000000f,  0.000000f,  0, 0.000000f,  1.000000f,  0},
        {-0.937749f, 0.347314f,  0, 0.000000f,  0.000000f,  0},
        {-1.000000f, 0.000000f,  0, 0.000000f,  0.000000f,  0},
        {0.000000f,  0.000000f,  0, -1.000000f, 0.000000f,  0},
        {0.000000f,  0.000000f,  0, 0.000000f,  -1.000000f, 0},
        {-0.920831f, 0.341049f,  0, -0.184166f, 0.042875f,  0},
        {0.980581f,  0.000000f,  0, 0.196116f,  0.000000f,  0},
        {0.976164f,  0.092968f,  0, 0.195233f,  0.018594f,  0},
        {0.976164f,  -0.092968f, 0, 0.195233f,  -0.018594f, 0},
        {0.982846f,  0.026635f,  0, 0.182186f,  0.010654f,  0},
        {0.983313f,  0.033791f,  0, 0.178416f,  0.011006f,  0},
        {-0.980581f, 0.000000f,  0, -0.196116f, 0.000000f,  0},
        {-0.942434f, -0.177818f, 0, -0.280952f, -0.035564f, 0},
        {-0.920831f, -0.341049f, 0, -0.184166f, -0.042875f, 0},
        {-0.939588f, 0.229568f,  0, -0.251069f, 0.037920f,  0},
        {-0.929818f, 0.247341f,  0, -0.269327f, 0.041529f,  0},
        {-0.945071f, 0.207015f,  0, -0.250444f, 0.035553f,  0},
        {0.730320f,  -0.429600f, 0, 0.524112f,  -0.085920f, 0},
        {0.730320f,  0.429600f,  0, 0.524112f,  0.085920f,  0},
        {0.983313f,  -0.033791f, 0, 0.178416f,  -0.011006f, 0},
        {0.982846f,  -0.026635f, 0, 0.182186f,  -0.010654f, 0},
        {-0.919538f, -0.340570f, 0, -0.183908f, -0.068114f, 0},
        {-0.929818f, -0.247341f, 0, -0.269327f, -0.041529f, 0},
        {-0.945071f, -0.207015f, 0, -0.250444f, -0.035553f, 0},
        {-0.924294f, 0.088028f,  0, -0.369718f, 0.035211f,  0},
        {-0.942434f, 0.177818f,  0, -0.280952f, 0.035564f,  0},
        {-0.976164f, 0.092968f,  0, -0.195233f, 0.018594f,  0},
        {-0.942675f, 0.223627f,  0, -0.244964f, 0.036715f,  0},
        {-0.919538f, 0.340570f,  0, -0.183908f, 0.068114f,  0},
        {-0.942675f, -0.223627f, 0, -0.244964f, -0.036715f, 0},
        {-0.939588f, -0.229568f, 0, -0.251069f, -0.037920f, 0},
        {-0.976164f, -0.092968f, 0, -0.195233f, -0.018594f, 0},
        {-0.924294f, -0.088028f, 0, -0.369718f, -0.035211f, 0},
        {0.924294f,  0.088028f,  0, 0.369718f,  0.035211f,  0},
        {0.924294f,  -0.088028f, 0, 0.369718f,  -0.035211f, 0},


};

float hx[HX_SIZE][1] = {
        {0.114896f},
        {0.119671f},
        {0.199735f},
        {1.300000f},
        {0.114896f},
        {0.119671f},
        {0.199735f},
        {1.300000f},
        {0.164176f},
        {0.136959f},
        {0.197136f},
        {0.197136f},
        {0.157410f},
        {0.160876f},
        {0.136959f},
        {0.160740f},
        {0.164176f},
        {0.156719f},
        {0.165424f},
        {0.152473f},
        {0.496346f},
        {0.496346f},
        {0.160876f},
        {0.157410f},
        {0.184185f},
        {0.165424f},
        {0.152473f},
        {0.185233f},
        {0.160740f},
        {0.137058f},
        {0.153904f},
        {0.184185f},
        {0.153904f},
        {0.156719f},
        {0.137058f},
        {0.185233f},
        {0.242119f},
        {0.242119f},
};

struct state eval_state(struct state state_x, struct command U) {

    struct state d_state;

    d_state.elevation = state_x.d_elevation;
    d_state.pitch = state_x.d_pitch;
    d_state.travel = state_x.d_travel;
    d_state.d_elevation = P[0] * cosi(state_x.elevation) + P[1] * sini(state_x.elevation) + P[2] * state_x.d_travel +
                          P[7] * cosi(state_x.pitch) * (U.u1 + U.u2);

    d_state.d_pitch =
            P[4] * sini(state_x.pitch) + P[3] * cosi(state_x.pitch) + P[5] * state_x.d_pitch + P[8] * (U.u1 - U.u2);

    d_state.d_travel = P[6] * state_x.d_travel + P[9] * sini(state_x.pitch) * (U.u1 + U.u2);

    state_x.elevation += SIM_STEP * d_state.elevation;
    state_x.pitch += SIM_STEP * d_state.pitch;
    state_x.travel += SIM_STEP * d_state.travel;
    state_x.d_elevation += SIM_STEP * d_state.d_elevation;
    state_x.d_pitch += SIM_STEP * d_state.d_pitch;
    state_x.d_travel += SIM_STEP * d_state.d_travel;

    return state_x;
}


int check_safety(struct state x) {

    float X[6][1] = {{x.elevation},
                      {x.pitch},
                      {x.travel},
                      {x.d_elevation},
                      {x.d_pitch},
                      {x.d_travel}};

    matrix_mult(Hx, X, HX_SIZE, 1, 6);

    int all_small = 1;
    int k = 0;
    for (k = 0; k < HX_SIZE; k++) {
        if (C[k][0] > hx[k][0]) {

            all_small = 0;

            break;
        }

    }

    //  if (x.elevation+0.333*x.pitch > -0.3 && x.elevation-0.333*x.pitch>-0.3  && x.elevation < 0.35 && x.d_elevation > -0.3 && x.d_elevation < 0.4  && x.d_pitch > -1.3 && x.d_pitch < 1.3)
    if (all_small == 1)
        return 1;
    else
        return 0;
}


struct command controller_safety(struct state sp, struct state x, struct controller_storage* cs){

    struct command U;

    cs->int_travel +=  x.travel;
    cs->int_pitch +=  x.pitch;
    cs->int_elevation +=  x.elevation;

//    U.u1 = -6.5 * (x.elevation-sp.elevation) - .701 * x.pitch  - 45.7161 * PERIOD * x.d_elevation -3.051 * PERIOD * x.d_pitch ; //-0.0333*cs->int_elevation -0.001*cs->int_pitch;
//    U.u2 = -6.5 * (x.elevation-sp.elevation) + .5701 * x.pitch - 45.7529 * PERIOD * x.d_elevation +5.970 * PERIOD*  x.d_pitch; //-0.03*cs->int_elevation +0.001*cs->int_pitch;

//    U.u1 =    -6.5 * (x.elevation - sp.elevation)  - .9701 * x.pitch - 55.7161 * PERIOD * x.d_elevation -7.051 * PERIOD * x.d_pitch; //-0.0333*cs->int_elevation -0.001*cs->int_pitch;
//        //left voltage
//    U.u2  =   -6.5 * (x.elevation - sp.elevation)  + .97701 * x.pitch - 55.7529 * PERIOD * x.d_elevation +10.970 * PERIOD*  x.d_pitch; //-0.03*cs->int_elevation +0.001*cs->int_pitch;


    U.u1 =    -6.5f * (x.elevation - sp.elevation)  - .801f * x.pitch - 200.0f * PERIOD * x.d_elevation -20.0f * PERIOD * x.d_pitch; //-0.0333*cs->int_elevation -0.001*cs->int_pitch;
    //left voltage
    U.u2  =   -6.5f * (x.elevation - sp.elevation)  + .6701f * x.pitch - 200.0f * PERIOD * x.d_elevation +24.0f * PERIOD*  x.d_pitch; //-0.03*cs->int_elevation +0.001*cs->int_pitch;


    U.u1 += 1.7f;
    U.u2 += 1.8f;

    cs->elevation2 = cs->elevation1;
    cs->elevation1 = x.elevation;

    cs->pitch2 = cs->pitch1;
    cs->pitch1 = x.pitch;

    cs->travel2 = cs->travel1;
    cs->travel1 = x.travel;


    U.u1 = voltage_max_min(U.u1);
    U.u2 = voltage_max_min(U.u2);
    return U;
}


struct state simulate_fixed_control(struct state init_state, struct command U, float time) {

    struct state state_x;

    state_x = init_state;

    int steps = time / SIM_STEP;
    int k = 0;
    for (k = 0; k < steps; k++) {

        state_x = eval_state(state_x, U);

        if (check_safety(state_x) == 0) {
            state_x.safe = 0;

            return state_x;
        }
    }
    state_x.safe = 1;


    return state_x;
}

struct state simulate_with_controller(struct state init_state, float time) {

    struct state state_x;

    state_x = init_state;

    int steps = time / SIM_STEP;

    struct controller_storage cs;
    cs.int_elevation = 0;
    cs.int_pitch = 0;
    cs.int_travel = 0;

    int k = 0;

    for (k = 0; k < steps; k++) {
        struct command U = controller_safety(sps, state_x, &cs);
        state_x = eval_state(state_x, U);
        if (check_safety(state_x) == 0) {

            state_x.safe = 0;

            return state_x;

        }

        state_x.safe = 1;
    }

    return state_x;

}


//The outcome determines weather the safety controller should be used or not
int decide(struct state current_state, struct command U, float time) {
    //struct state x2 = simulate_fixed_control(current_state, U, time);
    struct state x2 = simulate_fixed_control(current_state, U, RESTART_TIME);
    // struct state x10;
    if (x2.safe == 0)
        return 0;

    struct state x10 = simulate_with_controller(x2, 1);

    if (x2.safe == 1 && x10.safe == 1)
        return 1;
    else
        return 0;

}


/**
 * Blink LED "Task"
 *
 * @param  	
 *
 * @retval 	
 */
void led_blink(void *parameters) {

    static uint32_t toggle;
    /** 4GPIO (LED) in FPGA fabric */
    static uint32_t *ptr = (uint32_t *) 0x41200000;

    int float_volts[2];
    int sensors[3];

    float_volts[0] = 1;
    float_volts[1] = 3;

    sensors[0] = 0;
    sensors[1] = 0;
    sensors[2] = 0;

    for (;;) {

//        read_from_serial(sensors);
//        write_to_serial(float_volts);

        char cc = uart_getc(1);
        printk("cc is %c\n", cc);


        float_volts[0] = sensors[0];
        float_volts[1] += 1;

//        printk('sensor reading is %d', sensors[0]);

        toggle ^= 0xFF;
        *ptr = toggle;
        YIELD()
    }
}


int main() {

    /** Initialize hardware */
    hw_init();

//    printk(" * Secure bare metal VM: running ... \n\t");

    /** Generate tick every 1s */
    tick_set(200000);

    /* Calling Blinking Task (LED blink at 1s) */
    led_blink((void *) 0);

    /* This point will never be reached */
    for (;;);

}
