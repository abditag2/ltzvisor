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

#include<hw_zynq.h>
#include<printk.h>
#include <zynq_uart.h>

void led_blink(void *pvParameters);

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


int write_to_serial(int double_volts[]) {

    int voltages[2];
    unsigned char Txbuffer[10];
    char i = 0;
    Txbuffer[0] = 0xCC; // End Byte
    Txbuffer[9] = 0xFF; // Start Byte

    voltages[0] = double_volts[0];
    voltages[1] = double_volts[1];

    for (i = 0; i < 2; i++) {

        Txbuffer[4 * i + 1] = (char) (voltages[i] & 0xff); /* first byte */
        Txbuffer[4 * i + 2] = (char) (voltages[i] >> 8 & 0xff); /* second byte */
        Txbuffer[4 * i + 3] = (char) (voltages[i] >> 16 & 0xff); /* third byte */
        Txbuffer[4 * i + 4] = (char) (voltages[i] >> 24 & 0xff); /* fourth byte */

    }
    int k = 0;
    for (i = 0; i < 10; i ++){
//        printk(" c %d is %x ", i, Txbuffer[i]);
        uart_putc(1, Txbuffer[i]);

        k = 0;
        while (k < 10000){
            k = k + 1;
        }
    }

    return k;
}


void read_from_serial(int *sensor_readings) {

    char rxChar1[13];
//    int tmpSend[2];

    while (1) {

        char rxChar;
        rxChar = 0xAA;
        uart_putc(1, rxChar);

        int i = 0;
        for (i = 0; i < 13; i++) {
            rxChar1[i] = uart_getc(1);
        }

        sensor_readings[0] = *(unsigned int *) &rxChar1[0];
        sensor_readings[1] = *(unsigned int *) &rxChar1[4];
        sensor_readings[2] = *(unsigned int *) &rxChar1[8];
//        tmpSend[0] = (double) sensor_readings[0];
//        tmpSend[1] = (double) sensor_readings[1];
        //write_to_serial(tmpSend);
        break;

    }

    return;
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

    int double_volts[2];
    int sensors[3];

    double_volts[0] = 1;
    double_volts[1] = 3;

    sensors[0] = 1;
    sensors[1] = 2;
    sensors[2] = 3;

    for (;;) {
        write_to_serial(double_volts);
//        read_from_serial(sensors);

        double_volts[0] += 1;
        double_volts[1] += 1;

        toggle ^= 0xFF;
        *ptr = toggle;
        YIELD()
    }
}
