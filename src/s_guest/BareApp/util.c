//
// Created by fardin on 5/3/18.
//

#include "inc/util.h"

extern int safe_call_count;

void double2string(double num) {

    if (num < 0) {
        printk("-");
        num = -num;
    }

    unsigned int digit = 0;

    digit = (unsigned int) num / 10000.0;
    printk("%d", digit);
    num = num - digit * 10000.0;

    digit = (unsigned int) num / 1000.0;
    printk("%d", digit);
    num = num - digit * 1000.0;

    digit = (unsigned int) num / 100.0;
    printk("%d", digit);
    num = num - digit * 100.0;

    digit = (unsigned int) num / 10.0;
    printk("%d", digit);
    num = num - digit * 10.0;

    digit = (unsigned int) num;
    printk("%d", digit);
    num = num - digit;

    printk(".");

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d", digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d", digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d", digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d", digit);
    num = num - digit;

}

// Loop <delay> times in a way that the compiler won't optimize away
static inline void wait() {
    int count = 1000;
    asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
    : "=r"(count): [count]"0"(count) : "cc");
}

void write_to_serial(int float_volts[]) {

    int voltages[2];
    unsigned char Txbuffer[9];
    char i = 0;

    Txbuffer[0] = 0xCC; // End Byte

    voltages[0] = float_volts[0];
    voltages[1] = float_volts[1];

    for (i = 0; i < 2; i++) {

        Txbuffer[4 * i + 1] = (char) (voltages[i] & 0xff); /* first byte */
        Txbuffer[4 * i + 2] = (char) (voltages[i] >> 8 &
                                      0xff); /* second byte */
        Txbuffer[4 * i + 3] = (char) (voltages[i] >> 16 &
                                      0xff); /* third byte */
        Txbuffer[4 * i + 4] = (char) (voltages[i] >> 24 &
                                      0xff); /* fourth byte */

    }

    for (i = 0; i < 9; i++) {
        uart_putc(1, Txbuffer[i]);
        wait();
    }
}

void read_from_serial(int *sensor_readings) {

    unsigned char rxChar1[24];
    char rxChar = 0xAA;

    uart_putc(1, rxChar);
    wait();

    for (int i = 0; i < 24; i++) {
        rxChar1[i] = uart_getc(1);
//        wait();
    }

    sensor_readings[0] = *(int *) &rxChar1[0];
    sensor_readings[1] = *(int *) &rxChar1[4];
    sensor_readings[2] = *(int *) &rxChar1[8];
    sensor_readings[3] = *(int *) &rxChar1[12];
    sensor_readings[4] = *(int *) &rxChar1[16];
    sensor_readings[5] = *(int *) &rxChar1[20];

    return;
}


void safe_controller() {

    int sensors[6];
    int float_volts[2];

    read_from_serial(&sensors);

    double elevation = ((double) sensors[0]) / 10000.0;
    double pitch = ((double) sensors[1]) / 10000.0;
    double travel = ((double) sensors[2]) / 10000.0;

    double d_elevation = ((double) sensors[3]) / 10000.0;
    double d_pitch = ((double) sensors[4]) / 10000.0;
    double d_travel = ((double) sensors[5]) / 10000.0;


    double vol_left =
            -5.4617 * elevation
            - 1.7907 * pitch
            - 2.6956 * d_elevation
            - 0.4738 * d_pitch; //-0.0333*cs->int_elevation -0.001*cs->int_pitch;

    double vol_right =
            -5.5698 * elevation
            + 1.9211 * pitch
            - 2.7146 * d_elevation
            + 0.4659 * d_pitch; //-0.03*cs->int_elevation +0.001*cs->int_pitch;

//    To compensate for gravity effect
    vol_left += 0.325;
    vol_right += 0.325;

    printk("el %d\n", (int) (elevation * 10000.0));
    printk("pi %d\n", (int) (pitch * 10000.0));
    printk("de %d \n", (int) (d_elevation * 10000.0));
    printk("dp %d \n", (int) (d_pitch * 10000.0));

    float_volts[0] = (int) (vol_left * 10000.0);
    float_volts[1] = (int) (vol_right * 10000.0);

    printk("vl %d \n", float_volts[0]);
    printk("vr %d \n", float_volts[1]);

    printk("trusted\n");
    write_to_serial(float_volts);


//    safe_call_count += 1;
//    if (safe_call_count % 50 == 0){
//        printk("\n\t\t\t\tcalled 50 times. %d\n", (int) safe_call_count/50);
//    }
}