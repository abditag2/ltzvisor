//
// Created by fardin on 5/3/18.
//

#include "inc/util.h"
#define PERIOD 0.02

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

//
//
//void matrix_mult(double A[HX_SIZE][6], double B[6][1], int m, int n, int k) {
//
//    int r = 0;
//    int c = 0;
//    int kk = 0;
//    for (r = 0; r < m; r++) {
//        for (c = 0; c < n; c++) {
//            C[r][c] = 0;
//            for (kk = 0; kk < k; kk++) {
//                C[r][c] += A[r][kk] * B[kk][c];
//            }
//        }
//    }
//
//}
//
//
//double voltage_max_min(double voltage) {
//
//    if (voltage > MAX_VOLTAGE)
//        voltage = MAX_VOLTAGE;
//        //else if (voltage < 0)
//        //	voltage = 0;
//    else if (voltage < -MAX_VOLTAGE)
//        voltage = -MAX_VOLTAGE;
//    return voltage;
//}
//
//
void read_from_serial(int *sensor_readings) {

    char rxChar1[24];
    char rxChar = 0xAA;


    uart_putc(1, rxChar);
    wait();

    for (int i = 0; i < 24; i++) {
        rxChar1[i] = uart_getc(1);
        wait();
    }


    sensor_readings[0] = *(unsigned int *) &rxChar1[0];
    sensor_readings[1] = *(unsigned int *) &rxChar1[4];
    sensor_readings[2] = *(unsigned int *) &rxChar1[8];
    sensor_readings[3] = *(unsigned int *) &rxChar1[12];
    sensor_readings[4] = *(unsigned int *) &rxChar1[16];
    sensor_readings[5] = *(unsigned int *) &rxChar1[20];

//    printk("sensor1: %d\n", sensor_readings[0]);
//    printk("sensor2: %d\n", sensor_readings[1]);
//    printk("sensor3: %d\n", sensor_readings[2]);

    return;
}


void safe_controller() {

    int sensors[6];
    int float_volts[2];

    read_from_serial(&sensors);

    double d_elevation = sensors[0]/10000.0;
    double d_pitch = sensors[1]/10000.0;
    double d_travel = sensors[2]/10000.0;

    double elevation = sensors[3]/10000.0;
    double pitch = sensors[4]/10000.0;
    double travel = sensors[5]/10000.0;

    double vol_left =
            - 6.5 * elevation
            - 0.801 * pitch
            - 200.0 * PERIOD * d_elevation
            - 20.0 * PERIOD * d_pitch; //-0.0333*cs->int_elevation -0.001*cs->int_pitch;

    double vol_right = -6.5 * elevation +
           0.6701 * pitch -
           200.0 * PERIOD * d_elevation +
           24.0 * PERIOD * d_pitch; //-0.03*cs->int_elevation +0.001*cs->int_pitch;

    float_volts[0] = (int) (vol_left * 10000);
    float_volts[1] = (int) (vol_right * 10000);

    write_to_serial(float_volts);


//    safe_call_count += 1;
//    if (safe_call_count % 50 == 0){
//        printk("\n\t\t\t\tcalled 50 times. %d\n", (int) safe_call_count/50);
//    }
}