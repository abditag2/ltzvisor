

void init_platform();


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

double new_cc_set_point_double = 0.0;

void read_from_serial(int *sensor_readings) {

    unsigned char rxChar1[29];
    char rxChar = 0xBB;
    char recieved;
    unsigned char crc = 0;
    unsigned char recievd_crc = 0;
    int new_cc_set_point_int = 0;


    int i;
    while (1) {

        uart_putc(1, rxChar);
        wait();
        printk("waiting for bb: ");
        while (1) {
            recieved = uart_getc(1);
            printk("%d ", recieved);
            if (recieved == 0xFB) {
                break;
            }
        }
        printk("done!\n");

        printk("BB sent and reading : ");
        crc = 0;
        for (i = 0; i < 29; i++) {
            rxChar1[i] = uart_getc(1);
            printk("%d ", rxChar1[i]);
            crc = crc + rxChar1[i];
        }
        printk("!\n");
        recievd_crc = uart_getc(1);
        printk("crc is %d calc crc: %d\n", recievd_crc, crc);
        if (recievd_crc == crc) {
            printk("Crc match\n");
            break;
        }
        printk("Crc didn't match\n");
    }

    printk("got insec: ");
    for (i = 0; i < 29; i++) {
        printk("%d ", rxChar1[i]);
    }
    printk("\n");


    sensor_readings[0] = *(int *) &rxChar1[0];
    sensor_readings[1] = *(int *) &rxChar1[4];
    sensor_readings[2] = *(int *) &rxChar1[8];
    sensor_readings[3] = *(int *) &rxChar1[12];
    sensor_readings[4] = *(int *) &rxChar1[16];
    sensor_readings[5] = *(int *) &rxChar1[20];

    printk(" before ");
    if (rxChar1[24] == 0xF9) {
        printk("here ");

        new_cc_set_point_int = (int) ((unsigned char) (rxChar1[25]) |
                                      (unsigned char) (rxChar1[26]) << 8 |
                                      (unsigned char) (rxChar1[27]) << 16 |
                                      (unsigned char) (rxChar1[28]) << 24);

        printk("new cc 2 is %d\n", new_cc_set_point_int);
        new_cc_set_point_double = (double) (new_cc_set_point_int / 10000.0);
    }

    printk(" cc_set is :");
    double2string(new_cc_set_point_double);
    printk("\n");

    return;
}

void untrusted_controller() {

    int sensors[6];
    int float_volts[2];

    read_from_serial(&sensors);

    double elevation = sensors[0] / 10000.0;
    double pitch = sensors[1] / 10000.0;
    double travel = sensors[2] / 10000.0;

    double d_elevation = sensors[3] / 10000.0;
    double d_pitch = sensors[4] / 10000.0;
    double d_travel = sensors[5] / 10000.0;

//    Right is positive for travel
    double vol_left =
            -10.4617 * (elevation)
            - 1.7907 * pitch
            - 5 * (travel - new_cc_set_point_double)
            - 2.6956 * d_elevation
            - 0.4738 * d_pitch
            - 12.6 * d_travel;

    double vol_right =
            -10.5698 * (elevation)
            + 1.9211 * pitch
            + 5 * (travel - new_cc_set_point_double)
            - 2.7146 * d_elevation
            + 0.4659 * d_pitch
            + 12.6 * d_travel;

//    To compensate for gravity effect
    vol_left += 0.325;
    vol_right += 0.325;

//    vol_left = -2;
//    vol_right = 2;

    printk("el %d\n", (int) (elevation * 10000.0));
    printk("pi %d\n", (int) (pitch * 10000.0));
    printk("de %d \n", (int) (d_elevation * 10000.0));
    printk("dp %d \n", (int) (d_pitch * 10000.0));



//    vol_left = +2 ;
//    vol_right = -2;


    float_volts[0] = (int) (vol_left * 10000);
    float_volts[1] = (int) (vol_right * 10000);

    printk("vl %d \n", float_volts[0]);
    printk("vr %d \n", float_volts[1]);


    write_to_serial(float_volts);


//    safe_call_count += 1;
//    if (safe_call_count % 50 == 0){
//        printk("\n\t\t\t\tcalled 50 times. %d\n", (int) safe_call_count/50);
//    }
}


void main() {
    static int counter = 0;
    int i = 0;
    int j = 0;
    init_platform();
    printk("Non-Secure bare metal VM: running ... \n\r");

    for (i = 0; i < 12000; i++) {
        for (j = 0; j < 2000; j++) {
            /* Do nothing */
        }
    }

    while (1) {

        printk("untrusted\n", counter++);
        untrusted_controller();

        for (i = 0; i < 50; i++) {
            for (j = 0; j < 1000; j++) {
                /* Do nothing */
            }
        }
    }

    // END!!!
    while (1);
}


void init_platform() {

}
