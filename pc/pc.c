#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <linux/ioctl.h>
#include <pthread.h>

#define TO_UNSAFE_CONTROLLER 0
#define TO_SAFETY_CONTROLLER 1
#define TO_COMPLETE_UNSAFE_CONTROLLER 2

pthread_mutex_t start_simulation;
pthread_mutex_t lock_actuator_update;
pthread_mutex_t lock_sensor;

double eval_dim_with_controller_with_commad(int dim, double state[],
                                            double command[]) {

//    double p1 = -1.0000;
//    double p2 = -2.4000;
//    double p3 = -0.0943;
//    double p4 = 0.1200;
//    double p5 = 0.1200;
//    double p6 = -2.5000;
//    double p7 = -0.0200;
    double p8 = 0.200;
    double p9 = 2.1000;
    double p10 = 10.0000;

//    double e = state[0];
//    double p = state[1];
//    double y = state[2];
    double de = state[3];
    double dp = state[4];
    double dy = state[5];

    double rv = 0;

    if (dim == 0) {
        rv = de;
    } else if (dim == 1) {
        rv = dp;
    } else if (dim == 2) {
        rv = dy;
    } else if (dim == 3) {
        rv = -0.15 + p8 * command[0] + p8 * command[1];
    } else if (dim == 4) {
        rv = p9 * command[0] - p9 * command[1];
    } else if (dim == 5) {
        rv = (command[0] + command[1]) * state[1] * 0.2;
//        rv = 0;
    }

    return rv;
}


void simulate(double state[], double command[], double time) {
    double d_state[6];
    for (int i = 0; i < 6; i++) {
        d_state[i] = eval_dim_with_controller_with_commad(i, state, command);
    }

    for (int i = 0; i < 6; i++) {
        state[i] = state[i] + d_state[i] * time;
    }
}

double actuator_volts[2] = {0, 0};
int sensor_reading[6];


void write_to_file(double system_state[]) {

    FILE *f = fopen("state.csv", "a");
    if (f == NULL) {
        printf("error printing file\n");
        return;
    }


    /* print some text */
    fprintf(f, "%f,%f,%f,%f,%f,%f\n", system_state[0], system_state[1],
            system_state[2], system_state[3], system_state[4],
            system_state[5]);

    fflush(f);
    fclose(f);
}

void simulator() {
    double system_state[6] = {-0.3, 0, 0, 0, 0, 0};

    pthread_mutex_lock(&lock_sensor);
    for (int i = 0; i < 6; i++) {
        sensor_reading[i] = (int) (system_state[i] * 10000.0);
//            printf("%d,%d,%d,%d,%d,%d\n", sensor_reading[0],sensor_reading[1],sensor_reading[2],sensor_reading[3],sensor_reading[4],sensor_reading[5]);
    }
    pthread_mutex_unlock(&lock_sensor);

    double actuator_volts_local[2];

    int time = 0;

    pthread_mutex_lock(&start_simulation);
    pthread_mutex_unlock(&start_simulation);

    printf("\n\nstarting to get commands\n\n");

    printf("%f,%f,%f,%f,%f,%f\n", system_state[0], system_state[1],
           system_state[2], system_state[3], system_state[4], system_state[5]);

    while (1) {

        time++;
        if (time % 500 == 0) {
            printf("\nstate is: %f,%f,%f,%f,%f,%f\n", system_state[0],
                   system_state[1],
                   system_state[2], system_state[3], system_state[4],
                   system_state[5]);
            time = 0;
            write_to_file(system_state);

        }
        pthread_mutex_lock(&lock_actuator_update);
        actuator_volts_local[0] = actuator_volts[0];
        actuator_volts_local[1] = actuator_volts[1];
        pthread_mutex_unlock(&lock_actuator_update);

        simulate(system_state, actuator_volts, 0.001);

        pthread_mutex_lock(&lock_sensor);
        for (int i = 0; i < 6; i++) {
            sensor_reading[i] = (int) (system_state[i] * 10000.0);
//            printf("%d,%d,%d,%d,%d,%d\n", sensor_reading[0],sensor_reading[1],sensor_reading[2],sensor_reading[3],sensor_reading[4],sensor_reading[5]);
        }
        pthread_mutex_unlock(&lock_sensor);
        fflush(stdout);
        usleep(1000);
    }

}


int set_interface_attribs(int fd, int speed, int parity) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
//        printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN] = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}


void set_blocking(int fd, int should_block, int number_of_char) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? number_of_char : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("error %d setting term attributes", errno);
    }
}

#define SERIAL_DATA_SIZE 8

unsigned char receive_bytes(int fd, int *commands) {

    unsigned char buf8[SERIAL_DATA_SIZE];
    unsigned char buf;

    set_blocking(fd, 1, SERIAL_DATA_SIZE);
    int n = read(fd, &buf8, SERIAL_DATA_SIZE);

//    for (int i = 0; i < SERIAL_DATA_SIZE; i++) {
//        printf("READ: %x\n", buf8[i]);
//    }

    int vol_left = *(int *) buf8;
    int vol_right = *(int *) &buf8[4];

//    printf("vol left  : %d \n", vol_left);
//    printf("vol right : %d \n", vol_right);

    commands[0] = vol_left;
    commands[1] = vol_right;

    return buf8[8];
}


void send_serial(int fd, int values[], int target) {
    unsigned char Txbuffer[24];
    unsigned char safety[1];
    unsigned char crc[1] = {0};

    for (int i = 0; i < 6; i++) {
        Txbuffer[4 * i] = (unsigned char) (values[i] &
                                           0xff); /* first byte */
        Txbuffer[4 * i + 1] = (unsigned char) (values[i] >> 8 &
                                               0xff); /* second byte */
        Txbuffer[4 * i + 2] = (unsigned char) (values[i] >> 16 &
                                               0xff); /* third byte */
        Txbuffer[4 * i + 3] = (unsigned char) (values[i] >> 24 &
                                               0xff); /* fourth byte */
    }
    printf("sending chars: ");
    for (int i = 0; i < 24; i++) {
        printf("%d ", Txbuffer[i]);
        crc[0] = crc[0] + Txbuffer[i];
    }
    printf("\n");

    usleep(500);             // sleep enough to transmit the 7 plus
    if (target == TO_SAFETY_CONTROLLER) {
        safety[0] = 0xFD;
        write(fd, safety, 1);
    } else if (target == TO_UNSAFE_CONTROLLER || target == TO_COMPLETE_UNSAFE_CONTROLLER) {
        safety[0] = 0xFB;
        write(fd, safety, 1);
    }

    write(fd, Txbuffer, 24);

    if (target == TO_COMPLETE_UNSAFE_CONTROLLER){
//        If this case, send the wrong crc.
        crc[0] = 17;
    }

    write(fd, crc, 1);
    usleep(1000);             // sleep enough to transmit the 7 plus
//    printf("SEND: data is written.\n");

}

# define MAX_VOLTAGE 2
#define MIN_VOLTAGE -2

double voltage_max_min(double voltage) {

    if (voltage > MAX_VOLTAGE)
        voltage = MAX_VOLTAGE;
    else if (voltage < -MAX_VOLTAGE)
        voltage = -MAX_VOLTAGE;
    return voltage;
}


void main() {


    if (pthread_mutex_init(&lock_actuator_update, NULL) != 0) {
        printf("\n mutex init has failed\n");
        return 1;
    }

    if (pthread_mutex_init(&lock_sensor, NULL) != 0) {
        printf("\n mutex init has failed\n");
        return 1;
    }

    int commands[2];

    char *portname = "/dev/ttyACM0";
    printf("starting ... \n");
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("error %d opening %s: %s", errno, portname, strerror(errno));
        return;
    }

    set_interface_attribs(fd, B115200,
                          0);  // set speed to 115,200 bps, 8n1 (no parity)


    char c1[8] = "mmcinfo\n";
    char c2[38] = "fatload mmc 0 0x1C000000 LTZVisor.bin\n";
    char c3[14] = "go 0x1C000000\n";


    char newline[1] = "\n";

    for (int k = 0; k < 20; k++) {
        write(fd, newline, 1);
        sleep(1);
    }

    write(fd, c1, 8);
    printf("sent c1\n");
    sleep(1);

    write(fd, c2, 38);
    printf("sent c2\n");
    sleep(1);

    write(fd, c3, 14);
    printf("sent c3\n");
    sleep(1);

    set_blocking(fd, 0, 0);
    unsigned char cc;
    struct timeval stop, start;
    gettimeofday(&start, NULL);

    while (1) {
        gettimeofday(&stop, NULL);
        if (stop.tv_sec - start.tv_sec > 3) {
            break;
        }
        read(fd, &cc, 1);
        printf("%c", cc);
    }

    pthread_t thread1;
    int iret1;

//   Do not start simulation until the first command.
    pthread_mutex_lock(&start_simulation);
    iret1 = pthread_create(&thread1, NULL, simulator, fd);

    if (iret1) {
        fprintf(stderr, "Error - pthread_create() return code: %d\n", iret1);
        return;
    }
    printf("pthread_create() for thread 1 returns: %d\n", iret1);


    int first_time = 1;
    struct timeval last_time, now;
    double time_diff;

    int sensor_reading_local[6];
    int zeros[6] = {0 ,0 ,0 ,0 ,0 ,0 };

    while (1) {

        set_blocking(fd, 1, 1);
        int n = (int) read(fd, &cc, 1);

        if (cc == 0xcc) {
            printf("got cc\n");
            receive_bytes(fd, commands);
            pthread_mutex_lock(&lock_actuator_update);

            actuator_volts[0] = voltage_max_min(commands[0] / 10000.0);
            actuator_volts[1] = voltage_max_min(commands[1] / 10000.0);

            pthread_mutex_unlock(&lock_actuator_update);
            printf("recieved: \t %lf, %lf\n", actuator_volts[0],
                   actuator_volts[1]);

        } else if (cc == 0xaa) {
            printf("got aa\n");
            // When safety controller is reading
            pthread_mutex_unlock(&start_simulation);
            pthread_mutex_lock(&lock_sensor);

            for (int j = 0; j < 6; j++) {
                sensor_reading_local[j] = sensor_reading[j];
            }
            pthread_mutex_unlock(&lock_sensor);

            send_serial(fd, sensor_reading_local, TO_SAFETY_CONTROLLER);

            send_serial(fd, zeros, TO_COMPLETE_UNSAFE_CONTROLLER);

            printf("sending to trusted: \t %d, %d, %d, %d, %d, %d\n",
                   sensor_reading_local[0], sensor_reading_local[1],
                   sensor_reading_local[2], sensor_reading_local[3],
                   sensor_reading_local[4], sensor_reading_local[5]);


        } else if (cc == 0xbb) {
            printf("got bb\n");
            // When untrusted controller is reading
            pthread_mutex_lock(&lock_sensor);
            for (int j = 0; j < 6; j++) {
                sensor_reading_local[j] = sensor_reading[j];
            }

            pthread_mutex_unlock(&lock_sensor);
            send_serial(fd, sensor_reading_local, TO_UNSAFE_CONTROLLER);
            printf("sending to untrusted: \t %d, %d, %d, %d, %d, %d\n",
                   sensor_reading_local[0], sensor_reading_local[1],
                   sensor_reading_local[2], sensor_reading_local[3],
                   sensor_reading_local[4], sensor_reading_local[5]);
        } else {
            printf("%c", cc);
        }
    }


    pthread_join(thread1, NULL);

}
