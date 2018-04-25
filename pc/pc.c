#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

int
set_interface_attribs(int fd, int speed, int parity) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("error %d from tcgetattr", errno);
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
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}


//void
//set_blocking(int fd, int should_block) {
//    struct termios tty;
//    memset(&tty, 0, sizeof tty);
//    if (tcgetattr(fd, &tty) != 0) {
//        printf("error %d from tggetattr", errno);
//        return;
//    }
//
//    tty.c_cc[VMIN] = should_block ? 1 : 0;
//    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
//
//    if (tcsetattr(fd, TCSANOW, &tty) != 0)
//        printf("error %d setting term attributes", errno);
//}

void set_blocking(int fd, int should_block, int number_of_char) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        //printf("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? number_of_char : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        //printf("error %d setting term attributes", errno);
    }
}


#define SERIAL_DATA_SIZE 8

unsigned char read_8_bytes(int fd, int *commands) {
    printf("READ\n");
    unsigned char buf8[SERIAL_DATA_SIZE];
    unsigned char buf;

    while (1) {
        set_blocking(fd, 1, 1);
        int n = read(fd, &buf, 1);

        if (n != 1) {
            printf("n is not 1, n is: %d\n", n);
        }
        else {
            printf("READ: char read: %x\n", buf);
            if (buf == 0xCC) {
                int i;
                set_blocking(fd, 1, SERIAL_DATA_SIZE);
                n = read(fd, &buf8, SERIAL_DATA_SIZE);
                printf("READ: recieved bytes:\n");

                for (i = 0; i < SERIAL_DATA_SIZE; i++) {
                    printf("READ: %x\n", buf8[i]);
                }

                set_blocking(fd, 1, 1);
                int n = read(fd, &buf, 1);
                if (buf == 0xFF) {
                    printf("read END\n");
                    break;
                }

            }

        }

    }

    int vol_left = *(int *) buf8;
    int vol_right = *(int *) &buf8[4];

    printf("vol left  : %d \n", vol_left);
    printf("vol right : %d \n", vol_right);

    commands[0] = vol_left;
    commands[1] = vol_right;

    return buf8[8];
}


void send_serial(int fd, int sensor_readings[]) {

    unsigned char Txbuffer[13];
    int i;
    unsigned char buf;

    for (i = 0; i < 3; i++) {
        Txbuffer[4 * i] = (unsigned char) (sensor_readings[i] & 0xff); /* first byte */
        Txbuffer[4 * i + 1] = (unsigned char) (sensor_readings[i] >> 8 & 0xff); /* second byte */
        Txbuffer[4 * i + 2] = (unsigned char) (sensor_readings[i] >> 16 & 0xff); /* third byte */
        Txbuffer[4 * i + 3] = (unsigned char) (sensor_readings[i] >> 24 & 0xff); /* fourth byte */
    }

//	unsigned char start = 0xAA;
    Txbuffer[12] = 0xFF;

    while (1) {
        printf("SEND: in the while loop.\n");

        set_blocking(fd, 1, 1);
        int n = read(fd, &buf, 1);
        if (n == 1) {
            printf("SEND: char is: %x\n", buf);
            if (buf == 0xAA) {
                printf("SEND: I recieved BB\n");
                write(fd, Txbuffer, 13);
                usleep(1000);             // sleep enough to transmit the 7 plus
                printf("SEND: data is written.\n");
                break;
            }
        }
    }


}


void main() {

    char *portname = "/dev/ttyACM0";
    printf("starting ... \n");
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("error %d opening %s: %s", errno, portname, strerror(errno));
        return;
    }

    set_interface_attribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)

    int commands[2];
    int sensor_readings_calibrated[3];
    int vol_left, vol_right;

    while(1){
        read_8_bytes(fd, commands);
        vol_left = commands[0];
        vol_right = commands[1];
        printf("val0 is %d", vol_left);
        printf("val1 is %d", vol_right);
//
//        sensor_readings_calibrated[0] = 1;
//        sensor_readings_calibrated[1] = 2;
//        sensor_readings_calibrated[2] = 3;
//
//        send_serial(fd, sensor_readings_calibrated);
        usleep(1000);
    }
}
