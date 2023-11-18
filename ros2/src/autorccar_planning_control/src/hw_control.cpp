#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include "hw_control.h"

int hw_control::uart_init(char* tty) {
    fd = open(tty, O_RDWR | O_NOCTTY);
    struct termios  toptions;

    /* get current serial port settings */
    tcgetattr(fd, &toptions);

    /* set baud both ways */
    cfsetispeed(&toptions, B115200);

    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    /* Canonical mode */
    toptions.c_lflag |= ICANON;

    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    if (fd > 0)
        printf("[ %s ] opened as %i\n", tty, fd);
    else
        printf("Error %i from open: %s\n", errno, strerror(errno));
        
    return fd;
}

void hw_control::uart_tx(const int fd, const geometry_msgs::msg::Vector3& msg) {

    char msgTx[8];
    double conStr, conVel;
    
    double sf = -1.5;
    double fa = 206.0306;
    double fb = -40.6541;
    double fc = 5138.7189;

    double delta = msg.x;
    double vel = msg.y;
    int cmdMode = msg.z;
    
    delta = delta * -1;

    conStr = int(sf* delta * (180/M_PI)) + Steer_PWM_N; // [deg]
    conVel = int(fa*vel + fb*vel*vel + fc);

    if (conStr >= Steer_PWM_MAX)
        conStr = Steer_PWM_MAX;
    if (conStr <= Steer_PWM_MIN)
        conStr = Steer_PWM_MIN;

    if (conVel >= 5350) //ESC_PWM_MAX)
        conVel = 5350; //ESC_PWM_MAX;
    if (conVel <= ESC_PWM_MIN)
        conVel = ESC_PWM_MIN;

    msgTx[0] = 0xff; // header
    msgTx[1] = 0xfe; // header
    msgTx[2] = ((int)conStr >> 8) & 0xff;
    msgTx[3] = (int)conStr & 0xff;
    msgTx[4] = ((int)conVel >> 8) & 0xff;
    msgTx[5] = (int)conVel & 0xff;
    msgTx[6] = ((int)cmdMode >> 8) & 0xff;
    msgTx[7] = (int)cmdMode & 0xff;

    if (cmdMode == 1)        
        std::cout << "CMD " << cmdMode << "\t UART Tx : " << ceil(delta*1000)/1000 << ", " << ceil(vel*1000)/1000 \
    	    						  << " >> "<< conStr << ", " << conVel <<std::endl;

    int cnt = write(fd, msgTx, sizeof(msgTx));
}
