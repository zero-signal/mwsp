/*
 * mwsp - Multiwii Serial Protocol - interface library
 * Copyright (c) 2015 Michael Carter
 *
 * zerosignal1982@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>

#include <stdlib.h>

#include "mwsp.h"

#define BAUDRATE    B115200

#define SLEEP_SECS  (time_t) 0
#define SLEEP_NSECS (long)   5000000L  /* 1/200s */

#define _POSIX_SOURCE 1

/* Multiwii Serial Protocol */
#define MWSP_HEADER "\x24\x4D"
#define MWSP_FC_IN  0x3C
#define MWSP_FC_OUT 0x3E
#define MWSP_FC_ERR 0x21

/* request */
struct mwsp_request {
    char preamble[2];
    char direction;

    mwsp_header header;
};

/* serial port data structure */
struct termios newtios;

/***********************
 ** Utility functions **
 ***********************/

/* initialise serial port */
int mwsp_connect(char *port)
{
    int fd;

    /* open serial port */
    fd = open(port, O_RDWR | O_NOCTTY );
    if (fd < 0) { 
        return -1;
    }

    memset(&newtios, 0, sizeof(newtios));

    /* configure serial port */
    newtios.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtios.c_iflag = IGNPAR;
    newtios.c_oflag = 0;

    /* flush serial port and activate settings */
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&newtios);

    return fd;
}

/* close serial port */
int mwsp_disconnect(int fd)
{
    if(close(fd) < 0){
        return -1;
    } else {
        return 0;
    }
}

/* write generic data to the serial port */
int mwsp_write(int fd, void* data, int len)
{
    int wrtn;

    /* write data */
    tcflush(fd, TCIOFLUSH);
    wrtn = write(fd, data, len);

    if(wrtn < 0){
        return -1;
    } else {
        return wrtn;
    }
}

/* read generic data from the serial port */
int mwsp_read(int fd, char *buffer, int len)
{
    int rrtn;

    memset(buffer, 0, len);

    /* configure serial port to read the right amount of data */
    newtios.c_cc[VMIN]  = len;
    newtios.c_cc[VTIME] = 1;

    /* apply the settings */
    tcsetattr(fd,TCSANOW,&newtios);

    /* read data */
    rrtn = read(fd, buffer, len);
    tcflush(fd, TCIOFLUSH);

    if(rrtn < 0){
        return -1;
    } else {
        return rrtn;
    }
}

int mwsp_get_data(int fd, mwsp_header *header, int command, char *data, int len)
{
    /* TODO: Fix return values!!! */

    ssize_t wrtn, rrtn;
    char *buffer, *pbuf;

    buffer = malloc(sizeof(char) * (len + MWSP_HEADER_LEN));
    pbuf = buffer;

    /* construct a request */
    struct mwsp_request req;
    memset(&req, 0, sizeof(req));
    memset(data, 0, len);

    /* populate the preamble */
    strncpy(req.preamble, MWSP_HEADER, sizeof(req.preamble));
    req.direction = MWSP_FC_IN;

    req.header.length  = 0;
    req.header.command = command;

    /* calculate the checksum */
    req.header.chksum  = command;  /* TODO: do a real chksum calculation function here! */

    /* send the correct request */
    wrtn = mwsp_write(fd, &req, sizeof(req));
    if(wrtn != MWSP_HEADER_LEN){
        return -1;
    }

    /* read the repsonse */
    rrtn = mwsp_read(fd, buffer, (len + MWSP_HEADER_LEN));
    if(rrtn < 0){
        return -2;
    }

    /* check preamble */
    if(strncmp(buffer, MWSP_HEADER, 2) != 0) {
        return -3;
    }
    pbuf += (int) (sizeof(char) * (sizeof(MWSP_HEADER) - 1));

    /* check direction is correct, i.e '>' */
    if(*pbuf++ != MWSP_FC_OUT) {
        /* something went wrong, but try to give a meaningful error */
        if(*(--pbuf) == MWSP_FC_ERR){
            return -4;
        } else {
            return -5;
        }
    }

    /* check the length */
    if((uint8_t) *pbuf == len) {
        header->length = (uint8_t) *pbuf++;
    } else {
        return -6;
    }

    /* check the command */
    header->command = (uint8_t) *pbuf++; 

    /* if the size if correct, copy the data in the receiving structure */
    if(header->length > 0 && (header->length == len)){
        memcpy(data, pbuf, header->length);
        pbuf += (header->length);
    } else {
        return -7;
    }

    /* ensure the checksum received is correct */
    if ((uint8_t) *pbuf == compute_checksum(header->length, header->command, data)){
        header->chksum = (uint8_t) *pbuf;

        printf("Chksum: %i\n", header->chksum);
    } else {
        return -8;
    }

    free(buffer);

    return 0;
}

/* computes the checksum for a given request/response */
int compute_checksum(int len, int command, char *data)
{
    if((len < 0) || (command < 0)){
        return -1;
    } 
    
    int sum = len ^ command;

    if (len > 0){
        int i;
        for (i = 0; i < len; i++){
            sum ^= (uint8_t) *data++;
        }
    }

    return sum;
}

/***********************
 ** Request functions **
 ***********************/

/* 100: Request MWSP_IDENT */
int mwsp_req_ident(int fd, mwsp_ident *ident)
{
    return mwsp_get_data(fd, &(ident->header), MWSP_IDENT, ident->data.data, (int) sizeof(ident->data.data));
}

/* 101: Request MWSP_STATUS */
int mwsp_req_status(int fd, mwsp_status *status)
{
    return mwsp_get_data(fd, &(status->header), MWSP_STATUS, status->data.data, (int) sizeof(status->data.data));
}

/* 102: Request MWSP_RAW_IMU */
int mwsp_req_raw_imu(int fd, mwsp_raw_imu *raw_imu)
{
    return mwsp_get_data(fd, &(raw_imu->header), MWSP_RAW_IMU, raw_imu->data.data, (int) sizeof(raw_imu->data.data));
}

/* 103: Request MWSP_SERVO */
int mwsp_req_servo(int fd, mwsp_servo *servo)
{
    return mwsp_get_data(fd, &(servo->header), MWSP_SERVO, servo->data.data, (int) sizeof(servo->data.data));
}

/* 104: Request MWSP_MOTOR */
int mwsp_req_motor(int fd, mwsp_motor *motor)
{
    return mwsp_get_data(fd, &(motor->header), MWSP_MOTOR, motor->data.data, (int) sizeof(motor->data.data));
}

/* 105: Request MWSP_RC */
int mwsp_req_rc(int fd, mwsp_rc *rc)
{
    return mwsp_get_data(fd, &(rc->header), MWSP_RC, rc->data.data, (int) sizeof(rc->data.data));
}

/* 106: Request MWSP_RAW_GPS */
int mwsp_req_raw_gps(int fd, mwsp_raw_gps *raw_gps)
{
    return mwsp_get_data(fd, &(raw_gps->header), MWSP_RAW_GPS, raw_gps->data.data, (int) sizeof(raw_gps->data.data));
}

/* 107: Request MWSP_COMP_GPS */
int mwsp_req_comp_gps(int fd, mwsp_comp_gps *comp_gps)
{
    return mwsp_get_data(fd, &(comp_gps->header), MWSP_COMP_GPS, comp_gps->data.data, (int) sizeof(comp_gps->data.data));
}

/* 108: Request MWSP_ATTITUDE */
int mwsp_req_attitude(int fd, mwsp_attitude *attitude)
{
    return mwsp_get_data(fd, &(attitude->header), MWSP_ATTITUDE, attitude->data.data, (int) sizeof(attitude->data.data));
}

/* 109: Request MWSP_ALTITUDE */
int mwsp_req_altitude(int fd, mwsp_altitude *altitude)
{
    return mwsp_get_data(fd, &(altitude->header), MWSP_ALTITUDE, altitude->data.data, (int) sizeof(altitude->data.data));
}

/* 110: Request MWSP_ANALOG */
int mwsp_req_analog(int fd, mwsp_analog *analog)
{
    return mwsp_get_data(fd, &(analog->header), MWSP_ANALOG, analog->data.data, (int) sizeof(analog->data.data));
}

/* 111: Request MWSP_RC_TUNING */
int mwsp_req_rc_tuning(int fd, mwsp_rc_tuning *rc_tuning)
{
    return mwsp_get_data(fd, &(rc_tuning->header), MWSP_RC_TUNING, rc_tuning->data.data, (int) sizeof(rc_tuning->data.data));
}

/* 112: Request MWSP_PID */
int mwsp_req_pid(int fd, mwsp_pid *pid)
{
    return mwsp_get_data(fd, &(pid->header), MWSP_PID, pid->data.data, (int) sizeof(pid->data.data));
}

/* 114: Request MWSP_MISC */
int mwsp_req_misc(int fd, mwsp_misc *misc)
{
    return mwsp_get_data(fd, &(misc->header), MWSP_MISC, misc->data.data, (int) sizeof(misc->data.data));
}

/* 115: Request MWSP_MOTOR_PINS */
int mwsp_req_motor_pins(int fd, mwsp_motor_pins *pins)
{
    return mwsp_get_data(fd, &(pins->header), MWSP_MOTOR_PINS, pins->data.data, (int) sizeof(pins->data.data));
}

/* 118: Request MWSP_WP */
int mwsp_req_wp(int fd, mwsp_wp *wp)
{
    return mwsp_get_data(fd, &(wp->header), MWSP_WP, wp->data.data, (int) sizeof(wp->data.data));
}

/*******************
 ** Set functions **
 *******************/
