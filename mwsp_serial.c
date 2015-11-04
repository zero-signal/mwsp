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
#include <string.h>
#include <stdint.h>

#include <stdlib.h>

#include "mwsp_err.h"
#include "mwsp_util.h"
#include "mwsp_serial.h"
#include "mwsp_protocol.h"

#define BAUDRATE    B115200

#define MWSP_SERIAL_RETRY   3

/***********************
 ** Utility functions **
 ***********************/

/* initialise serial port */
int mwsp_connect(char *port)
{
    struct termios newtios;

    /* open serial port */
    int fd = open(port, O_RDWR | O_NOCTTY );
    if (fd < 0) { 
        return MWSP_FAILURE;
    }

    memset(&newtios, 0, sizeof(newtios));

    /* configure serial port */
    newtios.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtios.c_iflag = IGNPAR;
    newtios.c_oflag = 0;

    newtios.c_cc[VMIN]  = 0;
    newtios.c_cc[VTIME] = 1; 

    /* flush serial port and activate settings */
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd,TCSANOW,&newtios);

    return fd;
}

/* close serial port */
int mwsp_disconnect(int fd)
{
    if(close(fd) < 0){
        return MWSP_FAILURE;
    }

    return MWSP_SUCCESS;
}

/* write generic data to the serial port */
int mwsp_write(int fd, void* data, int len)
{
    /* write data */
    tcflush(fd, TCIOFLUSH);
    int wrt = write(fd, data, len);

    if(wrt < 0){
        return MWSP_ERR_WRITE;
    }

#ifdef DEBUG
    hex_dump("mwsp_write", data, len);
#endif

    return wrt;
}

/* read generic data from the serial port */
int mwsp_read(int fd, char *buffer, int len)
{
    char *pbuf;
    int ttl, rd, rem, cnt;

    ttl = rd = cnt = 0;
    rem = len;

    memset(buffer, 0, len);
    pbuf = buffer;

    do {
        rd = read(fd, pbuf, rem);

        /* if we read something */
        if(rd > 0){
            ttl += rd;
            rem -= rd;

            pbuf += rd;

            /* we have read all we expected */
            if(ttl == len){
                break;
            }

            /* if not, read some more */
            if((ttl > 0) && (ttl < len)){
                continue;
            }
        }
        cnt++;
    } while((ttl < len) && (cnt < MWSP_SERIAL_RETRY));

    tcflush(fd, TCIOFLUSH);

#ifdef DEBUG
    hex_dump("mwsp_read", buffer, len);
#endif

    if(ttl == 0){
        return MWSP_ERR_READ;
    }

    return ttl;
}
