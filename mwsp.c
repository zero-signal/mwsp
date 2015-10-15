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

#include <stdlib.h>

#include "mwsp.h"

#define BAUDRATE    B115200
#define SERIALPORT  "/dev/ttyUSB0"

#define _POSIX_SOURCE 1

/* Multiwii Serial Protocol */
#define MWSP_HEADER "\x24\x4D"
#define MWSP_FC_IN  0x3C
#define MWSP_FC_OUT 0x3E

/* request */
struct mwsp_request {
    char preamble[2];
    char direction;

    mwsp_header header;
};

/***********************
 ** Utility functions **
 ***********************/

/* initialise serial port */
int mwsp_connect(char *port)
{
    int fd;
    struct termios newtios;

    /* open serial port */
    fd = open(port, O_RDWR | O_NOCTTY );
    if (fd < 0) { 
        return -1;
    }

    memset(&newtios, 0, sizeof(newtios));

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

/* construct and send a request for data */
int mwsp_send_request(int fd, int length, int code)
{
    int wrtn;

    struct mwsp_request req;
    memset(&req, 0, sizeof(req));

    strncpy(req.preamble, MWSP_HEADER, sizeof(req.preamble));

    req.direction = MWSP_FC_IN;
    req.header.length  = length;
    req.header.command = code;
    req.header.chksum  = code;  /* TODO: do a real chksum calculation function here! */

    tcflush(fd, TCIOFLUSH);
    wrtn = write(fd, &req, sizeof(req));

    if(wrtn < 0){
        return -1;
    } else {
        return wrtn;
    }
}

/* read and parse the response from a request for data */
int mwsp_read_response(int fd, mwsp_header *header, char *data, int len)
{
    uint8_t rlen;
    ssize_t rrtn;

    char *pbuf;
    char buffer[512];

    memset(buffer, 0, (sizeof(char) * sizeof(buffer)));
    pbuf = buffer;

    /* read the data in and check the protocol header */
    rrtn = read(fd, buffer, (sizeof(char) * sizeof(buffer)));

    if(strncmp(buffer, MWSP_HEADER, 2) != 0) {
        return -1;
    }
    pbuf += (int) (sizeof(char) * (sizeof(MWSP_HEADER) - 1));

    /* check direction is correct, i.e '>' */
    if(*pbuf++ != MWSP_FC_OUT) {
        return -2;
    }

    /* check the length */
    rlen = (uint8_t) *pbuf++;
    if(rlen == len) {
        header->length = rlen;
    } else {
        return -2;
    }

    /* check the code */
    header->command = (uint8_t) *pbuf++; 

    /* if the size if correct, copy the data in the receiving structure */
    if(header->length > 0 && (header->length == len)){
        memcpy(data, pbuf, header->length);
        pbuf += (header->length - 1);
    } else {
        return -3;
    }

    /* check the checksum */
    header->chksum = (uint8_t) *pbuf;

    return rrtn;
}

/***********************
 ** Request functions **
 ***********************/

int mwsp_req_ident(int fd, mwsp_ident *ident)
{
    ssize_t wrtn, rrtn;

    if(ident != NULL){

        /* send the correct request */
        wrtn = mwsp_send_request(fd, 0, MWSP_IDENT);

        if(wrtn != MWSP_REQ_LENGTH){
            return -1;
        }

        /* give the flight controller time to respond */
        sleep(1);

        rrtn = mwsp_read_response(fd, &(ident->header), ident->data.data, (int) sizeof(ident->data.data));

        if(rrtn >= 0){
            /* check the header */
            printf("Length of data received is: %i\n", ident->header.length);
            printf("Command received is: %i\n", ident->header.command);
            printf("Checksum received is: %i\n", ident->header.chksum);

            printf("Version is: %i\n", ident->data.members.version);
            printf("Type is: %i\n", ident->data.members.type);
            printf("MWSP version is: %i\n", ident->data.members.mwsp_ver);
            printf("Capability is: %i\n", ident->data.members.capability);

            

            /* calculate the chksum */

        } else {
            return -2;
        }
    } else {
        return -3;
    }
    return rrtn;
}

/*******************
 ** Set functions **
 *******************/



/***********************
 ** Support functions **
 ***********************/
int check_res_header(void){

}
