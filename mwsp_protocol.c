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

#include <string.h>

#include <stdlib.h>

#include "mwsp_err.h"
#include "mwsp_util.h"
#include "mwsp_serial.h"
#include "mwsp_protocol.h"

/* initialises the header structure prior to transmission */
void mwsp_build_header(mwsp_header *header, int length, int command, int chksum)
{
    memset(header, 0, MWSP_HEADER_LEN);

    strncpy(header->preamble, MWSP_HEADER, (sizeof(char) * (sizeof(MWSP_HEADER) - 1)));

    header->direction = MWSP_FC_IN;
    header->length  = length;
    header->command = command;
    header->chksum  = chksum;
}

/* check the data received for correctness */
int mwsp_check_header(mwsp_header *header, char *buffer, int length, int command)
{
    /* check preamble */
    if(strncmp(buffer, MWSP_HEADER, 2) != 0) {
        return MWSP_ERR_PREAMBLE;
    }
    buffer += (int) (sizeof(char) * (sizeof(MWSP_HEADER) - 1));

    /* check direction is correct, i.e '>' */
    if(*buffer++ != MWSP_FC_OUT) {
        /* something went wrong, but try to give a meaningful error */
        if(*(--buffer) == MWSP_FC_ERR){
            return MWSP_ERR_FC;
        } else {
            return MWSP_ERR_UNKNOWN;
        }
    }

    /* check the length */
    if((uint8_t) *buffer == length) {
        header->length = (uint8_t) *buffer++;
    } else {
        return MWSP_ERR_HEADER_LEN;
    }

    /* check the command */
    if((uint8_t) *buffer == command){
        header->command = (uint8_t) *buffer++; 
    } else {
        return MWSP_ERR_COMMAND;
    }

    return MWSP_SUCCESS;
}


/* computes the checksum for a given request/response */
int mwsp_compute_checksum(int len, int command, char *buffer)
{
    if((len < 0) || (command < 0)){
        return MWSP_ERR_DATA;
    } 
    
    int sum = len ^ command;

    if (len > 0){
        int i;
        for (i = 0; i < len; i++){
            sum ^= (uint8_t) *buffer++;
        }
    }

    return sum;
}

/* reads the acknowledgement from a data set command */
int mwsp_read_ack(int fd, mwsp_header *header, int command)
{
    char buffer[MWSP_HEADER_LEN];

    memset(buffer, 0, (sizeof(char) * MWSP_HEADER_LEN));
    
    ssize_t rtn = mwsp_read(fd, buffer, (sizeof(char) * MWSP_HEADER_LEN));
    if(rtn != MWSP_HEADER_LEN) {
        return MWSP_ERR_READ;
    }

    /* check the received data */
    rtn = mwsp_check_header(header, buffer, 0, command);
    if(rtn < 0){
        return rtn;
    }

    /* make sure checksum is correct */
    if(header->chksum != command){
        return MWSP_ERR_CHKSUM;
    }

    return MWSP_SUCCESS;
}

/* get data from the flight controller */
int mwsp_get_data(int fd, int command, char *data, int length)
{
    char *buffer, *pbuf;
    mwsp_header header;

    /* initialise the data storage */
    buffer = malloc(sizeof(char) * (length + MWSP_HEADER_LEN));

    /* build the payload header */
    mwsp_build_header(&header, 0, command, command);  /* checksum will be 0 ^ command */

    /* send the request */
    ssize_t rtn = mwsp_write(fd, &header, MWSP_HEADER_LEN);
    if(rtn != MWSP_HEADER_LEN){
        return MWSP_ERR_WRITE;
    }

    /* read the repsonse */
    rtn = mwsp_read(fd, buffer, (length + MWSP_HEADER_LEN));
    if(rtn < 0){
        return MWSP_ERR_READ;
    }

    /* check the received data */
    rtn = mwsp_check_header(&header, buffer, length, command);
    if(rtn < 0){
        return rtn;
    }

    /* header checks out, copy the data */
    pbuf = buffer + (MWSP_HEADER_LEN - 1);
    memcpy(data, pbuf, header.length);
    pbuf += header.length;

    /* ensure the checksum received is correct */
    if ((uint8_t) *pbuf == mwsp_compute_checksum(header.length, header.command, data)){
        header.chksum = (uint8_t) *pbuf;
    } else {
        return MWSP_ERR_CHKSUM;
    }

    free(buffer);

    return MWSP_SUCCESS;
}

/* send data to the flight controller */
int mwsp_set_data(int fd, int command, char *data, int length, int ack)
{
    int chksum;
    char *buffer, *pbuf;
    mwsp_header header;

    /* initialise data storage */
    buffer = malloc(sizeof(char) * (length + MWSP_HEADER_LEN));
    pbuf = buffer;

    /* build the payload header */
    chksum = mwsp_compute_checksum(length, command, data);
    mwsp_build_header(&header, length, command, chksum);

    /*  build the output buffer */
    memcpy(pbuf, &header, (MWSP_HEADER_LEN - 1));    /* 1. copy header into buffer */
    pbuf += (MWSP_HEADER_LEN - 1);

    memcpy(pbuf, data, length);                      /* 2. copy data into buffer */
    pbuf += length;

    *pbuf = (uint8_t) header.chksum;                 /* 3. append checksum */

    /* send the data */
    ssize_t rtn = mwsp_write(fd, buffer, (length + MWSP_HEADER_LEN));
    if(rtn != (length + MWSP_HEADER_LEN)){
        return MWSP_ERR_WRITE;
    }

    /* if we expect a response, read and check it */
    if(ack & MWSP_ACK_YES){

        /* reset header */
        header.length = 0;
        header.chksum = command;

        rtn = mwsp_read_ack(fd, &header, command);
        if(rtn < 0){
            return rtn;
        }
    }

    free(buffer);

    return MWSP_SUCCESS;
}

/* send command to the flight controller */
int mwsp_send_command(int fd, int command, int ack)
{
    mwsp_header header;
    mwsp_build_header(&header, 0, command, command); /* sending no data, chksum == command */

    /* send the command */
    ssize_t rtn = mwsp_write(fd, &header, (sizeof(char) * MWSP_HEADER_LEN));
    if(rtn != MWSP_HEADER_LEN){
        return MWSP_ERR_WRITE;
    }

    /* if we expect a response, read and check it */
    if(ack & MWSP_ACK_YES){
        rtn = mwsp_read_ack(fd, &header, command);
        if(rtn < 0){
            return rtn;
        }
    }
    
    return MWSP_SUCCESS;
}
