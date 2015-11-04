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

#ifndef MWSP_PROTOCOL_H_
#define MWSP_PROTOCOL_H_

#include "mwsp_types.h"

#define MWSP_ACK_NO   0x0
#define MWSP_ACK_YES  0x1
 
/* Multiwii Serial Protocol */
#define MWSP_HEADER "\x24\x4D"
#define MWSP_FC_IN  0x3C
#define MWSP_FC_OUT 0x3E
#define MWSP_FC_ERR 0x21

#define MWSP_HEADER_LEN             6

/*
 * Function prototypes
 *
 */

int mwsp_read_ack(int fd, mwsp_header *header, int command);

void mwsp_build_header(mwsp_header *header, int length, int command, int chksum);
int mwsp_check_header(mwsp_header *header, char *buffer, int length, int command);

int mwsp_compute_checksum(int len, int command, char *buffer);

int mwsp_get_data(int fd, int command, char *data, int length);
int mwsp_set_data(int fd, int command, char *data, int length, int ack);
int mwsp_send_command(int fd, int command, int ack);

#endif /* MWSP_PROTOCOL_H_ */
