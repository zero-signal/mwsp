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

#ifndef MWSP_ERR_H_
#define MWSP_ERR_H_

/* mwsp errors */
#define MWSP_SUCCESS         0
#define MWSP_FAILURE        -1

#define MWSP_ERR_WRITE      -2
#define MWSP_ERR_READ       -3
#define MWSP_ERR_PREAMBLE   -4
#define MWSP_ERR_FC         -5
#define MWSP_ERR_HEADER_LEN -6
#define MWSP_ERR_COMMAND    -7
#define MWSP_ERR_DATA       -8
#define MWSP_ERR_CHKSUM     -9
#define MWSP_ERR_RANGE      -10

#define MWSP_ERR_UNKNOWN    -128

#endif /* MWSP_ERR_H_ */
