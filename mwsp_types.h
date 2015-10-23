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

#ifndef MWSP_TYPES_H_
#define MWSP_TYPES_H_

#include <stdint.h>

/*
 * Data structures
 *
 */

/* request/response header */
typedef struct mwsp_header {
    uint8_t  length;
    uint8_t  command;
    uint8_t  chksum;

} mwsp_header;

/* 100: MWSP_IDENT */
typedef struct mwsp_ident {
    mwsp_header header;

    union {
        char data[7];

        struct {
            uint8_t  version, type, mwsp_ver;
            uint32_t capability;
        } members;
    } data;
} mwsp_ident;

/* 101: MWSP_STATUS */
typedef struct mwsp_status {
    mwsp_header header;

    union {
        char data[11];

        struct {
            uint16_t cycle_time, i2c_errors, sensors;
            uint32_t flag;
            uint8_t  conf_current;
        } members;
    } data;
} mwsp_status;

/* 102: MWSP_RAW_IMU */
typedef struct mwsp_raw_imu {
    mwsp_header header;

    union {
        char data[18];

        struct {
            int16_t accx, accy, accz;
            int16_t gyrx, gyry, gyrz;
            int16_t magx, magy, magz;
        } members;
    } data;
} mwsp_raw_imu;

/* 106: MWSP_RAW_GPS */
typedef struct mwsp_raw_gps {
    mwsp_header header;

    union {
        char data[16];

        struct {
            uint8_t  fix, num_sats;
            uint32_t latitude, longitude;
            uint16_t altitude, velocity, course;
        } members;
    } data;
} mwsp_raw_gps;

/* 107: MWSP_COMP_GPS */
typedef struct mwsp_comp_gps {
    mwsp_header header;

    union {
        char data[5];

        struct {
            uint16_t dist_home, dir_home;
            uint8_t  update;
        } members;
    } data;
} mwsp_comp_gps;

#endif /* MWSP_TYPES_H_ */
