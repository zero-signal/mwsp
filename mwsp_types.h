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
    char preamble[2];
    char direction;

    uint8_t  length;
    uint8_t  command;
    uint8_t  chksum;

} mwsp_header;

/* 100: MWSP_IDENT */
typedef struct mwsp_ident {
    union {
        char data[7];

        struct {
            uint8_t  ver, type, mwsp_ver;
            uint32_t cap;
        } members;
    } data;
} mwsp_ident;

/* 101: MWSP_STATUS */
typedef struct mwsp_status {
    union {
        char data[11];

        struct {
            uint16_t cyc_time, i2c_err, sens;
            uint32_t flag;
            uint8_t  conf_cur;
        } members;
    } data;
} mwsp_status;

/* 102: MWSP_RAW_IMU */
typedef struct mwsp_raw_imu {
    union {
        char data[18];

        struct {
            int16_t accx, accy, accz;
            int16_t gyrx, gyry, gyrz;
            int16_t magx, magy, magz;
        } members;
    } data;
} mwsp_raw_imu;

/* 103: MWSP_SERVO */
typedef struct mwsp_servo {
    union {
        char data[16];

        struct {
            uint16_t servo1, servo2, servo3, servo4;
            uint16_t servo5, servo6, servo7, servo8;
        } members;
    } data;
} mwsp_servo;

/* 104: MWSP_MOTOR */
typedef struct mwsp_motor {
    union {
        char data[16];

        struct {
            uint16_t motor1, motor2, motor3, motor4;
            uint16_t motor5, motor6, motor7, motor8;
        } members;
    } data;
} mwsp_motor;

/* 105: MWSP_RC */
typedef struct mwsp_rc {
    union {
        char data[16];

        struct {
            uint16_t roll, pitch, yaw, thro;
            uint16_t aux1, aux2, aux3, aux4;
        } members;
    } data;
} mwsp_rc;

/* 106: MWSP_RAW_GPS */
typedef struct mwsp_raw_gps {
    union {
        char data[16];

        struct {
            uint8_t  fix, num_sats;
            uint32_t lat, lon;
            uint16_t alt, vel, course;
        } members;
    } data;
} mwsp_raw_gps;

/* 107: MWSP_COMP_GPS */
typedef struct mwsp_comp_gps {
    union {
        char data[5];

        struct {
            uint16_t dist_home, dir_home;
            uint8_t  upd;
        } members;
    } data;
} mwsp_comp_gps;

/* 108: MWSP_ATTITUDE */
typedef struct mwsp_attitude {
    union {
        char data[6];

        struct {
            int16_t angx, angy, head;
        } members;
    } data;
} mwsp_attitude;

/* 109: MWSP_ALTITUDE */
typedef struct mwsp_altitude {
    union {
        char data[6];

        struct {
            int32_t cm;
            int16_t cm_s;
        } members;
    } data;
} mwsp_altitude;

/* 110: MWSP_ANALOG */
typedef struct mwsp_analog {
    union {
        char data[7];

        struct {
            uint8_t vbat;
            uint16_t pwr, rssi, amps;
        } members;
    } data;
} mwsp_analog;

/* 111: MWSP_RC_TUNING */
typedef struct mwsp_rc_tuning {
    union {
        char data[7];

        struct {
            uint8_t rc_rate, rc_expo;
            uint8_t rp_rate, y_rate;
            uint8_t dyn_thr_pid;
            uint8_t thr_mid, thr_expo;
        } members;
    } data;
} mwsp_rc_tuning;

/* 112: MWSP_PID */
typedef struct mwsp_pid {
    union {
        char data[30];

        struct {
            uint8_t roll_p, roll_i, roll_d;
            uint8_t pitch_p, pitch_i, pitch_d;
            uint8_t yaw_p, yaw_i, yaw_d;
            uint8_t alt_p, alt_i, alt_d;
            uint8_t pos_p, pos_i, pos_d;
            uint8_t posr_p, posr_i, posr_d;
            uint8_t navr_p, navr_i, navr_d;
            uint8_t lvl_p, lvl_i, lvl_d;
            uint8_t mag_p, mag_i, mag_d;
            uint8_t val_p, vel_i, vel_d;
        } members;
    } data;
} mwsp_pid;

/* 114: MWSP_MISC */
typedef struct mwsp_misc {
    union {
        char data[22];

        struct {
            uint16_t pwr_trig, min_thr;
            uint16_t max_thr, min_com, fs_thr;
            uint16_t plog_arm;
            uint32_t plog_life;
            uint16_t mag_dec;
            uint8_t vb_scale, vb_warn1, vbwarn2, vb_crit;
        } members;
    } data;
} mwsp_misc;

/* 115: MWSP_MOTOR_PINS */
typedef struct mwsp_motor_pins {
    union {
        char data[8];

        struct {
            uint8_t mtr1, mtr2, mtr3, mtr4;
            uint8_t mtr5, mtr6, mtr7, mtr8;
        } members;
    } data;
} mwsp_motor_pins;

/* 118: MWSP_WP */
typedef struct mwsp_wp {
    union {
        char data[18];

        struct {
            uint8_t wp_no;
            uint32_t lat, lon, alt_hld;
            uint16_t head, tm_stay;
            uint8_t nav_flag;
        } members;
    } data;
} mwsp_wp;

/* 210: MWSP_SELECT_SETTING */
typedef struct mwsp_setting {
    union {
        char data[1];

        struct {
            uint8_t setting;
        } members;
    } data;
} mwsp_setting;

/* 211: MWSP_SET_HEAD */
typedef struct mwsp_heading {
    union {
        char data[2];

        struct {
            int16_t heading;
        } members;
    } data;
} mwsp_heading;


#endif /* MWSP_TYPES_H_ */
