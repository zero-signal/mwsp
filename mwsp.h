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

#ifndef MWSP_H_
#define MWSP_H_

#include "mwsp_types.h"

/* 
 * MWSP Messages 
 *
 * Mostly taken from Multwii firmware source
 * Copyright (c) - alexinparis and others
 *
 * https://github.com/multiwii/multiwii-firmware
 *
 */

#define MWSP_PRIVATE                1 

#define MWSP_IDENT                  100
#define MWSP_STATUS                 101
#define MWSP_RAW_IMU                102
#define MWSP_SERVO                  103
#define MWSP_MOTOR                  104
#define MWSP_RC                     105
#define MWSP_RAW_GPS                106
#define MWSP_COMP_GPS               107
#define MWSP_ATTITUDE               108
#define MWSP_ALTITUDE               109
#define MWSP_ANALOG                 110
#define MWSP_RC_TUNING              111
#define MWSP_PID                    112
#define MWSP_BOX                    113
#define MWSP_MISC                   114
#define MWSP_MOTOR_PINS             115
#define MWSP_BOXNAMES               116
#define MWSP_PIDNAMES               117
#define MWSP_WP                     118
#define MWSP_BOXIDS                 119
#define MWSP_SERVO_CONF             120

#define MWSP_NAV_STATUS             121
#define MWSP_NAV_CONFIG             122

#define MWSP_CELLS                  130

#define MWSP_SET_RAW_RC             200
#define MWSP_SET_RAW_GPS            201
#define MWSP_SET_PID                202
#define MWSP_SET_BOX                203
#define MWSP_SET_RC_TUNING          204
#define MWSP_ACC_CALIBRATION        205
#define MWSP_MAG_CALIBRATION        206
#define MWSP_SET_MISC               207
#define MWSP_RESET_CONF             208
#define MWSP_SET_WP                 209
#define MWSP_SELECT_SETTING         210
#define MWSP_SET_HEAD               211
#define MWSP_SET_SERVO_CONF         212
#define MWSP_SET_MOTOR              214
#define MWSP_SET_NAV_CONFIG         215

#define MWSP_SET_ACC_TRIM           239
#define MWSP_ACC_TRIM               240
#define MWSP_BIND                   241

#define MWSP_EEPROM_WRITE           250

#define MWSP_DEBUGMSG               253
#define MWSP_DEBUG                  254

/*
 * Function prototypes
 *
 */

int mwsp_get_ident(int fd, mwsp_ident *ident);
int mwsp_get_status(int fd, mwsp_status *status);
int mwsp_get_raw_imu(int fd, mwsp_raw_imu *raw_imu);
int mwsp_get_servo(int fd, mwsp_servo *servo);
int mwsp_get_motor(int fd, mwsp_motor *motor);
int mwsp_get_rc(int fd, mwsp_rc *rc);
int mwsp_get_raw_gps(int fd, mwsp_raw_gps *raw_gps);
int mwsp_get_comp_gps(int fd, mwsp_comp_gps *comp_gps);
int mwsp_get_attitude(int fd, mwsp_attitude *attitude);
int mwsp_get_altitude(int fd, mwsp_altitude *altitude);
int mwsp_get_analog(int fd, mwsp_analog *analog);
int mwsp_get_rc_tuning(int fd, mwsp_rc_tuning *rc_tuning);
int mwsp_get_pid(int fd, mwsp_pid *pid);
int mwsp_get_misc(int fd, mwsp_misc *misc);
int mwsp_get_motor_pins(int fd, mwsp_motor_pins *pins);
int mwsp_get_wp(int fd, mwsp_wp *wp);

int mwsp_set_raw_rc(int fd, mwsp_rc *rc);
int mwsp_set_raw_gps(int fd, mwsp_raw_gps *gps);
int mwsp_set_head(int fd, mwsp_heading *heading);
int mwsp_set_setting(int fd, mwsp_setting *setting);
int mwsp_set_motor(int fd, mwsp_motor *motor);

int mwsp_acc_calibration(int fd);
int mwsp_mag_calibration(int fd);
int mwsp_reset_conf(int fd);
int mwsp_bind(int fd);
int mwsp_eeprom_write(int fd);

int arm(int fd);
int disarm(int fd);

#endif /* MWSP_H_ */
