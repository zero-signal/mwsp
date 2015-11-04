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

#include <stdint.h>
#include <stdlib.h>

#include "mwsp.h"
#include "mwsp_err.h"
#include "mwsp_util.h"
#include "mwsp_protocol.h"

/* RC defaults */
#define RC_MIN      (uint16_t) 1000
#define RC_MID      (uint16_t) 1500
#define RC_MAX      (uint16_t) 2000

#define RC_HERTZ    20
#define RC_REFRESH  (int64_t) (NS_PER_SEC / RC_HERTZ)    

/* MWSP ranges */
#define MWSP_HEAD_MIN       (int16_t) -180
#define MWSP_HEAD_MAX       (int16_t)  180

#define MWSP_SETTING_MIN    (uint8_t) 0
#define MWSP_SETTING_MAX    (uint8_t) 2

#define MWSP_MOTOR_MIN      (uint16_t) 1000
#define MWSP_MOTOR_MAX      (uint16_t) 2000

#define MWSP_SERVO_MIN      (uint16_t) 1000
#define MWSP_SERVO_MAX      (uint16_t) 2000

/***********************
 ** Request functions **
 ***********************/

/* 100: Request MWSP_IDENT */
int mwsp_get_ident(int fd, mwsp_ident *ident)
{
    return mwsp_get_data(fd, MWSP_IDENT, ident->data.data, (int) sizeof(ident->data.data));
}

/* 101: Request MWSP_STATUS */
int mwsp_get_status(int fd, mwsp_status *status)
{
    return mwsp_get_data(fd, MWSP_STATUS, status->data.data, (int) sizeof(status->data.data));
}

/* 102: Request MWSP_RAW_IMU */
int mwsp_get_raw_imu(int fd, mwsp_raw_imu *raw_imu)
{
    return mwsp_get_data(fd, MWSP_RAW_IMU, raw_imu->data.data, (int) sizeof(raw_imu->data.data));
}

/* 103: Request MWSP_SERVO */
int mwsp_get_servo(int fd, mwsp_servo *servo)
{
    return mwsp_get_data(fd, MWSP_SERVO, servo->data.data, (int) sizeof(servo->data.data));
}

/* 104: Request MWSP_MOTOR */
int mwsp_get_motor(int fd, mwsp_motor *motor)
{
    return mwsp_get_data(fd, MWSP_MOTOR, motor->data.data, (int) sizeof(motor->data.data));
}

/* 105: Request MWSP_RC */
int mwsp_get_rc(int fd, mwsp_rc *rc)
{
    return mwsp_get_data(fd, MWSP_RC, rc->data.data, (int) sizeof(rc->data.data));
}

/* 106: Request MWSP_RAW_GPS */
int mwsp_get_raw_gps(int fd, mwsp_raw_gps *raw_gps)
{
    return mwsp_get_data(fd, MWSP_RAW_GPS, raw_gps->data.data, (int) sizeof(raw_gps->data.data));
}

/* 107: Request MWSP_COMP_GPS */
int mwsp_get_comp_gps(int fd, mwsp_comp_gps *comp_gps)
{
    return mwsp_get_data(fd, MWSP_COMP_GPS, comp_gps->data.data, (int) sizeof(comp_gps->data.data));
}

/* 108: Request MWSP_ATTITUDE */
int mwsp_get_attitude(int fd, mwsp_attitude *attitude)
{
    return mwsp_get_data(fd, MWSP_ATTITUDE, attitude->data.data, (int) sizeof(attitude->data.data));
}

/* 109: Request MWSP_ALTITUDE */
int mwsp_get_altitude(int fd, mwsp_altitude *altitude)
{
    return mwsp_get_data(fd, MWSP_ALTITUDE, altitude->data.data, (int) sizeof(altitude->data.data));
}

/* 110: Request MWSP_ANALOG */
int mwsp_get_analog(int fd, mwsp_analog *analog)
{
    return mwsp_get_data(fd, MWSP_ANALOG, analog->data.data, (int) sizeof(analog->data.data));
}

/* 111: Request MWSP_RC_TUNING */
int mwsp_get_rc_tuning(int fd, mwsp_rc_tuning *rc_tuning)
{
    return mwsp_get_data(fd, MWSP_RC_TUNING, rc_tuning->data.data, (int) sizeof(rc_tuning->data.data));
}

/* 112: Request MWSP_PID */
int mwsp_get_pid(int fd, mwsp_pid *pid)
{
    return mwsp_get_data(fd, MWSP_PID, pid->data.data, (int) sizeof(pid->data.data));
}

/* 114: Request MWSP_MISC */
int mwsp_get_misc(int fd, mwsp_misc *misc)
{
    return mwsp_get_data(fd, MWSP_MISC, misc->data.data, (int) sizeof(misc->data.data));
}

/* 115: Request MWSP_MOTOR_PINS */
int mwsp_get_motor_pins(int fd, mwsp_motor_pins *pins)
{
    return mwsp_get_data(fd, MWSP_MOTOR_PINS, pins->data.data, (int) sizeof(pins->data.data));
}

/* 118: Request MWSP_WP */
int mwsp_get_wp(int fd, mwsp_wp *wp)
{
    return mwsp_get_data(fd, MWSP_WP, wp->data.data, (int) sizeof(wp->data.data));
}

/*******************
 ** Set functions **
 *******************/

/* 200: MWSP_RAW_RC */
int mwsp_set_raw_rc(int fd, mwsp_rc *rc)
{
    ssize_t size = sizeof(rc->data.data);
    uint16_t *prc = (uint16_t *) &(rc->data.data);
    uint16_t *end = (uint16_t *) &(rc->data.data) + (size / sizeof(uint16_t));

    do {
        uint16_t val = (uint16_t) *prc++;
        if((val < MWSP_MOTOR_MIN) || (val > MWSP_MOTOR_MAX)){
            return MWSP_ERR_RANGE;
        }
    } while(prc < end);

    return mwsp_set_data(fd, MWSP_SET_RAW_RC, rc->data.data, (int) sizeof(rc->data.data), MWSP_ACK_NO);
}

/* 201: MWSP_RAW_GPS */
int mwsp_set_raw_gps(int fd, mwsp_raw_gps *gps)
{
    return mwsp_set_data(fd, MWSP_SET_RAW_GPS, gps->data.data, (int) sizeof(gps->data.data), MWSP_ACK_YES);
}

/* 210: MWSP_SELECT_SETTING */
int mwsp_set_setting(int fd, mwsp_setting *set)
{
    uint8_t setting = set->data.members.setting;

    if((setting < MWSP_SETTING_MIN) || (setting > MWSP_SETTING_MAX)){
        return MWSP_ERR_RANGE;
    }

    return mwsp_set_data(fd, MWSP_SELECT_SETTING, set->data.data, (int) sizeof(set->data.data), MWSP_ACK_YES);
}

/* 211: MWSP_SET_HEAD */
int mwsp_set_head(int fd, mwsp_heading *head)
{
    int16_t heading = head->data.members.heading;

    if((heading < MWSP_HEAD_MIN) || (heading > MWSP_HEAD_MAX)){
        return MWSP_ERR_RANGE;
    }

    return mwsp_set_data(fd, MWSP_SET_HEAD, head->data.data, (int) sizeof(head->data.data), MWSP_ACK_YES);
}

/* 214: MWSP_SET_MOTOR */
int mwsp_set_motor(int fd, mwsp_motor *motor)
{
    ssize_t size = sizeof(motor->data.data);
    uint16_t *pmotor = (uint16_t *) &(motor->data.data);
    uint16_t *end = (uint16_t *) &(motor->data.data) + (size / sizeof(uint16_t));

    do {
        uint16_t val = (uint16_t) *pmotor++;
        if((val < MWSP_MOTOR_MIN) || (val > MWSP_MOTOR_MAX)){
            return MWSP_ERR_RANGE;
        }
    } while(pmotor < end);

    return mwsp_set_data(fd, MWSP_SET_MOTOR, motor->data.data, (int) sizeof(motor->data.data), MWSP_ACK_YES);
}

/***********************
 ** Command functions **
 ***********************/

/* 205: MWSP_ACC_CALIBRATION */
int mwsp_acc_calibration(int fd)
{
    return mwsp_send_command(fd, MWSP_ACC_CALIBRATION, MWSP_ACK_YES);
}

/* 206: MWSP_MAG_CALIBRATION */
int mwsp_mag_calibration(int fd)
{
    return mwsp_send_command(fd, MWSP_MAG_CALIBRATION, MWSP_ACK_YES);
}

/* 208: MWSP_RESET_CONF */
int mwsp_reset_conf(int fd)
{
    return mwsp_send_command(fd, MWSP_RESET_CONF, MWSP_ACK_YES);
}

/* 240: MWSP_BIND */
int mwsp_bind(int fd)
{
    return mwsp_send_command(fd, MWSP_BIND, MWSP_ACK_YES);
}

/* 250: MWSP_EEPROM_WRITE */
int mwsp_eeprom_write(int fd)
{
    return mwsp_send_command(fd, MWSP_EEPROM_WRITE, MWSP_ACK_YES);
}

/*************************
 ** Composite functions **
 *************************/

/* Helper function to send RC data at correct
 * frequency, for a fixed period.
 */
int mwsp_send_arm_disarm(int fd, mwsp_rc *rc)
{
    int64_t start, pulse, now;
    pulse = now = start = mwsp_get_time();

    struct timespec ts, tr;

    ts.tv_sec = 0;
    ts.tv_nsec = RC_REFRESH;

    while(now < (start + NS_PER_SEC)){
        ssize_t ret = mwsp_set_raw_rc(fd, rc);
        if(ret < 0){
            return ret;
        }

        nanosleep(&ts,&tr);
        now = mwsp_get_time();
    }

    return MWSP_SUCCESS;
}

/* Arms the multicopter */
int mwsp_arm(int fd)
{
    mwsp_rc rc;

    /* set the rc data */
    rc.data.members.roll  = RC_MID;
    rc.data.members.pitch = RC_MID;
    rc.data.members.yaw   = RC_MAX;
    rc.data.members.thro  = RC_MIN;

    rc.data.members.aux1 = RC_MIN;
    rc.data.members.aux2 = RC_MIN;
    rc.data.members.aux3 = RC_MIN;
    rc.data.members.aux4 = RC_MIN;

    /* send rc data */
    return mwsp_send_arm_disarm(fd, &rc);
}

/* Disarms the multicopter */
int mwsp_disarm(int fd)
{
    mwsp_rc rc;

    /* set the rc data */
    rc.data.members.roll  = RC_MID;
    rc.data.members.pitch = RC_MID;
    rc.data.members.yaw   = RC_MIN;
    rc.data.members.thro  = RC_MIN;

    rc.data.members.aux1 = RC_MIN;
    rc.data.members.aux2 = RC_MIN;
    rc.data.members.aux3 = RC_MIN;
    rc.data.members.aux4 = RC_MIN;

    /* send rc data */
    return mwsp_send_arm_disarm(fd, &rc);
}
