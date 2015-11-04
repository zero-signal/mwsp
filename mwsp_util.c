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

#include <time.h>
#include <stdio.h>
#include <stdint.h>

#include "mwsp_util.h"

/* timer helper */
int64_t mwsp_get_time()
{
    struct timespec tm;

    if(clock_gettime(CLOCK_MONOTONIC, &tm) < 0){
        return -1;
    }

    return ((tm.tv_sec * NS_PER_SEC) + tm.tv_nsec);
}

#ifdef DEBUG
/* generic hex dump function - for debugging*/
void hex_dump(char *desc, void *addr, int len)
{
    int i;
    unsigned char buf[17];
    unsigned char *pc = (unsigned char*) addr;

    if (desc != NULL)
        printf ("%s:\n", desc);

    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            if (i != 0)
                printf ("  %s\n", buf);

            printf ("  %04x ", i);
        }

        printf (" %02x", pc[i]);

        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buf[i % 16] = '.';
        else
            buf[i % 16] = pc[i];
        buf[(i % 16) + 1] = '\0';
    }

    while ((i % 16) != 0) {
        printf ("   ");
        i++;
    }

    printf ("  %s\n", buf);
}
#endif
