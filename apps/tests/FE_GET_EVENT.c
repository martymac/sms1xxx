/*****************************************************************************
 * Copyright (C) 2015 Ganaël Laplanche
 *
 *   Inspired from VLC code :
 *   modules/access/dtv/linux.c (Copyright (C) 2011 Rémi Denis-Courmont)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sysexits.h>
#include <err.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <strings.h>

#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>

int main(void)
{
    int fd = -1;
    int err = 0;

    if((fd = open("/dev/dvb/adapter0/frontend0", O_RDWR)) < 0) {
        errx(EX_IOERR, "Cannot open frontend device /dev/dvb/adapter0/frontend0");
    }
    else {
        printf("Successfully opened frontend0 device !\n");
    }

    struct dvb_frontend_event ev;

    /* XXX FE_GET_EVENT always returns EWOULDBLOCK */
    err = ioctl(fd, FE_GET_EVENT, &ev);
    close(fd);

    printf("ioctl() returned %d (errno = %d)\n", err, errno);
    printf("fe_status = %d\n", ev.status);

    return (EX_OK);
}
