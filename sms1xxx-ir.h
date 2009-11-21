/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2009 - Ganaël Laplanche, http://contribs.martymac.org
 *
 *  This driver contains code taken from the FreeBSD dvbusb driver:
 *
 *  Copyright (C) 2006 - 2007 Raaf
 *  Copyright (C) 2004 - 2006 Patrick Boettcher
 *
 *  This driver contains code taken from the Linux siano driver:
 *
 *  Copyright (c), 2005-2008 Siano Mobile Silicon, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation;
 *
 *  Software distributed under the License is distributed on an "AS IS"
 *  basis, WITHOUT WARRANTY OF ANY KIND, either express or implied.
 *
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef SMS1XXX_IR_H
#define SMS1XXX_IR_H

#include "sms1xxx.h"

#define IR_DEFAULT_TIMEOUT  100

/* Board IR configuration */
struct sms1xxx_ir {
    /* IR module state */
    u8  module_avail;    /* Has board an IR module ?
                            XXX Should be based on GPIO cfg ? */
    u8  module_started;  /* Is IR module started ? */
    struct cdev *dev;    /* ir0 device */

    /* MSG_SMS_START_IR_REQ Msg parameters */
    u32 controller;
    u32 timeout;

    /* Data handling */
#define SMS1XXX_IR_BUFSIZE  1024
    u8 buf[SMS1XXX_IR_BUFSIZE]; /* our circular buffer */
    u16 woff;            /* write offset */
    u16 wavail;          /* space available for writing */
    u16 roff;            /* read offset */
    u16 ravail;          /* space available for writing */
    struct mtx lock;
#define IR_OPEN    0x0001
#define IR_SLEEP   0x0002
    u32 state;
};

struct sms1xxx_softc;

int sms1xxx_ir_init(struct sms1xxx_softc *);
int sms1xxx_ir_exit(struct sms1xxx_softc *);
int sms1xxx_ir_put_packet(struct sms1xxx_softc *, const u8 *, u32);

#endif
