/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008 - Ganaël Laplanche, http://contribs.martymac.com
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

#ifndef SMS1XXX_USB_H
#define SMS1XXX_USB_H

#include "sms1xxx.h"

int sms1xxx_usb_init(struct sms1xxx_softc *);
int sms1xxx_usb_exit(struct sms1xxx_softc *);

int sms1xxx_usb_ref(struct sms1xxx_softc *);
void sms1xxx_usb_deref(struct sms1xxx_softc *);

int sms1xxx_usb_xfers_start(struct sms1xxx_softc *);
int sms1xxx_usb_xfers_stop(struct sms1xxx_softc *);

int sms1xxx_usb_write(struct sms1xxx_softc *, const u8 *, u32) ;
int sms1xxx_usb_write_and_wait(struct sms1xxx_softc *, const u8 *, u32,
    unsigned char, unsigned int);

int sms1xxx_usb_setmode(struct sms1xxx_softc *, int) ;
int sms1xxx_usb_initdevice(struct sms1xxx_softc *, int) ;
int sms1xxx_usb_reloadstart(struct sms1xxx_softc *) ;
int sms1xxx_usb_reloadexec(struct sms1xxx_softc *) ;
int sms1xxx_usb_swdtrigger(struct sms1xxx_softc *, u32) ;
int sms1xxx_usb_getversion(struct sms1xxx_softc *) ;
int sms1xxx_usb_getstatistics(struct sms1xxx_softc *) ;
int sms1xxx_usb_getpidfilterlist(struct sms1xxx_softc *) ;
int sms1xxx_usb_setfrequency(struct sms1xxx_softc *, u32, u32);
int sms1xxx_usb_add_pid(struct sms1xxx_softc *, u16);
int sms1xxx_usb_remove_pid(struct sms1xxx_softc *, u16);

#endif
