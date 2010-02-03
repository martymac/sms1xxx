/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2010, Ganaël Laplanche, http://contribs.martymac.org
 *
 *  This driver contains code taken from the FreeBSD dvbusb driver:
 *
 *  Copyright (C) 2006-2007, Raaf
 *  Copyright (C) 2004-2006, Patrick Boettcher
 *
 *  This driver contains code taken from the Linux siano driver:
 *
 *  Siano Mobile Silicon, Inc.
 *  MDTV receiver kernel modules.
 *  Copyright (C) 2006-2009, Uri Shkolnik
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

#ifndef SMS1XXX_DEMUX_H
#define SMS1XXX_DEMUX_H

#include "sms1xxx.h"

#define PACKET_SIZE 188

#define TS_HAS_SYNC(p) (((p)[0]) == 0x47)
#define TS_GET_PID(p) (((((u16)((p)[1])) << 8)|((p)[2])) & PIDMAX)
#define TS_GET_CC(p) (((p)[3]) & 0xF)
#define TS_HAS_PAYLOAD(p) (((p)[3]) & 0x10)
#define TS_GET_HDR_LEN(p) ((((p)[3]) & 0x20) ? (((p)[4]) + 5) : (4))
#define TS_HAS_PUSI(p) (((p)[1]) & 0x40)
#define TS_GET_SECT_OFF(p) (((p)[0]) + 1)
#define TS_GET_SECT_TBLID(p) (((p)[0]))
#define TS_GET_SECT_LEN(p) (((((p)[1] << 8)|((p)[2])) & 0xFFF) + 3)

int sms1xxx_demux_init(struct sms1xxx_softc *);
void sms1xxx_demux_exit(struct sms1xxx_softc *);
void sms1xxx_demux_put_packet(struct sms1xxx_softc *, u8 *);
void sms1xxx_demux_pesbuf_reset(struct sms1xxx_softc *, int, char *);

#endif
