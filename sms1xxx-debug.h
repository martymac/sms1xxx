/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2014, Ganaël Laplanche, http://contribs.martymac.org
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

#ifndef SMS1XXX_DEBUG_H
#define SMS1XXX_DEBUG_H

#include "sms1xxx.h"

/* or-able TRACE values */
#define TRACE_PROBE     0x0001
#define TRACE_MODULE    0x0002
#define TRACE_OPEN      0x0004
#define TRACE_IOCTL     0x0008
#define TRACE_READ      0x0010
#define TRACE_DVR_READ  0x0020
#define TRACE_POLL      0x0040
#define TRACE_USB       0x0080
#define TRACE_USB_FLOW  0x0100
#define TRACE_USB_DUMP  0x0200
#define TRACE_FIRMWARE  0x0400
#define TRACE_FRONTEND  0x0800
#define TRACE_SECT      0x1000
#define TRACE_FILTERS   0x2000
#define TRACE_IR        0x4000
#define TRACE_GPIO      0x8000

#ifdef SMS1XXX_DEBUG
#define TRACE(R, FMT, ...) if(sms1xxxdbg & R) \
    printf("%s: " FMT, __func__ , ##__VA_ARGS__)
#else
#define TRACE(R, FMT, ...)
#endif /* SMS1XXX_DEBUG */

#ifdef SMS1XXX_DEBUG
/* sysctl(9) default debug level */
#ifndef SMS1XXX_DEBUG_DEFAULT_LEVEL
#define SMS1XXX_DEBUG_DEFAULT_LEVEL     0
#endif
void sms1xxx_dump_data(const u8 *buf, u32 len, char prefix);
#endif /* SMS1XXX_DEBUG */

#ifdef SMS1XXX_DIAGNOSTIC
struct sms1xxx_stats {
    u32 min_wavail;
    u32 max_wavail;
    u32 min_ravail;
    u32 max_ravail;
    u32 threshold;
    u32 interrupts;
    u32 bytes;
    u32 packetsmatched;
};
#define SMS1XXX_GET_STATS   _IOR('o',81,struct sms1xxx_stats)
#endif /* SMS1XXX_DIAGNOSTIC */

#endif /* SMS1XXX_DEBUG_H */
