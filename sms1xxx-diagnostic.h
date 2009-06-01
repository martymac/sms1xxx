/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 7.0 and higher:
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
 *
 * * * *
 *  Diagnostic-related structures and MACROS
 * * * *
 */

#ifndef SMS1XXX_DIAGNOSTIC_H
#define SMS1XXX_DIAGNOSTIC_H

#ifdef DIAGNOSTIC

struct sms1xxx_stats {
    u_int32_t min_wavail;
    u_int32_t max_wavail;
    u_int32_t min_ravail;
    u_int32_t max_ravail;
    u_int32_t threshold;
    u_int32_t interrupts;
    u_int32_t bytes;
    u_int32_t packetsmatched;
};

struct sms1xxx_bufs {
    u_int32_t dvrsize;
    u_int32_t sbufsize;
    u_int32_t threshold;
};

#define SMS1XXX_GET_STATS   _IOR('o',81,struct sms1xxx_stats)
#define SMS1XXX_GETBUFS     _IOR('o',82,struct sms1xxx_bufs)
#define SMS1XXX_SETBUFS     _IOW('o',83,struct sms1xxx_bufs)

#endif

#endif
