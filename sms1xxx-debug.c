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
 *
 * * * *
 *  Debug/trace/diagnostic functions
 * * * *
 */

#include "sms1xxx.h"
#include "sms1xxx-debug.h"

#ifdef SMS1XXX_DEBUG
void
sms1xxx_dump_data(const u8 *buf, u32 len, char prefix)
{
    const char *sep = "";
    u32 i;

    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            printf("%s%c ", sep, prefix);
            sep = "\n";
        }
        else if ((i % 4) == 0)
            printf(" ");
        printf("%02x", buf[i]);
    }
    printf("\n");
    return;
}
#endif
