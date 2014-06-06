/*
 * Simple MPEG/DVB parser to achieve network/service information without initial tuning data
 *
 * Copyright (C) 2006, 2007, 2008 Winfried Koehler 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 * Or, point your browser to http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 *
 * The author can be reached at: handygewinnspiel AT gmx DOT de
 *
 * The project's page is http://wirbel.htpc-forum.de/w_scan/index2.html
 */

#ifndef __SCAN_H__
#define __SCAN_H__

#include <stdio.h>
#include <errno.h>

extern int verbosity;

#define dprintf(level, fmt...)			\
	do {					\
		if (level <= verbosity)		\
			fprintf(stderr, fmt);	\
	} while (0)

#define dpprintf(level, fmt, args...) \
	dprintf(level, "%s:%d: " fmt, __FUNCTION__, __LINE__ , ##args)

#define fatal(fmt, args...) do { dpprintf(-1, "FATAL: " fmt , ##args); exit(1); } while(0)
#define error(msg...) dprintf(0, "ERROR: " msg)
#define errorn(msg) dprintf(0, "%s:%d: ERROR: " msg ": %d %m\n", __FUNCTION__, __LINE__, errno)
#define warning(msg...) dprintf(1, "WARNING: " msg)
#define info(msg...) dprintf(2, msg)
#define verbose(msg...) dprintf(3, msg)
#define moreverbose(msg...) dprintf(4, msg)
#define debug(msg...) dpprintf(5, msg)
#define verbosedebug(msg...) dpprintf(6, msg)

#endif

