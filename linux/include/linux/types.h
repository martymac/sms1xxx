/*
 * types.h
 *
 * Copyright (C) 2009-2014 - Ganaël Laplanche, http://contribs.martymac.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Lesser Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

/* Minimalistic linux/types.h to fit Linux header's includes */

#ifndef _LINUX_TYPES_H
#define _LINUX_TYPES_H

#include <sys/types.h>

typedef uint64_t __u64;
typedef uint32_t __u32;
typedef uint16_t __u16;
typedef uint8_t __u8;

typedef int64_t __s64;
typedef int32_t __s32;
typedef int16_t __s16;
typedef int8_t __s8;

typedef time_t __kernel_time_t;

#endif
