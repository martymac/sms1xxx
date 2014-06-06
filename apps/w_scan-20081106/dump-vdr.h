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

#ifndef __DUMP_VDR_H__
#define __DUMP_VDR_H__

#include <stdint.h>
#include <linux/dvb/frontend.h>


static const char *inv_name [] = {
	"0",
	"1",
	"999"
};

static const char *fec_name [] = {
	"0",
	"12",
	"23",
	"34",
	"45",
	"56",
	"67",
	"78",
	"89",
	"999"
};

static const char *qam_name [] = {
	"0",
	"16",
	"32",
	"64",
	"128",
	"256",
	"999"
};


static const char *bw_name [] = {
	"8",
	"7",
	"6",
	"999"
};


static const char *mode_name [] = {
	"2",
	"8",
	"999"
};

static const char *guard_name [] = {
	"32",
	"16",
	"8",
	"4",
	"999"
};


static const char *hierarchy_name [] = {
	"0",
	"1",
	"2",
	"4",
	"999"
};

extern
void vdr_dump_dvb_parameters (FILE *f, fe_type_t type,
		struct dvb_frontend_parameters *p,
		char polarity);

extern
void vdr_dump_service_parameter_set (FILE *f,
				 const char *service_name,
				 const char *provider_name,
				 fe_type_t type,
				 struct dvb_frontend_parameters *p,
				 char polarity,
				 int video_pid,
				 int pcr_pid,
				 uint16_t *audio_pid,
				 char audio_lang[][4],
                                 int audio_num,
				 int teletext_pid,
				 int scrambled,
				 int ac3_pid,
                                 int service_id,
				 int original_network_id,
				 int transport_stream_id,
				 int dump_provider,
				 uint16_t *ca_id,
				 int ca_num,
				 int ca_select,
				 int vdr_version,
				 int dump_channum,
				 int channel_num);

#endif

