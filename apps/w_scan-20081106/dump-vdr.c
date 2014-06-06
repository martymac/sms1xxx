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

#include <stdio.h>
#include "dump-vdr.h"
#include <linux/dvb/frontend.h>

/*

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

*/

void vdr_dump_dvb_parameters (FILE *f, fe_type_t type,
		struct dvb_frontend_parameters *p,
		char polarity)
{
	switch (type) {
	case FE_QAM:
		fprintf (f, ":%i:", p->frequency / 1000);
		fprintf (f, "M%s:C:", qam_name[p->u.qam.modulation]);
		fprintf (f, "%i:", p->u.qam.symbol_rate / 1000);
		break;

	case FE_OFDM:
		fprintf (f, ":%i:", p->frequency / 1000);
		fprintf (f, "I%s", inv_name[p->inversion]);
		fprintf (f, "B%s", bw_name[p->u.ofdm.bandwidth]);
		fprintf (f, "C%s", fec_name[p->u.ofdm.code_rate_HP]);
		fprintf (f, "D%s", fec_name[p->u.ofdm.code_rate_LP]);
		fprintf (f, "M%s", qam_name[p->u.ofdm.constellation]);
		fprintf (f, "T%s", mode_name[p->u.ofdm.transmission_mode]);
		fprintf (f, "G%s", guard_name[p->u.ofdm.guard_interval]);
		fprintf (f, "Y%s", hierarchy_name[p->u.ofdm.hierarchy_information]);
		fprintf (f, ":T:27500:");
		break;

	default:
		;
	};
}

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
				 int channel_num)
{
        int i;
	if (((ca_select > 0) || ((ca_select == 0) && (scrambled == 0)))) {
//	if ((video_pid || audio_pid[0]) && ((ca_select > 0) || ((ca_select == 0) && (scrambled == 0)))) {
		if ((dump_channum == 1) && (channel_num > 0))
			fprintf(f, ":@%i\n", channel_num);
		fprintf (f, "%s", service_name);
		if (dump_provider == 1)
			fprintf (f, ";%s", provider_name);
		vdr_dump_dvb_parameters (f, type, p, polarity);
		if ((pcr_pid != video_pid) && (video_pid > 0))
			fprintf (f, "%i+%i:", video_pid, pcr_pid);
		else
			fprintf (f, "%i:", video_pid);
		fprintf (f, "%i", audio_pid[0]);
		if ((vdr_version > 2) && audio_lang && audio_lang[0][0])
			fprintf (f, "=%.4s", audio_lang[0]);
	        for (i = 1; i < audio_num; i++) {
			fprintf (f, ",%i", audio_pid[i]);
			if ((vdr_version > 2) && audio_lang && audio_lang[i][0])
				fprintf (f, "=%.4s", audio_lang[i]);
                        }
		if (ac3_pid)
			fprintf (f, ";%i", ac3_pid);
		if (scrambled == 1) scrambled = ca_select;

// 8468 "German Digital Terrestrial Television | IRT on behalf of the German DVB-T Broadcasts"
// *only* valid for DVB-T in Germany.
//		if (type==FE_OFDM) {
//			original_network_id = 8468;
//		}
                if (vdr_version < 3) {
			original_network_id = 0;
			transport_stream_id = 0;
		}
		fprintf (f, ":%d:", teletext_pid);
		fprintf (f, "%x", ca_id[0]);
	        for (i = 1; i < ca_num; i++) {
			if (ca_id[i] == 0) continue;
                        fprintf (f, ",%x", ca_id[i]);
			}
		fprintf (f, ":%d:%d:%d:0", \
			service_id, \
			(transport_stream_id > 0)?original_network_id:0,
			transport_stream_id);
		fprintf (f, "\n");
	}
}

