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
 *
 *  referred standards:
 *
 *    ETSI EN 300 468 v1.7.1
 *    ETSI TR 101 211
 *    ETSI ETR 211
 *    ITU-T H.222.0
 *
 ##############################################################################
 * This is tool is derived from the dvb scan tool,
 * Copyright: Johannes Stezenbach <js@convergence.de> and others, GPLv2 and LGPL
 * (linuxtv-dvb-apps-1.1.0/utils/scan)
 *
 * Differencies:
 * - command line options
 * - detects dvb card automatically
 * - no need for initial tuning data, but therefore up to now no DVB-S support
 * - some adaptions for VDR syntax
 *
 * have phun, wirbel 2006/02/16
 ##############################################################################
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <assert.h>

//#include <linux/dvb/frontend.h>
//#include <linux/videodev.h>
#include <linux/dvb/dmx.h>


#include "list.h"
#include "dump-vdr.h"
#include "dump-xine.h"
#include "dump-dvbscan.h"
#include "dump-kaffeine.h"
#include "scan.h"
#include "atsc_psip_section.h"

static char demux_devname[80];

static struct dvb_frontend_info fe_info = {
	.type = -1
};

uint version = 20081106;
int verbosity = 2;

#define ATSC_VSB	0x01
#define ATSC_QAM	0x02
#define IS_ATSC_VSB(t)	t & ATSC_VSB
#define IS_ATSC_QAM(t)	t & ATSC_QAM

static int long_timeout;
static int tuning_speed = 1;
static int current_tp_only=1;
static int get_other_nits = 0;
static int add_frequencies = 0;
static int vdr_dump_provider;
static int vdr_dump_channum;
static int ATSC_type=ATSC_VSB;
static int no_ATSC_PSIP;
static int ca_select = 1;
static int serv_select = 3;	// 20080106: radio and tv as default (no service/other).
static int vdr_version = 4;
static int qam_no_auto = 0;	// 20060705

static enum fe_spectral_inversion caps_inversion	= INVERSION_AUTO;
static enum fe_code_rate caps_fec 			= FEC_AUTO;
static enum fe_modulation caps_qam			= QAM_AUTO;
static enum fe_modulation this_qam			= QAM_64;
static enum fe_modulation this_atsc			= VSB_8;
static enum fe_transmit_mode caps_transmission_mode 	= TRANSMISSION_MODE_AUTO;
static enum fe_guard_interval caps_guard_interval	= GUARD_INTERVAL_AUTO;
static enum fe_hierarchy caps_hierarchy			= HIERARCHY_AUTO;


enum table_type {
	PAT,
	PMT,
	SDT,
	NIT
};

enum format {
        OUTPUT_VDR,
	OUTPUT_PIDS,
	OUTPUT_XINE,
	OUTPUT_DVBSCAN_TUNING_DATA,
	OUTPUT_KAFFEINE
};
static enum format output_format = OUTPUT_VDR;



enum running_mode {
	RM_NOT_RUNNING = 0x01,
	RM_STARTS_SOON = 0x02,
	RM_PAUSING     = 0x03,
	RM_RUNNING     = 0x04
};

#define AUDIO_CHAN_MAX (32)
#define CA_SYSTEM_ID_MAX (16)

struct service {
	struct list_head list;
	int transport_stream_id;
	int service_id;
	char *provider_name;
	char *service_name;
	uint16_t pmt_pid;
	uint16_t pcr_pid;
	uint16_t video_pid;
	uint16_t audio_pid[AUDIO_CHAN_MAX];
	char audio_lang[AUDIO_CHAN_MAX][4];
	int audio_num;
	uint16_t ca_id[CA_SYSTEM_ID_MAX];
	int ca_num;
	uint16_t teletext_pid;
	uint16_t subtitling_pid;
	uint16_t ac3_pid;
	unsigned int type         : 8;
	unsigned int scrambled	  : 1;
	enum running_mode running;
	void *priv;
	int channel_num;
};

struct transponder {
	struct list_head list;
	struct list_head services;
	int network_id;
	int original_network_id;                /* onid patch by Hartmut Birr */
	int transport_stream_id;
	enum fe_type type;
	struct dvb_frontend_parameters param;
	unsigned int scan_done		  : 1;
	unsigned int last_tuning_failed	  : 1;
	unsigned int other_frequency_flag : 1;	/* DVB-T */
	int n_other_f;
	uint32_t *other_f;			/* DVB-T frequency-list descriptor */
};


struct section_buf {
	struct list_head list;
	const char *dmx_devname;
	unsigned int run_once  : 1;
	unsigned int segmented : 1;	/* segmented by table_id_ext */
	int fd;
	int pid;
	int table_id;
	int table_id_ext;
	int section_version_number;
	uint8_t section_done[32];
	int sectionfilter_done;
	unsigned char buf[1024];
	time_t timeout;
	time_t start_time;
	time_t running_time;
	struct section_buf *next_seg;	/* this is used to handle segmented tables (like NIT-other) */
};

static LIST_HEAD(scanned_transponders);
static LIST_HEAD(new_transponders);
static struct transponder *current_tp;


static void dump_dvb_parameters (FILE *f, struct transponder *p);

static void setup_filter (struct section_buf* s, const char *dmx_devname,
			  int pid, int tid, int tid_ext,
			  int run_once, int segmented, int timeout);
static void add_filter (struct section_buf *s);


/* According to the DVB standards, the combination of network_id and
 * transport_stream_id should be unique, but in real life the satellite
 * operators and broadcasters don't care enough to coordinate
 * the numbering. Thus we identify TPs by frequency (scan handles only
 * one satellite at a time). Further complication: Different NITs on
 * one satellite sometimes list the same TP with slightly different
 * frequencies, so we have to search within some bandwidth.
 */
static struct transponder *alloc_transponder(uint32_t frequency)
{
	struct transponder *tp = calloc(1, sizeof(*tp));

	tp->param.frequency = frequency;
	INIT_LIST_HEAD(&tp->list);
	INIT_LIST_HEAD(&tp->services);
	list_add_tail(&tp->list, &new_transponders);
	return tp;
}

static int is_same_transponder(uint32_t f1, uint32_t f2)
{
	uint32_t diff;
	if (f1 == f2)
		return 1;
	diff = (f1 > f2) ? (f1 - f2) : (f2 - f1);
	//FIXME: use symbolrate etc. to estimate bandwidth
	if (diff < 2000) {
		debug("f1 = %u is same TP as f2 = %u\n", f1, f2);
		return 1;
	}
	return 0;
}

static struct transponder *find_transponder(uint32_t frequency)
{
	struct list_head *pos;
	struct transponder *tp;

	list_for_each(pos, &scanned_transponders) {
		tp = list_entry(pos, struct transponder, list);
		if (is_same_transponder(tp->param.frequency, frequency))
			return tp;
	}
	list_for_each(pos, &new_transponders) {
		tp = list_entry(pos, struct transponder, list);
		if (is_same_transponder(tp->param.frequency, frequency))
			return tp;
	}
	return NULL;
}

static void copy_transponder(struct transponder *d, struct transponder *s)
{
	d->network_id = s->network_id;
	d->original_network_id = s->original_network_id;  /* onid patch by Hartmut Birr */
	d->transport_stream_id = s->transport_stream_id;
	d->type = s->type;
	memcpy(&d->param, &s->param, sizeof(d->param));
	d->scan_done = s->scan_done;
	d->last_tuning_failed = s->last_tuning_failed;
	d->other_frequency_flag = s->other_frequency_flag;
	d->n_other_f = s->n_other_f;
	if (d->n_other_f) {
		d->other_f = calloc(d->n_other_f, sizeof(uint32_t));
		memcpy(d->other_f, s->other_f, d->n_other_f * sizeof(uint32_t));
	} 

	else
		d->other_f = NULL;
}

/* service_ids are guaranteed to be unique within one TP
 * (the DVB standards say theay should be unique within one
 * network, but in real life...)
 */
static struct service *alloc_service(struct transponder *tp, int service_id)
{
	struct service *s = calloc(1, sizeof(*s));
	INIT_LIST_HEAD(&s->list);
	s->service_id = service_id;
	list_add_tail(&s->list, &tp->services);
	return s;
}

static struct service *find_service(struct transponder *tp, int service_id)
{
	struct list_head *pos;
	struct service *s;

	list_for_each(pos, &tp->services) {
		s = list_entry(pos, struct service, list);
		if (s->service_id == service_id)
			return s;
	}
	return NULL;
}


static void parse_ca_identifier_descriptor (const unsigned char *buf,
				     struct service *s) {
  unsigned char len = buf [1];
  int i;

  buf += 2;

  if (len > sizeof(s->ca_id)) {
    len = sizeof(s->ca_id);
    warning("too many CA system ids\n");
    }
  memcpy(s->ca_id, buf, len);
  s->ca_num=0;
  for (i = 0; i < len / sizeof(s->ca_id[0]); i++) {
    int id = ((s->ca_id[i] & 0x00FF) << 8) + ((s->ca_id[i] & 0xFF00) >> 8);
    s->ca_id[i] = id;
    moreverbose("  CA ID     : PID 0x%04x\n", s->ca_id[i]);
    s->ca_num++;
    }
}

static void parse_ca_descriptor (const unsigned char *buf, struct service *s) {
 unsigned char descriptor_length = buf [1];
 int CA_system_ID;
 // int CA_PID;		// not needed for VDR
 int found=0;
 int i;

 buf += 2;

 if (descriptor_length < 4) return;
 CA_system_ID = (buf[0] << 8) | buf[1];

 for (i=0; i<s->ca_num; i++)
   if (s->ca_id[i] == CA_system_ID)
     found++;

 if (!found) {
   if (s->ca_num + 1 >= CA_SYSTEM_ID_MAX)
     warning("TOO MANY CA SYSTEM IDs.\n");
   else {
      moreverbose("  CA ID     : PID 0x%04x\n", CA_system_ID);
      s->ca_id[s->ca_num]=CA_system_ID;
      s->ca_num++;
      }
    } 	
} 


static void parse_iso639_language_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len = buf [1];

	buf += 2;

	if (len >= 4) {
		debug("    LANG=%.3s %d\n", buf, buf[3]);
		memcpy(s->audio_lang[s->audio_num], buf, 3);
#if 0
		/* seems like the audio_type is wrong all over the place */
		//if (buf[3] == 0) -> normal
		if (buf[3] == 1)
			s->audio_lang[s->audio_num][3] = '!'; /* clean effects (no language) */
		else if (buf[3] == 2)
			s->audio_lang[s->audio_num][3] = '?'; /* for the hearing impaired */
		else if (buf[3] == 3)
			s->audio_lang[s->audio_num][3] = '+'; /* visually impaired commentary */
#endif
	}
}

static void parse_network_name_descriptor (const unsigned char *buf, void *dummy)
{
	unsigned char len = buf [1];

	info("Network Name '%.*s'\n", len, buf + 2);
}

static void parse_terrestrial_uk_channel_number (const unsigned char *buf, void *dummy)
{
	int i, n, channel_num, service_id;
	struct list_head *p1, *p2;
	struct transponder *t;
	struct service *s;

	// 32 bits per record
	n = buf[1] / 4;
	if (n < 1)
		return;

	// desc id, desc len, (service id, service number)
	buf += 2;
	for (i = 0; i < n; i++) {
		service_id = (buf[0]<<8)|(buf[1]&0xff);
		channel_num = (buf[2]&0x03<<8)|(buf[3]&0xff);
		debug("Service ID 0x%x has channel number %d ", service_id, channel_num);
		list_for_each(p1, &scanned_transponders) {
			t = list_entry(p1, struct transponder, list);
			list_for_each(p2, &t->services) {
				s = list_entry(p2, struct service, list);
				if (s->service_id == service_id)
					s->channel_num = channel_num;
			}
		}
		buf += 4;
	}
}


static long bcd32_to_cpu (const int b0, const int b1, const int b2, const int b3)
{
	return ((b0 >> 4) & 0x0f) * 10000000 + (b0 & 0x0f) * 1000000 +
	       ((b1 >> 4) & 0x0f) * 100000   + (b1 & 0x0f) * 10000 +
	       ((b2 >> 4) & 0x0f) * 1000     + (b2 & 0x0f) * 100 +
	       ((b3 >> 4) & 0x0f) * 10       + (b3 & 0x0f);
}


static const fe_code_rate_t fec_tab [8] = {
	FEC_AUTO, FEC_1_2, FEC_2_3, FEC_3_4,
	FEC_5_6, FEC_7_8, FEC_NONE, FEC_NONE
};


static const fe_modulation_t qam_tab [7] = {
	QAM_AUTO, QAM_16, QAM_32, QAM_64, QAM_128, QAM_256
};


static void parse_cable_delivery_system_descriptor (const unsigned char *buf,
					     struct transponder *t)
{
	if (!t) {
		warning("cable_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}
	t->type = FE_QAM;

	t->param.frequency = bcd32_to_cpu (buf[2], buf[3], buf[4], buf[5]);
	t->param.frequency *= 100;
	t->param.u.qam.fec_inner = fec_tab[buf[12] & 0x07];
	t->param.u.qam.symbol_rate = 10 * bcd32_to_cpu (buf[9],
							buf[10],
							buf[11],
							buf[12] & 0xf0);
	if ((buf[8] & 0x0f) > 5)
		t->param.u.qam.modulation = QAM_AUTO;
	else
		t->param.u.qam.modulation = qam_tab[buf[8] & 0x0f];
	t->param.inversion = caps_inversion;

	if (verbosity >= 5) {
		debug("0x%#04x/0x%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		if (t->scan_done)
			dprintf(5, " (done)");
		if (t->last_tuning_failed)
			dprintf(5, " (no signal)");
		dprintf(5, "\n");
	}
}


static void parse_terrestrial_delivery_system_descriptor (const unsigned char *buf,
						   struct transponder *t)
{
	static const fe_modulation_t m_tab [] = { QPSK, QAM_16, QAM_64, QAM_AUTO };
	static const fe_code_rate_t ofec_tab [8] = { FEC_1_2, FEC_2_3, FEC_3_4,
					       FEC_5_6, FEC_7_8 };
	struct dvb_ofdm_parameters *o;

	if (!t) {
		warning("terrestrial_delivery_system_descriptor outside transport stream definition (ignored)\n");
		return;
	}
	o = &t->param.u.ofdm;
	t->type = FE_OFDM;

	t->param.frequency = (buf[2] << 24) | (buf[3] << 16);
	t->param.frequency |= (buf[4] << 8) | buf[5];
	t->param.frequency *= 10;
	t->param.inversion = caps_inversion;

	o->bandwidth = BANDWIDTH_8_MHZ + ((buf[6] >> 5) & 0x3);
	o->constellation = m_tab[(buf[7] >> 6) & 0x3];
	o->hierarchy_information = HIERARCHY_NONE + ((buf[7] >> 3) & 0x3);

	if ((buf[7] & 0x7) > 4)
		o->code_rate_HP = FEC_AUTO;
	else
		o->code_rate_HP = ofec_tab [buf[7] & 0x7];

	if (((buf[8] >> 5) & 0x7) > 4)
		o->code_rate_LP = FEC_AUTO;
	else
		o->code_rate_LP = ofec_tab [(buf[8] >> 5) & 0x7];

	o->guard_interval = GUARD_INTERVAL_1_32 + ((buf[8] >> 3) & 0x3);

	o->transmission_mode = (buf[8] & 0x2) ?
			       TRANSMISSION_MODE_8K :
			       TRANSMISSION_MODE_2K;

	t->other_frequency_flag = (buf[8] & 0x01);

	if (verbosity >= 5) {
		debug("0x%#04x/0x%#04x ", t->network_id, t->transport_stream_id);
		dump_dvb_parameters (stderr, t);
		if (t->scan_done)
			dprintf(5, " (done)");
		if (t->last_tuning_failed)
			dprintf(5, " (no signal)");
		dprintf(5, "\n");
	}
}

static void parse_frequency_list_descriptor (const unsigned char *buf,
				      struct transponder *t)
{
	int n, i;
	typeof(*t->other_f) f;

	if (!t) {
		warning("frequency_list_descriptor outside transport stream definition (ignored)\n");
		return;
	}
	if (t->other_f)
		return;

	n = (buf[1] - 1) / 4;
	if (n < 1 || (buf[2] & 0x03) != 3)
		return;

	t->other_f = calloc(n, sizeof(*t->other_f));
	t->n_other_f = n;
	buf += 3;
	for (i = 0; i < n; i++) {
		f = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
		t->other_f[i] = f * 10;
		buf += 4;
	}
}

static void parse_service_descriptor (const unsigned char *buf, struct service *s)
{
	unsigned char len;
	unsigned char *src, *dest;

	s->type = buf[2];

	buf += 3;
	len = *buf;
	buf++;

	if (s->provider_name)
		free (s->provider_name);

	s->provider_name = malloc (len + 1);
	memcpy (s->provider_name, buf, len);
	s->provider_name[len] = '\0';

	/* remove control characters (FIXME: handle short/long name) */
	/* FIXME: handle character set correctly (e.g. via iconv)
	 * c.f. EN 300 468 annex A */
	for (src = dest = (unsigned char *) s->provider_name; *src; src++)
		if (*src >= 0x20 && (*src < 0x80 || *src > 0x9f))
			*dest++ = *src;
	*dest = '\0';
	if (!s->provider_name[0]) {
		/* zap zero length names */
		free (s->provider_name);
		s->provider_name = 0;
	}

	if (s->service_name)
		free (s->service_name);

	buf += len;
	len = *buf;
	buf++;

	s->service_name = malloc (len + 1);
	memcpy (s->service_name, buf, len);
	s->service_name[len] = '\0';

	/* remove control characters (FIXME: handle short/long name) */
	/* FIXME: handle character set correctly (e.g. via iconv)
	 * c.f. EN 300 468 annex A */
	for (src = dest = (unsigned char *) s->service_name; *src; src++)
		if (*src >= 0x20 && (*src < 0x80 || *src > 0x9f))
			*dest++ = *src;
	*dest = '\0';
	if (!s->service_name[0]) {
		/* zap zero length names */
		free (s->service_name);
		s->service_name = 0;
	}

	info("     %s(%s)\n",
	    s->service_name, s->provider_name);
}

static int find_descriptor(uint8_t tag, const unsigned char *buf,
		int descriptors_loop_len,
		const unsigned char **desc, int *desc_len)
{
	while (descriptors_loop_len > 0) {
		unsigned char descriptor_tag = buf[0];
		unsigned char descriptor_len = buf[1] + 2;

		if (!descriptor_len) {
			warning("descriptor_tag == 0x%02x, len is 0\n", descriptor_tag);
			break;
		}

		if (tag == descriptor_tag) {
			if (desc)
				*desc = buf;
			if (desc_len)
				*desc_len = descriptor_len;
			return 1;
		}

		buf += descriptor_len;
		descriptors_loop_len -= descriptor_len;
	}
	return 0;
}

static void parse_descriptors(enum table_type t, const unsigned char *buf,
			      int descriptors_loop_len, void *data)
{
	while (descriptors_loop_len > 0) {
		unsigned char descriptor_tag = buf[0];
		unsigned char descriptor_len = buf[1] + 2;

		if (!descriptor_len) {
			warning("descriptor_tag == 0x%02x, len is 0\n", descriptor_tag);
			break;
		}

		switch (descriptor_tag) {
		case 0x00:	break;		/* 0x00 MHP application_descriptor */
		case 0x01:	break;		/* 0x01 MHP application_name_desriptor */
		case 0x02:	break;		/* 0x02 MHP transport_protocol_descriptor */
		case 0x03:	break;		/* 0x03 dvb_j_application_descriptor() */
		case 0x04:	break;		/* 0x04 dvb_j_application_location_descriptor */
		case 0x09:			/* 0x09 ca_descriptor, 20080106 */
			if (t == PMT)
				parse_ca_descriptor (buf, data);	
			break;	
		case 0x0A:			/* 0x0A iso_639_language_descriptor */
			if (t == PMT)
				parse_iso639_language_descriptor (buf, data);
			break;
		case 0x0B:	break;		/* 0x0B application_icons_descriptor */
		case 0x13:	break;		/* 0x13 carousel_identifier_descriptor, to do */
		case 0x40:			/* 0x40 network_name_descriptor */
			if (t == NIT)
				parse_network_name_descriptor (buf, data);
			break;
		case 0x41:	break;		/* 0x41 service_list_descriptor */
		case 0x42:	break;		/* 0x42 stuffing_descriptor */
		case 0x43:	break;		/* 0x43 satellite_delivery_system_descriptor */
		case 0x44:			/* 0x44 cable_delivery_system_descriptor */
			if (t == NIT)
				parse_cable_delivery_system_descriptor (buf, data);
			break;
		case 0x45:			/* 0x45 vbi_data_descriptor */
		case 0x46:			/* 0x46 vbi_teletext_descriptor */
		case 0x47:			/* 0x47 bouquet_name_descriptor */
		case 0x48:			/* 0x48 service_descriptor */
			if (t == SDT)
				parse_service_descriptor (buf, data);
			break;
		case 0x49:	break;		/* 0x49 country_availability_descriptor */
		case 0x4A:	break;		/* 0x4A linkage_descriptor */
		case 0x4B:	break;		/* 0x4B nvod_reference_descriptor */
		case 0x4C:	break;		/* 0x4C time_shifted_service_descriptor */
		case 0x4D:	break;		/* 0x4D short_event_descriptor */
		case 0x4E:	break;		/* 0x4E extended_event_descriptor */
		case 0x4F:	break;		/* 0x4F time_shifted_event_descriptor */
		case 0x50:	break;		/* 0x50 component_descriptor */
		case 0x51:	break;		/* 0x51 mosaic_descriptor */
		case 0x52:	break;		/* 0x52 stream_identifier_descriptor */
		case 0x53:			/* 0x53 ca_identifier_descriptor */
			if (t == SDT)
				parse_ca_identifier_descriptor (buf, data);
			break;
		case 0x54:	break;		/* 0x54 content_descriptor */
		case 0x55:	break;		/* 0x55 parental_rating_descriptor */
		case 0x56:	break;		/* 0x56 teletext_descriptor */
		case 0x57:	break;		/* 0x57 telephone_descriptor */
		case 0x58:	break;		/* 0x58 local_time_offset_descriptor */
		case 0x59:	break;		/* 0x59 subtitling_descriptor */
		case 0x5A:			/* 0x5A terrestrial_delivery_system_descriptor */
			if (t == NIT)
				parse_terrestrial_delivery_system_descriptor (buf, data);
			break;
		case 0x5B:	break;		/* 0x5B multilingual_network_name_descriptor */
		case 0x5C:	break;		/* 0x5C multilingual_bouquet_name_descriptor */
		case 0x5D:	break;		/* 0x5D multilingual_service_name_descriptor */
		case 0x5E:	break;		/* 0x5E multilingual_component_descriptor */
		case 0x5F:	break;		/* 0x5F private_data_specifier_descriptor */
		case 0x60:	break;		/* 0x60 service_move_descriptor */
		case 0x61:	break;		/* 0x61 short_smoothing_buffer_descriptor */
		case 0x62:			/* 0x62 frequency_list_descriptor */
			if (t == NIT)
				parse_frequency_list_descriptor (buf, data);
			break;
		case 0x63:	break;		/* 0x63 partial_transport_stream_descriptor */
		case 0x64:	break;		/* 0x64 data_broadcast_descriptor */
		case 0x65:	break;		/* 0x65 scrambling_descriptor */
		case 0x66:	break;		/* 0x66 data_broadcast_id_descriptor */
		case 0x67:	break;		/* 0x67 transport_stream_descriptor */
		case 0x68:	break;		/* 0x68 dsng_descriptor */
		case 0x69:	break;		/* 0x69 pdc_descriptor */
		case 0x6A:	break;		/* 0x6A ac3_descriptor */
		case 0x6B:	break;		/* 0x6B ancillary_data_descriptor */
		case 0x6C:	break;		/* 0x6C cell_list_descriptor */
		case 0x6D:	break;		/* 0x6D cell_frequency_link_descriptor */
		case 0x6E:	break;		/* 0x6E announcement_support_descriptor */
		case 0x6F:	break;		/* 0x6F application_signalling_descriptor */
		case 0x71:	break;		/* 0x71 service_identifier_descriptor (ETSI TS 102 812, MHP) */
		case 0x72:	break;		/* 0x72 service_availbility_descriptor */
		case 0x73:	break;		/* 0x73 default_authority_descriptor (ETSI TS 102 323) */
		case 0x74:	break;		/* 0x74 related_content_descriptor (ETSI TS 102 323) */
		case 0x75:	break;		/* 0x75 tva_id_descriptor (ETSI TS 102 323) */
		case 0x76:	break;		/* 0x76 content_identifier_descriptor (ETSI TS 102 323) */
		case 0x77:	break;		/* 0x77 time_slice_fec_identifier_descriptor (ETSI EN 301 192) */
		case 0x78:	break;		/* 0x78 ecm_repetition_rate_descriptor (ETSI EN 301 192) */
		case 0x79:	break;		/* 0x79 s2_satellite_delivery_system_descriptor */
		case 0x7A:	break;		/* 0x7A enhanced_ac3_descriptor */
		case 0x7B:	break;		/* 0x7B dts_descriptor */
		case 0x7C:	break;		/* 0x7C aac_descriptor */
		case 0x7F:	break;		/* 0x7F extension_descriptor */		
		case 0x83:
			/* 0x83 is in the privately defined range of descriptor tags,
			 * so we parse this only if the user says so to avoid
			 * problems when 0x83 is something entirely different... */
			if (t == NIT && vdr_dump_channum)
				parse_terrestrial_uk_channel_number (buf, data);
			break;
		case 0xF2:	break;		// 0xF2 Private DVB Descriptor  Premiere.de, Content Transmission Descriptor

		default:
			verbosedebug("skip descriptor 0x%02x\n", descriptor_tag);
		};

		buf += descriptor_len;
		descriptors_loop_len -= descriptor_len;
	}
}


static void parse_pat(const unsigned char *buf, int section_length,
		      int transport_stream_id)
{
	while (section_length > 0) {
		struct service *s;
		int service_id = (buf[0] << 8) | buf[1];

		if (service_id == 0) {
			verbosedebug ("skipping %02x %02x %02x %02x (service_id == 0)\n", buf[0],buf[1],buf[2],buf[3]);
			buf += 4;		/*  skip nit pid entry... */
			section_length -= 4;
			continue;
		}
		/* SDT might have been parsed first... */
		s = find_service(current_tp, service_id);
		if (!s)
			s = alloc_service(current_tp, service_id);
		s->pmt_pid = ((buf[2] & 0x1f) << 8) | buf[3];
		if (!s->priv && s->pmt_pid) {
			s->priv = malloc(sizeof(struct section_buf));
			setup_filter(s->priv, demux_devname,
				     s->pmt_pid, 0x02, -1, 1, 0, 5);

			add_filter (s->priv);
		}

		buf += 4;
		section_length -= 4;
	};
}


static void parse_pmt (const unsigned char *buf, int section_length, int service_id)
{
	int program_info_len;
	struct service *s;
        char msg_buf[14 * AUDIO_CHAN_MAX + 1];
        char *tmp;
        int i;

	s = find_service (current_tp, service_id);
	if (!s) {
		error("PMT for service_id 0x%04x was not in PAT\n", service_id);
		return;
	}

	s->pcr_pid = ((buf[0] & 0x1f) << 8) | buf[1];
	program_info_len = ((buf[2] & 0x0f) << 8) | buf[3];

	// 20080106, search PMT program info for CA Ids
	buf +=4;
	section_length -= 4;

	while (program_info_len > 0) {
		int descriptor_length = ((int)buf[1]) + 2;
		parse_descriptors(PMT, buf, section_length, s);
		buf += descriptor_length;
		section_length   -= descriptor_length;
		program_info_len -= descriptor_length;
		}

	while (section_length > 0) {
		int ES_info_len = ((buf[3] & 0x0f) << 8) | buf[4];
		int elementary_pid = ((buf[1] & 0x1f) << 8) | buf[2];

		switch (buf[0]) { // das ist stream type
		case 0x01:// STREAMTYPE ISO/IEC 11172 Video
		case 0x02:// STREAMTYPE ITU-T Rec. H.262 | ISO/IEC 13818-2 Video | ISO/IEC 11172-2 constr. parameter video stream
			moreverbose("  VIDEO     : PID 0x%04x\n", elementary_pid);
			if (s->video_pid == 0)
				s->video_pid = elementary_pid;
			break;
		case 0x03:// STREAMTYPE_11172_AUDIO
		case 0x04:// STREAMTYPE_13818-3_AUDIO
			moreverbose("  AUDIO     : PID 0x%04x\n", elementary_pid);
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = elementary_pid;
				parse_descriptors (PMT, buf + 5, ES_info_len, s);
				s->audio_num++;
			}
			else
				warning("more than %i audio channels, truncating\n",
				     AUDIO_CHAN_MAX);
			break;
		case 0x05:// STREAMTYPE_13818_PRIVATE (ITU-T Rec. H.222.0 | ISO/IEC 13818-1 private sections)
		case 0x06:// STREAMTYPE_13818_PES_PRIVATE (ITU-T Rec. H.222.0 | ISO/IEC 13818-1 PES packets containing private data)
                   /* ITU-T Rec. H.222.0 | ISO/IEC 13818-1 PES packets containing private data*/

			if (find_descriptor(0x56, buf + 5, ES_info_len, NULL, NULL)) {
				moreverbose("  TELETEXT  : PID 0x%04x\n", elementary_pid);
				s->teletext_pid = elementary_pid;
				break;
			}
			else if (find_descriptor(0x59, buf + 5, ES_info_len, NULL, NULL)) {
				/* Note: The subtitling descriptor can also signal
				 * teletext subtitling, but then the teletext descriptor
				 * will also be present; so we can be quite confident
				 * that we catch DVB subtitling streams only here, w/o
				 * parsing the descriptor. */
				moreverbose("  SUBTITLING: PID 0x%04x\n", elementary_pid);
				s->subtitling_pid = elementary_pid;
				break;
			}
			else if (find_descriptor(0x6a, buf + 5, ES_info_len, NULL, NULL)) {
				moreverbose("  AC3       : PID 0x%04x\n", elementary_pid);
				s->ac3_pid = elementary_pid;
				break;
			}
                        /* we shouldn't reach this one, usually it should be Teletext, Subtitling or AC3 .. */
                        moreverbose("  unknown private data: PID 0x%04x\n", elementary_pid);
                        break;
 		case 0x07://ISO/IEC 13512 MHEG
			/*
			MHEG-5, or ISO/IEC 13522-5, is part of a set of international standards relating to the
			presentation of multimedia information, standardized by the Multimedia and Hypermedia Experts Group (MHEG).
			It is most commonly used as a language to describe interactive television services. */
 			moreverbose("  MHEG      : PID 0x%04x\n", elementary_pid);
 			break;
		case 0x08://ITU-T Rec. H.222.0 | ISO/IEC 13818-1 Annex A  DSM CC
 			moreverbose("  DSM CC    : PID 0x%04x\n", elementary_pid);
			break;
		case 0x09://ITU-T Rec. H.222.0 | ISO/IEC 13818-1/11172-1 auxiliary
 			moreverbose("  ITU-T Rec. H.222.0 | ISO/IEC 13818-1/11172-1 auxiliary : PID 0x%04x\n", elementary_pid);
			break;
		case 0x0A://ISO/IEC 13818-6 Multiprotocol encapsulation
 			moreverbose("  ISO/IEC 13818-6 Multiprotocol encapsulation    : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x0B:
                        /*
			Digital storage media command and control (DSM-CC) is a toolkit for control channels associated
			with MPEG-1 and MPEG-2 streams. It is defined in part 6 of the MPEG-2 standard (Extensions for DSM-CC).
			DSM-CC may be used for controlling the video reception, providing features normally found
			on VCR (fast-forward, rewind, pause, etc). It may also be used for a wide variety of other purposes
			including packet data transport. MPEG-2 ISO/IEC 13818-6 (part 6 of the MPEG-2 standard).

			DSM-CC defines or extends five distinct protocols:
			* User-User 
			* User-Network 
			* MPEG transport profiles (profiles to the standard MPEG transport protocol ISO/IEC 13818-1 to allow
				transmission of event, synchronization, download, and other information in the MPEG transport stream)
			* Download 
			* Switched Digital Broadcast-Channel Change Protocol (SDB/CCP)
				Enables a client to remotely switch from channel to channel in a broadcast environment.
				Used to attach a client to a continuous-feed session (CFS) or other broadcast feed. Sometimes used in pay-per-view.
			*/
 			moreverbose("  DSM-CC U-N Messages : PID 0x%04x\n", elementary_pid);
 			break;
		case 0x0C://ISO/IEC 13818-6 Stream Descriptors
 			moreverbose("  ISO/IEC 13818-6 Stream Descriptors : PID 0x%04x\n", elementary_pid);
			break;
		case 0x0D://ISO/IEC 13818-6 Sections (any type, including private data)
 			moreverbose("  ISO/IEC 13818-6 Sections (any type, including private data) : PID 0x%04x\n", elementary_pid);
			break;
		case 0x0E://ISO/IEC 13818-1 auxiliary
 			moreverbose("  ISO/IEC 13818-1 auxiliary : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x0F:
			moreverbose("  ADTS Audio Stream (usually AAC) : PID 0x%04x\n", elementary_pid);
			if (output_format == OUTPUT_VDR) break; /* not supported by VDR up to now. */
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = elementary_pid;
				parse_descriptors (PMT, buf + 5, ES_info_len, s);
				s->audio_num++;
			}
			else
				warning("more than %i audio channels, truncating\n",
				     AUDIO_CHAN_MAX);
			break;
		case 0x10://ISO/IEC 14496-2 Visual
 			moreverbose("  ISO/IEC 14496-2 Visual : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x11:
			moreverbose("  ISO/IEC 14496-3 Audio with LATM transport syntax as def. in ISO/IEC 14496-3/AMD1 : PID 0x%04x\n", elementary_pid);
			if (output_format == OUTPUT_VDR) break; /* not supported by VDR up to now. */
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = elementary_pid;
				parse_descriptors (PMT, buf + 5, ES_info_len, s);
				s->audio_num++;
			}
			else
				warning("more than %i audio channels, truncating\n",
				     AUDIO_CHAN_MAX);
			break;
 		case 0x12:
			moreverbose("  ISO/IEC 14496-1 SL-packetized stream or FlexMux stream carried in PES packets : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x13:
			moreverbose("  ISO/IEC 14496-1 SL-packetized stream or FlexMux stream carried in ISO/IEC 14496 sections : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x14:
			moreverbose("  ISO/IEC 13818-6 DSM-CC synchronized download protocol : PID 0x%04x\n", elementary_pid);
			break;
		case 0x15:
			moreverbose("  Metadata carried in PES packets using the Metadata Access Unit Wrapper : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x16:
			moreverbose("  Metadata carried in metadata_sections : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x17:
			moreverbose("  Metadata carried in ISO/IEC 13818-6 (DSM-CC) Data Carousel : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x18:
			moreverbose("  Metadata carried in ISO/IEC 13818-6 (DSM-CC) Object Carousel : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x19:
			moreverbose("  Metadata carried in ISO/IEC 13818-6 Synchronized Download Protocol using the Metadata Access Unit Wrapper : PID 0x%04x\n", elementary_pid);
			break;
 		case 0x1A:
			moreverbose("  IPMP stream (defined in ISO/IEC 13818-11, MPEG-2 IPMP) : PID 0x%04x\n", elementary_pid);
			break;
		case 0x1B:
			moreverbose("  AVC Video stream, ITU-T Rec. H.264 | ISO/IEC 14496-10 : PID 0x%04x\n", elementary_pid);
			if (s->video_pid == 0)
				s->video_pid = elementary_pid;
			break;
 		case 0x81:
			moreverbose("  Audio per ATSC A/53B [2] Annex B : PID 0x%04x\n", elementary_pid);
			if (output_format == OUTPUT_VDR) break; /* not supported by VDR up to now. */
			if (s->audio_num < AUDIO_CHAN_MAX) {
				s->audio_pid[s->audio_num] = elementary_pid;
				parse_descriptors (PMT, buf + 5, ES_info_len, s);
				s->audio_num++;
			}
			else
				warning("more than %i audio channels, truncating\n",
				     AUDIO_CHAN_MAX);
			break;
		default:
			moreverbose("  OTHER     : PID 0x%04x TYPE 0x%02x\n", elementary_pid, buf[0]);
		};

		buf += ES_info_len + 5;
		section_length -= ES_info_len + 5;
	};


        tmp = msg_buf;
        tmp += sprintf(tmp, "0x%04x (%.4s)", s->audio_pid[0], s->audio_lang[0]);

	if (s->audio_num > AUDIO_CHAN_MAX) {
		warning("more than %i audio channels: %i, truncating to %i\n",
		      AUDIO_CHAN_MAX, s->audio_num, AUDIO_CHAN_MAX);
		s->audio_num = AUDIO_CHAN_MAX;
	}

        for (i=1; i<s->audio_num; i++)
                tmp += sprintf(tmp, ", 0x%04x (%.4s)", s->audio_pid[i], s->audio_lang[i]);

        debug("0x%04x 0x%04x: %s -- %s, pmt_pid 0x%04x, vpid 0x%04x, apid %s\n",
	    s->transport_stream_id,
	    s->service_id,
	    s->provider_name, s->service_name,
	    s->pmt_pid, s->video_pid, msg_buf);
}


static void parse_nit (const unsigned char *buf, int section_length, int network_id)
{
	int descriptors_loop_len = ((buf[0] & 0x0f) << 8) | buf[1];

	if (section_length < descriptors_loop_len + 4)
	{
		warning("section too short: network_id == 0x%04x, section_length == %i, "
		     "descriptors_loop_len == %i\n",
		     network_id, section_length, descriptors_loop_len);
		return;
	}

	parse_descriptors (NIT, buf + 2, descriptors_loop_len, NULL);

	section_length -= descriptors_loop_len + 4;
	buf += descriptors_loop_len + 4;

	while (section_length > 6) {
		int transport_stream_id = (buf[0] << 8) | buf[1];
		struct transponder *t, tn;

		descriptors_loop_len = ((buf[4] & 0x0f) << 8) | buf[5];

		if (section_length < descriptors_loop_len + 4)
		{
			warning("section too short: transport_stream_id == 0x%04x, "
			     "section_length == %i, descriptors_loop_len == %i\n",
			     transport_stream_id, section_length,
			     descriptors_loop_len);
			break;
		}

		debug("transport_stream_id 0x%04x\n", transport_stream_id);

		memset(&tn, 0, sizeof(tn));
		tn.type = -1;
		tn.network_id = network_id;
		tn.original_network_id = (buf[2] << 8) | buf[3];	/* onid patch by Hartmut Birr */
		tn.transport_stream_id = transport_stream_id;

		parse_descriptors (NIT, buf + 6, descriptors_loop_len, &tn);

		if (tn.type == fe_info.type) {
			/* only add if develivery_descriptor matches FE type */
			t = find_transponder(tn.param.frequency);
			if ((!t) && (add_frequencies)) {
				info("found new transponder (%d)\n",tn.param.frequency/1000);
				t = alloc_transponder(tn.param.frequency);
				}
			if (t) {
				info("copying transponder info (%d)\n", tn.param.frequency/1000);
				copy_transponder(t, &tn);
				}
		}

		section_length -= descriptors_loop_len + 6;
		buf += descriptors_loop_len + 6;
	};
}


static void parse_sdt (const unsigned char *buf, int section_length,
		int transport_stream_id)
{
	buf += 3;	       /*  skip original network id + reserved field */

	while (section_length > 4) {
		int service_id = (buf[0] << 8) | buf[1];
		int descriptors_loop_len = ((buf[3] & 0x0f) << 8) | buf[4];
		struct service *s;

		if (section_length < descriptors_loop_len || !descriptors_loop_len)
		{
			warning("section too short: service_id == 0x%02x, section_length == %i, "
			     "descriptors_loop_len == %i\n",
			     service_id, section_length,
			     descriptors_loop_len);
			break;
		}

		s = find_service(current_tp, service_id);
		if (!s)
			/* maybe PAT has not yet been parsed... */
			s = alloc_service(current_tp, service_id);

		s->running = (buf[3] >> 5) & 0x7;
		s->scrambled = (buf[3] >> 4) & 1;

		parse_descriptors (SDT, buf + 5, descriptors_loop_len, s);

		section_length -= descriptors_loop_len + 5;
		buf += descriptors_loop_len + 5;
	};
}

/* ATSC PSIP VCT */
static void parse_atsc_service_loc_desc(struct service *s,const unsigned char *buf)
{
	struct ATSC_service_location_descriptor d = read_ATSC_service_location_descriptor(buf);
	int i;
	unsigned char *b = (unsigned char *) buf+5;

	s->pcr_pid = d.PCR_PID;
	for (i=0; i < d.number_elements; i++) {
		struct ATSC_service_location_element e = read_ATSC_service_location_element(b);
		switch (e.stream_type) {
			case 0x02: /* video */
				s->video_pid = e.elementary_PID;
				moreverbose("  VIDEO     : PID 0x%04x\n", e.elementary_PID);
				break;
			case 0x81: /* ATSC audio */
				if (s->audio_num < AUDIO_CHAN_MAX) {
					s->audio_pid[s->audio_num] = e.elementary_PID;
					s->audio_lang[s->audio_num][0] = (e.ISO_639_language_code >> 16) & 0xff;
					s->audio_lang[s->audio_num][1] = (e.ISO_639_language_code >> 8)  & 0xff;
					s->audio_lang[s->audio_num][2] =  e.ISO_639_language_code        & 0xff;
					s->audio_num++;
				}
				moreverbose("  AUDIO     : PID 0x%04x lang: %s\n",e.elementary_PID,s->audio_lang[s->audio_num-1]);

				break;
			default:
				warning("unhandled stream_type: %x\n",e.stream_type);
				break;
		};
		b += 6;
	}
}

static void parse_atsc_ext_chan_name_desc(struct service *s,const unsigned char *buf)
{
	unsigned char *b = (unsigned char *) buf+2;
	int i,j;
	int num_str = b[0];

	b++;
	for (i = 0; i < num_str; i++) {
		int num_seg = b[3];
		b += 4; /* skip lang code */
		for (j = 0; j < num_seg; j++) {
			int comp_type = b[0],/* mode = b[1],*/ num_bytes = b[2];

			switch (comp_type) {
				case 0x00:
					if (s->service_name)
						free(s->service_name);
					s->service_name = malloc(num_bytes * sizeof(char) + 1);
					memcpy(s->service_name,&b[3],num_bytes);
					s->service_name[num_bytes] = '\0';
					break;
				default:
					warning("compressed strings are not supported yet\n");
					break;
			}
			b += 3 + num_bytes;
		}
	}
}

static void parse_psip_descriptors(struct service *s,const unsigned char *buf,int len)
{
	unsigned char *b = (unsigned char *) buf;
	int desc_len;
	while (len > 0) {
		desc_len = b[1];
		switch (b[0]) {
			case ATSC_SERVICE_LOCATION_DESCRIPTOR_ID:
				parse_atsc_service_loc_desc(s,b);
				break;
			case ATSC_EXTENDED_CHANNEL_NAME_DESCRIPTOR_ID:
				parse_atsc_ext_chan_name_desc(s,b);
				break;
			default:
				warning("unhandled psip descriptor: %02x\n",b[0]);
				break;
		}
		b += 2 + desc_len;
		len -= 2 + desc_len;
	}
}

static void parse_psip_vct (const unsigned char *buf, int section_length,
		int table_id, int transport_stream_id)
{
	(void)section_length;
	(void)table_id;
	(void)transport_stream_id;

	int num_channels_in_section = buf[1];
	int i;
	int pseudo_id = 0xffff;
	unsigned char *b = (unsigned char *) buf + 2;

	for (i = 0; i < num_channels_in_section; i++) {
		struct service *s;
		struct tvct_channel ch = read_tvct_channel(b);

		switch (ch.service_type) {
			case 0x01:
				info("analog channels won't be put info channels.conf\n");
				break;
			case 0x02: /* ATSC TV */
			case 0x03: /* ATSC Radio */
				break;
			case 0x04: /* ATSC Data */
			default:
				continue;
		}

		if (ch.program_number == 0)
			ch.program_number = --pseudo_id;

		s = find_service(current_tp, ch.program_number);
		if (!s)
			s = alloc_service(current_tp, ch.program_number);

		if (s->service_name)
			free(s->service_name);

		s->service_name = malloc(7*sizeof(unsigned char));
		/* TODO find a better solution to convert UTF-16 */
		s->service_name[0] = ch.short_name0;
		s->service_name[1] = ch.short_name1;
		s->service_name[2] = ch.short_name2;
		s->service_name[3] = ch.short_name3;
		s->service_name[4] = ch.short_name4;
		s->service_name[5] = ch.short_name5;
		s->service_name[6] = ch.short_name6;

		parse_psip_descriptors(s,&b[32],ch.descriptors_length);

		s->channel_num = ch.major_channel_number << 10 | ch.minor_channel_number;

		if (ch.hidden) {
			s->running = RM_NOT_RUNNING;
			info("service is not running, pseudo program_number.");
		} else {
			s->running = RM_RUNNING;
			info("service is running.");
		}

		info(" Channel number: %d:%d. Name: '%s'\n",
			ch.major_channel_number, ch.minor_channel_number,s->service_name);

		b += 32 + ch.descriptors_length;
	}
}

static int get_bit (uint8_t *bitfield, int bit)
{
	return (bitfield[bit/8] >> (bit % 8)) & 1;
}

static void set_bit (uint8_t *bitfield, int bit)
{
	bitfield[bit/8] |= 1 << (bit % 8);
}


/**
 *   returns 0 when more sections are expected
 *	   1 when all sections are read on this pid
 *	   -1 on invalid table id
 */
static int parse_section (struct section_buf *s)
{
	const unsigned char *buf = s->buf;
	int table_id;
	int section_syntax_indicator;
	int section_length;
	int table_id_ext;
	int section_version_number;
	int current_next_indicator;
	int section_number;
	int last_section_number;
	int pcr_pid;
	int program_info_length;
	int i;

	table_id = buf[0];
	if (s->table_id != table_id)
		return -1;
	section_syntax_indicator = buf[1] & 0x80;
	section_length = (((buf[1] & 0x0f) << 8) | buf[2]) - 11;
	table_id_ext = (buf[3] << 8) | buf[4];				// p.program_number
	section_version_number = (buf[5] >> 1) & 0x1f;			// p.version_number = getBits (b, 0, 42, 5); -> 40 + 1 -> 5 bit weit? -> version_number = buf[5] & 0x3e;
	current_next_indicator = buf[5] & 0x01;
	section_number = buf[6];
	last_section_number = buf[7];
	pcr_pid = ((buf[8] & 0x1f) << 8) | buf[9];
	program_info_length = ((buf[10] & 0x0f) << 8) | buf[11];

	if (s->segmented && s->table_id_ext != -1 && s->table_id_ext != table_id_ext) {
		/* find or allocate actual section_buf matching table_id_ext */
		while (s->next_seg) {
			s = s->next_seg;
			if (s->table_id_ext == table_id_ext)
				break;
		}
		if (s->table_id_ext != table_id_ext) {
			assert(s->next_seg == NULL);
			s->next_seg = calloc(1, sizeof(struct section_buf));
			s->next_seg->segmented = s->segmented;
			s->next_seg->run_once = s->run_once;
			s->next_seg->timeout = s->timeout;
			s = s->next_seg;
			s->table_id = table_id;
			s->table_id_ext = table_id_ext;
			s->section_version_number = section_version_number;
		}
	}

	if (s->section_version_number != section_version_number ||
			s->table_id_ext != table_id_ext) {
		struct section_buf *next_seg = s->next_seg;

		if (s->section_version_number != -1 && s->table_id_ext != -1)
			debug("section version_number or table_id_ext changed "
				"%d -> %d / %04x -> %04x\n",
				s->section_version_number, section_version_number,
				s->table_id_ext, table_id_ext);
		s->table_id_ext = table_id_ext;
		s->section_version_number = section_version_number;
		s->sectionfilter_done = 0;
		memset (s->section_done, 0, sizeof(s->section_done));
		s->next_seg = next_seg;
	}

	buf += 8;

	if (!get_bit(s->section_done, section_number)) {
		set_bit (s->section_done, section_number);

		debug("pid 0x%02x tid 0x%02x table_id_ext 0x%04x, "
		    "%i/%i (version %i)\n",
		    s->pid, table_id, table_id_ext, section_number,
		    last_section_number, section_version_number);

		switch (table_id) {
		case 0x00:
			verbose("PAT\n");
			parse_pat (buf, section_length, table_id_ext);
			break;

		case 0x02:
			verbose("PMT 0x%04x for service 0x%04x\n", s->pid, table_id_ext);
			parse_pmt (buf, section_length, table_id_ext);
			break;
		case 0x41:
			verbose("////////////////////////////////////////////// NIT other\n");
		case 0x40:
			verbose("NIT (%s TS)\n", table_id == 0x40 ? "actual":"other");
			parse_nit (buf, section_length, table_id_ext);
			break;

		case 0x42:
		case 0x46:
			verbose("SDT (%s TS)\n", table_id == 0x42 ? "actual":"other");
			parse_sdt (buf, section_length, table_id_ext);
			break;
		case 0xC8:
		case 0xC9:
			verbose("ATSC VCT\n");
			parse_psip_vct(buf, section_length, table_id, table_id_ext);
			break;
		default:
			;
		};

		for (i = 0; i <= last_section_number; i++)
			if (get_bit (s->section_done, i) == 0)
				break;

		if (i > last_section_number)
			s->sectionfilter_done = 1;
	}

	if (s->segmented) {
		/* always wait for timeout; this is because we don't now how
		 * many segments there are
		 */
		return 0;
	}
	else if (s->sectionfilter_done)
		return 1;

	return 0;
}


static int read_sections (struct section_buf *s)
{
	int section_length, count;

	if (s->sectionfilter_done && !s->segmented)
		return 1;

	/* the section filter API guarantess that we get one full section
	 * per read(), provided that the buffer is large enough (it is)
	 */
	if (((count = read (s->fd, s->buf, sizeof(s->buf))) < 0) && errno == EOVERFLOW)
		count = read (s->fd, s->buf, sizeof(s->buf));
	if (count < 0) {
		errorn("read_sections: read error");
		return -1;
	}

	if (count < 4)
		return -1;

	section_length = ((s->buf[1] & 0x0f) << 8) | s->buf[2];

	if (count != section_length + 3)
		return -1;

	if (parse_section(s) == 1)
		return 1;

	return 0;
}


static LIST_HEAD(running_filters);
static LIST_HEAD(waiting_filters);
static int n_running;
// see http://www.linuxtv.org/pipermail/linux-dvb/2005-October/005577.html:
// #define MAX_RUNNING 32
#define MAX_RUNNING 27

static struct pollfd poll_fds[MAX_RUNNING];
static struct section_buf* poll_section_bufs[MAX_RUNNING];


static void setup_filter (struct section_buf* s, const char *dmx_devname,
			  int pid, int tid, int tid_ext,
			  int run_once, int segmented, int timeout)
{
	memset (s, 0, sizeof(struct section_buf));

	s->fd = -1;
	s->dmx_devname = dmx_devname;
	s->pid = pid;
	s->table_id = tid;

	s->run_once = run_once;
	s->segmented = segmented;

	if (long_timeout)
		s->timeout = 5 * timeout;
	else
		s->timeout = timeout;

	s->table_id_ext = tid_ext;
	s->section_version_number = -1;

	INIT_LIST_HEAD (&s->list);
}

static void update_poll_fds(void)
{
	struct list_head *p;
	struct section_buf* s;
	int i;

	memset(poll_section_bufs, 0, sizeof(poll_section_bufs));
	for (i = 0; i < MAX_RUNNING; i++)
		poll_fds[i].fd = -1;
	i = 0;
	list_for_each (p, &running_filters) {
		if (i >= MAX_RUNNING)
			fatal("too many poll_fds\n");
		s = list_entry (p, struct section_buf, list);
		if (s->fd == -1)
			fatal("s->fd == -1 on running_filters\n");
		verbosedebug("poll fd %d\n", s->fd);
		poll_fds[i].fd = s->fd;
		poll_fds[i].events = POLLIN;
		poll_fds[i].revents = 0;
		poll_section_bufs[i] = s;
		i++;
	}
	if (i != n_running)
		fatal("n_running is hosed\n");
}

static int start_filter (struct section_buf* s)
{
	struct dmx_sct_filter_params f;

	if (n_running >= MAX_RUNNING)
		goto err0;
	if ((s->fd = open (s->dmx_devname, O_RDWR)) < 0)
		goto err0;

	verbosedebug("start filter pid 0x%04x table_id 0x%02x\n", s->pid, s->table_id);

	memset(&f, 0, sizeof(f));

	f.pid = (uint16_t) s->pid;

	if (s->table_id < 0x100 && s->table_id > 0) {
		f.filter.filter[0] = (uint8_t) s->table_id;
		f.filter.mask[0]   = 0xff;
	}

	f.timeout = 0;
	f.flags = DMX_IMMEDIATE_START | DMX_CHECK_CRC;

	if (ioctl(s->fd, DMX_SET_FILTER, &f) == -1) {
		errorn ("ioctl DMX_SET_FILTER failed");
		goto err1;
	}

	s->sectionfilter_done = 0;
	time(&s->start_time);

	list_del_init (&s->list);  /* might be in waiting filter list */
	list_add (&s->list, &running_filters);

	n_running++;
	update_poll_fds();

	return 0;

err1:
	ioctl (s->fd, DMX_STOP);
	close (s->fd);
err0:
	return -1;
}


static void stop_filter (struct section_buf *s)
{
	verbosedebug("stop filter pid 0x%04x\n", s->pid);
	ioctl (s->fd, DMX_STOP);
	close (s->fd);
	s->fd = -1;
	list_del (&s->list);
	s->running_time += time(NULL) - s->start_time;

	n_running--;
	update_poll_fds();
}


static void add_filter (struct section_buf *s)
{
	verbosedebug("add filter pid 0x%04x\n", s->pid);
	if (start_filter (s))
		list_add_tail (&s->list, &waiting_filters);
}


static void remove_filter (struct section_buf *s)
{
	verbosedebug("remove filter pid 0x%04x\n", s->pid);
	stop_filter (s);

	while (!list_empty(&waiting_filters)) {
		struct list_head *next = waiting_filters.next;
		s = list_entry (next, struct section_buf, list);
		if (start_filter (s))
			break;
	};
}


static void read_filters (void)
{
	struct section_buf *s;
	int i, n, done;

	n = poll(poll_fds, n_running, 1000);
	if (n == -1)
		errorn("poll");

	for (i = 0; i < n_running; i++) {
		s = poll_section_bufs[i];
		if (!s)
			fatal("poll_section_bufs[%d] is NULL\n", i);
		if (poll_fds[i].revents)
			done = read_sections (s) == 1;
		else
			done = 0; /* timeout */
		if (done || time(NULL) > s->start_time + s->timeout) {
			if (s->run_once) {
				if (done)
					verbosedebug("filter done pid 0x%04x\n", s->pid);
				else
					info("Info: filter timeout pid 0x%04x\n", s->pid);
				remove_filter (s);
			}
		}
	}
}


static int mem_is_zero (const void *mem, int size)
{
	const char *p = mem;
	unsigned long i;

	for (i=0; i<size; i++) {
		if (p[i] != 0x00)
			return 0;
	}

	return 1;
}


static int __tune_to_transponder (int frontend_fd, struct transponder *t, int v)
{
	struct dvb_frontend_parameters p;
	fe_status_t s;
	int i;

	current_tp = t;

	if (mem_is_zero (&t->param, sizeof(struct dvb_frontend_parameters)))
		return -1;

	memcpy (&p, &t->param, sizeof(struct dvb_frontend_parameters));

	if ((verbosity >= 1) && (v > 0)) {
		dprintf(1, "tune to: ");
		dump_dvb_parameters (stderr, t);
		if (t->last_tuning_failed)
			dprintf(1, " (no signal)");
		dprintf(1, "\n");
	}

	if (ioctl(frontend_fd, FE_SET_FRONTEND, &p) == -1) {
		errorn("Setting frontend parameters failed");
		return -1;
	}

	for (i = 0; i < 5*tuning_speed; i++) {
		usleep (200000);

		if (ioctl(frontend_fd, FE_READ_STATUS, &s) == -1) {
			errorn("FE_READ_STATUS failed");
			return -1;
		}

		if (v > 0) verbose(">>> tuning status == 0x%02x\n", s);

		if (s & FE_HAS_LOCK) {
			t->last_tuning_failed = 0;
			return 0;
		}
	}

	if (v > 0)
          info("----------no signal----------\n");
        else 
          info("no signal(0x%02x)\n", s);

	t->last_tuning_failed = 1;

	return -1;
}

static int tune_to_transponder (int frontend_fd, struct transponder *t)
{
	/* move TP from "new" to "scanned" list */
	list_del_init(&t->list);
	list_add_tail(&t->list, &scanned_transponders);
	t->scan_done = 1;

	if (t->type != fe_info.type) {
		/* ignore cable descriptors in sat NIT and vice versa */
		t->last_tuning_failed = 1;
		return -1;
	}

	if (__tune_to_transponder (frontend_fd, t, 1) == 0)
		return 0;

	return __tune_to_transponder (frontend_fd, t, 1);
}


static int tune_to_next_transponder (int frontend_fd)
{
	struct list_head *pos, *tmp;
	struct transponder *t;

	list_for_each_safe(pos, tmp, &new_transponders) {
		t = list_entry (pos, struct transponder, list);
retry:
		if (tune_to_transponder (frontend_fd, t) == 0)
			return 0;
		if (t->other_frequency_flag &&
				t->other_f &&
				t->n_other_f) {
			t->param.frequency = t->other_f[t->n_other_f - 1];
			t->n_other_f--;
			info("retrying with f=%d\n", t->param.frequency);
			goto retry;
		}
	}
	return -1;
}

struct strtab {
	const char *str;
	int val;
};

static int check_frontend (int fd, int verbose) {
   fe_status_t status;
   uint16_t snr, signal;
   uint32_t ber, uncorrected_blocks;
   ioctl(fd, FE_READ_STATUS, &status);
   ioctl(fd, FE_READ_SIGNAL_STRENGTH, &signal);
   ioctl(fd, FE_READ_SNR, &snr);
   ioctl(fd, FE_READ_BER, &ber);
   ioctl(fd, FE_READ_UNCORRECTED_BLOCKS, &uncorrected_blocks);
   if (verbose) {
     info("signal %04x | snr %04x | ber %08x | unc %08x | ", \
		signal, snr, ber, uncorrected_blocks);
     if (status & FE_HAS_LOCK) info("FE_HAS_LOCK");
     info("\n");
     }
   return (status & FE_HAS_LOCK);
}



static int initial_tune (int frontend_fd)
{
unsigned int f, channel, cnt, ret, qam_parm;
struct transponder *t, *ptest;
struct transponder test;
struct dvb_frontend_parameters frontend_parameters;
test.type 				= FE_OFDM;
test.param.inversion 			= caps_inversion;
test.param.u.ofdm.bandwidth	 	= BANDWIDTH_7_MHZ;
test.param.u.ofdm.code_rate_HP 		= caps_fec;
test.param.u.ofdm.code_rate_LP 		= caps_fec;
test.param.u.ofdm.constellation 	= caps_qam;
test.param.u.ofdm.transmission_mode 	= caps_transmission_mode;
test.param.u.ofdm.guard_interval 	= caps_guard_interval;
test.param.u.ofdm.hierarchy_information = caps_hierarchy;
test.param.frequency 			= 177500000;
ptest=&test;
memcpy (&frontend_parameters, &ptest->param, sizeof(struct dvb_frontend_parameters));
if (fe_info.type == FE_OFDM) { // DVB-T
//	for (channel=5; channel <= 12; channel++) {
for (channel=5; channel <= 12; channel++) {
		f=142500000 + channel*7000000;
		test.type 				= FE_OFDM;
		test.param.inversion 			= caps_inversion;
		test.param.u.ofdm.bandwidth	 	= BANDWIDTH_7_MHZ;
		test.param.u.ofdm.code_rate_HP 		= caps_fec;
		test.param.u.ofdm.code_rate_LP 		= caps_fec;
		test.param.u.ofdm.constellation 	= caps_qam;
		test.param.u.ofdm.transmission_mode 	= caps_transmission_mode;
		test.param.u.ofdm.guard_interval 	= caps_guard_interval;
		test.param.u.ofdm.hierarchy_information = caps_hierarchy;
                test.param.frequency 			= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d bw%d", __FUNCTION__,__LINE__,f,7);
			continue;
			}
		usleep (1500000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(200000);
			}
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
                t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.ofdm.bandwidth 	= ptest->param.u.ofdm.bandwidth;
		t->param.u.ofdm.code_rate_HP	= ptest->param.u.ofdm.code_rate_HP;
		t->param.u.ofdm.code_rate_LP	= ptest->param.u.ofdm.code_rate_LP;
		t->param.u.ofdm.constellation	= ptest->param.u.ofdm.constellation;
		t->param.u.ofdm.transmission_mode = ptest->param.u.ofdm.transmission_mode;
		t->param.u.ofdm.guard_interval	= ptest->param.u.ofdm.guard_interval;
		t->param.u.ofdm.hierarchy_information = ptest->param.u.ofdm.hierarchy_information;
		info("signal ok (I%sB%sC%sD%sM%sT%sG%sY%s)\n",
			inv_name[t->param.inversion],
			bw_name[t->param.u.ofdm.bandwidth],
			fec_name[t->param.u.ofdm.code_rate_HP],
			fec_name[t->param.u.ofdm.code_rate_LP],
			qam_name[t->param.u.ofdm.constellation],
			mode_name[t->param.u.ofdm.transmission_mode],
			guard_name[t->param.u.ofdm.guard_interval],
			hierarchy_name[t->param.u.ofdm.hierarchy_information]);
		}	

	for (channel=21; channel<= 69; channel++) {
		f=306000000 + channel*8000000;
		test.type 				= FE_OFDM;
		test.param.inversion 			= caps_inversion;
		test.param.u.ofdm.bandwidth	 	= BANDWIDTH_8_MHZ;
		test.param.u.ofdm.code_rate_HP 		= caps_fec;
		test.param.u.ofdm.code_rate_LP 		= caps_fec;
		test.param.u.ofdm.constellation 	= caps_qam;
		test.param.u.ofdm.transmission_mode 	= caps_transmission_mode;
		test.param.u.ofdm.guard_interval 	= caps_guard_interval;
		test.param.u.ofdm.hierarchy_information = caps_hierarchy;
                test.param.frequency 			= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d bw%d", __FUNCTION__,__LINE__,f,8);
			continue;
			}
		usleep (1500000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(200000);
			}
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
                t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.ofdm.bandwidth 	= ptest->param.u.ofdm.bandwidth;
		t->param.u.ofdm.code_rate_HP	= ptest->param.u.ofdm.code_rate_HP;
		t->param.u.ofdm.code_rate_LP	= ptest->param.u.ofdm.code_rate_LP;
		t->param.u.ofdm.constellation	= ptest->param.u.ofdm.constellation;
		t->param.u.ofdm.transmission_mode = ptest->param.u.ofdm.transmission_mode;
		t->param.u.ofdm.guard_interval	= ptest->param.u.ofdm.guard_interval;
		t->param.u.ofdm.hierarchy_information = ptest->param.u.ofdm.hierarchy_information;
		info("signal ok (I%sB%sC%sD%sM%sT%sG%sY%s)\n",
			inv_name[t->param.inversion],
			bw_name[t->param.u.ofdm.bandwidth],
			fec_name[t->param.u.ofdm.code_rate_HP],
			fec_name[t->param.u.ofdm.code_rate_LP],
			qam_name[t->param.u.ofdm.constellation],
			mode_name[t->param.u.ofdm.transmission_mode],
			guard_name[t->param.u.ofdm.guard_interval],
			hierarchy_name[t->param.u.ofdm.hierarchy_information]);	
		}
	}


if (fe_info.type == FE_ATSC) { // ATSC
    if (IS_ATSC_VSB(ATSC_type)) {
	int base_offset = 0;
	this_atsc = VSB_8;
	for (channel=2; channel <= 69; channel++) {
		if (channel < 5)
			base_offset = 45000000;
		else if (channel < 7)
			base_offset = 49000000;
		else if (channel < 14)
			base_offset = 135000000;
		else
			base_offset = 389000000;

		f=base_offset + channel*6000000;

		test.type 				= FE_ATSC;
		test.param.u.vsb.modulation	 	= this_atsc;
		test.param.frequency 			= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d bw%d", __FUNCTION__,__LINE__,f,7);
			continue;
			}
		usleep (1500000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(200000);
			}
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
                t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.u.vsb.modulation 	= ptest->param.u.vsb.modulation;
		info("signal ok (ch%d VSB)\n",
			channel);
		}
	}
    if (IS_ATSC_QAM(ATSC_type)) {
	int base_offset = 0;
	this_atsc = QAM_256;
	for (channel=2; channel <= 133; channel++) {
		if (channel < 5)
			base_offset = 45000000;
		else if (channel < 7)
			base_offset = 49000000;
		else if (channel < 14)
			base_offset = 135000000;
		else if (channel < 17)
			base_offset = 39012500;
		else if (channel < 23)
			base_offset = 39000000;
		else if (channel < 25)
			base_offset = 81000000;
		else if (channel < 54)
			base_offset = 81012500;
		else if (channel < 95)
			base_offset = 81000000;
		else if (channel < 98)
			base_offset = -477000000;
		else if (channel < 100)
			base_offset = -476987500;
		else
			base_offset = 51000000;
 
		f=base_offset + channel*6000000;

		test.type 				= FE_ATSC;
		test.param.u.vsb.modulation	 	= this_atsc;
		test.param.frequency 			= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d bw%d", __FUNCTION__,__LINE__,f,7);
			continue;
			}
		usleep (1500000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(200000);
			}
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
                t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.u.vsb.modulation 	= ptest->param.u.vsb.modulation;
		info("signal ok (ch%d QAM)\n",
			channel);
		}

	}
    }

if (fe_info.type == FE_QAM) { // DVB-C
  // 20060705 
  for (qam_parm=0; qam_parm<6; qam_parm++) {
	//info ("qam_parm=M%s\n", qam_name[qam_parm]);
	this_qam=caps_qam;
        if (qam_no_auto)
		switch(qam_parm) {
			case 0: continue;
			case 1: continue;
			case 2: continue;			
			case 4: continue;
			default: this_qam=qam_tab[qam_parm];
			}
 
	ptest->type = FE_QAM;
	ptest->param.inversion = caps_inversion;
	ptest->param.u.qam.symbol_rate = 6900000;
	ptest->param.u.qam.fec_inner = FEC_NONE;
	ptest->param.u.qam.modulation = this_qam;		// 20060705 -> fixed
	ptest=&test;
	memcpy (&frontend_parameters, &ptest->param, sizeof(struct dvb_frontend_parameters));
	for (channel=2; channel <= 4; channel++) {
		f=36500000 + channel*7000000;
		test.type 			= FE_QAM;
		test.param.inversion 		= caps_inversion;
		test.param.u.qam.modulation 	= this_qam;
		test.param.u.qam.symbol_rate 	= 6900000;
		test.param.u.qam.fec_inner 	= caps_fec;
                test.param.frequency 		= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6900,this_qam);
			continue;
			}
		usleep (1000000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(100000);
			}
		if (ret == 0) {	// may be unusual symbolrate = 6875 ?
			test.type 			= FE_QAM;
			test.param.inversion 		= caps_inversion;
			test.param.u.qam.modulation 	= this_qam;
			test.param.u.qam.symbol_rate 	= 6875000;
			test.param.u.qam.fec_inner 	= caps_fec;
           	    	test.param.frequency 		= f;
			memcpy (&frontend_parameters, &ptest->param, \
				sizeof(struct dvb_frontend_parameters));
			if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
				dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6875,this_qam);
				continue;
				}
			usleep (1000000);
			for (cnt=0;cnt<5;cnt++) {
				ret = check_frontend(frontend_fd,0);
        			if (ret == 1) break;
				usleep(100000);
				}
			}		
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
		t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.qam.symbol_rate 	= ptest->param.u.qam.symbol_rate;
		t->param.u.qam.fec_inner 	= ptest->param.u.qam.fec_inner;
		t->param.u.qam.modulation 	= ptest->param.u.qam.modulation;
		info("signal ok (S%dC%sM%s)\n",
			t->param.u.qam.symbol_rate/1000,
			fec_name[t->param.u.qam.fec_inner],
			qam_name[t->param.u.qam.modulation]);
		}
	for (channel=1; channel <= 10; channel++) {
		f=100500000 + channel*7000000;
		test.type 			= FE_QAM;
		test.param.inversion 		= caps_inversion;
		test.param.u.qam.modulation 	= this_qam;
		test.param.u.qam.symbol_rate 	= 6900000;
		test.param.u.qam.fec_inner 	= caps_fec;
                test.param.frequency 		= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6900,this_qam);
			continue;
			}
		usleep (1000000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(100000);
			}
		if (ret == 0) {	// may be unusual symbolrate = 6875 ?
			test.type 			= FE_QAM;
			test.param.inversion 		= caps_inversion;
			test.param.u.qam.modulation 	= this_qam;
			test.param.u.qam.symbol_rate 	= 6875000;
			test.param.u.qam.fec_inner 	= caps_fec;
           	    	test.param.frequency 		= f;
			memcpy (&frontend_parameters, &ptest->param, \
				sizeof(struct dvb_frontend_parameters));
			if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
				dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6875,this_qam);
				continue;
				}
			usleep (1000000);
			for (cnt=0;cnt<5;cnt++) {
				ret = check_frontend(frontend_fd,0);
        			if (ret == 1) break;
				usleep(100000);
				}
			}		
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
		t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.qam.symbol_rate 	= ptest->param.u.qam.symbol_rate;
		t->param.u.qam.fec_inner 	= ptest->param.u.qam.fec_inner;
		t->param.u.qam.modulation 	= ptest->param.u.qam.modulation;
		info("signal ok (S%dC%sM%s)\n",
			t->param.u.qam.symbol_rate/1000,
			fec_name[t->param.u.qam.fec_inner],
			qam_name[t->param.u.qam.modulation]);
		}
	for (channel=1; channel <= 9; channel++) {
		f=97000000 + channel*8000000;
		test.type 			= FE_QAM;
		test.param.inversion 		= caps_inversion;
		test.param.u.qam.modulation 	= this_qam;
		test.param.u.qam.symbol_rate 	= 6900000;
		test.param.u.qam.fec_inner 	= caps_fec;
                test.param.frequency 		= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6900,this_qam);
			continue;
			}
		usleep (1000000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(100000);
			}
		if (ret == 0) {	// may be unusual symbolrate = 6875 ?
			test.type 			= FE_QAM;
			test.param.inversion 		= caps_inversion;
			test.param.u.qam.modulation 	= this_qam;
			test.param.u.qam.symbol_rate 	= 6875000;
			test.param.u.qam.fec_inner 	= caps_fec;
           	    	test.param.frequency 		= f;
			memcpy (&frontend_parameters, &ptest->param, \
				sizeof(struct dvb_frontend_parameters));
			if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
				dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6875,this_qam);
				continue;
				}
			usleep (1000000);
			for (cnt=0;cnt<5;cnt++) {
				ret = check_frontend(frontend_fd,0);
        			if (ret == 1) break;
				usleep(100000);
				}
			}		
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
		t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.qam.symbol_rate 	= ptest->param.u.qam.symbol_rate;
		t->param.u.qam.fec_inner 	= ptest->param.u.qam.fec_inner;
		t->param.u.qam.modulation 	= ptest->param.u.qam.modulation;
		info("signal ok (S%dC%sM%s)\n",
			t->param.u.qam.symbol_rate/1000,
			fec_name[t->param.u.qam.fec_inner],
			qam_name[t->param.u.qam.modulation]);
		}
	for (channel=5; channel <= 22; channel++) {
		f=142500000 + channel*7000000;
		test.type 			= FE_QAM;
		test.param.inversion 		= caps_inversion;
		test.param.u.qam.modulation 	= this_qam;
		test.param.u.qam.symbol_rate 	= 6900000;
		test.param.u.qam.fec_inner 	= caps_fec;
                test.param.frequency 		= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6900,this_qam);
			continue;
			}
		usleep (1000000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(100000);
			}
		if (ret == 0) {	// may be unusual symbolrate = 6875 ?
			test.type 			= FE_QAM;
			test.param.inversion 		= caps_inversion;
			test.param.u.qam.modulation 	= this_qam;
			test.param.u.qam.symbol_rate 	= 6875000;
			test.param.u.qam.fec_inner 	= caps_fec;
           	    	test.param.frequency 		= f;
			memcpy (&frontend_parameters, &ptest->param, \
				sizeof(struct dvb_frontend_parameters));
			if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
				dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6875,this_qam);
				continue;
				}
			usleep (1000000);
			for (cnt=0;cnt<5;cnt++) {
				ret = check_frontend(frontend_fd,0);
        			if (ret == 1) break;
				usleep(100000);
				}
			}		
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
		t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.qam.symbol_rate 	= ptest->param.u.qam.symbol_rate;
		t->param.u.qam.fec_inner 	= ptest->param.u.qam.fec_inner;
		t->param.u.qam.modulation 	= ptest->param.u.qam.modulation;
		info("signal ok (S%dC%sM%s)\n",
			t->param.u.qam.symbol_rate/1000,
			fec_name[t->param.u.qam.fec_inner],
			qam_name[t->param.u.qam.modulation]);
		} 
	for (channel=21; channel <= 90; channel++) {        /* debug: channel = (346 - 138) / 8; { */
		f=138000000 + channel*8000000;
		test.type 			= FE_QAM;
		test.param.inversion 		= caps_inversion;
		test.param.u.qam.modulation 	= this_qam;
		test.param.u.qam.symbol_rate 	= 6900000;
		test.param.u.qam.fec_inner 	= caps_fec;
                test.param.frequency 		= f;
		memcpy (&frontend_parameters, &ptest->param, \
			sizeof(struct dvb_frontend_parameters));
		if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
			dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6900,this_qam);
			continue;
			}
		usleep (1000000);
		info("%d: ", frontend_parameters.frequency/1000);
		for (cnt=0;cnt<5;cnt++) {
			ret = check_frontend(frontend_fd,0);
        		if (ret == 1) break;
			usleep(100000);
			}
		if (ret == 0) {	/* may be unusual symbolrate = 6875 ? */
			test.type 			= FE_QAM;
			test.param.inversion 		= caps_inversion;
			test.param.u.qam.modulation 	= this_qam;
			test.param.u.qam.symbol_rate 	= 6875000;
			test.param.u.qam.fec_inner 	= caps_fec;
           	    	test.param.frequency 		= f;
			memcpy (&frontend_parameters, &ptest->param, \
				sizeof(struct dvb_frontend_parameters));
			if (ioctl(frontend_fd, FE_SET_FRONTEND, &frontend_parameters) < 0) {
				dprintf(1, "%s:%d: Setting frontend parameters failed f%d sr%d m%d", __FUNCTION__,__LINE__,f,6875,this_qam);
				continue;
				}
			usleep (1000000);
			for (cnt=0;cnt<5;cnt++) {
				ret = check_frontend(frontend_fd,0);
        			if (ret == 1) break;
				usleep(100000);
				}
			}		
		if (ret == 0) {
			info("\n");
			continue;
			}
		if (__tune_to_transponder (frontend_fd, ptest,0) < 0)
			continue;
		t = alloc_transponder(f);
		t->type 			= ptest->type;
		t->param.inversion 		= ptest->param.inversion;
		t->param.u.qam.symbol_rate 	= ptest->param.u.qam.symbol_rate;
		t->param.u.qam.fec_inner 	= ptest->param.u.qam.fec_inner;
		t->param.u.qam.modulation 	= ptest->param.u.qam.modulation;
		info("signal ok (S%dC%sM%s)\n",
			t->param.u.qam.symbol_rate/1000,
			fec_name[t->param.u.qam.fec_inner],
			qam_name[t->param.u.qam.modulation]);
		}
	  if (caps_qam == QAM_AUTO) break;	// 20060705
	  }
	}
return tune_to_next_transponder(frontend_fd);
}

static void scan_tp_atsc(void)
{
	struct section_buf s0,s1,s2;

	if (no_ATSC_PSIP) {
		setup_filter(&s0, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
		add_filter(&s0);
	} else {
		if (ATSC_type & 0x1) {
			setup_filter(&s0, demux_devname, 0x1ffb, 0xc8, -1, 1, 0, 5); /* terrestrial VCT */
			add_filter(&s0);
		}
		if (ATSC_type & 0x2) {
			setup_filter(&s1, demux_devname, 0x1ffb, 0xc9, -1, 1, 0, 5); /* cable VCT */
			add_filter(&s1);
		}
		setup_filter(&s2, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
		add_filter(&s2);
	}

	do {
		read_filters ();
	} while (!(list_empty(&running_filters) &&
		   list_empty(&waiting_filters)));
}

static void scan_tp_dvb (void)
{
	struct section_buf s0;
	struct section_buf s1;
	struct section_buf s2;
	struct section_buf s3;

	/**
	 *  filter timeouts > min repetition rates specified in ETR211
	 */
	setup_filter (&s0, demux_devname, 0x00, 0x00, -1, 1, 0, 5); /* PAT */
	setup_filter (&s1, demux_devname, 0x11, 0x42, -1, 1, 0, 5); /* SDT */

	add_filter (&s0);
	add_filter (&s1);

	if (!current_tp_only || output_format != OUTPUT_PIDS) {
		setup_filter (&s2, demux_devname, 0x10, 0x40, -1, 1, 0, 15); /* NIT */
		add_filter (&s2);
		if (get_other_nits) {
			/* get NIT-others
			 * Note: There is more than one NIT-other: one per
			 * network, separated by the network_id.
			 */
			setup_filter (&s3, demux_devname, 0x10, 0x41, -1, 1, 1, 15);
			add_filter (&s3);
		}
	}

	do {
		read_filters ();
	} while (!(list_empty(&running_filters) &&
		   list_empty(&waiting_filters)));
}

static void scan_tp(void)
{
	switch(fe_info.type) {
		case FE_QPSK:
		case FE_QAM:
		case FE_OFDM:
			scan_tp_dvb();
			break;
		case FE_ATSC:
			scan_tp_atsc();
			break;
		default:
			break;
	}
}

static void network_scan (int frontend_fd)
{
	if (initial_tune (frontend_fd) < 0) {
		error("Sorry - i couldn't get any working frequency/transponder\n Nothing to scan!!\n");
		return;
	}

	do {
		scan_tp();
	} while (tune_to_next_transponder(frontend_fd) == 0);
}


static char sat_polarisation (struct transponder *t)
{
	return 'h';
}


static void dump_lists (void)
{
	struct list_head *p1, *p2;
	struct transponder *t;
	struct service *s;
	int n = 0, i, index = 0;
	char sn[20];

	list_for_each(p1, &scanned_transponders) {
		t = list_entry(p1, struct transponder, list);
		list_for_each(p2, &t->services) {
			n++;
		}
	}
	info("dumping lists (%d services)\n", n);
	list_for_each(p1, &scanned_transponders) {
		t = list_entry(p1, struct transponder, list);
		if (output_format == OUTPUT_DVBSCAN_TUNING_DATA) {
			dvbscan_dump_tuningdata (stdout,
				   t->type,
				   &t->param,
				   index);
			index++;
			continue;
			}			
		list_for_each(p2, &t->services) {
			s = list_entry(p2, struct service, list);

			if (!s->service_name) {
				/* not in SDT */
				snprintf(sn, sizeof(sn), "[%04x]", s->service_id);
				s->service_name = strdup(sn);
			}
			/* ':' is field separator in vdr service lists */
			for (i = 0; s->service_name[i]; i++) {
				if (s->service_name[i] == ':')
					s->service_name[i] = ' ';
			}
			for (i = 0; s->provider_name && s->provider_name[i]; i++) {
				if (s->provider_name[i] == ':')
					s->provider_name[i] = ' ';
			}
			if (s->video_pid && !(serv_select & 1))						// vpid, this is tv
				continue; /* no TV services */
			if (!s->video_pid &&  (s->audio_num || s->ac3_pid) && !(serv_select & 2))	// no vpid, but apid or ac3pid, this is radio
				continue; /* no radio services */
			if (!s->video_pid && !(s->audio_num || s->ac3_pid) && !(serv_select & 4))	// no vpid, no apid, no ac3pid, this is service/other
				continue; /* no data/other services */
			if (s->scrambled && !ca_select)							// caid, this is scrambled tv or radio
				continue; /* FTA only */
			switch (output_format)
			{
			  case OUTPUT_VDR:
				vdr_dump_service_parameter_set (stdout,
						    s->service_name,
						    s->provider_name,
						    t->type,
						    &t->param,
						    sat_polarisation(t),
						    s->video_pid,
						    s->pcr_pid,
						    s->audio_pid,
						    s->audio_lang,
						    s->audio_num,
						    s->teletext_pid,
						    s->scrambled,
						    //FIXME: s->subtitling_pid
						    s->ac3_pid,
						    s->service_id,
						    t->original_network_id,	// onid patch by Hartmut Birr
						    t->transport_stream_id,
						    vdr_dump_provider,
						    s->ca_id,
						    s->ca_num,
						    ca_select,
						    vdr_version,
						    vdr_dump_channum,
						    s->channel_num);
				break;
			  case OUTPUT_KAFFEINE:
				kaffeine_dump_service_parameter_set (stdout,
						    s->service_name,
						    s->provider_name,
						    t->type,
						    &t->param,
						    sat_polarisation(t),
						    s->video_pid,
						    s->pcr_pid,
						    s->audio_pid,
						    //FIXME: s->audio_lang
						    s->audio_num,
						    s->teletext_pid,
						    s->scrambled,
						    //FIXME: s->subtitling_pid
						    s->ac3_pid,
						    s->service_id,
						    t->original_network_id,	// onid patch by Hartmut Birr
						    t->transport_stream_id,
						    vdr_dump_provider,
						    ca_select,
						    vdr_version,
						    vdr_dump_channum,
						    s->channel_num);
				break;
			  case OUTPUT_XINE:
				xine_dump_service_parameter_set (stdout,
						   s->service_name,
						   s->provider_name,
						   t->type,
						   &t->param,
						   s->video_pid,
						   s->audio_pid,
						   s->service_id);
				break;
			  default:
				break;
			  }
		}
	}
	info("Done.\n");
}

static void handle_sigint(int sig)
{
	error("interrupted by SIGINT, dumping partial result...\n");
	dump_lists();
	exit(2);
}

static const char *usage = "\n"
	"usage: %s [options...] \n"
	"	-a N	use device /dev/dvb/adapterN/ [default: auto detect]\n"
	"	-f type	frontend type\n"
	"		What programs do you want to search for?\n"
	"		a = atsc (vsb/qam)\n"
	"		c = cable\n"
	"		t = terrestrian [default]\n"
	"	-A N	specify ATSC type\n"
	"		1 = Terrestrial [default]\n"
	"		2 = Cable\n"
	"		3 = both, Terrestrial and Cable\n"
	"	-P 	do not use ATSC PSIP tables for scanning\n"
	"	   	(but only PAT and PMT) (applies for ATSC only)\n"
	"	-i N	spectral inversion setting for cable TV\n"
	"		DVB-T: always off\n"
	"		DVB-C (0: off, 1: on, 2: auto [default])\n"
	"	-F	use long filter timeout\n"
	"	-t N	tuning timeout\n"
	"		1 = fastest [default]\n"
	"		2 = medium\n"
	"		3 = slowest\n"
	"	-k	generate channels.dvb for kaffeine\n"
	"	-o N	VDR version / channels.conf format\n"
	"		2 = VDR-1.2.x (depriciated)\n"
	"		3 = VDR-1.3.x (depriciated)\n"
	"		4 = VDR-1.4.x/VDR-1.5.x (default)\n"
	"	-R N	radio channels\n"
	"		0 = don't search radio channels\n"
	"		1 = search radio channels [default]\n"
	"	-T N	TV channels\n"
	"		0 = don't search TV channels\n"
	"		1 = search radio TV [default]\n"
	"	-O N	Other Services\n"
	"		0 = don't search other services [default]\n"
	"		1 = search other services\n"
	"	-E N	Conditional Access (encrypted channels)\n"
	"		N=0 gets only Free TV channels\n"
	"		N=1 search also encrypted channels [default]\n"
	"	-X	tzap/czap/xine output instead of vdr channels.conf\n"
	"	-x	generate initial tuning data for (dvb-)scan\n"
	"	-v 	verbose (repeat for more)\n"
	"	-q 	quiet   (repeat for less)\n";


void
bad_usage(char *pname)
{
		fprintf (stderr, usage, pname);

}

int main (int argc, char **argv)
{
	char frontend_devname [80];
	int adapter = 999, frontend = 0, demux = 0;
	int opt, i, j;
	int frontend_fd;
	int fe_open_mode;
	int frontend_type = FE_OFDM;
	int Radio_Services = 1;
	int TV_Services = 1;
	int Other_Services = 0; // 20080106: don't search other services by default.
	int retVersion = 0;
	int discover = 0;
	int a=0,c=0,t=0;
	while ((opt = getopt(argc, argv, "ha:f:A:Pi:Ft:o:R:T:O:E:vVqXxk")) != -1) {
		switch (opt) {
		case 'a':
			adapter = strtoul(optarg, NULL, 0);
			break;
		case 'f':
			if (strcmp(optarg, "t") == 0) frontend_type = FE_OFDM;
			if (strcmp(optarg, "c") == 0) frontend_type = FE_QAM;
			if (strcmp(optarg, "a") == 0) frontend_type = FE_ATSC;
			if (strcmp(optarg, "?") == 0) discover++;
			break;
		case 'i':
			caps_inversion = strtoul(optarg, NULL, 0);
			break;
		case 'F':
			long_timeout = 1;
			break;
		case 't':
			tuning_speed = strtoul(optarg, NULL, 0);
			if ((tuning_speed < 1)) bad_usage(argv[0]);
			if ((tuning_speed > 3)) bad_usage(argv[0]);
			break;
		case 'o':
			vdr_version = strtoul(optarg, NULL, 0);
			if (vdr_version != 2) {
				vdr_dump_provider = 1;
				}
			break;
		case 'R':
			Radio_Services = strtoul(optarg, NULL, 0);
			if ((Radio_Services < 0)) bad_usage(argv[0]);
			if ((Radio_Services > 1)) bad_usage(argv[0]);
			break;
		case 'T':
			TV_Services = strtoul(optarg, NULL, 0);
			if ((TV_Services < 0)) bad_usage(argv[0]);
			if ((TV_Services > 1)) bad_usage(argv[0]);
			break;
		case 'O':
			Other_Services = strtoul(optarg, NULL, 0);
			if ((Other_Services < 0)) bad_usage(argv[0]);
			if ((Other_Services > 1)) bad_usage(argv[0]);
			break;
		case 'E':
			ca_select = strtoul(optarg, NULL, 0);
			break;
		case 'v':
			verbosity++;
			break;
		case 'V':
			retVersion++;
			break;
		case 'q':
			if (--verbosity < 0)
				verbosity = 0;
			break;
		case 'X':
			output_format = OUTPUT_XINE;
			break;
		case 'x':
			output_format = OUTPUT_DVBSCAN_TUNING_DATA;
			break;
		case 'k':
			output_format = OUTPUT_KAFFEINE;
			break;
		case 'P':
			no_ATSC_PSIP = 1;
			break;
		case 'A':
			ATSC_type = strtoul(optarg,NULL,0);
			if (ATSC_type == 0 || ATSC_type > 3) {
				bad_usage(argv[0]);
				return -1;
			}
			/* if -A is specified, it implies -f a */
			frontend_type = FE_ATSC;
			break;
		default:
			bad_usage(argv[0]);
			return -1;
		};
	}
	if (retVersion) {
		info ("%d", version);
		return 0;
		}
	if (discover) {
		discover=0;
		fe_open_mode = O_RDWR | O_NONBLOCK;
		for (i=0; i < 8; i++) {
		  snprintf (frontend_devname, sizeof(frontend_devname),  		"/dev/dvb/adapter%i/frontend0", i);
		  if ((frontend_fd = open (frontend_devname, fe_open_mode)) < 0)
			continue;
		  if (ioctl(frontend_fd, FE_GET_INFO, &fe_info) == -1) {
			close (frontend_fd);
			continue;
			}		  
		  if (fe_info.type == FE_OFDM)
			t++;
		  else
			if (fe_info.type == FE_QAM)
					c++;
		  else
			if (fe_info.type == FE_ATSC)
					a++;
                        close (frontend_fd);
		  }
		info ("%2d%2d%2d", t, c, a);
		return 0;
		}
	info("w_scan version %d\n", version);
	serv_select = 1 * TV_Services + 2 * Radio_Services + 4 * Other_Services;
	if  ((caps_inversion > INVERSION_AUTO) || (caps_inversion < INVERSION_OFF)) {
		info("Inversion out of range!\n");
		bad_usage(argv[0]);
		return -1;
		}
	if  (((adapter > 7) && (adapter != 999)) || (adapter < 0)) {
		info("Invalid adapter: out of range (0..7)\n");
		bad_usage(argv[0]);
		return -1;
		}

	if ( adapter == 999 ) {
		info("Info: using DVB adapter auto detection.\n");
		fe_open_mode = O_RDWR | O_NONBLOCK;
		for (i=0; i < 8; i++) {
		  for (j=0; j < 4; j++) {
		    snprintf (frontend_devname, sizeof(frontend_devname), "/dev/dvb/adapter%i/frontend%i", i, j);
		    if ((frontend_fd = open (frontend_devname, fe_open_mode)) < 0) {
			continue;
			}
		    /* determine FE type and caps */
		    if (ioctl(frontend_fd, FE_GET_INFO, &fe_info) == -1) {
			info("   ERROR: unable to determine frontend type\n");
			close (frontend_fd);
			continue;
			}
		    if (fe_info.type == frontend_type) {
			if (fe_info.type == FE_OFDM) 
			  info("   Found DVB-T frontend. Using adapter %s\n",frontend_devname);
                        else if (fe_info.type == FE_ATSC)
			  info("   Found ATSC frontend. Using adapter %s\n",frontend_devname);
                        else
			  info("   Found DVB-C frontend. Using adapter %s\n",frontend_devname);                     
			close (frontend_fd);
			adapter=i;
			frontend=j;
			i=999;
			break;
			}
                     else {
			info("   Wrong type, ignoring frontend %s\n",frontend_devname);
			close (frontend_fd);
			}

		  }
		}
	}
	snprintf (frontend_devname, sizeof(frontend_devname),
		  "/dev/dvb/adapter%i/frontend%i", adapter, frontend);

	snprintf (demux_devname, sizeof(demux_devname),
		  "/dev/dvb/adapter%i/demux%i", adapter, demux);

	for (i = 0; i < MAX_RUNNING; i++)
		poll_fds[i].fd = -1;

	fe_open_mode = O_RDWR;
	if (adapter == 999) 
		fatal("***** NO USEABLE DVB CARD FOUND. *****\nPlease check wether dvb driver is loaded and\nverify that no dvb application (i.e. vdr) is running.\n");
        if ((frontend_fd = open (frontend_devname, fe_open_mode)) < 0)
		fatal("failed to open '%s': %d %m\n", frontend_devname, errno);
	info("-_-_-_-_ Getting frontend capabilities-_-_-_-_ \n");
	/* determine FE type and caps */
	if (ioctl(frontend_fd, FE_GET_INFO, &fe_info) == -1)
		fatal("FE_GET_INFO failed: %d %m\n", errno);
	if (fe_info.type == FE_OFDM) {
		info("frontend %s supports\n", fe_info.name);
		if (fe_info.caps & FE_CAN_INVERSION_AUTO) {
		  info("INVERSION_AUTO\n");
		  caps_inversion=INVERSION_AUTO;
		  }
		else {
		  info("INVERSION_AUTO not supported, trying INVERSION_OFF.\n");
		  caps_inversion=INVERSION_OFF;
		  }
		if (fe_info.caps & FE_CAN_QAM_AUTO) {
		  info("QAM_AUTO\n");
		  caps_qam=QAM_AUTO;
		  }
		else {
		  info("QAM_AUTO not supported, trying QAM_64.\n");
		  caps_qam=QAM_64;
		  }
		if (fe_info.caps & FE_CAN_TRANSMISSION_MODE_AUTO) {
		  info("TRANSMISSION_MODE_AUTO\n");
		  caps_transmission_mode=TRANSMISSION_MODE_AUTO;
		  }
		else {
		  info("TRANSMISSION_MODE not supported, trying TRANSMISSION_MODE_8K.\n");
		  caps_transmission_mode=TRANSMISSION_MODE_8K;
		  }
		if (fe_info.caps & FE_CAN_GUARD_INTERVAL_AUTO) {
		  info("GUARD_INTERVAL_AUTO\n");
		  caps_guard_interval=GUARD_INTERVAL_AUTO;
		  }
		else {
		  info("GUARD_INTERVAL_AUTO not supported, trying GUARD_INTERVAL_1_8.\n");
		  caps_guard_interval=GUARD_INTERVAL_1_8;
		  }
		if (fe_info.caps & FE_CAN_HIERARCHY_AUTO) {
		  info("HIERARCHY_AUTO\n");
		  caps_hierarchy=HIERARCHY_AUTO;
		  }
		else {
		  info("HIERARCHY_AUTO not supported, trying HIERARCHY_NONE.\n");
		  caps_hierarchy=HIERARCHY_NONE;
		  }
		if (fe_info.caps & FE_CAN_FEC_AUTO) {
		  info("FEC_AUTO\n");
		  caps_fec=FEC_AUTO;
		  }
		else {
		  info("FEC_AUTO not supported, trying FEC_NONE.\n");
		  caps_fec=FEC_NONE;
		  }
		}
	if (fe_info.type == FE_QAM) {
		info("frontend %s supports\n", fe_info.name);
		if (fe_info.caps & FE_CAN_INVERSION_AUTO) {
		  info("INVERSION_AUTO\n");
		  caps_inversion=INVERSION_AUTO;
		  }
		else {
		  info("INVERSION_AUTO not supported, trying INVERSION_OFF.\n");
		  caps_inversion=INVERSION_OFF;
		  }
		if (fe_info.caps & FE_CAN_QAM_AUTO) {
		  info("QAM_AUTO\n");
		  caps_qam=QAM_AUTO;
		  }
		else {
		  info("QAM_AUTO not supported, trying QAM_64 and QAM_256.\n");
		  caps_qam=QAM_64;
		  qam_no_auto++;
		  }
		if (fe_info.caps & FE_CAN_FEC_AUTO) {
		  info("FEC_AUTO\n");
		  caps_fec=FEC_AUTO;
		  }
		else {
		  info("FEC_AUTO not supported, trying FEC_NONE.\n");
		  caps_fec=FEC_NONE;
		  }
		}
	if (fe_info.type == FE_ATSC) {
		info("frontend %s supports\n", fe_info.name);
		if (fe_info.caps & FE_CAN_8VSB) {
		  info("FE_CAN_8VSB\n");
		  }
		if (fe_info.caps & FE_CAN_16VSB) {
		  info("FE_CAN_16VSB\n");
		  }
		if (fe_info.caps & FE_CAN_QAM_64) {
		  info("FE_CAN_QAM_64\n");
		  }
		if (fe_info.caps & FE_CAN_QAM_256) {
		  info("FE_CAN_QAM_256\n");
		  }
		}
	info("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_ \n");

	signal(SIGINT, handle_sigint);

	network_scan (frontend_fd);

	close (frontend_fd);

	dump_lists ();

	return 0;
}

static void dump_dvb_parameters (FILE *f, struct transponder *t)
{
	struct dvb_frontend_parameters *p=&t->param;
	switch (output_format) {
		case OUTPUT_VDR:
			vdr_dump_dvb_parameters(f, t->type, &t->param,
					sat_polarisation (t));
			break;
		case OUTPUT_DVBSCAN_TUNING_DATA:
			fprintf (f, "%i ", p->frequency/1000);
			break;
		default:
			break;
	}
}
