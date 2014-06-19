/*
 * dvb_frontend.h
 *
 * Copyright (C) 2001 convergence integrated media GmbH
 * Copyright (C) 2004 convergence GmbH
 *
 * Written by Ralph Metzler
 * Overhauled by Holger Waechtler
 * Kernel I2C stuff by Michael Hunold <hunold@convergence.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
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

/* Minimal version of drivers/media/dvb-core/dvb_frontend.h
   from Linux' source tree */

#ifndef _DVB_FRONTEND_H_
#define _DVB_FRONTEND_H_

struct dvb_frontend_tune_settings {
	int min_delay_ms;
	int step_size;
	int max_drift;
};

struct dtv_frontend_properties {

	/* Cache State */
	u32			state;

	u32			frequency;
	fe_modulation_t		modulation;

	fe_sec_voltage_t	voltage;
	fe_sec_tone_mode_t	sectone;
	fe_spectral_inversion_t	inversion;
	fe_code_rate_t		fec_inner;
	fe_transmit_mode_t	transmission_mode;
	u32			bandwidth_hz;	/* 0 = AUTO */
	fe_guard_interval_t	guard_interval;
	fe_hierarchy_t		hierarchy;
	u32			symbol_rate;
	fe_code_rate_t		code_rate_HP;
	fe_code_rate_t		code_rate_LP;

	fe_pilot_t		pilot;
	fe_rolloff_t		rolloff;

	fe_delivery_system_t	delivery_system;

	enum fe_interleaving	interleaving;

	/* ISDB-T specifics */
	u8			isdbt_partial_reception;
	u8			isdbt_sb_mode;
	u8			isdbt_sb_subchannel;
	u32			isdbt_sb_segment_idx;
	u32			isdbt_sb_segment_count;
	u8			isdbt_layer_enabled;
	struct {
	    u8			segment_count;
	    fe_code_rate_t	fec;
	    fe_modulation_t	modulation;
	    u8			interleaving;
	} layer[3];

	/* Multistream specifics */
	u32			stream_id;

	/* ATSC-MH specifics */
	u8			atscmh_fic_ver;
	u8			atscmh_parade_id;
	u8			atscmh_nog;
	u8			atscmh_tnog;
	u8			atscmh_sgn;
	u8			atscmh_prc;

	u8			atscmh_rs_frame_mode;
	u8			atscmh_rs_frame_ensemble;
	u8			atscmh_rs_code_mode_pri;
	u8			atscmh_rs_code_mode_sec;
	u8			atscmh_sccc_block_mode;
	u8			atscmh_sccc_code_mode_a;
	u8			atscmh_sccc_code_mode_b;
	u8			atscmh_sccc_code_mode_c;
	u8			atscmh_sccc_code_mode_d;

	u32			lna;

	/* statistics data */
	struct dtv_fe_stats	strength;
	struct dtv_fe_stats	cnr;
	struct dtv_fe_stats	pre_bit_error;
	struct dtv_fe_stats	pre_bit_count;
	struct dtv_fe_stats	post_bit_error;
	struct dtv_fe_stats	post_bit_count;
	struct dtv_fe_stats	block_error;
	struct dtv_fe_stats	block_count;
};

#endif
