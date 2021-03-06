/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2014, Gana�l Laplanche, http://contribs.martymac.org
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

#ifndef SMS1XXX_FRONTEND_H
#define SMS1XXX_FRONTEND_H

#include "sms1xxx.h"

void sms1xxx_frontend_init(struct sms1xxx_softc *);
void sms1xxx_frontend_exit(struct sms1xxx_softc *);

int sms1xxx_frontend_read_status(struct sms1xxx_softc *, fe_status_t *);
int sms1xxx_frontend_read_ber(struct sms1xxx_softc *, u32 *);
int sms1xxx_frontend_read_signal_strength(struct sms1xxx_softc *, u16 *);
int sms1xxx_frontend_read_snr(struct sms1xxx_softc *, u16 *);
int sms1xxx_frontend_read_ucblocks(struct sms1xxx_softc *, u32 *);
int sms1xxx_frontend_set_frontend(struct sms1xxx_softc *,
    struct dvb_frontend_parameters *);
int sms1xxx_frontend_get_frontend(struct sms1xxx_softc *,
    struct dvb_frontend_parameters *);
int sms1xxx_frontend_set_properties(struct sms1xxx_softc *sc,
    struct dtv_properties *);
int sms1xxx_frontend_get_properties(struct sms1xxx_softc *sc,
    struct dtv_properties *);
int sms1xxx_frontend_get_tune_settings(struct sms1xxx_softc *,
    struct dvb_frontend_tune_settings *);
void sms1xxx_frontend_stats_not_ready(struct sms1xxx_softc *sc);

#endif
