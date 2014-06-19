/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2012, Ganaël Laplanche, http://contribs.martymac.org
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
 *
 * * * *
 *  Device frontend
 *  (handles frontend0 device).
 * * * *
 */

/* poll(2) stuff */
#include <sys/poll.h>

/* File-descriptor ioctls */
#include <sys/filio.h>

/* Our stuff */
#include "sms1xxx.h"
#include "sms1xxx-usb.h"
#include "sms1xxx-frontend.h"

/* Linux stuff */
#include "linux/dvb/frontend.h"
#include "linux/dvb/version.h"

static d_open_t  sms1xxx_frontend_open;
static d_close_t sms1xxx_frontend_close;
static d_poll_t sms1xxx_frontend_poll;
static d_ioctl_t sms1xxx_frontend_ioctl;

static struct cdevsw sms1xxx_frontend_cdevsw = {
    .d_version  = D_VERSION,
    .d_flags    = D_NEEDGIANT,
    .d_open     = sms1xxx_frontend_open,
    .d_close    = sms1xxx_frontend_close,
    .d_poll     = sms1xxx_frontend_poll,
    .d_ioctl    = sms1xxx_frontend_ioctl,
    .d_name     = "sms1xxx_frontend",
};

/*********************
 * Private functions *
 *********************/

static int
sms1xxx_frontend_open(struct cdev *dev, int flag, int mode, struct thread *p)
{
    int err = 0;
    struct sms1xxx_softc *sc = dev->si_drv1;
#ifdef SMS1XXX_DEBUG
    int unit = device_get_unit(sc->sc_dev);
    TRACE(TRACE_OPEN,"flag=%d mode=%d unit=%d\n",flag,mode,unit);
#endif

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    if(sc->feopen) {
        TRACE(TRACE_OPEN,"busy!\n");
        return (EBUSY);
    }
    sc->feopen = 1;
    err =  sms1xxx_usb_ref(sc);
    if(err)
        sc->feopen = 0;
    return (err);
}

static int
sms1xxx_frontend_close(struct cdev *dev, int flag, int mode, struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
#ifdef SMS1XXX_DEBUG
    int unit = device_get_unit(sc->sc_dev);
    TRACE(TRACE_OPEN,"flag=%d mode=%d unit=%d\n",flag,mode,unit);
#endif

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_OPEN,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    sc->feopen = 0;
    sms1xxx_usb_deref(sc);
    return (0);
}

static int
sms1xxx_frontend_poll(struct cdev *dev, int events, struct thread *p)
{
    int revents = 0;
    TRACE(TRACE_POLL,"thread=%p events=%d\n", p, events);

    /* Not implemented, so no blocking */
    if(events & (POLLIN | POLLRDNORM))
        revents |= events & (POLLIN | POLLRDNORM);
    if(events & (POLLOUT | POLLWRNORM))
        revents |= events & (POLLOUT | POLLWRNORM);
    if (events & (POLLPRI | POLLRDBAND))
        revents |= events & (POLLPRI | POLLRDBAND);

    return (revents);
}

static int
sms1xxx_frontend_ioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flag,
    struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
    struct sms1xxx_frontend *fe;
    void *arg = addr;
    int err = 0;

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_IOCTL,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    fe = sc->device->frontend;

    switch(cmd) {
        case FE_GET_INFO:
            TRACE(TRACE_IOCTL,"FE_GET_INFO\n");
            memcpy(arg,&fe->info, sizeof(struct dvb_frontend_info));
            break;
        case FE_READ_STATUS:
            err = fe->read_status(sc,(fe_status_t*)arg);
            TRACE(TRACE_IOCTL,"FE_READ_STATUS=%d\n",err);
            break;
        case FE_READ_BER:
            err = fe->read_ber(sc,(u32*)arg);
            TRACE(TRACE_IOCTL,"FE_READ_BER=%d\n",err);
            break;
        case FE_READ_UNCORRECTED_BLOCKS:
            err = fe->read_ucblocks(sc,(u32*)arg);
            TRACE(TRACE_IOCTL,"FE_READ_UNCORRECTED_BLOCKS\n");
            break;
        case FE_READ_SIGNAL_STRENGTH:
            err = fe->read_signal_strength(sc,(u16*)arg);
            TRACE(TRACE_IOCTL,"FE_READ_SIGNAL_STRENGTH=%d\n",err);
            break;
        case FE_READ_SNR:
            err = fe->read_snr(sc,(u16*)arg);
            TRACE(TRACE_IOCTL,"FE_READ_SNR=%d\n",err);
            break;
        case FE_SET_FRONTEND:
            err = fe->set_frontend(sc,(struct dvb_frontend_parameters *)arg);
            TRACE(TRACE_IOCTL,"FE_SET_FRONTEND=%d\n",err);
            break;
        case FE_GET_FRONTEND:
            err = fe->get_frontend(sc,(struct dvb_frontend_parameters *)arg);
            TRACE(TRACE_IOCTL,"FE_GET_FRONTEND\n");
            break;
        case FE_GET_EVENT:
            {
                /* XXX Events are not implemented (yet)
                 * we just return a dummy and EWOULDBLOCK.
                 */
                struct dvb_frontend_event *ev = arg;
                fe_status_t st;
                fe->read_status(sc, &st);
                ev->status = st;
                fe->get_frontend(sc,&ev->parameters);
                err = EWOULDBLOCK;
                TRACE(TRACE_IOCTL,"FE_GET_EVENT (status=%d)\n",st);
                break;
            }
        case FE_SET_PROPERTY:
            err = fe->set_property(sc,(struct dtv_properties *)arg);
            TRACE(TRACE_IOCTL,"FE_SET_PROPERTY\n");
            break;
        case FE_GET_PROPERTY:
            err = fe->get_property(sc,(struct dtv_properties *)arg);
            TRACE(TRACE_IOCTL,"FE_GET_PROPERTY\n");
            break;
        case FIOASYNC:
            TRACE(TRACE_IOCTL,"FIOASYNC\n");
            break;
        case FIONBIO:
            TRACE(TRACE_IOCTL,"FIONBIO\n");
            break;
        default:
            ERR("unknown ioctl: 0x%lx\n",cmd);
            err = ENOTTY;
            break;
    }
    return (err);
}

/* Change status LED state on a frontend event */
static inline int sms1xxx_frontend_status_led_feedback(struct sms1xxx_softc *sc)
{
    if (sc->fe_status & FE_HAS_LOCK)
        return sms1xxx_gpio_status_led_feedback(
            sc,
            (sc->sms_stat_dvb.ReceptionData.BER == 0) ?
            SMS_LED_HI : SMS_LED_LO);
    else
        return sms1xxx_gpio_status_led_feedback(sc, SMS_LED_OFF);
}

/********************
 * Public functions *
 ********************/

void
sms1xxx_frontend_init(struct sms1xxx_softc *sc)
{
    sc->frontenddev = make_dev(&sms1xxx_frontend_cdevsw,
        device_get_unit(sc->sc_dev),
        UID_ROOT, GID_WHEEL, 0666,
        "dvb/adapter%d/frontend0",
        device_get_unit(sc->sc_dev));
    if (sc->frontenddev != NULL)
        sc->frontenddev->si_drv1 = sc;
    TRACE(TRACE_MODULE,"created frontend0 device, addr=%p\n",sc->frontenddev);
}

void
sms1xxx_frontend_exit(struct sms1xxx_softc *sc)
{
    if(sc->frontenddev != NULL) {
        TRACE(TRACE_MODULE,"destroying frontend0, addr=%p\n",sc->frontenddev);
        destroy_dev(sc->frontenddev);
        sc->frontenddev = NULL;
    }
}

int
sms1xxx_frontend_read_status(struct sms1xxx_softc *sc, fe_status_t *status)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = sms1xxx_usb_getstatistics(sc);

    *status=sc->fe_status;

    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_ber(struct sms1xxx_softc *sc, u32 *ber)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = sms1xxx_usb_getstatistics(sc);

    *ber=sc->sms_stat_dvb.ReceptionData.BER;

    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_signal_strength(struct sms1xxx_softc *sc, u16 *strength)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = sms1xxx_usb_getstatistics(sc);

    if (sc->sms_stat_dvb.ReceptionData.InBandPwr < -95)
        *strength = 0;
    else if (sc->sms_stat_dvb.ReceptionData.InBandPwr > -29)
        *strength = 100;
    else
        *strength = (sc->sms_stat_dvb.ReceptionData.InBandPwr + 95) * 3 / 2;

    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_snr(struct sms1xxx_softc *sc, u16 *snr)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = sms1xxx_usb_getstatistics(sc);

    *snr=sc->sms_stat_dvb.ReceptionData.SNR;

    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_ucblocks(struct sms1xxx_softc *sc, u32 *ucblocks)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = sms1xxx_usb_getstatistics(sc);

    *ucblocks=sc->sms_stat_dvb.ReceptionData.ErrorTSPackets;

    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

static int
sms1xxx_frontend_fep_to_c(const struct dvb_frontend_parameters *p,
    struct dtv_frontend_properties *c);

int
sms1xxx_frontend_set_frontend(struct sms1xxx_softc *sc,
    struct dvb_frontend_parameters *fep)
{
    TRACE(TRACE_IOCTL,"\n");

    /* Update DVBv3 fe_params state */
    memcpy(&sc->fe_params, fep,
        sizeof(struct dvb_frontend_parameters));

    /* Update DVBv5 dtv_property_cache state */
    /* XXX Force DVB-T */
    sc->dtv_property_cache.delivery_system = SYS_DVBT;
    return (sms1xxx_frontend_fep_to_c(
        (const struct dvb_frontend_parameters *)&sc->fe_params,
        &sc->dtv_property_cache
    ) || sms1xxx_usb_setfrequency(
        sc,
        sc->fe_params.frequency + sc->device->freq_offset,
        sc->fe_params.u.ofdm.bandwidth
    ));
}

int
sms1xxx_frontend_get_frontend(struct sms1xxx_softc *sc,
    struct dvb_frontend_parameters *fep)
{
    TRACE(TRACE_IOCTL,"\n");
    memcpy(fep, &sc->fe_params,
        sizeof(struct dvb_frontend_parameters));
    return (0);
}

int
sms1xxx_frontend_get_tune_settings(struct sms1xxx_softc *sc,
    struct dvb_frontend_tune_settings *tune)
{
    TRACE(TRACE_IOCTL,"\n");
    tune->min_delay_ms = 400;
    tune->step_size = 250000;
    tune->max_drift = 0;
    return (0);
}

/* Below are functions that add minimal DVBv5 support */

enum dvbv3_emulation_type {
    DVBV3_UNKNOWN,
    DVBV3_QPSK,
    DVBV3_QAM,
    DVBV3_OFDM,
    DVBV3_ATSC,
};

static enum dvbv3_emulation_type
dvbv3_type(u32 delivery_system)
{
    switch (delivery_system) {
        case SYS_DVBC_ANNEX_A:
        case SYS_DVBC_ANNEX_C:
            return DVBV3_QAM;
        case SYS_DVBS:
        case SYS_DVBS2:
        case SYS_TURBO:
        case SYS_ISDBS:
        case SYS_DSS:
            return DVBV3_QPSK;
        case SYS_DVBT:
        case SYS_DVBT2:
        case SYS_ISDBT:
        case SYS_DTMB:
            return DVBV3_OFDM;
        case SYS_ATSC:
        case SYS_ATSCMH:
        case SYS_DVBC_ANNEX_B:
            return DVBV3_ATSC;
        case SYS_UNDEFINED:
        case SYS_ISDBC:
        case SYS_DVBH:
        case SYS_DAB:
        default:
            /*
             * Doesn't know how to emulate those types and/or
             * there's no frontend driver from this type yet
             * with some emulation code, so, we're not sure yet how
             * to handle them, or they're not compatible with a DVBv3 call.
             */
            return DVBV3_UNKNOWN;
    }
}

/* Clear DVBv5 property cache
   Upstream name: dvb_frontend_clear_cache() */
static int
sms1xxx_frontend_clear_cache(struct sms1xxx_softc *sc)
{
    struct dtv_frontend_properties *c = &sc->dtv_property_cache;
    int i;
    u32 delsys;

    delsys = c->delivery_system;
    memset(c, 0, offsetof(struct dtv_frontend_properties, strength));
    c->delivery_system = delsys;

    c->state = DTV_CLEAR;

    TRACE(TRACE_FRONTEND,"Clearing cache for delivery system %d\n",
        c->delivery_system);

    c->transmission_mode = TRANSMISSION_MODE_AUTO;
    c->bandwidth_hz = 0;    /* AUTO */
    c->guard_interval = GUARD_INTERVAL_AUTO;
    c->hierarchy = HIERARCHY_AUTO;
    c->symbol_rate = 0;
    c->code_rate_HP = FEC_AUTO;
    c->code_rate_LP = FEC_AUTO;
    c->fec_inner = FEC_AUTO;
    c->rolloff = ROLLOFF_AUTO;
    c->voltage = SEC_VOLTAGE_OFF;
    c->sectone = SEC_TONE_OFF;
    c->pilot = PILOT_AUTO;

    c->isdbt_partial_reception = 0;
    c->isdbt_sb_mode = 0;
    c->isdbt_sb_subchannel = 0;
    c->isdbt_sb_segment_idx = 0;
    c->isdbt_sb_segment_count = 0;
    c->isdbt_layer_enabled = 0;
    for (i = 0; i < 3; i++) {
        c->layer[i].fec = FEC_AUTO;
        c->layer[i].modulation = QAM_AUTO;
        c->layer[i].interleaving = 0;
        c->layer[i].segment_count = 0;
    }

    c->stream_id = NO_STREAM_ID_FILTER;

    switch (c->delivery_system) {
        case SYS_DVBS:
        case SYS_DVBS2:
        case SYS_TURBO:
            c->modulation = QPSK;   /* implied for DVB-S in legacy API */
            c->rolloff = ROLLOFF_35;/* implied for DVB-S */
            break;
        case SYS_ATSC:
            c->modulation = VSB_8;
            break;
        default:
            c->modulation = QAM_AUTO;
            break;
    }

    c->lna = LNA_AUTO;

    return (0);
}

/* DVBv3 dvb_frontend_parameters to DVBv5 dtv_frontend_properties cache sync
   Upstream name: dtv_property_cache_sync() */
static int
sms1xxx_frontend_fep_to_c(const struct dvb_frontend_parameters *p,
    struct dtv_frontend_properties *c)
{
    c->frequency = p->frequency;
    c->inversion = p->inversion;

    switch (dvbv3_type(c->delivery_system)) {
        case DVBV3_QPSK:
            c->symbol_rate = p->u.qpsk.symbol_rate;
            c->fec_inner = p->u.qpsk.fec_inner;
            break;
        case DVBV3_QAM:
            c->symbol_rate = p->u.qam.symbol_rate;
            c->fec_inner = p->u.qam.fec_inner;
            c->modulation = p->u.qam.modulation;
            break;
        case DVBV3_OFDM:
            switch (p->u.ofdm.bandwidth) {
                case BANDWIDTH_10_MHZ:
                    c->bandwidth_hz = 10000000;
                    break;
                case BANDWIDTH_8_MHZ:
                    c->bandwidth_hz = 8000000;
                    break;
                case BANDWIDTH_7_MHZ:
                    c->bandwidth_hz = 7000000;
                    break;
                case BANDWIDTH_6_MHZ:
                    c->bandwidth_hz = 6000000;
                    break;
                case BANDWIDTH_5_MHZ:
                    c->bandwidth_hz = 5000000;
                    break;
                case BANDWIDTH_1_712_MHZ:
                    c->bandwidth_hz = 1712000;
                    break;
                case BANDWIDTH_AUTO:
                    c->bandwidth_hz = 0;
            }

            c->code_rate_HP = p->u.ofdm.code_rate_HP;
            c->code_rate_LP = p->u.ofdm.code_rate_LP;
            c->modulation = p->u.ofdm.constellation;
            c->transmission_mode = p->u.ofdm.transmission_mode;
            c->guard_interval = p->u.ofdm.guard_interval;
            c->hierarchy = p->u.ofdm.hierarchy_information;
            break;
        case DVBV3_ATSC:
            c->modulation = p->u.vsb.modulation;
            if (c->delivery_system == SYS_ATSCMH)
                break;
            if ((c->modulation == VSB_8) || (c->modulation == VSB_16))
                c->delivery_system = SYS_ATSC;
            else
                c->delivery_system = SYS_DVBC_ANNEX_B;
            break;
        case DVBV3_UNKNOWN:
            TRACE(TRACE_FRONTEND,"Unknown delivery system\n");
            return (EINVAL);
    }

    return (0);
}

/* DVBv5 dtv_frontend_properties cache to DVBv3 dvb_frontend_parameters sync
   Upstream name: dtv_property_legacy_params_sync() */
static int
sms1xxx_frontend_c_to_fep(const struct dtv_frontend_properties *c,
    struct dvb_frontend_parameters *p)
{
    p->frequency = c->frequency;
    p->inversion = c->inversion;

    switch (dvbv3_type(c->delivery_system)) {
        case DVBV3_QPSK:
            p->u.qpsk.symbol_rate = c->symbol_rate;
            p->u.qpsk.fec_inner = c->fec_inner;
            break;
        case DVBV3_QAM:
            p->u.qam.symbol_rate = c->symbol_rate;
            p->u.qam.fec_inner = c->fec_inner;
            p->u.qam.modulation = c->modulation;
            break;
        case DVBV3_OFDM:
            switch (c->bandwidth_hz) {
                case 10000000:
                    p->u.ofdm.bandwidth = BANDWIDTH_10_MHZ;
                    break;
                case 8000000:
                    p->u.ofdm.bandwidth = BANDWIDTH_8_MHZ;
                    break;
                case 7000000:
                    p->u.ofdm.bandwidth = BANDWIDTH_7_MHZ;
                    break;
                case 6000000:
                    p->u.ofdm.bandwidth = BANDWIDTH_6_MHZ;
                    break;
                case 5000000:
                    p->u.ofdm.bandwidth = BANDWIDTH_5_MHZ;
                    break;
                case 1712000:
                    p->u.ofdm.bandwidth = BANDWIDTH_1_712_MHZ;
                    break;
                case 0:
                default:
                    p->u.ofdm.bandwidth = BANDWIDTH_AUTO;
            }
            p->u.ofdm.code_rate_HP = c->code_rate_HP;
            p->u.ofdm.code_rate_LP = c->code_rate_LP;
            p->u.ofdm.constellation = c->modulation;
            p->u.ofdm.transmission_mode = c->transmission_mode;
            p->u.ofdm.guard_interval = c->guard_interval;
            p->u.ofdm.hierarchy_information = c->hierarchy;
            break;
        case DVBV3_ATSC:
            p->u.vsb.modulation = c->modulation;
            break;
        case DVBV3_UNKNOWN:
            TRACE(TRACE_FRONTEND,"Unknown delivery system\n");
            return (EINVAL);
    }

    return (0);
}

/* Set a single frontend property
   Upstream name: dtv_property_process_set() */
static int
sms1xxx_frontend_set_single_property(struct sms1xxx_softc *sc,
    struct dtv_property *tvp)
{
    int err = 0;
    struct dtv_frontend_properties *c = &sc->dtv_property_cache;

    switch(tvp->cmd) {
        case DTV_CLEAR:
            /*
             * Reset a cache of data specific to the frontend here. This does
             * not affect hardware.
             */
            sms1xxx_frontend_clear_cache(sc);
            break;
        case DTV_TUNE:
            c->state = tvp->cmd;

            /* Update DVBv3 fe_params state */
            err = sms1xxx_frontend_c_to_fep(
                (const struct dtv_frontend_properties *)c,
                &sc->fe_params);
            /* TODO: Insert here default values for tuning settings -
               as in dtv_set_frontend() ? */

            TRACE(TRACE_FRONTEND,"Finalised property cache\n");
            /* Effective tuning is then performed through 
               a call to sms1xxx_usb_setfrequency() within
               sms1xxx_frontend_set_properties() */
            break;
        case DTV_FREQUENCY:
            c->frequency = tvp->u.data;
            break;
        case DTV_MODULATION:
            c->modulation = tvp->u.data;
            break;
        case DTV_BANDWIDTH_HZ:
            c->bandwidth_hz = tvp->u.data;
            break;
        case DTV_INVERSION:
            c->inversion = tvp->u.data;
            break;
        case DTV_SYMBOL_RATE:
            c->symbol_rate = tvp->u.data;
            break;
        case DTV_INNER_FEC:
            c->fec_inner = tvp->u.data;
            break;
        case DTV_PILOT:
            c->pilot = tvp->u.data;
            break;
        case DTV_ROLLOFF:
            c->rolloff = tvp->u.data;
            break;
        case DTV_DELIVERY_SYSTEM:
            /* XXX Force DVB-T */
            c->delivery_system = SYS_DVBT;
            break;
        case DTV_CODE_RATE_HP:
            c->code_rate_HP = tvp->u.data;
            break;
        case DTV_CODE_RATE_LP:
            c->code_rate_LP = tvp->u.data;
            break;
        case DTV_GUARD_INTERVAL:
            c->guard_interval = tvp->u.data;
            break;
        case DTV_TRANSMISSION_MODE:
            c->transmission_mode = tvp->u.data;
            break;
        case DTV_HIERARCHY:
            c->hierarchy = tvp->u.data;
            break;
        case DTV_INTERLEAVING:
            c->interleaving = tvp->u.data;
            break;

        /* Multistream support */
        case DTV_STREAM_ID:
        case DTV_DVBT2_PLP_ID_LEGACY:
            c->stream_id = tvp->u.data;
            break;

        default:
            TRACE(TRACE_FRONTEND,"Unsupported frontend property (%d)\n",
                tvp->cmd);
            err = EINVAL;
    }

    return (err);
}

/* Get a single frontend property
   Upstream name: dtv_property_process_get() */
static int
sms1xxx_frontend_get_single_property(struct sms1xxx_softc *sc,
    struct dtv_property *tvp)
{
    int err = 0;
    struct dtv_frontend_properties *c = &sc->dtv_property_cache;

    switch(tvp->cmd) {
        case DTV_ENUM_DELSYS:
            /* XXX Force DVB-T */
            tvp->u.buffer.len = 1;
            tvp->u.buffer.data[0] = SYS_DVBT;
            break;
        case DTV_FREQUENCY:
            tvp->u.data = c->frequency;
            break;
        case DTV_MODULATION:
            tvp->u.data = c->modulation;
            break;
        case DTV_BANDWIDTH_HZ:
            tvp->u.data = c->bandwidth_hz;
            break;
        case DTV_INVERSION:
            tvp->u.data = c->inversion;
            break;
        case DTV_SYMBOL_RATE:
            tvp->u.data = c->symbol_rate;
            break;
        case DTV_INNER_FEC:
            tvp->u.data = c->fec_inner;
            break;
        case DTV_PILOT:
            tvp->u.data = c->pilot;
            break;
        case DTV_ROLLOFF:
            tvp->u.data = c->rolloff;
            break;
        case DTV_DELIVERY_SYSTEM:
            tvp->u.data = c->delivery_system;
            break;
        case DTV_API_VERSION:
            tvp->u.data = (DVB_API_VERSION << 8) | DVB_API_VERSION_MINOR;
            break;
        case DTV_CODE_RATE_HP:
            tvp->u.data = c->code_rate_HP;
            break;
        case DTV_CODE_RATE_LP:
            tvp->u.data = c->code_rate_LP;
            break;
        case DTV_GUARD_INTERVAL:
            tvp->u.data = c->guard_interval;
            break;
        case DTV_TRANSMISSION_MODE:
            tvp->u.data = c->transmission_mode;
            break;
        case DTV_HIERARCHY:
            tvp->u.data = c->hierarchy;
            break;
        case DTV_INTERLEAVING:
            tvp->u.data = c->interleaving;
            break;

        /* Multistream support */
        case DTV_STREAM_ID:
        case DTV_DVBT2_PLP_ID_LEGACY:
            tvp->u.data = c->stream_id;
            break;

        /* Fill quality measures */
        /* TODO: stats unsupported yet */
        case DTV_STAT_SIGNAL_STRENGTH:
            tvp->u.st = c->strength;
            break;
        case DTV_STAT_CNR:
            tvp->u.st = c->cnr;
            break;
        case DTV_STAT_PRE_ERROR_BIT_COUNT:
            tvp->u.st = c->pre_bit_error;
            break;
        case DTV_STAT_PRE_TOTAL_BIT_COUNT:
            tvp->u.st = c->pre_bit_count;
            break;
        case DTV_STAT_POST_ERROR_BIT_COUNT:
            tvp->u.st = c->post_bit_error;
            break;
        case DTV_STAT_POST_TOTAL_BIT_COUNT:
            tvp->u.st = c->post_bit_count;
            break;
        case DTV_STAT_ERROR_BLOCK_COUNT:
            tvp->u.st = c->block_error;
            break;
        case DTV_STAT_TOTAL_BLOCK_COUNT:
            tvp->u.st = c->block_count;
            break;
        default:
            TRACE(TRACE_FRONTEND,"Unsupported frontend property (%d)\n", tvp->cmd);
            err = EINVAL;
    }

    return (err);
}

/* Set an array of frontend properties
   Upstream name: dvb_frontend_ioctl_properties()/FE_SET_PROPERTY */
int
sms1xxx_frontend_set_properties(struct sms1xxx_softc *sc,
    struct dtv_properties *parg)
{
    TRACE(TRACE_IOCTL,"\n");
    struct dtv_property *tvp = NULL;
    struct dtv_frontend_properties *c = &sc->dtv_property_cache;
    int err = 0;
    int i;

    /* Put an arbitrary limit on the number of messages that can
     * be sent at once */
    if ((parg->num == 0) || (parg->num > DTV_IOCTL_MAX_MSGS))
        return (EINVAL);

    tvp = malloc(parg->num * sizeof(struct dtv_property), M_USBDEV, M_WAITOK);
    if (tvp == NULL) {
        return (ENOMEM);
    }

    memcpy(tvp, parg->props, parg->num * sizeof(struct dtv_property));
    for (i = 0; i < parg->num; i++) {
        err = sms1xxx_frontend_set_single_property(sc, tvp + i);
        (tvp + i)->result = err;
        if (err != 0) {
            memcpy(parg->props, tvp, parg->num * sizeof(struct dtv_property));
            free(tvp, M_USBDEV);
            return (err);
        }
    }
    memcpy(parg->props, tvp, parg->num * sizeof(struct dtv_property));
    free(tvp, M_USBDEV);

    /* Check cache state and tune if DTV_TUNE property has been set */
    if (c->state == DTV_TUNE) {
        TRACE(TRACE_FRONTEND,"Property cache is full, tuning! sc=%p\n",sc);
        return (sms1xxx_usb_setfrequency(
            sc,
            sc->fe_params.frequency + sc->device->freq_offset,
            sc->fe_params.u.ofdm.bandwidth
        ));
        /* TODO: reset c->state ? */
    }

    return (0);
}

/* Get an array of frontend properties
   Upstream name: dvb_frontend_ioctl_properties()/FE_GET_PROPERTY */
int
sms1xxx_frontend_get_properties(struct sms1xxx_softc *sc,
    struct dtv_properties *parg)
{
    TRACE(TRACE_IOCTL,"\n");
    struct dtv_property *tvp = NULL;
    int err = 0;
    int i;

    /* Put an arbitrary limit on the number of messages that can
     * be sent at once */
    if ((parg->num == 0) || (parg->num > DTV_IOCTL_MAX_MSGS))
        return (EINVAL);

    tvp = malloc(parg->num * sizeof(struct dtv_property), M_USBDEV, M_WAITOK);
    if (tvp == NULL) {
        return (ENOMEM);
    }

    memcpy(tvp, parg->props, parg->num * sizeof(struct dtv_property));
    for (i = 0; i < parg->num; i++) {
        err = sms1xxx_frontend_get_single_property(sc, tvp + i);
        (tvp + i)->result = err;
        if (err != 0) {
            memcpy(parg->props, tvp, parg->num * sizeof(struct dtv_property));
            free(tvp, M_USBDEV);
            return (err);
        }
    }
    memcpy(parg->props, tvp, parg->num * sizeof(struct dtv_property));
    free(tvp, M_USBDEV);

    return (0);
}
