/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2010, Ganaël Laplanche, http://contribs.martymac.org
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
    int err = 0;
    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1)
        err = sms1xxx_usb_getstatistics(sc);
    if(!err) *status=sc->fe_status;
    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_ber(struct sms1xxx_softc *sc, u32 *ber)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = 0;
    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1)
        err = sms1xxx_usb_getstatistics(sc);
    if(!err) *ber=sc->sms_stat_dvb.ReceptionData.BER;
    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_ucblocks(struct sms1xxx_softc *sc, u32 *ucblocks)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = 0;
    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1)
        err = sms1xxx_usb_getstatistics(sc);
    if(!err) *ucblocks=sc->sms_stat_dvb.ReceptionData.ErrorTSPackets;
    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_signal_strength(struct sms1xxx_softc *sc, u16 *strength)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = 0;
    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1)
        err = sms1xxx_usb_getstatistics(sc);
    if(!err) {
        if (sc->sms_stat_dvb.ReceptionData.InBandPwr < -95)
            *strength = 0;
        else if (sc->sms_stat_dvb.ReceptionData.InBandPwr > -29)
            *strength = 100;
        else
            *strength = (sc->sms_stat_dvb.ReceptionData.InBandPwr + 95) * 3 / 2;
    }
    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_read_snr(struct sms1xxx_softc *sc, u16 *snr)
{
    TRACE(TRACE_IOCTL,"\n");
    int err = 0;
    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1)
        err = sms1xxx_usb_getstatistics(sc);
    if(!err) *snr=sc->sms_stat_dvb.ReceptionData.SNR;
    sms1xxx_frontend_status_led_feedback(sc);
    return (err);
}

int
sms1xxx_frontend_set_frontend(struct sms1xxx_softc *sc,
    struct dvb_frontend_parameters *fep)
{
    TRACE(TRACE_IOCTL,"\n");
    return (sms1xxx_usb_setfrequency(
        sc,
        fep->frequency + sc->device->freq_offset,
        fep->u.ofdm.bandwidth
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
