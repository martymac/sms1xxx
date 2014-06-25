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
 *  Demuxer implementation
 *  (handles dvr0 and demux0.x devices).
 * * * *
 */

/* uio(9) stuff */
#include <sys/types.h>
#include <sys/uio.h>

/* poll(2) stuff */
#include <sys/poll.h>

/* File-descriptor ioctls */
#include <sys/filio.h>

/* Read flags (O_NONBLOCK) */
#include <sys/fcntl.h>

/* Our stuff */
#include "sms1xxx.h"
#include "sms1xxx-demux.h"
#include "sms1xxx-usb.h"

/* Linux stuff */
#include "linux/dvb/dmx.h"

/********************************
 * Dvr (private functions only) *
 ********************************/

static d_open_t  sms1xxx_dvr_open;
static d_close_t sms1xxx_dvr_close;
static d_read_t sms1xxx_dvr_read;
static d_poll_t sms1xxx_dvr_poll;

static struct cdevsw sms1xxx_dvr_cdevsw = {
    .d_version  = D_VERSION,
    .d_flags    = D_NEEDGIANT,
    .d_open     = sms1xxx_dvr_open,
    .d_close    = sms1xxx_dvr_close,
    .d_read     = sms1xxx_dvr_read,
    .d_poll     = sms1xxx_dvr_poll,
    .d_name     = "sms1xxx_dvr",
};

static int
sms1xxx_dvr_open(struct cdev *dev, int flag, int mode, struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
#ifdef SMS1XXX_DEBUG
    int unit = device_get_unit(sc->sc_dev);
    TRACE(TRACE_OPEN,"flag=%d, mode=%d, unit=%d\n", flag, mode, unit);
#endif

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    if(sc->dvr.state & DVR_OPEN)
        return (EBUSY);

    sc->dvr.state |= DVR_OPEN;
    return (0);
}

static int
sms1xxx_dvr_close(struct cdev *dev, int flag, int mode, struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
#ifdef SMS1XXX_DEBUG
    int unit = device_get_unit(sc->sc_dev);
    TRACE(TRACE_OPEN,"flag=%d, mode=%d, filter=%d\n",
        flag, mode, unit);
#endif
    /* Wake up readers ! */
    if(sc->dvr.state & DVR_SLEEP) {
        wakeup(&sc->dvr);
    }
    if(sc->dvr.state & DVR_POLL) {
        sc->dvr.state &= ~DVR_POLL;
        selwakeuppri(&sc->dvr.rsel,PZERO);
    }

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    sc->dvr.state &= ~DVR_OPEN;
    return (0);
}

static inline int
sms1xxx_dvr_get_data(struct sms1xxx_softc *sc, struct uio *uio)
{
    int err = 0;
    u32 todo, total;
#if 0
    sc->dvr.threshold = MIN(uio->uio_resid, DVRBUFSIZE/3);
    if(sc->dvr.threshold < PACKET_SIZE)
        sc->dvr.threshold = PACKET_SIZE;
#endif
    if(sc->dvr.ravail < sc->dvr.threshold)
        return (EBUSY);

    TRACE(TRACE_DVR_READ,"wants: %zu bytes, got: %u bytes\n",
        uio->uio_resid, sc->dvr.ravail);

    total = MIN(uio->uio_resid, sc->dvr.ravail);
    todo = MIN(total, sc->dvr.size - sc->dvr.roff);

    if(total > sc->dvr.size) {
        ERR("total > dvrsize, this shouldn't happen! %d > %d\n",
            total, sc->dvr.size);
        return (-EOVERFLOW);
    }

    err = uiomove(sc->dvr.buf + sc->dvr.roff, todo, uio);
    if(err)
        return (-err);

    if(todo < total) {
        sc->dvr.roff = total - todo;
        err = uiomove(sc->dvr.buf, sc->dvr.roff, uio);
        if(err)
            return (-err);
    }
    else {
        sc->dvr.roff += total;
    }
    if(sc->dvr.roff == sc->dvr.size)
        sc->dvr.roff = 0;

    mtx_lock(&sc->dvr.lock);
    sc->dvr.ravail -= total;
    sc->dvr.wavail += total;
#ifdef SMS1XXX_DIAGNOSTIC
       if(sc->dvr.wavail > sc->stats.max_wavail)
           sc->stats.max_wavail = sc->dvr.wavail;
       if(sc->dvr.ravail < sc->stats.min_ravail)
           sc->stats.min_ravail = sc->dvr.ravail;
#endif
    mtx_unlock(&sc->dvr.lock);
    return (uio->uio_resid);
}

static int
sms1xxx_dvr_read(struct cdev *dev, struct uio *uio, int flag)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
    int err = 0;

    TRACE(TRACE_DVR_READ,"\n");

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    if(sc->dvr.state & DVR_SLEEP) {
        ERR("read already in progress\n");
        return (EBUSY);
    }

    while((err = sms1xxx_dvr_get_data(sc,uio)) > 0) {
        if(flag & O_NONBLOCK)
            return (EWOULDBLOCK);

        sc->dvr.state |= DVR_SLEEP;
        err = tsleep(&sc->dvr, PZERO | PCATCH, "dvrrd", 0);
        sc->dvr.state &= ~DVR_SLEEP;

        if(sc->sc_dying) {
            TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
            return (ENXIO);
        }
        else if(err)
            return (err);
    }
    return (-err);
}

static int
sms1xxx_dvr_poll(struct cdev *dev, int events, struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
    int revents = 0;
    TRACE(TRACE_DVR_READ,"thread=%p, events=%d\n", p, events);

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return ((events & (POLLIN | POLLOUT | POLLRDNORM |
            POLLWRNORM)) | POLLHUP);
    }

    /* XXX POLLERR POLLPRI */
    if(events & (POLLIN | POLLRDNORM)) {
        if(sc->dvr.ravail < sc->dvr.threshold) { 
            selrecord(p,&sc->dvr.rsel);
            sc->dvr.state |= DVR_POLL;
            TRACE(TRACE_DVR_READ,"will block\n");
        }
        else {
            revents |= events & (POLLIN | POLLRDNORM);
            TRACE(TRACE_DVR_READ,"poll not blocking\n");
        }
    }
    if(events & (POLLOUT | POLLWRNORM)) {
        /* Write is not allowed, so no point blocking on it */
        revents |= events & (POLLOUT | POLLWRNORM);
    }
    return (revents);
}

/***********
 * Demuxer *
 ***********/

static d_open_t  sms1xxx_demux_open;
static d_close_t sms1xxx_demux_close;
static d_ioctl_t sms1xxx_demux_ioctl;
static d_read_t sms1xxx_demux_read;
static d_poll_t sms1xxx_demux_poll;

static struct cdevsw sms1xxx_demux_cdevsw = {
    .d_version  = D_VERSION,
    .d_flags    = D_NEEDGIANT | D_NEEDMINOR,
    .d_open     = sms1xxx_demux_open,
    .d_close    = sms1xxx_demux_close,
    .d_ioctl    = sms1xxx_demux_ioctl,
    .d_read     = sms1xxx_demux_read,
    .d_poll     = sms1xxx_demux_poll,
    .d_name     = "sms1xxx_demux",
};

static int
sms1xxx_demux_write_section(struct sms1xxx_softc *sc, u8 *p,
    struct filter *f);

static void
sms1xxx_demux_sectbuf_reset(struct sms1xxx_softc *sc,
    struct filter *f, int wake, char *msg);

static void
sms1xxx_demux_clone(void *arg, struct ucred *cred, char *name,
    int namelen, struct cdev **dev);

/*****************************
 * Demuxer private functions *
 *****************************/

/* Callback function for destroy_dev_sched_cb()
   Avoids race with dev_rel() when fastly opening/closing demux0 */
static void
sms1xxx_demux_filter_release(void *arg)
{
    struct filter *f = arg;
    f->dev = NULL;
    f->pid = PIDFREE;
    return;
}

/* Called on each sms1xxx_demux_start() */
static int
sms1xxx_demux_stream_ref(struct sms1xxx_softc *sc)
{
    int err = 0;
    TRACE(TRACE_USB,"refcnt %d -> %d\n",sc->streamrefs,sc->streamrefs + 1);
    sc->streamrefs++;

    /*
        if(sc->streamrefs == 1) {
            err = sms1xxx_usb_stream_start(sc);
            if(err)
                sc->streamrefs = 0;
        }
    */
    return (err);
}

/* Called on each sms1xxx_demux_stop() */
static int
sms1xxx_demux_stream_deref(struct sms1xxx_softc *sc)
{
    int err = 0;
    TRACE(TRACE_USB,"refcnt %d -> %d\n",sc->streamrefs,sc->streamrefs - 1);
    sc->streamrefs--;

    /*
        XXX Find a way to completely stop the device when no more PID set ?
        if(sc->streamrefs <= 0) {
            err = sms1xxx_usb_stream_stop(sc);
            if(err)
                sc->streamrefs++;
        }
    */
    return (err);
}

static int
sms1xxx_demux_start(struct sms1xxx_softc *sc, struct filter *f)
{
    int err = 0;
    TRACE(TRACE_FILTERS, "sc=%p, filter=%p\n", sc, f);
    if(f->pid == PIDEMPTY)
        return (EINVAL);

    f->pid &= ~PIDSTOPPED;
    if(!(f->state & FILTER_STREAMING)) {
        if((err = sms1xxx_demux_stream_ref(sc)) == 0) 
            f->state |= FILTER_STREAMING;
    }
    return (err);
}

static int
sms1xxx_demux_stop(struct sms1xxx_softc *sc, struct filter *f)
{
    int err = 0;
    TRACE(TRACE_FILTERS, "sc=%p, filter=%p\n", sc, f);
    if(f->pid == PIDEMPTY)
        return (EINVAL);

    if(f->state & FILTER_STREAMING) {
        if((err = sms1xxx_demux_stream_deref(sc)) == 0)
            f->state &= ~FILTER_STREAMING;
    }
    f->pid |= PIDSTOPPED;
    return (err);
}

static void
sms1xxx_demux_sectbuf_reset(struct sms1xxx_softc *sc, struct filter *f,
    int wake, char *msg)
{
    TRACE(TRACE_SECT,"pid: %hu reason: %s\n", f->pid, msg);
    mtx_lock(&sc->filterlock);
    f->woff = 0;
    f->roff = 0;
    f->wavail = SECTBUFSIZE;
    f->ravail = 0;
    f->wtodo = 0;
    f->rtodo = 0;
    f->size = SECTBUFSIZE;
    f->sectcnt = 0;
    mtx_unlock(&sc->filterlock);

    if(wake) {
        f->state |= FILTER_OVERFLOW;
        /* Wake up readers ! */
        if(f->state & FILTER_SLEEP)
            wakeup(f);
        if(f->state & FILTER_POLL) {
            f->state &= ~FILTER_POLL;
            selwakeuppri(&f->rsel, PZERO);
        }
    }
}

static int
sms1xxx_demux_write_section(struct sms1xxx_softc *sc, u8 *p,
    struct filter *f)
{
    u8 hlen, len, total, count;
    u8 *payload;
    if(!TS_HAS_PAYLOAD(p))
        return (1);

    hlen = TS_GET_HDR_LEN(p);
    if(hlen >= PACKET_SIZE) {
        sms1xxx_demux_sectbuf_reset(sc, f, 1, "corrupt packet");
        return (1);
    }

    payload = p + hlen;
    len = PACKET_SIZE - hlen;

    if(TS_HAS_PUSI(p)) {
        u8 off = TS_GET_SECT_OFF(payload);
        if((hlen + off + 3) > PACKET_SIZE) {
            sms1xxx_demux_sectbuf_reset(sc, f, 1, "corrupt packet");
            return (1);
        }
#ifdef SMS1XXX_DIAGNOSTIC
        /* XXX Check if the old section is finished */
        if(f->wtodo != 0) {
            WARN("pid: %hu section not finished but new pusi "
                "hlen %d off %d todo %d\n", f->pid,
                hlen, off, f->wtodo);
        }
#endif
        /* Section start */
        payload += off; len -= off;
        if((TS_GET_SECT_TBLID(payload) & f->mask) != f->value) {
            f->wtodo = 0;
            return (0);
        }
        f->wtodo = TS_GET_SECT_LEN(payload);
        TRACE(TRACE_SECT,"pid: %hu writing new section len: %d\n",
            f->pid, f->wtodo);
    }
    if(f->wtodo == 0)
        return (0);

    total = MIN(f->wtodo,len);
    if(total > f->wavail) {
#ifdef SMS1XXX_DIAGNOSTIC
        WARN("pid: %hu no section buffer space available\n",f->pid);
#endif
        sms1xxx_demux_sectbuf_reset(sc, f, 1, "no buffer space");
        return (0);
    }
    if(total > f->size) {
        ERR("total > f->size, this shouldn't happen! %d > %d\n",
            total,f->size);
        return (1);
    }

    TRACE(TRACE_SECT,"pid: %hu writing %d bytes to section\n",
        f->pid,total);
    count = MIN(total,f->size - f->woff);
    memcpy(f->buf + f->woff,payload,count);

    if(count < total) {
        f->woff = total - count;
        memcpy(f->buf,payload+count,f->woff);
    }
    else {
        f->woff += total;
    }
    if(f->woff == f->size)
        f->woff = 0;

    f->wtodo -= total;
    if(f->wtodo == 0) {
        mtx_lock(&sc->filterlock);
        f->wavail -= total;
        f->ravail += total;
        f->sectcnt++;
        f->state &= ~FILTER_OVERFLOW;
        mtx_unlock(&sc->filterlock);
        TRACE(TRACE_SECT,"pid: %hu writing section done sectcnt: %d\n"
            ,f->pid, f->sectcnt);

        /* Wake up readers ! */
        if(f->state & FILTER_SLEEP)
            wakeup(f);
        if(f->state & FILTER_POLL) {
            f->state &= ~FILTER_POLL;
            selwakeuppri(&f->rsel, PZERO);
        }
    }
    else {
        mtx_lock(&sc->filterlock);
        f->wavail -= total;
        f->ravail += total;
        mtx_unlock(&sc->filterlock);
    }
#ifdef SMS1XXX_DIAGNOSTIC
    if(total < len && TS_GET_SECT_TBLID(payload+total) != 0xFF)
        WARN("possible section(s) discarded\n");
#endif
    return (0);
}

static void
sms1xxx_demux_clone(void *arg, struct ucred *cred, char *name,
    int namelen, struct cdev **dev)
{
    char buf[40];
    int filtnr = 0;
    int unit;
    struct sms1xxx_softc *sc = arg;

    TRACE(TRACE_OPEN,"name=%s\n",name);

    if (*dev != NULL || sc == NULL)
        return;

    snprintf(buf,39,"dvb/adapter%d/demux0",device_get_unit(sc->sc_dev));
    if(strcmp(name,buf) != 0)
        return;

    for(filtnr = 0; filtnr < MAX_FILTERS; ++filtnr) {
        if(sc->filter[filtnr].pid == PIDFREE)
            break;
    }

    if(filtnr >= MAX_FILTERS) {
        ERR("no more filter free\n");
        return;
    }

    if(sc->filter[filtnr].dev != NULL) {
        ERR("filter %d not free\n", filtnr);
        return;
    }

    /* Clones are shared among devices, so handle unit numbers
       manually by reserving a range of MAX_FILTERS units per-device
       (clone_create will fail setting unit number automatically :
       unit number assignment is done by crawling sc's (only) clones
       and will not detect clones created for other devices, resulting
       in trying to set twice the same unit number). */
    unit = device_get_unit(sc->sc_dev) * MAX_FILTERS + filtnr;

    if(clone_create(&sc->demux_clones, &sms1xxx_demux_cdevsw, &unit, dev, 0)) {
        *dev = make_dev_credf(MAKEDEV_REF, &sms1xxx_demux_cdevsw,
            unit, NULL, UID_ROOT, GID_WHEEL,
            0666, "dvb/adapter%d/demux0.%d",
            device_get_unit(sc->sc_dev), filtnr);
        if(*dev != NULL) {
            TRACE(TRACE_MODULE,"created demux0.%d device, addr=%p\n",
                filtnr, *dev);
            (*dev)->si_flags |= SI_CHEAPCLONE;
            (*dev)->si_drv1 = sc;
            (*dev)->si_drv2 = &sc->filter[filtnr];  /* map filter to device */
            sc->filter[filtnr].dev = *dev;          /* map device to filter */
            sc->filter[filtnr].pid = PIDCLONED;
        }
    }
}

static int
sms1xxx_demux_open(struct cdev *dev, int flag, int mode, struct thread *p)
{
    int err = 0;
    struct sms1xxx_softc *sc;
    struct filter *f;

    TRACE(TRACE_OPEN,"flag=%d, mode=%d, f=%p\n", flag, mode, dev->si_drv2);

    sc = dev->si_drv1;
    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }

    f = dev->si_drv2;
    if (f == NULL) {
        ERR("no filter found\n");
        return (ENOENT);
    }
    if(f->pid != PIDCLONED) {
        TRACE(TRACE_OPEN,"busy!\n");
        return (EBUSY);
    }

    err = sms1xxx_usb_ref(sc);
    if(!err)
        f->pid = PIDEMPTY;

    return (err);
}

static inline int
sms1xxx_demux_read_section(struct sms1xxx_softc *sc, struct filter *f,
    struct uio *uio)
{
    int err = 0;
    u32 roff, rtodo, count, total;
    TRACE(TRACE_SECT,"pid: %hu sectcnt: %d overflow: %s\n", f->pid,
        f->sectcnt, f->state & FILTER_OVERFLOW ? "yes" : "no" );

    if(f->sectcnt == 0)
        return (EBUSY);
    else if(f->state & FILTER_OVERFLOW) {
        f->state &= ~FILTER_OVERFLOW;
        return (-EOVERFLOW);
    }

    mtx_lock(&sc->filterlock);
    if(f->rtodo == 0) {
        if(f->roff <= (f->size - 3)) {
            f->rtodo = TS_GET_SECT_LEN(f->buf + f->roff);
        }
        else if(f->roff == (f->size - 2)) {
            f->rtodo = ((((((f->buf[f->roff+1]) << 8)|(f->buf[0])))
                & 0xFFF) + 3);
        }
        else {
            f->rtodo = TS_GET_SECT_LEN(f->buf - 1);
        }
        TRACE(TRACE_SECT,"pid: %hu reading new section len: %d\n",
            f->pid, f->rtodo);
    }

    rtodo = f->rtodo;
    roff = f->roff;
    total = MIN(uio->uio_resid,rtodo);
    count = MIN(total, f->size - roff);
    mtx_unlock(&sc->filterlock);

    if(total > f->size) {
        ERR("total > fsize, this shouldn't happen! %d > %d\n",
            total, f->size);
        return (-EOVERFLOW);
    }

    TRACE(TRACE_SECT,"pid: %hu reading %d bytes from section\n",
        f->pid, total);

    err = uiomove(f->buf + roff, count, uio);
    if(err)
        return (-err);

    if(count < total) {
        roff = total - count;
        err = uiomove(f->buf,roff, uio);
        if(err)
            return (-err);
    }
    else {
        roff += total;
    }
    if(roff == f->size)
        roff = 0;

    rtodo -= total;
    if(rtodo == 0) {
        TRACE(TRACE_SECT,"pid: %hu reading section done sectcnt: %d\n"
            ,f->pid, f->sectcnt - 1);
    }
    mtx_lock(&sc->filterlock);
    if(f->state & FILTER_OVERFLOW) {
        f->state &= ~FILTER_OVERFLOW;
        mtx_unlock(&sc->filterlock);
        if(rtodo == 0)
            return (0);
        else
            return (-EOVERFLOW);
    }
    f->sectcnt--;
    f->rtodo = rtodo;
    f->roff = roff;
    f->ravail -= total;
    f->wavail += total;
    mtx_unlock(&sc->filterlock);
    return (0);
}

static int
sms1xxx_demux_read(struct cdev *dev, struct uio *uio, int flag)
{
    int err = 0;
    struct sms1xxx_softc *sc;
    struct filter *f;

    TRACE(TRACE_READ,"flag=%d, f=%p\n", flag, dev->si_drv2);

    sc = dev->si_drv1;
    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }

    f = dev->si_drv2;
    if (f == NULL) {
        ERR("no filter found\n");
        return (EBADF);
    }
    if(f->state & FILTER_SLEEP) {
        ERR("read already in progress\n");
        return (EBUSY);
    }

    while((err = sms1xxx_demux_read_section(sc,f,uio)) > 0) {
        if(flag & O_NONBLOCK)
            return (EWOULDBLOCK);

        f->state |= FILTER_SLEEP;
        err = tsleep(f, PZERO | PCATCH, "dmxrd", 0);
        f->state &= ~FILTER_SLEEP;

        if(sc->sc_dying) {
            TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
            return (ENXIO);
        }
        else if(err)
            return (err);
    }
    return (-err);
}

static int
sms1xxx_demux_poll(struct cdev *dev, int events, struct thread *p)
{
    int revents = 0;
    struct sms1xxx_softc *sc;
    struct filter *f;

    TRACE(TRACE_POLL,"thread=%p, events=%d, f=%p\n", p, events, dev->si_drv2);

    sc = dev->si_drv1;
    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return ((events & (POLLIN | POLLOUT | POLLRDNORM |
            POLLWRNORM)) | POLLHUP);
    }

    f = dev->si_drv2;
    if (f == NULL) {
        ERR("no filter found\n");
        return (EFAULT);
    }

    /* XXX POLLERR POLLPRI */
    if(events & (POLLIN | POLLRDNORM)) {
        if(f->sectcnt == 0) { 
            selrecord(p,&f->rsel);
            f->state |= FILTER_POLL;
            TRACE(TRACE_POLL,"will block\n");
        }
        else {
            revents |= events & (POLLIN | POLLRDNORM);
            TRACE(TRACE_POLL,"poll not blocking\n");
        }
    }
    if(events & (POLLOUT | POLLWRNORM)) {
        /* Write is not allowed, so no point blocking on it */
        revents |= events & (POLLOUT | POLLWRNORM);
    }
    return (revents);
}

static int
sms1xxx_demux_close(struct cdev *dev, int flag, int mode, struct thread *p)
{
    struct sms1xxx_softc *sc;
    struct filter *f;

    TRACE(TRACE_OPEN,"flag=%d, mode=%d, f=%p\n", flag, mode, dev->si_drv2);

    sc = dev->si_drv1;
    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    f = dev->si_drv2;
    if (f == NULL) {
        ERR("no filter found\n");
        return (EBADF);
    }

    /* Remove PID filter if not already done */
    if(!(f->pid & PIDSTOPPED)) {
        if(pid_value(f->pid) <= PIDMAX)
            sc->device->pid_filter(sc,pid_value(f->pid),0);
        sms1xxx_demux_stop(sc,f);
    }
    f->pid = PIDEMPTY;

    sms1xxx_usb_deref(sc);
    if(f->buf != NULL) {
        free(f->buf,M_USBDEV);
        f->buf = NULL;
    }
    f->pid = PIDCLONED;

    /* Destroy the clone */
    TRACE(TRACE_MODULE,"destroying demux device, addr=%p\n", dev);
    dev->si_drv1 = NULL;
    dev->si_drv2 = NULL;
    destroy_dev_sched_cb(dev, sms1xxx_demux_filter_release, f);

    return (0);
}

static int
sms1xxx_demux_ioctl(struct cdev *dev, u_long cmd, caddr_t addr, int flag,
    struct thread *p)
{
    int err = 0;
    struct sms1xxx_softc *sc;
    struct filter *f;
    void *arg = addr;

    TRACE(TRACE_MODULE,"f=%p\n", dev->si_drv2);

    sc = dev->si_drv1;
    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }

    f = dev->si_drv2;
    if (f == NULL) {
        ERR("no filter found\n");
        return (EBADF);
    }

    if(f->state & FILTER_BUSY) {
        ERR("ioctl on filter in progress\n");
        return (EBUSY);
    }

    /* Just return 0 for this special ioctl */
    if(cmd == FIONBIO)
        return (err);

    f->state |= FILTER_BUSY;
    switch(cmd) {
    case DMX_START:
        if(pid_value(f->pid) > PIDMAX)
            err = EINVAL;
        if(!err)
            err = sc->device->pid_filter(sc,pid_value(f->pid),1);
        if(!err)
            err = sms1xxx_demux_start(sc,f);
        TRACE(TRACE_IOCTL,"DMX_START=%d\n",err);
        break;
    case DMX_STOP:
        if(pid_value(f->pid) <= PIDMAX) {
            err = sc->device->pid_filter(sc,pid_value(f->pid),0);
            if(!err)
                err = sms1xxx_demux_stop(sc,f);
        }
        TRACE(TRACE_IOCTL,"DMX_STOP=%d\n",err);
        break;
    case DMX_SET_FILTER:
        {
            /* XXX Not fully implemented: no crc checking and
             * only uses first byte for filtering (table id)
             */
            struct dmx_sct_filter_params *p = arg;
            TRACE(TRACE_IOCTL,"DMX_SET_FILTER  (pid=%d, value=%d, f=%p)\n",
                p->pid, p->filter.filter[0], f);

            err = EINVAL;
            if(p->flags & DMX_ONESHOT) {
                ERR("oneshot filter not supported yet\n");
                break;
            }
            if(p->pid > PIDMAX) {
                ERR("invalid pid: %hu\n",p->pid);
                break;
            }
            /* cancel previous filter, if any */
            if(pid_value(f->pid) <= PIDMAX) {
                sc->device->pid_filter(sc,pid_value(f->pid),0);
                sms1xxx_demux_stop(sc,f);
            }

            /* XXX p->timeout */
            f->pid = p->pid|PIDSTOPPED;
            if(f->buf == NULL) {
                f->buf = malloc(SECTBUFSIZE,M_USBDEV,M_WAITOK);
                if(f->buf == NULL) {
                    ERR("failed to allocate filter buf\n");
                    err = ENOMEM;
                    f->pid = PIDEMPTY;
                    break;
                }
            }

            sms1xxx_demux_sectbuf_reset(sc, f, 0, "init");
            f->type = FILTER_TYPE_SECT;
            f->mask = p->filter.mask[0];
            f->value = p->filter.filter[0];

            if(p->flags & DMX_IMMEDIATE_START) {
                err = sc->device->pid_filter(sc,p->pid,1);
                if(!err)
                    err = sms1xxx_demux_start(sc,f);
            }
            break;
        }
    case DMX_SET_PES_FILTER:
        {
            struct dmx_pes_filter_params *p = arg;
            TRACE(TRACE_IOCTL,"DMX_SET_PES_FILTER (pid=%d, f=%p)\n", p->pid, f);

            err = EINVAL;
            if(p->input != DMX_IN_FRONTEND) {
                ERR("only frontend as input supported\n");
                break;
            }
            if(p->output != DMX_OUT_TS_TAP) {
                ERR("only dvr as output supported\n");
                break;
            }
            if(p->pid > PIDMAX) {
                ERR("invalid pid: %hu\n",p->pid);
                break;
            }
            /* cancel previous filter, if any */
            if(pid_value(f->pid) <= PIDMAX) {
                sc->device->pid_filter(sc,pid_value(f->pid),0);
                sms1xxx_demux_stop(sc,f);
            }

            f->pid = p->pid|PIDSTOPPED;
            f->type = FILTER_TYPE_PES;

            if(p->flags & DMX_IMMEDIATE_START) {
                err = sc->device->pid_filter(sc,p->pid,1);
                if(!err)
                    err = sms1xxx_demux_start(sc,f);
            }
            break;
        }
    case DMX_SET_BUFFER_SIZE:
        err = ENOTTY;
        ERR("DMX_SET_BUFFER_SIZE ioctl not implemented\n");
        break;
    case DMX_GET_STC:
        err = ENOTTY;
        ERR("DMX_GET_STC ioctl not implemented\n");
        break;
#ifdef SMS1XXX_DIAGNOSTIC
    case SMS1XXX_GET_STATS:
        {
            struct sms1xxx_stats *stats = arg;
            memcpy(stats,&sc->stats,sizeof(struct sms1xxx_stats));
            stats->threshold = sc->dvr.threshold;
            sc->stats.interrupts = 0;
            sc->stats.bytes = 0;
            sc->stats.packetsmatched = 0;
            sc->stats.max_wavail = 0;
            sc->stats.min_wavail = sc->dvr.size;
            sc->stats.max_ravail = 0;
            sc->stats.min_ravail = sc->dvr.size;
            break;
        }
#endif
    default:
        err = ENOTTY;
        ERR("unknown ioctl: 0x%lx\n",cmd);
        break;
    }
    f->state &= ~FILTER_BUSY;
    return (err);
}

/****************************
 * Demuxer public functions *
 ****************************/

/* Called from attach */
int
sms1xxx_demux_init(struct sms1xxx_softc *sc)
{
    /* Mutexes */
    mtx_init(&sc->dvr.lock, "DVB Dvr lock", NULL, MTX_DEF);
    mtx_init(&sc->filterlock, "DVB Filter lock", NULL, MTX_DEF);

    /* DVR */
    sc->dvr.threshold = THRESHOLD;
    sc->dvr.size = DVRBUFSIZE;
    sc->dvr.buf = malloc(sc->dvr.size, M_USBDEV, M_WAITOK);
    if(sc->dvr.buf == NULL) {
        ERR("could not allocate dvr buf\n");
        mtx_destroy(&sc->filterlock);
        mtx_destroy(&sc->dvr.lock);
        return (ENOMEM);
    }
    sms1xxx_demux_pesbuf_reset(sc, 0, "init");
    sc->dvr.state = 0;

    /* Filters */
    int i;
    for(i = 0; i < MAX_FILTERS; ++i) {
        sms1xxx_demux_filter_release(&sc->filter[i]);
    }

    /* Devices */
    clone_setup(&sc->demux_clones);
    sc->clonetag = EVENTHANDLER_REGISTER(dev_clone, sms1xxx_demux_clone,
                         sc, 1000);
    sc->dvr.dev = make_dev(&sms1xxx_dvr_cdevsw,
        device_get_unit(sc->sc_dev),
        UID_ROOT, GID_WHEEL, 0666,
        "dvb/adapter%d/dvr0",
        device_get_unit(sc->sc_dev));
    if (sc->dvr.dev != NULL)
        sc->dvr.dev->si_drv1 = sc;

    TRACE(TRACE_MODULE,"created dvr0 device, addr=%p\n",sc->dvr.dev);
    return (0);
}

/* Called from detach */
void
sms1xxx_demux_exit(struct sms1xxx_softc *sc)
{
    /* Wake up readers ! */
    if(sc->dvr.state & DVR_SLEEP) {
        wakeup(&sc->dvr);
    }
    if(sc->dvr.state & DVR_POLL) {
        sc->dvr.state &= ~DVR_POLL;
        selwakeuppri(&sc->dvr.rsel,PZERO);
    }

    /* Devices */
    if(sc->dvr.dev != NULL) {
        TRACE(TRACE_MODULE,"destroying dvr0, addr=%p\n",sc->dvr.dev);
        destroy_dev(sc->dvr.dev);
        sc->dvr.dev = NULL;
    }
    if(sc->clonetag != NULL) {
        EVENTHANDLER_DEREGISTER(dev_clone,sc->clonetag);
        sc->clonetag = NULL;
    }

    /* Destroy remaining clones */
    for(int i = 0; i < MAX_FILTERS; ++i) {
        if((sc->filter[i].pid != PIDFREE) &&
            (sc->filter[i].dev != NULL)) {
            TRACE(TRACE_MODULE,"destroying demux0.%d device, addr=%p\n",
                i,sc->filter[i].dev);
            destroy_dev_sched_cb(sc->filter[i].dev,
                sms1xxx_demux_filter_release, &sc->filter[i]);
        }
    }
    if(sc->demux_clones != NULL) {
        drain_dev_clone_events();
        clone_cleanup(&sc->demux_clones);
        destroy_dev_drain(&sms1xxx_demux_cdevsw);
        sc->demux_clones = NULL;
    }

    /* DVR */
    sc->dvr.state = 0;
    sms1xxx_demux_pesbuf_reset(sc, 0, "exit");
    if(sc->dvr.buf != NULL) {
        free(sc->dvr.buf, M_USBDEV);
        sc->dvr.buf = NULL;
    }

    /* Mutexes */
    mtx_destroy(&sc->filterlock);
    mtx_destroy(&sc->dvr.lock);
}

/* Called from usb interrupt callback */
void
sms1xxx_demux_put_packet(struct sms1xxx_softc *sc, u8 *p)
{
    int i, done = 0, dvrdone = 0;
    u16 pid = TS_GET_PID(p);
#ifdef SMS1XXX_DIAGNOSTIC
    u8 cc = TS_GET_CC(p);
#endif

    for(i = 0; i < MAX_FILTERS && !done ; ++i) {
         struct filter *f = &sc->filter[i];
         if(pid != f->pid && f->pid != PIDALL) {
             continue;
         }
         /* dvr0 device */
         if(f->type == FILTER_TYPE_PES && !dvrdone) {
             if(sc->dvr.wavail >= PACKET_SIZE) {
                 memcpy(sc->dvr.buf+sc->dvr.woff,p,PACKET_SIZE);
                 sc->dvr.woff += PACKET_SIZE;
                 if(sc->dvr.woff == sc->dvr.size)
                     sc->dvr.woff = 0;

                 mtx_lock(&sc->dvr.lock);
                 sc->dvr.wavail -= PACKET_SIZE;
                 sc->dvr.ravail += PACKET_SIZE;
#ifdef SMS1XXX_DIAGNOSTIC
                 if(sc->dvr.wavail < sc->stats.min_wavail)
                     sc->stats.min_wavail = sc->dvr.wavail;
                 if(sc->dvr.ravail > sc->stats.max_ravail)
                     sc->stats.max_ravail = sc->dvr.ravail;
#endif
                 mtx_unlock(&sc->dvr.lock);
                 if(sc->dvr.nobufs != 0) {
                     WARN("%u packets dropped due to no "
                         "buffer space\n", sc->dvr.nobufs);
                     sc->dvr.nobufs = 0;
                 }
             }
             else {
                 sc->dvr.nobufs++;
             }
             if(sc->dvr.ravail >= sc->dvr.threshold) {
                 /* Wake up readers ! */
                 if(sc->dvr.state & DVR_SLEEP)
                     wakeup(&sc->dvr);
                 if(sc->dvr.state & DVR_POLL) {
                     sc->dvr.state &= ~DVR_POLL;
                     selwakeuppri(&sc->dvr.rsel, PZERO);
                 }
             }
             dvrdone = 1;
         }
         /* demux0.n devices */
         else if(f->type == FILTER_TYPE_SECT) {
#ifndef SMS1XXX_DIAGNOSTIC
             u8 cc = TS_GET_CC(p);
#endif
             if(cc == f->cc)
                 done = sms1xxx_demux_write_section(sc, p, f);
             else {
                 sms1xxx_demux_sectbuf_reset(sc, f, 1,
                    "discontinuity");
                 done = 1;
             }
#ifndef SMS1XXX_DIAGNOSTIC
             f->cc = (cc + 1) & 0xF;
#endif
         }
#ifdef SMS1XXX_DIAGNOSTIC
         if(cc != sc->filter[i].cc &&
         cc != sc->filter[i].cc - 1) {
             WARN("discontinuity for pid: %d expected: %x "
                "got: %x\n", pid,sc->filter[i].cc,cc);
         }
         sc->stats.packetsmatched++;
         sc->filter[i].cc = (cc + 1) & 0xF;
#endif
    }
    return;
}

void
sms1xxx_demux_pesbuf_reset(struct sms1xxx_softc *sc,
    int wake, char *msg)
{
    TRACE(TRACE_SECT,"reason: %s\n",msg);

    mtx_lock(&sc->dvr.lock);
    sc->dvr.roff = 0;
    sc->dvr.woff = 0;
    sc->dvr.ravail = 0;
    sc->dvr.wavail = sc->dvr.size;
    mtx_unlock(&sc->dvr.lock);

    if(wake) {
        /* Wake up readers ! */
        if(sc->dvr.state & DVR_SLEEP) {
            wakeup(&sc->dvr);
        }
        if(sc->dvr.state & DVR_POLL) {
            sc->dvr.state &= ~DVR_POLL;
            selwakeuppri(&sc->dvr.rsel,PZERO);
        }
    }
}
