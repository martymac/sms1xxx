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
 *  IR management
 * * * *
 */

/* uid(9) stuff */
#include <sys/types.h>
#include <sys/uio.h>

/* Read flags (O_NONBLOCK) */
#include <sys/fcntl.h>

#include "sms1xxx.h"
#include "sms1xxx-ir.h"
#include "sms1xxx-usb.h"

/*************************
 *** Private functions ***
 *************************/

static d_open_t  sms1xxx_ir_open;
static d_close_t sms1xxx_ir_close;
static d_read_t sms1xxx_ir_read;

static struct cdevsw sms1xxx_ir_cdevsw = {
    .d_version  = D_VERSION,
    .d_flags    = D_NEEDGIANT,
    .d_open     = sms1xxx_ir_open,
    .d_close    = sms1xxx_ir_close,
    .d_read     = sms1xxx_ir_read,
    .d_name     = "sms1xxx_ir",
};

static int
sms1xxx_ir_open(struct cdev *dev, int flag, int mode, struct thread *p)
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
    if(sc->ir.state & IR_OPEN)
        return (EBUSY);

    /* Reset buffer states */
    sc->ir.woff =
    sc->ir.roff = 0;
    mtx_lock(&sc->ir.lock);
    sc->ir.wavail = SMS1XXX_IR_BUFSIZE;
    sc->ir.ravail = 0;
    mtx_unlock(&sc->ir.lock);

    sc->ir.state |= IR_OPEN;
    return (0);
}

static int
sms1xxx_ir_close(struct cdev *dev, int flag, int mode, struct thread *p)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
#ifdef SMS1XXX_DEBUG
    int unit = device_get_unit(sc->sc_dev);
    TRACE(TRACE_OPEN,"flag=%d, mode=%d, filter=%d\n",
        flag, mode, unit);
#endif

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    sc->ir.state &= ~IR_OPEN;
    return (0);
}

static inline int
sms1xxx_ir_get_data(struct sms1xxx_softc *sc, struct uio *uio)
{
    TRACE(TRACE_IR,"wants %zu bytes, ravail=%u, roff=%u\n",
        uio->uio_resid, sc->ir.ravail, sc->ir.roff);

    int err = 0;
    u32 len = MIN(uio->uio_resid, sc->ir.ravail);
    u32 rlen = len;
    u32 saved_roff = sc->ir.roff;

    if(sc->ir.roff + rlen > SMS1XXX_IR_BUFSIZE) {
        err = uiomove(sc->ir.buf + sc->ir.roff,
            SMS1XXX_IR_BUFSIZE - sc->ir.roff, uio);
        if(err)
          return (-err);
        rlen -= SMS1XXX_IR_BUFSIZE - sc->ir.roff;
        sc->ir.roff = 0;
    }
    err = uiomove(sc->ir.buf + sc->ir.roff, rlen, uio);
    if(err) {
      /* Restore original read offset (to leave
         offsets coherent) and return */
      sc->ir.roff = saved_roff;
      return (-err);
    }
    sc->ir.roff += rlen;

    mtx_lock(&sc->ir.lock);
    sc->ir.wavail += len;
    sc->ir.ravail -= len;
    mtx_unlock(&sc->ir.lock);

    return (uio->uio_resid);
}

static int
sms1xxx_ir_read(struct cdev *dev, struct uio *uio, int flag)
{
    struct sms1xxx_softc *sc = dev->si_drv1;
    int err = 0;

    TRACE(TRACE_IR,"\n");

    if(sc == NULL || sc->sc_dying) {
        TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
        return (ENXIO);
    }
    if(sc->ir.state & IR_SLEEP) {
        ERR("read already in progress\n");
        return (EBUSY);
    }

    while((err = sms1xxx_ir_get_data(sc,uio)) > 0) {
        if(flag & O_NONBLOCK)
            return (EWOULDBLOCK);

        sc->ir.state |= IR_SLEEP;
        err = tsleep(&sc->ir, PZERO | PCATCH, "irrd", 0);
        sc->ir.state &= ~IR_SLEEP;

        if(sc->sc_dying) {
            TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
            return (ENXIO);
        }
        else if(err)
            return (err);
    }
    return (-err);
}

/* Set board-specific IR options */
static void
sms1xxx_ir_config(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_IR,"\n");

    if(sc->ir.module_started) {
        WARN("IR module already configured\n");
        return;
    }

    switch(sc->sc_type) {
        case SMS1XXX_BOARD_HAUPPAUGE_WINDHAM:
            sc->ir.module_avail = 1;
            sc->ir.timeout = IR_DEFAULT_TIMEOUT;
            sc->ir.controller = 0;
        break;
        default:
            sc->ir.module_avail = 0;
        break;
    }
    return;
}

/*************************
 *** Public functions ***
 *************************/

/* Start IR module */
int
sms1xxx_ir_init(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_IR,"\n");

    if(sc->ir.module_started) {
        WARN("IR module already initialized\n");
        return (ENXIO);
    }

    /* Configure IR module */
    sms1xxx_ir_config(sc);

    if(!sc->ir.module_avail) {
        /* No IR module supported */
        return (ENXIO);
    }

    /* Start IR events */
    if (sms1xxx_usb_ir_start(sc)) {
        ERR("could not start IR module\n");
        sc->ir.module_started = 0;
        return (ENXIO);
    }

    /* Initialize data handling */
    mtx_init(&sc->ir.lock, "DVB IR lock", NULL, MTX_DEF);
    sc->ir.wavail = SMS1XXX_IR_BUFSIZE;
    sc->ir.woff =
    sc->ir.roff =
    sc->ir.ravail = 0;

    /* Create device */
    sc->ir.dev = make_dev(&sms1xxx_ir_cdevsw,
        device_get_unit(sc->sc_dev),
        UID_ROOT, GID_WHEEL, 0444,
        "dvb/adapter%d/ir0",
        device_get_unit(sc->sc_dev));
    if (sc->ir.dev != NULL)
        sc->ir.dev->si_drv1 = sc;

    TRACE(TRACE_MODULE,"created ir0 device, addr=%p\n",sc->ir.dev);

    sc->ir.module_started = 1;
    return (0);
}

/* Stop IR module */
int
sms1xxx_ir_exit(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_IR,"\n");

    if(!sc->ir.module_started) {
        return (ENXIO);
    }

    /* Wake up reader */
    if(sc->ir.state & IR_SLEEP) {
        wakeup(&sc->ir);
    }

    /* Destroy device */
    if(sc->ir.dev != NULL) {
        TRACE(TRACE_MODULE,"destroying ir0, addr=%p\n",sc->ir.dev);
        destroy_dev(sc->ir.dev);
        sc->ir.dev = NULL;
    }

    /* Un-initialize data handling */
    sc->ir.woff =
    sc->ir.wavail =
    sc->ir.roff =
    sc->ir.ravail = 0;
    mtx_destroy(&sc->ir.lock);

    sc->ir.module_started = 0;
    return (0);
}

/* Handle received IR data */
int
sms1xxx_ir_put_packet(struct sms1xxx_softc *sc, const u8 *p, u32 len)
{
    TRACE(TRACE_IR,"received %d bytes, wavail=%u, woff=%u\n",
        len, sc->ir.wavail, sc->ir.woff);

#ifdef SMS1XXX_DEBUG
    /* Dump raw data */
    TRACE(TRACE_IR,"dumping %d bytes :\n", len);
    if(sms1xxxdbg & TRACE_IR)
        sms1xxx_dump_data(p, len, '+');
#endif

    if(!sc->ir.module_started) {
        if(sc->ir.module_avail)
            WARN("received IR data while IR module not started, "
                 "ignoring...\n");
        else
            WARN("received IR data while no IR module supported, "
                 "ignoring...\n");
        return (EIO);
    }
    /* Do not store data if nobody is reading */
    if(!(sc->ir.state & IR_OPEN))
        return (EIO);

    /* XXX Should we ignore all bytes instead of trying to fill
       the whole buffer ? */
    if(len > sc->ir.wavail) {
        TRACE(TRACE_IR, "not enough space in buffer, ignoring %d bytes\n",
            len - sc->ir.wavail);
        len = sc->ir.wavail;
    }
    u32 rlen = len;
    if(sc->ir.woff + rlen > SMS1XXX_IR_BUFSIZE) {
        memcpy(sc->ir.buf + sc->ir.woff, p, SMS1XXX_IR_BUFSIZE - sc->ir.woff);
        rlen -= SMS1XXX_IR_BUFSIZE - sc->ir.woff;
        sc->ir.woff = 0;
    }
    memcpy(sc->ir.buf + sc->ir.woff, p, rlen);
    sc->ir.woff += rlen;

    mtx_lock(&sc->ir.lock);
    sc->ir.wavail -= len;
    sc->ir.ravail += len;
    mtx_unlock(&sc->ir.lock);

    /* Wake up reader */
    if(sc->ir.state & IR_SLEEP) {
        wakeup(&sc->ir);
    }
    return (0);
}
