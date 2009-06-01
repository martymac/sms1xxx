/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 7.0 and higher:
 *
 *  Copyright (C) 2008 - Ganaël Laplanche, http://contribs.martymac.com
 *
 *  This driver contains code taken from the FreeBSD dvbusb driver:
 *
 *  Copyright (C) 2006 - 2007 Raaf
 *  Copyright (C) 2004 - 2006 Patrick Boettcher
 *
 *  This driver contains code taken from the Linux siano driver:
 *
 *  Copyright (c), 2005-2008 Siano Mobile Silicon, Inc.
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

#ifndef SMS1XXX_H
#define SMS1XXX_H

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/param.h>
#include <sys/bus.h>

/* usbdi(9) stuff */
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>

/* dev_clone(9) stuff */
#include <sys/param.h>
#include <sys/conf.h>

/* selrecord(9) stuff */
#include <sys/selinfo.h>

/* Our stuff */
#include "sms1xxx-coreapi.h"
#ifdef DIAGNOSTIC
#include "sms1xxx-diagnostic.h"
#endif

/* Linux stuff */
#include "linux/dvb/frontend.h"

#define SMS1XXXUNIT(n) (minor(n))

#define TRACE_PROBE     0x0001
#define TRACE_MODULE    0x0002
#define TRACE_OPEN      0x0004
#define TRACE_IOCTL     0x0008
#define TRACE_READ      0x0010
#define TRACE_DVR_READ  0x0020
#define TRACE_POLL      0x0040
#define TRACE_USB       0x0080
#define TRACE_USB_FLOW  0x0100
#define TRACE_FIRMWARE  0x0200
#define TRACE_CC        0x0400
#define TRACE_SECT      0x0800
#define TRACE_FILTERS   0x1000

#ifdef USB_DEBUG
extern int sms1xxxdbg;
#define TRACE(R, FMT, ...) if(sms1xxxdbg & R) \
    printf("%s: " FMT, __func__ , ##__VA_ARGS__)
#else
#define TRACE(R, FMT, ...)
#endif

#define ERR(FMT, ...) printf("%s: Error : " FMT, __func__ , ##__VA_ARGS__)
#define WARN(FMT, ...) printf("%s: Warning : " FMT, __func__ , ##__VA_ARGS__)
#define INFO(FMT, ...) printf("%s: Info : " FMT, __func__ , ##__VA_ARGS__)

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif

struct sms1xxx_softc;

struct sms1xxx_frontend {
    struct dvb_frontend_info info;

    int(*set_frontend)(struct sms1xxx_softc *sc,
        struct dvb_frontend_parameters *params);
    int(*get_frontend)(struct sms1xxx_softc *sc,
        struct dvb_frontend_parameters *params);
    int(*get_tune_settings)(struct sms1xxx_softc *sc,
        struct dvb_frontend_tune_settings *tune);

    int(*read_status)(struct sms1xxx_softc *sc, fe_status_t *status);
    int(*read_ber)(struct sms1xxx_softc *sc, u32 *ber);
    int(*read_ucblocks)(struct sms1xxx_softc *sc, u32 *ucblocks);
    int(*read_signal_strength)(struct sms1xxx_softc *sc, u16 *strength);
    int(*read_snr)(struct sms1xxx_softc *sc, u16 *snr);
};

struct sms1xxx_device {
    int mode;                           /* current mode */
    int requested_mode;                 /* mode to set when attaching device */
    bool fw_loading;                    /* firmware is loading */

    int freq_offset;                    /* frequency offset used when tuning */

    const char * firmware[DEVICE_MODE_MAX]; /* supported firmwares */

    int bulk_ctrl_endpoint;             /* ctrl endpoint */
    int bulk_rcv_endpoint;              /* receive endpoint */

    struct sms1xxx_frontend *frontend;

    int (*pid_filter) (struct sms1xxx_softc*, u16, int); /* PID filter */
};

struct sms1xxx_softc {
    device_t sc_dev;
    usbd_device_handle udev;
    usbd_interface_handle iface;
    struct sms1xxx_device *device;

    /* Output */
    usbd_pipe_handle opipe;
    usbd_xfer_handle oxfer;

    /* Input data stream/command results */
    usbd_pipe_handle ipipe;
    struct sms1xxx_sbuf {
        struct sms1xxx_softc *sc;
        usbd_xfer_handle xfer;
        u_char *buf;
    } sbuf[MAXXFERS];
    u_int32_t sbufsize;

    /* State */
    int sc_dying;
    int usbinit;
    int bulk_ctrl_busy;
    int usbrefs;

    /* Dialog synchronization helpers */
#define FRONTEND_TIMEOUT    (3 * 1000)  /* timeout (msec) for frontend
                                           operations (set PID...) */
#define DLG_INIT(status, value)         ((status) &= ~(value))
#define DLG_COMPLETE(status, value)     ((status) |= (value))
#define DLG_ISCOMPLETE(status, value)   ((status) & (value))
#define DLG_STAT_DONE       0x1 /* get statistics operation */
#define DLG_TUNE_DONE       0x2 /* tuning operation */
#define DLG_PID_DONE        0x4 /* pid operations */
    char dlg_status;

    /* Frontend */
    struct cdev *frontenddev;
    struct dvb_frontend_parameters fe_params;
    fe_status_t fe_status;
    u32 fe_ber, fe_unc;
    u16 fe_snr, fe_signal_strength;
    int feopen;

    /* Demuxer */
    int streamrefs;
    eventhandler_tag clonetag;
    struct clonedevs *demux_clones;
#define MAX_FILTERS 16
/* PID values */
#define PIDMAX      0x1FFF /* maximum pid value that is valid for filtering */
#define PIDALL      0x2000 /* special value for all pids */
/* PID status */
#define PIDSTOPPED  0x4000 /* pid initialised but not used for filtering */
#define PIDEMPTY    0xFFFE /* filter open but pid not initialized */
#define PIDCLONED   0xFFFD /* filter has a cloned device ready */
#define PIDFREE     0xFFFF /* filter available for use (initial state) */
    struct filter {
        struct cdev *dev;  /* asociated device */
        u_char *buf;
        u_int16_t pid;
        u_int16_t woff;
        u_int16_t roff;
        u_int16_t wavail;
        u_int16_t ravail;
        u_int16_t wtodo;
        u_int16_t rtodo;
#define SECTBUFSIZE         8192
        u_int16_t size;
        u_int16_t sectcnt;
#define FILTER_TYPE_PES     0
#define FILTER_TYPE_SECT    1
        u_char type;
        u_char mask;
        u_char value;
        u_char cc;
#define FILTER_BUSY         0x01
#define FILTER_STREAMING    0x02
#define FILTER_SLEEP        0x04
#define FILTER_POLL         0x08
#define FILTER_OVERFLOW     0x10
        u_char state;
        struct selinfo rsel;
    } filter[MAX_FILTERS];
    struct mtx filterlock;

    /* DVR */
#define DVRBUFSIZE          (10000 * PACKET_SIZE)
#define THRESHOLD           (8 * PACKET_SIZE)
    struct {
        u_char *buf;
        u_int32_t roff;
        u_int32_t woff;
        u_int32_t ravail;
        u_int32_t wavail;
        u_int32_t threshold;
        u_int32_t size;
        u_int32_t nobufs;
#define DVR_OPEN    0x0001
#define DVR_SLEEP   0x0002
#define DVR_POLL    0x0004
#define DVR_AWAKE   0x0008
        u_int32_t state;
        struct mtx lock;
        struct cdev *dev;
        struct selinfo rsel;
    } dvr;

#ifdef DIAGNOSTIC
    struct sms1xxx_stats stats;
#endif
};

extern devclass_t sms1xxx_devclass;

#endif
