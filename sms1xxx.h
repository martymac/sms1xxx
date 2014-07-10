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
 */

#ifndef SMS1XXX_H
#define SMS1XXX_H

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
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
#include "sms1xxx-debug.h"
#include "sms1xxx-ir.h"
#include "sms1xxx-gpio.h"

/* Linux stuff */
#include "linux/dvb/frontend.h"
#include "linux/dvb/dmx.h"
#include "dvb_frontend.h"

/* sysctl(9) defaults */
#ifndef SMS1XXX_DEFAULT_FREQ_OFFSET
#define SMS1XXX_DEFAULT_FREQ_OFFSET  0
#endif

#ifdef SMS1XXX_DEBUG
extern int sms1xxxdbg;
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
    int(*set_property)(struct sms1xxx_softc *sc,
        struct dtv_properties *parg);
    int(*get_property)(struct sms1xxx_softc *sc,
        struct dtv_properties *parg);
    int(*get_tune_settings)(struct sms1xxx_softc *sc,
        struct dvb_frontend_tune_settings *tune);

    int(*read_status)(struct sms1xxx_softc *sc, fe_status_t *status);
    int(*read_ber)(struct sms1xxx_softc *sc, u32 *ber);
    int(*read_ucblocks)(struct sms1xxx_softc *sc, u32 *ucblocks);
    int(*read_signal_strength)(struct sms1xxx_softc *sc, u16 *strength);
    int(*read_snr)(struct sms1xxx_softc *sc, u16 *snr);
};

struct sms1xxx_device {
    int requested_mode;                 /* mode to set when attaching device */
    bool fw_loading;                    /* firmware is loading */

    int freq_offset;                    /* frequency offset used when tuning */

    struct sms1xxx_frontend *frontend;

    int (*pid_filter) (struct sms1xxx_softc*, u16, int); /* PID filter */
};

struct sms1xxx_softc {
    device_t sc_dev;
    struct usb_device *udev;

    struct sms1xxx_device *device;

    /* Xfers */
    struct usb_xfer *sc_xfer[SMS1XXX_N_TRANSFER];
    struct mtx sc_mtx;

    /* tx */
    struct sms1xxx_data sc_tx_data[SMS1XXX_BULK_TX_BUFS];
    sms1xxx_datahead sc_tx_inactive; /* free buffers */
    sms1xxx_datahead sc_tx_pending;  /* buffers waiting for being xmitted */
    sms1xxx_datahead sc_tx_active;   /* buffers currently being xmitted */

    /* rx */
    struct sms1xxx_data sc_rx_data;  /* no data copy, will use internal
                                        xfer's frame buffer */

    /* State */
    u8  sc_iface_index;              /* current interface */
    unsigned long sc_type;           /* interface type */
    int sc_dying;                    /* is sc dying ? */
    int usbinit;                     /* USB already initialized */
    int usbrefs;                     /* frontend & demux references */
    int mode;                        /* current mode */

    /* IR */
    struct sms1xxx_ir ir;            /* infrared (IR) state and data */

    /* GPIO */
    struct sms1xxx_gpio gpio;        /* GPIO state and data */

    /* Dialog synchronization helpers */
#define FRONTEND_TIMEOUT                2500 /* timeout (msec) for frontend
                                                operations (set PID...) */
#define DLG_INIT(status, value)         ((status) &= ~(value))
#define DLG_COMPLETE(status, value)     ((status) |= (value))
#define DLG_ISCOMPLETE(status, value)   ((status) & (value))
#define DLG_STAT_DONE                   0x0001 /* get statistics operation */
#define DLG_TUNE_DONE                   0x0002 /* tuning operation */
#define DLG_PID_DONE                    0x0004 /* pid operations */
#define DLG_INIT_DONE                   0x0008 /*  (init) */
#define DLG_RELOAD_START_DONE           0x0010 /* firmware upload (init) */
#define DLG_DATA_DOWNLOAD_DONE          0x0020 /* firmware upload */
#define DLG_SWDOWNLOAD_TRIGGER_DONE     0x0040 /* firmware upload (start) */
#define DLG_IR_DONE                     0x0080 /* IR module start */
#define DLG_GPIO_CONFIG_DONE            0x0100 /* GPIO configuration done */
#define DLG_GPIO_SET_DONE               0x0200 /* GPIO set done */
#define DLG_GPIO_GET_DONE               0x0400 /* GPIO get done */
    u16 dlg_status;

    /* Frontend */
    struct cdev *frontenddev;
    struct dvb_frontend_parameters fe_params;           /* DVBv3 FE struct */
    struct dtv_frontend_properties dtv_property_cache;  /* DVBv5 cache */
    fe_status_t fe_status;
    int feopen;

    /* DVB Statistics */
    struct SMSHOSTLIB_STATISTICS_DVB_S sms_stat_dvb;

    /* Demuxer */
    int streamrefs;
    eventhandler_tag clonetag;
    struct clonedevs *demux_clones;
#define MAX_FILTERS     32      /* maximum number of active filters
                                   per device */
/* PID values */
#define PIDALL          0x2000  /* special value for all pids */
#define PIDMAX_TS       0x1FFF  /* maximum pid value that is valid inside
                                   TS packets (13 bit, without PIDALL) */
#define PIDMAX          0x2000  /* maximum pid value that is valid
                                   for filtering (including special PIDALL) */
/* PID status */
#define PIDSTOPPED      0x4000  /* pid initialised but not used for filtering */
#define pid_value(pid)  ((pid) & (PIDMAX_TS | PIDALL)) /* remove status
                                                          from PID */
/* Our special values */
#define PIDEMPTY        0xFFFD  /* filter open but pid not initialized */
#define PIDCLONED       0xFFFE  /* filter has a cloned device ready */
#define PIDFREE         0xFFFF  /* filter available for use (initial state) */
    struct filter {
        struct cdev *dev;  /* asociated device */
        u8 *buf;
        u16 pid;
        u16 woff;
        u16 roff;
        u16 wavail;
        u16 ravail;
        u16 wtodo;
        u16 rtodo;
#define SECTBUFSIZE         8192
        u16 size;
        u16 cnt;               /* data object counter (sections or PES packets)
                                  depending on output filter type */
#define FILTER_TYPE_NONE    0
#define FILTER_TYPE_SEC     1
#define FILTER_TYPE_PES     2
        u8 type;
        dmx_output_t output;    /* DMX_OUT_TS_TAP or
                                   DMX_OUT_TSDEMUX_TAP */
        u8 mask;
        u8 value;
        u8 cc;
#define FILTER_BUSY         0x01
#define FILTER_STREAMING    0x02
#define FILTER_SLEEP        0x04
#define FILTER_POLL         0x08
#define FILTER_OVERFLOW     0x10
        u8 state;
        struct selinfo rsel;
    } filter[MAX_FILTERS];
    struct mtx filterlock;

    /* DVR */
#define DVRBUFSIZE          (10000 * PACKET_SIZE)
#define THRESHOLD           (8 * PACKET_SIZE)
    struct {
        u8 *buf;
        u32 roff;
        u32 woff;
        u32 ravail;
        u32 wavail;
        u32 size;
        u32 nobufs;
#define DVR_OPEN    0x0001
#define DVR_SLEEP   0x0002
#define DVR_POLL    0x0004
        u32 state;
        struct mtx lock;
        struct cdev *dev;
        struct selinfo rsel;
    } dvr;

#ifdef SMS1XXX_DIAGNOSTIC
    struct sms1xxx_stats stats;
#endif
};

extern devclass_t sms1xxx_devclass;

#endif
