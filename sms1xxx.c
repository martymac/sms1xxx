/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
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
 *
 * * * *
 *  Main module code
 * * * *
 */

/* SYSCTL(9) stuff */
#include <sys/types.h>
#include <sys/sysctl.h>

/* DRIVER_MODULE(9) stuff */
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include "sms1xxx.h"
#include "sms1xxx-firmware.h"
#include "sms1xxx-usb.h"
#include "sms1xxx-frontend.h"
#include "sms1xxx-demux.h"

struct sms1xxx_frontend frontend = {
   .info = {
       .name               = "Siano Mobile Digital SMS1xxx",
       .type               = FE_OFDM,
       .frequency_min      = 44250000,
       .frequency_max      = 867250000,
       .frequency_stepsize = 250000,
       .caps = FE_CAN_INVERSION_AUTO |
           FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
           FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
           FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_64 |
           FE_CAN_QAM_AUTO | FE_CAN_TRANSMISSION_MODE_AUTO |
           FE_CAN_GUARD_INTERVAL_AUTO |
           FE_CAN_RECOVER |
           FE_CAN_HIERARCHY_AUTO,
    },

    .set_frontend = sms1xxx_frontend_set_frontend,
    .get_frontend = sms1xxx_frontend_get_frontend,
    .get_tune_settings = sms1xxx_frontend_get_tune_settings,

    .read_status = sms1xxx_frontend_read_status,
    .read_ber = sms1xxx_frontend_read_ber,
    .read_ucblocks = sms1xxx_frontend_read_ucblocks,
    .read_signal_strength = sms1xxx_frontend_read_signal_strength,
    .read_snr = sms1xxx_frontend_read_snr,
};

static int
sms1xxx_pid_filter(struct sms1xxx_softc *sc, u16 pid, int onoff)
{
    int err = 0;

    if (onoff != 0)
        err = sms1xxx_usb_add_pid(sc,pid);
    else
        err = sms1xxx_usb_remove_pid(sc,pid);

#ifdef USB_DEBUG
    sms1xxx_usb_getpidfilterlist(sc);
#endif

    return (err);
}

struct sms1xxx_device sms1xxx = {
    .mode = DEVICE_MODE_NONE,
    .requested_mode = DEVICE_MODE_DEFAULT,
    .fw_loading = FALSE,

    .freq_offset = 0,

    .firmware = {
        "SMS100x_Dvbt.inp",                 /* DEVICE_MODE_DVBT */
        "SMS100x_Dvbh.inp",                 /* DEVICE_MODE_DVBH */
        "SMS100x_DabTdmb.inp",              /* DEVICE_MODE_DAB_TDMB */
        "",                                 /* DEVICE_MODE_DAB_TDMB_DABIP */
        "SMS100x_Dvbt.inp"                  /* DEVICE_MODE_DVBT_BDA */
    },

    .frontend = &frontend,
    .pid_filter = sms1xxx_pid_filter,
};

/* Sysctls */
SYSCTL_NODE(_hw_usb, OID_AUTO, sms1xxx, CTLFLAG_RW, 0, "USB sms1xxx");
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, requested_mode, CTLFLAG_RW,
    &sms1xxx.requested_mode, DEVICE_MODE_DEFAULT,
    "Requested mode (0=DVB-T, 1=DVB-H, 2=DAB/T-DMB, 4=DVB-T/BDA)");
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, mode, CTLFLAG_RD,
    &sms1xxx.mode, DEVICE_MODE_NONE,
    "Running mode (-1=NONE, 0=DVB-T, 1=DVB-H, 2=DAB/T-DMB, 4=DVB-T/BDA)");
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, freq_offset, CTLFLAG_RW,
    &sms1xxx.freq_offset, 0,
    "Global frequency offset (Hz)");
#ifdef USB_DEBUG
int sms1xxxdbg = 0;
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, debug, CTLFLAG_RW,
    &sms1xxxdbg, 0,
    "sms1xxx debug level");
#endif

static const struct usb_device_id sms1xxx_devs[] = {
    /* Siano SMS1000 adapter USB1.1 */
    {USB_VPI(USB_VID_SIANO, USB_PID_SIANO_SMS1000_USB11, 0)},
    /* Siano DBM adapter USB1.1 */
    {USB_VPI(USB_VID_SIANO, USB_PID_SIANO_DMB_USB11, 0)},
#if 0
/* XXX Not tested */
    /* Hauppauge Catamount */
    {USB_VPI(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_CATAMOUNT, 0)},
#endif
};

static device_probe_t sms1xxx_probe;
static device_attach_t sms1xxx_attach;
static device_detach_t sms1xxx_detach;

devclass_t sms1xxx_devclass;

static device_method_t sms1xxx_methods[] = {
    DEVMETHOD(device_probe, sms1xxx_probe),
    DEVMETHOD(device_attach, sms1xxx_attach),
    DEVMETHOD(device_detach, sms1xxx_detach),
    {0,0}
};

static driver_t sms1xxx_driver = {
    "sms1xxx",
    sms1xxx_methods,
    sizeof(struct sms1xxx_softc)
};

MODULE_DEPEND(sms1xxx, usb, 1, 1, 1);

static int
sms1xxx_probe(device_t self)
{
    struct usb_attach_arg *uaa = device_get_ivars(self);

    TRACE(TRACE_PROBE,"\n");

    if (uaa->usb_mode != USB_MODE_HOST)
        return (ENXIO);

    return (usbd_lookup_id_by_uaa(sms1xxx_devs, sizeof(sms1xxx_devs), uaa));
}

static int
sms1xxx_attach(device_t self)
{
    struct sms1xxx_softc *sc = device_get_softc(self);
    struct usb_attach_arg *uaa = device_get_ivars(self);

    TRACE(TRACE_PROBE,"vendor=%x, product=%x release=%04x\n",
        uaa->info.idVendor, uaa->info.idProduct, uaa->info.bcdDevice);

    device_set_usb_desc(self);
    mtx_init(&sc->sc_mtx, "sms1xxx", NULL, MTX_DEF);

    sc->sc_dev = self;
    sc->udev = uaa->device;

    sc->sc_iface_index = uaa->info.bIfaceIndex;
    TRACE(TRACE_PROBE,"attaching to interface=%d\n",
        uaa->info.bIfaceIndex);

    sc->device = &sms1xxx;
    sc->sc_dying = 0;
    sc->usbrefs = 0;
    sc->dlg_status = 0;

    if(!sc->device->fw_loading) {
        /* Cold device detected, firmware load necessary */
        TRACE(TRACE_PROBE,"device not ready, mode=%d, requested_mode=%d\n",
            sc->device->mode,sc->device->requested_mode);

        /* Init / start xfers */
        if(sms1xxx_usb_init(sc) || sms1xxx_usb_xfers_start(sc)) {
            ERR("could not setup USB communication\n");
            /* Reset device status */
            sc->sc_dying = 1;
            sc->device->mode = DEVICE_MODE_NONE;
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }

        /* Validate requested mode */
        if((sc->device->requested_mode < DEVICE_MODE_DVBT) ||
            (sc->device->requested_mode > DEVICE_MODE_DVBT_BDA) ||
            (!sc->device->firmware[sc->device->requested_mode][0])) {
                ERR("invalid mode specified %d, using default mode %d\n",
                sc->device->requested_mode,DEVICE_MODE_DEFAULT);
                sc->device->requested_mode = DEVICE_MODE_DEFAULT;
        }

        /* Load firmware */
        TRACE(TRACE_FIRMWARE, "loading firmware %s\n",
            sc->device->firmware[sc->device->requested_mode]) ;
        if(sms1xxx_firmware_load(sc, sc->device->requested_mode)) {
            ERR("failed to load firmware\n");
            /* Reset device status */
            sc->device->fw_loading = FALSE;
            sc->sc_dying = 1;
            sc->device->mode = DEVICE_MODE_NONE;
            sms1xxx_usb_xfers_stop(sc);
            sms1xxx_usb_exit(sc);
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }
        INFO("firmware loaded, device should re-attach now\n");
    }
    else {
        /* Firmware loaded, let's rock ! */

        /* Update device status */
        sc->device->mode = sc->device->requested_mode;
        sc->device->fw_loading = FALSE;

        TRACE(TRACE_PROBE,"firmware loaded, mode=%d\n",
            sc->device->mode);

        /* Init / start xfers */
        if(sms1xxx_usb_init(sc) || sms1xxx_usb_xfers_start(sc)) {
            ERR("could not setup USB communication\n");
            sc->sc_dying = 1;
            sc->device->mode = DEVICE_MODE_NONE;
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }

        /* XXX Fully initialize the device as the official
           driver seems to do. May not be necessary */
        sms1xxx_usb_initdevice(sc, sc->device->mode);
        sms1xxx_usb_setfrequency(sc,
            sc->device->frontend->info.frequency_min,BANDWIDTH_7_MHZ);

        sms1xxx_demux_init(sc);
        sms1xxx_frontend_init(sc);

        INFO("device ready, mode=%d\n", sc->device->mode);
    }
    return (0);
}

static int
sms1xxx_detach(device_t self)
{
    struct sms1xxx_softc *sc = device_get_softc(self);
    int last_mode = sc->device->mode ;

    TRACE(TRACE_PROBE,"sc=%p\n",sc);

    sc->sc_dying = 1;

    /* Reset device status */
    sc->device->mode = DEVICE_MODE_NONE;

    if (last_mode != DEVICE_MODE_NONE) {
        sms1xxx_frontend_exit(sc);
        sms1xxx_demux_exit(sc);
    }
    sms1xxx_usb_xfers_stop(sc);
    sms1xxx_usb_exit(sc);

    mtx_destroy(&sc->sc_mtx);

    return (0);
}

DRIVER_MODULE(sms1xxx, uhub, sms1xxx_driver,
    sms1xxx_devclass, NULL, 0);
