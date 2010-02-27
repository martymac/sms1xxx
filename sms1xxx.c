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
#include "sms1xxx-ir.h"

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

#ifdef SMS1XXX_DEBUG
    sms1xxx_usb_getpidfilterlist(sc);
#endif

    return (err);
}

#ifdef SMS1XXX_DEBUG
static int
sms1xxx_sysctl_led_status(SYSCTL_HANDLER_ARGS)
{
    int err = 0;
    int led_status = SMS_LED_OFF;

    struct sms1xxx_softc *sc = device_get_softc(oidp->oid_arg1);
    if (sc == NULL)
        return (EINVAL);

    err = sysctl_handle_int(oidp, &led_status, 0, req);

    if (err != 0 || req->newptr == NULL)
        return (err);

    sms1xxx_gpio_power_led_feedback(sc, led_status);
    sms1xxx_gpio_status_led_feedback(sc, led_status);

    return (0);
}
#endif

struct sms1xxx_device sms1xxx = {
    .requested_mode = DEVICE_MODE_DEFAULT,
    .fw_loading = FALSE,

    .freq_offset = SMS1XXX_DEFAULT_FREQ_OFFSET,

    .frontend = &frontend,
    .pid_filter = sms1xxx_pid_filter,
};

/* Sysctls */
SYSCTL_NODE(_hw_usb, OID_AUTO, sms1xxx, CTLFLAG_RW, 0, "USB sms1xxx");
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, requested_mode, CTLFLAG_RW,
    &sms1xxx.requested_mode, DEVICE_MODE_DEFAULT,
    "Requested mode (0=DVB-T, 1=DVB-H, 2=DAB/T-DMB, 4=DVB-T/BDA, 5=ISDBT, "
    "6=ISDBT/BDA)");
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, freq_offset, CTLFLAG_RW,
    &sms1xxx.freq_offset, SMS1XXX_DEFAULT_FREQ_OFFSET,
    "Global frequency offset (Hz)");
#ifdef SMS1XXX_DEBUG
int sms1xxxdbg = SMS1XXX_DEBUG_DEFAULT_LEVEL;
SYSCTL_INT(_hw_usb_sms1xxx, OID_AUTO, debug, CTLFLAG_RW,
    &sms1xxxdbg, SMS1XXX_DEBUG_DEFAULT_LEVEL,
    "sms1xxx debug level");
#endif

static const struct usb_device_id sms1xxx_devs[] = {
    /* Siano SMS1000 adapter - USB1.1 */
    {USB_VPI(0x187f, 0x0010, SMS1XXX_BOARD_SIANO_STELLAR)},
    /* Siano DBM adapter - USB1.1 */
    {USB_VPI(0x187f, 0x0100, SMS1XXX_BOARD_SIANO_STELLAR)},
    /* Hauppauge WinTV MiniStick 25 Jahre Edition (1295), SMS1102, USB2.0 */
    /* Hauppauge WinTV MiniStick HD (1245), SMS1102, USB2.0 */
    {USB_VPI(0x2040, 0x5500, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},

#if 0
/* Not tested */
    {USB_VPI(0x187f, 0x0200, SMS1XXX_BOARD_SIANO_NOVA_A)},
    {USB_VPI(0x187f, 0x0201, SMS1XXX_BOARD_SIANO_NOVA_B)},
    {USB_VPI(0x187f, 0x0300, SMS1XXX_BOARD_SIANO_VEGA)},
    {USB_VPI(0x2040, 0x1700, SMS1XXX_BOARD_HAUPPAUGE_CATAMOUNT)},
    {USB_VPI(0x2040, 0x1800, SMS1XXX_BOARD_HAUPPAUGE_OKEMO_A)},
    {USB_VPI(0x2040, 0x1801, SMS1XXX_BOARD_HAUPPAUGE_OKEMO_B)},
    {USB_VPI(0x2040, 0x2000, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD)},
    {USB_VPI(0x2040, 0x2009, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD_R2)},
    {USB_VPI(0x2040, 0x200a, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD)},
    {USB_VPI(0x2040, 0x2010, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD)},
    {USB_VPI(0x2040, 0x2011, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD)},
    {USB_VPI(0x2040, 0x2019, SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD)},
    {USB_VPI(0x2040, 0x5510, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0x5520, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0x5530, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0x5580, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0x5590, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x187f, 0x0202, SMS1XXX_BOARD_SIANO_NICE)},
    {USB_VPI(0x187f, 0x0301, SMS1XXX_BOARD_SIANO_VENICE)},
    {USB_VPI(0x2040, 0xb900, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xb910, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xb980, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xb990, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xc000, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xc010, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xc080, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
    {USB_VPI(0x2040, 0xc090, SMS1XXX_BOARD_HAUPPAUGE_WINDHAM)},
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
    struct sysctl_ctx_list *sctx;
    struct sysctl_oid *soid;

    TRACE(TRACE_PROBE,"vendor=%x, product=%x release=%04x\n",
        uaa->info.idVendor, uaa->info.idProduct, uaa->info.bcdDevice);

    device_set_usb_desc(self);
    mtx_init(&sc->sc_mtx, "sms1xxx", NULL, MTX_DEF);

    sc->sc_dev = self;
    sc->udev = uaa->device;

    sc->sc_iface_index = uaa->info.bIfaceIndex;
    TRACE(TRACE_PROBE,"attaching to interface=%d\n",
        uaa->info.bIfaceIndex);

    sc->sc_type = USB_GET_DRIVER_INFO(uaa);
    sc->device = &sms1xxx;
    sc->sc_dying = 0;
    sc->usbrefs = 0;
    sc->mode = DEVICE_MODE_NONE;
    sc->dlg_status = 0;
    sc->ir.module_started = 0;

    if(!sc->device->fw_loading) {
        /* Cold device detected, firmware load necessary */
        TRACE(TRACE_PROBE,"device not ready, requested_mode=%d\n",
            sc->device->requested_mode);

        /* Init / start xfers */
        if(sms1xxx_usb_init(sc) || sms1xxx_usb_xfers_start(sc)) {
            ERR("could not setup USB communication\n");
            /* Reset device status */
            sc->sc_dying = 1;
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }

        /* Validate requested mode */
        if(sms1xxx_firmware_name(sc->sc_type, sc->device->requested_mode) ==
            NULL) {
            ERR("invalid mode specified %d, using default mode %d\n",
            sc->device->requested_mode, DEVICE_MODE_DEFAULT);
            sc->device->requested_mode = DEVICE_MODE_DEFAULT;
        }

        /* Load firmware */
        if(sms1xxx_firmware_load(sc, sc->device->requested_mode)) {
            ERR("failed to load firmware\n");
            /* Reset device status */
            sc->device->fw_loading = FALSE;
            sc->sc_dying = 1;
            sms1xxx_usb_xfers_stop(sc);
            sms1xxx_usb_exit(sc);
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }

        if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1) {
            /* Terratec Cinergy Piranha detaches and re-attaches
               itself automatically, so return now */
            INFO("firmware loaded, device should re-attach now\n");
            return (0);
        }
    }
    /* FALLTHROUGH for SMS1XXX_FAMILY2 */

    /* Firmware loaded, let's rock ! */
    /* Update device status */
    sc->mode = sc->device->requested_mode;
    sc->device->fw_loading = FALSE;

    TRACE(TRACE_PROBE,"firmware loaded, mode=%d\n", sc->mode);

    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1) {
        /* Init / start xfers for SMS1XXX_FAMILY1, once re-attached */
        if(sms1xxx_usb_init(sc) || sms1xxx_usb_xfers_start(sc)) {
            ERR("could not setup USB communication\n");
            sc->sc_dying = 1;
            sc->mode = DEVICE_MODE_NONE;
            mtx_destroy(&sc->sc_mtx);
            return (ENXIO);
        }
    }

#ifdef SMS1XXX_DEBUG
    sms1xxx_usb_getversion(sc);
#endif

    /* Set up mode, start demuxer and frontend */
    sms1xxx_usb_initdevice(sc, sc->mode);
    if(sms1xxx_demux_init(sc)) {
        ERR("could not start demuxer\n");
        sc->sc_dying = 1;
        sc->mode = DEVICE_MODE_NONE;
        sms1xxx_usb_xfers_stop(sc);
        sms1xxx_usb_exit(sc);
        mtx_destroy(&sc->sc_mtx);
        return (ENXIO);
    }
    sms1xxx_frontend_init(sc);

    /* Start IR module */
    sms1xxx_ir_init(sc);

    /* Initialize GPIO */
    sms1xxx_gpio_init(sc);

    INFO("device ready, mode=%d\n", sc->mode);

    /* Add sysctls */
    sctx = device_get_sysctl_ctx(sc->sc_dev);
    soid = device_get_sysctl_tree(sc->sc_dev);
    SYSCTL_ADD_INT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "mode",
        CTLFLAG_RD, &sc->mode, 0,
        "Running mode (-1=NONE, 0=DVB-T, 1=DVB-H, 2=DAB/T-DMB, 4=DVB-T/BDA, "
        "5=ISDBT, 6=ISDBT/BDA)");
    if(sc->ir.module_avail) {
        SYSCTL_ADD_INT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO,
            "ir_module_started",
            CTLFLAG_RD, &sc->ir.module_started, 0,
            "IR module available and started (0=No, 1=Yes)");
    }
    if(sc->gpio.lna_ctrl != SMS_GPIO_PIN_UNDEF) {
        SYSCTL_ADD_INT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO,
            "use_lna",
            CTLFLAG_RW, &sc->gpio.use_lna, 0,
            "Use LNA when tuning (0=No, 1=Yes, 2=Force off)");
    }
#ifdef SMS1XXX_DEBUG
    if((sc->gpio.led_lo != SMS_GPIO_PIN_UNDEF) &&
        (sc->gpio.led_hi != SMS_GPIO_PIN_UNDEF) &&
        (sc->gpio.led_power != SMS_GPIO_PIN_UNDEF)) {
        SYSCTL_ADD_PROC(sctx, SYSCTL_CHILDREN(soid), OID_AUTO,
            "led_status",
            CTLTYPE_INT | CTLFLAG_RW, sc->sc_dev, sizeof(sc->sc_dev),
            sms1xxx_sysctl_led_status, "I",
            "Set LED status (or-able, Off=0, Low=1, High=2, Power=4)");
            /* SMS_LED_OFF,SMS_LED_LO,SMS_LED_HI,SMS_LED_PW */
    }
#endif

    return (0);
}

static int
sms1xxx_detach(device_t self)
{
    struct sms1xxx_softc *sc = device_get_softc(self);
    int last_mode = sc->mode ;

    TRACE(TRACE_PROBE,"sc=%p\n",sc);

    sc->sc_dying = 1;

    /* Reset device status */
    sc->mode = DEVICE_MODE_NONE;

    if (last_mode != DEVICE_MODE_NONE) {
        sms1xxx_gpio_exit(sc);
        sms1xxx_ir_exit(sc);
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
