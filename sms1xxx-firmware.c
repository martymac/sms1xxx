/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2014, Ganaël Laplanche, http://contribs.martymac.org
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
 *  Firmware management
 * * * *
 */

/* firmware(9) stuff */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/linker.h>
#include <sys/firmware.h>

/* le..toh(9) stuff */
#include <sys/endian.h>

#include "sms1xxx.h"
#include "sms1xxx-coreapi.h"
#include "sms1xxx-firmware.h"
#include "sms1xxx-usb.h"
#include "sms1xxx-endian.h"

static int sms1xxx_firmware_load_family1(struct sms1xxx_softc *,
    const u8 *, u32);
static int sms1xxx_firmware_load_family2(struct sms1xxx_softc *,
    const u8 *, u32);

static char *sms1xxx_firmwares
    [((SMS1XXX_TYPE_MAX & SMS1XXX_TYPE_MASK) >> 16) + 1]
    [DEVICE_MODE_MAX + 1] = {
/* STELLAR */
    { "stellar_dvbt.fw",       /* DVBT */
      "stellar_dvbh.fw",       /* DVBH */
      "stellar_dabtdmb.fw",    /* DAB_TDMB */
      "",                      /* DAB_TDMB_DABIP */
      "stellar_dvbt.fw",       /* DVBT_BDA */
      "",                      /* DEVICE_MODE_ISDBT */
      "",                      /* DEVICE_MODE_ISDBT_BDA */
      "",                      /* DEVICE_MODE_CMMB */
      "" },                    /* DEVICE_MODE_RAW_TUNER */
/* NOVA_A */
    { "",                      /* DVBT */
      "",                      /* DVBH */
      "",                      /* DAB_TDMB */
      "",                      /* DAB_TDMB_DABIP */
      "",                      /* DVBT_BDA */
      "",                      /* DEVICE_MODE_ISDBT */
      "",                      /* DEVICE_MODE_ISDBT_BDA */
      "",                      /* DEVICE_MODE_CMMB */
      "" },                    /* DEVICE_MODE_RAW_TUNER */
/* NOVA_B */
    { "novab0_dvbbda.fw",      /* DVBT */
      "novab0_dvbbda.fw",      /* DVBH */
      "novab0_tdmb.fw",        /* DAB_TDMB */
      "",                      /* DAB_TDMB_DABIP */
      "novab0_dvbbda.fw",      /* DVBT_BDA */
      "novab0_isdbtbda.fw",    /* DEVICE_MODE_ISDBT */
      "novab0_isdbtbda.fw",    /* DEVICE_MODE_ISDBT_BDA */
      "",                      /* DEVICE_MODE_CMMB */
      "" },                    /* DEVICE_MODE_RAW_TUNER */
/* VEGA */
    { "",                      /* DVBT */
      "",                      /* DVBH */
      "",                      /* DAB_TDMB */
      "",                      /* DAB_TDMB_DABIP */
      "",                      /* DVBT_BDA */
      "",                      /* DEVICE_MODE_ISDBT */
      "",                      /* DEVICE_MODE_ISDBT_BDA */
      "",                      /* DEVICE_MODE_CMMB */
      "" },                    /* DEVICE_MODE_RAW_TUNER */
};

/* Return a firmware name given type and mode
   Return NULL if an invalid type/mode is specified or if no firmware found */ 
char *
sms1xxx_firmware_name(unsigned long type, int mode) {
    /* Check type, mode, and firmware existence */
    if (((type & SMS1XXX_TYPE_MASK)>=(SMS1XXX_TYPE_MIN & SMS1XXX_TYPE_MASK))
        && ((type & SMS1XXX_TYPE_MASK)<=(SMS1XXX_TYPE_MAX & SMS1XXX_TYPE_MASK))
        && (mode >= DEVICE_MODE_MIN)
        && (mode <= DEVICE_MODE_MAX)
        && (sms1xxx_firmwares[(type & SMS1XXX_TYPE_MASK) >> 16][mode][0])) {
        return sms1xxx_firmwares[(type & SMS1XXX_TYPE_MASK) >> 16][mode] ;
    }
    return (NULL);
}

/* Load a firmware to the device using
   the firmware(9) facility (family1 devices) */
static int
sms1xxx_firmware_load_family1(struct sms1xxx_softc *sc, const u8 *data,
    u32 datasize)
{
    if (sc->sc_dying)
        return (ENXIO);

    if ((data == NULL) || (datasize == 0))
        return (EINVAL);

    TRACE(TRACE_FIRMWARE, "data=%p, datasize=%d\n", data, datasize);
    return sms1xxx_usb_rawwrite(sc, data, datasize);
}

/* Load a firmware to the device using
   the firmware(9) facility (family2 devices) */
static int
sms1xxx_firmware_load_family2(struct sms1xxx_softc *sc, const u8 *data,
    u32 datasize)
{
    if (sc->sc_dying)
        return (ENXIO);

    if ((data == NULL) || (datasize == 0))
        return (EINVAL);

    TRACE(TRACE_FIRMWARE, "data=%p, datasize=%d\n", data, datasize);

    const struct sms_firmware *firmware = (const struct sms_firmware *)data;
    u32 mem_address = le32toh(firmware->start_address);
    const u8 *payload = firmware->payload;
    int err = 0;

    if (sc->mode != DEVICE_MODE_NONE) {
        /* send reload start command on warm devices */
        err = sms1xxx_usb_reloadstart(sc);
        if(err != 0) {
            TRACE(TRACE_FIRMWARE, "reloadstart() failed, err=%d\n", err);
            return (EIO);
        }
        mem_address = *(const u32*) &payload[20];
    }

    while(datasize && (err == 0)) {
        /* Upload firmware using chunks <= SMS_MAX_PAYLOAD_SIZE */
        int payload_size = min((int) datasize, SMS_MAX_PAYLOAD_SIZE);

        struct sms_data_download Msg;
        Msg.x_msg_header.msg_type = MSG_SMS_DATA_DOWNLOAD_REQ;
        Msg.x_msg_header.msg_src_id = 0;
        Msg.x_msg_header.msg_dst_id = HIF_TASK;
        Msg.x_msg_header.msg_length =
            (u16)(sizeof(struct sms_msg_hdr) + sizeof(u32) + payload_size);
        Msg.x_msg_header.msg_flags = 0;

        Msg.mem_addr = mem_address;
        memcpy(Msg.payload, payload, payload_size);

        TRACE(TRACE_FIRMWARE, "sending firmware chunk, size=%d, "
            "remaining=%d\n", payload_size, datasize - payload_size);

        sms1xxx_endian_handle_tx_message(&Msg);
        /* XXX honour SMS_ROM_NO_RESPONSE and use sms1xxx_usb_write() ? */
        err = sms1xxx_usb_write_and_wait(sc, (u8*)&Msg,
            Msg.x_msg_header.msg_length, DLG_DATA_DOWNLOAD_DONE,
            FRONTEND_TIMEOUT);

        if(err == 0) {
            payload += payload_size;
            datasize -= payload_size;
            mem_address += payload_size;
        }
    }

#ifdef SMS1XXX_DEBUG
    if ((err != 0) || (datasize > 0))
        TRACE(TRACE_FIRMWARE, "firmware upload error, err=%d, done=%zu, "
            "remaining=%d\n", err, payload - firmware->payload, datasize);
#endif

    if (err == 0) {
        /* send reload exec command on warm devices */
        if (sc->mode != DEVICE_MODE_NONE)
            err = sms1xxx_usb_reloadexec(sc);
        /* or software download trigger on cold ones */
        else
            err = sms1xxx_usb_swdtrigger(sc, firmware->start_address);

        TRACE(TRACE_FIRMWARE, "trigger done, err=%d\n", err);
    }
    return (err);
}

/* Wrapper that loads a firmware to the device using
   family1 or family2-specific functions */
int
sms1xxx_firmware_load(struct sms1xxx_softc *sc, int mode)
{
    const struct firmware *fw;
    char *fw_name;
    int err = 0;

    TRACE(TRACE_FIRMWARE, "sc=%p, mode=%d\n", sc, mode);

    if (sc->sc_dying)
        return (ENXIO);

    fw_name = sms1xxx_firmware_name(sc->sc_type, mode);
    if (fw_name == NULL) {
        ERR("no firmware support for type 0x%lx, mode %d\n", sc->sc_type, mode);
        return (EINVAL);
    }

    TRACE(TRACE_FIRMWARE, "uploading firmware %s\n", fw_name);
    fw = firmware_get(fw_name);
    if (fw) {
        switch(sc->sc_type & SMS1XXX_FAMILY_MASK) {
            case SMS1XXX_FAMILY1:
                err = sms1xxx_firmware_load_family1(sc, fw->data, fw->datasize);
                break;
            case SMS1XXX_FAMILY2:
                err = sms1xxx_firmware_load_family2(sc, fw->data, fw->datasize);
                break;
            default:
                ERR("invalid device type 0x%lx\n",
                    sc->sc_type & SMS1XXX_FAMILY_MASK);
                err = (EINVAL);
                break;
        }
        firmware_put(fw, FIRMWARE_UNLOAD);
        if (err == 0) {
            sc->mode = DEVICE_MODE_NONE;
            sc->device->requested_mode = mode;
            sc->device->fw_loading = TRUE;
            INFO("successfully uploaded firmware %s (%zd bytes)\n",
                fw_name, fw->datasize);
        }
        else {
            ERR("failed to upload firmware %s\n", fw_name);
        }
    }
    else {
        ERR("firmware %s not available (is module loaded ?)\n", fw_name);
        err = (EINVAL);
    }
    return (err);
}
