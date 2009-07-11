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
 *  Firmware management
 * * * *
 */

/* firmware(9) stuff */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/linker.h>
#include <sys/firmware.h>

#include "sms1xxx.h"
#include "sms1xxx-coreapi.h"
#include "sms1xxx-firmware.h"
#include "sms1xxx-usb.h"

/* Load a firmware to the device using
   the firmware(9) facility */
int
sms1xxx_firmware_load(struct sms1xxx_softc *sc, int mode)
{
    const struct firmware *fw;
    int err = 0;

    TRACE(TRACE_FIRMWARE, "sc=%p, mode=%d\n", sc, mode);

    if (sc->sc_dying)
        return (ENXIO);

    if ((mode < DEVICE_MODE_DVBT) ||
        (mode > DEVICE_MODE_DVBT_BDA) ||
        (!sc->device->firmware[mode][0])) {
        ERR("invalid mode specified %d\n", mode);
        return (-EINVAL);
    }

    fw = firmware_get(sc->device->firmware[mode]);
    if (fw) {
        err = sms1xxx_usb_write(sc, fw->data, fw->datasize);
        if (err == 0) {
            sc->device->mode = DEVICE_MODE_NONE;
            sc->device->requested_mode = mode;
            sc->device->fw_loading = TRUE;
            INFO("successfully uploaded firmware %s (%zd bytes)\n",
                sc->device->firmware[mode],fw->datasize);
            firmware_put(fw, FIRMWARE_UNLOAD);
            return (0);
        }
        else {
            ERR("failed to upload firmware %s\n",
                sc->device->firmware[mode]);
            firmware_put(fw, FIRMWARE_UNLOAD);
        }
    }
    else {
        ERR("firmware %s not available (is module loaded ?)\n",
            sc->device->firmware[mode]);
    }
    return (-ENOMEM);
}
