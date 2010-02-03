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
 *  USB communications handling
 * * * *
 */

/* sleep(9) stuff */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>

#include "sms1xxx.h"
#include "sms1xxx-usb.h"
#include "sms1xxx-demux.h"
#include "sms1xxx-firmware.h"
#include "sms1xxx-endian.h"
#include "sms1xxx-gpio.h"

static void sms1xxx_usb_tx_cb(struct usb_xfer *, usb_error_t);
static void sms1xxx_usb_rx_cb(struct usb_xfer *, usb_error_t);

static const struct usb_config sms1xxx_config[SMS1XXX_N_TRANSFER] = {
       [SMS1XXX_BULK_TX] = {
               .type = UE_BULK,
               .endpoint = USB_SIANO_ENDP_CTRL,
               .direction = UE_DIR_OUT,
               .bufsize = SMS1XXX_BULK_TX_BUFS_SIZE,
               .timeout = 250,
               .callback = &sms1xxx_usb_tx_cb,
       },
       [SMS1XXX_BULK_RX] = {
               .type = UE_BULK,
               .endpoint = USB_SIANO_ENDP_RCV,
               .direction = UE_DIR_IN,
               .bufsize = SMS1XXX_BULK_RX_BUFS_SIZE,
               .timeout = FRONTEND_TIMEOUT + 500,
               .callback = &sms1xxx_usb_rx_cb,
               .flags = {
                   .pipe_bof = 1,
                   .short_xfer_ok = 1,
               },
       },
};

/****************************
 * Initialization functions *
 ****************************/

/* Initialize USB input/output pipes and xfers */
int
sms1xxx_usb_init(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB, "sc=%p init=%d\n", sc, sc->usbinit);

    if(sc->usbinit < 0) {
        ERR("USB (Un)Initialisation in progress\n");
        return (EBUSY);
    }
    sc->usbinit = -1;

    /* Setup USB xfers */
    if(usbd_transfer_setup(sc->udev, &sc->sc_iface_index, sc->sc_xfer,
      sms1xxx_config, SMS1XXX_N_TRANSFER, sc, &sc->sc_mtx)) {
        ERR("could not setup xfers\n");
        sc->usbinit = 0;
        return (EIO);
    }

    sc->usbinit = 1;
    return (0);
}

/* Un-initialize USB input/output pipes and xfers */
int
sms1xxx_usb_exit(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

    if(sc->usbinit < 0) {
        ERR("USB (Un)Initialisation in progress\n");
        return (EBUSY);
    }
    sc->usbinit = -1;

    /* Unsetup USB xfers */
    usbd_transfer_unsetup(sc->sc_xfer, SMS1XXX_N_TRANSFER);

    sc->usbinit = 0;
    return (0);
}

/* Called on each open() of a demux device */
int
sms1xxx_usb_ref(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"refcnt %d -> %d\n",sc->usbrefs,sc->usbrefs + 1);
    sc->usbrefs++;
    return (0);
}

/* Called on each close() of a demux device */
void
sms1xxx_usb_deref(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"refcnt %d -> %d\n",sc->usbrefs,sc->usbrefs - 1);
    sc->usbrefs--;
}

/**********************
 * Callback functions *
 **********************/

/* Analyze packets received from the device */
static void
sms1xxx_usb_get_packets(struct sms1xxx_softc *sc, u8 *packet, u32 bytes)
{
    u32 offset = 0;
    int stats_updated = 0;

    if (bytes == 0) {
        ERR("null message\n");
        return;
    }

    struct SmsMsgHdr_ST *phdr = (struct SmsMsgHdr_ST *)packet;
    sms1xxx_endian_handle_message_header(phdr);

    TRACE(TRACE_USB_FLOW,"got %d bytes "
        "(msgLength=%d, msgType=%d, msgFlags=%d)\n",
        bytes, phdr->msgLength, phdr->msgType, phdr->msgFlags);

    if (bytes < phdr->msgLength) {
        ERR("invalid message : msgLength=%d, received only %d bytes\n",
            phdr->msgLength, bytes);
        return;
    }

    if ((phdr->msgFlags & MSG_HDR_FLAG_SPLIT_MSG) &&
        (bytes > phdr->msgLength)) {
        /* Compute start offset */
        offset = bytes - phdr->msgLength;
        if(offset < sizeof(struct SmsMsgHdr_ST)) {
            ERR("invalid message : offset too small\n");
            return;
        }
        /* Copy header to its new location */
        memcpy((u8 *)phdr + offset, phdr, sizeof(struct SmsMsgHdr_ST));

        TRACE(TRACE_USB_FLOW, "split message detected, offset=%d, "
            "header=%p, new=%p\n", offset, phdr, (u8 *)phdr + offset);

        /* Move pointer to its new location */
        phdr = (struct SmsMsgHdr_ST *)((u8 *)phdr + offset);
    }
    bytes = phdr->msgLength;

    /* Do we need to re-route ? */
    if ((phdr->msgType == MSG_SMS_HO_PER_SLICES_IND) ||
            (phdr->msgType == MSG_SMS_TRANSMISSION_IND)) {
        if (sc->mode == DEVICE_MODE_DVBT_BDA)
            phdr->msgDstId = DVBT_BDA_CONTROL_MSG_ID;
    }

    sms1xxx_endian_handle_rx_message(phdr);

    switch (phdr->msgType) {
        case MSG_SMS_DVBT_BDA_DATA:
            {
                TRACE(TRACE_USB_FLOW,"handling MSG_SMS_DVBT_BDA_DATA\n");
                /* Send data to demuxer using chunks of PACKET_SIZE bytes */
                /* Number of bytes remaining */
                u32 remaining = bytes - sizeof(struct SmsMsgHdr_ST);
                /* Number of chunks sent */
                u32 sent = 0;
                while (remaining >= PACKET_SIZE) {
                    TRACE(TRACE_USB_FLOW,"sending packet %d to demuxer, "
                        "addr = %p\n",sent,
                        ((u8*)(phdr+1))+(sent*PACKET_SIZE));
                    sms1xxx_demux_put_packet(sc,
                        ((u8*)(phdr+1))+(sent*PACKET_SIZE));
                    sent++;
                    remaining -= PACKET_SIZE;
                }
                TRACE(TRACE_USB_FLOW,"sent %d packets to demuxer, "
                    "%d bytes remaining\n",sent,remaining);
                if (remaining > 0) {
                    WARN("junk data received, %d bytes skipped\n",
                        remaining);
                }
                break;
            }
        case MSG_SMS_RF_TUNE_RES:
            TRACE(TRACE_USB_FLOW,"handling MSG_SMS_RF_TUNE_RES\n");
            sms1xxx_demux_pesbuf_reset(sc, 1, "channel zap");
            DLG_COMPLETE(sc->dlg_status, DLG_TUNE_DONE);
            break;
        /* SMS1000 statistics message */
        case MSG_SMS_GET_STATISTICS_RES:
            TRACE(TRACE_USB_FLOW,"handling MSG_SMS_GET_STATISTICS_RES\n");
            struct SmsMsgStatisticsInfo_ST *p =
                (struct SmsMsgStatisticsInfo_ST *)(phdr + 1);
            sc->sms_stat_dvb.ReceptionData.IsDemodLocked =
                p->Stat.IsDemodLocked;
            sc->sms_stat_dvb.ReceptionData.SNR = p->Stat.SNR;
            sc->sms_stat_dvb.ReceptionData.BER = p->Stat.BER;
            sc->sms_stat_dvb.ReceptionData.ErrorTSPackets =
                p->Stat.BERErrorCount;
            sc->sms_stat_dvb.ReceptionData.InBandPwr = p->Stat.InBandPwr;
            stats_updated = 1;
            DLG_COMPLETE(sc->dlg_status, DLG_STAT_DONE);
            break;
        /* SMS11xx statistics messages */
        case MSG_SMS_SIGNAL_DETECTED_IND:
            TRACE(TRACE_USB_FLOW,"handling MSG_SMS_SIGNAL_DETECTED_IND\n");
            sc->sms_stat_dvb.TransmissionData.IsDemodLocked = 1;
            stats_updated = 1;
            break;
        case MSG_SMS_NO_SIGNAL_IND:
            TRACE(TRACE_USB_FLOW,"handling MSG_SMS_NO_SIGNAL_IND\n");
            sc->sms_stat_dvb.TransmissionData.IsDemodLocked = 0;
            stats_updated = 1;
            break;
        case MSG_SMS_TRANSMISSION_IND:
            {
                TRACE(TRACE_USB_FLOW,"handling MSG_SMS_TRANSMISSION_IND\n");

                u32 *pMsgData = (u32 *) phdr + 1;

                pMsgData++;
                memcpy(&sc->sms_stat_dvb.TransmissionData, pMsgData,
                        sizeof(struct TRANSMISSION_STATISTICS_S));

                CORRECT_STAT_BANDWIDTH(sc->sms_stat_dvb.TransmissionData);
                CORRECT_STAT_TRANSMISSON_MODE(
                        sc->sms_stat_dvb.TransmissionData);

                stats_updated = 1;
                break;
            }
        case MSG_SMS_HO_PER_SLICES_IND:
            {
                TRACE(TRACE_USB_FLOW,"handling MSG_SMS_HO_PER_SLICES_IND\n");

                u32 *pMsgData = (u32 *) phdr + 1;
                struct RECEPTION_STATISTICS_S *pReceptionData =
                        &sc->sms_stat_dvb.ReceptionData;
                struct SRVM_SIGNAL_STATUS_S SignalStatusData;

                pMsgData++;
                SignalStatusData.result = pMsgData[0];
                SignalStatusData.snr = pMsgData[1];
                SignalStatusData.inBandPower = (s32) pMsgData[2];
                SignalStatusData.tsPackets = pMsgData[3];
                SignalStatusData.etsPackets = pMsgData[4];
                SignalStatusData.constellation = pMsgData[5];
                SignalStatusData.hpCode = pMsgData[6];
                SignalStatusData.tpsSrvIndLP = pMsgData[7] & 0x03;
                SignalStatusData.tpsSrvIndHP = pMsgData[8] & 0x03;
                SignalStatusData.cellId = pMsgData[9] & 0xFFFF;
                SignalStatusData.reason = pMsgData[10];
                SignalStatusData.requestId = pMsgData[11];
                pReceptionData->IsRfLocked = pMsgData[16];
                pReceptionData->IsDemodLocked = pMsgData[17];
                pReceptionData->ModemState = pMsgData[12];
                pReceptionData->SNR = pMsgData[1];
                pReceptionData->BER = pMsgData[13];
                pReceptionData->RSSI = pMsgData[14];
                CORRECT_STAT_RSSI(sc->sms_stat_dvb.ReceptionData);

                pReceptionData->InBandPwr = (s32) pMsgData[2];
                pReceptionData->CarrierOffset = (s32) pMsgData[15];
                pReceptionData->TotalTSPackets = pMsgData[3];
                pReceptionData->ErrorTSPackets = pMsgData[4];

                /* TS PER */
                if ((SignalStatusData.tsPackets + SignalStatusData.etsPackets)
                        > 0) {
                    pReceptionData->TS_PER = (SignalStatusData.etsPackets
                            * 100) / (SignalStatusData.tsPackets
                            + SignalStatusData.etsPackets);
                } else {
                    pReceptionData->TS_PER = 0;
                }

                pReceptionData->BERBitCount = pMsgData[18];
                pReceptionData->BERErrorCount = pMsgData[19];

                pReceptionData->MRC_SNR = pMsgData[20];
                pReceptionData->MRC_InBandPwr = pMsgData[21];
                pReceptionData->MRC_RSSI = pMsgData[22];

                stats_updated = 1;
                break;
            }
        case MSG_SMS_GET_VERSION_EX_RES:
            {
#ifdef SMS1XXX_DEBUG
                if(sms1xxxdbg & TRACE_USB_FLOW) {
                    struct SmsVersionRes_ST *ver =
                        (struct SmsVersionRes_ST *) phdr;
                    TRACE(TRACE_USB_FLOW,
                        "handling MSG_SMS_GET_VERSION_EX_RES\n");
                    TRACE(TRACE_USB_FLOW,
                        "dumping version information :\n");
                
                    char label[MSG_VER_LABEL_SIZE + 1];
                    strncpy(label, ver->TextLabel, sizeof(label));
                    printf("+ label:        %s\n", label);
                    printf("+ chipset:      0x%x\n", ver->ChipModel);
                    printf("+ firmware id:  %d\n", ver->FirmwareId);
                    printf("+ supp. protos: 0x%x\n", ver->SupportedProtocols);
                    printf("+ version:      %d.%d\n", ver->VersionMajor,
                        ver->VersionMinor);
                    printf("+ romversion:   %d.%d\n", ver->RomVersionMajor,
                        ver->RomVersionMinor);
                }
#endif
                break;
            }
        case MSG_SMS_GET_PID_FILTER_LIST_RES:
        /* 'REQ' instead of 'RES' : FAMILY1 firmware bug ? */
        case MSG_SMS_GET_PID_FILTER_LIST_REQ:
            {
                TRACE(TRACE_USB_FLOW,
                    "handling MSG_SMS_GET_PID_FILTER_LIST_RES\n");

#ifdef SMS1XXX_DEBUG
                struct SmsMsgData_ST *pdata =
                    (struct SmsMsgData_ST *)((u8 *) packet);
                u32 nfilt = pdata->msgData[1];
                u32 *filterList = &(pdata->msgData[2]);

                TRACE(TRACE_FILTERS, "found %d filters in stack\n", nfilt);
                for(int i = 0; i < nfilt; ++i) {
                    TRACE(TRACE_FILTERS, "filter %u(0x%x) set\n",
                        filterList[i], filterList[i]);
                }
#endif
                DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
                break;
            }
        case MSG_SMS_ADD_PID_FILTER_RES:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_ADD_PID_FILTER_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
            break;
        case MSG_SMS_REMOVE_PID_FILTER_RES:
            TRACE(TRACE_USB_FLOW,
                "handling MSG_SMS_REMOVE_PID_FILTER_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
            break;
        case MSG_SMS_INIT_DEVICE_RES:
            TRACE(TRACE_USB_FLOW,
                "handling MSG_SMS_INIT_DEVICE_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_INIT_DONE);
            break;
        case MSG_SW_RELOAD_START_RES:
            TRACE(TRACE_USB_FLOW,
                "handling MSG_SW_RELOAD_START_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_RELOAD_START_DONE);
            break;
        case MSG_SMS_DATA_DOWNLOAD_RES:
            TRACE(TRACE_USB_FLOW,
                "handling MSG_SMS_DATA_DOWNLOAD_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_DATA_DOWNLOAD_DONE);
            break;
        case MSG_SMS_SWDOWNLOAD_TRIGGER_RES:
            TRACE(TRACE_USB_FLOW,
                "handling MSG_SMS_SWDOWNLOAD_TRIGGER_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_SWDOWNLOAD_TRIGGER_DONE);
            break;
        case MSG_SMS_START_IR_RES:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_START_IR_RES\n");
            INFO("IR module started\n");
            DLG_COMPLETE(sc->dlg_status, DLG_IR_DONE);
            break;
        case MSG_SMS_IR_SAMPLES_IND:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_IR_SAMPLES_IND\n");
            sms1xxx_ir_put_packet(sc,
                (const u8 *)((u8 *)phdr + sizeof(struct SmsMsgHdr_ST)),
                (u32)phdr->msgLength - sizeof(struct SmsMsgHdr_ST));
            break;
        case MSG_SMS_GPIO_CONFIG_RES:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_GPIO_CONFIG_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_GPIO_CONFIG_DONE);
            break;
        case MSG_SMS_GPIO_CONFIG_EX_RES:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_GPIO_CONFIG_EX_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_GPIO_CONFIG_DONE);
            break;
        case MSG_SMS_GPIO_SET_LEVEL_RES:
            TRACE(TRACE_USB_FLOW, "handling MSG_SMS_GPIO_SET_LEVEL_RES\n");
            DLG_COMPLETE(sc->dlg_status, DLG_GPIO_SET_DONE);
            break;
        case MSG_SMS_GPIO_GET_LEVEL_RES:
            {
                TRACE(TRACE_USB_FLOW, "handling MSG_SMS_GPIO_GET_LEVEL_RES\n");
                u32 *msgdata = (u32 *)phdr;
                sc->gpio.get_res = msgdata[1];
                DLG_COMPLETE(sc->dlg_status, DLG_GPIO_GET_DONE);
                break;
            }
        default:
            TRACE(TRACE_USB_FLOW,"unhandled msg type (%d)\n", phdr->msgType);
#ifdef SMS1XXX_DEBUG
            /* Dump raw data */
            TRACE(TRACE_USB_DUMP,"dumping %d bytes :\n", bytes);
            if(sms1xxxdbg & TRACE_USB_DUMP)
                sms1xxx_dump_data((const u8*)phdr, bytes, '+');
#endif
            break;
    }

    /* Update frontend status given received stats */
    if (stats_updated) {
        if (sc->sms_stat_dvb.ReceptionData.IsDemodLocked) {
            sc->fe_status = FE_HAS_SIGNAL | FE_HAS_CARRIER
                | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;

        } else {
            sc->fe_status = 0;
        }
    }
}

static void
sms1xxx_usb_tx_cb(struct usb_xfer *xfer, usb_error_t error)
{
    struct sms1xxx_softc *sc = usbd_xfer_softc(xfer);
    struct usb_page_cache *pc;
    struct sms1xxx_data *data;

    mtx_assert(&sc->sc_mtx, MA_OWNED);

    if(sc->sc_dying)
        return;

    switch(USB_GET_STATE(xfer)) {
        case USB_ST_TRANSFERRED:
            data = STAILQ_FIRST(&sc->sc_tx_active);
            if (data == NULL)
                goto tx_setup;
            STAILQ_REMOVE_HEAD(&sc->sc_tx_active, next);
            STAILQ_INSERT_TAIL(&sc->sc_tx_inactive, data, next);
            /* FALLTHROUGH */
        case USB_ST_SETUP:
        tx_setup:
            data = STAILQ_FIRST(&sc->sc_tx_pending);
            if (data == NULL)
                return;
            STAILQ_REMOVE_HEAD(&sc->sc_tx_pending, next);
            pc = usbd_xfer_get_frame(xfer, 0);
            usbd_copy_in(pc, 0, data->buf, data->buflen); 
            usbd_xfer_set_frame_len(xfer, 0, data->buflen);
            STAILQ_INSERT_TAIL(&sc->sc_tx_active, data, next);
            usbd_transfer_submit(xfer);
            break;
        default:
            break;
    }
    return;
}

/* Main callback function, calls sms1xxx_usb_get_packets */
static void
sms1xxx_usb_rx_cb(struct usb_xfer *xfer, usb_error_t error)
{
    struct sms1xxx_softc *sc = usbd_xfer_softc(xfer);

    mtx_assert(&sc->sc_mtx, MA_OWNED);

    if(sc->sc_dying)
        return;

    switch(USB_GET_STATE(xfer)) {
    case USB_ST_TRANSFERRED:
        usbd_xfer_frame_data(xfer, 0, (void*)&sc->sc_rx_data.buf,
            &sc->sc_rx_data.buflen);
#ifdef SMS1XXX_DIAGNOSTIC
        sc->stats.interrupts++;
        sc->stats.bytes += sc->sc_rx_data.buflen;
#endif
        sms1xxx_usb_get_packets(sc, sc->sc_rx_data.buf, sc->sc_rx_data.buflen);
        /* FALLTHROUGH */
    case USB_ST_SETUP:
    rx_setup:
        usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
        usbd_transfer_submit(xfer);
        break;
    default:
        if (error != USB_ERR_CANCELLED) {
            /* try to clear stall first */
            usbd_xfer_set_stall(xfer);
            goto rx_setup;
        }
        break;
    }
    return;
}

int
sms1xxx_usb_xfers_start(struct sms1xxx_softc *sc)
{
    int i;

    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

    if(sc->usbinit <= 0) {
        ERR("usb not initialized\n");
        return (ENXIO);
    }

    /* Output : initialize TX STAILQs */
    STAILQ_INIT(&sc->sc_tx_inactive);
    STAILQ_INIT(&sc->sc_tx_pending);
    STAILQ_INIT(&sc->sc_tx_active);
    for (i = 0; i < SMS1XXX_BULK_TX_BUFS; i++) {
        struct sms1xxx_data *data = &sc->sc_tx_data[i];
        data->buf =
            malloc(SMS1XXX_BULK_TX_BUFS_SIZE, M_USBDEV, M_NOWAIT | M_ZERO);
        if (data->buf == NULL) {
            TRACE(TRACE_USB,"could not allocate TX buffer!\n");
            return (ENOMEM);
        }
        STAILQ_INSERT_TAIL(&sc->sc_tx_inactive, data, next);
    }

    /* Input : RX transfer */
    mtx_lock(&sc->sc_mtx);
    usbd_transfer_start(sc->sc_xfer[SMS1XXX_BULK_RX]);
    mtx_unlock(&sc->sc_mtx);

    return (0);
}

int
sms1xxx_usb_xfers_stop(struct sms1xxx_softc *sc)
{
    int i;

    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

    if(sc->usbinit <= 0) {
        ERR("usb not initialized\n");
        return (ENXIO);
    }

    mtx_lock(&sc->sc_mtx);
    usbd_transfer_stop(sc->sc_xfer[SMS1XXX_BULK_RX]);
    mtx_unlock(&sc->sc_mtx);
    usbd_transfer_drain(sc->sc_xfer[SMS1XXX_BULK_RX]);

    /* Input : RX transfer */
    mtx_lock(&sc->sc_mtx);
    usbd_transfer_stop(sc->sc_xfer[SMS1XXX_BULK_TX]);
    mtx_unlock(&sc->sc_mtx);
    usbd_transfer_drain(sc->sc_xfer[SMS1XXX_BULK_TX]);

    /* Output : un-initialize TX STAILQs */
    for (i = 0; i < SMS1XXX_BULK_TX_BUFS; i++) {
        struct sms1xxx_data *data = &sc->sc_tx_data[i];
        free(data->buf, M_USBDEV);
    }

    return (0);
}

/*******************
 * Write functions *
 *******************/

/* Write data to the device */
int
sms1xxx_usb_rawwrite(struct sms1xxx_softc *sc, const u8 *wbuf, u32 wlen)
{
    int offset, bsize;
    struct sms1xxx_data *data;

    TRACE(TRACE_USB_FLOW,"wlen=%d\n",wlen);
    if(sc->usbinit <= 0) {
        ERR("usb not initialized\n");
        return (ENXIO);
    }

    for (offset = 0; offset < wlen; offset += bsize) {
        if ((wlen - offset) > SMS1XXX_BULK_TX_BUFS_SIZE)
            bsize = SMS1XXX_BULK_TX_BUFS_SIZE;
        else
            bsize = wlen - offset;

        mtx_lock(&sc->sc_mtx);
        data = STAILQ_FIRST(&sc->sc_tx_inactive);
        if (data == NULL) {
            /* Wait a few ms for free buffers */
            usb_pause_mtx(&sc->sc_mtx, USB_MS_TO_TICKS(5));
            data = STAILQ_FIRST(&sc->sc_tx_inactive);
            if (data == NULL) {
                mtx_unlock(&sc->sc_mtx);
                return (ENOBUFS);
            }
        }
        STAILQ_REMOVE_HEAD(&sc->sc_tx_inactive, next);
        memcpy(data->buf, wbuf + offset, bsize);
        data->buflen = bsize;
        STAILQ_INSERT_TAIL(&sc->sc_tx_pending, data, next);

        usbd_transfer_start(sc->sc_xfer[SMS1XXX_BULK_TX]);
        mtx_unlock(&sc->sc_mtx);
    }
    TRACE(TRACE_USB_FLOW,"sent %d bytes\n",offset);
    return (0);
}

/* Write data to the device, handling endianness conversions */
int
sms1xxx_usb_write(struct sms1xxx_softc *sc, u8 *wbuf, u32 wlen)
{
    TRACE(TRACE_USB_FLOW,"\n");

    sms1xxx_endian_handle_message_header(wbuf);
    return (sms1xxx_usb_rawwrite(sc, (const u8 *)wbuf, wlen));
}


/* Wait for a response from the device
   XXX To be improved */
static int
sms1xxx_usb_wait(struct sms1xxx_softc *sc,
    u16 completion, unsigned int delay_ms)
{
    struct timeval tv_start;
    struct timeval tv;
    int ms;

    microtime(&tv_start);

    do {
        pause("pause",5);

        microtime(&tv);
        timevalsub(&tv,&tv_start);
        ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;

        if(sc->sc_dying) {
            TRACE(TRACE_MODULE,"dying! sc=%p\n",sc);
            return (ECANCELED);
        }

        if (DLG_ISCOMPLETE(sc->dlg_status, completion))
            return (0);
    }
    while (ms < delay_ms);

    return (ETIMEDOUT);
}

/* Write data to the device and wait for a response */
int
sms1xxx_usb_rawwrite_and_wait(struct sms1xxx_softc *sc, const u8 *wbuf,
    u32 wlen, u16 completion, unsigned int delay_ms)
{
    int err = 0;

    TRACE(TRACE_USB_FLOW,"completion=%u(0x%x),delay_ms=%d\n",completion,
        completion,delay_ms);

    DLG_INIT(sc->dlg_status, completion);
    err = sms1xxx_usb_rawwrite(sc, wbuf, wlen);
    if (err == 0) {
        err = sms1xxx_usb_wait(sc, completion, delay_ms);
    }

    TRACE(TRACE_USB_FLOW,"done,completion=%u(0x%x),err=%d\n",completion,
        completion,err);

    return (err);
}

/* Write data to the device and wait for a response,
   handling endianness conversions */
int
sms1xxx_usb_write_and_wait(struct sms1xxx_softc *sc, u8 *wbuf,
    u32 wlen, u16 completion, unsigned int delay_ms)
{
    TRACE(TRACE_USB_FLOW,"\n");

    sms1xxx_endian_handle_message_header(wbuf);
    return (sms1xxx_usb_rawwrite_and_wait(sc, (const u8 *)wbuf,
        wlen, completion, delay_ms));
}

/***************************
 * Higher-level operations *
 ***************************/

/* Set requested mode and ask device to reattach */
int
sms1xxx_usb_setmode(struct sms1xxx_softc *sc, int mode)
{
    TRACE(TRACE_USB,"mode=%d\n",mode);

    struct SmsMsgHdr_ST Msg = {
        MSG_SW_RELOAD_REQ,
        0,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    if (sms1xxx_firmware_name(sc->sc_type, mode) == NULL) {
        ERR("invalid mode specified %d\n", mode);
        return (EINVAL);
    }

    /* Set requested mode and ask device to re-attach */
    sc->mode = DEVICE_MODE_NONE ;
    sc->device->requested_mode = mode ;
    sc->device->fw_loading = FALSE ;
    INFO("asking device to reset, target mode=%d\n",mode);

    return (sms1xxx_usb_write(sc, (u8*)&Msg, sizeof(Msg)));
}

/* Initialize the device after firmware loading */
int
sms1xxx_usb_initdevice(struct sms1xxx_softc *sc, int mode)
{
    TRACE(TRACE_USB,"mode=%d\n", mode);

    struct SmsMsgData_ST InitMsg;

    if ((sms1xxx_firmware_name(sc->sc_type, mode) == NULL) ||
        (mode != sc->device->requested_mode)) {
        ERR("invalid mode specified %d\n", mode);
        return (EINVAL);
    }

    InitMsg.xMsgHeader.msgType  = MSG_SMS_INIT_DEVICE_REQ;
    InitMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    InitMsg.xMsgHeader.msgDstId = HIF_TASK;
    InitMsg.xMsgHeader.msgLength = sizeof(InitMsg);
    InitMsg.xMsgHeader.msgFlags = 0;
    InitMsg.msgData[0] = mode;

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&InitMsg, sizeof(InitMsg),
        DLG_INIT_DONE, FRONTEND_TIMEOUT));
}

/* Prepare a *warm* device to firmware upload */
int
sms1xxx_usb_reloadstart(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgHdr_ST Msg = {
        MSG_SW_RELOAD_START_REQ,
        0,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_RELOAD_START_DONE, FRONTEND_TIMEOUT));
}

/* Execute firmware on a *warm* device after firmware upload */
/* XXX Does not seem to work : alignment pb ? */
int
sms1xxx_usb_reloadexec(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgHdr_ST Msg = {
        MSG_SW_RELOAD_EXEC_REQ,
        0,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    return (sms1xxx_usb_write(sc, (u8*)&Msg, sizeof(Msg)));
}

/* Execute firmware on a *cold* device after firmware upload */
int
sms1xxx_usb_swdtrigger(struct sms1xxx_softc *sc, u32 start_address)
{
    int err = 0;

    TRACE(TRACE_USB,"start_address=%d\n", start_address);

    /* SmsMsgData_ST with extra data */
    int msgSize = sizeof(struct SmsMsgData_ST) + (sizeof(u32) * 4);

    struct SmsMsgData_ST *TriggerMsg = malloc(msgSize, M_USBDEV, M_WAITOK);
    if(TriggerMsg == NULL) {
        ERR("could not allocate msg buffer\n");
        return (ENOMEM);
    }

    TriggerMsg->xMsgHeader.msgType = MSG_SMS_SWDOWNLOAD_TRIGGER_REQ;
    TriggerMsg->xMsgHeader.msgSrcId = 0;
    TriggerMsg->xMsgHeader.msgDstId = HIF_TASK;
    TriggerMsg->xMsgHeader.msgLength = msgSize;
    TriggerMsg->xMsgHeader.msgFlags = 0;

    TriggerMsg->msgData[0] = start_address; /* Entry point */
    TriggerMsg->msgData[1] = 5;             /* Priority */
    TriggerMsg->msgData[2] = 0x200;         /* Stack size */
    TriggerMsg->msgData[3] = 0;             /* Parameter */
    TriggerMsg->msgData[4] = 4;             /* Task ID */

    sms1xxx_endian_handle_tx_message(TriggerMsg);
    /* XXX honour SMS_ROM_NO_RESPONSE and use sms1xxx_usb_write() ? */
    err = sms1xxx_usb_write_and_wait(sc, (u8*)TriggerMsg,
        msgSize, DLG_SWDOWNLOAD_TRIGGER_DONE, FRONTEND_TIMEOUT);

    free(TriggerMsg, M_USBDEV);
    return (err);
}

/* Get device version information
   The result is only used for information purpose ;
   it is not (yet ?) used by the driver
   XXX Seems to work on FAMILY2 devices only */
int
sms1xxx_usb_getversion(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgHdr_ST Msg = {
        MSG_SMS_GET_VERSION_EX_REQ,
        0,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    return sms1xxx_usb_write(sc, (u8*)&Msg, sizeof(Msg));
}

/* Get device statistics on SMS1000 (useless on
   SMS11xx, since stats are regularly received
   in *_IND messages) */
int
sms1xxx_usb_getstatistics(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgHdr_ST Msg = {
        MSG_SMS_GET_STATISTICS_REQ,
        DVBT_BDA_CONTROL_MSG_ID,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_STAT_DONE, FRONTEND_TIMEOUT));
}

/* Get hardware PID filter list from device
   This list is only used for information purpose ;
   it is not (yet ?) used by the driver */
int
sms1xxx_usb_getpidfilterlist(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgHdr_ST Msg = {
        MSG_SMS_GET_PID_FILTER_LIST_REQ,
        DVBT_BDA_CONTROL_MSG_ID,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_PID_DONE, FRONTEND_TIMEOUT));
}

/* Tune device to a given frequency / bandwidth */
int
sms1xxx_usb_setfrequency(struct sms1xxx_softc *sc, u32 frequency,
    fe_bandwidth_t bandwidth)
{
    INFO("set frequency=%u, bandwidth=%u\n", frequency, bandwidth);

    struct {
        struct SmsMsgHdr_ST Msg;
        u32    Data[3];
    } Msg;

    if((frequency < sc->device->frontend->info.frequency_min) ||
       (frequency > sc->device->frontend->info.frequency_max)) {
        ERR("invalid frequency specified %d\n", frequency);
        return (EINVAL);
    }

    sc->fe_status = FE_HAS_SIGNAL;

    Msg.Msg.msgType   = MSG_SMS_RF_TUNE_REQ;
    Msg.Msg.msgSrcId  = DVBT_BDA_CONTROL_MSG_ID;
    Msg.Msg.msgDstId  = HIF_TASK;
    Msg.Msg.msgLength = sizeof(Msg);
    Msg.Msg.msgFlags  = 0;
    Msg.Data[0] = frequency;
    switch (bandwidth) {
        case BANDWIDTH_8_MHZ: Msg.Data[1] = BW_8_MHZ; break;
        case BANDWIDTH_7_MHZ: Msg.Data[1] = BW_7_MHZ; break;
        case BANDWIDTH_6_MHZ: Msg.Data[1] = BW_6_MHZ; break;
        case BANDWIDTH_AUTO: return (EOPNOTSUPP);
        default: return (EINVAL);
    }
    Msg.Data[2] = 12000000;

    /* Try to enable LNA first, if requested (and supported) */
    if(sc->gpio.use_lna != 0) {
        /* Enable LNA */
        if(sc->gpio.use_lna == 1)
            sms1xxx_gpio_lna_control(sc, 1);
        /* Explicitly disable LNA */
        else if(sc->gpio.use_lna == 2)
            sms1xxx_gpio_lna_control(sc, 0);
    }

    /* see cmcdvb.c, called via smsdvb_sendrequest_and_wait() */
    sms1xxx_endian_handle_tx_message(&Msg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_TUNE_DONE, FRONTEND_TIMEOUT));
}

/* Add a PID to hardware PID filter list */
int
sms1xxx_usb_add_pid(struct sms1xxx_softc *sc, u16 pid)
{
    TRACE(TRACE_USB,"add PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return (EINVAL);
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_ADD_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    sms1xxx_endian_handle_tx_message(&PidMsg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT));
}

/* Remove a PID from hardware PID filter list */
int
sms1xxx_usb_remove_pid(struct sms1xxx_softc *sc, u16 pid)
{
    TRACE(TRACE_USB,"remove PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return (EINVAL);
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_REMOVE_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    sms1xxx_endian_handle_tx_message(&PidMsg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT));
}

/************
 * Infrared *
 ************/

/* Start IR module */
int
sms1xxx_usb_ir_start(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    struct SmsMsgData_ST2 Msg;
    Msg.xMsgHeader.msgType = MSG_SMS_START_IR_REQ;
    Msg.xMsgHeader.msgSrcId = 0;
    Msg.xMsgHeader.msgDstId = HIF_TASK;
    Msg.xMsgHeader.msgLength = sizeof(struct SmsMsgData_ST2);
    Msg.xMsgHeader.msgFlags = 0;
    Msg.msgData[0] = sc->ir.controller;
    Msg.msgData[1] = sc->ir.timeout;

    sms1xxx_endian_handle_tx_message(&Msg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_IR_DONE, FRONTEND_TIMEOUT));
}

/********
 * GPIO *
 ********/

/* Configure GPIO */
int
sms1xxx_usb_gpio_configure(struct sms1xxx_softc *sc, u32 pin_num,
    struct sms1xxx_gpio_config *gpio_config)
{
    TRACE(TRACE_USB,"pin_num=%d\n", pin_num);

    u32 Translatedpin_num = 0;
    u32 GroupNum = 0;
    u32 ElectricChar;
    u32 groupCfg;

    struct SetGpioMsg {
        struct SmsMsgHdr_ST xMsgHeader;
        u32 msgData[6];
    } Msg;

    if (pin_num > MAX_GPIO_PIN_NUMBER)
        return (EINVAL);

    if (gpio_config == NULL)
        return (EINVAL);

    Msg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    Msg.xMsgHeader.msgDstId = HIF_TASK;
    Msg.xMsgHeader.msgFlags = 0;
    Msg.xMsgHeader.msgLength = (u16) sizeof(struct SetGpioMsg);
    Msg.msgData[0] = pin_num;

    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) != SMS1XXX_FAMILY2) {
        Msg.xMsgHeader.msgType = MSG_SMS_GPIO_CONFIG_REQ;
        if (sms1xxx_gpio_get_pin_params(pin_num, &Translatedpin_num,
            &GroupNum, &groupCfg) != 0)
            return (EINVAL);

        Msg.msgData[1] = Translatedpin_num;
        Msg.msgData[2] = GroupNum;
        ElectricChar = (gpio_config->PullUpDown)
            | (gpio_config->InputCharacteristics << 2)
            | (gpio_config->OutputSlewRate << 3)
            | (gpio_config->OutputDriving << 4);
        Msg.msgData[3] = ElectricChar;
        Msg.msgData[4] = gpio_config->Direction;
        Msg.msgData[5] = groupCfg;
    } else {
        Msg.xMsgHeader.msgType = MSG_SMS_GPIO_CONFIG_EX_REQ;
        Msg.msgData[1] = gpio_config->PullUpDown;
        Msg.msgData[2] = gpio_config->OutputSlewRate;
        Msg.msgData[3] = gpio_config->OutputDriving;
        Msg.msgData[4] = gpio_config->Direction;
        Msg.msgData[5] = 0;
    }

    sms1xxx_endian_handle_tx_message(&Msg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_GPIO_CONFIG_DONE, FRONTEND_TIMEOUT));
}

/* Set GPIO pin level */
int
sms1xxx_usb_gpio_set_level(struct sms1xxx_softc *sc, u8 pin_num, u8 new_level) {
    TRACE(TRACE_USB, "pin_num=%d, new_level=%d\n", pin_num, new_level);

    struct SetGpioMsg {
        struct SmsMsgHdr_ST xMsgHeader;
        u32 msgData[3]; /* keep it 3 ! */
    } Msg;

    if ((new_level > 1) || (pin_num > MAX_GPIO_PIN_NUMBER) ||
        (pin_num > MAX_GPIO_PIN_NUMBER))
        return (EINVAL);

    Msg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    Msg.xMsgHeader.msgDstId = HIF_TASK;
    Msg.xMsgHeader.msgFlags = 0;
    Msg.xMsgHeader.msgType = MSG_SMS_GPIO_SET_LEVEL_REQ;
    Msg.xMsgHeader.msgLength = (u16) sizeof(struct SetGpioMsg);
    Msg.msgData[0] = pin_num;
    Msg.msgData[1] = new_level;

    sms1xxx_endian_handle_tx_message(&Msg);
    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_GPIO_SET_DONE, FRONTEND_TIMEOUT));
}

/* Get GPIO pin level */
int
sms1xxx_usb_gpio_get_level(struct sms1xxx_softc *sc, u8 pin_num,
    u8 *level) {
    TRACE(TRACE_USB, "pin_num=%d\n", pin_num);

    int rc = 0;

    struct SetGpioMsg {
        struct SmsMsgHdr_ST xMsgHeader;
        u32 msgData[2];
    } Msg;

    if (pin_num > MAX_GPIO_PIN_NUMBER)
        return (EINVAL);

    Msg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    Msg.xMsgHeader.msgDstId = HIF_TASK;
    Msg.xMsgHeader.msgFlags = 0;
    Msg.xMsgHeader.msgType = MSG_SMS_GPIO_GET_LEVEL_REQ;
    Msg.xMsgHeader.msgLength = (u16) sizeof(struct SetGpioMsg);
    Msg.msgData[0] = pin_num;
    Msg.msgData[1] = 0;

    sms1xxx_endian_handle_tx_message(&Msg);
    rc = sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_GPIO_GET_DONE, FRONTEND_TIMEOUT);

    *level = sc->gpio.get_res; /* XXX implicit cast (u8)sc->gpio_get_res */
    return (rc);
}
