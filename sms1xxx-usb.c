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
               .flags = {
                   .short_xfer_ok = 1,
               },
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
    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

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
sms1xxx_usb_get_packets(struct sms1xxx_softc *sc, u8 *packet,
    u32 bytes)
{
    if (bytes > 0) {
        struct SmsMsgHdr_ST *phdr = (struct SmsMsgHdr_ST *)((u8 *) packet);

        TRACE(TRACE_USB_FLOW,"got %d bytes "
            "(msgLength=%d,msgType=%d,msgFlags=%d)\n",
            bytes,phdr->msgLength,phdr->msgType,phdr->msgFlags);

        if (bytes < phdr->msgLength) {
            ERR("invalid message : msgLength=%d, received %d",
                phdr->msgLength,bytes);
            return;
        }

        bytes = phdr->msgLength;

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
                DLG_COMPLETE(sc->dlg_status, DLG_TUNE_DONE);
                break;
            case MSG_SMS_GET_STATISTICS_RES:
                TRACE(TRACE_USB_FLOW,"handling MSG_SMS_GET_STATISTICS_RES\n");
                struct SmsMsgStatisticsInfo_ST *p =
                    (struct SmsMsgStatisticsInfo_ST *)(phdr + 1);

                if (p->Stat.IsDemodLocked) {
                    sc->fe_status = FE_HAS_SIGNAL |
                                    FE_HAS_CARRIER |
                                    FE_HAS_VITERBI |
                                    FE_HAS_SYNC |
                                    FE_HAS_LOCK;

                    sc->fe_snr = p->Stat.SNR;
                    sc->fe_ber = p->Stat.BER;
                    sc->fe_unc = p->Stat.BERErrorCount;

                    if (p->Stat.InBandPwr < -95)
                        sc->fe_signal_strength = 0;
                    else if (p->Stat.InBandPwr > -29)
                        sc->fe_signal_strength = 100;
                    else
                        sc->fe_signal_strength =
                            (p->Stat.InBandPwr + 95) * 3 / 2;
                }
                else {
                    sc->fe_status = 0;
                    sc->fe_snr =
                    sc->fe_ber =
                    sc->fe_unc =
                    sc->fe_signal_strength = 0;
                }
                DLG_COMPLETE(sc->dlg_status, DLG_STAT_DONE);
                break;
            case MSG_SMS_GET_VERSION_EX_RES:
                {
/*  XXX No response : alignment pb ? */
/*
                    struct SmsVersionRes_ST *ver =
                        (struct SmsVersionRes_ST *) phdr;
                    TRACE(TRACE_USB_FLOW,"handling MSG_SMS_GET_VERSION_EX_RES "
                          "firmw=%d, protos=0x%x, ver=%d.%d\n",
                          ver->FirmwareId, ver->SupportedProtocols,
                          ver->RomVersionMajor, ver->RomVersionMinor);
*/
                    break;
                }
            case MSG_SMS_GET_PID_FILTER_LIST_REQ:
                /* 'REQ' instead of 'RES' : not a typo ! */
                {
                    TRACE(TRACE_USB_FLOW,
                        "handling MSG_SMS_GET_PID_FILTER_LIST_REQ\n");

#ifdef USB_DEBUG
                    struct SmsMsgData_ST *pdata =
                        (struct SmsMsgData_ST *)((u8 *) packet);
                    u32 nfilt = pdata->msgData[1];
                    u32 *filterList = &(pdata->msgData[2]);

                    TRACE(TRACE_FILTERS, "found %d filters in stack\n", nfilt);
                    for(int i = 0; i < nfilt; ++i) {
                        TRACE(TRACE_FILTERS,"filter %u(0x%x) set\n",
                            filterList[i], filterList[i]);
                    }
#endif
                    DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
                    break;
                }
            case MSG_SMS_ADD_PID_FILTER_RES:
                TRACE(TRACE_USB_FLOW,"handling MSG_SMS_ADD_PID_FILTER_RES\n");
                DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
                break;
            case MSG_SMS_REMOVE_PID_FILTER_RES:
                TRACE(TRACE_USB_FLOW,
                    "handling MSG_SMS_REMOVE_PID_FILTER_RES\n");
                DLG_COMPLETE(sc->dlg_status, DLG_PID_DONE);
                break;
            default:
                TRACE(TRACE_USB_FLOW,"unhandled msg type\n");
                break;
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
        usbd_xfer_frame_data(xfer, 0, (void*)&sc->sc_rx_data.buf, &sc->sc_rx_data.buflen);
#ifdef DIAGNOSTIC
        sc->stats.interrupts++;
        sc->stats.bytes += sc->sc_rx_data.buflen;
#endif
        sms1xxx_usb_get_packets(sc, sc->sc_rx_data.buf, sc->sc_rx_data.buflen);

        if(sc->dvr.state & DVR_AWAKE) {
            sc->dvr.state &= ~DVR_AWAKE;
            if(sc->dvr.state & DVR_SLEEP)
                wakeup(&sc->dvr);
            if(sc->dvr.state & DVR_POLL) {
                sc->dvr.state &= ~DVR_POLL;
                selwakeuppri(&sc->dvr.rsel, PZERO);
            }
        }
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

    /* Initialize TX STAILQs */
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

    /* Input, only for warm devices */
    if(sc->device->mode != DEVICE_MODE_NONE) {
        /* DVR stuff */
        sc->dvr.threshold = THRESHOLD;
        sc->dvr.size = DVRBUFSIZE;
        if(sc->dvr.buf == NULL) {
            sc->dvr.buf = malloc(sc->dvr.size, M_USBDEV, M_WAITOK);
            if(sc->dvr.buf == NULL) {
                ERR("could not allocate dvr buf\n");
                return (ENOMEM);
            }
            sc->dvr.roff = 0;
            sc->dvr.woff = 0;
            sc->dvr.ravail = 0;
            sc->dvr.wavail = sc->dvr.size;
        }
        mtx_lock(&sc->sc_mtx);
        usbd_transfer_start(sc->sc_xfer[SMS1XXX_BULK_RX]);
        mtx_unlock(&sc->sc_mtx);
    }

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

    /* Input, only for warm devices */
    if(sc->device->mode != DEVICE_MODE_NONE) {
        mtx_lock(&sc->sc_mtx);
        usbd_transfer_stop(sc->sc_xfer[SMS1XXX_BULK_RX]);
        mtx_unlock(&sc->sc_mtx);
        usbd_transfer_drain(sc->sc_xfer[SMS1XXX_BULK_RX]);

        /* DVR */
        sc->dvr.ravail = 0;
        sc->dvr.wavail = 0;
        if(sc->dvr.buf != NULL) {
            free(sc->dvr.buf, M_USBDEV);
            sc->dvr.buf = NULL;
        }
    }

    mtx_lock(&sc->sc_mtx);
    usbd_transfer_stop(sc->sc_xfer[SMS1XXX_BULK_TX]);
    mtx_unlock(&sc->sc_mtx);
    usbd_transfer_drain(sc->sc_xfer[SMS1XXX_BULK_TX]);

    /* Uninitialize TX STAILQs */
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
sms1xxx_usb_write(struct sms1xxx_softc *sc, const u8 *wbuf, u32 wlen)
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

/* Wait for a response from the device
   XXX To be improved */
static int
sms1xxx_usb_wait(struct sms1xxx_softc *sc,
    unsigned char completion, unsigned int delay_ms)
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
            return (USB_ERR_CANCELLED);
        }

        if (DLG_ISCOMPLETE(sc->dlg_status, completion))
            return (0);
    }
    while (ms < delay_ms);

    return (USB_ERR_TIMEOUT);
}

/* Write data to the device and wait for a response */
int
sms1xxx_usb_write_and_wait(struct sms1xxx_softc *sc, const u8 *wbuf,
    u32 wlen, unsigned char completion, unsigned int delay_ms)
{
    int err = 0;

    TRACE(TRACE_USB_FLOW,"completion=%u(0x%x),delay_ms=%d\n",completion,
        completion,delay_ms);

    DLG_INIT(sc->dlg_status, completion);
    err = sms1xxx_usb_write(sc, wbuf, wlen);
    if (err == 0) {
        err = sms1xxx_usb_wait(sc, completion, delay_ms);
    }

    TRACE(TRACE_USB_FLOW,"done, err=%d\n",err);

    return (err);
}

/***************************
 * Higher-level operations *
 ***************************/

/* Set requested mode and ask device to reattach */
int sms1xxx_usb_setmode(struct sms1xxx_softc *sc, int mode)
{
    TRACE(TRACE_USB,"mode=%d\n",mode);

    struct SmsMsgHdr_ST Msg = {
        MSG_SW_RELOAD_REQ,
        0,
        HIF_TASK,
        sizeof(struct SmsMsgHdr_ST),
        0
    };

    if ((mode < DEVICE_MODE_DVBT) ||
        (mode > DEVICE_MODE_DVBT_BDA) ||
        (!sc->device->firmware[mode][0])) {
        ERR("invalid mode specified %d\n", mode);
        return (-EINVAL);
    }

    /* Set requested mode and ask device to re-attach */
    sc->device->mode = DEVICE_MODE_NONE ;
    sc->device->requested_mode = mode ;
    sc->device->fw_loading = FALSE ;
    INFO("asking device to reset, target mode=%d\n",mode);

    return (sms1xxx_usb_write(sc, (u8*)&Msg, sizeof(Msg)));
}

/* Initialize the device *after* firmware loading
   Does not seem to be necessary to make the device
   work, anyway... */
int sms1xxx_usb_initdevice(struct sms1xxx_softc *sc, int mode)
{
    TRACE(TRACE_USB,"mode=%d\n", mode);

    struct SmsMsgData_ST InitMsg;

    if ((mode < DEVICE_MODE_DVBT) ||
        (mode > DEVICE_MODE_DVBT_BDA) ||
        (!sc->device->firmware[mode][0]) ||
        (mode != sc->device->requested_mode)) {
        ERR("invalid mode specified %d\n", mode);
        return (-EINVAL);
    }

    InitMsg.xMsgHeader.msgType  = MSG_SMS_INIT_DEVICE_REQ;
    InitMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    InitMsg.xMsgHeader.msgDstId = HIF_TASK;
    InitMsg.xMsgHeader.msgLength = sizeof(InitMsg);
    InitMsg.xMsgHeader.msgFlags = 0;
    InitMsg.msgData[0] = mode;

    return (sms1xxx_usb_write(sc, (u8*)&InitMsg, sizeof(InitMsg)));
}

/* Get device version information
   The result is only used for information purpose ;
   it is not (yet ?) used by the driver
   XXX No response : alignment pb ? */
/*
int sms1xxx_usb_getversion(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"\n");

    void *buffer = malloc(sizeof(struct SmsMsgHdr_ST) + SMS_DMA_ALIGNMENT,
                   M_USBDEV,M_WAITOK);
    struct SmsMsgHdr_ST *Msg =
        (struct SmsMsgHdr_ST *) SMS_ALIGN_ADDRESS(buffer);
    int rc;

    if (!buffer)
        return (-ENOMEM);

    SMS_INIT_MSG(Msg, MSG_SMS_GET_VERSION_EX_REQ,
             sizeof(struct SmsMsgHdr_ST));

    rc = sms1xxx_usb_write(sc, (u8*)Msg, Msg->msgLength);

    free(buffer,M_USBDEV);

    return (rc);
}
*/

/* Get device statistics */
int sms1xxx_usb_getstatistics(struct sms1xxx_softc *sc)
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
int sms1xxx_usb_getpidfilterlist(struct sms1xxx_softc *sc)
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
int sms1xxx_usb_setfrequency(struct sms1xxx_softc *sc, u32 frequency,
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
        return (-EINVAL);
    }

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
        case BANDWIDTH_AUTO: return (-EOPNOTSUPP);
        default: return (-EINVAL);
    }
    Msg.Data[2] = 12000000;

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_TUNE_DONE, FRONTEND_TIMEOUT));
}

/* Add a PID to hardware PID filter list */
int sms1xxx_usb_add_pid(struct sms1xxx_softc *sc, u16 pid)
{
    TRACE(TRACE_USB,"add PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return (-EINVAL);
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_ADD_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT));
}

/* Remove a PID from hardware PID filter list */
int sms1xxx_usb_remove_pid(struct sms1xxx_softc *sc, u16 pid)
{
    TRACE(TRACE_USB,"remove PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return (-EINVAL);
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_REMOVE_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    return (sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT));
}
