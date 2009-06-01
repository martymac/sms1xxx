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

/****************************
 * Initialization functions *
 ****************************/

/* Initialize USB input/output pipes and xfers */
int
sms1xxx_usb_init(struct sms1xxx_softc *sc)
{
    int i;
    TRACE(TRACE_USB,"init=%d\n",sc->usbinit);

    if(sc->usbinit == -1) {
        ERR("USB Initialisation in progress\n");
        return EBUSY;
    }
    sc->usbinit = -1;

    /* Init control */
    if(sc->opipe == NULL) {
        if(usbd_open_pipe(sc->iface,
            sc->device->bulk_ctrl_endpoint,
            0, &sc->opipe)) {
            ERR("could not open ctrl out pipe\n");
            sc->usbinit = 0;
            return EIO;
        }
    }
    if(sc->oxfer == NULL) {
        if((sc->oxfer = usbd_alloc_xfer(sc->udev)) == NULL) {
            ERR("could not allocate ctrl out xfer\n");
            sc->usbinit = 0;
            return EIO;
        }
    }

    /* Input, only for warm devices */
    if(sc->device->mode != DEVICE_MODE_NONE) {
        /* DVR */
        sc->dvr.threshold = THRESHOLD;
        sc->dvr.size = DVRBUFSIZE;
        if(sc->dvr.buf == NULL) {
            sc->dvr.buf = malloc(sc->dvr.size, M_USBDEV, M_WAITOK);
            if(sc->dvr.buf == NULL) {
                ERR("could not allocate dvr buf\n");
                sc->usbinit = 0;
                return ENOMEM;
            }
            sc->dvr.roff = 0;
            sc->dvr.woff = 0;
            sc->dvr.ravail = 0;
            sc->dvr.wavail = sc->dvr.size;
        }

        /* Pipe and xfers */
        if(sc->ipipe == NULL) {
            if(usbd_open_pipe(sc->iface,
            sc->device->bulk_rcv_endpoint | UT_READ,
            0, &sc->ipipe)) {
                ERR("could not open data in pipe\n");
                sc->usbinit = 0;
                return EIO;
            }
        }

        sc->sbufsize = SBUFSIZE;
        for(i = 0; i < MAXXFERS; i++) {
            usbd_xfer_handle xfer;
            void *p;

            if((xfer = usbd_alloc_xfer(sc->udev)) == NULL) {
                ERR("could not allocate stream xfer %d\n",i+1);
                return EIO;
            }
            if((p = usbd_alloc_buffer(xfer,sc->sbufsize)) == NULL){
                usbd_free_xfer(xfer);
                ERR("could not allocate stream buffer %d\n",i+1);
                return ENOMEM;
            }
            sc->sbuf[i].xfer = xfer;
            sc->sbuf[i].buf = p;
            sc->sbuf[i].sc = sc;
        }
    }

    sc->usbinit = 1;
    return 0;
}

/* Un-initialize USB input/output pipes and xfers */
void
sms1xxx_usb_exit(struct sms1xxx_softc *sc)
{
    int i;
    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

    sc->usbinit = 0;

    /* Cleanup control */
    if(sc->opipe != NULL) {
        usbd_abort_pipe(sc->opipe);
        usbd_close_pipe(sc->opipe);
        sc->opipe = NULL;
    }
    if(sc->oxfer != NULL) {
        usbd_free_xfer(sc->oxfer);
        sc->oxfer = NULL;
    }

    /* Cleanup stream */
    if(sc->ipipe != NULL) {
        usbd_abort_pipe(sc->ipipe);
        usbd_close_pipe(sc->ipipe);
        sc->ipipe = NULL;
    }
    for(i = 0; i < MAXXFERS; ++i) {
        if(sc->sbuf[i].xfer != NULL) {
            /* Implicit buffer free */
            usbd_free_xfer(sc->sbuf[i].xfer);
            sc->sbuf[i].xfer = NULL;
        }
    }

    /* DVR */
    sc->dvr.ravail = 0;
    sc->dvr.wavail = 0;
    if(sc->dvr.buf != NULL) {
        free(sc->dvr.buf, M_USBDEV);
        sc->dvr.buf = NULL;
    }
}

#ifdef DIAGNOSTIC
int
sms1xxx_usb_reinit(struct sms1xxx_softc *sc)
{
    int  i;
    void *p;
    usbd_xfer_handle xfer;
    sc->dvr.ravail = 0;
    sc->dvr.wavail = 0;

    if(sc->dvr.buf != NULL) {
        free(sc->dvr.buf, M_USBDEV);
        sc->dvr.buf = NULL;
    }
    sc->dvr.buf = malloc(sc->dvr.size, M_USBDEV, M_NOWAIT);
    if(sc->dvr.buf == NULL) {
        ERR("failed to allocate dvr buffer\n");
        return ENOMEM;
    }
    sc->dvr.roff = 0;
    sc->dvr.woff = 0;
    sc->dvr.ravail = 0;
    sc->dvr.wavail = sc->dvr.size;

    for(i = 0; i < MAXXFERS; ++i) {
        if(sc->sbuf[i].xfer != NULL) {
            /* Implicit buffer free */
            usbd_free_xfer(sc->sbuf[i].xfer);
            sc->sbuf[i].xfer = NULL;
        }
        if((xfer = usbd_alloc_xfer(sc->udev)) == NULL) {
            ERR("could not allocate stream xfer: %d\n",i);
            return EIO;
        }
        if((p = usbd_alloc_buffer(xfer,sc->sbufsize)) == NULL){
            usbd_free_xfer(xfer);
            ERR("could not allocate stream buffer: %d\n",i);
            return ENOMEM;
        }
        sc->sbuf[i].xfer = xfer;
        sc->sbuf[i].buf = p;
        sc->sbuf[i].sc = sc;
    }
    return 0;
}
#endif

/* Called on each open() of a demux device */
int
sms1xxx_usb_ref(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_USB,"refcnt %d -> %d\n",sc->usbrefs,sc->usbrefs + 1);
    sc->usbrefs++;
    return 0;
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
sms1xxx_usb_get_packets(struct sms1xxx_softc *sc, u_char *packet,
    u_int32_t bytes)
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
                    u_int32_t remaining = bytes - sizeof(struct SmsMsgHdr_ST);
                    /* Number of chunks sent */
                    u_int32_t sent = 0;
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
                    for(int i=0;i < nfilt;++i) {
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

/* Main callback function, calls sms1xxx_usb_get_packets */
static void
sms1xxx_usb_read_cb(usbd_xfer_handle xfer,usbd_private_handle p,
    usbd_status status)
{
    struct sms1xxx_sbuf *sbuf = p;
    struct sms1xxx_softc *sc = sbuf->sc;
    u_int32_t cnt;

#ifdef DIAGNOSTIC
    sc->stats.interrupts++;
#endif

    usbd_get_xfer_status(xfer, NULL, NULL, &cnt, NULL);
    TRACE(TRACE_USB_FLOW,"xfer: %p status: %d cnt: %d\n",xfer,status,cnt);

    if(status == USBD_CANCELLED || sc->sc_dying) {
        return;
    }
    else if(status == USBD_STALLED) {
        ERR("endpoint stalled, clearing...\n");
        usbd_clear_endpoint_stall_async(sc->ipipe);
    }
    else if(status != USBD_NORMAL_COMPLETION) {
        ERR("usb xfer returned with status: %s\n",usbd_errstr(status));
    }
    else if(cnt > sc->sbufsize) {
        ERR("too many bytes: %d corrupt?\n",cnt);
    }
    else {
#ifdef DIAGNOSTIC
        sc->stats.bytes += cnt;
#endif
        sms1xxx_usb_get_packets(sc,sbuf->buf,cnt);
    }

    if(sc->dvr.state & DVR_AWAKE) {
        sc->dvr.state &= ~DVR_AWAKE;
        if(sc->dvr.state & DVR_SLEEP)
            wakeup(&sc->dvr);
        if(sc->dvr.state & DVR_POLL) {
            sc->dvr.state &= ~DVR_POLL;
            selwakeuppri(&sc->dvr.rsel, PZERO);
        }
    }
    usbd_setup_xfer(xfer, sc->ipipe, sbuf, sbuf->buf, sc->sbufsize,
        USBD_NO_COPY | USBD_SHORT_XFER_OK,
        USBD_NO_TIMEOUT, sms1xxx_usb_read_cb);

    usbd_transfer(xfer);
}

int
sms1xxx_usb_xfers_start(struct sms1xxx_softc *sc)
{
    int i, err;
    TRACE(TRACE_USB,"sc=%p init=%d\n",sc,sc->usbinit);

    if(sc->usbinit <= 0) {
        ERR("usb not initialized\n");
        return ENXIO;
    }

    for(i = 0; i < MAXXFERS; i++) {
        TRACE(TRACE_USB,"starting up xfer %d\n",i);
        usbd_setup_xfer(sc->sbuf[i].xfer, sc->ipipe, &sc->sbuf[i],
            sc->sbuf[i].buf, sc->sbufsize, USBD_NO_COPY |
            USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT,
            sms1xxx_usb_read_cb);

        err = usbd_transfer(sc->sbuf[i].xfer);
        if(err != USBD_NORMAL_COMPLETION && err != USBD_IN_PROGRESS)
            ERR("could not transfer xfer %d, err =  %d\n",i,err);
    }
    return 0;
}

/*******************
 * Write functions *
 *******************/

/* Write data to the device */
int
sms1xxx_usb_write(struct sms1xxx_softc *sc, const u8 *wbuf, u_int32_t wlen)
{
    int err = 1;

    TRACE(TRACE_USB,"wlen=%d\n",wlen);
    if(sc->usbinit <= 0) {
        ERR("usb not initialized\n");
        return ENXIO;
    }
    if(sc->bulk_ctrl_busy)
        return EBUSY;
    sc->bulk_ctrl_busy = 1;
    err = usbd_bulk_transfer(sc->oxfer,sc->opipe,0,2000,
        __DECONST(u8*,wbuf),&wlen,"dvbwb");

    if(err) {
        ERR("output bulk transfer failed: %d\n",err);
        if (err == USBD_INTERRUPTED)    err = EINTR;
        else if (err == USBD_TIMEOUT)   err = ETIMEDOUT;
        else                            err = EIO;
    }
    sc->bulk_ctrl_busy = 0;
    return err;
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
            return USBD_CANCELLED;
        }

        if (DLG_ISCOMPLETE(sc->dlg_status, completion))
            return 0;
    }
    while (ms < delay_ms);

    return USBD_TIMEOUT;
}

/* Write data to the device and wait for a response */
int
sms1xxx_usb_write_and_wait(struct sms1xxx_softc *sc, const u8 *wbuf,
    u_int32_t wlen, unsigned char completion, unsigned int delay_ms)
{
    int err;

    TRACE(TRACE_USB,"completion=%u(0x%x),delay_ms=%d\n",completion,
        completion,delay_ms);

    DLG_INIT(sc->dlg_status, completion);
    err = sms1xxx_usb_write(sc, wbuf, wlen);
    if (err == 0) {
        err = sms1xxx_usb_wait(sc, completion, delay_ms);
    }

    TRACE(TRACE_USB,"done, err=%d\n",err);

    return err;
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
        return -EINVAL;
    }

    /* Set requested mode and ask device to re-attach */
    sc->device->mode = DEVICE_MODE_NONE ;
    sc->device->requested_mode = mode ;
    sc->device->fw_loading = FALSE ;
    INFO("asking device to reset, target mode=%d\n",mode);

    return sms1xxx_usb_write(sc, (u8*)&Msg, sizeof(Msg));
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
        return -EINVAL;
    }

    InitMsg.xMsgHeader.msgType  = MSG_SMS_INIT_DEVICE_REQ;
    InitMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    InitMsg.xMsgHeader.msgDstId = HIF_TASK;
    InitMsg.xMsgHeader.msgLength = sizeof(InitMsg);
    InitMsg.xMsgHeader.msgFlags = 0;
    InitMsg.msgData[0] = mode;

    return sms1xxx_usb_write(sc, (u8*)&InitMsg, sizeof(InitMsg));
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
        return -ENOMEM;

    SMS_INIT_MSG(Msg, MSG_SMS_GET_VERSION_EX_REQ,
             sizeof(struct SmsMsgHdr_ST));

    rc = sms1xxx_usb_write(sc, (u8*)Msg, Msg->msgLength);

    free(buffer,M_USBDEV);

    return rc;
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

    return sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_STAT_DONE, FRONTEND_TIMEOUT);
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

    return sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_PID_DONE, FRONTEND_TIMEOUT);
}

/* Tune device to a given frequency / bandwidth */
int sms1xxx_usb_setfrequency(struct sms1xxx_softc *sc,u32 frequency,
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
        return -EINVAL;
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
        case BANDWIDTH_AUTO: return -EOPNOTSUPP;
        default: return -EINVAL;
    }
    Msg.Data[2] = 12000000;

    return sms1xxx_usb_write_and_wait(sc, (u8*)&Msg, sizeof(Msg),
        DLG_TUNE_DONE, FRONTEND_TIMEOUT);
}

/* Add a PID to hardware PID filter list */
int sms1xxx_usb_add_pid(struct sms1xxx_softc *sc,uint16_t pid)
{
    TRACE(TRACE_USB,"add PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return -EINVAL;
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_ADD_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    return sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT);
}

/* Remove a PID from hardware PID filter list */
int sms1xxx_usb_remove_pid(struct sms1xxx_softc *sc,uint16_t pid)
{
    TRACE(TRACE_USB,"remove PID %u(0x%x)\n", pid, pid);

    struct SmsMsgData_ST PidMsg;

    if(pid > PIDMAX) {
        ERR("invalid pid specified %d\n", pid);
        return -EINVAL;
    }

    PidMsg.xMsgHeader.msgType  = MSG_SMS_REMOVE_PID_FILTER_REQ;
    PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
    PidMsg.xMsgHeader.msgDstId = HIF_TASK;
    PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
    PidMsg.xMsgHeader.msgFlags = 0;
    PidMsg.msgData[0] = pid;

    return sms1xxx_usb_write_and_wait(sc, (u8*)&PidMsg, sizeof(PidMsg),
        DLG_PID_DONE, FRONTEND_TIMEOUT);
}
