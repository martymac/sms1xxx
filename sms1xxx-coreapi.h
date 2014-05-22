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
 *  Device-related structures and MACROS
 * * * *
 */

#ifndef SMS1XXX_COREAPI_H
#define SMS1XXX_COREAPI_H

#include <sys/types.h>

/* Useful types */
typedef u_int8_t	u8;
typedef u_int16_t	u16;
typedef u_int32_t	u32;
typedef int32_t		s32;
typedef boolean_t	bool;

/* Vendor IDs */
#define USB_VID_SIANO                           0x187f
/* XXX Not tested
#define USB_VID_HAUPPAUGE                       0x2040
*/

/* Product IDs */
#define USB_PID_SIANO_SMS1000_USB11             0x0010
#define USB_PID_SIANO_DMB_USB11                 0x0100
/* XXX Not tested
#define USB_PID_HAUPPAUGE_CATAMOUNT             0x1700
*/

/* Supported modes */
enum SMS_DEVICE_MODE {
    DEVICE_MODE_NONE = -1,
    DEVICE_MODE_DVBT = 0,
    DEVICE_MODE_DVBH,
    DEVICE_MODE_DAB_TDMB,
    DEVICE_MODE_DAB_TDMB_DABIP,
    DEVICE_MODE_DVBT_BDA,
    DEVICE_MODE_MAX,
};
#define DEVICE_MODE_DEFAULT DEVICE_MODE_DVBT_BDA

/* Interfaces */
#define USB_SIANO_INTF_ROM                      0x00
#define USB_SIANO_INTF_FWUPLD                   0x01

/* Endpoints */
#define USB_SIANO_ENDP_CTRL                     0x02
#define USB_SIANO_ENDP_RCV                      0x81

/* URBS */
#define SBUFSIZE                                0x1000
#define MAXXFERS                                10

/* Bandwidths */
#define BW_8_MHZ                                0
#define BW_7_MHZ                                1
#define BW_6_MHZ                                2

/* Message handling */
#define HIF_TASK                                11
#define DVBT_BDA_CONTROL_MSG_ID                 201

#define MSG_SMS_RF_TUNE_REQ                     561
#define MSG_SMS_RF_TUNE_RES                     562
#define MSG_SMS_INIT_DEVICE_REQ                 578
#define MSG_SMS_INIT_DEVICE_RES                 579
#define MSG_SMS_ADD_PID_FILTER_REQ              601
#define MSG_SMS_ADD_PID_FILTER_RES              602
#define MSG_SMS_REMOVE_PID_FILTER_REQ           603
#define MSG_SMS_REMOVE_PID_FILTER_RES           604
#define MSG_SMS_GET_PID_FILTER_LIST_REQ         608
#define MSG_SMS_GET_PID_FILTER_LIST_RES         609
#define MSG_SMS_GET_STATISTICS_REQ              615
#define MSG_SMS_GET_STATISTICS_RES              616
#define MSG_SMS_GET_VERSION_EX_REQ              668
#define MSG_SMS_GET_VERSION_EX_RES              669
#define MSG_SMS_DVBT_BDA_DATA                   693
#define MSG_SW_RELOAD_REQ                       697

#define SMS_INIT_MSG_EX(ptr, type, src, dst, len) do { \
    (ptr)->msgType = type; (ptr)->msgSrcId = src; (ptr)->msgDstId = dst; \
    (ptr)->msgLength = len; (ptr)->msgFlags = 0; \
} while (0)
#define SMS_INIT_MSG(ptr, type, len) \
    SMS_INIT_MSG_EX(ptr, type, 0, HIF_TASK, len)

#define SMS_DMA_ALIGNMENT                       16
#define SMS_ALIGN_ADDRESS(addr) \
    ((((uintptr_t)(addr)) + (SMS_DMA_ALIGNMENT-1)) & ~(SMS_DMA_ALIGNMENT-1))

/****************************
 * Communication structures *
 ****************************/

struct SmsMsgHdr_ST {
    u16 msgType;
    u8  msgSrcId;
    u8  msgDstId;
    u16 msgLength; /* Length of entire message, including header */
    u16 msgFlags;
};

struct SmsMsgData_ST {
    struct SmsMsgHdr_ST xMsgHeader;
    u32                 msgData[1];
};

struct SMSHOSTLIB_STATISTICS_ST {
    u32 Reserved;           /* Reserved */

    /* Common parameters */
    u32 IsRfLocked;         /* 0 - not locked, 1 - locked */
    u32 IsDemodLocked;      /* 0 - not locked, 1 - locked */
    u32 IsExternalLNAOn;    /* 0 - external LNA off, 1 - external LNA on */

    /* Reception quality */
    s32 SNR;                /* dB */
    u32 BER;                /* Post Viterbi BER [1E-5] */
    u32 FIB_CRC;            /* CRC errors percentage, valid only for DAB */
    u32 TS_PER;             /* Transport stream PER, 0xFFFFFFFF indicate N/A,
                             * valid only for DVB-T/H */
    u32 MFER;               /* DVB-H frame error rate in percentage,
                             * 0xFFFFFFFF indicate N/A, valid only for DVB-H */
    s32  RSSI; /* dBm */
    s32  InBandPwr;         /* In band power in dBM */
    s32  CarrierOffset;     /* Carrier Offset in bin/1024 */

    /* Transmission parameters, valid only for DVB-T/H */
    u32 Frequency;          /* Frequency in Hz */
    u32 Bandwidth;          /* Bandwidth in MHz */
    u32 TransmissionMode;   /* Transmission Mode, for DAB modes 1-4,
                             * for DVB-T/H FFT mode carriers in Kilos */
    u32 ModemState;         /* from SMS_DvbModemState_ET */
    u32 GuardInterval;      /* Guard Interval, 1 divided by value */
    u32 CodeRate;           /* Code Rate from SMS_DvbModemState_ET */
    u32 LPCodeRate;         /* Low Priority Code Rate from
                               SMS_DvbModemState_ET */
    u32 Hierarchy;          /* Hierarchy from SMS_Hierarchy_ET */
    u32 Constellation;      /* Constellation from SMS_Constellation_ET */

    /* Burst parameters, valid only for DVB-H */
    u32 BurstSize;          /* Current burst size in bytes */
    u32 BurstDuration;      /* Current burst duration in mSec */
    u32 BurstCycleTime;     /* Current burst cycle time in mSec */
    u32 CalculatedBurstCycleTime; /* Current burst cycle time in mSec,
                                   * as calculated by demodulator */
    u32 NumOfRows;          /* Number of rows in MPE table */
    u32 NumOfPaddCols;      /* Number of padding columns in MPE table */
    u32 NumOfPunctCols;     /* Number of puncturing columns in MPE table */
    /* Burst parameters */
    u32 ErrorTSPackets;     /* Number of erroneous transport-stream packets */
    u32 TotalTSPackets;     /* Total number of transport-stream packets */
    u32 NumOfValidMpeTlbs;  /* Number of MPE tables which do not include
                             * errors after MPE RS decoding */
    u32 NumOfInvalidMpeTlbs; /* Number of MPE tables which include errors
                              * after MPE RS decoding */
    u32 NumOfCorrectedMpeTlbs; /* Number of MPE tables which were corrected
                                * by MPE RS decoding */

    /* Common params */
    u32 BERErrorCount;      /* Number of errornous SYNC bits. */
    u32 BERBitCount;        /* Total number of SYNC bits. */

    /* Interface information */
    u32 SmsToHostTxErrors;  /* Total number of transmission errors. */

    /* DAB/T-DMB */
    u32 PreBER;             /* DAB/T-DMB only: Pre Viterbi BER [1E-5] */

    /* DVB-H TPS parameters */
    u32 CellId;             /* TPS Cell ID in bits 15..0, bits 31..16 zero;
                             * if set to 0xFFFFFFFF cell_id not yet recovered */
};

struct SmsMsgStatisticsInfo_ST {
    u32 RequestResult;

    struct SMSHOSTLIB_STATISTICS_ST Stat;

    /* Split the calc of the SNR in DAB */
    u32 Signal; /* dB */
    u32 Noise;  /* dB */

};

struct SmsVersionRes_ST {
    struct SmsMsgHdr_ST xMsgHeader;

    u16     ChipModel;          /* e.g. 0x1102 for SMS-1102 "Nova" */
    u8      Step;               /* 0 - Step A */
    u8      MetalFix;           /* 0 - Metal 0 */

    u8      FirmwareId;         /* 0xFF ï¿½ ROM, otherwise the
                                 * value indicated by
                                 * SMSHOSTLIB_DEVICE_MODES_E */
    u8      SupportedProtocols; /* Bitwise OR combination of
                                 * supported protocols */

    u8      VersionMajor;
    u8      VersionMinor;
    u8      VersionPatch;
    u8      VersionFieldPatch;

    u8      RomVersionMajor;
    u8      RomVersionMinor;
    u8      RomVersionPatch;
    u8      RomVersionFieldPatch;

    u8      TextLabel[34];
};

#endif
