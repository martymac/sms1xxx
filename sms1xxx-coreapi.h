/*  SMS1XXX - Siano DVB-T USB driver for FreeBSD 8.0 and higher:
 *
 *  Copyright (C) 2008-2009 - Ganaël Laplanche, http://contribs.martymac.org
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
typedef u_int8_t    u8;
typedef u_int16_t   u16;
typedef u_int32_t   u32;
typedef int32_t     s32;
typedef boolean_t   bool;

/* Card flavours */
/* Families */
#define SMS1XXX_FAMILY_MASK                 0xff000000UL
#define SMS1XXX_FAMILY1                     0x00000000UL /* USB 1.1 */
#define SMS1XXX_FAMILY2                     0x01000000UL /* USB 2.0 */

/* Types */
#define SMS1XXX_TYPE_MASK                   0x00ff0000UL /* used as index for
                                                            sms1xxx_firmwares
                                                            array */
#define SMS1XXX_TYPE_STELLAR               (0x00000000UL | SMS1XXX_FAMILY1)
#define SMS1XXX_TYPE_NOVA_A                (0x00010000UL | SMS1XXX_FAMILY2)
#define SMS1XXX_TYPE_NOVA_B                (0x00020000UL | SMS1XXX_FAMILY2)
#define SMS1XXX_TYPE_VEGA                  (0x00030000UL | SMS1XXX_FAMILY2)
#define SMS1XXX_TYPE_MIN                    SMS1XXX_TYPE_STELLAR
#define SMS1XXX_TYPE_MAX                    SMS1XXX_TYPE_VEGA

/* Boards */
#define SMS1XXX_BOARD_MASK                  0x0000ff00UL
#define SMS1XXX_BOARD_UNKNOWN              (0x00000000UL | SMS1XXX_TYPE_STELLAR)
#define SMS1XXX_BOARD_SIANO_STELLAR        (0x00000100UL | SMS1XXX_TYPE_STELLAR)
#if 0
#define SMS1XXX_BOARD_SIANO_NOVA_A         (0x00000200UL | SMS1XXX_TYPE_NOVA_A)
#define SMS1XXX_BOARD_SIANO_NOVA_B         (0x00000300UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_VEGA           (0x00000400UL | SMS1XXX_TYPE_VEGA)
#define SMS1XXX_BOARD_HAUPPAUGE_CATAMOUNT  (0x00000500UL | SMS1XXX_TYPE_STELLAR)
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_A    (0x00000600UL | SMS1XXX_TYPE_NOVA_A)
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_B    (0x00000700UL | SMS1XXX_TYPE_NOVA_B)
#endif
#define SMS1XXX_BOARD_HAUPPAUGE_WINDHAM    (0x00000800UL | SMS1XXX_TYPE_NOVA_B)
#if 0
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD \
                                           (0x00000900UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD_R2 \
                                           (0x00000a00UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_NICE           (0x00000b00UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_VENICE         (0x00000c00UL | SMS1XXX_TYPE_VEGA)
#endif

/* Vendor IDs */
#define USB_VID_SIANO                      0x187f
#define USB_VID_HAUPPAUGE                  0x2040

/* Product IDs */
#define USB_PID_SIANO_SMS1000              0x0010
#define USB_PID_SIANO_DMB                  0x0100
#define USB_PID_HAUPPAUGE_MINISTICK        0x5500

/* Supported modes */
enum SMS1XXX_DEVICE_MODE {
    DEVICE_MODE_NONE = -1,
    DEVICE_MODE_DVBT = 0,
    DEVICE_MODE_DVBH,
    DEVICE_MODE_DAB_TDMB,
    DEVICE_MODE_DAB_TDMB_DABIP,
    DEVICE_MODE_DVBT_BDA,
    DEVICE_MODE_ISDBT,
    DEVICE_MODE_ISDBT_BDA,
    DEVICE_MODE_CMMB,
    DEVICE_MODE_RAW_TUNER,
};
#define DEVICE_MODE_DEFAULT                DEVICE_MODE_DVBT_BDA
#define DEVICE_MODE_MIN                    DEVICE_MODE_DVBT
#define DEVICE_MODE_MAX                    DEVICE_MODE_RAW_TUNER

/* Interfaces */
#define USB_SIANO_INTF_ROM                 0x00
#define USB_SIANO_INTF_FWUPLD              0x01

/* Endpoints */
#define USB_SIANO_ENDP_CTRL                0x02
#define USB_SIANO_ENDP_RCV                 0x81

/* Xfers  */
enum {
    SMS1XXX_BULK_RX,
    SMS1XXX_BULK_TX,
    SMS1XXX_N_TRANSFER,
};

struct sms1xxx_data {
    u8*                                     buf;
    u32                                     buflen;
    STAILQ_ENTRY(sms1xxx_data)              next;
};
typedef STAILQ_HEAD(, sms1xxx_data) sms1xxx_datahead;
#define SMS1XXX_BULK_TX_BUFS                8
#define SMS1XXX_BULK_TX_BUFS_SIZE           512

/* USB1.1 Terratec Cinergy Piranha mostly receives 3956 bytes :
   (sizeof(struct SmsMsgHdr_ST) + 21 * PACKET_SIZE)
   USB2.0 devices need more because those 21 packets may appear
   following an offset (see MSG_HDR_FLAG_SPLIT_MSG) */
#define SMS1XXX_BULK_RX_BUFS_SIZE           16384

/* Bandwidths */
#define BW_8_MHZ                            0
#define BW_7_MHZ                            1
#define BW_6_MHZ                            2

/* Message handling */
#define HIF_TASK                            11
#define DVBT_BDA_CONTROL_MSG_ID             201

#define MSG_SMS_RF_TUNE_REQ                 561
#define MSG_SMS_RF_TUNE_RES                 562
#define MSG_SMS_INIT_DEVICE_REQ             578
#define MSG_SMS_INIT_DEVICE_RES             579
#define MSG_SMS_ADD_PID_FILTER_REQ          601
#define MSG_SMS_ADD_PID_FILTER_RES          602
#define MSG_SMS_REMOVE_PID_FILTER_REQ       603
#define MSG_SMS_REMOVE_PID_FILTER_RES       604
#define MSG_SMS_GET_PID_FILTER_LIST_REQ     608
#define MSG_SMS_GET_PID_FILTER_LIST_RES     609
#define MSG_SMS_GET_STATISTICS_REQ          615
#define MSG_SMS_GET_STATISTICS_RES          616
#define MSG_SMS_HO_PER_SLICES_IND           630
#define MSG_SMS_DATA_DOWNLOAD_REQ           660
#define MSG_SMS_DATA_DOWNLOAD_RES           661
#define MSG_SMS_SWDOWNLOAD_TRIGGER_REQ      664
#define MSG_SMS_SWDOWNLOAD_TRIGGER_RES      665
#define MSG_SMS_GET_VERSION_EX_REQ          668
#define MSG_SMS_GET_VERSION_EX_RES          669
#define MSG_SMS_DVBT_BDA_DATA               693
#define MSG_SW_RELOAD_REQ                   697
#define MSG_SW_RELOAD_START_REQ             702
#define MSG_SW_RELOAD_START_RES             703
#define MSG_SW_RELOAD_EXEC_REQ              704
#define MSG_SW_RELOAD_EXEC_RES              705
#define MSG_SMS_TRANSMISSION_IND            782
#define MSG_SMS_START_IR_REQ                800
#define MSG_SMS_START_IR_RES                801
#define MSG_SMS_IR_SAMPLES_IND              802
#define MSG_SMS_SIGNAL_DETECTED_IND         827
#define MSG_SMS_NO_SIGNAL_IND               828

/****************************
 * Communication structures *
 ****************************/

#define MSG_HDR_FLAG_SPLIT_MSG          4
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

struct SmsMsgData_ST2 {
    struct SmsMsgHdr_ST xMsgHeader;
    u32                 msgData[2];
};

#define SMS_MAX_PAYLOAD_SIZE            240
struct SmsDataDownload_ST {
    struct SmsMsgHdr_ST xMsgHeader;
    u32                 MemAddr;
    u8                  Payload[SMS_MAX_PAYLOAD_SIZE];
};

struct SmsFirmware_ST {
    u32 CheckSum;
    u32 Length;
    u32 StartAddress;
    u8  Payload[1];
};

struct SMSHOSTLIB_STATISTICS_S {
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
                               valid only for DVB-T/H */
    u32 MFER;               /* DVB-H frame error rate in percentage,
                               0xFFFFFFFF indicate N/A, valid only for DVB-H */
    s32 RSSI;               /* dBm */
    s32 InBandPwr;          /* In band power in dBM */
    s32 CarrierOffset;      /* Carrier Offset in bin/1024 */

    /* Transmission parameters */
    u32 Frequency;          /* Frequency in Hz */
    u32 Bandwidth;          /* Bandwidth in MHz,
                               valid only for DVB-T/H */
    u32 TransmissionMode;   /* Transmission Mode, for DAB modes 1-4,
                               for DVB-T/H FFT mode carriers in Kilos */
    u32 ModemState;         /* from SMSHOSTLIB_DVB_MODEM_STATE_ET,
                               valid only for DVB-T/H */
    u32 GuardInterval;      /* Guard Interval from
                               SMSHOSTLIB_GUARD_INTERVALS_ET,
                               valid only for DVB-T/H */
    u32 CodeRate;           /* Code Rate from SMSHOSTLIB_CODE_RATE_ET,
                               valid only for DVB-T/H */
    u32 LPCodeRate;         /* Low Priority Code Rate from
                               SMSHOSTLIB_CODE_RATE_ET,
                               valid only for DVB-T/H */
    u32 Hierarchy;          /* Hierarchy from SMSHOSTLIB_HIERARCHY_ET,
                               valid only for DVB-T/H */
    u32 Constellation;      /* Constellation from
                               SMSHOSTLIB_CONSTELLATION_ET,
                               valid only for DVB-T/H */

    /* Burst parameters, valid only for DVB-H */
    u32 BurstSize;          /* Current burst size in bytes,
                               valid only for DVB-H */
    u32 BurstDuration;      /* Current burst duration in mSec,
                               valid only for DVB-H */
    u32 BurstCycleTime;     /* Current burst cycle time in mSec,
                               valid only for DVB-H */
    u32 CalculatedBurstCycleTime;   /* Current burst cycle time in mSec,
                                       as calculated by demodulator,
                                       valid only for DVB-H */
    u32 NumOfRows;          /* Number of rows in MPE table,
                               valid only for DVB-H */
    u32 NumOfPaddCols;      /* Number of padding columns in MPE table,
                               valid only for DVB-H */
    u32 NumOfPunctCols;     /* Number of puncturing columns in MPE table,
                               valid only for DVB-H */
    u32 ErrorTSPackets;     /* Number of erroneous
                               transport-stream packets */
    u32 TotalTSPackets;     /* Total number of transport-stream packets */
    u32 NumOfValidMpeTlbs;  /* Number of MPE tables which do not include
                               errors after MPE RS decoding */
    u32 NumOfInvalidMpeTlbs;/* Number of MPE tables which include errors
                               after MPE RS decoding */
    u32 NumOfCorrectedMpeTlbs;      /* Number of MPE tables which were
                                       corrected by MPE RS decoding */
    /* Common params */
    u32 BERErrorCount;      /* Number of errornous SYNC bits. */
    u32 BERBitCount;        /* Total number of SYNC bits. */

    /* Interface information */
    u32 SmsToHostTxErrors;  /* Total number of transmission errors. */

    /* DAB/T-DMB */
    u32 PreBER;             /* DAB/T-DMB only: Pre Viterbi BER [1E-5] */

    /* DVB-H TPS parameters */
    u32 CellId;             /* TPS Cell ID in bits 15..0, bits 31..16 zero;
                               if set to 0xFFFFFFFF cell_id not yet recovered */
    u32 DvbhSrvIndHP;       /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 DvbhSrvIndLP;       /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */

    u32 NumMPEReceived;     /* DVB-H, Num MPE section received */

    u32 ReservedFields[10]; /* Reserved */
};

/* Statistics information returned as response for
 * SmsHostApiGetStatistics_Req, SMS1000 */
struct SmsMsgStatisticsInfo_ST {
    u32 RequestResult;

    struct SMSHOSTLIB_STATISTICS_S Stat;

    /* Split the calc of the SNR in DAB */
    u32 Signal; /* dB */
    u32 Noise;  /* dB */

};

struct PID_STATISTICS_DATA_S {
    struct PID_BURST_S {
        u32 size;
        u32 padding_cols;
        u32 punct_cols;
        u32 duration;
        u32 cycle;
        u32 calc_cycle;
    } burst;

    u32 tot_tbl_cnt;
    u32 invalid_tbl_cnt;
    u32 tot_cor_tbl;
};

struct PID_DATA_S {
    u32 pid;
    u32 num_rows;
    struct PID_STATISTICS_DATA_S pid_statistics;
};

#define CORRECT_STAT_RSSI(_stat) ((_stat).RSSI *= -1)
#define CORRECT_STAT_BANDWIDTH(_stat) (_stat.Bandwidth = 8 - _stat.Bandwidth)
#define CORRECT_STAT_TRANSMISSON_MODE(_stat) \
    if (_stat.TransmissionMode == 0) \
        _stat.TransmissionMode = 2; \
    else if (_stat.TransmissionMode == 1) \
        _stat.TransmissionMode = 8; \
    else \
        _stat.TransmissionMode = 4;

struct TRANSMISSION_STATISTICS_S {
    u32 Frequency;          /* Frequency in Hz */
    u32 Bandwidth;          /* Bandwidth in MHz */
    u32 TransmissionMode;   /* FFT mode carriers in Kilos */
    u32 GuardInterval;      /* Guard Interval from
                               SMSHOSTLIB_GUARD_INTERVALS_ET */
    u32 CodeRate;           /* Code Rate from SMSHOSTLIB_CODE_RATE_ET */
    u32 LPCodeRate;         /* Low Priority Code Rate from
                               SMSHOSTLIB_CODE_RATE_ET */
    u32 Hierarchy;          /* Hierarchy from SMSHOSTLIB_HIERARCHY_ET */
    u32 Constellation;      /* Constellation from SMSHOSTLIB_CONSTELLATION_ET */

    /* DVB-H TPS parameters */
    u32 CellId;             /* TPS Cell ID in bits 15..0, bits 31..16 zero;
                               if set to 0xFFFFFFFF cell_id not yet recovered */
    u32 DvbhSrvIndHP;       /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 DvbhSrvIndLP;       /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 IsDemodLocked;      /* 0 - not locked, 1 - locked */
};

struct RECEPTION_STATISTICS_S {
    u32 IsRfLocked;         /* 0 - not locked, 1 - locked */
    u32 IsDemodLocked;      /* 0 - not locked, 1 - locked */
    u32 IsExternalLNAOn;    /* 0 - external LNA off, 1 - external LNA on */

    u32 ModemState;         /* from SMSHOSTLIB_DVB_MODEM_STATE_ET */
    s32 SNR;                /* dB */
    u32 BER;                /* Post Viterbi BER [1E-5] */
    u32 BERErrorCount;      /* Number of erronous SYNC bits. */
    u32 BERBitCount;        /* Total number of SYNC bits. */
    u32 TS_PER;             /* Transport stream PER, 0xFFFFFFFF indicate N/A */
    u32 MFER;               /* DVB-H frame error rate in percentage,
                               0xFFFFFFFF indicate N/A, valid only for DVB-H */
    s32 RSSI;               /* dBm */
    s32 InBandPwr;          /* In band power in dBM */
    s32 CarrierOffset;      /* Carrier Offset in bin/1024 */
    u32 ErrorTSPackets;     /* Number of erroneous transport-stream packets */
    u32 TotalTSPackets;     /* Total number of transport-stream packets */

    s32 MRC_SNR;            /* dB */
    s32 MRC_RSSI;           /* dBm */
    s32 MRC_InBandPwr;      /* In band power in dBM */
};

/* Statistics information returned as response for
 * SmsHostApiGetStatisticsEx_Req for DVB applications, SMS1100 and up */
struct SMSHOSTLIB_STATISTICS_DVB_S {
    /* Reception */
    struct RECEPTION_STATISTICS_S ReceptionData;

    /* Transmission parameters */
    struct TRANSMISSION_STATISTICS_S TransmissionData;

    /* Burst parameters, valid only for DVB-H */
#define SRVM_MAX_PID_FILTERS 8
    struct PID_DATA_S PidData[SRVM_MAX_PID_FILTERS];
};

struct SRVM_SIGNAL_STATUS_S {
    u32 result;
    u32 snr;
    u32 tsPackets;
    u32 etsPackets;
    u32 constellation;
    u32 hpCode;
    u32 tpsSrvIndLP;
    u32 tpsSrvIndHP;
    u32 cellId;
    u32 reason;

    s32 inBandPower;
    u32 requestId;
};

struct SmsVersionRes_ST {
    struct SmsMsgHdr_ST xMsgHeader;

    u16 ChipModel;          /* e.g. 0x1102 for SMS-1102 "Nova" */
    u8  Step;               /* 0 - Step A */
    u8  MetalFix;           /* 0 - Metal 0 */

    u8  FirmwareId;         /* 0xFF if ROM, otherwise the
                             * value indicated by
                             * SMSHOSTLIB_DEVICE_MODES_E */
    u8  SupportedProtocols; /* Bitwise OR combination of
                             * supported protocols */

    u8  VersionMajor;
    u8  VersionMinor;
    u8  VersionPatch;
    u8  VersionFieldPatch;

    u8  RomVersionMajor;
    u8  RomVersionMinor;
    u8  RomVersionPatch;
    u8  RomVersionFieldPatch;

#define MSG_VER_LABEL_SIZE            34
    u8  TextLabel[MSG_VER_LABEL_SIZE];
};

#endif
