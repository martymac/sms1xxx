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
typedef u_int64_t   u64;
typedef int64_t     s64;
#if !defined(__bool_true_false_are_defined)
typedef boolean_t   bool;
#endif

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
#define SMS1XXX_BOARD_SIANO_NOVA_A         (0x00000200UL | SMS1XXX_TYPE_NOVA_A)
#define SMS1XXX_BOARD_SIANO_NOVA_B         (0x00000300UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_VEGA           (0x00000400UL | SMS1XXX_TYPE_VEGA)
#define SMS1XXX_BOARD_HAUPPAUGE_CATAMOUNT  (0x00000500UL | SMS1XXX_TYPE_STELLAR)
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_A    (0x00000600UL | SMS1XXX_TYPE_NOVA_A)
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_B    (0x00000700UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_HAUPPAUGE_WINDHAM    (0x00000800UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD \
                                           (0x00000900UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD_R2 \
                                           (0x00000a00UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_NICE           (0x00000b00UL | SMS1XXX_TYPE_NOVA_B)
#define SMS1XXX_BOARD_SIANO_VENICE         (0x00000c00UL | SMS1XXX_TYPE_VEGA)

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

#define MSG_SMS_GPIO_CONFIG_REQ             507
#define MSG_SMS_GPIO_CONFIG_RES             508
#define MSG_SMS_GPIO_SET_LEVEL_REQ          509
#define MSG_SMS_GPIO_SET_LEVEL_RES          510
#define MSG_SMS_GPIO_GET_LEVEL_REQ          511
#define MSG_SMS_GPIO_GET_LEVEL_RES          512
#define MSG_SMS_RF_TUNE_REQ                 561
#define MSG_SMS_RF_TUNE_RES                 562
#define MSG_SMS_INIT_DEVICE_REQ             578
#define MSG_SMS_INIT_DEVICE_RES             579
#define MSG_SMS_ADD_PID_FILTER_REQ          601
#define MSG_SMS_ADD_PID_FILTER_RES          602
#define MSG_SMS_REMOVE_PID_FILTER_REQ       603
#define MSG_SMS_REMOVE_PID_FILTER_RES       604
#define MSG_SMS_DAB_CHANNEL                 607 /* XXX not used yet */
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
#define MSG_SMS_DATA_MSG                    699 /* XXX not used yet */
#define MSG_SW_RELOAD_START_REQ             702
#define MSG_SW_RELOAD_START_RES             703
#define MSG_SW_RELOAD_EXEC_REQ              704
#define MSG_SW_RELOAD_EXEC_RES              705
#define MSG_SMS_GPIO_CONFIG_EX_REQ          712
#define MSG_SMS_GPIO_CONFIG_EX_RES          713
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


/* statistics information returned as response for
 * SmsHostApiGetstatistics_Req */
struct sms_stats {
    u32 reserved;           /* reserved */

    /* Common parameters */
    u32 is_rf_locked;       /* 0 - not locked, 1 - locked */
    u32 is_demod_locked;    /* 0 - not locked, 1 - locked */
    u32 is_external_lna_on; /* 0 - external LNA off, 1 - external LNA on */

    /* Reception quality */
    s32 SNR;                /* dB */
    u32 ber;                /* Post Viterbi ber [1E-5] */
    u32 FIB_CRC;            /* CRC errors percentage, valid only for DAB */
    u32 ts_per;             /* Transport stream PER,
                               0xFFFFFFFF indicate N/A, valid only for
                               DVB-T/H */
    u32 MFER;               /* DVB-H frame error rate in percentage,
                               0xFFFFFFFF indicate N/A, valid only for DVB-H */
    s32 RSSI;               /* dBm */
    s32 in_band_pwr;        /* In band power in dBM */
    s32 carrier_offset;     /* Carrier Offset in bin/1024 */

    /* Transmission parameters */
    u32 frequency;          /* frequency in Hz */
    u32 bandwidth;          /* bandwidth in MHz, valid only for DVB-T/H */
    u32 transmission_mode;  /* Transmission Mode, for DAB modes 1-4,
                               for DVB-T/H FFT mode carriers in Kilos */
    u32 modem_state;        /* from SMSHOSTLIB_DVB_MODEM_STATE_ET,
                               valid only for DVB-T/H */
    u32 guard_interval;     /* Guard Interval from
                               SMSHOSTLIB_GUARD_INTERVALS_ET,
                               valid only for DVB-T/H */
    u32 code_rate;          /* Code Rate from SMSHOSTLIB_CODE_RATE_ET,
                               valid only for DVB-T/H */
    u32 lp_code_rate;       /* Low Priority Code Rate from
                               SMSHOSTLIB_CODE_RATE_ET, valid only for
                               DVB-T/H */
    u32 hierarchy;          /* hierarchy from SMSHOSTLIB_HIERARCHY_ET,
                               valid only for DVB-T/H */
    u32 constellation;      /* constellation from
                               SMSHOSTLIB_CONSTELLATION_ET, valid only for
                               DVB-T/H */

    /* Burst parameters, valid only for DVB-H */
    u32 burst_size;         /* Current burst size in bytes,
                               valid only for DVB-H */
    u32 burst_duration;     /* Current burst duration in mSec,
                               valid only for DVB-H */
    u32 burst_cycle_time;   /* Current burst cycle time in mSec,
                               valid only for DVB-H */
    u32 calc_burst_cycle_time;  /* Current burst cycle time in mSec,
                                   as calculated by demodulator, valid only for
                                   DVB-H */
    u32 num_of_rows;        /* Number of rows in MPE table,
                               valid only for DVB-H */
    u32 num_of_padd_cols;   /* Number of padding columns in MPE table,
                               valid only for DVB-H */
    u32 num_of_punct_cols;  /* Number of puncturing columns in MPE table,
                               valid only for DVB-H */
    u32 error_ts_packets;   /* Number of erroneous transport-stream packets */
    u32 total_ts_packets;   /* Total number of transport-stream packets */
    u32 num_of_valid_mpe_tlbs;  /* Number of MPE tables which do not include
                                   errors after MPE RS decoding */
    u32 num_of_invalid_mpe_tlbs;/* Number of MPE tables which include errors
                                   after MPE RS decoding */
    u32 num_of_corrected_mpe_tlbs;  /* Number of MPE tables which were
                                       corrected by MPE RS decoding */

    /* Common params */
    u32 ber_error_count;    /* Number of errornous SYNC bits. */
    u32 ber_bit_count;      /* Total number of SYNC bits. */

    /* Interface information */
    u32 sms_to_host_tx_errors;  /* Total number of transmission errors. */

    /* DAB/T-DMB */
    u32 pre_ber;            /* DAB/T-DMB only: Pre Viterbi ber [1E-5] */

    /* DVB-H TPS parameters */
    u32 cell_id;            /* TPS Cell ID in bits 15..0, bits 31..16 zero;
                               if set to 0xFFFFFFFF cell_id not yet recovered */
    u32 dvbh_srv_ind_hp;    /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 dvbh_srv_ind_lp;    /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */

    u32 num_mpe_received;   /* DVB-H, Num MPE section received */

    u32 reservedFields[10]; /* reserved */
};

/* Statistics information returned as response for
 * SmsHostApiGetStatistics_Req, SMS1000 */
struct sms_msg_statistics_info {
    u32 request_result;

    struct sms_stats stat;

    /* Split the calc of the SNR in DAB */
    u32 signal; /* dB */
    u32 noise;  /* dB */

};

struct sms_pid_stats_data {
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

struct sms_pid_data {
    u32 pid;
    u32 num_rows;
    struct sms_pid_stats_data pid_statistics;
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

struct sms_tx_stats {
    u32 frequency;          /* frequency in Hz */
    u32 bandwidth;          /* bandwidth in MHz */
    u32 transmission_mode;  /* FFT mode carriers in Kilos */
    u32 guard_interval;     /* Guard Interval from
                               SMSHOSTLIB_GUARD_INTERVALS_ET */
    u32 code_rate;          /* Code Rate from SMSHOSTLIB_CODE_RATE_ET */
    u32 lp_code_rate;       /* Low Priority Code Rate from
                               SMSHOSTLIB_CODE_RATE_ET */
    u32 hierarchy;          /* hierarchy from SMSHOSTLIB_HIERARCHY_ET */
    u32 constellation;      /* constellation from
                               SMSHOSTLIB_CONSTELLATION_ET */

    /* DVB-H TPS parameters */
    u32 cell_id;            /* TPS Cell ID in bits 15..0, bits 31..16 zero;
                               if set to 0xFFFFFFFF cell_id not yet recovered */
    u32 dvbh_srv_ind_hp;    /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 dvbh_srv_ind_lp;    /* DVB-H service indication info, bit 1 -
                               Time Slicing indicator, bit 0 - MPE-FEC
                               indicator */
    u32 is_demod_locked;    /* 0 - not locked, 1 - locked */
};

struct sms_rx_stats {
    u32 is_rf_locked;       /* 0 - not locked, 1 - locked */
    u32 is_demod_locked;    /* 0 - not locked, 1 - locked */
    u32 is_external_lna_on; /* 0 - external LNA off, 1 - external LNA on */

    u32 modem_state;        /* from SMSHOSTLIB_DVB_MODEM_STATE_ET */
    s32 SNR;                /* dB */
    u32 ber;                /* Post Viterbi ber [1E-5] */
    u32 ber_error_count;    /* Number of erroneous SYNC bits. */
    u32 ber_bit_count;      /* Total number of SYNC bits. */
    u32 ts_per;             /* Transport stream PER,
                               0xFFFFFFFF indicate N/A */
    u32 MFER;               /* DVB-H frame error rate in percentage,
                               0xFFFFFFFF indicate N/A, valid only for DVB-H */
    s32 RSSI;               /* dBm */
    s32 in_band_pwr;        /* In band power in dBM */
    s32 carrier_offset;     /* Carrier Offset in bin/1024 */
    u32 error_ts_packets;   /* Number of erroneous
                               transport-stream packets */
    u32 total_ts_packets;   /* Total number of transport-stream packets */

    s32 MRC_SNR;            /* dB */
    s32 MRC_RSSI;           /* dBm */
    s32 mrc_in_band_pwr;    /* In band power in dBM */
};

/* statistics information returned as response for
 * SmsHostApiGetstatisticsEx_Req for DVB applications, SMS1100 and up */
struct sms_stats_dvb {
    /* Reception */
    struct sms_rx_stats reception_data;

    /* Transmission parameters */
    struct sms_tx_stats transmission_data;

    /* Burst parameters, valid only for DVB-H */
#define SRVM_MAX_PID_FILTERS 8
    struct sms_pid_data pid_data[SRVM_MAX_PID_FILTERS];
};

struct RECEPTION_STATISTICS_PER_SLICES_S {
    u32 result;
    u32 snr;
    s32 in_band_power;
    u32 ts_packets;
    u32 ets_packets;
    u32 constellation;
    u32 hp_code;
    u32 tps_srv_ind_lp;
    u32 tps_srv_ind_hp;
    u32 cell_id;
    u32 reason;
    u32 request_id;
    u32 modem_state;        /* from SMSHOSTLIB_DVB_MODEM_STATE_ET */

    u32 ber;                /* Post Viterbi BER [1E-5] */
    s32 RSSI;               /* dBm */
    s32 carrier_offset;     /* Carrier Offset in bin/1024 */

    u32 is_rf_locked;       /* 0 - not locked, 1 - locked */
    u32 is_demod_locked;    /* 0 - not locked, 1 - locked */

    u32 ber_bit_count;      /* Total number of SYNC bits. */
    u32 ber_error_count;    /* Number of erroneous SYNC bits. */

    s32 MRC_SNR;            /* dB */
    s32 mrc_in_band_pwr;    /* In band power in dBM */
    s32 MRC_RSSI;           /* dBm */
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
