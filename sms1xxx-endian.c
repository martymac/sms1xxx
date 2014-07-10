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
 *  Endianness handling
 * * * *
 */

#include <sys/endian.h>

#include "sms1xxx.h"
#include "sms1xxx-endian.h"

/* Handle TX message data */
void sms1xxx_endian_handle_tx_message(void *buffer)
{
#if BYTE_ORDER == BIG_ENDIAN
    struct SmsMsgData_ST *msg = (struct SmsMsgData_ST *)buffer;
    int i;
    int msgWords;

    if(msg == NULL)
        return;

    switch (msg->xMsgHeader.msgType) {
        /* special case for MSG_SMS_DATA_DOWNLOAD_REQ
           that embeds payload not to be swapped */
        case MSG_SMS_DATA_DOWNLOAD_REQ:
            msg->msgData[0] = le32toh(msg->msgData[0]);
            break;
        default:
            msgWords = (msg->xMsgHeader.msgLength -
                    sizeof(struct SmsMsgHdr_ST))/4;

            for (i = 0; i < msgWords; i++)
                msg->msgData[i] = le32toh(msg->msgData[i]);

            break;
    }
#endif /* BIG_ENDIAN */
    return;
}

/* Handle RX message data */
void sms1xxx_endian_handle_rx_message(void *buffer)
{
#if BYTE_ORDER == BIG_ENDIAN
    struct SmsMsgData_ST *msg = (struct SmsMsgData_ST *)buffer;
    int i;
    int msgWords;

    if(msg == NULL)
        return;

    switch (msg->xMsgHeader.msgType) {
        case MSG_SMS_GET_VERSION_EX_RES:
        {
            struct SmsVersionRes_ST *ver =
                (struct SmsVersionRes_ST *) msg;
            ver->ChipModel = le16toh(ver->ChipModel);
            break;
        }
        case MSG_SMS_DVBT_BDA_DATA:
        case MSG_SMS_DAB_CHANNEL:
        case MSG_SMS_DATA_MSG:
        {
            break;
        }
        default:
        {
            msgWords = (msg->xMsgHeader.msgLength -
                    sizeof(struct SmsMsgHdr_ST))/4;
            for (i = 0; i < msgWords; i++)
                msg->msgData[i] = le32toh(msg->msgData[i]);
            break;
        }
    }
#endif /* BIG_ENDIAN */
    return;
}

/* Handle message header */
void sms1xxx_endian_handle_message_header(void *msg)
{
#if BYTE_ORDER == BIG_ENDIAN
    struct SmsMsgHdr_ST *phdr = (struct SmsMsgHdr_ST *)msg;

    if(phdr == NULL)
        return;

    phdr->msgType = le16toh(phdr->msgType);
    phdr->msgLength = le16toh(phdr->msgLength);
    phdr->msgFlags = le16toh(phdr->msgFlags);
#endif /* BIG_ENDIAN */
    return;
}
