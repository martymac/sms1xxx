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
 */

#ifndef SMS1XXX_GPIO_H
#define SMS1XXX_GPIO_H

#include "sms1xxx.h"

#define MAX_GPIO_PIN_NUMBER                     31
#define SMS_GPIO_PIN_UNDEF                      (MAX_GPIO_PIN_NUMBER + 1)

struct sms1xxx_gpio_config {
#define SMS_GPIO_DIRECTION_INPUT                0
#define SMS_GPIO_DIRECTION_OUTPUT               1
    u8 Direction;

#define SMS_GPIO_PULL_UP_DOWN_NONE              0
#define SMS_GPIO_PULL_UP_DOWN_PULLDOWN          1
#define SMS_GPIO_PULL_UP_DOWN_PULLUP            2
#define SMS_GPIO_PULL_UP_DOWN_KEEPER            3
    u8 PullUpDown;

#define SMS_GPIO_INPUT_CHARACTERISTICS_NORMAL   0
#define SMS_GPIO_INPUT_CHARACTERISTICS_SCHMITT  1
    u8 InputCharacteristics;

#define SMS_GPIO_OUTPUT_SLEW_RATE_FAST          0 /* 10xx */
#define SMS_GPIO_OUTPUT_SLEW_RATE_SLOW          1 /* 10xx */


#define SMS_GPIO_OUTPUT_SLEW_RATE_0_45_V_NS     0 /* 11xx */
#define SMS_GPIO_OUTPUT_SLEW_RATE_0_9_V_NS      1 /* 11xx */
#define SMS_GPIO_OUTPUT_SLEW_RATE_1_7_V_NS      2 /* 11xx */
#define SMS_GPIO_OUTPUT_SLEW_RATE_3_3_V_NS      3 /* 11xx */
    u8 OutputSlewRate;

#define SMS_GPIO_OUTPUT_DRIVING_S_4mA           0 /* 10xx */
#define SMS_GPIO_OUTPUT_DRIVING_S_8mA           1 /* 10xx */
#define SMS_GPIO_OUTPUT_DRIVING_S_12mA          2 /* 10xx */
#define SMS_GPIO_OUTPUT_DRIVING_S_16mA          3 /* 10xx */

#define SMS_GPIO_OUTPUT_DRIVING_1_5mA           0 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_2_8mA           1 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_4mA             2 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_7mA             3 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_10mA            4 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_11mA            5 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_14mA            6 /* 11xx */
#define SMS_GPIO_OUTPUT_DRIVING_16mA            7 /* 11xx */
    u8 OutputDriving;
};

/* Board GPIO configuration and data */
struct sms1xxx_gpio {
    /* Board-specific PIN configuration */
    int led_power;
    int led_lo;
    int led_hi;
    int lna_ctrl;
    int rf_switch;

    /* Other */
    u32 get_res;          /* last GET result */
    int status_led_state; /* current status led state */
    u8 use_lna;           /* sysctl, use LNA when tuning ? */
};

int sms1xxx_gpio_get_pin_params(u32, u32 *, u32 *, u32 *);
int sms1xxx_gpio_init(struct sms1xxx_softc *);
int sms1xxx_gpio_exit(struct sms1xxx_softc *);

#define SMS_LED_OFF 0
#define SMS_LED_LO  1
#define SMS_LED_HI  2
#define SMS_LED_PW  4
int sms1xxx_gpio_status_led_feedback(struct sms1xxx_softc *, int);
int sms1xxx_gpio_power_led_feedback(struct sms1xxx_softc *, int);
int sms1xxx_gpio_lna_control(struct sms1xxx_softc *, int);

#endif
