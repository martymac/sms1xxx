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
 *  GPIO management
 * * * *
 */

#include "sms1xxx.h"
#include "sms1xxx-gpio.h"
#include "sms1xxx-usb.h"

/*************************
 *** Private functions ***
 *************************/

/* Configure and set GPIO level */
static int
sms1xxx_gpio_set(struct sms1xxx_softc *sc, int pin, int enable)
{
    TRACE(TRACE_GPIO, "pin=%d, enable=%d\n", pin, enable);

    int lvl, ret;
    u32 gpio;

    struct sms1xxx_gpio_config gpioconfig;
    gpioconfig.Direction            = SMS_GPIO_DIRECTION_OUTPUT;
    gpioconfig.PullUpDown           = SMS_GPIO_PULL_UP_DOWN_NONE;
    gpioconfig.InputCharacteristics = SMS_GPIO_INPUT_CHARACTERISTICS_NORMAL;

    if ((sc->sc_type & SMS1XXX_FAMILY_MASK) == SMS1XXX_FAMILY1) {
        gpioconfig.OutputSlewRate   = SMS_GPIO_OUTPUT_SLEW_RATE_FAST;
        gpioconfig.OutputDriving    = SMS_GPIO_OUTPUT_DRIVING_S_4mA;
    }
    else {
        gpioconfig.OutputSlewRate   = SMS_GPIO_OUTPUT_SLEW_RATE_0_45_V_NS;
        gpioconfig.OutputDriving    = SMS_GPIO_OUTPUT_DRIVING_4mA;
    }

    if (pin == 0)
        return (EINVAL);

    if (pin < 0) {
        /* inverted gpio */
        gpio = pin * -1;
        lvl = enable ? 0 : 1;
    } else {
        gpio = pin;
        lvl = enable ? 1 : 0;
    }

    ret = sms1xxx_usb_gpio_configure(sc, gpio, &gpioconfig);
    if (ret != 0)
        return (ret);

    return (sms1xxx_usb_gpio_set_level(sc, gpio, lvl));
}

/* Set board-specific GPIO options */
static void
sms1xxx_gpio_config(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_GPIO, "\n");

    sc->gpio.led_power =
    sc->gpio.led_lo    =
    sc->gpio.led_hi    =
    sc->gpio.lna_ctrl  =
    sc->gpio.rf_switch = SMS_GPIO_PIN_UNDEF;

    sc->gpio.use_lna  = 0;

    switch(sc->sc_type) {
        case SMS1XXX_BOARD_HAUPPAUGE_WINDHAM:
            sc->gpio.led_power = 26;
            sc->gpio.led_lo    = 27;
            sc->gpio.led_hi    = 28;
        break;
        case SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD:
            sc->gpio.lna_ctrl  = 29;
            sc->gpio.rf_switch = 17;
        break;
        case SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD_R2:
            sc->gpio.lna_ctrl  = -1;
        break;
    }
    return;
}

/* Locally set current status led state */
static int
sms1xxx_gpio_status_led_state(struct sms1xxx_softc *sc, int state)
{
    if (state >= 0)
        sc->gpio.status_led_state = state;
    return (sc->gpio.status_led_state);
}

/*************************
 *** Public functions ***
 *************************/

/* Translate GPIO pin nums */
int
sms1xxx_gpio_get_pin_params(u32 pin_num, u32 *translated_pin_num,
    u32 *group_num, u32 *group_cfg)
{
    *group_cfg = 1;

    if (pin_num <= 1)    {
        *translated_pin_num = 0;
        *group_num = 9;
        *group_cfg = 2;
    } else if (pin_num >= 2 && pin_num <= 6) {
        *translated_pin_num = 2;
        *group_num = 0;
        *group_cfg = 2;
    } else if (pin_num >= 7 && pin_num <= 11) {
        *translated_pin_num = 7;
        *group_num = 1;
    } else if (pin_num >= 12 && pin_num <= 15) {
        *translated_pin_num = 12;
        *group_num = 2;
        *group_cfg = 3;
    } else if (pin_num == 16) {
        *translated_pin_num = 16;
        *group_num = 23;
    } else if (pin_num >= 17 && pin_num <= 24) {
        *translated_pin_num = 17;
        *group_num = 3;
    } else if (pin_num == 25) {
        *translated_pin_num = 25;
        *group_num = 6;
    } else if (pin_num >= 26 && pin_num <= 28) {
        *translated_pin_num = 26;
        *group_num = 4;
    } else if (pin_num == 29) {
        *translated_pin_num = 29;
        *group_num = 5;
        *group_cfg = 2;
    } else if (pin_num == 30) {
        *translated_pin_num = 30;
        *group_num = 8;
    } else if (pin_num == 31) {
        *translated_pin_num = 31;
        *group_num = 17;
    } else
        return (-1);

    *group_cfg <<= 24;

    return (0);
}

/* Configure and initialize GPIO */
int
sms1xxx_gpio_init(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_GPIO, "\n");

    /* Set board-specific GPIO options */
    sms1xxx_gpio_config(sc);

    /* Turn off all LEDs but power LED */
    sms1xxx_gpio_power_led_feedback(sc, SMS_LED_PW);
    sms1xxx_gpio_status_led_feedback(sc, SMS_LED_OFF);

    /* Turn off LNA */
    /* XXX rf_switch is turned off by sms1xxx_gpio_lna_control */
    sms1xxx_gpio_lna_control(sc, 0);

    return (0);
}

/* Un-initialize GPIO */
int
sms1xxx_gpio_exit(struct sms1xxx_softc *sc)
{
    TRACE(TRACE_GPIO, "\n");

    /* Turn off all LEDs */
    sms1xxx_gpio_power_led_feedback(sc, SMS_LED_OFF);
    sms1xxx_gpio_status_led_feedback(sc, SMS_LED_OFF);

    /* Turn off LNA */
    /* XXX rf_switch is turned off by sms1xxx_gpio_lna_control */
    sms1xxx_gpio_lna_control(sc, 0);

    return (0);
}

/***************************
 * Higher-level operations *
 ***************************/

/* Feedback on status LEDs */
int
sms1xxx_gpio_status_led_feedback(struct sms1xxx_softc *sc, int state)
{
    TRACE(TRACE_GPIO, "state=%d\n", state);

    int err;

    /* don't touch GPIO if LEDs are already set */
    if (sms1xxx_gpio_status_led_state(sc, -1) == state)
        return (0);

    if ((sc->gpio.led_lo == SMS_GPIO_PIN_UNDEF) ||
        (sc->gpio.led_hi == SMS_GPIO_PIN_UNDEF))
        return (EINVAL);

    err = sms1xxx_gpio_set(sc, sc->gpio.led_lo, (state & SMS_LED_LO) ? 1 : 0);
    if (err != 0)
        return (err);
    err = sms1xxx_gpio_set(sc, sc->gpio.led_hi, (state & SMS_LED_HI) ? 1 : 0);
    if (err != 0)
        return (err);
    sms1xxx_gpio_status_led_state(sc, state);
    return (0);
}

/* Feedback on power LED */
int
sms1xxx_gpio_power_led_feedback(struct sms1xxx_softc *sc, int state)
{
    TRACE(TRACE_GPIO, "state=%d\n", state);

    if (sc->gpio.led_power == SMS_GPIO_PIN_UNDEF)
        return (EINVAL);

    return (sms1xxx_gpio_set(sc, sc->gpio.led_power,
        (state & SMS_LED_PW) ? 1 : 0));
}

/* Turn LNA control and RF switch ON or OFF */
int
sms1xxx_gpio_lna_control(struct sms1xxx_softc *sc, int onoff)
{
    TRACE(TRACE_GPIO, "onoff=%d\n", onoff);

    int ret;

    if (sc->gpio.lna_ctrl == SMS_GPIO_PIN_UNDEF)
        return (EINVAL);

    if (sc->gpio.rf_switch != SMS_GPIO_PIN_UNDEF) {
        ret = sms1xxx_gpio_set(sc, sc->gpio.rf_switch, onoff ? 1 : 0);
        if (ret != 0)
            return (ret);
    }

    return (sms1xxx_gpio_set(sc, sc->gpio.lna_ctrl, onoff ? 1 : 0));
}
