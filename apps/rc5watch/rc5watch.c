/*  rc5watch : tool to watch and decode rc5 SMS1XXX IR events
 *
 *  Copyright (C) 2008-2009 - Ganaël Laplanche, http://contribs.martymac.org
 *
 *  This tool contains code taken from the Linux siano driver:
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
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>

#define RC5_PUSH_BIT(dst, bit, pos) \
    { dst <<= 1; dst |= bit; pos++; }

/* collected RC5 data */
struct {
    uint32_t ir_pos;
    uint32_t ir_word;
    int8_t ir_toggle;
} ir_data;
/* user options */
int next_sample=0;  /* = 0 : do not increment, unlimited watch
                       > 0 : increment up to max_sample */
int wait_toggle=0;  /* = 0 : wait for toggle change */
int count_toggle=0; /* = 0 : count toggle changes instead of events */

void
rc5_event(int8_t toggle, uint32_t addr, uint32_t cmd)
{
    uint8_t toggle_changed = (ir_data.ir_toggle != toggle);
    /* keep toggle */
    ir_data.ir_toggle = toggle;

    if(next_sample > 0) {
        if(count_toggle) {
            if(toggle_changed)
                next_sample++;
        }
        else
            next_sample++;
    }

    if(wait_toggle) {
        if (toggle_changed)
            printf("%d:%d:%d\n", toggle, addr, cmd);
    }
    else
        printf("%d:%d:%d\n", toggle, addr, cmd);

    return;
}

/* Decode raw bit pattern to RC5 code */
/* taken from ir-functions.c */
uint32_t
rc5_decode(unsigned int code)
{
    unsigned int pair;
    unsigned int rc5 = 0;
    int i;

    for (i = 0; i < 14; ++i) {
        pair = code & 0x3;
        code >>= 2;

        rc5 <<= 1;
        switch (pair) {
        case 0:
        case 2:
            break;
        case 1:
            rc5 |= 1;
            break;
        case 3:
            fprintf(stderr, "bad code\n");
            return (0);
        }
    }
    return (rc5);
}

void
rc5_parse_word(void)
{
    #define RC5_START(x)    (((x)>>12)&3)
    #define RC5_TOGGLE(x)   (((x)>>11)&1)
    #define RC5_ADDR(x)     (((x)>>6)&0x1F)
    #define RC5_INSTR(x)    ((x)&0x3F)

    int i, j = 0;
    uint32_t rc5_word = 0;

    /* Reverse the IR word direction */
    for (i = 0 ; i < 28 ; i++)
        RC5_PUSH_BIT(rc5_word, (ir_data.ir_word>>i)&1, j)

    rc5_word = rc5_decode(rc5_word);

    rc5_event(RC5_TOGGLE(rc5_word),
                RC5_ADDR(rc5_word),
                RC5_INSTR(rc5_word));
}

void
rc5_accumulate_bits(const int32_t ir_sample)
{
    #define RC5_TIME_GRANULARITY    200
    #define RC5_DEF_BIT_TIME        889
    #define RC5_MAX_SAME_BIT_CONT   4
    #define RC5_WORD_LEN            27 /* 28 bit */

    uint32_t i, j;
    int32_t delta_time;
    uint32_t time = (ir_sample > 0) ? ir_sample : (0 - ir_sample);
    uint32_t level = (ir_sample < 0) ? 0 : 1;

    for (i = RC5_MAX_SAME_BIT_CONT; i > 0; i--) {
        delta_time = time - (i*RC5_DEF_BIT_TIME) + RC5_TIME_GRANULARITY;
        if (delta_time < 0)
            continue; /* not so many consecutive bits */
        if (delta_time > (2 * RC5_TIME_GRANULARITY)) {
            /* timeout */
            if (ir_data.ir_pos == (RC5_WORD_LEN-1))
                /* complete last bit */
                RC5_PUSH_BIT(ir_data.ir_word, level, ir_data.ir_pos)

            if (ir_data.ir_pos == RC5_WORD_LEN)
                rc5_parse_word();
            else if (ir_data.ir_pos) /* timeout within a word */
                fprintf(stderr, "error parsing a word\n");

            ir_data.ir_pos = 0;
            ir_data.ir_word = 0;
            break;
        }
        /* The time is within the range of this number of bits */
        for (j = 0 ; j < i ; j++)
            RC5_PUSH_BIT(ir_data.ir_word, level, ir_data.ir_pos)

        break;
    }
}

void usage(void)
{
    printf("usage: rc5watch [-h] [-n num] [-t] [-T] [path to ir0 device]\n");
    printf("-h : this help\n");
    printf("-n : display <num> events, then exit\n");
    printf("-t : wait for toggle change to display events\n");
    printf("-T : (use with -n) restrict count to toggle changes "
        "instead of events\n");
    printf("output format: 'toggle:address:command'\n");
    return;
}

int main(int argc, char** argv)
{
    char *dev;
    int ch;
    long max_sample = 0;

    /* Each sample is 4-bytes long */
    uint8_t buf[4];
    const int32_t *sample = (const int32_t *)buf;

    ir_data.ir_pos =
    ir_data.ir_word = 0;
    ir_data.ir_toggle = -1;

    /* Options handling */
    while ((ch = getopt(argc, argv, "n:tTh?")) != -1)
        switch(ch) {
        case 'n':
            {
                char *p;
                next_sample = 1;
                max_sample = strtol(optarg, &p, 10);
                if(errno == EINVAL) {
                   fprintf(stderr, "-n : cannot convert value\n");
                   return (1);
                }
                else if(errno == ERANGE) {
                   fprintf(stderr, "-n : invalid value, too high\n");
                   return (1);
                }
                if ((*p) || (max_sample <= 0)) {
                   fprintf(stderr, "-n : invalid value, must be > 0\n");
                   usage();
                   return (1);
                }
                if (wait_toggle)
                    count_toggle = 1; /* used with '-n', '-t' implies '-T' */
            }
            break;
        case 't':
            wait_toggle = 1;
            if(next_sample > 0)
                count_toggle = 1; /* used with '-n', '-t' implies '-T' */
            break;
        case 'T':
            count_toggle = 1;
            break;
        case 'h':
        case '?':
        default:
            usage();
            return (0);
            break;
        };
    argc -= optind;
    argv += optind;

    /* device filename */
    if (argc > 1) {
      fprintf(stderr, "too many arguments\n");
      usage();
      return (1);
    }
    else if (argc == 1) {
        dev = argv[0];
    }
    else {
        dev = "/dev/dvb/adapter0/ir0";
    }

    int irfd = open(dev, O_RDONLY);
    if(irfd < 0) {
        fprintf(stderr, "cannot open device %s\n", dev);
        return (1);
    }

    setbuf(stdout, NULL); /* output not buffered */
    int i = 0;
    while((next_sample <= max_sample) && ((i = read(irfd, buf, 4)) > 0)) {
        rc5_accumulate_bits(sample[0]);
    }
    close(irfd);
    return (0);
}
