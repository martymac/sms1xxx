#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sysexits.h>
#include <err.h>
#include <sys/ioctl.h>

#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>

int main(void)
{
    int fd = -1;

    if((fd = open("/dev/dvb/adapter0/frontend0", O_RDWR)) < 0) {
        err(EX_IOERR, "Cannot open frontend device /dev/dvb/adapter0/frontend0");
    }
    else {
        printf("Successfully opened frontend0 device !\n");
    }

    struct dtv_property prop[2] = {
        { .cmd = DTV_API_VERSION },
        { .cmd = DTV_ENUM_DELSYS },
    };
    struct dtv_properties props = {
        .num = 2,
        .props = prop
    };

    if (ioctl(fd, FE_GET_PROPERTY, &props) != 0) {
        close(fd);
        errx(EX_IOERR, "FE_GET_PROPERTY ioctl() failed\n");
    }
    close(fd);

    printf("Found kernel API v%u.%u, user API v%u.%u\n",
        prop[0].u.data >> 8, prop[0].u.data & 0xFF,
        DVB_API_VERSION, DVB_API_VERSION_MINOR);

    for (size_t i = 0; i < prop[1].u.buffer.len; i++) {
        uint8_t sys = prop[1].u.buffer.data[i];
        printf("Found system %u\n", sys);
    }

    return (EX_OK);
}
