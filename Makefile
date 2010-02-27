INSTALL_MAN?=   install -m 444
MANPREFIX?= /usr/local
GZIP_CMD?=  /usr/bin/gzip -nf -9

all:
	${MAKE} -f Makefile.kld all

debug:
	echo '#define SMS1XXX_DIAGNOSTIC 1' > opt_usb.h
	echo '#define SMS1XXX_DEBUG 1' >> opt_usb.h
	echo '#define SMS1XXX_DEBUG_DEFAULT_LEVEL 0xfdff' >> opt_usb.h
	${MAKE} DEBUG_FLAGS=-g -f Makefile.kld all

mydebug:
	echo '#define SMS1XXX_DIAGNOSTIC 1' > opt_usb.h
	echo '#define SMS1XXX_DEBUG 1' >> opt_usb.h
	echo '#define SMS1XXX_DEBUG_DEFAULT_LEVEL 0xfdff' >> opt_usb.h
	echo '#define SMS1XXX_DEFAULT_FREQ_OFFSET 166000' >> opt_usb.h
	${MAKE} DEBUG_FLAGS=-g -f Makefile.kld all

diagnostic:
	echo '#define SMS1XXX_DIAGNOSTIC 1' > opt_usb.h
	${MAKE} -f Makefile.kld all

installkld:
	${MAKE} -f Makefile.kld install

installman:
	${INSTALL_MAN} man/man4/sms1xxx.4 ${MANPREFIX}/man/man4
	${GZIP_CMD} ${MANPREFIX}/man/man4/sms1xxx.4

install: installkld installman

clean:
	${MAKE} -f Makefile.kld clean
