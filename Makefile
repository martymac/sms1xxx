all:
	${MAKE} -f Makefile.kld all

install: all
	${MAKE} -f Makefile.kld install

clean:
	${MAKE} -f Makefile.kld clean

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
