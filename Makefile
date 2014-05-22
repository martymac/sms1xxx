all:
	${MAKE} -f Makefile.kld all

install:
	${MAKE} -f Makefile.kld install

clean:
	${MAKE} -f Makefile.kld clean

debug:
	echo '#define USB_DEBUG 1' > opt_usb.h
	echo '#define DIAGNOSTIC 1' >> opt_usb.h
	${MAKE} -f Makefile.kld all

diagnostic:
	echo '#define DIAGNOSTIC 1' > opt_usb.h
	${MAKE} -f Makefile.kld all
