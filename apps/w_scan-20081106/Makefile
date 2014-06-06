CC ?= gcc
CFLAGS = -MD -g -Wall -O2 -I../../linux/include -I../../linux/include/uapi
LFLAGS = -g -Wall

OBJS = dump-vdr.o dump-xine.o dump-dvbscan.o dump-kaffeine.o scan.o section.o atsc_psip_section.o
SRCS = $(OBJS:.o=.c)

TARGET = w_scan
VERSION = $(shell sed -n '/uint version/p' scan.c | sed -e 's/uint version = //' | sed -e 's/;//')
TMPDIR = w_scan-$(VERSION)

$(TARGET): $(OBJS)
	$(CC) $(LFLAGS) -o $(TARGET) $(OBJS)
	$(RM) *.o *.d
.c.o:
	$(CC) $(CFLAGS) --static -c $< -o $@

install:
	install -m 755 $(TARGET) /usr/bin
	install -m 755 w_scan_start.sh /usr/bin
	mkdir -p /usr/share/w_scan
	install pci.ids /usr/share/w_scan
	install pci.classes /usr/share/w_scan
	install usb.ids /usr/share/w_scan
	install usb.classes /usr/share/w_scan

dist:
	@-rm -rf $(TMPDIR)
	@mkdir $(TMPDIR)
	@cp -a *.c $(TMPDIR)
	@cp -a *.h $(TMPDIR)
	@cp -a Makefile $(TMPDIR)
	@cp -a pci.* $(TMPDIR)
	@cp -a usb.* $(TMPDIR)
	@cp -a README $(TMPDIR)
	@cp -a COPYING $(TMPDIR)
	@chmod -x $(TMPDIR)/*
	@cp -a w_scan $(TMPDIR)
	@cp -a w_scan_start.sh $(TMPDIR)
	@chmod +x $(TMPDIR)/w_scan_start.sh
	@tar cjf w_scan-$(VERSION).tar.bz2 $(TMPDIR)
	@-rm -rf $(TMPDIR)/
	@echo Distribution package created as w_scan-$(VERSION).tar.bz2
		
clean:
	$(RM) *.o *.d $(TARGET)

-include $(wildcard *.d) dummy

