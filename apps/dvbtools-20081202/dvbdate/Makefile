CFLAGS+=-std=c89
CPPFLAGS=-DNEWSTRUCT -I../../../linux/include -I../../../linux/include/uapi

CC?=gcc
all: dvbdate

dvbdate: dvbdate.o options.o

install: dvbdate
	cp dvbdate /usr/bin
clean:
	rm -f *.o dvbdate
