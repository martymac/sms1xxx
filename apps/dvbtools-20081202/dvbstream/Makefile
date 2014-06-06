CC?=gcc
CFLAGS=-g -Wall -O2
CPPFLAGS=-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -I../../../linux/include -I../../../linux/include/uapi
OBJS=dvbstream dumprtp ts_filter rtpfeed rtp.o 

ifdef UK
  CPPFLAGS += -DUK
endif

ifdef FINLAND
  CPPFLAGS += -DFINLAND
endif

ifdef FINLAND2
  CPPFLAGS += -DFINLAND2
endif

all: $(OBJS)

dvbstream: dvbstream.c rtp.o tune.o mpegtools/ctools.o mpegtools/remux.o mpegtools/transform.o mpegtools/ringbuffy.o
	$(CC) $(CPPFLAGS) $(CFLAGS) -o dvbstream dvbstream.c rtp.o tune.o mpegtools/ctools.o mpegtools/remux.o mpegtools/transform.o mpegtools/ringbuffy.o

dumprtp: dumprtp.c rtp.o 
	$(CC) $(CPPFLAGS) $(CFLAGS) -o dumprtp dumprtp.c rtp.o

rtpfeed: rtpfeed.c rtp.o 
	$(CC) $(CPPFLAGS) $(CFLAGS) -o rtpfeed rtpfeed.c rtp.o

rtp.o: rtp.c rtp.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o rtp.o rtp.c

tune.o: tune.c tune.h dvb_defaults.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o tune.o tune.c

ts_filter: ts_filter.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o ts_filter ts_filter.c

clean:
	rm -f  *.o mpegtools/*.o *~ $(OBJS)
