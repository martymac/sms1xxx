CC = gcc
CFLAGS = -MD -Wall -g -O2 -I../../include -I../lib
LFLAGS = -Wall -g -O2
RM = rm -f

TARGETS = tzap femon
OBJS = tzap.o femon.o

all: $(OBJS) $(TARGETS)
	@echo
	@echo "--------------------------------------------------------------------------------"
	@echo " please copy an appropriate channels.conf-XXX channel list for DVB-S/C/T"
	@echo
	@echo "   to ~/.tzap/channels.conf"
	@echo
	@echo " and then call ./tzap"
	@echo "--------------------------------------------------------------------------------"
	@echo

.c.o:
	$(CC) $(CFLAGS) -o $@ -c $<

.o:
	$(CC) $(LFLAGS) -o $@ $<

clean:
	$(RM) $(TARGETS) core* *.o *.d .depend

-include $(wildcard *.d) dummy

