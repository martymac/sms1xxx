CPPFLAGS+=  -I../../linux/include -I../../linux/include/uapi

all: FE_GET_PROPERTY test_dvr test_pes test_sections test_tapdmx

FE_GET_PROPERTY: FE_GET_PROPERTY.c
	$(CC) $(CPPFLAGS) -o FE_GET_PROPERTY FE_GET_PROPERTY.c

test_dvr: test_dvr.c
	$(CC) $(CPPFLAGS) -o test_dvr test_dvr.c

test_pes: test_pes.c
	$(CC) $(CPPFLAGS) -lutil -o test_pes test_pes.c

test_sections: test_sections.c
	$(CC) $(CPPFLAGS) -lutil -o test_sections test_sections.c

test_tapdmx: test_tapdmx.c
	$(CC) $(CPPFLAGS) -o test_tapdmx test_tapdmx.c

clean:
	rm -f FE_GET_PROPERTY test_dvr test_pes test_sections test_tapdmx
