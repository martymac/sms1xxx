CPPFLAGS+=  -I../../linux/include -I../../linux/include/uapi

all: FE_GET_PROPERTY

FE_GET_PROPERTY: FE_GET_PROPERTY.c
	$(CC) $(CPPFLAGS) -o FE_GET_PROPERTY FE_GET_PROPERTY.c

clean:
	rm -f FE_GET_PROPERTY
