INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lpthread -lrt

HFILES= 
CFILES= main.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	main

clean:
	-rm -f *.o *.d
	-rm -f main
	-rm -f *.ppm

distclean:
	-rm -f *.o *.d

main: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)
	@echo "To run the code, a few arguments have to be provided on the command line"
	@echo "To generate 1800 frames at a resolution of 640 X 480 with socket disabled,"
	@echo "type: sudo ./main 640 480 1800 0"
	@echo "To generate 1800 frames at a resolution of 640 X 480 with socket enabled,"
	@echo "type: sudo ./main 640 480 1800 1"
	@echo "The frequency can be changed by defining and undefining the HERTZ macro in main.c"



depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
