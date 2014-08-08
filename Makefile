# Makefile - GpsBoatC

DEBUG	= -g -O0
#DEBUG	= -O3
CC	= gcc
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm

SRC	=	main.cpp TinyGPS.cpp HMC5883L.cpp Arduino.cpp ADXL345.cpp tools.cpp

OBJ	=	$(SRC:.cpp=.o)

BINS	=	$(SRC:.cpp=)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(OBJ)
#	gcc -o $@ $^ $(LDFLAGS) $(LDLIBS)
	gcc -o gpsboat $^ $(LDFLAGS) $(LDLIBS)

test:
	gcc -o test test.cpp HMC58X3.cpp ADXL345.cpp $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o
