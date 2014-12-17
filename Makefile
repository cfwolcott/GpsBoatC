# Makefile - GpsBoatC

DEBUG	= -g -O0
#DEBUG	= -O3
CC	= g++
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm

SRC	=	main.cpp TinyGPS++.cpp HMC6343.cpp Arduino.cpp tools.cpp
OBJ	=	$(SRC:.cpp=.o) liblcd.a
EXEC	=	gpsboat

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(SRC) $(EXEC)

$(EXEC): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(LDLIBS) $(OBJ) -o $@

depend: .depend
.depend: $(SRC)
	rm -f ./.depend
	$(CC) $(CFLAGS) -MM $^ -MF  ./.depend
include .depend

.PHONY: clean
clean:
	rm *.o $(EXEC) -rf

test:
	gcc -o test test.cpp HMC6343.cpp $(LDFLAGS) $(LDLIBS)

