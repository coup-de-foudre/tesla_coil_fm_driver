CFLAGS += -Wall -fexceptions -pthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = fm_transmitter

CPP=$(CCPREFIX)g++

INCLUDE=-I./include -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include -I/usr/local/include -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include-fixed -I/usr/include/arm-linux-gnueabihf -I/usr/include


all: main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o
	$(CPP) $(CFLAGS) $(INCLUDE) -o $(TARGET) main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o

wave_reader.o: wave_reader.cpp wave_reader.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c wave_reader.cpp

stdin_reader.o: stdin_reader.cpp stdin_reader.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c stdin_reader.cpp

error_reporter.o: error_reporter.cpp error_reporter.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c error_reporter.cpp

transmitter.o: transmitter.cpp transmitter.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c transmitter.cpp

main.o: main.cpp
	$(CPP) $(CFLAGS) $(INCLUDE) -c main.cpp

clean:
	rm *.o
