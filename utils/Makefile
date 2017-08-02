CPP = $(CCPREFIX)g++

INCLUDE = -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include -I/usr/local/include -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include-fixed -I/usr/include/arm-linux-gnueabihf -I/usr/include

LIBRARIES = -lasound -lm

CFLAGS += -fexceptions -O3 -fpermissive -fno-strict-aliasing -std=c++14 $(INCLUDE) $(LIBRARIES)

latency: latency.c
	$(CPP) $(CFLAGS) -o latency latency.c
