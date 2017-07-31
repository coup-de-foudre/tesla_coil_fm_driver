CFLAGS += -Wall -fexceptions -pthread -lasound -lm -O3 -fpermissive -fno-strict-aliasing -std=c++14
TARGET = fm_transmitter
SHELL = /bin/bash
CPP=$(CCPREFIX)g++

INCLUDE=-I./include -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include -I/usr/local/include -I/usr/lib/gcc/arm-linux-gnueabihf/4.9/include-fixed -I/usr/include/arm-linux-gnueabihf -I/usr/include

PACKAGES=libasound2-dev

$(TARGET): main.o error_reporter.o wave_reader.o alsa_reader.o transmitter.o
	$(CPP) $(CFLAGS) $(INCLUDE) -o $(TARGET) main.o error_reporter.o wave_reader.o alsa_reader.o transmitter.o

.PHONY: all
all: packages $(TARGET)

wave_reader.o: wave_reader.cpp wave_reader.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c wave_reader.cpp

alsa_reader.o: alsa_reader.cpp alsa_reader.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c alsa_reader.cpp

error_reporter.o: error_reporter.cpp error_reporter.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c error_reporter.cpp

transmitter.o: transmitter.cpp transmitter.h peripherals.h
	$(CPP) $(CFLAGS) $(INCLUDE) -c transmitter.cpp

main.o: main.cpp
	$(CPP) $(CFLAGS) $(INCLUDE) -c main.cpp

.PHONY: install
install: fm_transmitter
	sudo cp fm_transmitter /usr/local/bin

.PHONY: daemon
daemon: install
	sudo cp ./system_configuration/fm_transmitter.service.__alsa__ /lib/systemd/system/fm_transmitter.service
	sudo systemctl daemon-reload
	sudo systemctl enable fm_transmitter
	sudo systemctl start fm_transmitter

.PHONY: uninstall-daemon
uninstall-daemon:
	sudo systemctl disable fm_transmitter  || echo "No need to disable fm_transmitter"
	sudo systemctl stop fm_transmitter || echo "fm_transmitter service not running"
	sudo rm -f /lib/systemd/system/fm_transmitter.service
	sudo systemctl daemon-reload

.PHONY: uninstall
uninstall: uninstall-daemon
	sudo rm -f /usr/local/bin/fm_transmitter

.PHONY: packages $(PACKAGES)
packages: $(PACKAGES)

$(PACKAGES):
	[ -z "`! dpkg -l | grep $@ -c >>/dev/null`" ] && sudo apt-get install -y $@

clean:
	rm *.o
