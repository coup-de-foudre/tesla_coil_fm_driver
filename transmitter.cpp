/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2015, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/fm_transmitter

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "transmitter.h"
#include "wave_reader.h"
#include "stdin_reader.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

using std::ostringstream;

// Let's define PERIPHERAL_BASE 0x3F000000

// GPIO Function Select 0 GPFSEL0
// GPFSEL0 = PERIPHERAL_BASE + GPIO_BASE
#define GPIO_BASE 0x00200000

// Clock Manager General Purpose Clocks Control CM_GP0CTL
#define CLK0_BASE 0x00101070
#define CLK1_BASE 0x00101078
#define CLK2_BASE 0x00101080

/// Clock Manager General Purpose Clock Divisors CM_GP0DIV
#define CLK0DIV_BASE 0x00101074
#define CLK1DIV_BASE 0x0010107c
#define CLK2DIV_BASE 0x00101084

// The ST system timer which provides a 64-bit system counter
//   CLO 32-bit lower part of the counter at register 0x7E003004 (virtual)
//   CHI 32-bit upper part of the counter at register 0x7E003008 (virtual)
#define TCNT_BASE 0x00003004

// The password for many of the peripherals
#define PASSWORD 0x5A000000

#define STDIN_READ_DELAY 700000

#define ACCESS(base, offset) *(volatile unsigned*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)



double Transmitter::centerFreqMHz_ = 0.0;
double Transmitter::spreadMHz_ = 0.0;
double Transmitter::spreadFactor_ = 0.0;
double Transmitter::currentValue_ = 0.0;

bool Transmitter::isTransmitting_ = false;
unsigned Transmitter::frameOffset_ = 0;
vector<float>* Transmitter::buffer_ = NULL;
void* Transmitter::peripherals_ = NULL;


Transmitter::Transmitter()
{
    bool isBcm2835 = true;

    FILE* pipe = popen("uname -m", "r");
    if (pipe) {
        char inputBuffer[64];
        string machine = "";
        while (!feof(pipe)) {
            if (fgets(inputBuffer, 64, pipe)) {
                machine += inputBuffer;
            }
        }
        pclose(pipe);

        machine = machine.substr(0, machine.length() - 1);
        if (machine != "armv6l") {
            isBcm2835 = false;
        }
    }

    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw ErrorReporter("Cannot open /dev/mem (permission denied)");
    }

    peripherals_ = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, isBcm2835 ? 0x20000000 : 0x3F000000);
    close(memFd);
    if (peripherals_ == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals_ (mmap error)");
    }
}

Transmitter::~Transmitter()
{
    munmap(peripherals_, 0x002FFFFF);
    // TODO: Reset the GPIO?
}

Transmitter* Transmitter::getInstance()
{
    static Transmitter instance;
    return &instance;
}

void Transmitter::setClockDivisor(double value){
    currentValue_ = value;
    double divisor = clockFreqMHz_ / (spreadMHz_ * value + centerFreqMHz_);
    unsigned clockDivisor;
    if (divisor <= 0.0) {
        clockDivisor = 1;
    } else if (divisor >= 4095.5) {
        clockDivisor = 1 << 24;
    } else {
        clockDivisor = (unsigned) (divisor * 4096.0 + 0.5);
    }
    ACCESS(peripherals_, CLK0DIV_BASE) = PASSWORD | (0x00FFFFFF & clockDivisor) ;
}

void Transmitter::play(string filename,
                       double centerFreqMHz,
                       double spreadMHz,
                       bool loop)
{
    if (isTransmitting_) {
        throw ErrorReporter("Cannot play, transmitter already in use");
    }

    WaveReader* waveReader = NULL;
    StdinReader* stdin = NULL;
    AudioFormat* format;
    bool readStdin = filename == "-";

    if (!readStdin) {
        waveReader = new WaveReader(filename);
        format = waveReader->getFormat();
    } else {
        stdin = StdinReader::getInstance();
        format = stdin->getFormat();
        usleep(STDIN_READ_DELAY);
    }

    centerFreqMHz_ = centerFreqMHz;
    spreadMHz_ = spreadMHz;
    // From the second-order Taylor expansion, scaled by 2^12
    spreadFactor_ = 4096.0 * (spreadMHz / centerFreqMHz) * (clockFreqMHz_ / centerFreqMHz);

    isTransmitting_ = true;
    doStop = false;

    unsigned bufferFrames = (unsigned)((unsigned long long)format->sampleRate * BUFFER_TIME / 1000000);

    buffer_ = (!readStdin) ? waveReader->getFrames(bufferFrames, 0) : stdin->getFrames(bufferFrames, doStop);

    pthread_t thread;
    void* params = (void*)&format->sampleRate;

    int returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, params);
    if (returnCode) {
        if (!readStdin) {
            delete waveReader;
        }
        delete format;
        ostringstream oss;
        oss << "Cannot create new thread (code: " << returnCode << ")";
        throw ErrorReporter(oss.str());
    }

    usleep(BUFFER_TIME / 2);

    bool doPlay = true;
    while (doPlay && !doStop) {
        while ((readStdin || !waveReader->isEnd(frameOffset_ + bufferFrames)) && !doStop) {
            if (buffer_ == NULL) {
                buffer_ = (!readStdin) ? waveReader->getFrames(bufferFrames, frameOffset_ + bufferFrames) : stdin->getFrames(bufferFrames, doStop);
            }
            usleep(BUFFER_TIME / 2);
        }
        if (loop && !readStdin && !doStop) {
            isTransmitting_ = false;

            buffer_ = waveReader->getFrames(bufferFrames, 0);

            pthread_join(thread, NULL);

            isTransmitting_ = true;

            returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, params);
            if (returnCode) {
                if (!readStdin) {
                    delete waveReader;
                }
                delete format;
                ostringstream oss;
                oss << "Cannot create new thread (code: " << returnCode << ")";
                throw ErrorReporter(oss.str());
            }
        } else {
            doPlay = false;
        }
    }
    isTransmitting_ = false;

    pthread_join(thread, NULL);

    if (!readStdin) {
        delete waveReader;
    }
    delete format;
}

void* Transmitter::transmit(void* params)
{
    unsigned long long currentMicroseconds, startMicroseconds, playbackStartMicroseconds;
    unsigned offset, length, temp;
    vector<float>* frames = NULL;
    float value = 0.0;
    float* data;
    unsigned sampleRate = *(unsigned*)(params);

    // This was wrapped in a #ifndef NO_PREEMP guard
    float prevValue = 0.0;
    float preemp = 0.75 - 250000.0 / (float)(sampleRate * 75);

    // Set up clock and peripherals

    // Clear GPFSEL0 bits 12, 13, 14 and set bit 14 (GPIO pin 4 alternate function 1, which is GPCLK0)
    ACCESS(peripherals_, GPIO_BASE) = (ACCESS(peripherals_, GPIO_BASE) & 0xFFFF8FFF) | (0x01 << 14);

    // This enables all 3...
    //ACCESS(peripherals_, GPIO_BASE) = (ACCESS(peripherals_, GPIO_BASE) & 0xFFE00FFF) | (0x01 << 14) | (0x01 << 17) | (0x01 << 20);


    // Set up the clock manager
    // PASSWD (0x5A << 24)  // required
    // MASH (0x01 << 9)  // 1-stage mash filter
    // ENAB (0x01 << 4) // enable the clock
    // SRC (0x06)  // 6 = PLLD per  (either runs at 400 or 500 MHz)



    ACCESS(peripherals_, CLK0_BASE) =
        (0x5A << 24) |
        (0x01 << 9) |
        (0x01 << 4) |
        0x06;
    // ACCESS(peripherals_, CLK1_BASE) =
    //  (0x5A << 24) |
    //  (0x01 << 9) |
    //  (0x01 << 4) |
    //  0x06;
    //ACCESS(peripherals_, CLK2_BASE) =
    //  (0x5A << 24) |
    //  (0x01 << 9) |
    //  (0x01 << 4) |
    //  0x06;

    frameOffset_ = 0;

    // playbackStartMicroseconds = current clock timer
    playbackStartMicroseconds = ACCESS64(peripherals_, TCNT_BASE);
    currentMicroseconds = playbackStartMicroseconds;
    startMicroseconds = playbackStartMicroseconds;

    while (isTransmitting_) {
        while ((buffer_ == NULL) && isTransmitting_) {
            usleep(1);
            currentMicroseconds = ACCESS64(peripherals_, TCNT_BASE);
        }
        if (!isTransmitting_) {
            break;
        }
        frames = buffer_;

        frameOffset_ = (currentMicroseconds - playbackStartMicroseconds) * (sampleRate) / 1000000;
        buffer_ = NULL;

        length = frames->size();
        data = &(*frames)[0];

        offset = 0;

        while (true) {
            temp = offset;
            if (offset >= length) {
                offset -= length;
                break;
            }

            value = data[offset];

            // Preemphasis correction
            value = value + (value - prevValue) * preemp;

            // Clip value
            value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);

            // 5A << 24 is the password, 16.0 = 16MHz the spread of the signal?
            // NB: newDivisor is a 24-bit number, with 12-bit integral and 12-bit fractional parts.
            setClockDivisor(value);

            while (temp >= offset) {
                asm("nop");  // Super tight timing loop will run CPU at full blast
                currentMicroseconds = ACCESS64(peripherals_, TCNT_BASE);

                // TODO: This overflows about every 71 minutes, which will result an immediate shutoff
                offset = (currentMicroseconds - startMicroseconds) * (sampleRate) / 1000000;
            }
            prevValue = value;
        }

        startMicroseconds = ACCESS64(peripherals_, TCNT_BASE);
        delete frames;
    }

    // Reset to zero
    ACCESS(peripherals_, CLK0_BASE) = (0x5A << 24);
    //ACCESS(peripherals_, CLK1_BASE) = (0x5A << 24);
    //ACCESS(peripherals_, CLK2_BASE) = (0x5A << 24);

    return NULL;
}

AudioFormat* Transmitter::getFormat(string filename)
{
    WaveReader* file;
    StdinReader* stdin;
    AudioFormat* format;

   if (filename != "-") {
        file = new WaveReader(filename);
        format = file->getFormat();
        delete file;
    } else {
        stdin = StdinReader::getInstance();
        format = stdin->getFormat();
    }

   return format;
}

void Transmitter::stop()
{
    doStop = true;
}
