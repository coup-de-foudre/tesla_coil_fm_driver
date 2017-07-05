/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2015, Marcin Kondej, 2017 Mike McCoy
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
#include "peripherals.h"
#include <sstream>
#include <unistd.h>
#include <sys/mman.h>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "TemplateArgumentsIssues"
using std::ostringstream;

#define STDIN_READ_DELAY 700000


double Transmitter::centerFreqMHz_ = 0.0;
double Transmitter::spreadMHz_ = 0.0;
double Transmitter::currentValue_ = 0.0;

bool Transmitter::isTransmitting_ = false;
unsigned Transmitter::frameOffset_ = 0;
vector<float>* Transmitter::buffer_ = NULL;
void* Transmitter::peripheralsBase_ = NULL;


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

    peripheralsBase_ = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd,
                            isBcm2835 ? BCM2835_PERIPHERAL_BASE : PERIPHERAL_BASE);
    close(memFd);
    if (peripheralsBase_ == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals_ (mmap error)");
    }
}

Transmitter::~Transmitter()
{
    munmap(peripheralsBase_, 0x002FFFFF);
    // TODO: Reset the GPIO?
}

Transmitter* Transmitter::getInstance()
{
    static Transmitter instance;
    return &instance;
}

// Set the transmit frequency offset from the current spreadFreqMHz and centerFreqMHz
void Transmitter::setTransmitValue(double value){
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
    ACCESS(peripheralsBase_, CLK0DIV_BASE) = PASSWORD | (0x00FFFFFF & clockDivisor) ;
}

// Set the center frequency and update the transmission
void Transmitter::setCenterFreqMHz(double centerFreqMHz) {
    centerFreqMHz_ = centerFreqMHz;
    return setTransmitValue(currentValue_);
}

// Set the spread and update the transmission
void Transmitter::setSpreadMHz(double spreadMHz) {
    spreadMHz_ = spreadMHz;
    return setTransmitValue(currentValue_);
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
    StdinReader* stdinReader = NULL;
    AudioFormat* format;
    bool readStdin = filename == "-";

    if (!readStdin) {
        waveReader = new WaveReader(filename);
        format = waveReader->getFormat();
    } else {
        stdinReader = StdinReader::getInstance();
        format = stdinReader->getFormat();
        usleep(STDIN_READ_DELAY);
    }

    centerFreqMHz_ = centerFreqMHz;
    spreadMHz_ = spreadMHz;

    isTransmitting_ = true;
    doStop = false;

    unsigned bufferFrames = (unsigned)((unsigned long long)format->sampleRate * BUFFER_TIME / 1000000);

    buffer_ = (!readStdin) ? waveReader->getFrames(bufferFrames, 0) : stdinReader->getFrames(bufferFrames, doStop);

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
        while ((readStdin || !waveReader->isEnd((unsigned int) (frameOffset_ + bufferFrames))) && !doStop) {
            if (buffer_ == NULL) {
                if (!readStdin) {
                    buffer_ = waveReader->getFrames(bufferFrames, (unsigned int) (frameOffset_ + bufferFrames));
                } else {
                    buffer_ = stdinReader->getFrames(bufferFrames, doStop);
                }
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
    unsigned long offset;
    unsigned long length;
    unsigned long temp;
    vector<float>* frames = NULL;
    double value;
    float* data;
    unsigned sampleRate = *(unsigned*)(params);

    // Set up clock and peripherals

    // Clear GPFSEL0 bits 12, 13, 14 and set bit 14 (GPIO pin 4 alternate function 1, which is GPCLK0)
    ACCESS(peripheralsBase_, GPIO_BASE) = (ACCESS(peripheralsBase_, GPIO_BASE) & 0xFFFF8FFF) | (0x01 << 14);

    // This enables all 3...
    //ACCESS(peripherals_, GPIO_BASE) = (ACCESS(peripherals_, GPIO_BASE) & 0xFFE00FFF) | (0x01 << 14) | (0x01 << 17) | (0x01 << 20);

    // Set up the clock manager
    // PASSWD (0x5A << 24)  // required
    // MASH (0x01 << 9)  // 1-stage mash filter
    // ENAB (0x01 << 4) // enable the clock
    // SRC (0x06)  // 6 = PLLD per  (either runs at 400 or 500 MHz)
    ACCESS(peripheralsBase_, CLK0_BASE) =
        (0x5A << 24) |
        (0x01 << 9) |
        (0x01 << 4) |
        0x06;

    frameOffset_ = 0;

    // playbackStartMicroseconds = current clock timer
    playbackStartMicroseconds = ACCESS64(peripheralsBase_, TCNT_BASE);
    currentMicroseconds = playbackStartMicroseconds;
    startMicroseconds = playbackStartMicroseconds;

    while (isTransmitting_) {
        while ((buffer_ == NULL) && isTransmitting_) {
            usleep(1);
            currentMicroseconds = ACCESS64(peripheralsBase_, TCNT_BASE);
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

            // Clip value
            value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);
            setTransmitValue(value);

            while (temp >= offset) {
                asm("nop");  // Super tight timing loop will run CPU at full blast
                currentMicroseconds = ACCESS64(peripheralsBase_, TCNT_BASE);

                offset = ((currentMicroseconds - startMicroseconds) * (sampleRate)) / 1000000;
            }
        }

        startMicroseconds = ACCESS64(peripheralsBase_, TCNT_BASE);
        delete frames;
    }

    // Reset to zero
    setTransmitValue(0.0);

    return NULL;
}

AudioFormat* Transmitter::getFormat(string filename)
{
    WaveReader* file = NULL;
    StdinReader* stdinReader = NULL;
    AudioFormat* format = NULL;

   if (filename != "-") {
        file = new WaveReader(filename);
        format = file->getFormat();
        delete file;
    } else {
        stdinReader = StdinReader::getInstance();
        format = stdinReader->getFormat();
    }

   return format;
}

void Transmitter::stop()
{
    doStop = true;
}

#pragma clang diagnostic pop
#pragma clang diagnostic pop