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
#include <plog/Log.h>

#include <iostream>

using std::ostringstream;

#define STDIN_READ_DELAY 700000

#define HEX_STREAM(X) "0x" << std::setfill('0') << std::setw(8) << std::hex << X


double Transmitter::centerFreqMHz_ = 0.0;
double Transmitter::spreadMHz_ = 0.0;
double Transmitter::currentValue_ = 0.0;

bool Transmitter::isTransmitting_ = false;
long long unsigned Transmitter::frameOffset_ = 0;
vector<float>* Transmitter::buffer_ = NULL;
void* Transmitter::mmapPeripherals_ = NULL;

unsigned Transmitter::clockOffsetAddr_ = CM_GP0CTL;

/*
 * Code flow plan:
 *
 * Transmitter::Transmitter()
 *    - Current:
 *        - memory map peripherals
 *    - Should:
 *        - memory map peripherals,
 *        - safely start the clock
 *             :: disable clock
 *             :: set divisor safely before start
 *             :: wait for busy bit to clear
 *             :: enable clock
 *             :: wind transmit frequency to center frequency over period of time
 *
 * Transmitter::~Transmitter()
 *    - Current:
 *        - unmap peripherals
 *    - Should:
 *        - gracefully stop clock
 *            :: wind transmit frequency away from center frequency over period of time
 *            :: disable clock
 *            :: wait for busy bit to clear
 *        - unmap peripherals
 *
 *
 * Transmitter::play()
 *    - Current:
 *        - Reads first frame
 *        - Starts transmit thread with sample rate set
 *        -
  *    - Should:
 *        - TODO
 *
 * Transmitter::transmit()
 *    - Current:
 *        - enables clock
 *        - sets divisor
 *        - plays sample when enough time passes (if ready)
 *    -
 */

void* mmapPeripherals() {
    LOG_DEBUG << "Memory mapping peripherals";

    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw ErrorReporter("Cannot open /dev/mem (permission denied)");
    }
    void* peripheralsBase = mmap(NULL,
                                 PERIPHERALS_LENGTH,
                                 PROT_READ | PROT_WRITE,
                                 MAP_SHARED,
                                 memFd,
                                 PERIPHERALS_BASE);
    close(memFd);
    if (peripheralsBase == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals_ (mmap error)");
    }
    return peripheralsBase;
}


/**
 * Hard shutdown of the clock
 *
 * Returns the initial clock manager state.
 */
unsigned Transmitter::clkShutdownHard() {
    LOG_DEBUG << "Hard shutdown of clock " << HEX_STREAM(clockOffsetAddr_);

    // Disable clock and wait for it to become available
    unsigned cmCtlInitialState = ACCESS(mmapPeripherals_, clockOffsetAddr_);
    ACCESS(mmapPeripherals_, clockOffsetAddr_) = (cmCtlInitialState & 0x00FFFFEF) | CM_PASSWD;
    while (true) {
        volatile bool enabledOrBusy =
            ACCESS(mmapPeripherals_, clockOffsetAddr_) & (0x01 << 4 | 0x01 << 7);
        if (!enabledOrBusy) {
            LOG_DEBUG << "Clock shutdown complete";
            break;
        }
        usleep(1);
    }
    return cmCtlInitialState;
}


/**
 * Hard init of the clock
 *
 * Returns the clock divisor.
 */
unsigned Transmitter::clkInitHard(double freqMHz, bool shutdown=true) {
    if (shutdown) clkShutdownHard();

    LOG_DEBUG << "Hard-init of clock " << HEX_STREAM(clockOffsetAddr_)
              << " at frequency " << freqMHz << " MHz";

    unsigned clkDivisor = clkDivisorSet(freqMHz);

    // Set GPIO pin 4 alternate function 1 (GPCLK0)
    ACCESS(mmapPeripherals_, GPFSEL0) =
        (ACCESS(mmapPeripherals_, GPFSEL0) & 0xFFFF8FFF) | (0x01 << 14);

    // Set up the clock manager
    ACCESS(mmapPeripherals_, CM_GP0CTL) =
        CM_PASSWD | CM_MASH1 | CM_ENAB | CM_SRC_PLLD;

    return clkDivisor;
}


/**
 * Soft-start the clock
 */
unsigned clkStartSoft(double finalFreqMHz, double startFreqMHz, unsigned slewTimeMs) {
    return 0; // TODO
}

/**
 * Set the clock divisor from a given frequency
 *
 * Returns the new clock divisor.
 */
unsigned Transmitter::clkDivisorSet(double targetFreqMHz) {
    double divisor = PLLD_FREQ_MHZ / targetFreqMHz;
    unsigned clockDivisor;
    if (divisor <= 0.0) {
        clockDivisor = 1;
    } else if (divisor >= 4095.5) {
        clockDivisor = 1 << 24;
    } else {
        clockDivisor = (unsigned) (divisor * 4096.0 + 0.5);
    }
    ACCESS(mmapPeripherals_, CM_GP0DIV) = CM_PASSWD | (0x00FFFFFF & clockDivisor);

    return clockDivisor;
}

/**
 * Safely initialize the clock on GPIO pin 4
 */
// void Transmitter::initClock() {
//     LOG_INFO << "Initializing clock";

//     clkShutdownHard(CM_GP0CTL); // TODO save initial state?

//     // Set GPIO pin 4 alternate function 0 (GPCLK0): Clear bits 12 and 13, and set bit 14
//     unsigned gpfSel0InitialState_ = ACCESS(mmapPeripherals_, GPFSEL0);
//     ACCESS(mmapPeripherals_, GPFSEL0) = (gpfSel0InitialState_ & 0xFFFF8FFF) | (0x01 << 14);
//     LOG_DEBUG << "Initial GPFSEL0 state: " << HEX_STREAM(gpfSel0InitialState_);


//     // Set up the clock manager
//     // PASSWD (0x5A << 24)  // required
//     // MASH (0x01 << 9)  // 1-stage mash filter
//     // ENAB (0x01 << 4) // enable the clock
//     // SRC (0x06)  // 6 = PLLD per  (runs at 500 MHz)
//     ACCESS(mmapPeripherals_, CM_GP0CTL) =
//         (0x5A << 24) |
//         (0x01 << 9) |
//         (0x01 << 4) |
//         0x06;

//     // TODO: Reset clock divisor to safe value?
//     unsigned cmGp0DivPreviousState_ = ACCESS(mmapPeripherals_, CM_GP0DIV);
//     LOGD << "Initial CM_GP0DIV state: " << HEX_STREAM(cmGp0DivPreviousState_);


//     // This enables all 3...
//     //ACCESS(peripherals_, GPFSEL0) = (ACCESS(peripherals_, GPFSEL0) & 0xFFE00FFF) | (0x01 << 14) | (0x01 << 17) | (0x01 << 20);
// }

Transmitter::Transmitter()
{
    LOG_DEBUG << "Initializing transmitter";
    mmapPeripherals_ = mmapPeripherals();
    // TODO soft-start
}

Transmitter::~Transmitter()
{
    munmap((void*)mmapPeripherals_, PERIPHERALS_LENGTH);
    // TODO: Reset the GPIO
}

Transmitter* Transmitter::getInstance()
{
    static Transmitter instance;
    return &instance;
}

// Set the transmit frequency offset from the current spreadFreqMHz and centerFreqMHz
void Transmitter::setTransmitValue(double value){
    currentValue_ = value;
    double targetFreqMHz = (spreadMHz_ * value + centerFreqMHz_);
    clkDivisorSet(targetFreqMHz);
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
        LOG_ERROR << "Cannot play, transmitter already in use";
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
    usleep(100); // DEBUGGING

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
    clkInitHard(centerFreqMHz_);

    // Set up clock and peripherals
    frameOffset_ = 0;

    // playbackStartMicroseconds = current clock timer
    playbackStartMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
    currentMicroseconds = playbackStartMicroseconds;
    startMicroseconds = playbackStartMicroseconds;
    while (isTransmitting_) {
        while ((buffer_ == NULL) && isTransmitting_) {
            usleep(1);
            currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
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
                currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);

                offset = ((currentMicroseconds - startMicroseconds) * (sampleRate)) / 1000000;
            }
        }

        startMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
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
    // TODO: Soft shutdown

    doStop = true;
    clkShutdownHard();
}

