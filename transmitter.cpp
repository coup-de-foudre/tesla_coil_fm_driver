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
#include "alsa_reader.h"
#include "peripherals.h"
#include <sstream>
#include <unistd.h>
#include <sys/mman.h>
#include <plog/Log.h>
#include <thread>


#include <iostream>

using std::ostringstream;

#define STDIN_READ_DELAY 700000

#define HEX_STREAM(X) "0x" << std::setfill('0') << std::setw(8) << std::hex << X


double Transmitter::centerFreqMHz_ = 0.0;
double Transmitter::spreadMHz_ = 0.0;
double Transmitter::currentValue_ = 0.0;
void* Transmitter::mmapPeripherals_ = NULL;


bool Transmitter::isTransmitting_ = false;
long long unsigned Transmitter::frameOffset_ = 0;
vector<float>* Transmitter::buffer_ = NULL;
volatile bool Transmitter::doStop = false;

std::mutex Transmitter::transmitMutex_;

unsigned Transmitter::clockOffsetAddr_ = CM_GP0CTL;
AbstractReader* Transmitter::reader_ = NULL;

// TODO: Not hard coded?
const double softOffDifferenceMHz_ = 0.050; // 250kHz off the center is "soft off"
const unsigned slewTimeMicroseconds_ = 3000000; // 1 second on/off slew

// Pause between frequency updates to avoid overloading the clock manager
const unsigned updateDelayMicroseconds_ = 10; // 100 kHz updates


/**
 * Memory-map the peripherals addresses
 */
inline void* mmapPeripherals() {
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


Transmitter::Transmitter(AbstractReader* reader) {
    LOG_DEBUG << "Initializing transmitter";
    reader_ = reader;
    mmapPeripherals_ = mmapPeripherals();
}


Transmitter::~Transmitter() {
    LOG_DEBUG << "Deleting transmitter";
    if (!doStop) {
        this->stop();
    }
    reader_->stop(true);
    munmap((void*)mmapPeripherals_, PERIPHERALS_LENGTH);
}


/**
 * Get singleton instance of transmitter
 */
Transmitter* Transmitter::getInstance(AbstractReader* reader,
                                      double centerFreqMHz,
                                      double spreadMHz) {
    centerFreqMHz_ = centerFreqMHz;
    spreadMHz_ = spreadMHz;
    static Transmitter instance(reader);
    return &instance;
}


/**
 * Slew clock from one frequency to another
 *
 * Return the final clock divisor
 */
unsigned Transmitter::clkSlew(double finalFreqMHz,
                              double startFreqMHz,
                              double slewTimeMicroseconds) {

    LOG_DEBUG << "Clock slew: finalFreqMHz=" << finalFreqMHz
              << ", startFreqMHz=" << startFreqMHz
              << ", slewTimeMicroseconds=" << (double)slewTimeMicroseconds;

    volatile unsigned long long startMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
    volatile unsigned long long currentMicroseconds = startMicroseconds;

    while (true) {
        if (slewTimeMicroseconds < currentMicroseconds - startMicroseconds) {
            break;
        }
        usleep(updateDelayMicroseconds_);
        currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
        double targetFreqMHz = startFreqMHz +
            ((double)(currentMicroseconds - startMicroseconds) / (double)(slewTimeMicroseconds)) *
            (finalFreqMHz - startFreqMHz);
        clkDivisorSet(targetFreqMHz);
    }

    // Final update to target frequency
    usleep(updateDelayMicroseconds_);
    return clkDivisorSet(finalFreqMHz);
}


/**
 * Hard shutdown of the clock
 *
 * Returns the clock manager state prior to shutdown
 */
unsigned Transmitter::clkShutdownHard(bool lock=true) {
    LOG_DEBUG << "Hard shutdown of clock " << HEX_STREAM(clockOffsetAddr_);

    // Lock the mutex for this scope
    if (lock) {
        LOG_DEBUG << "Acquiring lock...";
        transmitMutex_.lock();
    }
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

    transmitMutex_.unlock();
    return cmCtlInitialState;
}


/**
 * Hard init of the clock
 *
 * Returns the clock divisor.
 */
unsigned Transmitter::clkInitHard(double freqMHz, bool lock=true) {
    LOG_DEBUG << "Hard-init of clock " << HEX_STREAM(clockOffsetAddr_)
              << " to frequency " << freqMHz << " MHz";

    if (lock) {
        LOG_DEBUG << "Acquiring lock...";
        transmitMutex_.lock();
    }

    clkShutdownHard(false);

    unsigned clkDivisor = clkDivisorSet(freqMHz);

    // Set GPIO pin 4 alternate function 1 (GPCLK0)
    ACCESS(mmapPeripherals_, GPFSEL0) =
        (ACCESS(mmapPeripherals_, GPFSEL0) & 0xFFFF8FFF) | (0x01 << 14);

    // Set up the clock manager
    ACCESS(mmapPeripherals_, CM_GP0CTL) =
        CM_PASSWD | CM_MASH1 | CM_ENAB | CM_SRC_PLLD;

    if (lock) {
        transmitMutex_.unlock();
    }
    return clkDivisor;
}

/**
 * Soft shutdown of the clock
 *
 * Returns the state of the clock manager prior to shutdown
 */
unsigned Transmitter::clkShutdownSoft() {
    LOG_DEBUG << "Soft shutdown of clock";

    LOG_DEBUG << "Acquiring lock...";
    transmitMutex_.lock();
    clkSlew(centerFreqMHz_ + softOffDifferenceMHz_, centerFreqMHz_, slewTimeMicroseconds_);
    unsigned previousState =  clkShutdownHard(false);
    transmitMutex_.unlock();
    return previousState;
}


/**
 * Soft init of the clock
 *
 * Returns final clock divisor.
 */
unsigned Transmitter::clkInitSoft() {
    LOG_DEBUG << "Starting soft-init of clock to " << centerFreqMHz_ << " MHz";
    double startFreqMHz =  centerFreqMHz_ + softOffDifferenceMHz_;
    std::lock_guard<std::mutex> lock(transmitMutex_);
    clkInitHard(startFreqMHz, false);
    return clkSlew(centerFreqMHz_,  centerFreqMHz_ + softOffDifferenceMHz_, slewTimeMicroseconds_);
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
 * Set the transmit frequency offset from the current spreadFreqMHz and centerFreqMHz
 */
void Transmitter::setTransmitValue(double value){
    currentValue_ = value;
    double targetFreqMHz = (spreadMHz_ * value + centerFreqMHz_);
    clkDivisorSet(targetFreqMHz);
}


/**
 * Set the center frequency and update the transmission
 */
void Transmitter::setCenterFreqMHz(double centerFreqMHz) {
    centerFreqMHz_ = centerFreqMHz;
    return setTransmitValue(currentValue_);
}


/**
 * Set the spread and update the transmission
 */
void Transmitter::setSpreadMHz(double spreadMHz) {
    spreadMHz_ = spreadMHz;
    return setTransmitValue(currentValue_);
}


/**
 * Play from ALSA device or a file
 */
/*
void Transmitter::play(string filename,
                       string alsaDevice,
                       double centerFreqMHz,
                       double spreadMHz,
                       bool loop)
{
    LOG_DEBUG << "Playing: filename=" << filename
              << ", centerFreqMHz=" << centerFreqMHz
              << ", spreadMHz=" << spreadMHz
              << ", loop=" << loop;

    if (isTransmitting_) {
        LOG_ERROR << "Cannot play, transmitter already in use";
        throw ErrorReporter("Cannot play, transmitter already in use");
    }

    WaveReader* waveReader = NULL;
    AlsaReader* alsaReader = NULL;
    AudioFormat* format;
    bool readAlsa = (filename == "-");

    centerFreqMHz_ = centerFreqMHz;
    spreadMHz_ = spreadMHz;
    clkInitSoft();

    doStop = false;
    if (!readAlsa) {
        waveReader = new WaveReader(filename);
        format = waveReader->getFormat();
    } else {
        alsaReader = AlsaReader::getInstance(alsaDevice);
        format = alsaReader->getFormat();
        usleep(STDIN_READ_DELAY);
    }

    buffer_ = (!readAlsa) ? waveReader->getFrames(BUFFER_FRAMES, 0) : alsaReader->getFrames(BUFFER_FRAMES, doStop);

    std::thread activeThread (Transmitter::transmitThread, format->sampleRate);
    bool doPlay = true;
    while (doPlay && !doStop) {
        while ((readAlsa || !waveReader->isEnd((unsigned int) (frameOffset_ + BUFFER_FRAMES))) && !doStop) {
            if (buffer_ == NULL) {
                if (!readAlsa) {
                    buffer_ = waveReader->getFrames(BUFFER_FRAMES, (unsigned int) (frameOffset_ + BUFFER_FRAMES));
                } else {
                    buffer_ = alsaReader->getFrames(BUFFER_FRAMES, doStop);
                }
            }
            usleep(1);
        }
        if (loop && !readAlsa && !doStop) {
            isTransmitting_ = false;
            activeThread.join();
            buffer_ = waveReader->getFrames(BUFFER_FRAMES, 0);
            frameOffset_ = 0;
            std::thread newThread (Transmitter::transmitThread, format->sampleRate);
            newThread.swap(activeThread);
        } else {
            doPlay = false;
        }
    }
    isTransmitting_ = false;
    LOG_DEBUG << "Waiting for tranmsit thread to finish...";
    activeThread.join();
    LOG_DEBUG << "Transmit thread finished";

    if (!readAlsa) {
        delete waveReader;
    }
    delete format;
}
/**/


/**
 * Initialize and run the transmitter in a separate thread
 */
void Transmitter::run() {
    clkInitSoft();
    do {
        LOG_DEBUG << "Starting new thransmit thread";
        std::thread activeThread (Transmitter::transmit);
        activeThread.join();
        LOG_DEBUG << "Transmit thread finished. Resetting reader.";
        reader_->reset();
    } while (!doStop);
}


/**
 * Transmit all frames from a reader
 */
void Transmitter::transmit() {

    // TODO: Set this as a class variable in run()
    AudioFormat* format = reader_->getFormat();
    unsigned long sampleRate = format->sampleRate;
    delete format;

    vector<float>* frames = NULL;

    LOG_DEBUG << "Starting transmitter with sample rate " << sampleRate << "Hz";
    LOG_DEBUG << "Acquiring transmit lock...";
    transmitMutex_.lock();
    LOG_DEBUG << "Lock acquired";

    unsigned long nFramesProcessed = 0;
    const unsigned long long startMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
    volatile unsigned long long currentMicroseconds = startMicroseconds;

    // TODO: Remove reader_ from this transmitter code a queue in run()
    // so that this thread does not have any signal processing to do, just
    // timing
    while (!doStop && !reader_->isEnd()) {
        if (!reader_->getFrames(frames)) {
            LOG_DEBUG << "Transmit underrun: No frames available";
            continue;
        }
        // TODO: Signal processing... would be best in another thread

        LOG_DEBUG << "Got frames of size " << frames->size();
        for (unsigned ii = 0; ii < frames->size(); ii++) {

            unsigned long nextFrameTrigger = (nFramesProcessed * 1000000) / sampleRate;
            // Spin-wait
            while (!doStop && currentMicroseconds - startMicroseconds < nextFrameTrigger) {
                asm("nop");
                currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
            }
            if (doStop) break;
            setTransmitValue((*frames)[ii]);
            nFramesProcessed++;
        }

        delete frames;
        frames = NULL;
    }

    transmitMutex_.unlock();
    LOG_DEBUG << "Total frames transmitted: " << nFramesProcessed;
}

void* Transmitter::transmitThread(unsigned sampleRate)
{
    unsigned long long currentMicroseconds, startMicroseconds, playbackStartMicroseconds;
    unsigned long offset;
    unsigned long length;
    unsigned long temp;
    vector<float>* frames = NULL;
    double value;
    float* data;

    LOG_DEBUG << "Starting transmitter with sample rate " << sampleRate;
    frameOffset_ = 0;

    // playbackStartMicroseconds = current clock timer
    playbackStartMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
    currentMicroseconds = playbackStartMicroseconds;
    startMicroseconds = playbackStartMicroseconds;

    LOG_DEBUG << "Acquiring lock...";
    transmitMutex_.lock();
    isTransmitting_ = true;

    while (isTransmitting_) {
        while ((buffer_ == NULL) && isTransmitting_) {
            usleep(1);
        }
        if (!isTransmitting_) {
            break;
        }
        frames = buffer_;
        buffer_ = NULL;

        currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
        frameOffset_ = (currentMicroseconds - playbackStartMicroseconds) * (sampleRate) / 1000000;

        length = frames->size();
        data = &(*frames)[0];

        // Transmit entire buffer
        offset = 0;
        while (offset < length) {
            temp = offset;
            value = data[offset];

            // Clip value
            value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);
            setTransmitValue(value);

            // Spin-wait until next sample time passes
            while (temp >= offset) {
                asm("nop");
                currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
                offset = ((currentMicroseconds - startMicroseconds) * (sampleRate)) / 1000000;
            }
        }

        startMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
        delete frames;
    }

    // Reset to zero
    setTransmitValue(0.0);
    LOG_DEBUG << "Transmitter shut down";
    transmitMutex_.unlock();
    return NULL;
}

void Transmitter::stop()
{
    doStop = true;
    isTransmitting_ = false;
    clkShutdownSoft();
}

