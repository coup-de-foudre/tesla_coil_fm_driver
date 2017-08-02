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
#include <pthread.h>

#include <iostream>

using std::ostringstream;

#define STDIN_READ_DELAY 700000

#define HEX_STREAM(X) "0x" << std::setfill('0') << std::setw(8) << std::hex << X


float Transmitter::centerFreqMHz_ = 0.0;
float Transmitter::spreadMHz_ = 0.0;
float Transmitter::currentValue_ = 0.0;
void* Transmitter::mmapPeripherals_ = NULL;

volatile bool Transmitter::doStop_ = false;

std::mutex Transmitter::transmitMutex_;

unsigned Transmitter::clockOffsetAddr_ = CM_GP0CTL;
AbstractReader* Transmitter::reader_ = NULL;

// Garbage from transmitter
boost::lockfree::spsc_queue<std::vector<float>*> Transmitter::garbage(256);

// TODO: Not hard coded?
const float softOffDifferenceMHz_ = 0.050; // 250kHz off the center is "soft off"
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
    if (!doStop_) {
        this->stop();
    }
    reader_->stop(true);
    munmap((void*)mmapPeripherals_, PERIPHERALS_LENGTH);
}


/**
 * Get singleton instance of transmitter
 */
Transmitter* Transmitter::getInstance(AbstractReader* reader,
                                      float centerFreqMHz,
                                      float spreadMHz) {
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
unsigned Transmitter::clkSlew(float finalFreqMHz,
                              float startFreqMHz,
                              float slewTimeMicroseconds) {

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
        float targetFreqMHz = startFreqMHz +
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
unsigned Transmitter::clkInitHard(float freqMHz, bool lock=true) {
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
void Transmitter::clkShutdownSoft() {
    LOG_DEBUG << "Soft shutdown of clock";
    LOG_DEBUG << "Acquiring lock...";
    transmitMutex_.lock();
    volatile bool enabled =
        ACCESS(mmapPeripherals_, clockOffsetAddr_) & (0x01 << 4);

    if (enabled) {
        clkSlew(centerFreqMHz_ + softOffDifferenceMHz_, centerFreqMHz_, slewTimeMicroseconds_);
        clkShutdownHard(false);
    } else {
        LOG_DEBUG << "Clock is not enabled. Not shutitng down.";
    }
    transmitMutex_.unlock();
}


/**
 * Soft init of the clock
 *
 * Returns final clock divisor.
 */
unsigned Transmitter::clkInitSoft() {
    LOG_DEBUG << "Starting soft-init of clock to " << centerFreqMHz_ << " MHz";
    float startFreqMHz =  centerFreqMHz_ + softOffDifferenceMHz_;
    std::lock_guard<std::mutex> lock(transmitMutex_);
    clkInitHard(startFreqMHz, false);
    return clkSlew(centerFreqMHz_,  centerFreqMHz_ + softOffDifferenceMHz_, slewTimeMicroseconds_);
}


/**
 * Set the clock divisor from a given frequency
 *
 * Returns the new clock divisor.
 */
inline unsigned Transmitter::clkDivisorSet(float targetFreqMHz) {
    const float divisor = PLLD_FREQ_MHZ / targetFreqMHz;
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
inline void Transmitter::setTransmitValue(float value){
    currentValue_ = value;
    float targetFreqMHz = (spreadMHz_ * value + centerFreqMHz_);
    clkDivisorSet(targetFreqMHz);
}


/**
 * Set the center frequency and update the transmission
 */
inline void Transmitter::setCenterFreqMHz(float centerFreqMHz) {
    centerFreqMHz_ = centerFreqMHz;
    return setTransmitValue(currentValue_);
}


/**
 * Set the spread and update the transmission
 */
inline void Transmitter::setSpreadMHz(float spreadMHz) {
    spreadMHz_ = spreadMHz;
    return setTransmitValue(currentValue_);
}


/**
 * Initialize and run the transmitter in a separate thread
 */
void Transmitter::run(bool loop) {
    clkInitSoft();
    std::thread garbageThread(Transmitter::garbageCollector, &garbage);
    do {
        LOG_DEBUG << "Starting new thransmit thread";

        sched_param sch_params;
        sch_params.sched_priority = 1;
        std::thread thread (Transmitter::transmit);
        if(pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sch_params)) {
            LOG_ERROR << "Failed to set Thread scheduling : " << std::strerror(errno);
        }
        thread.join();
        LOG_DEBUG << "Transmit thread finished. Resetting reader.";
        reader_->reset();
    } while (!doStop_ && loop);

    doStop_ = true;
    garbageThread.join();
}


/**
 * Delete the garbage from the queue
 */
void Transmitter::garbageCollector(boost::lockfree::spsc_queue<std::vector<float>*> *garbage) {
    LOG_DEBUG << "Garbage collector started";
    while (!doStop_) {
        usleep(500000); // TODO: Allow configuration
        int numCollected = 0;
        garbage->consume_all([&numCollected](vector<float>* element) -> void {
                delete element;
                numCollected++;
            });
        if (numCollected > 0) {
            LOG_DEBUG << "Garbage collected " << numCollected << " buffers";
        }
    }
}


/**
 * Transmit loop from queue
 */
void Transmitter::transmit() {

    // TODO: Set this as a class variable in run()
    AudioFormat* format = reader_->getFormat();
    float sampleRate = (float)format->sampleRate;
    delete format;

    vector<float>* frames = NULL;

    LOG_DEBUG << "Starting transmitter with sample rate " << sampleRate << "Hz";
    LOG_DEBUG << "Acquiring transmit lock...";
    transmitMutex_.lock();
    LOG_DEBUG << "Lock acquired";

    // TODO: Remove reader_ from this transmitter code a queue in run()
    // so that this thread does not have any signal processing to do, just
    // timing
    while (!doStop_ && !reader_->isEnd()) {
        // Spin-wait
        if (!reader_->getFrames(frames)) {
            continue;
        }

        // TODO: Signal processing... would be best in another thread?
        volatile unsigned long long frameStartMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
        unsigned bufferSize = frames->size();
        for (float ii = 0; ii < bufferSize; ii++) {
            unsigned long nextFrameTrigger = (unsigned) ((ii * 1000000.0) / sampleRate + 0.5);

            volatile unsigned long long currentMicroseconds;

            // Spin-wait
            do {
                currentMicroseconds = ACCESS64(mmapPeripherals_, ST_CLO);
            } while(!doStop_ &&
                    ((currentMicroseconds - frameStartMicroseconds) < nextFrameTrigger));

            if (doStop_) break;
            setTransmitValue((*frames)[ii]);
        }
        // Push frames to garbage queue for deletion elsewhere
        garbage.push(frames);
    }
    transmitMutex_.unlock();

    LOG_DEBUG << "Transmitter shut down";
}


void Transmitter::stop()
{
    reader_->stop(false);
    doStop_ = true;
    clkShutdownSoft();
}

