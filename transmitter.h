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

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "error_reporter.h"
#include "audio_format.h"
#include <vector>
#include <thread>
#include <mutex>

#define BUFFER_FRAMES 2048

using std::vector;
using std::string;

class Transmitter {
    // TODO remove statics and create singleton:
    // https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

 public:
    virtual ~Transmitter();

    void play(string filename, string alsaDevice, double frequencyMHz, double spreadMHz, bool loop);
    void stop();

    static Transmitter* getInstance();
    static AudioFormat* getFormat(string filename, string alsaDevice);

private:
    Transmitter();

    static void setTransmitValue(double value);
    static void* transmit(unsigned sampleRate);

    static unsigned clkSlew(double finalFreqMHz,
                            double startFreqMHz,
                            double slewTimeMicroseconds);
    static unsigned clkShutdownHard(bool lock);
    static unsigned clkShutdownSoft();

    static unsigned clkInitHard(double freqMHz, bool lock);
    static unsigned clkInitSoft();

    static unsigned clkDivisorSet(double targetFreqMHz);
    void setCenterFreqMHz(double centerFreqMHz);
    void setSpreadMHz(double spreadMHz);
    void initClock();

    static double centerFreqMHz_;
    static double spreadMHz_;
    static double currentValue_;
    static void* mmapPeripherals_;
    static unsigned clockOffsetAddr_;

    static std::mutex transmitMutex_;

    static vector<float>* buffer_;
    static unsigned long long frameOffset_;
    static bool isTransmitting_;
    static bool doStop;

};

#endif // TRANSMITTER_H
