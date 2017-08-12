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

#include <vector>
#include <thread>
#include <mutex>
#include <boost/lockfree/spsc_queue.hpp>
//#include "error_reporter.h"
#include "audio_format.h"
#include "abstract_reader.h"
#include "noisegate.h"

#define BUFFER_FRAMES 2048

using std::vector;
using std::string;

class Transmitter {
    // TODO remove statics and create singleton:
    // https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

 public:
  virtual ~Transmitter();

  void play(string filename, string alsaDevice, float frequencyMHz, float spreadMHz, bool loop);
  void stop();

  static Transmitter* getInstance(AbstractReader* reader, float centerFreqMHz, float spreadMHz);
  static AudioFormat* getFormat(string filename, string alsaDevice);
  static void run(bool loop);

private:
  Transmitter(AbstractReader* reader);

  static void transmit();
  static void setTransmitValue(float value);

  static unsigned clkSlew(float finalFreqMHz,
			  float startFreqMHz,
			  float slewTimeMicroseconds);
  static unsigned clkShutdownHard(bool lock);
  static void  clkShutdownSoft();
  static unsigned clkInitHard(float freqMHz, bool lock);
  static unsigned clkInitSoft();

  static unsigned clkDivisorSet(float targetFreqMHz);

  static float getCurrentTransmitFrequencyMHz();
  void setCenterFreqMHz(float centerFreqMHz);
  void setSpreadMHz(float spreadMHz);
  static void garbageCollector(boost::lockfree::spsc_queue<std::vector<float>*> *garbage);

  static AbstractReader* reader_;
  static float centerFreqMHz_;
  static float spreadMHz_;
  static float currentValue_;
  static void* mmapPeripherals_;
  static unsigned clockOffsetAddr_;
  volatile static bool doStop_;
  static std::mutex transmitMutex_;

  static boost::lockfree::spsc_queue<std::vector<float>*> garbage;
};

#endif // TRANSMITTER_H
