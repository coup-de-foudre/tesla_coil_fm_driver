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

#include <iostream>
#include "transmitter.h"
#include "wave_reader.h"
#include "alsa_reader.h"
#include <cstdlib>
#include <csignal>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>


using namespace std;

Transmitter* transmitter = NULL;

void sigIntHandler(int sigNum)
{
    if (transmitter != NULL) {
        LOG_INFO << "Stopping...";
        transmitter->stop();
    }
}

int main(int argc, char** argv)
{
    double frequencyMHz = 100.0;
    double spreadMHz = 0.078;

    bool loop = false;
    string filename;
    bool debugLog = false;
    bool showUsage = true;
    string alsaDevice = "plughw:1,0";
    int startDelay = 0;

    for (int i = 1; i < argc; i++) {
        if (string("-f") == argv[i]) {
            if (i < argc - 1) {
                frequencyMHz = ::atof(argv[i + 1]);
                i++;
            }
        } else if (string("-s") == argv[i]) {
            if (i < argc - 1) {
                spreadMHz = ::atof(argv[i + 1]);
                i++;
            }
        } else if (string("-r") == argv[i]) {
            loop = true;
        } else if (string("-v") == argv[i]) {
            debugLog = true;
        } else if (string("-d") == argv[i]) {
            if (i < argc - 1) {
                alsaDevice = argv[i+1];
                i++;
            }
        } else if (string("-D") == argv[i]) {
            startDelay = atoi(argv[i+1]);
            i++;
        } else {
            if (i == argc - 1) {
                showUsage = false;
                filename = argv[i];
            }
        }
    }

    // PLog documentation at https://github.com/SergiusTheBest/plog
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;

    if (debugLog) {
        plog::init(plog::debug, &consoleAppender);
    } else {
        plog::init(plog::info, &consoleAppender);
    }

    if (showUsage) {
        cout << "Usage: " << argv[0]
             << " [-f frequencyMHz=100.0] [-s spreadMHz=0.078]"
             << " [-v] [-r] [-d alsa-device] [-D seconds] FILE" << endl;
        return 0;
    }

    LOG_INFO << "Running with params: ";
    LOG_INFO << "frequencyMHz\t" << frequencyMHz;
    LOG_INFO << "spreadMHz   \t" << spreadMHz;
    LOG_INFO << "filename    \t" << filename;
    LOG_INFO << "startDelay  \t" << startDelay;

    sleep(startDelay);

    signal(SIGINT, sigIntHandler);

    AbstractReader* reader = NULL;
    if (filename == "-") {
        reader = AlsaReader::getInstance(alsaDevice);
    } else {
        reader = new WaveReader(filename);
    }
    AudioFormat* format = reader->getFormat();
    LOG_INFO << "Playing: " << ((filename != "-") ? filename : "stdin") << ", "
             << format->sampleRate << " Hz, "
             << format->bitsPerSample << " bits, "
             << ((format->channels > 0x01) ? "stereo" : "mono");
    delete format;

    transmitter = Transmitter::getInstance(reader, frequencyMHz, spreadMHz);
    try {
        transmitter->run(loop);
        transmitter->stop();
    } catch (exception &error) {
        LOG_ERROR << "Error: " << error.what();
        transmitter->stop();
        return 1;
    }
    return 0;
}
