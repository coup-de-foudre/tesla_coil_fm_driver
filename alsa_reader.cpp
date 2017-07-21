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

#include "alsa_reader.h"
#include "error_reporter.h"
#include <sstream>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <alsa/asoundlib.h>


using std::ostringstream;

bool AlsaReader::doStop = false;
bool AlsaReader::isReading = false;
bool AlsaReader::isDataAccess = false;
vector<char> AlsaReader::stream;
std::string AlsaReader::alsaDevice_ = "plughw:1,0";

AlsaReader::AlsaReader(std::string alsaDevice)
{
    alsaDevice_ = alsaDevice;
    int returnCode = pthread_create(&thread, NULL, &AlsaReader::readStdin, NULL);
    if (returnCode) {
        ostringstream oss;
        oss << "Cannot create new thread (code: " << returnCode << ")";
        throw ErrorReporter(oss.str());
    }

    while (!isReading) {
        usleep(1);
    }
}

AlsaReader::~AlsaReader()
{
    doStop = true;
    pthread_join(thread, NULL);
}

AlsaReader* AlsaReader::getInstance(string alsaDevice)
{
    static AlsaReader instance(alsaDevice);
    return &instance;
}

void *AlsaReader::readStdin(void *params)
{
    int err;
    unsigned int rate = STREAM_SAMPLE_RATE;
    snd_pcm_t *capture_handle;
    snd_pcm_hw_params_t *hw_params;

    if ((err = snd_pcm_open (&capture_handle, alsaDevice_.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf (stderr, "cannot open audio device %s (%s)\n",
             "fd",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
        fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf (stderr, "cannot set access type (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        fprintf (stderr, "cannot set sample format (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &rate, 0)) < 0) {
        fprintf (stderr, "cannot set sample rate (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, 1)) < 0) {
        fprintf (stderr, "cannot set channel count (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
        fprintf (stderr, "cannot set parameters (%s)\n",
             snd_strerror (err));
        exit (1);
    }

    snd_pcm_hw_params_free (hw_params);

    if ((err = snd_pcm_prepare (capture_handle)) < 0) {
        fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
             snd_strerror (err));
        exit (1);
    }


    char *readBuffer = new char[1024];

    while (!doStop) {
        isReading = true;

        while (isDataAccess && !doStop) {
            usleep(1);
        }
        if (doStop) {
            break;
        }
        unsigned streamSize = (unsigned int) stream.size();
        if (streamSize < MAX_STREAM_SIZE) {
		    int _len = (streamSize + 1024 > MAX_STREAM_SIZE) ? MAX_STREAM_SIZE - streamSize : 1024;
    		int bytes = snd_pcm_readi(capture_handle, readBuffer, _len / 2) * 2; // /2 and *2 because snd_pcm_readi works in units of 16bit frames

            if (bytes > 0) {
                stream.insert(stream.end(), readBuffer, readBuffer + bytes);
            }
        }

        isReading = false;
        usleep(1);
    }

    delete readBuffer;
    snd_pcm_close(capture_handle);

    return NULL;
}

vector<float>* AlsaReader::getFrames(unsigned frameCount, bool &forceStop)
{
    while (isReading && !forceStop) {
        usleep(1);
    }
    if (forceStop) {
        doStop = true;
        return NULL;
    }

    isDataAccess = true;

    unsigned offset, bytesToRead, bytesPerFrame;
    unsigned streamSize = (unsigned int) stream.size();
    if (!streamSize) {
        isDataAccess = false;
        return NULL;
    }

    vector<float> *frames = new vector<float>();
    bytesPerFrame = (STREAM_BITS_PER_SAMPLE >> 3) * STREAM_CHANNELS;
    bytesToRead = frameCount * bytesPerFrame;

    if (bytesToRead > streamSize) {
        bytesToRead = streamSize - streamSize % bytesPerFrame;
        frameCount = bytesToRead / bytesPerFrame;
    }

    vector<char> data;
    data.resize(bytesToRead);
    memcpy(&(data[0]), &(stream[0]), bytesToRead);
    stream.erase(stream.begin(), stream.begin() + bytesToRead);
    isDataAccess = false;

    for (unsigned i = 0; i < frameCount; i++) {
        offset = bytesPerFrame * i;
        if (STREAM_CHANNELS != 1) {
            if (STREAM_BITS_PER_SAMPLE != 8) {
                frames->push_back(((int)(signed char)data[offset + 1] + (int)(signed char)data[offset + 3]) / (float)0x100);
            } else {
                frames->push_back(((int)data[offset] + (int)data[offset + 1]) / (float)0x100 - 1.0f);
            }
        } else {
            if (STREAM_BITS_PER_SAMPLE != 8) {
                frames->push_back((signed char)data[offset + 1] / (float)0x80);
            } else {
                frames->push_back(data[offset] / (float)0x80 - 1.0f);
            }
        }
    }

    return frames;
}

AudioFormat* AlsaReader::getFormat()
{
    AudioFormat* format = new AudioFormat;
    format->sampleRate = STREAM_SAMPLE_RATE;
    format->bitsPerSample = STREAM_BITS_PER_SAMPLE;
    format->channels = STREAM_CHANNELS;
    return format;
}
