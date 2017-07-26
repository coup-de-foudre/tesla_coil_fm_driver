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
#include "plog/Log.h"
#include <sstream>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <alsa/asoundlib.h>
#include <boost/lockfree/spsc_queue.hpp>

namespace bl = boost::lockfree;
using std::ostringstream;

bool AlsaReader::doStop = false;
bool AlsaReader::isReading = false;
bool AlsaReader::isDataAccess = false;
vector<float> AlsaReader::stream;
std::string AlsaReader::alsaDevice_ = "plughw:1,0";


int AlsaReader::setParams(snd_pcm_t* &capture_handle) {
    int err;
    snd_pcm_hw_params_t *hw_params;
    unsigned int rate = STREAM_SAMPLE_RATE;

    if ((err = snd_pcm_open (&capture_handle, alsaDevice_.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        LOG_ERROR <<  "cannot open audio device " << alsaDevice_ << " in blocking mode: " << snd_strerror(err);
        return err;
    }

    if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
        LOG_ERROR << "cannot allocate hardware parameter structure: " << snd_strerror(err);
        return err;
    }

    if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
        LOG_ERROR <<  "cannot initialize hardware parameter structure: " << snd_strerror(err);
        return err;
    }

    if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        LOG_ERROR <<  "cannot set access type: " << snd_strerror(err);
        return err;
    }
    if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_FLOAT_LE)) < 0) {
        LOG_ERROR <<  "cannot set sample format: " << snd_strerror(err);
        return err;
    }

    unsigned int rrate = rate;
    if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &rrate, 0)) < 0) {
        LOG_ERROR << "cannot set sample rate: " << snd_strerror(err);
        return err;
    }

    if (rrate != rate) {
        LOG_ERROR << "rate does not match: requested " << rate << "Hz, got " << rrate << "Hz";
        return -EINVAL;
    }

    if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, 1)) < 0) {
        LOG_ERROR << "cannot set channel count: " << snd_strerror (err);
        return err;
    }

    if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
        LOG_ERROR << "cannot set parameters: " << snd_strerror (err);
        return err;
    }

    snd_pcm_hw_params_free (hw_params);

    if ((err = snd_pcm_prepare (capture_handle)) < 0) {
        LOG_ERROR <<  "cannot prepare audio interface for use: " << snd_strerror (err);
        return err;
    }
    return 0;
}

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

/*
void* doTheThing() {
    // PRODUCER
    snd_pcm_t *capture_handle;
    bl::spsc_queue<std::vector<float>, bl::capacity<1>> queue;
    char[ALSA_FRAME_BUFFER_LENGTH * ALSA_FRAME_BYTES] readBuffer; // Fuck your stack.

    if (setParams(capture_handle)) {
        LOG_ERROR << "Unable to set ALSA parameters. Exiting.";
        // TODO: How to ensure transmitter shutdown?
        return;
    }

    while (!doStop) {
        int numFrames = snd_pcm_readi(capture_handle, readBuffer, ALSA_FRAME_BUFFER_LENGTH);

        if (numFrames != ALSA_FRAME_BUFFER_LENGTH) {
            LOG_ERROR << "Asked for " << ALSA_FRAME_BUFFER_LENGTH << " frames, got " << numFrames;
            continue; // TODO: Recover?
        }

        std::vector<float> valueBuffer;
        valueBuffer.reserve(numFrames);
        for (int i = 0; i < numFrames; i++) {
            float value = 0.0;
            const float maxScale = 1 << (4 * ALSA_FRAME_BYTES - 1);
            for (int j = 0; j < ALSA_FRAME_BYTES; j++) {
                int scale = 1 << (4 * j - 1);
                value += scale * (float) readBuffer[ALSA_FRAME_BYTES * i + j];
            }
            valueBuffer.push_back(value / maxScale);
        }
    }
}
*/

void* AlsaReader::readStdin(void *params)
{
    snd_pcm_t *capture_handle;
    int err;
    float *readBuffer = new float[1024];

    if ((err = setParams(capture_handle)) != 0) {
        LOG_ERROR << "Exiting" ;
        usleep(1000);
        exit(1);
    }

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
    		int numFrames = snd_pcm_readi(capture_handle, readBuffer, _len);

            if (numFrames > 0) {
                stream.insert(stream.end(), readBuffer, readBuffer + numFrames);
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

    if (streamSize < frameCount) {
        frameCount = streamSize;
    }
    vector<float>::const_iterator first = stream.begin();
    vector<float>::const_iterator last = stream.begin() + frameCount;
    vector<float> *frames = new vector<float>(first, last);

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
