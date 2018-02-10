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
#include <atomic>
#include <boost/lockfree/queue.hpp>

namespace bl = boost::lockfree;
using std::ostringstream;

bool AlsaReader::doStop = false;
bool AlsaReader::isDataAccess = false;
std::string AlsaReader::alsaDevice_ = "hw:1,0";

boost::lockfree::queue<std::vector<float>*> AlsaReader::queue(1);


AlsaReader::AlsaReader(std::string alsaDevice)
{
    alsaDevice_ = alsaDevice;
    int returnCode = pthread_create(&thread, NULL, &AlsaReader::read, NULL);
    if (returnCode) {
        ostringstream oss;
        oss << "Cannot create new thread (code: " << returnCode << ")";
        throw ErrorReporter(oss.str());
    }
}


AlsaReader::~AlsaReader()
{
    doStop = true;
}


AlsaReader* AlsaReader::getInstance(string alsaDevice)
{
    static AlsaReader instance(alsaDevice);
    return &instance;
}


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

    if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, STREAM_CHANNELS)) < 0) {
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


void* AlsaReader::read(void* params) {
    snd_pcm_t *capture_handle;
    unsigned bufferLength = ALSA_FRAME_BUFFER_LENGTH * STREAM_CHANNELS;;
    float* readBuffer = new float[bufferLength];
#if STREAM_CHANNELS == 2
    float* readBuffer2 = new float[bufferLength / 2];
#endif

    if (setParams(capture_handle)) {
        LOG_ERROR << "Unable to set ALSA parameters. Exiting.";
        doStop = 1;
    }

    while (!doStop) {
        int numFrames = snd_pcm_readi(capture_handle, readBuffer, bufferLength / STREAM_CHANNELS);
        if (numFrames < 0) {
            LOG_ERROR << "Error reading from ALSA device: " << snd_strerror(numFrames);
            doStop = 1;
            break;
        } else if ((unsigned)numFrames != (bufferLength / STREAM_CHANNELS)) {
            LOG_ERROR << "Asked for " << bufferLength << " frames, got " << numFrames;
            doStop = 1;
            break;
        }

#if STREAM_CHANNELS == 2
	for (unsigned i = 0; i < bufferLength / 2; ++i) {
		readBuffer2[i] = readBuffer[i * 2];
	}
        std::vector<float>* values = new std::vector<float>(readBuffer2, readBuffer2 + (bufferLength / 2));
#else
        std::vector<float>* values = new std::vector<float>(readBuffer, readBuffer + bufferLength);
#endif

        // Drain buffer
        int numConsumed = 0;
        queue.consume_all([&numConsumed] (vector<float>* element) -> void {
                numConsumed += element->size();
                delete element;
            });
        if (numConsumed > 0) {
            LOG_DEBUG << "Buffer drain conusmed " << numConsumed << " frames";
        }
        if( !queue.push(values) ) {
            LOG_WARNING << "Unable to add frames to queue?";
        }
    }

    delete readBuffer;
    return NULL;
}

void AlsaReader::stop(bool block) {
    doStop = true;
    if (block) {
        pthread_join(thread, NULL);
    }
}


/**
 * Alsa readers cannot be reset
 */
void AlsaReader::reset() {
    AlsaReader::getInstance(alsaDevice_)->stop(true);
}


/**
 * Non-blocking thread-safe read for getting a frame from the buffer
 */
bool AlsaReader::getFrames(vector<float>* &result) {
    return queue.pop(result);
}

vector<float>* AlsaReader::getFrames(unsigned frameCount, bool &forceStop) {
    vector<float>* result;
    while ( !queue.pop(result) ) {
        LOG_DEBUG << "Unable to get frames: queue empty";
        usleep(1);
    }
    return result;
}

AudioFormat* AlsaReader::getFormat() {
    AudioFormat* format = new AudioFormat;
    format->sampleRate = STREAM_SAMPLE_RATE;
    format->bitsPerSample = STREAM_BITS_PER_SAMPLE;
    format->channels = STREAM_CHANNELS;
    return format;
}
