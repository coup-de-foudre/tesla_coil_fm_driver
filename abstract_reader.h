#ifndef ABSTRACT_READER_H
#define ABSTRACT_READER_H
#include <vector>
#include "audio_format.h"

/**
 * Abstract PCM audio reader
 */
class AbstractReader {
 public:

  virtual ~AbstractReader() {};

  /**
   * Get a frame buffer from the input (non-blocking)
   */
  virtual bool getFrames(std::vector<float>* &result) = 0;

  /**
   * Signal a stop to the reader
   */
  virtual void stop(bool block) = 0;

  /**
   * Get the format of the input
   */
  virtual AudioFormat* getFormat() = 0;

  /**
   * Are we at the end of the stream?
   */
  virtual bool isEnd() {
    return false;
  }
};

#endif // ABSTRACT_READER_H
