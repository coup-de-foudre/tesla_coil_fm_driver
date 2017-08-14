#ifndef FM_TRANSMITTER_NOISE_GATE_H
#define FM_TRANSMITTER_NOISE_GATE_H

#include <cmath>
#include <vector>
#include <algorithm>


namespace noisegate {

  /**
   * Power estimator
   */
  class PowerEstimator {

  public:

    /**
     * Constructor
     * 
     * @param sampleRateHz the sample rate
     * @param halfLifeSeconds the half-life of the power estimate
     */
    PowerEstimator(float sampleRateHz, float halfLifeSeconds):
      state_(0.0) {
      float halfLifeSamples = halfLifeSeconds * sampleRateHz;
      float decayConstant = std::log(2) / halfLifeSamples;
      scaleFactor_ = std::exp(-decayConstant);
    }

    /**
     * Get the current RMS power estimate from the next sample
     */ 
    float apply(float input) {
      float result = scaleFactor_ * state_ + (1.0 - scaleFactor_) * (input * input); 
      state_ = result;
      return std::sqrt(result);
    }

  private:
    float scaleFactor_;
    float state_;
  };

  /**
   * Noise gate, specialized to a lightning machine
   */
  class NoiseGate {

  public:
    
    /**
     * @param sampleRateHz the sample rate Hz
     * @param attackSeconds half-life of the attack in seconds
     * @param decaySecond half-life of the decay decay in seconds
     * @param triggerDb trigger decibel level (should typically be <= 0.0)
     */
    NoiseGate(float sampleRateHz,
	      float attackSeconds,
	      float decaySeconds,
	      float triggerDb):
      powerEstimator_(sampleRateHz, std::min(attackSeconds, decaySeconds)) {
      
      triggerLevel_ = std::pow(10.0, triggerDb / 20.0);
      lastOutput_ = 0.0;

      float attackSamples = attackSeconds * sampleRateHz;
      attackFactor_ = std::exp( -std::log(2) / attackSamples);
      
      float decaySamples = decaySeconds * sampleRateHz;
      decayFactor_ = std::exp( -std::log(2) / decaySamples);
    }

    /**
     * Apply the noise gate to the buffers
     *
     * @param inputBuffer the input values
     * @param gateLevelBuffer the gate level output in range [0.0, 1.0]. Levels 
     *            near zero mean no gate, levels near one mean gate.
     */
    void apply(std::vector<float> &inputBuffer,
	       std::vector<float> &gateLevelBuffer) {
      gateLevelBuffer.resize(inputBuffer.size());

      for (unsigned i = 0; i < inputBuffer.size(); i++) {
	gateLevelBuffer[i] = this->apply(inputBuffer[i]);
      }
    }

    /**
     * Apply noise gate to the next frame
     */
    float apply(float input) {
      float powerLevel = powerEstimator_.apply(input);
      bool gateIsOff = powerLevel > triggerLevel_;
      float nextOutput;
      if (gateIsOff) {
	nextOutput = attackFactor_ * lastOutput_;
      } else {
	nextOutput = 1.0 - decayFactor_ * (1.0 - lastOutput_); 
      }
      lastOutput_ = nextOutput;
      return nextOutput;
    }
    
  private:

    float attackFactor_;
    float decayFactor_;
    float triggerLevel_;
    float lastOutput_;
    PowerEstimator powerEstimator_;
  
  };

} // namespace noisegate

#endif  // FM_TRANSMITTER_NOISE_GATE_H
