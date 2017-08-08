#ifndef FM_TRANSMITTER_NOISE_GATE_H
#define FM_TRANSMITTER_NOISE_GATE_H

/**
 * Special noise gate
 */
class NoiseGate {

 public:

  NoiseGate(float attackUs, float decayUs, float triggerDb, float suppressionFactor);

  NoiseGate(const NoiseGate &noiseGate);
  
 private:

  /**
   * Attack microseconds
   */
  float attackUs;

  /**
   * Decay microseconds
   */
  float decayUs;

  /**
   * Decibles to trigger "on"
   */
  float triggerDb;

  /**
   * Fraction to suppress
   */
  float suppressionFactor;
    
  
}


NoiseGate::NoiseGate(float attackUs,
		     float decayUs,
		     float triggerDb,
		     float suppressionFactor):
  attackUs(attackUs),
  decayUs(decayUs),
  triggerDb(triggerDb),
  suppressionFactor(suppressionFactor) {
    
}

NoiseGate::NoiseGate(const NoiseGate &noiseGate) {
  return 
    }


#endif  // FM_TRANSMITTER_NOISE_GATE_H
