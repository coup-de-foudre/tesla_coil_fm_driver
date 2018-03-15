/*

  Copyright (c) 2017-2018, Michael McCoy

  All rights reserved.

  // TODO: Apply license
 */
#ifndef PWM_BCM2837_H
#define PWM_BCM2837_H

#include "peripherals.h"

#include <algorithm>

#include "plog/Log.h"


namespace pwm {
  using namespace peripherals;

  class PwmController {

  public:

    PwmController(double freqMHz,
                  double dutyCycle = 0.5,
                  PWM_CHANNEL channel = PWM_CHANNEL::CH1): p(Peripherals::getInstance())  {

      // TODO: Mode choice
      // TODO: Lock per-pin for thread-safety?

      cmRegister = CM_CTL::PWM;

      // Fractional part of clock divisor ignored with MASH0
      mash = CM_MASH::MASH0;

      clockSource = CM_SRC::PLLD;
      clockFreqMHz = CM_FREQ::PLLD_MHZ;

      // Set clock divisor to give about 2048 different PWM "clock periods" (S)
      // This is defensive: some online references suggest that S is a 12-bit
      // number, despite being a 32-bit register.

      // TODO: Check that this works with a scope.
      clockDivisor.divI = (unsigned)std::max(2.0, clockFreqMHz / freqMHz / 2048.0) ;
      clockDivisor.divF = 0;
      pwmChannel = channel;
      baseFreqMHz = clockFreqMHz / (clockDivisor.divI + clockDivisor.divF/(4096.0));

      std::vector<PWM_CTL> pwmModes = {
        PWM_CTL::MSEN1,  // Enable M/S transmittion
        PWM_CTL::PWEN1,  // Enable channel 1 (which maps to PWM0?)
        PWM_CTL::MSEN2,  // Enable M/S transmittion
        PWM_CTL::PWEN2,  // Enable channel 2 (which maps to PWM1?)
      };

      // PATCH: Enable more PWM pins
      p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);
      p.gpioFunctionSelect(FSEL::FSEL13, FSEL_MODE::ALT0);
      p.gpioFunctionSelect(FSEL::FSEL18, FSEL_MODE::ALT5);
      p.gpioFunctionSelect(FSEL::FSEL19, FSEL_MODE::ALT5);

      /*
      if (pwmChannel == PWM_CHANNEL::CH1) {
        p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);
      } else {
        p.gpioFunctionSelect(FSEL::FSEL13, FSEL_MODE::ALT0);
      } 
      */

      LOG_DEBUG << "Setting up PWM with parameters";
      LOG_DEBUG << "cmRegister: " <<  HEX_STREAM(cmRegister);
      LOG_DEBUG << "clockSource: " << HEX_STREAM(clockSource);
      LOG_DEBUG << "clockDivisor: " << clockDivisor.divI << "(I) " << clockDivisor.divF << "(F)";
      LOG_DEBUG << "mash: " << HEX_STREAM(mash);
      LOG_DEBUG << "pwmChannel: " << HEX_STREAM(pwmChannel);
      LOG_DEBUG << "baseFreqMHz: " << baseFreqMHz;

      p.pwmCtlSet(pwmModes);
      setTargetFreqMHz(freqMHz);
      p.clockInit(cmRegister, clockDivisor, mash, clockSource);
    }

    ~PwmController() {
      LOG_DEBUG << "Shutting PWM down.";
      shutdown();
    }

    /**
     * Set the frequency to freqMHz
     *
     * Note: Rounding may result in a different frequency given by the
     * hardware. Use `getHardwareFreqMHz` to get the frequency the
     * hardware is currently set to.
     *
     */
    inline void setTargetFreqMHz(double freqMHz) {
      targetFreqMHz = freqMHz;
      pwmPeriod = (unsigned) std::max(2.0, std::round(baseFreqMHz / freqMHz));
      p.pwmRangeSet(pwmChannel, pwmPeriod);
      setTargetDutyCycle(targetDutyCycle);
    }

    /**
     * Get the current target frequency
     *
     * Note: Rounding may result in a different frequency given by the
     * hardware. Use `getHardwareFreqMHz` to get the frequency the
     * hardware is currently set to.
     */
    inline double getTargetFreqMHz() {
      return targetFreqMHz;
    }

    /**
     * Get the current hardware frequency
     */
    inline double getHardwareFreqMHz() {
      // TODO: Read from hardware register
      return baseFreqMHz / pwmPeriod;
    }

    /**
     * Set the duty cycle (between 0.0 and 1.0)
     *
     * Note: Rounding may result in a different duty cycle set by the
     * hardware. Use `getHardwareDutyCycle` to get the duty cycle
     * directly from the hardware.
     */
    inline void setTargetDutyCycle(double dutyCycle) {
      targetDutyCycle = dutyCycle;
      pwmDutyCycle = round(targetDutyCycle * pwmPeriod);
      p.pwmDataSet(pwmChannel, pwmDutyCycle);
    }

    /**
     * Set the duty cycle (between 0.0 and 1.0)
     *
     * Note: Rounding may result in a different duty cycle set by the
     * hardware. Use `getHardwareDutyCycle` to get the duty cycle
     * directly from the hardware.
     */
    inline double getTargetDutyCycle() {
      return targetDutyCycle;
    }

    /**
     * Get the current hardware duty cycle
     */
    inline double getHardwareDutyCycle() {
      // TODO: Read from hardware register
      return ((double)pwmDutyCycle) / pwmPeriod;
    }

    /**
     * Shut down the PWM
     */
    void shutdown() {
      p.clockShutdown(cmRegister);
    }

  private:

    Peripherals& p;
    double targetFreqMHz;
    double targetDutyCycle;
    PWM_CHANNEL pwmChannel;

    CM_CTL cmRegister;
    CM_MASH mash;
    CM_SRC clockSource;
    float clockFreqMHz;
    CLOCK_DIV clockDivisor;
    double baseFreqMHz;

    unsigned pwmPeriod;
    unsigned pwmDutyCycle;

    std::vector<PWM_CTL> pwmModes;
  };

} // namespace pwm


#endif // PWM_BCM2837_H
