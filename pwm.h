/*

  Copyright (c) 2017-2018, Michael McCoy

  All rights reserved.

 */
#ifndef PWM_BCM2837_H
#define PWM_BCM2837_H

#include <plog/Log.h>
#include "peripherals.h"


namepsace pwm {

  enum class BCM_PIN : unsigned {
    PIN12 = 12,
    PIN13 = 13,
    PIN18 = 18,
    PIN19 = 19,
  };

  class PwmController {

  public:

    PwmController(double freqMHz,
		  double dutyCycle = 0.5,
		  PWM_CHANNEL channel = PWM_CHANNEL::CH1)  {

      // TODO: Mode choice
      // TODO: Lock per-pin for thread-safety?

      p = peripherals::Peripherals::getInstance();
      cmRegister = CM_CTL::PWM;
      mash = CM_MASH::MASH0;
      clockSource = CM_SRC::PLLD;
      clockFreqMHz = CM_FREQ::PLLD_MHZ;
      clockDivisor.divI = 2;  // Apparently a minumum of 2 is required
      clockDivisor.divF = 0;
      pwmChannel = channel;

      std::vector<PWM_CTL> pwmModes = {
	PWM_CTL::MSEN1,  // Enable M/S transmittion
	PWM_CTL::PWEN1,  // Enable channel 1 (which maps to PWM0?)
	PWM_CTL::MSEN2,  // Enable M/S transmittion
	PWM_CTL::PWEN2,  // Enable channel 2 (which maps to PWM1?)
      };

      LOG_DEBUG << "Setting up PWM with parameters";
      LOG_DEBUG << "cmRegister: " <<  HEX_STREAM(cmRegister);
      LOG_DEBUG << "clockSource: " << HEX_STREAM(clockSource);
      LOG_DEBUG << "clockDivisor: " << clockDivisor.divI << "(I) " << clockDivisor.divF << "(F)";
      LOG_DEBUG << "mash: " << HEX_STREAM(mash);
      LOG_DEBUG << "pwmChannel: " << HEX_STREAM(pwmChannel);

      if (pwmChannel == PWM_CHANNEL::CH1) {
	p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);
      } else {
	p.gpioFunctionSelect(FSEL::FSEL13, FSEL_MODE::ALT0);
      }

      p.pwmCtlSet(pwmModes);
      setTargetFreqMHz(freqMHz);
      p.clockInit(cmRegister, clockDivisor, mash, clockSource);
    }

    ~PwmController() {
      LOG_DEBUG << "Shutting down";
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
    void setTargetFreqMHz(double freqMHz) {
      targetFreqMHz = freqMHz;

      unsigned pwmPeriod = 2; // This is the period S
      unsigned pwmDutyCycle = 1; // This is duty M

      double baseFrequency = (clockDivisor.divI + clockDivisor.divF/(4096.0));

      pwmPeriod = (unsigned) max(2.0, freqMHz / baseFrequency);
      p.pwmRangeSet(pwmChannel, pwmPeriod);

      // Upon changing frequency, need to coordinate the duty cycle
      setTargetDutyCycle(targetDutyCycle);
    }

    /**
     * Get the current target frequency
     *
     * Note: Rounding may result in a different frequency given by the
     * hardware. Use `getHardwareFreqMHz` to get the frequency the
     * hardware is currently set to.
     */
    double getTargetFreqMHz() {
      return targetFreqMHz;
    }

    /**
     * Get the current hardware frequency
     */
    double getHardwareFreqMHz();

    /**
     * Set the duty cycle (between 0.0 and 1.0)
     *
     * Note: Rounding may result in a different duty cycle set by the
     * hardware. Use `getHardwareDutyCycle` to get the duty cycle
     * directly from the hardware.
     */
    void setTargetDutyCycle(double dutyCycle) {
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
    double getTargetDutyCycle() {
      return targetDutyCycle;
    }

    /**
     * Get the current hardware duty cycle
     */
    double getHardwareDutyCycle();

    /**
     * Shut down the PWM
     */
    void shutdown() {
      p.clockShutdown(cmRegister);
    }

  private:

    peripherals::Peripherals& p;
    double targetFreqMHz;
    double targetDutyCycle;

    CM_CTL cmRegister;
    CM_MASH mash;
    PWM_CHANNEL channel;
    CM_SRC clockSource;
    float clockFreqMHz;
    CLOCK_DIV clockDivisor;
    FSEL bcmPin;

    unsigned pwmPeriod;
    unsigned pwmDutyCycle;

    std::vector<PWM_CTL> pwmModes = {
      PWM_CTL::MSEN1,  // Enable M/S transmittion
      PWM_CTL::PWEN1,  // Enable channel 1 (which maps to PWM0?)
      PWM_CTL::MSEN2,  // Enable M/S transmittion
      PWM_CTL::PWEN2,   // Enable channel 2 (which maps to PWM1?)
    };

  };

} // namespace pwm


#endif // PWM_BCM2837_H
