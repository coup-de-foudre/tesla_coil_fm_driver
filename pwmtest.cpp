/*
  Copyright (c) 2018, Michael McCoy
*/
#include <csignal>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "peripherals.h"

#define HEX_STREAM(X) "0x" << std::setfill('0') << std::setw(8) << std::hex << (unsigned)X


using namespace peripherals;

void pwmTest(Peripherals& p) {

  CM_CTL cmRegister = CM_CTL::PWMCTL;
  CM_MASH mash = CM_MASH::MASH0;

  // Set the duty cycle to 50% at 100.2MHz
  CM_SRC clockSource = CM_SRC::PLLD;
  float clockFreqMHz = CM_FREQ::PLLD_MHZ;
  CLOCK_DIV clockDivisor;
  clockDivisor.divI = 3;
  clockDivisor.divF = 0;
  unsigned pwmPeriod = 2; // This is the period S
  unsigned pwmDutyCycle = 1; // This is duty M

  float outputFreqMHz = clockFreqMHz / (clockDivisor.divI + clockDivisor.divF/(4096.0)) / pwmPeriod;

  std::vector<PWM_CTL> pwmModes = {
    PWM_CTL::MSEN1,  // Enable M/S transmittion
    PWM_CTL::PWEN1,  // Enable channel 1 (which maps to PWM0?)
    PWM_CTL::MSEN2,  // Enable M/S transmittion
    PWM_CTL::PWEN2,   // Enable channel 2 (which maps to PWM1?)
  };

  LOG_INFO << "Setting up PWM with parameters";
  LOG_INFO << "clockSource: " << HEX_STREAM(clockSource);
  LOG_INFO << "mash: " << HEX_STREAM(mash);
  LOG_INFO << "clockDivisor: " << clockDivisor.divI << "(I) " << clockDivisor.divF << "(F)";
  LOG_INFO << "pwmPeriod: " << pwmPeriod;
  LOG_INFO << "pwmDutyCycle: " << pwmDutyCycle;
  LOG_INFO << "Freq (MHz): " << outputFreqMHz;

  p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);
  p.clockInit(cmRegister, clockDivisor, mash, clockSource); // TODO: enum class
  p.pwmCtlSet(pwmModes);
  p.pwmRangeSet(PWM_CHANNEL::CH1, pwmPeriod);
  p.pwmDataSet(PWM_CHANNEL::CH1, pwmDutyCycle);

  //TODO: usleep 1e6, then turn off GPIO and quit
  LOG_DEBUG << "Running for 3 seconds";
  sleep(3);

  LOG_DEBUG << "Shutting down";
  p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::INPUT);
}

int main(int argc, char** argv) {
  Peripherals& peripherals = Peripherals::getInstance();

  // PLog documentation at https://github.com/SergiusTheBest/plog
  static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
  plog::init(plog::debug, &consoleAppender);

  LOG_DEBUG << "Starting test";
  pwmTest(peripherals);
  LOG_DEBUG << "Test finished";
}
