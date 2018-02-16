/*
  Copyright (c) 2018, Michael McCoy
*/
#include <csignal>
#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "peripherals.h"

using namespace peripherals;

void pwmTest(Peripherals& p) {
  
  CM_CTL cmRegister = CM_CTL::PWMCTL;
  CM_MASH mash = CM_MASH::MASH0;
  
  // Set the duty cycle to 50% at 100MHz
  CM_SRC clockSource = CM_SRC::PLLD;
  unsigned clockDivisor = 1;
  unsigned pwmPeriod = 500; // This is the period S
  unsigned pwmDutyCycle = 250; // This is duty M
  
  std::vector<PWM_CTL> pwmModes = {
    PWM_CTL::MSEN1,  // Enable M/S transmittion
    PWM_CTL::PWEN1   // Enable channel 1 (which maps to PWM0?)
  };


  LOG_DEBUG << "Setting up PWM";

  p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::INPUT);
  p.clockInit(cmRegister, clockDivisor, mash, clockSource); // TODO: enum class
  p.pwmCtlSet(pwmModes);
  p.pwmRangeSet(PWM_CHANNEL::CH1, pwmPeriod);
  p.pwmDataSet(PWM_CHANNEL::CH1, pwmDutyCycle);
  p.gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);
  
  //TODO: usleep 1e6, then turn off GPIO and quit
  LOG_DEBUG << "Running for 1 second";
  sleep(1);

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
