/*
  
  Copyright (c) 2017-2018, Michael McCoy

  All rights reserved. 

 */
#ifndef PWM_BCM2837_H
#define PWM_BCM2837_H

#include <plog/Log.h>
#include "peripherals.h"

namepsace pwm {

  void pwmInit() {
    // TODO: Mode choice
    // TODO: Lock for thread-safety?

    LOG_DEBUG << "Initializing PWM";

    peripherals::Peripherals& p = peripherals::Peripherals::getInstance();

  }
}


#endif // PWM_BCM2837_H
