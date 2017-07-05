//
// Created by Michael B. McCoy on 7/4/17.
//

#ifndef FM_TRANSMITTER_PERIPHERALS_H
#define FM_TRANSMITTER_PERIPHERALS_H

// Let's define PERIPHERAL_BASE 0x3F000000
#define BCM2835_PERIPHERAL_BASE  0x20000000
#define PERIPHERAL_BASE 0x3F000000

// GPIO Function Select 0 GPFSEL0
// GPFSEL0 = PERIPHERAL_BASE + GPIO_BASE
#define GPIO_BASE 0x00200000

// Clock Manager General Purpose Clocks Control CM_GP0CTL
#define CLK0_BASE 0x00101070
#define CLK1_BASE 0x00101078
#define CLK2_BASE 0x00101080

/// Clock Manager General Purpose Clock Divisors CM_GP0DIV
#define CLK0DIV_BASE 0x00101074
#define CLK1DIV_BASE 0x0010107c
#define CLK2DIV_BASE 0x00101084

// The ST system timer which provides a 64-bit system counter
//   CLO 32-bit lower part of the counter at register 0x7E003004 (virtual)
//   CHI 32-bit upper part of the counter at register 0x7E003008 (virtual)
#define TCNT_BASE 0x00003004

// The password for many of the peripherals
#define PASSWORD 0x5A000000

#define ACCESS(base, offset) *(volatile unsigned*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

#endif //FM_TRANSMITTER_PERIPHERALS_H
