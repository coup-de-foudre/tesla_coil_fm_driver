#ifndef FM_TRANSMITTER_PERIPHERALS_H
#define FM_TRANSMITTER_PERIPHERALS_H

/**
 *  Base (physical) addresses for peripherals
 */
#ifdef __BCM2835__
#define PERIPHERALS_BASE 0x20000000
#else
#define PERIPHERALS_BASE 0x3F000000
#endif

// Total size of the peripherals registers
#define PERIPHERALS_LENGTH 0x002FFFFF


/**
 * GPIO control registers
 */
// Function Select
#define GPFSEL0 0x00200000
#define GPFSEL1 0x00200004
#define GPFSEL2 0x00200008
#define GPFSEL3 0x0020000C
#define GPFSEL4 0x00200010
#define GPFSEL5 0x00200014

// Pin set
#define GPSET0 0x0020001C
#define GPSET1 0x00200020

// Pin clear
#define GPCLR0 0x00200028
#define GPCLR1 0x0020002C

// Pin level
#define GPLVL0 0x00200034
#define GPLVL1 0x00200038

// Pin event detect status
#define GPEDS0 0x00200040
#define GPEDS1 0x00200044

// Pin rising edge detect enable
#define GPREN0 0x0020004C
#define GPREN1 0x00200050

// Pin falling edge detect enable
#define GPFEN0 0x00200058
#define GPFEN1 0x0020005C

// Pin high detect enable
#define GPHEN0 0x00200064
#define GPHEN1 0x00200068

// Pin low detect enable
#define GPLEN0 0x00200070
#define GPLEN1 0x00200074

// Pin async rising edge detect enable
#define GPAREN0 0x0020007C
#define GPAREN1 0x00200080

// Pin async falling edge detect enable
#define GPAFEN0 0x00200088
#define GPAFEN1 0x0020008C

// Pin pull-up/down enable
#define GPPUD 0x00200094

// Pin pull-up/down enable clock
#define GPPUDCLK0 0x00200098
#define GPPUDCLK1 0x0020009C



/**
 * CM: General-purpose clock manager
 */
// Control registers
#define CM_GP0CTL 0x00101070
#define CM_GP1CTL 0x00101078
#define CM_GP2CTL 0x00101080

// Clock divisor registers
#define CM_GP0DIV 0x00101074
#define CM_GP1DIV 0x0010107c
#define CM_GP2DIV 0x00101084

// Clock-manger password (required to write to *CTL and *DIV)
#define CM_PASSWD 0x5A000000

// Clock MASH filters
#define CM_MASH1 (0x01 << 9)
#define CM_MASH2 (0x02 << 9)
#define CM_MASH3 (0x03 << 9)

// Clock output flip (test/debug only)
#define CM_FLIP (0x01 << 8)

// Clock manageer busy
#define CM_BUSY (0x01 << 7)

// Clock generator kill (test/debug only)
#define CM_KILL (0x01 << 5)

// Clock manager enable
#define CM_ENAB (0x01 << 4)

// Clock manager clock set
#define CM_SRC_GND (0x00)
#define CM_SRC_OSC (0x01)
#define CM_SRC_TST0 (0x02)  // test/debug only
#define CM_SRC_TST1 (0x01)  // test/debug only
#define CM_SRC_PLLA (0x04)
#define CM_SRC_PLLC (0x05)
#define CM_SRC_PLLD (0x06)
#define CM_SRC_HDMI (0x07)

// Clock frequencies
#define PLLA_FREQ_MHZ 650.0
#define PLLC_FREQ_MHZ 200.0
#define PLLD_FREQ_MHZ 500.0


/**
 * ST: System timer provides a 64-bit system counter
 */
// System Timer Control/Status
#define ST_CS 0x00003000

// Least-significant 32 bits
#define ST_CLO 0x00003004

// Most-significant 32 bits
#define ST_CHI 0x00003008


#define ACCESS(base, offset) *(volatile unsigned*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)


#endif //FM_TRANSMITTER_PERIPHERALS_H
