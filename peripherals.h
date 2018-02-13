/*

  Copyright (c) 2017-2018, Michael McCoy

  All rights reserved.

 */

#ifndef BCM2837_PERIPHERALS_H
#define BCM2837_PERIPHERALS_H

#include <exception>

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
#define CM_GP0CTL  0x00101070
#define CM_GP1CTL  0x00101078
#define CM_GP2CTL  0x00101080
#define CM_PCMCTL  0x00101098
#define CM_PWMCTL  0x001010a0
#define CM_UARTCTL 0x001010f0

// Clock divisor registers
#define CM_GP0DIV  0x00101074
#define CM_GP1DIV  0x0010107c
#define CM_GP2DIV  0x00101084
#define CM_PCMDIV  0x0010109c
#define CM_PWMDIV  0x001010a4
#define CM_UARTDIV 0x001010f4

// Clock-manger password (required to write to *CTL and *DIV)
#define CM_PASSWD 0x5A000000

// Clock MASH filters
#define CM_MASH0 (0x00 << 9)
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
#define OSC_FREQ_MHZ  19.2
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


#define ACCESS(base, offset) *(volatile unsigned*)((unsigned)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((unsigned)base + offset)


namespace peripherals {

  struct MemoryMapPeripheralsException : public std::exception {

    const char* what() const throw () {
      return "Unable to memory map peripherals";
    }

  };

  /**
   * Low-level access to BCM2837 SoC peripherals
   */
  class Peripherals {

    // Uses singleton pattern
    // https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

    // TODO: Thread-safe access?

  public:

    /**
     * Get instance of the peripherals class
     */
    static Peripherals& getInstance() {
      static Peripherals instance;
      return instance;
    }

    /**
     * Get the base address of the peripherals
     *
     * @return: The base address of the peripherals,
     *          or NULL if unable to access the peripherals
     */
    void* getPeripheralsBase() {
      return peripheralsBase_;
    }

    /**
     * Read the microseconds from the SystemTimer register
     */
    inline unsigned long long systemTimerMicroseconds() {
      return ACCESS64(peripheralsBase_, ST_CLO);
    }

    /**
     *
     */
    // TODO: use enum classes to set these
    inline void clockInit(unsigned cmRegister,
			  unsigned clockDivisor,
			  unsigned mash,
			  unsigned clockSource) {

      unsigned divRegister = cmRegister + 4;
      
      // Disable clock
      volatile unsigned cmState = ACCESS(peripheralsBase_, cmRegister);
      ACCESS(peripheralsBase_, cmRegister) = (cmState & !CM_ENAB) | CM_PASSWD;

      // Wait for clock to become available
      do {
	cmState = ACCESS(peripheralsBase_, cmRegister);
      }	while (cmState & CM_BUSY);

      // Set clock divisor
      ACCESS(peripheralsBase_, divRegister) = CM_PASSWD | (0x00FFFFFF & clockDivisor);

      // Configure
      unsigned clockConfig = CM_PASSWD | mash | CM_ENAB | clockSource;
      ACCESS(peripheralsBase_, cmRegister) = clockConfig;
    }

    /**
     * 
     */
    void pwmInit() {
      // TODO: Lock?
      // TODO: choose one (PWM0, PIN12), (PWM1, PIN13), (PWM0, PIN18), (PWM1, PIN19), (PWM0, PIN40), (PWM1, PIN45)
      // TODO: Choose clock (OSC, PLLA, PLLC, PLLD)
      // TODO: Choose clock divisor, set mash filter, etc. ? 

      clockInit(CM_PWMCTL, 1, CM_MASH0, CM_SRC_PLLD);

      
      // 2. Choose the pin function
      // (PWM0, PIN12) => GPFSEL1, set bits 8-6 to 0x100 (ALT function zero)
      unsigned fselBase = GPFSEL1;
      unsigned fselBits = 0x7 << 5;
      unsigned fsel = 0x100 << 5;

      unsigned initialState = ACCESS(peripheralsBase_, fselBase);
      unsigned goalState = (initialState & (!fselBits)) | fsel;

      ACCESS(peripheralsBase_, fselBase) = goalState;
    }
    
    // Deleting these helps ensure singletons
    Peripherals(Peripherals const&) = delete;
    void operator=(Peripherals const&) = delete;
    
  private:

    /**
     * Memory-map the peripherals on contsruction
     */
    Peripherals() {
      peripheralsBase_ = mmapPeripherals();
    }

    /**
     * Unmap peripherals on destruction
     *
     * This is completely unnecessary for singletons, but this
     * is the necessary "cleanup" in case someone refactors the
     * code.
     */
    ~Peripherals() {
      munmap((void*)peripheralsBase_, PERIPHERALS_LENGTH);
      peripheralsBase_ = (void*) NULL;
    }

    /**
     * Memory-map the peripherals addresses
     */
    static void* mmapPeripherals() {
      int memFd;
      if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
	return (void*) NULL;
      }

      void* mappedBase = mmap(NULL,
			      PERIPHERALS_LENGTH,
			      PROT_READ | PROT_WRITE,

			      MAP_SHARED,
			      memFd,
			      PERIPHERALS_BASE);
      close(memFd);

      if (mappedBase == MAP_FAILED) {
	throw new MemoryMapPeripheralsException();
      }

      return mappedBase;
    }

    void* peripheralsBase_;

  };

} // namespace peripherals

#endif // BCM2837_PERIPHERALS_H
