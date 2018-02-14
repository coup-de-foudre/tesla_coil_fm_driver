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
 * GPIO Function Select Registers
 */

// Function select registers
enum class GPFSEL_REGISTER : unsigned {
  GPFSEL0 = 0x00200000,
  GPFSEL1 = 0x00200004,
  GPFSEL2 = 0x00200008,
  GPFSEL3 = 0x0020000C,
  GPFSEL4 = 0x00200010,
  GPFSEL5 = 0x00200014
};

//
enum class FSEL : unsigned {
  FSEL00, FSEL01, FSEL02, FSEL03, FSEL04, FSEL05, FSEL06, FSEL07, FSEL08, FSEL09,
  FSEL10, FSEL11, FSEL12, FSEL13, FSEL14, FSEL15, FSEL16, FSEL17, FSEL18, FSEL19,
  FSEL20, FSEL21, FSEL22, FSEL23, FSEL24, FSEL25, FSEL26, FSEL27, FSEL28, FSEL29,
  FSEL30, FSEL31, FSEL32, FSEL33, FSEL34, FSEL35, FSEL36, FSEL37, FSEL38, FSEL39,
  FSEL40, FSEL41, FSEL42, FSEL43, FSEL44, FSEL45, FSEL46, FSEL47, FSEL48, FSEL49,
  FSEL50, FSEL51, FSEL52, FSEL53
};

enum class FSEL_MODE : unsigned {
  INPUT = 0x000,
  OUTPUT = 0x001,
  ALT0 = 0x100,
  ALT1 = 0x101,
  ALT2 = 0x110,
  ALT3 = 0x111,
  ALT4 = 0x011,
  ALT5 = 0x010,
  MASK = 0x111
};

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

// Clock-manager password (required to write to CM_*CTL and CM_*DIV)
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
 * PWM: Pulse-width modulation control and status registers
 */
enum class PWM_REGISTER : unsigned {
  CTL =  0x0020C000,  // Control
  STA =  0x0020C004,  // Status
  DMAC = 0x0020C008,  // DMA configuration
  RNG1 = 0x0020C010,  // Channel 1 range
  DAT1 = 0x0020C014,  // Channel 1 data
  FIF1 = 0x0020C018,  // FIFO input
  RNG2 = 0x0020C020,  // Channel 2 range
  DAT2 = 0x0020C024   // Channel 2 input
};

/*
PWENi is used to enable/disable the corresponding channel. Setting this bit to 1
enables the channel and transmitter state machine. All registers and FIFO is writable
without setting this bit.
MODEi bit is used to determine mode of operation. Setting this bit to 0 enables PWM
mode. In this mode data stored in either PWM_DATi or FIFO is transmitted by pulse
width modulation within the range defined by PWM_RNGi. When this mode is used
MSENi defines whether to use PWM algorithm. Setting MODEi to 1 enables serial
mode, in which data stored in either PWM_DATi or FIFO is transmitted serially within
the range defined by PWM_RNGi. Data is transmitted MSB first and truncated or zeropadded
depending on PWM_RNGi. Default mode is PWM.
RPTLi is used to enable/disable repeating of the last data available in the FIFO just
before it empties. When this bit is 1 and FIFO is used, the last available data in the
FIFO is repeatedly sent. This may be useful in PWM mode to avoid duty cycle gaps. If
the FIFO is not used this bit does not have any effect. Default operation is do-notrepeat.

SBITi defines the state of the output when no transmission takes place. It also defines
the zero polarity for the zero padding in serialiser mode. This bit is padded between
two consecutive transfers as well as tail of the data when PWM_RNGi is larger than bit
depth of data being transferred. this bit is zero by default.
POLAi is used to configure the polarity of the output bit. When set to high the final
output is inverted. Default operation is no inversion.
USEFi bit is used to enable/disable FIFO transfer. When this bit is high data stored in
the FIFO is used for transmission. When it is low, data written to PWM_DATi is
transferred. This bit is 0 as default.
CLRF is used to clear the FIFO. Writing a 1 to this bit clears the FIFO. Writing 0 has no
effect. This is a single shot operation and reading the bit always returns 0.
MSENi is used to determine whether to use PWM algorithm or simple M/S ratio
transmission. When this bit is high M/S transmission is used. This bit is zero as default.
When MODEi is 1, this configuration bit has no effect.
 */
enum class PWM_CTL : unsigned {
  MSEN2 = 1 << 15,  // Channel 2 M/S enable
  USEF2 = 1 << 13,  // Channel 2 use FIFO
  POLA2 = 1 << 12,  // Channel 2 polarity
  SBIT2 = 1 << 11,  // Channel 2 silence bit
  RPTL2 = 1 << 10,  // Channel 2 repeat last data
  MODE2 = 1 << 9,   // Channel 2 mode
  PWEN2 = 1 << 8,   // Channel 2 enable
  MSEN1 = 1 << 7,   // Channel 1 M/S enable
  CLRF1 = 1 << 6,   // Clear FIFO
  USEF1 = 1 << 5,   // Channel 1 use FIFO
  POLA1 = 1 << 4,   // Channel 1 polarity
  SBIT1 = 1 << 3,   // Chanel 1 silence bit
  RPTL1 = 1 << 2,   // Channel 1 repeat last data
  MODE1 = 1 << 1,   // Channel 1 mode
  PWEN1 = 1 << 0    // Channel 1 enable
};

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
     * @returns The base address of the peripherals,
     *          or NULL if unable to access the peripherals
     */
    void* getPeripheralsBase() {
      return peripheralsBase_;
    }

    /**
     * Read microseconds from the SystemTimer register
     */
    inline unsigned long long systemTimerMicroseconds() {
      return ACCESS64(peripheralsBase_, ST_CLO);
    }

    /**
     * Clock shutdown sequence
     *
     * @returns The clock manager state after shutdown
     */
    inline unsigned clockShutdown(unsigned cmRegister) {

      // Disable clock
      volatile unsigned cmState = ACCESS(peripheralsBase_, cmRegister);
      ACCESS(peripheralsBase_, cmRegister) =
	CM_PASSWD | (0x00FFFFFF & cmState & !CM_ENAB);

      // Wait for clock to become available
      do {
	cmState = ACCESS(peripheralsBase_, cmRegister);
      }	while (cmState & CM_BUSY);

      return cmState;
    }

    /**
     * Set the clock divisor
     *
     * To avoid glitches, call clockShutdown() before calling this method.
     *
     * @param clockDivisor A 24-bit clock divisor (12-bit integral part,
     * 12-bit fractional part).
     */
    inline void clockDivisorSet(unsigned cmRegister, unsigned clockDivisor) {
      unsigned divRegister = cmRegister + 4;
      ACCESS(peripheralsBase_, divRegister) =
	CM_PASSWD | (0x00FFFFFF & clockDivisor);
    }

    /**
     * Clock initialization sequence
     *
     * @param cmRegister The control register for the clock (CM_*CTL)
     * @param clockDivisor A 24-bit clock divisor (12-bit integral part,
     *        12-bit fractional part).
     * @param mash The MASH filter setting (CM_MASH*)
     * @param clockSource The source of the clock (CM_SRC_*)
     */
    inline void clockInit(unsigned cmRegister,
			  unsigned clockDivisor,
			  unsigned mash,
			  unsigned clockSource) {

      clockShutdown(cmRegister);
      clockDivisorSet(cmRegister, clockDivisor);

      unsigned clockConfig =
	CM_PASSWD | (0x00FFFFFF & (mash | CM_ENAB | clockSource));

      ACCESS(peripheralsBase_, cmRegister) = clockConfig;
    }


    /**
     * Set GPIO function
     */
    inline void gpioFunctionSelect(FSEL fsel, FSEL_MODE mode) {

      unsigned offset =
	(unsigned) GPFSEL_REGISTER::GPFSEL0 + 4 * ((unsigned) fsel / 10);
      unsigned shift = 3 * ((unsigned) fsel % 10);

      unsigned initialState = ACCESS(peripheralsBase_, offset);
      unsigned goalState =
	((unsigned) mode << shift) | (initialState & !(unsigned) FSEL_MODE::MASK);

      ACCESS(peripheralsBase_, offset) = goalState;
    }


    /**
     *
     */
    void pwmInit() {
      // TODO: Lock?
      // TODO: choose one (PWM0, PIN12), (PWM1, PIN13), (PWM0, PIN18), (PWM1, PIN19), (PWM0, PIN40), (PWM1, PIN45)
      // TODO: Choose clock (OSC, PLLA, PLLC, PLLD)
      // TODO: Choose clock divisor, set mash filter, etc. ?

      unsigned cmRegister = CM_PWMCTL;
      unsigned mash = CM_MASH0;
      unsigned clockSource = CM_SRC_PLLD;
      unsigned clockDivisor = 1;

      clockInit(cmRegister, clockDivisor, mash, clockSource);
      gpioFunctionSelect(FSEL::FSEL12, FSEL_MODE::ALT0);

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
