/*

  Copyright (c) 2017-2018, Michael McCoy

  All rights reserved.

 */

#ifndef BCM2837_PERIPHERALS_H
#define BCM2837_PERIPHERALS_H

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <exception>
#include <vector>

#include <plog/Log.h>
#define HEX_STREAM(X) "0x" << std::setfill('0') << std::setw(8) << std::hex << (unsigned)X


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

enum class GP_REGISTER : unsigned {
  // Function select registers
  GPFSEL0 = 0x00200000,
  GPFSEL1 = 0x00200004,
  GPFSEL2 = 0x00200008,
  GPFSEL3 = 0x0020000C,
  GPFSEL4 = 0x00200010,
  GPFSEL5 = 0x00200014,

  // Pin set
  GPSET0 = 0x0020001C,
  GPSET1 = 0x00200020,

  // Pin clear
  GPCLR0 = 0x00200028,
  GPCLR1 = 0x0020002C,

  // Pin level
  GPLVL0 = 0x00200034,
  GPLVL1 = 0x00200038,

  // Pin event detect status
  GPEDS0 = 0x00200040,
  GPEDS1 = 0x00200044,

  // Pin rising edge detect enable
  GPREN0 = 0x0020004C,
  GPREN1 = 0x00200050,

  // Pin falling edge detect enable
  GPFEN0 = 0x00200058,
  GPFEN1 = 0x0020005C,

  // Pin high detect enable
  GPHEN0 = 0x00200064,
  GPHEN1 = 0x00200068,

  // Pin low detect enable
  GPLEN0 = 0x00200070,
  GPLEN1 = 0x00200074,

  // Pin async rising edge detect enable
  GPAREN0 = 0x0020007C,
  GPAREN1 = 0x00200080,

  // Pin async falling edge detect enable
  GPAFEN0 = 0x00200088,
  GPAFEN1 = 0x0020008C,

  // Pin pull-up/down enable
  GPPUD = 0x00200094,

  // Pin pull-up/down enable clock
  GPPUDCLK0 = 0x00200098,
  GPPUDCLK1 = 0x0020009C
};

/**
 * Function select options
 */
enum class FSEL : unsigned {
  FSEL00, FSEL01, FSEL02, FSEL03, FSEL04, FSEL05, FSEL06, FSEL07, FSEL08, FSEL09,
  FSEL10, FSEL11, FSEL12, FSEL13, FSEL14, FSEL15, FSEL16, FSEL17, FSEL18, FSEL19,
  FSEL20, FSEL21, FSEL22, FSEL23, FSEL24, FSEL25, FSEL26, FSEL27, FSEL28, FSEL29,
  FSEL30, FSEL31, FSEL32, FSEL33, FSEL34, FSEL35, FSEL36, FSEL37, FSEL38, FSEL39,
  FSEL40, FSEL41, FSEL42, FSEL43, FSEL44, FSEL45, FSEL46, FSEL47, FSEL48, FSEL49,
  FSEL50, FSEL51, FSEL52, FSEL53
};

/**
 * Function select modes
 */
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

/**
 * CM: General-purpose clock manager
 */
enum class CM_CTL : unsigned {
  GP0  = 0x00101070,
  GP1  = 0x00101078,
  GP2  = 0x00101080,
  PCM  = 0x00101098,
  PWM  = 0x001010a0,
  UART = 0x001010f0
};

enum class CM_DIV : unsigned {
  GP0DIV  = 0x00101074,
  GP1DIV  = 0x0010107c,
  GP2DIV  = 0x00101084,
  PCMDIV  = 0x0010109c,
  PWMDIV  = 0x001010a4,
  UARTDIV = 0x001010f4
};

enum class CM_MODE : unsigned {
  // Clock-manager password (required to write to CM_*CTL and CM_*DIV)
  PASSWD = 0x5A000000,

  // Clock output flip (test/debug only)
  FLIP = (0x01 << 8),

  // Clock manager busy
  BUSY = (0x01 << 7),

  // Clock generator kill (test/debug only)
  KILL = (0x01 << 5),

  // Clock manager enable
  ENAB = (0x01 << 4),
};

enum class CM_MASH : unsigned {
  // Clock MASH filters
  MASH0 = (0x00 << 9),
  MASH1 = (0x01 << 9),
  MASH2 = (0x02 << 9),
  MASH3 = (0x03 << 9)
};

enum class CM_SRC : unsigned {
  // Clock manager clock set
  GND  = (0x00),
  OSC  = (0x01),
  TST0 = (0x02),  // test/debug only
  TST1 = (0x01),  // test/debug only
  PLLA = (0x04),
  PLLC = (0x05),
  PLLD = (0x06),
  HDMI = (0x07)
};

// Clock frequencies
namespace CM_FREQ {
  constexpr float OSC_MHZ  = 19.2;
  constexpr float PLLA_MHZ = 650.0;
  constexpr float PLLC_MHZ = 200.0;
  constexpr float PLLD_MHZ = 500.0;
};


/**
 * PWM: Pulse-width modulation control and status registers
 */
enum class PWM_REGISTER : unsigned {
  CTL  = 0x0020C000,  // Control
  STA  = 0x0020C004,  // Status
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

enum class PWM_CHANNEL : unsigned {
  CH1 = 1,
  CH2 = 2
};

/**
 * ST: System timer provides a 64-bit system counter
 */
enum class ST_REGISTER : unsigned {
  // System Timer Control/Status
  CS = 0x00003000,

  // Least-significant 32 bits
  CLO = 0x00003004,

  // Most-significant 32 bits
  CHI = 0x00003008
};

#define ACCESS(base, offset) *(volatile unsigned*)((unsigned)base + (unsigned)offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((unsigned)base + (unsigned)offset)


namespace peripherals {

  /**
   * Clock divisor (12-bit integral, 12-bit fractional part)
   */
  struct CLOCK_DIV {
    unsigned divI : 12;
    unsigned divF : 12;
  };

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
      return ACCESS64(peripheralsBase_, ST_REGISTER::CLO);
    }

    /**
     * Clock shutdown sequence
     *
     * @returns The clock manager state after shutdown
     */
    inline unsigned clockShutdown(CM_CTL ctlRegister) {

      // Disable clock
      volatile unsigned cmState = ACCESS(peripheralsBase_, ctlRegister);
      ACCESS(peripheralsBase_, ctlRegister) =
	cmPwd(setBits(cmState, ~(unsigned)CM_MODE::ENAB, (unsigned)CM_MODE::ENAB));

      // Wait for clock to become available
      do {
	cmState = ACCESS(peripheralsBase_, ctlRegister);
      }	while (cmState & (unsigned)CM_MODE::BUSY);

      return cmState;
    }

    /**
     * Set the clock divisor
     *
     * To avoid glitches, call clockShutdown() before calling this method.
     *
     * @param divRegister The clock divisor register
     * @param clockDivisor A 24-bit clock divisor (12-bit integral part,
     *        12-bit fractional part).
     */
    inline void clockDivisorSet(CM_DIV divRegister, CLOCK_DIV clockDiv) {
      unsigned clockDivisor = (clockDiv.divI << 12) | clockDiv.divF;
      ACCESS(peripheralsBase_, divRegister) = cmPwd(clockDivisor);
    }

    /**
     * Clock initialization sequence
     *
     * @param cmRegister The control register for the clock (CM_*CTL)
     * @param clockDivisor Clock divisor (12-bit integral part, 12-bit fractional)
     * @param mash The MASH filter setting (CM_MASH*)
     * @param clockSource The source of the clock (CM_SRC_*)
     */
    inline void clockInit(CM_CTL ctlRegister,
			  CLOCK_DIV clockDiv,
			  CM_MASH mash,
			  CM_SRC clockSource) {
      CM_DIV divRegister = (CM_DIV)((unsigned) ctlRegister + 4);
      clockShutdown(ctlRegister);
      clockDivisorSet(divRegister, clockDiv);

      unsigned clockConfig =
	cmPwd((unsigned)mash | (unsigned)CM_MODE::ENAB | (unsigned)clockSource);
      ACCESS(peripheralsBase_, ctlRegister) = clockConfig;
    }


    /**
     * Set GPIO function
     */
    inline void gpioFunctionSelect(FSEL fsel, FSEL_MODE mode) {

      unsigned offset =
	(unsigned) GP_REGISTER::GPFSEL0 + 4 * ((unsigned) fsel / 10);
      unsigned shift = 3 * ((unsigned) fsel % 10);

      unsigned initialState = ACCESS(peripheralsBase_, offset);
      unsigned goalState =
	setBits(initialState, (unsigned) mode << shift, (unsigned) FSEL_MODE::MASK);

      ACCESS(peripheralsBase_, offset) = goalState;
    }

    /**
     * Set PWM Control
     *
     * @param modes The modes to enable. The corresponding bits are set
     *        to one in the PWM_CTL register, others are set to zero.
     *
     */
    inline void pwmCtlSet(std::vector<PWM_CTL> modes) {
      unsigned offset = (unsigned) PWM_REGISTER::CTL;
      unsigned setValue = 0;

      for (PWM_CTL mode : modes) {
	setValue |= (unsigned) mode;
      }

      ACCESS(peripheralsBase_, offset) = setValue;
    }

    /**
     * Set PWM range
     */
    inline void pwmRangeSet(PWM_CHANNEL channel, unsigned range) {
      PWM_REGISTER offset = (channel == PWM_CHANNEL::CH1) ?
	PWM_REGISTER::RNG1 : PWM_REGISTER::RNG2;
      ACCESS(peripheralsBase_, offset) = range;
    }

    /**
     * Set PWM data
     */
    inline void pwmDataSet(PWM_CHANNEL channel, unsigned data) {
      PWM_REGISTER offset = (channel == PWM_CHANNEL::CH1) ?
	PWM_REGISTER::DAT1 : PWM_REGISTER::DAT2;
      ACCESS(peripheralsBase_, offset) = data;
    }


    inline void clearPwmStatus() {
      ACCESS(peripheralsBase_, PWM_REGISTER::STA) = 0x0000FFFF;
    }

    inline unsigned getPwmStatus() {
      return ACCESS(peripheralsBase_, PWM_REGISTER::STA);
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

    /**
     * Set bits to given value
     *
     * @param word The value to modify
     * @param bits The values to set
     * @param mask Bits to change
     */
    static inline unsigned setBits(unsigned word, unsigned bits, unsigned mask) {
      return (word & ~mask) | (bits & mask);
    }

    /**
     * Add clock manager password to word
     */
    static inline unsigned cmPwd(unsigned word) {
      return setBits(word, (unsigned)CM_MODE::PASSWD, 0xFF000000);
    }

    void* peripheralsBase_;

  };

} // namespace peripherals

#endif // BCM2837_PERIPHERALS_H
