/*
 * Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
 * Board initialization for Olimex LPC11C24
 */

#include "board.hpp"
#include <chip.h>
#include <cstdlib>
#include <cstring>
#include <numeric>


static constexpr unsigned long PDRUNCFGUSEMASK = 0x0000ED00U;
static constexpr unsigned long PDRUNCFGMASKTMP = 0x000000FFU;

const std::uint32_t OscRateIn = 12000000; ///< External crystal
const std::uint32_t ExtRateIn = 0;

std::uint32_t SystemCoreClock = 12000000; ///< Initialized to default clock value, will be changed on init

namespace board
{
namespace
{
static int mode_poll;	/* Poll/Interrupt mode flag */

constexpr unsigned TargetSystemCoreClock = 48000000;

constexpr unsigned ErrorLedPort = 1;
constexpr unsigned ErrorLedPin  = 10;

constexpr unsigned StatusLedPort = 1;
constexpr unsigned StatusLedPin  = 11;

struct PinMuxGroup
{
    unsigned pin      : 8;
    unsigned modefunc : 24;
};

constexpr PinMuxGroup pinmux[] =
{
    { IOCON_PIO1_10, IOCON_FUNC0 | IOCON_MODE_INACT },                                          // Error LED
    { IOCON_PIO1_11, IOCON_FUNC0 | IOCON_MODE_INACT },                                          // Status LED
    { IOCON_PIO0_4,  (IOCON_FUNC1 | IOCON_SFI2C_EN)}, /* PIO0_4 used for SCL */
    { IOCON_PIO0_5,  (IOCON_FUNC1 | IOCON_SFI2C_EN)}, /* PIO0_5 used for SDA */
    { IOCON_PIO1_7,  IOCON_FUNC1 | IOCON_HYS_EN | IOCON_MODE_PULLUP },                          // UART_TXD
};


void sysctlPowerDown(unsigned long powerdownmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun |= (powerdownmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void sysctlPowerUp(unsigned long powerupmask)
{
    unsigned long pdrun = LPC_SYSCTL->PDRUNCFG & PDRUNCFGMASKTMP;
    pdrun &= ~(powerupmask & PDRUNCFGMASKTMP);
    LPC_SYSCTL->PDRUNCFG = pdrun | PDRUNCFGUSEMASK;
}

void initWatchdog()
{
    Chip_WWDT_Init(LPC_WWDT);                                   // Initialize watchdog
    sysctlPowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);                  // Enable watchdog oscillator
    Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 4);                   // WDT osc rate 0.6 MHz / 4 = 150 kHz
    Chip_Clock_SetWDTClockSource(SYSCTL_WDTCLKSRC_WDTOSC, 1);   // Clocking watchdog from its osc, div rate 1
    Chip_WWDT_SetTimeOut(LPC_WWDT, 37500);                      // 1 sec (hardcoded to reduce code size)
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);          // Mode: reset on timeout
    Chip_WWDT_Start(LPC_WWDT);                                  // Go
}

void initClock()
{
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);   // Enable system oscillator
    for (volatile int i = 0; i < 1000; i++) { }

    Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
    sysctlPowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);

    /*
     * Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
     * MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
     * FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
     * FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range)
     */
    Chip_Clock_SetupSystemPLL(3, 1);
    sysctlPowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);
    while (!Chip_Clock_IsSystemPLLLocked()) { }

    Chip_Clock_SetSysClockDiv(1);

    Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

    Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

    SystemCoreClock = Chip_Clock_GetSystemClockRate();

    while (SystemCoreClock != TargetSystemCoreClock) { }  // Loop forever if the clock failed to initialize properly
}

void initGpio()
{
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_IOCON;
    LPC_SYSCTL->SYSAHBCLKCTRL |= 1 << SYSCTL_CLOCK_GPIO;

    for (unsigned i = 0; i < (sizeof(pinmux) / sizeof(PinMuxGroup)); i++)
    {
        LPC_IOCON->REG[pinmux[i].pin] = pinmux[i].modefunc;
    }

    LPC_GPIO[ErrorLedPort].DIR  |= 1 << ErrorLedPin;
    LPC_GPIO[StatusLedPort].DIR |= 1 << StatusLedPin;
}

static void Init_I2C_PinMux(void)
{
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1);
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if (!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(I2C0_IRQn);
	}
	else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(I2C0_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, uint32_t speed)
{
	Init_I2C_PinMux();

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 1);
}

/* State machine handler for I2C0 and I2C1 */
//static void i2c_state_handling(I2C_ID_T id)
//{
//	if (Chip_I2C_IsMasterActive(id)) {
//		Chip_I2C_MasterStateHandler(id);
//	}
//	else {
//		Chip_I2C_SlaveStateHandler(id);
//	}
//}

void init()
{
    Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2_06V, SYSCTL_BODINTVAL_RESERVED1);
    Chip_SYSCTL_EnableBODReset();

    initWatchdog();
    initClock();
    initGpio();
    i2c_app_init(I2C0, 400000);

    resetWatchdog();
}

} // namespace

void die()
{
    static const volatile unsigned& DHCSR = *reinterpret_cast<unsigned*>(0xE000EDF0U);

    while (true)
    {
        if ((DHCSR & 1U) != 0)
        {
            __asm volatile ("bkpt #0\n");   // Break into the debugger
        }
    }
}

#if __GNUC__
__attribute__((optimize(0)))     // Optimization must be disabled lest it hardfaults in the IAP call
#endif
void readUniqueID(std::uint8_t out_uid[UniqueIDSize])
{
    unsigned aligned_array[5] = {};  // out_uid may be unaligned, so we need to use temp array
    unsigned iap_command = 58;
    reinterpret_cast<void(*)(void*, void*)>(0x1FFF1FF1)(&iap_command, aligned_array);
    std::memcpy(out_uid, &aligned_array[1], 16);
}

void setStatusLed(bool state)
{
    LPC_GPIO[StatusLedPort].DATA[1 << StatusLedPin] = static_cast<unsigned long>(!state) << StatusLedPin;
}

void setErrorLed(bool state)
{
    LPC_GPIO[ErrorLedPort].DATA[1 << ErrorLedPin] = static_cast<unsigned long>(!state) << ErrorLedPin;
}

void resetWatchdog()
{
    Chip_WWDT_Feed(LPC_WWDT);
}

} // namespace board

extern "C"
{

void SystemInit();

void SystemInit()
{
    board::init();
}

}
