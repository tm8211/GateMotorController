#include <pm.h>
#include <gpio.h>
#include <flashc.h>

#include "pll_init.h"
#include "user_board.h"

/**
 * @brief Start PLL @ 48Mhz to run with
 */
void Start_PLL(void)
{
    volatile avr32_pm_t *pm = &AVR32_PM;

    // Switch the main clock to OSC0
    pm_switch_to_osc0(pm, BOARD_OSC0_HZ, BOARD_OSC0_STARTUP_US);

    // For FOSC0 = 12.288 MHz:
    // Setup PLL0 on OSC0, mul=15 ,no divisor, lockcount=16, ie. 12 MHz x 16 = 196.608 MHz output.
	// Division by 2 (in options) yields PLL0 output as 98.304 MHz.
    // The formula is here:
    // if (PLLDIV > 0)
    //     fVCO = (PLLMUL+1)/(PLLDIV) • fOSC;
    // else
    //     fVCO = 2*(PLLMUL+1) • fOSC;
	// Lock count defines the PLL settling time (0x00 .. 0x3F.)
    pm_pll_setup(
        pm,     // volatile avr32_pm_t *pm
        0,      // PLL number(0 for PLL0, 1 for PLL1)
        15,     // PLL MUL in the PLL formula
        1,      // PLL DIV in the PLL formula
        0,      // OSC number (0 for osc0, 1 for osc1)
        16);    // unsigned int lockcount

    // Set PLL options to run @ 96 MHz
    pm_pll_set_option(
        pm,     // volatile avr32_pm_t *pm
        0,      // PLL number(0 for PLL0, 1 for PLL1)
        0,      // Set to 1 for VCO frequency range 80-180MHz, set to 0 for VCO frequency range 160-240Mhz
        1,      // Divide the PLL output frequency by 2 (this settings does not change the FVCO value)
        0);     // 1 Disable the Wide-Bandwidth Mode (Wide-Bandwidth mode allow a faster startup time and
                // out-of-lock time). 0 to enable the Wide-Bandwidth Mode.

    // Enable PLL0 and wait for it to lock.
    pm_pll_enable(pm, 0);
    pm_wait_for_pll0_locked(pm);
#if 0
#if defined(BOARD_OSC1_HZ)
    // We need to enable the crystal by hand. We didn't need to do that for OSC0
	// because the call to pm_switch_to_osc0() contains all that housekeeping.
	// But here we don't want to "switch to" the crystal.
    pm_enable_osc1_crystal(pm, BOARD_OSC1_HZ);      // Enable the OSC1 in crystal mode
    pm_enable_clk1(pm, BOARD_OSC1_STARTUP_US);

    // For FOSC1 = 12.000 MHz:
    // Setup PLL1 on OSC1, mul=15 ,no divisor, ie. 12 MHz x 15 = 180 MHz output.
	// Division by 2 (in options) yields PLL0 output as 96.000 MHz.
	// Lock count defines the PLL settling time (0x00 .. 0x3F.)
    // The formula is here:
    // if (PLLDIV > 0)
    //     fVCO = (PLLMUL+1)/(PLLDIV) • fOSC;
    // else
    //     fVCO = 2*(PLLMUL+1) • fOSC;
    pm_pll_setup(
        pm,     // volatile avr32_pm_t *pm
        1,      // PLL number(0 for PLL0, 1 for PLL1)
        15,     // PLL MUL in the PLL formula
        1,      // PLL DIV in the PLL formula
        1,      // OSC number (0 for osc0, 1 for osc1)
        16);    // unsigned int lockcount

    // Set PLL options to run @ 96.000 MHz
    pm_pll_set_option(
        pm,     // volatile avr32_pm_t *pm
        1,      // PLL number(0 for PLL0, 1 for PLL1)
        0,      // Set to 1 for VCO frequency range 80-180MHz, set to 0 for VCO frequency range 160-240Mhz
        1,      // Divide the PLL output frequency by 2 (this settings does not change the FVCO value)
        0);     // 1 Disable the Wide-Bandwidth Mode (Wide-Bandwidth mode allow a faster startup time and
                // out-of-lock time). 0 to enable the Wide-Bandwidth Mode.

    // Enable PLL1 and wait for it to lock.
    pm_pll_enable(pm, 1);
    pm_wait_for_pll1_locked(pm);

#endif /* defined(BOARD_OSC1_HZ) */
#endif
    // Select clocks as follows, considering that we are fed with PLL0 (98.304 MHz)
    // fHSB = fMain/2, or 49.152 MHz
    // fPBA = fMain/8, or 12.288 MHz
    // fPBB = fMain/2, or 49.152 MHz
    // ========================================
    // The formula (applies to all three channels, they are the same.)
    //
    // if (xDIV)  // Is the divisor enabled?
    //     fBUS = fMain / 2^(xSEL+1);   // Yes
    // else
    //     fBUS = fMain;                // No
    pm_cksel(
        pm,     // volatile avr32_pm_t *pm
        true,   // PBADIV: Peripheral Bus A clock divisor enable
        0,      // PBASEL: Peripheral Bus A divisor tap (fPBA = fMain/2, or 49.152 MHz)
        true,   // PBBDIV: Peripheral Bus B clock divisor enable
        0,      // PBBSEL: Peripheral Bus B select (fPBB = fMain/2, or 49.152 MHz)
        1,      // CPUDIV: CPU/HSB clock divisor enable
        0);     // CPUSEL: CPU/HSB clock divisor tap (fHSB = fMain/2, or 49.152 MHz)

    // One wait state at 48 MHz (required at fHSB > 33 MHz)
    flashc_set_wait_state(1);

    // Select the PLL0 as the clock source.
    pm_switch_to_clock(pm, AVR32_PM_MCCTRL_MCSEL_PLL0);
#if 0
#if defined(BOARD_OSC1_HZ)
    // Timers, communication modules, and other modules connected to external circuitry may require
    // specific clock frequencies to operate correctly. The Power Manager contains an implementation
    // defined number of generic clocks that can provide a wide range of accurate clock frequencies.
    // Each generic clock module runs from either Oscillator 0 or 1, or PLL0 or 1. The selected source
    // can optionally be divided by any even integer up to 512. Each clock can be independently
    // enabled and disabled, and is also automatically disabled along with peripheral clocks by the
    // Sleep Controller.

    // Setup generic clock number 0 on PLL, with OSC0/PLL0, no divisor.
    // A generic clock is enabled by writing the CEN bit in GCCTRL to 1. Each generic clock can use
    // either Oscillator 0 or 1 or PLL0 or 1 as source, as selected by the PLLSEL and OSCSEL bits.
    // The source clock can optionally be divided by writing DIVEN to 1 and the division factor to DIV,
    // resulting in the output frequency:
    //
    // fGCLK = fSRC / (2*(DIV+1))
    //
	// In this particular case we enable the generic clock to the USB module. Since the PLL1 is
	// producing 96.000 MHz we want to divide it by 2 to whittle it down to 48.000 +/- 0.25%.
	// The 48 MHz is the only clock that the USB module can use.
	//
    pm_gc_setup(
        pm,     // volatile avr32_pm_t *pm
        AVR32_PM_GCLK_USBB, // Generic clock number
        1,      // Use Osc (=0) or PLL (=1)
        1,      // Select Osc0/PLL0 or Osc1/PLL1
        1,      // DIVEN: Generic clock divisor enable
        0);     // DIV: Generic clock divisor (divide by 2)

    // Enable Generic clock 4 */
    pm_gc_enable(pm, AVR32_PM_GCLK_USBB);

#endif
#endif
#if 0 // Enable only for debugging, or if you really need to output the generic clock...
    pm_gc_setup(
        pm,     // volatile avr32_pm_t *pm
        AVR32_PM_GCLK_GCLK0, // Generic clock number
        1,      // Use Osc (=0) or PLL (=1)
        0,      // Select Osc0/PLL0 or Osc1/PLL1
        0,      // DIVEN: Generic clock divisor enable
        1);     // DIV: Generic clock divisor (divide by 0)

    // Enable Generic clock 0 */
    pm_gc_enable(pm, AVR32_PM_GCLK_GCLK0);

    /* Output the clock AVR32_PM_GCLK_GCLK0 to a GPIO (PB19) */
    gpio_enable_module_pin(AVR32_PM_GCLK_0_1_PIN, AVR32_PM_GCLK_0_1_FUNCTION);
#endif
}
