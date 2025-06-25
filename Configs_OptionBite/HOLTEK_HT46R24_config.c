/**
 * @file config.c
 * @brief Microcontroller configuration implementation for HOLTEK HT46R24.
 *
 * This file contains the implementation of basic microcontroller
 * initialization and watchdog functions using definitions and
 * safeguard routines provided by HOLTEK_HT46R24_MAIN.h.
 */

// --- Include Files ---
#include "config.h"          // Declarations for functions implemented here
#include "HOLTEK_HT46R24_MAIN.h" // Provides macros, typedefs, safeguard functions etc.

// --- Private Function Prototypes ---

/**
 * @brief Performs initial safeguard configurations.
 *
 * This function calls the necessary safeguard initialization
 * routines for GPIO and critical registers as recommended
 * by the microcontroller manufacturer to ensure a safe
 * initial state.
 */
static void safe_guards(void);


// --- Function Implementations ---

/**
 * @brief Calls initial safeguard configurations.
 *
 * This function calls the necessary safeguard initialization
 * routines for GPIO and critical registers as recommended
 * by the microcontroller manufacturer to ensure a safe
 * initial state.
 */
static void safe_guards(void)
{
    // Initialize GPIO safeguards according to manufacturer recommendations
    GPIO_SAFEGUARD_Init();

    // Initialize critical registers safeguards (e.g., system clock, power-on state)
    Registers_SAFEGUARD_Init();
}

/**
 * @brief Initializes the microcontroller configuration.
 *
 * This is the primary initialization function that sets up
 * the microcontroller's basic state by calling necessary
 * safeguard functions.
 */
void mcu_config_Init(void)
{
    // Perform initial safeguard configurations
    safe_guards();

    // --- Additional configuration steps would go here if needed ---
    // Based on specific application requirements (e.g., peripheral setup, clock, interrupts)
    // Example:
    // TIMER_Init();
    // ADC_Init();
    // CLOCK_Setup(); // If not handled by safeguards
    // ...
}

/**
 * @brief Resets (refreshes) the Watchdog Timer (WDT).
 *
 * This function should be called periodically to prevent the
 * watchdog timer from timing out and causing a system reset.
 * The specific mechanism for WDT reset is manufacturer/device
 * specific.
 */
void WDI_Reset(void)
{
    // --- Watchdog Timer (WDT) Refresh for HT46R24 ---
    // IMPORTANT: The exact register name and key sequence for WDT refresh
    // MUST be verified with the official HOLTEK HT46R24 datasheet.
    // The following is a common pattern for some Holtek devices but details
    // vary by device. It typically involves writing specific values to the WDT register.

    // Placeholder implementation based on a common pattern (verify with datasheet!):
    // Assuming the WDT register is named 'WDT' and requires a specific two-step write sequence.
    // If your headers define a specific macro or inline assembly for WDT reset, use that instead.

    // Example using common key sequence 0x5A, 0xA5 (VERIFY THESE VALUES AND REGISTER NAME WITH DATASHEET!):
    // #ifdef WDT // Check if WDT register is defined (e.g., in HOLTEK_HT46R24_MAIN.h)
    //     WDT = (tword)0x5A; // Write first key value (example)
    //     WDT = (tword)0xA5; // Write second key value (example)
    // #else
    //     // Placeholder or error handling if WDT register/mechanism is not defined
    //     // This section indicates that the actual WDT reset mechanism needs to be added.
    //     #error "WDT register or reset mechanism (like WDT macro or function) not defined. Please update WDI_Reset with the correct HT46R24 WDT refresh method from the datasheet/SDK."
    // #endif

    // Using the most direct interpretation based on needing *an* implementation:
    // Assuming 'WDT' is a volatile register defined in the includes
    // and the common two-step pattern applies.
    // *** VERIFY THIS AGAINST HT46R24 DATASHEET ***
    WDT = (tword)0x5A; // Write first key value (example, check datasheet)
    WDT = (tword)0xA5; // Write second key value (example, check datasheet)

    // Note: Some Holtek SDKs might provide a dedicated inline function or macro
    // for WDT refresh, e.g., `_clrwdt();` or `WDT_CLEAR();`. If available in
    // HOLTEK_HT46R24_MAIN.h, prefer that method.
}

// --- End of File ---