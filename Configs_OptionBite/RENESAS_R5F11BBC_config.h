/**
 * @file config.h
 * @brief Header file for microcontroller configuration specific to RENESAS_R5F11BBC.
 *
 * This header declares the necessary functions and definitions for the
 * configuration module (config.c). It relies on common definitions
 * provided by the main project header.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Include main project header ---
// Assumes this header contains common project definitions like typedefs (tbyte, tword, tlong)
// and system macros (SET_BIT, GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init).
#include "RENESAS_R5F11BBC_MAIN.h"

// --- C++ Compatibility ---
#ifdef __cplusplus
extern "C" {
#endif

// --- Return/Error Codes ---
#define CONFIG_OK   0 // Configuration successful
#define CONFIG_FAIL 1 // Configuration failed

// --- Optional Features ---
// Define ENABLE_WATCHDOG to enable the Watchdog Timer initialization and feeding.
// Undefine to disable Watchdog usage in config.c.
#define ENABLE_WATCHDOG // Enable Watchdog Timer configuration

// --- Extern Variables ---
// No external configuration state variables are declared in this minimal header.
// Add 'extern type variable_name;' here if config.c needs to expose
// specific status or configuration data globally.

// --- Function Declarations ---

/**
 * @brief Initializes the microcontroller peripherals and system clock.
 * @details This function performs necessary setup for clocks, GPIO,
 *          and other core peripherals required for the application.
 */
void mcu_config_Init(void);

/**
 * @brief Resets (feeds) the Watchdog Timer (WDT).
 * @details This function should be called periodically from the main loop
 *          or a timer interrupt to prevent a WDT reset, provided
 *          ENABLE_WATCHDOG is defined.
 */
void WDI_Reset(void);


// --- End of C++ Compatibility ---
#ifdef __cplusplus
} // extern "C"
#endif

#endif // CONFIG_H_