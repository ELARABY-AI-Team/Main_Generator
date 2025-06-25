/**
 * @file config.h
 * @brief Configuration header file for the RENESAS R5F11BBC microcontroller.
 *
 * This header defines the necessary declarations and configurations
 * required by the config.c implementation. It relies on common types
 * and macros defined in RENESAS_R5F11BBC_MAIN.h.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Wrap C code declarations with extern "C" for C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

// 🔗 Linking to main.h - Contains common types (tbyte, tword, tlong),
// bit manipulation macros (SET_BIT, etc.), and potentially base register definitions
// and safety macros (GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init).
// We assume this file has been generated and contains these definitions.
#include "RENESAS_R5F11BBC_MAIN.h"

// ----------------------------------------------------------------------------
// 🔧 Function Declarations
// ----------------------------------------------------------------------------

/**
 * @brief Initializes the microcontroller peripherals and core settings.
 *
 * This function performs essential initializations like clock setup,
 * peripheral configurations (GPIO, timers, etc.), and potentially
 * enables necessary safeguards.
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (WDI).
 *
 * This function should be called periodically to prevent the watchdog
 * from timing out and causing a system reset.
 */
void WDI_Reset(void);


// ----------------------------------------------------------------------------
// 🔧 Optional Features and Configuration Settings
// ----------------------------------------------------------------------------

// Define ENABLE_WATCHDOG to enable watchdog timer configuration and usage.
// Comment indicates this is an assumed setting. Change if watchdog is not used.
#define ENABLE_WATCHDOG     1U // Assume watchdog is enabled, change to 0U if not used

// Example: Define a default watchdog timeout value (assuming milliseconds)
// This value is a placeholder; please configure based on system requirements
#define WATCHDOG_TIMEOUT_MS 1000U // Assumed watchdog timeout in milliseconds, configure appropriately


// If config.c needs to expose global state or status variables,
// declare them here using 'extern'.
// Example (commented out):
// extern tbyte config_status; // Example: Status variable (e.g., indicating initialization success/failure)


// ----------------------------------------------------------------------------
// 📘 Symbolic Return/Error Codes
// ----------------------------------------------------------------------------

// Define symbolic codes for function return values
#define CONFIG_OK   0   // Configuration successful
#define CONFIG_FAIL -1  // Configuration failed

// ----------------------------------------------------------------------------
// 📘 Notes
// ----------------------------------------------------------------------------

// This file should be kept minimal, declaring only what is needed by config.c
// and external modules interacting with configuration functions.
// All core types and generic macros are expected to be in RENESAS_R5F11BBC_MAIN.h.


// End of extern "C" block
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */