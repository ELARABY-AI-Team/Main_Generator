```c++
/**
 * @file config.h
 * @brief Configuration header file for HOLTEK HT46R24 microcontroller.
 *
 * This file declares functions, variables, and defines specific to the
 * system's initial configuration and watchdog management. It relies on
 * definitions provided in HOLTEK_HT46R24_MAIN.h for base types and hardware access.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Includes ---
// Include main header file for base types (tbyte, tword, etc.), register definitions,
// and standard macros (SET_BIT, GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init, etc.).
#include "HOLTEK_HT46R24_MAIN.h"

// --- C++ Compatibility ---
// Ensure compatibility if used in a C++ project
#ifdef __cplusplus
extern "C" {
#endif

// --- Symbolic Return Codes ---
// Define standard return codes for configuration functions
#define CONFIG_OK       (0) ///< Configuration function executed successfully
#define CONFIG_FAIL     (1) ///< Configuration function encountered an error

// --- Feature Configuration ---
// Define optional build features. Uncomment a define to enable the feature.
// #define ENABLE_WATCHDOG // Uncomment to enable Watchdog Timer functionality

// --- Global Configuration Status/Variables ---
// Declare external variables used by config.c (defined in config.c)
// These variables typically hold configuration state or status.
extern tbyte g_mcu_initialized; // Flag indicating if initial MCU configuration is complete

// --- Function Declarations ---
// Declare functions provided by config.c

/**
 * @brief Initializes the microcontroller peripherals and core settings.
 *
 * This function is responsible for setting up the clock, GPIO directions
 * and initial states, peripheral modules (timers, interrupts, etc.),
 * and any other required initial configurations for the HT46R24.
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (WDI - Watchdog Timer In).
 *
 * This function is used to prevent a system reset by periodically
 * servicing the watchdog timer. It writes a specific sequence to the
 * watchdog register.
 *
 * @note This function is typically only relevant and effective if the
 *       Watchdog Timer has been enabled during initialization
 *       (e.g., via mcu_config_Init and potentially controlled by ENABLE_WATCHDOG).
 */
void WDI_Reset(void);

// --- End of C++ Compatibility ---
#ifdef __cplusplus
}
#endif

#endif // CONFIG_H_
```