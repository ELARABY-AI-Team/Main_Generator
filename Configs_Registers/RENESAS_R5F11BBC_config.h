/**
 * @file config.h
 * @brief Configuration header file for RENESAS_R5F11BBC microcontroller.
 *
 * This file declares functions and definitions related to the initial
 * microcontroller configuration, clock settings, and peripheral setup.
 * It relies on definitions and types provided in the RENESAS_R5F11BBC_MAIN.h file.
 *
 * Note: This file should only declare items needed by config.c or other modules
 * that interact with the configuration functions.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Include Directives ---
/**
 * @brief Include the main microcontroller header file.
 * This header is assumed to contain essential typedefs (tbyte, tword, tlong),
 * bit manipulation macros (SET_BIT, CLR_BIT, etc.), and potential
 * GPIO/Register safeguard macros (GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init, etc.).
 * DO NOT REDEFINE these in this file.
 */
#include "RENESAS_R5F11BBC_MAIN.h"

// --- C++ Compatibility ---
#ifdef __cplusplus
extern "C" {
#endif

// --- Symbolic Return/Error Codes ---
/**
 * @brief Return code indicating successful operation.
 */
#define CONFIG_OK   0

/**
 * @brief Return code indicating operation failure.
 */
#define CONFIG_FAIL 1

// --- Optional Feature Configuration ---
/**
 * @brief Define this macro to enable Watchdog Timer (WDT) initialization and feeding.
 * If not defined, the Watchdog Timer functionality within config.c might be excluded
 * or simply do nothing.
 */
// #define ENABLE_WATCHDOG // Uncomment this line in config.h or the project settings to enable WDT

// --- Function Prototypes ---

/**
 * @brief Initializes the microcontroller unit (MCU) and essential peripherals.
 * This includes setting up the system clock, basic GPIO states, and potentially
 * other core configurations required before the main application loop starts.
 * It should call necessary safeguard functions from main.h if applicable.
 * @note Assumes the implementation in config.c uses typedefs and macros from RENESAS_R5F11BBC_MAIN.h.
 * @note Specific clock and peripheral settings will be implemented in config.c.
 */
void mcu_config_Init(void);

/**
 * @brief Resets or feeds the Watchdog Timer (WDT).
 * This function is intended to prevent an unwanted MCU reset by clearing or
 * feeding the WDT counter before it expires.
 * @note The specific implementation in config.c depends on the R5F11BBC's WDT peripheral registers.
 *       The name "WDI_Reset" might be slightly confusing; it typically means feeding the watchdog
 *       to *prevent* an MCU reset, not causing one. (Assumed interpretation: feeding/clearing the WDT)
 * @note This function is typically called periodically within the main application loop
 *       if the ENABLE_WATCHDOG macro is defined.
 */
void WDI_Reset(void);

// --- Extern Variables (if needed) ---
// Declare any global configuration state variables here if they need to be accessed
// from other files. Keep this minimal. Example:
// extern tbyte g_config_status; // Example: Global variable to hold configuration status (CONFIG_OK/CONFIG_FAIL)

// --- End of C++ Compatibility Wrapper ---
#ifdef __cplusplus
}
#endif

#endif // CONFIG_H_