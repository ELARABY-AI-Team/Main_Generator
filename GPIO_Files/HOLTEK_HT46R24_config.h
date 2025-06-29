/**
 * @file config.h
 * @brief Configuration header file for the HOLTEK HT46R24 microcontroller.
 *
 * Defines the public interface for microcontroller configuration functions
 * and essential configuration options. Assumes necessary base types and
 * macros are defined in HOLTEK_HT46R24_MAIN.h.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Link to main.h ---
// Assumes HOLTEK_HT46R24_MAIN.h provides base types (tbyte, tword, etc.)
// and common macros (SET_BIT, GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init, etc.)
#include "HOLTEK_HT46R24_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Return/Error Codes ---

/**
 * @brief Configuration successful return code.
 */
#define CONFIG_OK   0

/**
 * @brief Configuration failed return code.
 */
#define CONFIG_FAIL 1

// --- Configuration Options ---

/**
 * @brief Define to enable the Watchdog Timer (WDT).
 *        Comment out or undefine to disable WDT.
 */
#define ENABLE_WATCHDOG

// --- Function Declarations ---

/**
 * @brief Initializes the microcontroller's core peripherals, clock system,
 *        and essential configurations.
 *
 * This function should be called early in the application startup.
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (WDT).
 *
 * This function should be called periodically if the WDT is enabled.
 */
void WDI_Reset(void);

// --- External Variables (if needed) ---
// Example:
// extern tbyte config_status; /**< Current status of the configuration module */


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */