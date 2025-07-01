/**
 * @file config.h
 * @brief Configuration header file for STM32F401RC MCU.
 *
 * This file declares functions, types, and defines necessary for the MCU configuration
 * routines implemented in config.c. It relies on basic types and utility macros
 * defined in the main header file.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Link to the main project header file which contains basic types and macros
// Assumes STM32F401RC_MAIN.h defines types like tbyte, tword, tlong,
// and macros like SET_BIT(), CLR_BIT(), IS_BIT_SET(), etc.
// It also assumes safeguard macros like GPIO_SAFEGUARD_Init() are available.
#include "STM32F401RC_MAIN.h"

// Ensure compatibility with C++ compilers
#ifdef __cplusplus
extern "C" {
#endif

/* --- Return / Error Codes --- */
/** @defgroup CONFIG_ReturnCodes Configuration Return Codes
 * @{
 */
#define CONFIG_OK     0    /**< Configuration successful */
#define CONFIG_FAIL  -1    /**< Configuration failed */
/** @} */ // end of CONFIG_ReturnCodes


/* --- Optional Feature Defines --- */
/** @defgroup CONFIG_Features Configuration Feature Flags
 * @{
 */
// Optional: Enable Watchdog Timer (IWDG or WWDG) initialization
// Set to 1 to enable watchdog functionality, 0 to disable.
// The specific watchdog (IWDG/WWDG) and its parameters are handled in config.c
#define ENABLE_WATCHDOG 1
/** @} */ // end of CONFIG_Features


/* --- Function Declarations --- */
/** @defgroup CONFIG_Functions Configuration Functions
 * @{
 */

/**
 * @brief Initializes the microcontroller configuration.
 *
 * This function performs essential MCU setup tasks such as clock configuration,
 * peripheral initialization (GPIOs, system peripherals, etc.), and optional
 * feature setup based on the defined feature flags (e.g., watchdog).
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (WDI - Watchdog Independent?).
 *
 * This function is intended to be called periodically to prevent the watchdog timer
 * from timing out and resetting the MCU. It typically reloads the watchdog counter.
 * (Assuming 'WDI' refers to Watchdog Independent; please verify based on project context)
 */
void WDI_Reset(void);

/** @} */ // end of CONFIG_Functions


/* --- External Variables --- */
// Declare any configuration state or status variables that need to be
// accessed from other files. Keep this minimal.
// Example: extern tbyte config_status_flag; // Example - uncomment if needed


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */