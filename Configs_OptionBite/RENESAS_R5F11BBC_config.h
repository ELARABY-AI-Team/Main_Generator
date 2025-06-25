#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * @file config.h
 * @brief Configuration header file for RENESAS_R5F11BBC microcontroller.
 *
 * Defines functions and constants necessary for system initialization
 * and configuration. Relies on definitions provided in RENESAS_R5F11BBC_MAIN.h.
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------

// Include the main application header. This header is assumed to provide
// core definitions, typedefs (like tbyte, tword, tlong), and utility
// macros (like SET_BIT, CLEAR_BIT) and safety functions (like
// GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init) used throughout the project.
// We explicitly rely on these here and do not redefine them.
#include "RENESAS_R5F11BBC_MAIN.h"

//------------------------------------------------------------------------------
// C++ Compatibility
//------------------------------------------------------------------------------

// Ensure C-style linkage for C++ compilers.
#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------

/**
 * @brief Initializes microcontroller peripherals and configuration settings.
 *
 * This function performs essential MCU setup, including clock configuration,
 * basic GPIO setup, and potentially other initializations required before
 * the main application loop begins. It is the primary entry point for
 * hardware configuration.
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (WDI).
 *
 * This function should be called periodically within the application's main
 * loop or relevant task to prevent a watchdog timer reset.
 */
void WDI_Reset(void);

//------------------------------------------------------------------------------
// Return/Error Codes
//------------------------------------------------------------------------------

/** @brief Indicates that a configuration function executed successfully. */
#define CONFIG_OK   (0)

/** @brief Indicates that a configuration function encountered an error or failure. */
#define CONFIG_FAIL (-1)

//------------------------------------------------------------------------------
// Optional Feature Flags
// Add or remove these defines based on conditional compilation needs for
// configuration features (e.g., enabling specific peripherals or modes).
//------------------------------------------------------------------------------

/**
 * @brief Define to enable the Watchdog Timer configuration and periodic feeding.
 * If commented out, WDI features might be disabled in config.c.
 */
#define ENABLE_WATCHDOG

// Example: Define to enable a specific peripheral initialization
//#define ENABLE_PERIPHERAL_X

//------------------------------------------------------------------------------
// External Variables
// Declare any configuration-related state variables that need to be
// accessible from other files. Keep this minimal.
//------------------------------------------------------------------------------

// Example: Extern declaration for a configuration status flag (if needed)
// extern volatile tbyte g_config_status;


//------------------------------------------------------------------------------
// End C++ Compatibility
//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif

#endif // CONFIG_H_