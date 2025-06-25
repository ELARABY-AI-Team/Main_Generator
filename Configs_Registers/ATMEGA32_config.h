/**
 * @file config.h
 * @brief ATMEGA32 MCU Configuration Header File.
 *
 * This header file declares functions and defines configuration settings
 * used by the config.c implementation file. It relies on typedefs
 * and macros provided by ATMEGA32_MAIN.h.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Link to the main system header which contains common types and macros
// Assumes ATMEGA32_MAIN.h has already been generated and contains
// tbyte, tword, tlong, SET_BIT, CLEAR_BIT, TOGGLE_BIT, CHECK_BIT,
// GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init, etc.
#include "ATMEGA32_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Symbolic return code for success.
 */
#define CONFIG_OK   0x00

/**
 * @brief Symbolic return code for failure.
 */
#define CONFIG_FAIL 0x01

// --- Optional Configuration Features ---

/**
 * @brief Define this macro to enable the Watchdog Timer configuration
 *        and usage within config.c.
 *        Comment out or undefine to disable watchdog features.
 */
#define ENABLE_WATCHDOG   1


// --- Function Declarations ---

/**
 * @brief Initializes essential MCU peripherals and settings
 *        like clock, GPIO direction, basic module setup.
 *        It is expected to call safeguard functions like
 *        GPIO_SAFEGUARD_Init and Registers_SAFEGUARD_Init
 *        internally if needed by config.c implementation.
 */
void mcu_config_Init(void);

/**
 * @brief Triggers a watchdog timer reset if the watchdog is enabled.
 *        This function is typically called periodically in the main loop
 *        or critical tasks to prevent a reset.
 */
void WDI_Reset(void);


// --- Extern Variables ---
// No extern variables are strictly needed in this minimal header for
// the declared functions. Add them here if config.c needs to expose
// state or status variables to other modules.

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */