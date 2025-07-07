/***********************************************************************************************************************
* File Name      : config.h
* Description    : Header file for ATMEGA32 microcontroller configuration.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------------------------------------------------------*/
/*--- INCLUDES ---*/
/* Include main header file for common typedefs and macros */
#include "ATMEGA32_MAIN.h" // Assumes ATMEGA32_MAIN.h contains tbyte, tword, tlong, SET_BIT, CLR_BIT, TOGGLE_BIT, CHECK_BIT, GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init, etc.

/*--------------------------------------------------------------------------------------------------------------------*/
/*--- SYMBOLIC CONSTANTS ---*/

/**
 * @brief Configuration function return status: OK
 */
#define CONFIG_OK     (0)

/**
 * @brief Configuration function return status: Failure
 */
#define CONFIG_FAIL   (1)

/*--------------------------------------------------------------------------------------------------------------------*/
/*--- CONFIGURATION OPTIONS ---*/

/**
 * @brief Define to enable Watchdog Timer functionality.
 *        Comment out or undefine to disable the Watchdog.
 */
#define ENABLE_WATCHDOG

/*
 * Add other configuration options here if needed, e.g.:
 * #define USE_EXTERNAL_OSC    // Define if using external crystal/oscillator
 * #define SYS_FREQ_HZ         (8000000UL) // Define system clock frequency (assumed value, please change if different)
 */


/*--------------------------------------------------------------------------------------------------------------------*/
/*--- FUNCTION DECLARATIONS ---*/

/**
 * @brief Initializes basic MCU configurations (clock, watchdog if enabled, etc.).
 *        Relies on safeguarding macros from ATMEGA32_MAIN.h.
 */
void mcu_config_Init(void);

/**
 * @brief Forces a Watchdog Timer reset.
 *        Only effective if Watchdog is enabled and configured.
 */
void WDI_Reset(void);


/*--------------------------------------------------------------------------------------------------------------------*/
/*--- EXTERNAL VARIABLES ---*/

/*
 * Declare extern variables here if config.c needs to share state with other files.
 * Example:
 * extern tbyte system_state; // Example: state variable managed by config
 */


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */