/***********************************************************************************************************************
* File Name      : config.h
* Description    : Header file for microcontroller configuration and initialization functions.
* Author         : Technology Inovation Software Team (Based on user request)
* Tester         : 
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-07
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

// Ensure the main ATMEGA32 definitions are included
// This file is assumed to contain necessary typedefs (tbyte, tword, tlong),
// bit manipulation macros (SET_BIT, CLR_BIT, etc.), and safety macros.
#include "ATMEGA32_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
*  GLOBAL DEFINES
***********************************************************************************************************************/

// Symbolic return/error codes
#define CONFIG_OK   (0)     // Configuration successful
#define CONFIG_FAIL (1)     // Configuration failed

// Optional Feature Macros
// Define this macro to enable Watchdog Timer initialization and usage
// If commented out, Watchdog related functions might be empty or excluded by the compiler.
#define ENABLE_WATCHDOG 

/***********************************************************************************************************************
*  GLOBAL FUNCTION PROTOTYPES
***********************************************************************************************************************/

/**
 * @brief Initializes the microcontroller configuration.
 * This function is responsible for setting up system clock, peripherals,
 * GPIO directions, and other initial system settings before the main loop.
 * It is expected to use safeguards like GPIO_SAFEGUARD_Init()
 * and Registers_SAFEGUARD_Init() if needed, which are assumed to be defined in ATMEGA32_MAIN.h.
 *
 * @param None
 */
void mcu_config_Init(void);

/**
 * @brief Triggers a Watchdog Timer reset.
 * This function should clear the Watchdog Timer to prevent a system reset.
 * It is typically called periodically in the main loop or interrupt handlers
 * to indicate that the system is operating correctly.
 * This function is only effective if ENABLE_WATCHDOG is defined.
 *
 * @param None
 */
void WDI_Reset(void);


/***********************************************************************************************************************
*  GLOBAL EXTERNS
***********************************************************************************************************************/

// Declare extern config state/status variables if needed.
// Example: extern tbyte g_config_status; // Example configuration status variable

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */