/***********************************************************************************************************************
* File Name      : config.h
* Description    : Header file for device configuration module (config.c)
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

/***********************************************************************************************************************
* Includes
***********************************************************************************************************************/
/* Include main header for common types, macros, and device-specific includes */
/* Assumes STM32F401RC_MAIN.h contains definitions for tbyte, tword, tlong, SET_BIT, etc., */
/* as well as necessary peripheral register definitions and safeguard macros. */
#include "STM32F401RC_MAIN.h"

/***********************************************************************************************************************
* C++ Compatibility
***********************************************************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
* Global Symbols / Defines
***********************************************************************************************************************/

/*
 * @brief Symbolic return/error codes for configuration operations.
 *        (Note: Main initialization functions declared below are void,
 *         these codes are for potential internal helper functions or status variables)
 */
#define CONFIG_OK   ((tbyte)0x00) /* Configuration successful */
#define CONFIG_FAIL ((tbyte)0x01) /* Configuration failed     */


/*
 * @brief Enable/Disable Watchdog timer initialization.
 *        Define this macro to enable the watchdog initialization in config.c.
 *        Comment or undefine to disable.
 *        (you assum that please change it based on project requirements)
 */
#define ENABLE_WATCHDOG


/***********************************************************************************************************************
* Global Function Declarations
***********************************************************************************************************************/

/**
 * @brief Initializes the microcontroller peripherals and clocks.
 *        This includes system clock, GPIOs, and other essential configurations.
 * @param  None
 * @retval None
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (if enabled).
 *        This function should be called periodically to prevent system reset.
 * @param  None
 * @retval None
 */
void WDI_Reset(void);


/***********************************************************************************************************************
* External Variables (Declare if needed)
***********************************************************************************************************************/

/* Example: Declare global configuration status variable if needed */
// extern tbyte g_config_status;


/***********************************************************************************************************************
* C++ Compatibility
***********************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */