/***********************************************************************************************************************
* File Name      : config.h
* Description    : Header file for configuration functions and settings.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ESP32
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 🔗 Linking to main.h:
 * Include the main header file to reuse typedefs, bit macros, and safety macros.
 * This ensures consistency and avoids redefinitions.
 */
#include "ESP32_MAIN.h"

/***********************************************************************************************************************
 *                                      Symbolic Return/Error Codes                                                    *
 ***********************************************************************************************************************/

/**
 * @brief Represents a successful configuration operation.
 */
#define CONFIG_OK           (0)

/**
 * @brief Represents a failed configuration operation.
 */
#define CONFIG_FAIL         (1)

/***********************************************************************************************************************
 *                                      Optional Features                                                              *
 ***********************************************************************************************************************/

/**
 * @brief Define this macro to enable the Watchdog Timer functionality.
 * If commented out, Watchdog features will typically be disabled in config.c.
 */
#define ENABLE_WATCHDOG


/***********************************************************************************************************************
 *                                      Configuration Functions                                                        *
 ***********************************************************************************************************************/

/**
 * @brief Initializes the microcontroller peripherals and system settings.
 * This function should perform essential setup like clock configuration,
 * peripheral initialization, and safety mechanism enablement.
 * Note: Function signature requires void return and parameters.
 */
void mcu_config_Init(void);

/**
 * @brief Triggers a watchdog reset.
 * This function should interact with the Watchdog Timer to cause a system reset.
 * Note: Function signature requires void return and parameters.
 */
void WDI_Reset(void);


/***********************************************************************************************************************
 *                                      External Variables (If Needed)                                               *
 ***********************************************************************************************************************/

/*
 * Declare any configuration state or status variables that need to be accessed
 * from other files using the 'extern' keyword here.
 * Example: extern tbyte config_status;
 */

/*
 * Note on Assumptions:
 * This header file primarily contains declarations and simple definitions.
 * Any assumptions about specific hardware registers, timing values, or
 * peripheral configurations would reside in the corresponding config.c
 * implementation file. If a value or register address was assumed in config.c
 * during implementation, a comment should be placed beside it in that file
 * explaining the assumption and indicating it might need review/change.
 */


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */