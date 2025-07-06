/***********************************************************************************************************************
* File Name      : config.h
* Description    : Header file defining configurations and interface for config.c.
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
/*
 * Include the main header file which provides core types (tbyte, tword, tlong),
 * bit manipulation macros (SET_BIT, etc.), and potentially safeguard functions.
 * All definitions from main.h are reused here.
 */
#include "STM32F401RC_MAIN.h" // Assumed main.h exists and contains necessary typedefs and macros.

/***********************************************************************************************************************
* Macro definitions
***********************************************************************************************************************/

/*
 * Wrap C declarations with extern "C" for C++ compatibility.
 */
#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Define to enable or disable the watchdog timer initialization and usage.
 *        Uncomment to enable, comment out to disable watchdog functionality.
 */
#define ENABLE_WATCHDOG // Assuming watchdog usage is a common requirement

/*
 * @brief Symbolic return/error codes for configuration functions.
 *        Using tbyte type as defined in STM32F401RC_MAIN.h.
 */
#define CONFIG_OK   ((tbyte)0)
#define CONFIG_FAIL ((tbyte)1)

/***********************************************************************************************************************
* Typedef definitions
***********************************************************************************************************************/
/*
 * All required typedefs (like tbyte, tword, tlong) are provided by including
 * "STM32F401RC_MAIN.h". No local typedefs are defined here to avoid redundancy.
 */

/***********************************************************************************************************************
* External variables
***********************************************************************************************************************/
/*
 * Declare configuration state/status variables here if they need to be
 * accessed by other modules. Defined in config.c.
 * Example: extern tbyte g_config_status;
 */

/***********************************************************************************************************************
* Global function prototypes
***********************************************************************************************************************/

/**
 * @brief Initializes the MCU configuration.
 *        system services (like watchdog if enabled), etc.
 *        Reuses GPIO and register safeguard macros from main.h if used in config.c.
 * @return CONFIG_OK if initialization is successful, CONFIG_FAIL otherwise.
 */
tbyte mcu_config_Init(void);

/**
 * @brief Triggers a watchdog reset.
 *        This function is responsible for initiating a system reset via the watchdog timer.
 *        (Assumes the watchdog module (IWDG/WWDG) is enabled and configured correctly
 *         in the mcu_config_Init or elsewhere. This function likely just services/kicks it).
 */
void WDI_Reset(void); // Assuming WDI stands for Watchdog Initiate or Reset


/*
 * End of extern "C" block.
 */
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */

/***********************************************************************************************************************
* End of File
***********************************************************************************************************************/