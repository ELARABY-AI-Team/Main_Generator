/***********************************************************************************************************************
* File Name      : config.h
* Description    : Configuration settings and function prototypes for the microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      :
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

/*
 * Link to main.h for shared typedefs (tbyte, tword, tlong)
 * and shared macros (SET_BIT, GPIO_SAFEGUARD_Init, etc.).
 * Do not redefine them here.
 */
#include "_MAIN.h" // Assuming main.h is generated as _MAIN.h

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 *  MACRO DEFINITIONS
 ***********************************************************************************************************************/

/* Define return/error codes for configuration functions */
#define CONFIG_OK  (0)  /**< Configuration successful */
#define CONFIG_FAIL (1)  /**< Configuration failed */

/*
 * Define optional features via macros.
 * Enable/disable features by changing the value (e.g., 1 for enabled, 0 for disabled).
 */
#define ENABLE_WATCHDOG (1) // Assume watchdog is an optional feature, enable/disable here. Please change if not applicable.

/***********************************************************************************************************************
 *  EXTERNAL DECLARATIONS
 ***********************************************************************************************************************/

/*
 * Declare configuration state variables if config.c needs to expose its internal state
 * to other modules. Omitted for minimalism unless specifically required.
 * Example: extern tbyte config_initialization_status;
 */

/***********************************************************************************************************************
 *  FUNCTION PROTOTYPES
 ***********************************************************************************************************************/

/**
 * @brief Initializes essential microcontroller configurations (clocks, peripherals, etc.).
 * @param[in] None.
 * @param[out] None.
 * @return None.
 * @details This function should be called early in the boot process to set up the MCU.
 */
void mcu_config_Init(void);

/**
 * @brief Triggers a Watchdog Timer (WDI) reset.
 * @param[in] None.
 * @param[out] None.
 * @return None.
 * @details This function is typically used to manually trigger a system reset via the WDI.
 */
void WDI_Reset(void);


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */