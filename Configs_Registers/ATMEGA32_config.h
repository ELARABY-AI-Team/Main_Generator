```c++
/**
 * @file config.h
 * @brief Configuration header file for the ATMEGA32 microcontroller.
 *        Defines interfaces and configuration options used by config.c.
 *        Relies on ATMEGA32_MAIN.h for core typedefs and macros.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Include necessary shared definitions ---
// Assumes ATMEGA32_MAIN.h contains typedefs like tbyte, tword, tlong,
// and macros like SET_BIT, CLR_BIT, GET_BIT, TOGGLE_BIT,
// and safety functions like GPIO_SAFEGUARD_Init, Registers_SAFEGUARD_Init.
#include "ATMEGA32_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Optional Feature Configuration ---

// Define ENABLE_WATCHDOG if the Watchdog Timer should be enabled during configuration
// #define ENABLE_WATCHDOG // Uncomment this line to enable the watchdog timer (Assumption: User decides this)

// --- Return/Error Codes ---

#define CONFIG_OK   (0) // Configuration successful
#define CONFIG_FAIL (1) // Configuration failed (Assumption: Standard 0 for success, non-zero for failure)

// --- External Variables ---

// Global status variable for configuration state (Assumption: config.c might expose its status)
// This variable uses the tbyte typedef from ATMEGA32_MAIN.h
extern tbyte config_status;


// --- Function Prototypes ---

/**
 * @brief Initializes the microcontroller peripherals, clocks, and system settings
 *        as required by the application. This function is typically called once
 *        at the beginning of main().
 *        It is expected to call necessary safeguarding functions like
 *        Registers_SAFEGUARD_Init() and potentially GPIO_SAFEGUARD_Init().
 */
void mcu_config_Init(void);

/**
 * @brief Triggers a Watchdog Timer reset if the Watchdog is enabled and configured.
 *        This function is intended to be called to reset the MCU under specific
 *        application conditions. The actual mechanism depends on the WDT setup
 *        within config.c (e.g., setting WDCE and WDE in WDTCSR).
 */
void WDI_Reset(void);


#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
```