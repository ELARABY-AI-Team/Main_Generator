#ifndef CONFIG_H_
#define CONFIG_H_

#include "ATMEGA32_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the MCU configuration (clock, ports, safety features)
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer
 */
void WDI_Reset(void);

#ifdef ENABLE_WATCHDOG
#define WATCHDOG_TIMEOUT 500  // Assume 500ms timeout - adjust according to your needs
#endif

#define CONFIG_OK          0   // Configuration successful
#define CONFIG_FAIL       -1   // Configuration failed

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */