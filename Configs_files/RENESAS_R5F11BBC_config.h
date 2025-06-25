/*
 ******************************************************************************
 * File: config.h
 *
 * Description:
 *   Microcontroller configuration header file for the RENESAS_R5F11BBC.
 *   Declares functions and defines constants/features related to the
 *   initial system setup.
 *
 *   This file relies on type and macro definitions provided in
 *   RENESAS_R5F11BBC_MAIN.h and assumes necessary GPIO/Register
 *   safeguard macros like GPIO_SAFEGUARD_Init() are available from there.
 *
 * Copyright (c) [Your Company/Name] - [Current Year]
 * All Rights Reserved.
 ******************************************************************************
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// --- Includes ---
// Include the main project header for shared types, macros (like tbyte, SET_BIT),
// and assumed utility functions (like GPIO_SAFEGUARD_Init).
#include "RENESAS_R5F11BBC_MAIN.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Symbolic Constants ---

/**
 * @brief Configuration status code indicating success.
 */
#define CONFIG_OK           ((tbyte)0u)

/**
 * @brief Configuration status code indicating failure.
 */
#define CONFIG_FAIL         ((tbyte)1u)

// --- Optional Features ---

/**
 * @brief Feature flag to enable or disable the Watchdog Timer functionality.
 *
 * Set to 1 to enable the watchdog and require periodic WDI_Reset() calls.
 * Set to 0 to disable watchdog management in config.c.
 */
#define ENABLE_WATCHDOG     (1u) // Example: Enable Watchdog

// --- Function Declarations ---

/**
 * @brief Initializes the microcontroller configuration.
 *
 * This function performs necessary low-level setup including
 * clock configuration, basic GPIO setup using GPIO_SAFEGUARD_Init(),
 * and peripheral enablement using Registers_SAFEGUARD_Init(),
 * as required at system startup.
 *
 * Assumes GPIO_SAFEGUARD_Init() and Registers_SAFEGUARD_Init() are
 * declared and implemented elsewhere (e.g., in RENESAS_R5F11BBC_MAIN.h
 * or a separate system file included by main.h) and used here.
 */
void mcu_config_Init(void);

/**
 * @brief Resets the Watchdog Timer (Windowed Watchdog Timer - WDT).
 *
 * This function should be called periodically within the allowed window
 * to prevent a Watchdog Timer reset if ENABLE_WATCHDOG is defined
 * and non-zero. The specific implementation details (like the register
 * to write to) are handled in config.c.
 */
void WDI_Reset(void);

// --- Extern Variables ---
// (Optional: Declare any configuration-related state variables here if needed externally)
// Example (uncomment if required):
// extern tbyte config_status;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */