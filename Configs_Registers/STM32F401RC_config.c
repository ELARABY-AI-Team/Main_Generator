/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      :
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "_config.h"
#include "_MAIN.h"

/**
 * @brief Initializes GPIO and peripheral register safeguards.
 */
static void safe_guards(void)
{
    GPIO_SAFEGUARD_Init();      // Initialize GPIO safeguards
    Registers_SAFEGUARD_Init(); // Initialize peripheral register safeguards
}

/**
 * @brief Initializes Low Voltage Reset (LVR) threshold for 3.3V operation.
 */
static void LVR_init(void)
{
    // Assumed — please change if needed
    // Configure LVR threshold register for 3.3V operation
    LVR_THRESHOLD_REG = LVR_THRESHOLD_3V3_VALUE; // Set LVR threshold
}

/**
 * @brief Enables the Low Voltage Reset (LVR).
 */
static void LVR_Enable(void)
{
    // Assumed — please change if needed
    // Set the LVR enable bit in the control register
    SET_BIT(LVR_CTRL_REG, LVR_ENABLE_BIT); // Enable LVR module
}

/**
 * @brief Initializes the Watchdog Timer (WDT) for a timeout >= 8ms.
 */
static void WDT_INIT(void)
{
    // Assumed — please change if needed
    // Configure WDT prescaler and postscaler for desired timeout
    WDT_PRESCALER_REG = WDT_PRESCALER_VALUE;         // Set WDT prescaler
    WDT_POSTSCALER_REG = WDT_POSTSCALER_GE_8MS_VALUE; // Set WDT postscaler (>= 8ms)
}

/**
 * @brief Enables the Watchdog Timer (WDT).
 */
void WDT_Enable(void)
{
    // Assumed — please change if needed
    // Set the WDT enable bit in the control register
    SET_BIT(WDT_CTRL_REG, WDT_ENABLE_BIT); // Enable WDT module
}

/**
 * @brief Configures the microcontroller clock source to Internal.
 */
void CLC_Init(void)
{
    volatile tword timeout_counter = CLOCK_SWITCH_TIMEOUT_TICKS; // Use timeout macro from _MAIN.h

    // Assumed — please change if needed
    // Select the internal clock source
    CLOCK_SELECT_REG = CLOCK_SRC_INTERNAL_VALUE; // Set clock source to internal

    // Wait for the clock source to switch and become active
    // Check the clock status register for the internal clock active flag
    while ((CLOCK_STATUS_REG & CLOCK_SRC_INTERNAL_ACTIVE_FLAG) == 0 && timeout_counter > 0)
    {
        timeout_counter--; // Decrement timeout counter
    }

    // Note: As per function signature, cannot return error code on timeout.
    // Add error handling or logging here if needed in a modified version.
}

/**
 * @brief Refreshes (pets) the watchdog timer to prevent reset.
 */
void WDI_Reset(void)
{
    // Assumed — please change if needed
    // Write the watchdog refresh key value to the refresh register
    WDT_REFRESH_REG = WDT_REFRESH_KEY_VALUE; // Refresh the watchdog timer
}

/**
 * @brief Initializes the microcontroller configuration peripherals.
 *
 * This includes safeguards, LVR, WDT, and clock configuration.
 */
void mcu_config_Init(void)
{
    safe_guards();  // Initialize GPIO and register safeguards
    LVR_init();     // Initialize LVR threshold
    LVR_Enable();   // Enable LVR
    WDT_INIT();     // Initialize WDT settings
    WDT_Enable();   // Enable WDT
    CLC_Init();     // Configure and switch to internal clock source
}