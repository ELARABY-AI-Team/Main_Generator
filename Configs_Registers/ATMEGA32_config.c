/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "ATMEGA32_config.h"
#include "ATMEGA32_MAIN.h"

// Define a maximum loop count for timeouts if a specific time delay or flag
// polling is required and F_CPU is not suitable for direct time calculations,
// or assume a macro like MAX_TIMEOUT_LOOP_COUNT exists in ATMEGA32_MAIN.h.
#ifndef MAX_TIMEOUT_LOOP_COUNT
// Assumed generous loop count for polling/delays — please change if needed
#define MAX_TIMEOUT_LOOP_COUNT 100000UL
#endif

/**
 * @brief Initializes the Low Voltage Reset threshold for operation at 3.3V.
 *        Note: Standard ATMEGA32 BOD threshold (2.7V or 4.0V) is fuse-set.
 *        This function assumes ATMEGA32_MAIN.h provides macros for potential
 *        external voltage monitoring or initialization steps supporting 3.3V.
 */
static void LVR_init(void)
{
    // Assumed: ATMEGA32_MAIN.h provides macros/logic to configure the LVR feature.
    // This might involve setting thresholds in a dedicated LVR/BOD control register
    // if available, or initializing external monitoring hardware.
    // Example placeholder using hypothetical macros:
    // LVR_CONFIG_REG = (LVR_CONFIG_REG & ~LVR_THRESHOLD_MASK) | LVR_THRESHOLD_3V3_VALUE; // Assumed — please change if needed
}

/**
 * @brief Enables the Low Voltage Reset feature.
 *        Note: Standard ATMEGA32 BOD enable is fuse-set. This function assumes
 *        ATMEGA32_MAIN.h provides macros to enable the feature if possible at runtime.
 */
static void LVR_Enable(void)
{
    // Assumed: ATMEGA32_MAIN.h provides macros/logic to enable the LVR feature.
    // This might involve setting an enable bit in a control register or activating
    // external monitoring hardware.
    // Example placeholder using a hypothetical macro:
    // SET_BIT(LVR_CONTROL_REG, LVR_ENABLE_BIT); // Assumed — please change if needed
}

/**
 * @brief Initializes the Watchdog Timer configuration to a timeout period >= 8ms.
 *        Sets the WDT prescaler for the smallest available timeout >= 8ms, which is 16ms on ATMEGA32.
 */
static void WDT_INIT(void)
{
    // Configure WDT for 16ms (WDP2=0, WDP1=0, WDP0=1).
    // Sequence is critical:
    // 1. Write logical one to WDTOE and WDE bits in WDTCR within 4 clock cycles.
    WDTCR_REG = (1 << WDTOE_BIT) | (1 << WDE_BIT); // Assumed WDTCR_REG, WDTOE_BIT, WDE_BIT macros exist in ATMEGA32_MAIN.h
    // 2. Immediately (within next 4 clock cycles) write logical one to WDE and the desired prescaler bits.
    WDTCR_REG = (1 << WDE_BIT) | (1 << WDP0_BIT);  // Assumed WDP0_BIT macro exists (WDP1_BIT=0, WDP2_BIT=0 for 16ms)
}

/**
 * @brief Sets up basic system safeguards by initializing GPIO and other key registers.
 */
static void safe_guards(void)
{
    // Initialize GPIO safeguard settings
    GPIO_SAFEGUARD_Init(); // Assumed macro/function call from ATMEGA32_MAIN.h
    // Initialize other critical registers safeguard settings
    Registers_SAFEGUARD_Init(); // Assumed macro/function call from ATMEGA32_MAIN.h
}

/**
 * @brief Enables the Watchdog Timer after it has been configured.
 * @return int - Always returns 0 on success.
 */
int WDT_Enable(void)
{
    // WDT_INIT should be called before this function.
    // Enable the Watchdog Timer by setting the WDE bit in WDTCR.
    SET_BIT(WDTCR_REG, WDE_BIT); // Assumed WDTCR_REG, WDE_BIT macros exist in ATMEGA32_MAIN.h
    return 0; // Success (WDT enable command issued)
}

/**
 * @brief Configures the system clock source to Internal and waits for stability.
 *        Note: Primary clock source on ATMEGA32 is fuse-set. This function assumes
 *        internal RC is the desired source and may involve calibration or waiting
 *        if ATMEGA32_MAIN.h provides macros for this.
 * @return int - 0 on success, <0 on failure (timeout waiting for stability).
 */
int CLC_Init(void)
{
    volatile tword timeout_counter = 0; // Use tword typedef from ATMEGA32_MAIN.h
    int status = -1; // Default to failure (timeout)

    // Assumed: Internal RC oscillator is the target clock source, likely via fuses.
    // This function attempts to wait for its stability or calibration completion.
    // Standard ATMEGA32 internal RC often lacks a runtime 'ready' flag.
    // This implementation uses a timeout loop structure assuming a flag check is possible
    // via ATMEGA32_MAIN.h definitions. If not, the loop provides a fixed delay.

    // Example polling logic assuming ATMEGA32_MAIN.h provides status register and ready bit:
    // while (timeout_counter < MAX_TIMEOUT_LOOP_COUNT)
    // {
    //     // Check if the clock source is ready/stable or calibration complete
    //     // Assuming ATMEGA32_MAIN.h defines CLK_STATUS_REG and CLK_READY_BIT
    //     if (READ_BIT(CLK_STATUS_REG, CLK_READY_BIT)) // Assumed — please change if needed
    //     {
    //         status = 0; // Success
    //         break; // Exit timeout loop
    //     }
    //     timeout_counter++;
    // }

    // If ATMEGA32_MAIN.h defines macros for a specific ready flag:
#if defined(CLK_STATUS_REG) && defined(CLK_READY_BIT)
    // Wait loop is active and checks the flag. Return status based on loop outcome.
     while (timeout_counter < MAX_TIMEOUT_LOOP_COUNT)
     {
         if (READ_BIT(CLK_STATUS_REG, CLK_READY_BIT))
         {
             status = 0; // Success
             break;
         }
         timeout_counter++;
     }
     return status; // Returns 0 on success, -1 on timeout

#else
    // If no specific runtime ready flag for internal RC, assume stability after
    // power-up and/or calibration if performed elsewhere. The timeout loop
    // above acts as a fixed delay. Return success as there's no flag to fail on.
    // The loop is still executed as a delay based on MAX_TIMEOUT_LOOP_COUNT.
     while (timeout_counter < MAX_TIMEOUT_LOOP_COUNT)
     {
        // Dummy loop for delay if no flag is polled
        timeout_counter++;
     }
    return 0; // Success (assuming internal RC is stable or ready without explicit flag check)
#endif
}

/**
 * @brief Resets (refreshes) the Watchdog Timer to prevent a system reset.
 */
void WDI_Reset(void)
{
    // Sequence to refresh WDT (clear WDT counter):
    // 1. Write logical one to WDTOE (Watchdog Turn-off Enable) and WDE (Watchdog Enable) in WDTCR.
    //    This write must be completed within four clock cycles.
    // Use macros from ATMEGA32_MAIN.h.
    WDTCR_REG = (1 << WDTOE_BIT) | (1 << WDE_BIT); // Assumed WDTCR_REG, WDTOE_BIT, WDE_BIT macros exist
    // 2. Immediately following step 1, write logical one to WDE and the *same* prescaler bits
    //    (WDP) as set during WDT_INIT (16ms configuration). This must be done within the next four clock cycles.
    WDTCR_REG = (1 << WDE_BIT) | (1 << WDP0_BIT);  // Assumed WDP0_BIT macro exists (and implies WDP1/WDP2 are 0 for 16ms)
}

/**
 * @brief Initializes the microcontroller configuration including safeguards, LVR, WDT, and clock.
 *        Calls necessary initialization functions in a specific sequence.
 */
void mcu_config_Init(void)
{
    // Call initialization functions in the specified strict sequence.
    // Note: This function is void and does not return status or check
    // return values of subordinate functions like WDT_Enable or CLC_Init.
    safe_guards();     // 1. Apply basic system safeguards for GPIO and registers
    LVR_init();        // 2. Initialize Low Voltage Reset settings/threshold
    LVR_Enable();      // 3. Enable Low Voltage Reset feature
    WDT_INIT();        // 4. Initialize Watchdog Timer (set timeout period >= 8ms)
    WDT_Enable();      // 5. Enable Watchdog Timer (returns int, but ignored as mcu_config_Init is void)
    CLC_Init();        // 6. Configure Clock Source to Internal and wait (returns int, but ignored as mcu_config_Init is void)
}