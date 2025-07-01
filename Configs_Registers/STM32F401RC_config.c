/**
 * @file config.c
 * @brief Provides microcontroller configuration functions for STM32F401RC.
 *
 * This file implements the configuration functions declared in
 * STM32F401RC_config.h, setting up essential system features
 * including safeguards, LVR, WDT, and clock source.
 *
 * Assumed definitions and includes:
 * - Peripheral base addresses and structures (RCC, PWR, WWDG) from STM32F401RC_MAIN.h
 *   or a file it includes (e.g., CMSIS stm32f4xx.h).
 * - Register bit definition macros (e.g., RCC_CR_HSION_Pos, PWR_CR_PVDE_Msk, etc.)
 *   from STM32F401RC_MAIN.h or included headers.
 * - Register manipulation macros (SET_BIT, CLEAR_BIT, MODIFY_REG, READ_BIT)
 *   from STM32F401RC_MAIN.h.
 * - GPIO_SAFEGUARD_Init() and Registers_SAFEGUARD_Init() functions declared in
 *   STM32F401RC_MAIN.h (assumed to return int, 0 for success).
 * - Configuration constants (LVR_THRESHOLD_PLS_BITS, WWDG_INIT_T, etc.)
 *   are defined in STM32F401RC_MAIN.h or locally with #ifndef checks.
 */

#include "STM32F401RC_config.h"
#include "STM32F401RC_MAIN.h"

// --- Configuration Constants (Assumed to be defined in STM32F401RC_MAIN.h or similar) ---

// Low Voltage Reset Threshold
// Assumes a specific PLS level is defined for operation in the 3.3V range.
// Level 6 (approx 2.8V threshold) is a common choice below 3.3V for robustness.
#ifndef LVR_THRESHOLD_PLS_BITS
// Assumed: PLS bits for ~2.8V threshold (Level 6 = 110 binary)
// Please verify this corresponds to Level 6 in your specific header/datasheet.
#define LVR_THRESHOLD_PLS_BITS    (0x6UL << PWR_CR_PLS_Pos) // that you assum that please change it
#endif

// WWDG Configuration
// Assumes values for T, W, and Prescaler that yield >= 8ms timeout
// based on the final PCLK1 frequency (set in CLC_Init).
// With PCLK1 = 16MHz (from HSI direct), Prescaler=8 (WDGTB=3), T=0x7F, W=0x50:
// Timeout = (4096 * 2^WDGTB * (T[6:0] - 0x3F)) / PCLK1
// Timeout = (4096 * 8 * (0x7F - 0x3F)) / 16e6 = (4096 * 8 * 64) / 16e6 ≈ 131 ms (> 8ms).
#ifndef WWDG_INIT_T
#define WWDG_INIT_T               0x7FUL // that you assum that please change it: Initial WWDG counter value (T[6:0]), must be > W. Max is 0x7F.
#endif
#ifndef WWDG_INIT_W
#define WWDG_INIT_W               0x50UL // that you assum that please change it: WWDG window value (W[6:0]), must be < T+1 and >= 0x40.
#endif
#ifndef WWDG_INIT_PRESCALER_BITS
// WDGTB[1:0]: 00=/1, 01=/2, 10=/4, 11=/8
// Assumed: WWDG Prescaler = 8 (WDGTB = 3)
#define WWDG_INIT_PRESCALER_BITS  (0x3UL << WWDG_CFR_WDGTB_Pos) // that you assum that please change it
#endif

// Clock Configuration Timeout
#ifndef CLOCK_TIMEOUT_COUNT
#define CLOCK_TIMEOUT_COUNT       100000UL // Assumed: Generic timeout count for clock readiness loops.
#endif

// --- Error Codes ---
// Defined locally as they are specific return values for these functions.
// Could also be defined in STM32F401RC_MAIN.h if desired.
#define CONFIG_SUCCESS             0
#define CONFIG_ERR_SAFEGUARD      -1
// #define CONFIG_ERR_LVR_INIT    -2 // Currently LVR init/enable don't return errors based on hardware flags
// #define CONFIG_ERR_LVR_ENABLE  -3
#define CONFIG_ERR_WDT_INIT       -4
#define CONFIG_ERR_CLOCK_TIMEOUT  -5


// --- Local Function Declarations (Static) ---

/**
 * @brief Initializes necessary safety features like GPIO and register defaults.
 * Calls GPIO_SAFEGUARD_Init() and Registers_SAFEGUARD_Init().
 * @return 0 on success, negative on failure.
 */
static int safe_guards(void);

/**
 * @brief Initializes the Low Voltage Reset (LVR) threshold.
 * Configures the Power Voltage Detector (PVD) level selection.
 * @return 0 on success. (Returns int for consistency, no explicit failure check)
 */
static int LVR_init(void);

/**
 * @brief Enables the Low Voltage Reset (LVR) detection.
 * Enables the Power Voltage Detector (PVD).
 * @return 0 on success. (Returns int for consistency, no explicit failure check)
 */
static int LVR_Enable(void);

/**
 * @brief Initializes the Window Watchdog Timer (WWDG) configuration.
 * Sets the prescaler, window value, and initial counter value.
 * Assumes PCLK1 frequency is set by CLC_Init for timeout calculation.
 * @return 0 on success. (Returns int for consistency, no explicit failure check)
 */
static int WDT_INIT(void);


// --- Function Implementations ---

/**
 * @brief Initializes necessary safety features.
 * Calls GPIO_SAFEGUARD_Init() and Registers_SAFEGUARD_Init().
 * @return 0 on success, negative on failure.
 */
static int safe_guards(void) {
    int ret = CONFIG_SUCCESS;

    // Assume GPIO_SAFEGUARD_Init() and Registers_SAFEGUARD_Init() return int (0 for success).
    // Handle potential failures from the safeguard functions.
    if (GPIO_SAFEGUARD_Init() != 0) {
        ret = CONFIG_ERR_SAFEGUARD;
    }

    // Call Registers_SAFEGUARD_Init regardless, but capture its result.
    // A more complex handler might differentiate between GPIO and Register safeguard failures.
    if (Registers_SAFEGUARD_Init() != 0) {
        // If a previous step failed, keep that error code. Otherwise, set registers error.
        if (ret == CONFIG_SUCCESS) { // Check if previous call was successful
            ret = CONFIG_ERR_SAFEGUARD; // Use a single error code for safeguard failure
        }
    }

    return ret;
}

/**
 * @brief Initializes the Low Voltage Reset (LVR) threshold.
 * Configures the Power Voltage Detector (PVD) level selection.
 * @return 0 on success.
 */
static int LVR_init(void) {
    // Configure the PVD level selection (PLS) in the Power Control Register (PWR->CR).
    // Assumes LVR_THRESHOLD_PLS_BITS is defined and holds the bit value
    // for the desired threshold for 3.3V operation (e.g., 2.8V threshold).
    // PWR_CR_PLS_Msk assumed standard CMSIS mask for PLS bits.
    MODIFY_REG(PWR->CR, PWR_CR_PLS_Msk, LVR_THRESHOLD_PLS_BITS); // Clear existing PLS bits and set new ones

    return CONFIG_SUCCESS; // Setting a register value typically doesn't fail at this level.
}

/**
 * @brief Enables the Low Voltage Reset (LVR) detection.
 * Enables the Power Voltage Detector (PVD).
 * @return 0 on success.
 */
static int LVR_Enable(void) {
    // Enable the Power Voltage Detector (PVD) by setting the PVDE bit in PWR->CR.
    // PWR_CR_PVDE_Msk assumed standard CMSIS mask for PVDE bit.
    SET_BIT(PWR->CR, PWR_CR_PVDE_Msk);

    return CONFIG_SUCCESS; // Enabling PVD typically doesn't fail if PWR clock is enabled (it usually is by default).
}

/**
 * @brief Initializes the Window Watchdog Timer (WWDG) configuration.
 * Sets the prescaler, window value, and initial counter value.
 * Assumes PCLK1 frequency is set by CLC_Init for timeout calculation.
 * @return 0 on success.
 */
static int WDT_INIT(void) {
    // Enable WWDG clock in RCC->APB1ENR.
    // RCC_APB1ENR_WWDGEN_Msk assumed standard CMSIS mask for WWDGEN bit.
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN_Msk);

    // Wait for the clock to stabilize (optional but good practice, though WWDG doesn't have a ready flag)
    // A small delay or dummy read might be added here if needed for bus synchronization.

    // Configure WWDG Control and Status Register (WWDG->CFR).
    // Set Prescaler (WDGTB) and Window value (W).
    // Assumes WWDG_INIT_PRESCALER_BITS and WWDG_INIT_W are defined constants.
    // WWDG_CFR_WDGTB_Msk, WWDG_CFR_W_Msk, WWDG_CFR_W_Pos assumed standard CMSIS definitions.
    MODIFY_REG(WWDG->CFR,
               WWDG_CFR_WDGTB_Msk | WWDG_CFR_W_Msk, // Clear Prescaler and Window bits
               WWDG_INIT_PRESCALER_BITS | (WWDG_INIT_W << WWDG_CFR_W_Pos)); // Set new values

    // Load the initial counter value (T) into WWDG->CR[6:0].
    // This value must be greater than the Window value (W).
    // Assumes WWDG_INIT_T is a defined constant.
    // WWDG_CR_T_Msk and WWDG_CR_T_Pos assumed standard CMSIS mask and position.
    // This initial value MUST be written BEFORE the watchdog is enabled via WDGA bit.
    MODIFY_REG(WWDG->CR, WWDG_CR_T_Msk, WWDG_INIT_T << WWDG_CR_T_Pos);

    return CONFIG_SUCCESS; // WWDG register setup typically doesn't fail.
}

/**
 * @brief Enables the Window Watchdog Timer (WWDG).
 * Starts the WWDG countdown. Must be called after WDT_INIT.
 * @return 0 on success.
 */
int WDT_Enable(void) {
    // Enable WWDG by setting the WDGA bit in WWDG->CR.
    // WWDG_CR_WDGA_Msk assumed standard CMSIS mask for WDGA bit.
    SET_BIT(WWDG->CR, WWDG_CR_WDGA_Msk);

    return CONFIG_SUCCESS; // Enabling the WWDG peripheral typically doesn't fail if its clock is on.
}


/**
 * @brief Configures the system clock source to Internal (HSI).
 * Sets up HSI as SYSCLK and configures bus prescalers for a default setup.
 * @return 0 on success, negative on failure (e.g., timeout during clock readiness).
 */
int CLC_Init(void) {
    __IO uint32_t timeout = 0;

    // 1. Enable HSI (High-Speed Internal oscillator, typically 16MHz on STM32F401).
    // RCC_CR_HSION_Pos assumed standard CMSIS position for HSION bit.
    SET_BIT(RCC->CR, (1UL << RCC_CR_HSION_Pos)); // Set HSION bit

    // 2. Wait for HSI to be ready.
    // RCC_CR_HSIRDY_Pos assumed standard CMSIS position for HSIRDY bit.
    timeout = CLOCK_TIMEOUT_COUNT; // Assumed CLOCK_TIMEOUT_COUNT is defined.
    while (READ_BIT(RCC->CR, (1UL << RCC_CR_HSIRDY_Pos)) == 0) { // Poll HSIRDY bit
        if (timeout-- == 0) {
            // HSI failed to start within the defined timeout period.
            return CONFIG_ERR_CLOCK_TIMEOUT;
        }
    }

    // 3. Configure bus prescalers (AHB, APB1, APB2).
    // Setting all to Div1 prescaler for a direct 16MHz on all buses (from HSI).
    // This is a simple and safe initial configuration.
    // RCC_CFGR_HPRE_Pos, RCC_CFGR_PPRE1_Pos, RCC_CFGR_PPRE2_Pos assumed standard CMSIS positions.
    MODIFY_REG(RCC->CFGR,
               (0xFUL << RCC_CFGR_HPRE_Pos) | // AHB Prescaler Mask (4 bits)
               (0x7UL << RCC_CFGR_PPRE1_Pos) | // APB1 Prescaler Mask (3 bits)
               (0x7UL << RCC_CFGR_PPRE2_Pos), // APB2 Prescaler Mask (3 bits)
               (0x0UL << RCC_CFGR_HPRE_Pos) |  // Set AHB Prescaler to Div1 (code 0000)
               (0x0UL << RCC_CFGR_PPRE1_Pos) | // Set APB1 Prescaler to Div1 (code 000)
               (0x0UL << RCC_CFGR_PPRE2_Pos)); // Set APB2 Prescaler to Div1 (code 000)

    // 4. Select HSI as System Clock source.
    // RCC_CFGR_SW_Msk and RCC_CFGR_SW_HSI assumed standard CMSIS mask/value for SW bits.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI);

    // 5. Wait for HSI to be selected as System Clock source (check SWS bits).
    // RCC_CFGR_SWS_Msk and RCC_CFGR_SWS_HSI assumed standard CMSIS mask/value for SWS bits.
    timeout = CLOCK_TIMEOUT_COUNT; // Reset timeout counter.
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) { // Poll SWS bits
        if (timeout-- == 0) {
            // System clock switch failed within the defined timeout period.
            return CONFIG_ERR_CLOCK_TIMEOUT;
        }
    }

    return CONFIG_SUCCESS; // Clock configuration completed successfully.
}


/**
 * @brief Main microcontroller configuration initialization function.
 * Calls all necessary setup functions in the specified sequence.
 * @return 0 on success, negative on first encountered error.
 */
int mcu_config_Init(void) {
    int ret = CONFIG_SUCCESS;

    // 1. Safe guards initialization
    ret = safe_guards();
    if (ret != CONFIG_SUCCESS) {
        // Return immediately on safeguard failure, as system state might be critical.
        return ret;
    }

    // 2. LVR initialization
    ret = LVR_init();
    // LVR init typically doesn't fail, but check return for robustness.
    if (ret != CONFIG_SUCCESS) { return ret; }

    // 3. LVR Enable
    ret = LVR_Enable();
    // LVR enable typically doesn't fail, but check return for robustness.
    if (ret != CONFIG_SUCCESS) { return ret; }

    // 4. WDT initialization
    ret = WDT_INIT();
     if (ret != CONFIG_SUCCESS) {
         // Return immediately on WDT init failure.
        return ret;
    }

    // 5. WDT Enable (WDT starts counting down after this!)
    ret = WDT_Enable();
     if (ret != CONFIG_SUCCESS) {
         // Return immediately on WDT enable failure.
        return ret;
    }
    // Note: WDT is now active and must be refreshed periodically by WDI_Reset().

    // 6. Clock initialization
    // Note: The WDT timing calculation in WDT_INIT assumes the PCLK1
    // frequency that will be set here (16MHz from HSI direct).
    // If CLC_Init were to configure a different PCLK1 frequency (e.g., from PLL),
    // the WDT timing would change AFTER it has started, which might not be desired.
    // The current implementation follows the requested sequence.
    ret = CLC_Init();
    if (ret != CONFIG_SUCCESS) {
         // Return immediately on clock init failure.
        return ret;
    }

    return CONFIG_SUCCESS; // All configuration steps completed successfully.
}


/**
 * @brief Resets/refreshes the Window Watchdog Timer (WWDG).
 * Writes the configured T value to the WWDG_CR register to reload the counter.
 * This prevents a watchdog reset, provided the refresh occurs within the valid window.
 */
void WDI_Reset(void) {
    // Refresh the WWDG by writing the initial T value to the counter register (WWDG->CR[6:0]).
    // This reloads the downcounter to the value previously written during WDT_INIT.
    // Assumes WWDG_INIT_T is the correct T value to write back.
    // WWDG_CR_T_Msk and WWDG_CR_T_Pos assumed standard CMSIS mask/position for T[6:0].
    // The refresh must occur when the counter value is between W and T (inclusive of T, exclusive of W).
    // Writing to WWDG_CR outside this window or when counter reaches 0x3F causes a reset.
    WWDG->CR = (WWDG->CR & ~WWDG_CR_T_Msk) | (WWDG_INIT_T << WWDG_CR_T_Pos);
}