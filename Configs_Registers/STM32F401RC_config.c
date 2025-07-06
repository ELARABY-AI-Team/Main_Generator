/***********************************************************************************************************************
* File Name      : config.c
* Description    : MCU Configuration functions
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F401RC_config.h"
#include "STM32F401RC_MAIN.h"

// Assuming a timeout value macro exists in STM32F401RC_MAIN.h
#ifndef TIMEOUT_VALUE
#define TIMEOUT_VALUE 0xFFFF // Assumed — please change if needed
#endif

// --- Static Helper Functions ---

/**
 * @brief Performs initial hardware and register safeguards.
 */
static void safe_guards(void)
{
    // Call safeguard initialization functions from MAIN.h
    GPIO_SAFEGUARD_Init();      // Initialize GPIO safeguards
    Registers_SAFEGUARD_Init(); // Initialize register safeguards
}

/**
 * @brief Initializes Low Voltage Reset (LVR) threshold for 3.3V operation.
 * @note Assumes macros for PWR register and voltage scale/level exist in MAIN.h.
 */
static void LVR_init(void)
{
    // Configure the Power Control System (PVD/LVR) threshold
    // Assumed: PWR_CR_VOS_SCALE1 for 3.3V operating range and a suitable LVR level.
    // Assumed: PWR_CR_PLS_LEVELx macros exist for PVD/LVR levels.
    // Using Level 1 (~2.3V) as an example threshold from datasheet options.
    MODIFY_REG(PWR->CR, PWR_CR_VOS_MASK | PWR_CR_PLS_MASK, PWR_CR_VOS_SCALE1 | PWR_CR_PLS_LEVEL1); // Assumed macros — please change if needed
}

/**
 * @brief Enables the Low Voltage Reset (LVR).
 * @note Assumes macro for PVD enable bit in PWR_CR exists in MAIN.h.
 */
static void LVR_Enable(void)
{
    // Enable Power Voltage Detector (PVD), which is used for LVR
    SET_BIT(PWR->CR, PWR_CR_PVDEN); // Assumed macro — please change if needed
}

/**
 * @brief Initializes the Independent Watchdog Timer (IWDG).
 * @note Configures IWDG for a timeout >= 8ms using LSI clock (~32kHz).
 *       Assumes IWDG registers and key/prescaler/reload macros exist in MAIN.h.
 */
static void WDT_INIT(void)
{
    // Unlock IWDG registers for write access to PR and RLR
    IWDG->KR = IWDG_KR_ENABLE_ACCESS; // Assumed macro — please change if needed

    // Set prescaler to /32 (for LSI ~32kHz, period becomes ~1ms per count).
    // Timeout = (Reload + 1) * Prescaler / LSI_Freq
    // With Prescaler 32 and Reload 255: (255+1)*32/32000 = 256*32/32000 = 8192/32000 = ~0.256s (256ms), which is > 8ms.
    IWDG->PR = IWDG_PR_DIV32; // Assumed macro for /32 prescaler — please change if needed

    // Set reload value
    IWDG->RLR = 255; // Assumed value 255 provides sufficient timeout based on prescaler choice — please change if needed

    // Hardware automatically locks PR and RLR again after the first write.
    // The KR register remains accessible for reload command.
}


// --- Non-Static Configuration Functions ---

/**
 * @brief Starts the Independent Watchdog Timer (IWDG).
 * @note Assumes IWDG start key macro exists in MAIN.h.
 */
void WDT_Enable(void)
{
     // Write the start key to IWDG->KR to start the counter
    IWDG->KR = IWDG_KR_START; // Assumed macro — please change if needed
}

/**
 * @brief Configures the microcontroller clock using the internal HSI oscillator.
 * @return 0 if successful, -1 otherwise.
 * @note Assumes RCC and FLASH registers and related macros exist in MAIN.h.
 */
int CLC_Init(void)
{
    uint32_t timeout = 0;

    // 1. Enable HSI oscillator (16MHz internal RC)
    SET_BIT(RCC->CR, RCC_CR_HSION); // Assumed macro — please change if needed

    // 2. Wait for HSI to be ready
    timeout = TIMEOUT_VALUE;
    while ((READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0) && (timeout > 0)) // Assumed macro — please change if needed
    {
        timeout--;
    }
    if (timeout == 0)
    {
        // HSI failed to start or become ready within timeout
        return -1;
    }

    // 3. Configure Flash prefetch, instruction cache, data cache, and latency
    // For 16MHz HSI, 0 wait states is sufficient.
    // Assumed FLASH_ACR register and related macros exist.
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_MASK, FLASH_ACR_LATENCY_0WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN); // Assumed macros — please change if needed

    // 4. Select HSI as the system clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_MASK, RCC_CFGR_SW_HSI); // Assumed macros for SW field and HSI value — please change if needed

    // 5. Wait for system clock switch status flag to confirm HSI is selected
    timeout = TIMEOUT_VALUE;
     while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS_HSI) == 0) && (timeout > 0)) // Assumed macro for SWS status check — please change if needed
    {
        timeout--;
    }
    if (timeout == 0)
    {
        // System clock switch to HSI failed within timeout
        return -1;
    }

    // Clock configuration successful
    return 0;
}

/**
 * @brief Reloads (feeds) the Independent Watchdog Timer (IWDG).
 * @note Prevents the watchdog timer from resetting the microcontroller.
 *       Assumes IWDG reload key macro exists in MAIN.h.
 */
void WDI_Reset(void)
{
    // Write the reload key to IWDG->KR
    IWDG->KR = IWDG_KR_RELOAD; // Assumed macro — please change if needed
}

// --- Main Configuration Function ---

/**
 * @brief Initializes the microcontroller configuration including safeguards, LVR, WDT, and clock.
 * @return 0 if successful, -1 otherwise.
 */
int mcu_config_Init(void)
{
    int ret = 0;

    // 1. Apply initial safeguards for GPIOs and Registers
    safe_guards();

    // 2. Initialize LVR threshold (for 3.3V operation)
    LVR_init();

    // 3. Enable LVR
    LVR_Enable();

    // 4. Initialize Watchdog Timer (WDT) parameters (prescaler, reload)
    WDT_INIT();

    // 5. Enable Watchdog Timer (WDT) - This starts the counter.
    //    Feeding the WDT should begin immediately after this call.
    WDT_Enable(); // Note: WDT is often started right before the main application loop begins

    // 6. Configure Clock Source to Internal HSI
    ret = CLC_Init();
    if (ret != 0)
    {
        // Clock configuration failed
        return -1;
    }

    // All initial configurations successful
    return 0;
}