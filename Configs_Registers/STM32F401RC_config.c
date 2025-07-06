/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F401RC_config.h"
#include "STM32F401RC_MAIN.h" // Assumed to contain necessary macros, typedefs, and CMSIS includes

// Define a reasonable timeout value for clock operations
#define CLOCK_TIMEOUT ((tword)0x100000)

// Define a value for the WDT reload register (e.g., for ~11ms with prescaler 32)
#define WDT_RELOAD_VALUE ((tword)10) // (10 + 1) * (32 / LSI_freq) ~= 11ms if LSI is 32kHz

// Define the prescaler value for WDT (e.g., 32)
// Prescaler values: 0=/4, 1=/8, 2=/16, 3=/32, 4=/64, 5=/128, 6=/256
#define WDT_PRESCALER_VALUE ((tword)3) // Prescaler /32

// Define PVD level for ~2.8V threshold
// Note: STM32 PVD levels via PLS bits do not include 3.3V directly.
// Level 6 corresponds to 2.8V. This is assumed as the intended setting below 3.3V.
#define LVR_PVD_LEVEL_2_8V ((tword)6) // Corresponds to PLS[2:0] = 110

/**
 * @brief   Initializes system safeguards for GPIO and Registers.
 */
static void safe_guards(void)
{
    // Initialize GPIO safeguards
    GPIO_SAFEGUARD_Init();

    // Initialize Registers safeguards
    Registers_SAFEGUARD_Init();
}

/**
 * @brief   Initializes the Low Voltage Reset (LVR) threshold.
 *          Assumes LVR refers to configuring the Power Voltage Detector (PVD).
 * @return  0 on success.
 */
static int LVR_init(void)
{
    // Enable the Power peripheral clock
    // Assumed RCC_APB1ENR and RCC_APB1ENR_PWREN are defined in MAIN.h
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN); // Assumed — please change if needed

    // Select the PVD threshold level
    // Assumed PWR_CR and PWR_CR_PLS_Msk, PWR_CR_PLS_Pos are defined in MAIN.h
    MODIFY_REG(PWR->CR, PWR_CR_PLS_Msk, (LVR_PVD_LEVEL_2_8V << PWR_CR_PLS_Pos)); // Assumed — please change if needed

    return 0; // Success
}

/**
 * @brief   Enables the Low Voltage Reset (LVR).
 *          Assumes LVR refers to enabling the Power Voltage Detector (PVD).
 * @return  0 on success.
 */
static int LVR_Enable(void)
{
    // Enable the PVD
    // Assumed PWR_CR and PWR_CR_PVDE are defined in MAIN.h
    SET_BIT(PWR->CR, PWR_CR_PVDE); // Assumed — please change if needed

    return 0; // Success
}


/**
 * @brief   Initializes the Watchdog Timer (WDT) with a timeout >= 8ms.
 *          Assumes WDT refers to the Independent Watchdog (IWDG).
 * @return  0 on success.
 */
static int WDT_INIT(void)
{
    // Enable write access to IWDG_PR and IWDG_RLR
    // Assumed IWDG->KR is defined in MAIN.h
    IWDG->KR = 0x5555; // Enable register access key

    // Set Prescaler value
    // Assumed IWDG->PR and IWDG_PR_PR_Msk are defined in MAIN.h
    MODIFY_REG(IWDG->PR, IWDG_PR_PR_Msk, WDT_PRESCALER_VALUE);

    // Set Reload value
    // Assumed IWDG->RLR and IWDG_RLR_RL_Msk are defined in MAIN.h
    MODIFY_REG(IWDG->RLR, IWDG_RLR_RL_Msk, WDT_RELOAD_VALUE);

    // Note: WDT is enabled in WDT_Enable()

    return 0; // Success
}

/**
 * @brief   Enables the Watchdog Timer (WDT).
 *          Assumes WDT refers to the Independent Watchdog (IWDG).
 * @return  0 on success.
 */
int WDT_Enable(void)
{
    // Start the IWDG
    // Assumed IWDG->KR is defined in MAIN.h
    IWDG->KR = 0xCCCC; // Start IWDG key

    return 0; // Success
}

/**
 * @brief   Configures the system clock using the Internal clock source (HSI).
 * @return  0 on success, <0 on failure (timeout).
 */
int CLC_Init(void)
{
    tword timeout = CLOCK_TIMEOUT;

    // Enable HSI oscillator
    // Assumed RCC->CR and RCC_CR_HSION are defined in MAIN.h
    SET_BIT(RCC->CR, RCC_CR_HSION); // Assumed — please change if needed

    // Wait for HSI to be ready
    // Assumed RCC_CR_HSIRDY is defined in MAIN.h
    while (!READ_BIT(RCC->CR, RCC_CR_HSIRDY)) // Assumed — please change if needed
    {
        if ((timeout--) == 0)
        {
            return -1; // Timeout waiting for HSI ready
        }
    }

    // Select HSI as the system clock source
    // Assumed RCC->CFGR and RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI are defined in MAIN.h
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_HSI); // Assumed — please change if needed

    // Wait for HSI to be selected as the system clock source
    // Assumed RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI are defined in MAIN.h
    timeout = CLOCK_TIMEOUT;
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) // Assumed — please change if needed
    {
        if ((timeout--) == 0)
        {
            return -2; // Timeout waiting for SYSCLK switch to HSI
        }
    }

    // AHB, APB1, APB2 prescalers are assumed to be left at their reset default (/1)

    return 0; // Success
}

/**
 * @brief   Refreshes the Watchdog Timer (WDT).
 *          Assumes WDT refers to the Independent Watchdog (IWDG).
 */
void WDI_Reset(void)
{
    // Reload the IWDG counter
    // Assumed IWDG->KR is defined in MAIN.h
    IWDG->KR = 0xAAAA; // Reload IWDG key
}

/**
 * @brief   Initializes the microcontroller configuration including
 *          safeguards, LVR, WDT, and clock.
 */
void mcu_config_Init(void)
{
    // Initialize system safeguards
    safe_guards();

    // Initialize Low Voltage Reset (LVR) configuration
    LVR_init();

    // Enable Low Voltage Reset (LVR)
    LVR_Enable();

    // Initialize Watchdog Timer (WDT) configuration
    WDT_INIT();

    // Enable Watchdog Timer (WDT)
    WDT_Enable();

    // Configure Clock source
    CLC_Init();
}