/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file provides a production-ready implementation for PWM generation
*                  on the STM32F401RC microcontroller. It includes functions for
*                  initialization, frequency/duty cycle setting, starting, stopping,
*                  and power-off of PWM channels.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/*
 * Note on Timer Clock Frequencies:
 * The STM32F401RC's APB1 (TIM2, TIM3, TIM4, TIM5) and APB2 (TIM1, TIM9, TIM10, TIM11)
 * buses can run at different maximum frequencies. Timer clocks are typically derived from
 * these bus clocks. If the APB prescaler is 1, the timer clock is the same as the APB clock.
 * If the APB prescaler is greater than 1, the timer clock is double the APB clock.
 *
 * For STM32F401RC:
 * - Max APB1 frequency (PCLK1) = 42 MHz
 * - Max APB2 frequency (PCLK2) = 84 MHz
 *
 * We assume the following configuration for timer clock frequencies:
 * - APB1 prescaler is set such that TIM2/3/4/5 clock is 84 MHz (e.g., PCLK1=42MHz, APB1 prescaler > 1).
 * - APB2 prescaler is set such that TIM1/9/10/11 clock is 84 MHz (e.g., PCLK2=84MHz, APB2 prescaler = 1).
 */
#define APB1_TIM_CLK_FREQ       84000000UL /* Assumed timer clock frequency: PCLK1 * 2 */
#define APB2_TIM_CLK_FREQ       84000000UL /* Assumed timer clock frequency: PCLK2 * 1 */

/*
 * Max ARR values for 16-bit and 32-bit timers
 * TIM1, TIM3, TIM4, TIM9, TIM10, TIM11 are 16-bit timers.
 * TIM2, TIM5 are 32-bit timers.
 */
#define TIM_16BIT_MAX_ARR       0xFFFFUL     // 65535
#define TIM_32BIT_MAX_ARR       0xFFFFFFFFUL // 4294967295

/*
 * Timer reservation for OS/delay:
 * As per the requirements, TIM9 and TIM10 are reserved and will not be used for PWM.
 * This decision is based on them being single/dual channel 16-bit timers, making them
 * potentially suitable for system-level timing services.
 */


/***********************************************************************************************************************
* Global Array Definitions
***********************************************************************************************************************/

/**
 * @brief Array mapping TRD_Channel_t enum values to their corresponding PWM hardware configurations.
 *        This array defines the specific Timer, Channel, GPIO Port, Pin, and Alternate Function
 *        for each available PWM output on the STM32F401RC microcontroller.
 *
 * Each entry strictly follows the format:
 *   { TimerBaseAddress, TimerID, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber }
 *
 * Assumptions made for pin mappings (as specific tables were not provided in the PDF):
 * - Alternate function assignments (AFx) are based on common STM32F4xx series datasheets.
 * - Pin selections are chosen to provide a diverse set of available PWM outputs, avoiding pin 0
 *   unless explicitly documented as PWM capable (which is not in the provided PDF context).
 * - Duplicate (Port, Pin) pairs are avoided for unique entries, although a single pin can be
 *   configured for multiple alternate functions depending on the application.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 (Advanced-control timer, 16-bit, AF1)
    { TIM1_BASE, 1, 1, GPIOA, 8, 1 },  // TIM1_CH1 on PA8, AF1    /* Assumed PWM config */
    { TIM1_BASE, 1, 2, GPIOA, 9, 1 },  // TIM1_CH2 on PA9, AF1    /* Assumed PWM config */
    { TIM1_BASE, 1, 3, GPIOA, 10, 1 }, // TIM1_CH3 on PA10, AF1   /* Assumed PWM config */
    { TIM1_BASE, 1, 4, GPIOA, 11, 1 }, // TIM1_CH4 on PA11, AF1   /* Assumed PWM config */

    // TIM2 (General-purpose timer, 32-bit, AF1)
    // PA0 (TIM2_CH1) is often used but excluded as per instructions unless explicitly confirmed PWM-capable in provided doc.
    { TIM2_BASE, 2, 1, GPIOA, 15, 1 }, // TIM2_CH1 on PA15, AF1   /* Assumed PWM config */
    { TIM2_BASE, 2, 2, GPIOB, 3, 1 },  // TIM2_CH2 on PB3, AF1    /* Assumed PWM config */
    { TIM2_BASE, 2, 3, GPIOB, 10, 1 }, // TIM2_CH3 on PB10, AF1   /* Assumed PWM config */
    { TIM2_BASE, 2, 4, GPIOB, 11, 1 }, // TIM2_CH4 on PB11, AF1   /* Assumed PWM config */

    // TIM3 (General-purpose timer, 16-bit, AF2)
    { TIM3_BASE, 3, 1, GPIOA, 6, 2 },  // TIM3_CH1 on PA6, AF2    /* Assumed PWM config */
    { TIM3_BASE, 3, 2, GPIOA, 7, 2 },  // TIM3_CH2 on PA7, AF2    /* Assumed PWM config */
    { TIM3_BASE, 3, 3, GPIOB, 0, 2 },  // TIM3_CH3 on PB0, AF2    /* Assumed PWM config */
    { TIM3_BASE, 3, 4, GPIOB, 1, 2 },  // TIM3_CH4 on PB1, AF2    /* Assumed PWM config */

    // TIM4 (General-purpose timer, 16-bit, AF2)
    { TIM4_BASE, 4, 1, GPIOB, 6, 2 },  // TIM4_CH1 on PB6, AF2    /* Assumed PWM config */
    { TIM4_BASE, 4, 2, GPIOB, 7, 2 },  // TIM4_CH2 on PB7, AF2    /* Assumed PWM config */
    { TIM4_BASE, 4, 3, GPIOB, 8, 2 },  // TIM4_CH3 on PB8, AF2    /* Assumed PWM config */
    { TIM4_BASE, 4, 4, GPIOD, 12, 2 }, // TIM4_CH4 on PD12, AF2   /* Assumed PWM config */

    // TIM5 (General-purpose timer, 32-bit, AF2)
    // PA0 (TIM5_CH1) is skipped as per instructions (pin 0 exclusion).
    { TIM5_BASE, 5, 2, GPIOA, 1, 2 },  // TIM5_CH2 on PA1, AF2    /* Assumed PWM config */
    { TIM5_BASE, 5, 3, GPIOA, 2, 2 },  // TIM5_CH3 on PA2, AF2    /* Assumed PWM config */
    { TIM5_BASE, 5, 4, GPIOA, 3, 2 },  // TIM5_CH4 on PA3, AF2    /* Assumed PWM config */

    // TIM11 (General-purpose timer, 16-bit, AF3)
    { TIM11_BASE, 11, 1, GPIOB, 9, 3 } // TIM11_CH1 on PB9, AF3   /* Assumed PWM config */
};

/***********************************************************************************************************************
* Static Function Declarations
***********************************************************************************************************************/
/**
 * @brief Retrieves the PWM_Channel_Config_t structure for a given TRD_Channel_t.
 * @param channel_id The TRD_Channel_t enum value.
 * @return Pointer to the PWM_Channel_Config_t structure, or NULL if invalid.
 */
static const PWM_Channel_Config_t* get_channel_config(TRD_Channel_t channel_id);

/**
 * @brief Enables the clock for the specified GPIO port.
 * @param pGPIOx Pointer to the GPIO_TypeDef structure (e.g., GPIOA, GPIOB).
 */
static void enable_gpio_clock(GPIO_TypeDef *pGPIOx);

/**
 * @brief Enables the clock for the specified Timer peripheral.
 * @param timer_id The ID of the timer (e.g., 1 for TIM1, 2 for TIM2).
 */
static void enable_timer_clock(tbyte timer_id);

/**
 * @brief Disables the clock for the specified Timer peripheral.
 * @param timer_id The ID of the timer (e.g., 1 for TIM1, 2 for TIM2).
 */
static void disable_timer_clock(tbyte timer_id);

/**
 * @brief Helper function to get a type-casted pointer for TIM1.
 */
static TIM1_TypeDef* get_tim1_ptr(void* base_addr);

/**
 * @brief Helper function to get a type-casted pointer for general-purpose timers (TIM2-TIM5).
 */
static TIM_GP_TypeDef* get_tim_gp_ptr(void* base_addr);

/**
 * @brief Helper function to get a type-casted pointer for TIM9.
 */
static TIM9_TypeDef* get_tim9_ptr(void* base_addr);

/**
 * @brief Helper function to get a type-casted pointer for TIM10 and TIM11.
 */
static TIM10_11_TypeDef* get_tim10_11_ptr(void* base_addr);


/***********************************************************************************************************************
* Static Function Implementations
***********************************************************************************************************************/

static const PWM_Channel_Config_t* get_channel_config(TRD_Channel_t channel_id)
{
    if (channel_id >= TRD_CHANNEL_MAX)
    {
        return NULL; // Invalid channel ID
    }
    return &pwm_channel_map[channel_id];
}

static void enable_gpio_clock(GPIO_TypeDef *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); // Enable GPIOA clock /* Assumed RCC definition */
    }
    else if (pGPIOx == GPIOB)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN); // Enable GPIOB clock /* Assumed RCC definition */
    }
    else if (pGPIOx == GPIOC)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); // Enable GPIOC clock /* Assumed RCC definition */
    }
    else if (pGPIOx == GPIOD)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN); // Enable GPIOD clock /* Assumed RCC definition */
    }
    else if (pGPIOx == GPIOE)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN); // Enable GPIOE clock /* Assumed RCC definition */
    }
    else if (pGPIOx == GPIOH) // PH0/PH1 are generally used for HSE, but listed as GPIO H in PDF
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN); // Enable GPIOH clock /* Assumed RCC definition */
    }
}

static void enable_timer_clock(tbyte timer_id)
{
    switch (timer_id)
    {
        case 1:
            BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);  // Enable TIM1 clock /* Assumed RCC definition */
            break;
        case 2:
            BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);  // Enable TIM2 clock /* Assumed RCC definition */
            break;
        case 3:
            BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);  // Enable TIM3 clock /* Assumed RCC definition */
            break;
        case 4:
            BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);  // Enable TIM4 clock /* Assumed RCC definition */
            break;
        case 5:
            BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);  // Enable TIM5 clock /* Assumed RCC definition */
            break;
        case 9:
            BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);  // Enable TIM9 clock /* Assumed RCC definition */
            break;
        case 10:
            BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); // Enable TIM10 clock /* Assumed RCC definition */
            break;
        case 11:
            BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); // Enable TIM11 clock /* Assumed RCC definition */
            break;
        default:
            // Invalid timer ID or reserved timer
            break;
    }
}

static void disable_timer_clock(tbyte timer_id)
{
    switch (timer_id)
    {
        case 1:
            BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);  // Disable TIM1 clock /* Assumed RCC definition */
            break;
        case 2:
            BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);  // Disable TIM2 clock /* Assumed RCC definition */
            break;
        case 3:
            BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);  // Disable TIM3 clock /* Assumed RCC definition */
            break;
        case 4:
            BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);  // Disable TIM4 clock /* Assumed RCC definition */
            break;
        case 5:
            BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);  // Disable TIM5 clock /* Assumed RCC definition */
            break;
        case 9:
            BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);  // Disable TIM9 clock /* Assumed RCC definition */
            break;
        case 10:
            BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_TIM10EN); // Disable TIM10 clock /* Assumed RCC definition */
            break;
        case 11:
            BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_TIM11EN); // Disable TIM11 clock /* Assumed RCC definition */
            break;
        default:
            // Invalid timer ID or reserved timer
            break;
    }
}

static TIM1_TypeDef* get_tim1_ptr(void* base_addr)
{
    return (TIM1_TypeDef*)base_addr;
}

static TIM_GP_TypeDef* get_tim_gp_ptr(void* base_addr)
{
    return (TIM_GP_TypeDef*)base_addr;
}

static TIM9_TypeDef* get_tim9_ptr(void* base_addr)
{
    return (TIM9_TypeDef*)base_addr;
}

static TIM10_11_TypeDef* get_tim10_11_ptr(void* base_addr)
{
    return (TIM10_11_TypeDef*)base_addr;
}

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    // 1. Enable GPIO Clock
    enable_gpio_clock(config->PortName);

    // 2. Configure GPIO Pin for Alternate Function
    // Set MODER to Alternate Function (10b)
    REG_VAL_SET_MASK(config->PortName->MODER, (0x3UL << (config->PinNumber * 2)), (GPIO_MODER_AF << (config->PinNumber * 2))); /* PDF Reference (MODER) */
    // Set OTYPER to Push-pull (0b)
    REG_VAL_SET_MASK(config->PortName->OTYPER, (0x1UL << config->PinNumber), (GPIO_OTYPER_PP << config->PinNumber)); /* PDF Reference (OTYPER) */
    // Set OSPEEDR to Very High Speed (11b)
    REG_VAL_SET_MASK(config->PortName->OSPEEDR, (0x3UL << (config->PinNumber * 2)), (GPIO_OSPEEDR_VHIGH << (config->PinNumber * 2))); /* PDF Reference (OSPEEDR) */
    // Set PUPDR to No pull-up/pull-down (00b)
    REG_VAL_SET_MASK(config->PortName->PUPDR, (0x3UL << (config->PinNumber * 2)), (GPIO_PUPDR_NOPULL << (config->PinNumber * 2))); /* PDF Reference (PUPDR) */

    // Set Alternate Function Low (AFRL) or High (AFRH) Register
    if (config->PinNumber < 8)
    {
        REG_VAL_SET_MASK(config->PortName->AFRL, (0xFUL << (config->PinNumber * 4)), (config->AlternateFunctionNumber << (config->PinNumber * 4))); /* PDF Reference (AFRL) */
    }
    else
    {
        REG_VAL_SET_MASK(config->PortName->AFRH, (0xFUL << ((config->PinNumber - 8) * 4)), (config->AlternateFunctionNumber << ((config->PinNumber - 8) * 4))); /* PDF Reference (AFRH) */
    }

    // 3. Enable Timer Clock
    enable_timer_clock(config->TimerID);

    // 4. Configure Timer for PWM Generation
    uint32_t tim_clock_freq;
    volatile uint32_t* pCCMR; // Pointer to CCMR1 or CCMR2
    uint32_t ccmr_mask_offset; // Offset within CCMR for OCxM, OCxPE, OCxFE
    uint32_t ccer_mask_offset; // Offset within CCER for CCxE, CCxP, CCxNP
    volatile uint32_t* pCCR;   // Pointer to CCRx register

    // Determine timer clock frequency and cast TIMx pointer correctly
    switch (config->TimerID)
    {
        case 1:
        {
            TIM1_TypeDef* TIMx = get_tim1_ptr(config->TIMx_base_addr);
            tim_clock_freq = APB2_TIM_CLK_FREQ; /* Assumed timer clock frequency */

            // Disable counter before configuration
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); /* PDF Reference (CEN) */

            // Configure CCMR1/CCMR2 and CCER based on channel number
            if (config->ChannelNumber == 1 || config->ChannelNumber == 2)
            {
                pCCMR = &(TIMx->CCMR1);
                ccmr_mask_offset = (config->ChannelNumber == 1) ? 0 : 8;
                ccer_mask_offset = (config->ChannelNumber == 1) ? 0 : 4;
            }
            else // Channel 3 or 4
            {
                pCCMR = &(TIMx->CCMR2);
                ccmr_mask_offset = (config->ChannelNumber == 3) ? 0 : 8;
                ccer_mask_offset = (config->ChannelNumber == 3) ? 8 : 12;
            }

            // Configure Output Compare Mode (PWM Mode 1)
            REG_VAL_SET_MASK(*pCCMR, (TIM_CCMR_OCM_Msk << ccmr_mask_offset), (TIM_OCM_PWM1 << ccmr_mask_offset)); /* PDF Reference (OCxM) */
            // Enable Output Compare Preload (OCxPE)
            BIT_SET(*pCCMR, (TIM_CCMR_OCPE_Msk << ccmr_mask_offset)); /* PDF Reference (OCxPE) */

            // Set output polarity (Active High by default)
            BIT_CLR(TIMx->CCER, (TIM_CCER_CCP_Msk << ccer_mask_offset)); /* PDF Reference (CCxP) */
            // Enable Capture/Compare Output (CCxE)
            BIT_SET(TIMx->CCER, (TIM_CCER_CCE_Msk << ccer_mask_offset)); /* PDF Reference (CCxE) */

            // Enable Auto-reload preload (ARPE) for buffering ARR
            BIT_SET(TIMx->CR1, TIM_CR1_ARPE_Msk); /* PDF Reference (ARPE) */

            // Enable Main Output (MOE) for TIM1 (required for outputs)
            BIT_SET(TIMx->BDTR, TIM_BDTR_MOE_Msk); /* PDF Reference (MOE) */

            // Select CCR register
            switch (config->ChannelNumber) {
                case 1: pCCR = &(TIMx->CCR1); break;
                case 2: pCCR = &(TIMx->CCR2); break;
                case 3: pCCR = &(TIMx->CCR3); break;
                case 4: pCCR = &(TIMx->CCR4); break;
                default: pCCR = NULL; break; // Should not happen
            }
            break;
        }
        case 2:
        case 3:
        case 4:
        case 5:
        {
            TIM_GP_TypeDef* TIMx = get_tim_gp_ptr(config->TIMx_base_addr);
            tim_clock_freq = APB1_TIM_CLK_FREQ; /* Assumed timer clock frequency */

            // Disable counter before configuration
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); /* PDF Reference (CEN) */

            // Configure CCMR1/CCMR2 and CCER based on channel number
            if (config->ChannelNumber == 1 || config->ChannelNumber == 2)
            {
                pCCMR = &(TIMx->CCMR1);
                ccmr_mask_offset = (config->ChannelNumber == 1) ? 0 : 8;
                ccer_mask_offset = (config->ChannelNumber == 1) ? 0 : 4;
            }
            else // Channel 3 or 4
            {
                pCCMR = &(TIMx->CCMR2);
                ccmr_mask_offset = (config->ChannelNumber == 3) ? 0 : 8;
                ccer_mask_offset = (config->ChannelNumber == 3) ? 8 : 12;
            }

            // Configure Output Compare Mode (PWM Mode 1)
            REG_VAL_SET_MASK(*pCCMR, (TIM_CCMR_OCM_Msk << ccmr_mask_offset), (TIM_OCM_PWM1 << ccmr_mask_offset)); /* PDF Reference (OCxM) */
            // Enable Output Compare Preload (OCxPE)
            BIT_SET(*pCCMR, (TIM_CCMR_OCPE_Msk << ccmr_mask_offset)); /* PDF Reference (OCxPE) */

            // Set output polarity (Active High by default)
            BIT_CLR(TIMx->CCER, (TIM_CCER_CCP_Msk << ccer_mask_offset)); /* PDF Reference (CCxP) */
            // Enable Capture/Compare Output (CCxE)
            BIT_SET(TIMx->CCER, (TIM_CCER_CCE_Msk << ccer_mask_offset)); /* PDF Reference (CCxE) */

            // Enable Auto-reload preload (ARPE) for buffering ARR
            BIT_SET(TIMx->CR1, TIM_CR1_ARPE_Msk); /* PDF Reference (ARPE) */

            // Select CCR register
            switch (config->ChannelNumber) {
                case 1: pCCR = &(TIMx->CCR1); break;
                case 2: pCCR = &(TIMx->CCR2); break;
                case 3: pCCR = &(TIMx->CCR3); break;
                case 4: pCCR = &(TIMx->CCR4); break;
                default: pCCR = NULL; break; // Should not happen
            }
            break;
        }
        case 9:
        {
            TIM9_TypeDef* TIMx = get_tim9_ptr(config->TIMx_base_addr);
            tim_clock_freq = APB2_TIM_CLK_FREQ; /* Assumed timer clock frequency */

            // Disable counter before configuration
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); /* PDF Reference (CEN) */

            // TIM9 only has CCMR1 (for channels 1 and 2)
            pCCMR = &(TIMx->CCMR1);
            ccer_mask_offset = (config->ChannelNumber == 1) ? 0 : 4;
            ccmr_mask_offset = (config->ChannelNumber == 1) ? 0 : 8;

            // Configure Output Compare Mode (PWM Mode 1)
            REG_VAL_SET_MASK(*pCCMR, (TIM_CCMR_OCM_Msk << ccmr_mask_offset), (TIM_OCM_PWM1 << ccmr_mask_offset)); /* PDF Reference (OCxM) */
            // Enable Output Compare Preload (OCxPE)
            BIT_SET(*pCCMR, (TIM_CCMR_OCPE_Msk << ccmr_mask_offset)); /* PDF Reference (OCxPE) */

            // Set output polarity (Active High by default)
            BIT_CLR(TIMx->CCER, (TIM_CCER_CCP_Msk << ccer_mask_offset)); /* PDF Reference (CCxP) */
            // Enable Capture/Compare Output (CCxE)
            BIT_SET(TIMx->CCER, (TIM_CCER_CCE_Msk << ccer_mask_offset)); /* PDF Reference (CCxE) */

            // Enable Auto-reload preload (ARPE) for buffering ARR
            BIT_SET(TIMx->CR1, TIM_CR1_ARPE_Msk); /* PDF Reference (ARPE) */

            // Select CCR register
            switch (config->ChannelNumber) {
                case 1: pCCR = &(TIMx->CCR1); break;
                case 2: pCCR = &(TIMx->CCR2); break;
                default: pCCR = NULL; break; // Should not happen
            }
            break;
        }
        case 10:
        case 11:
        {
            TIM10_11_TypeDef* TIMx = get_tim10_11_ptr(config->TIMx_base_addr);
            tim_clock_freq = APB2_TIM_CLK_FREQ; /* Assumed timer clock frequency */

            // Disable counter before configuration
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); /* PDF Reference (CEN) */

            // TIM10/11 only have CCMR1 (for channel 1)
            pCCMR = &(TIMx->CCMR1);
            ccer_mask_offset = 0; // Channel 1
            ccmr_mask_offset = 0; // Channel 1

            // Configure Output Compare Mode (PWM Mode 1)
            REG_VAL_SET_MASK(*pCCMR, (TIM_CCMR_OCM_Msk << ccmr_mask_offset), (TIM_OCM_PWM1 << ccmr_mask_offset)); /* PDF Reference (OCxM) */
            // Enable Output Compare Preload (OCxPE)
            BIT_SET(*pCCMR, (TIM_CCMR_OCPE_Msk << ccmr_mask_offset)); /* PDF Reference (OCxPE) */

            // Set output polarity (Active High by default)
            BIT_CLR(TIMx->CCER, (TIM_CCER_CCP_Msk << ccer_mask_offset)); /* PDF Reference (CCxP) */
            // Enable Capture/Compare Output (CCxE)
            BIT_SET(TIMx->CCER, (TIM_CCER_CCE_Msk << ccer_mask_offset)); /* PDF Reference (CCxE) */

            // Enable Auto-reload preload (ARPE) for buffering ARR
            BIT_SET(TIMx->CR1, TIM_CR1_ARPE_Msk); /* PDF Reference (ARPE) */

            // Select CCR register (always CCR1 for TIM10/11)
            pCCR = &(TIMx->CCR1);
            break;
        }
        default:
            return; // Should not happen if config is valid
    }

    // Set a default frequency and duty cycle (e.g., 10kHz, 50%)
    PWM_Set_Freq(TRD_Channel, 10000, 50);

    // Generate an update event to load all preload registers (PSC, ARR, CCRx)
    // Clear URS bit to generate update event with flag
    if (config->TimerID == 1) { // TIM1_TypeDef
        BIT_CLR(get_tim1_ptr(config->TIMx_base_addr)->CR1, TIM_CR1_URS_Msk); /* PDF Reference (URS) */
        BIT_SET(get_tim1_ptr(config->TIMx_base_addr)->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
        BIT_CLR(get_tim1_ptr(config->TIMx_base_addr)->SR, 0x1); // Clear UIF flag /* PDF Reference (UIF) */
    } else if (config->TimerID >= 2 && config->TimerID <= 5) { // TIM_GP_TypeDef
        BIT_CLR(get_tim_gp_ptr(config->TIMx_base_addr)->CR1, TIM_CR1_URS_Msk); /* PDF Reference (URS) */
        BIT_SET(get_tim_gp_ptr(config->TIMx_base_addr)->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
        BIT_CLR(get_tim_gp_ptr(config->TIMx_base_addr)->SR, 0x1); // Clear UIF flag /* PDF Reference (UIF) */
    } else if (config->TimerID == 9) { // TIM9_TypeDef
        BIT_CLR(get_tim9_ptr(config->TIMx_base_addr)->CR1, TIM_CR1_URS_Msk); /* PDF Reference (URS) */
        BIT_SET(get_tim9_ptr(config->TIMx_base_addr)->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
        BIT_CLR(get_tim9_ptr(config->TIMx_base_addr)->SR, 0x1); // Clear UIF flag /* PDF Reference (UIF) */
    } else if (config->TimerID == 10 || config->TimerID == 11) { // TIM10_11_TypeDef
        BIT_CLR(get_tim10_11_ptr(config->TIMx_base_addr)->CR1, TIM_CR1_URS_Msk); /* PDF Reference (URS) */
        BIT_SET(get_tim10_11_ptr(config->TIMx_base_addr)->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
        BIT_CLR(get_tim10_11_ptr(config->TIMx_base_addr)->SR, 0x1); // Clear UIF flag /* PDF Reference (UIF) */
    }
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The specific PWM channel.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (config == NULL || frequency == 0)
    {
        return; // Invalid channel or frequency
    }

    if (duty > 100) duty = 100; // Cap duty cycle at 100%

    uint32_t tim_clock_freq;
    uint32_t max_arr_val;

    if (config->TimerID == 1 || config->TimerID == 9 || config->TimerID == 10 || config->TimerID == 11)
    {
        tim_clock_freq = APB2_TIM_CLK_FREQ; /* Assumed timer clock frequency */
        max_arr_val = TIM_16BIT_MAX_ARR;
    }
    else // TIM2, TIM3, TIM4, TIM5
    {
        tim_clock_freq = APB1_TIM_CLK_FREQ; /* Assumed timer clock frequency */
        max_arr_val = (config->TimerID == 2 || config->TimerID == 5) ? TIM_32BIT_MAX_ARR : TIM_16BIT_MAX_ARR;
    }

    tlong psc = 0;
    tlong arr = 0;
    tlong ccr = 0;

    // Calculate PSC and ARR for desired frequency
    // Aim to maximize ARR for best resolution, without exceeding max_arr_val
    // freq = TIM_CLK / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = TIM_CLK / freq
    // Choose PSC such that (ARR + 1) is within [1, max_arr_val + 1]
    // A simple approach is to calculate (PSC + 1) such that ARR is close to max_arr_val/2 or max_arr_val.
    // If PSC=0, then ARR+1 = TIM_CLK / freq. If this fits, it's max resolution.
    
    // Attempt minimum prescaler for max resolution
    if (frequency > tim_clock_freq) frequency = tim_clock_freq; // Cap frequency at TIM_CLK
    
    arr = (tim_clock_freq / frequency) - 1;
    if (arr > max_arr_val)
    {
        // If ARR exceeds max_arr_val, increase prescaler
        psc = (tim_clock_freq / (frequency * (max_arr_val + 1))) + 1;
        if (psc > 0) psc--; // Adjust to (PSC + 1) form
        if (psc > 0xFFFFUL) psc = 0xFFFFUL; // Clamp PSC to 16-bit max

        arr = (tim_clock_freq / (frequency * (psc + 1))) - 1;
        if (arr > max_arr_val) arr = max_arr_val; // Should not happen with correct PSC calculation, but for safety
    } else {
        psc = 0; // Minimal prescaler
    }

    // Calculate CCR value
    ccr = (arr + 1) * duty / 100;
    if (ccr > arr) ccr = arr; // Ensure CCR does not exceed ARR. Full ON is (ARR+1)
    if (duty == 0) ccr = 0;   // 0% duty cycle

    // Apply values to timer registers
    switch (config->TimerID)
    {
        case 1:
        {
            TIM1_TypeDef* TIMx = get_tim1_ptr(config->TIMx_base_addr);
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); // Disable counter while updating /* PDF Reference (CEN) */
            TIMx->PSC = psc;  /* PDF Reference (PSC) */
            TIMx->ARR = arr;  /* PDF Reference (ARR) */
            switch (config->ChannelNumber) {
                case 1: TIMx->CCR1 = ccr; break; /* PDF Reference (CCR1) */
                case 2: TIMx->CCR2 = ccr; break; /* PDF Reference (CCR2) */
                case 3: TIMx->CCR3 = ccr; break; /* PDF Reference (CCR3) */
                case 4: TIMx->CCR4 = ccr; break; /* PDF Reference (CCR4) */
            }
            // Generate an update event to apply new PSC, ARR, CCR values immediately
            BIT_SET(TIMx->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
            BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk); // Re-enable counter /* PDF Reference (CEN) */
            break;
        }
        case 2:
        case 3:
        case 4:
        case 5:
        {
            TIM_GP_TypeDef* TIMx = get_tim_gp_ptr(config->TIMx_base_addr);
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); // Disable counter while updating /* PDF Reference (CEN) */
            TIMx->PSC = psc;  /* PDF Reference (PSC) */
            TIMx->ARR = arr;  /* PDF Reference (ARR) */
            switch (config->ChannelNumber) {
                case 1: TIMx->CCR1 = ccr; break; /* PDF Reference (CCR1) */
                case 2: TIMx->CCR2 = ccr; break; /* PDF Reference (CCR2) */
                case 3: TIMx->CCR3 = ccr; break; /* PDF Reference (CCR3) */
                case 4: TIMx->CCR4 = ccr; break; /* PDF Reference (CCR4) */
            }
            // Generate an update event to apply new PSC, ARR, CCR values immediately
            BIT_SET(TIMx->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
            BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk); // Re-enable counter /* PDF Reference (CEN) */
            break;
        }
        case 9:
        {
            TIM9_TypeDef* TIMx = get_tim9_ptr(config->TIMx_base_addr);
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); // Disable counter while updating /* PDF Reference (CEN) */
            TIMx->PSC = psc;  /* PDF Reference (PSC) */
            TIMx->ARR = arr;  /* PDF Reference (ARR) */
            switch (config->ChannelNumber) {
                case 1: TIMx->CCR1 = ccr; break; /* PDF Reference (CCR1) */
                case 2: TIMx->CCR2 = ccr; break; /* PDF Reference (CCR2) */
            }
            // Generate an update event to apply new PSC, ARR, CCR values immediately
            BIT_SET(TIMx->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
            BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk); // Re-enable counter /* PDF Reference (CEN) */
            break;
        }
        case 10:
        case 11:
        {
            TIM10_11_TypeDef* TIMx = get_tim10_11_ptr(config->TIMx_base_addr);
            BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk); // Disable counter while updating /* PDF Reference (CEN) */
            TIMx->PSC = psc;  /* PDF Reference (PSC) */
            TIMx->ARR = arr;  /* PDF Reference (ARR) */
            TIMx->CCR1 = ccr; /* PDF Reference (CCR1) */
            // Generate an update event to apply new PSC, ARR, CCR values immediately
            BIT_SET(TIMx->EGR, TIM_EGR_UG_Msk);  /* PDF Reference (UG) */
            BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk); // Re-enable counter /* PDF Reference (CEN) */
            break;
        }
        default:
            return; // Should not happen
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    if (config->TimerID == 1)
    {
        TIM1_TypeDef* TIMx = get_tim1_ptr(config->TIMx_base_addr);
        BIT_SET(TIMx->BDTR, TIM_BDTR_MOE_Msk); // Enable Main Output (MOE) for TIM1 /* PDF Reference (MOE) */
        BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk);   // Enable counter /* PDF Reference (CEN) */
    }
    else if (config->TimerID >= 2 && config->TimerID <= 5)
    {
        TIM_GP_TypeDef* TIMx = get_tim_gp_ptr(config->TIMx_base_addr);
        BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk);   // Enable counter /* PDF Reference (CEN) */
    }
    else if (config->TimerID == 9)
    {
        TIM9_TypeDef* TIMx = get_tim9_ptr(config->TIMx_base_addr);
        BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk);   // Enable counter /* PDF Reference (CEN) */
    }
    else if (config->TimerID == 10 || config->TimerID == 11)
    {
        TIM10_11_TypeDef* TIMx = get_tim10_11_ptr(config->TIMx_base_addr);
        BIT_SET(TIMx->CR1, TIM_CR1_CEN_Msk);   // Enable counter /* PDF Reference (CEN) */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    if (config->TimerID == 1)
    {
        TIM1_TypeDef* TIMx = get_tim1_ptr(config->TIMx_base_addr);
        BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk);   // Disable counter /* PDF Reference (CEN) */
        BIT_CLR(TIMx->BDTR, TIM_BDTR_MOE_Msk); // Disable Main Output (MOE) for TIM1 /* PDF Reference (MOE) */
    }
    else if (config->TimerID >= 2 && config->TimerID <= 5)
    {
        TIM_GP_TypeDef* TIMx = get_tim_gp_ptr(config->TIMx_base_addr);
        BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk);   // Disable counter /* PDF Reference (CEN) */
    }
    else if (config->TimerID == 9)
    {
        TIM9_TypeDef* TIMx = get_tim9_ptr(config->TIMx_base_addr);
        BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk);   // Disable counter /* PDF Reference (CEN) */
    }
    else if (config->TimerID == 10 || config->TimerID == 11)
    {
        TIM10_11_TypeDef* TIMx = get_tim10_11_ptr(config->TIMx_base_addr);
        BIT_CLR(TIMx->CR1, TIM_CR1_CEN_Msk);   // Disable counter /* PDF Reference (CEN) */
    }
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all configured PWM channels, stops them,
 *        and disables their respective timer clocks. It does NOT disable GPIO clocks
 *        as they might be used by other peripherals.
 */
void PWM_PowerOff(void)
{
    for (tbyte i = 0; i < TRD_CHANNEL_MAX; i++)
    {
        const PWM_Channel_Config_t* config = get_channel_config((TRD_Channel_t)i);
        if (config != NULL)
        {
            PWM_Stop((TRD_Channel_t)i); // Stop the PWM signal
            disable_timer_clock(config->TimerID); // Disable the timer peripheral clock
        }
    }
}