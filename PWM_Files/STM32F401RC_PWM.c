/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file provides the production-ready implementation for PWM functionalities
*                  on STM32F401RC microcontroller, strictly following the specified requirements.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-22
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include <stdint.h> // For uint32_t, uint8_t

// Dummy definitions for types and enums from STM32F401RC_PWM.h
// In a real project, these would be in a dedicated header file (e.g., STM32F401RC_PWM.h and common types.h).
typedef uint32_t tlong;
typedef uint8_t tbyte;

// Define TRD_Channel_t based on the pwm_channel_map size for enumeration.
typedef enum {
    TRD_TIM1_CH1,
    TRD_TIM1_CH2,
    TRD_TIM1_CH3,
    TRD_TIM1_CH4,
    TRD_TIM3_CH1,
    TRD_TIM3_CH2,
    TRD_TIM3_CH3,
    TRD_TIM3_CH4,
    TRD_TIM4_CH1,
    TRD_TIM4_CH2,
    TRD_TIM4_CH3,
    TRD_TIM4_CH4,
    TRD_TIM9_CH1,
    TRD_TIM9_CH2,
    TRD_CHANNEL_COUNT // Must be the last element to get the count
} TRD_Channel_t;

/*
 * @brief  Dummy WDT_Reset function.
 *         In a real production environment, this would interface with a Watchdog Timer service
 *         to prevent system resets due to inactivity or infinite loops.
 */
void WDT_Reset(void) {
    // Implement actual WDT reset logic here in a production environment.
    // For this example, it's a placeholder.
}

/*
 * @brief  Bare-metal Peripheral Register Definitions for STM32F401RC.
 *         These definitions are based on the STM32F401RC Reference Manual (RM0368 Rev 5).
 *         In a typical project, these would be provided by CMSIS/HAL headers (e.g., stm32f4xx.h).
 */

// Base addresses (refer to RM0368, Section 3.3 Memory Map)
#define PERIPH_BASE           (0x40000000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)

// RCC Base Address (refer to RM0368, Section 3.3 Memory Map, AHB1 Bus)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)

// GPIO Base Addresses (refer to RM0368, Section 3.3 Memory Map, AHB1 Bus)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)

// Timer Base Addresses (refer to RM0368, Section 3.3 Memory Map)
// APB2 Timers
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

// APB1 Timers
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
// TIM2 and TIM5 are reserved as per requirements.

// Peripheral Register Structures (simplified from common CMSIS definitions)
// (refer to RM0368, sections 8.4 for GPIO, 12.4 for TIM1, 13.4 for TIM2-5, 14.4/14.5 for TIM9-11)

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR; // Offset 0x30
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR; // Offset 0x40
    volatile uint32_t APB2ENR; // Offset 0x44
    // ... other RCC registers beyond the scope of this file
} RCC_TypeDef;

typedef struct {
    volatile uint32_t MODER;    // Offset 0x00
    volatile uint32_t OTYPER;   // Offset 0x04
    volatile uint32_t OSPEEDR;  // Offset 0x08
    volatile uint32_t PUPDR;    // Offset 0x0C
    volatile uint32_t IDR;      // Offset 0x10
    volatile uint32_t ODR;      // Offset 0x14
    volatile uint32_t BSRR;     // Offset 0x18
    volatile uint32_t LCKR;     // Offset 0x1C
    volatile uint32_t AFRL;     // Offset 0x20 (AFR[0] in RM)
    volatile uint32_t AFRH;     // Offset 0x24 (AFR[1] in RM)
} GPIO_TypeDef;

// Generic TIM_TypeDef for general-purpose timers.
// Advanced-control TIM1 has RCR and BDTR. Other timers may have OR register.
// To handle this generically for bare-metal, we cast to the base address
// and access the registers by their known offsets.
// However, using a struct for clarity:
typedef struct {
    volatile uint32_t CR1;      // Offset 0x00
    volatile uint32_t CR2;      // Offset 0x04
    volatile uint32_t SMCR;     // Offset 0x08
    volatile uint32_t DIER;     // Offset 0x0C
    volatile uint32_t SR;       // Offset 0x10
    volatile uint32_t EGR;      // Offset 0x14
    volatile uint32_t CCMR1;    // Offset 0x18
    volatile uint32_t CCMR2;    // Offset 0x1C (Only for TIMs with >2 channels)
    volatile uint32_t CCER;     // Offset 0x20
    volatile uint32_t CNT;      // Offset 0x24
    volatile uint32_t PSC;      // Offset 0x28
    volatile uint32_t ARR;      // Offset 0x2C
    volatile uint32_t RCR;      // Offset 0x30 (TIM1 only, reserved for others)
    volatile uint32_t CCR1;     // Offset 0x34
    volatile uint32_t CCR2;     // Offset 0x38
    volatile uint32_t CCR3;     // Offset 0x3C
    volatile uint32_t CCR4;     // Offset 0x40
    volatile uint32_t BDTR;     // Offset 0x44 (TIM1 only, reserved for others)
    volatile uint32_t DCR;      // Offset 0x48
    volatile uint32_t DMAR;     // Offset 0x4C
    volatile uint32_t OR;       // Offset 0x50 (TIM2, TIM5, TIM11 only)
} TIM_TypeDef;

// Peripheral Pointers
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

// RCC Peripheral Clock Enable Register Bits (refer to RM0368, Section 6.3 Clock Control Register)
#define RCC_AHB1ENR_GPIOAEN   (1UL << 0)   // GPIOA clock enable
#define RCC_AHB1ENR_GPIOBEN   (1UL << 1)   // GPIOB clock enable
#define RCC_AHB1ENR_GPIOCEN   (1UL << 2)   // GPIOC clock enable

#define RCC_APB1ENR_TIM3EN    (1UL << 1)   // TIM3 clock enable
#define RCC_APB1ENR_TIM4EN    (1UL << 2)   // TIM4 clock enable

#define RCC_APB2ENR_TIM1EN    (1UL << 0)   // TIM1 clock enable
#define RCC_APB2ENR_TIM9EN    (1UL << 16)  // TIM9 clock enable
#define RCC_APB2ENR_TIM10EN   (1UL << 17)  // TIM10 clock enable
#define RCC_APB2ENR_TIM11EN   (1UL << 18)  // TIM11 clock enable

// GPIO Register Bit Masks and Values (refer to RM0368, Section 8.4 GPIO Registers)
#define GPIO_MODER_AF_MODE    (0x2UL)      // Alternate function mode (MODER bits 10)
#define GPIO_OTYPER_PUSHPULL  (0x0UL)      // Output push-pull (OTYPER bit 0)
#define GPIO_OTYPER_OPENDRAIN (0x1UL)      // Output open-drain (OTYPER bit 1) /* Corrected/Added */
#define GPIO_OSPEEDR_VERY_HIGH (0x3UL)     // Very high speed (OSPEEDR bits 11)
#define GPIO_PUPDR_NOPULL     (0x0UL)      // No pull-up, pull-down (PUPDR bits 00)

// Timer Control Register 1 (CR1) bits (refer to RM0368, Section 12.4.1 / 13.4.1 / 14.4.1 / 14.5.1)
#define TIM_CR1_CEN_Pos       (0U)
#define TIM_CR1_CEN           (1UL << TIM_CR1_CEN_Pos)     // Counter enable
#define TIM_CR1_UDIS_Pos      (1U)
#define TIM_CR1_UDIS          (1UL << TIM_CR1_UDIS_Pos)    // Update disable
#define TIM_CR1_URS_Pos       (2U)
#define TIM_CR1_URS           (1UL << TIM_CR1_URS_Pos)     // Update request source
#define TIM_CR1_OPM_Pos       (3U)
#define TIM_CR1_OPM           (1UL << TIM_CR1_OPM_Pos)     // One pulse mode
#define TIM_CR1_DIR_Pos       (4U)    /* Added */
#define TIM_CR1_DIR           (1UL << TIM_CR1_DIR_Pos)     /* Added */ // Counter direction (0=Up, 1=Down)
#define TIM_CR1_CMS_Pos       (5U)
#define TIM_CR1_CMS_MASK      (0x3UL << TIM_CR1_CMS_Pos)   /* Added */ // Center-aligned mode selection mask
#define TIM_CR1_CMS_EDGE      (0x0UL << TIM_CR1_CMS_Pos)   // Edge-aligned mode
#define TIM_CR1_ARPE_Pos      (7U)
#define TIM_CR1_ARPE          (1UL << TIM_CR1_ARPE_Pos)    // Auto-reload preload enable

// Timer Capture/Compare Mode Register (CCMRx) bits (refer to RM0368, Section 12.4.7/8 etc.)
// OCxM: Output Compare mode. Bits 6:4 or 14:12 for CCMR1, etc.
#define TIM_CCMR_OC_MODE_PWM1 (0x6UL)    // 110: PWM mode 1
#define TIM_CCMR_OC_MODE_PWM2 (0x7UL)    // 111: PWM mode 2
// OCxPE: Output Compare preload enable. Bit 3 or 11.
#define TIM_CCMR_OCxPE_Pos    (3U)
#define TIM_CCMR_OCxPE        (1UL << TIM_CCMR_OCxPE_Pos)
// OCxFE: Output Compare fast enable. Bit 2 or 10.
#define TIM_CCMR_OCxFE_Pos    (2U)
#define TIM_CCMR_OCxFE        (1UL << TIM_CCMR_OCxFE_Pos)
// CCxS: Capture/Compare selection. Bits 1:0 or 9:8. Must be 00 for output.
#define TIM_CCMR_CCxS_OUTPUT_Pos (0U)
#define TIM_CCMR_CCxS_OUTPUT     (0x0UL << TIM_CCMR_CCxS_OUTPUT_Pos)

// Timer Capture/Compare Enable Register (CCER) bits (refer to RM0368, Section 12.4.9 etc.)
#define TIM_CCER_CCxE_Pos     (0U)
#define TIM_CCER_CCxE         (1UL << TIM_CCER_CCxE_Pos)   // Enable CCx output
#define TIM_CCER_CCxP_Pos     (1U)
#define TIM_CCER_CCxP         (1UL << TIM_CCER_CCxP_Pos)   // OCx polarity (0=active high, 1=active low)

// Timer Event Generation Register (EGR) bits (refer to RM0368, Section 12.4.6 etc.)
#define TIM_EGR_UG_Pos        (0U)
#define TIM_EGR_UG            (1UL << TIM_EGR_UG_Pos)      // Update generation

// Timer Break and Dead-Time Register (BDTR) for TIM1 (refer to RM0368, Section 12.4.18)
#define TIM_BDTR_MOE_Pos      (15U)
#define TIM_BDTR_MOE          (1UL << TIM_BDTR_MOE_Pos)    // Main Output Enable

// System Clock Frequency (assumed for timer calculations)
// Max HCLK for F401RC is 84MHz. APB1 prescaler set to divide by 2, APB2 by 1.
// This results in PCLK1 = 42MHz, PCLK2 = 84MHz.
// Timers connected to APB1 (TIM3, TIM4) have their clock doubled if APB prescaler is > 1.
// Timers connected to APB2 (TIM1, TIM9, TIM10, TIM11) have their clock equal to PCLK2.
// Thus, all configured timers will effectively run at 84MHz. /* PDF Reference */
#define TIMER_CLOCK_FREQUENCY_HZ 84000000UL


/**
 * @brief Defines the configuration for a single PWM channel.
 *        This structure maps a logical TRD_Channel_t to its specific hardware components.
 */
typedef struct {
    TIM_TypeDef*    TIMx;           /**< Pointer to the Timer peripheral register base address */
    GPIO_TypeDef*   PortName;       /**< Pointer to the GPIO port register base address */
    uint8_t         PinNumber;      /**< GPIO pin number (0-15) */
    uint8_t         ChannelNumber;  /**< Timer channel number (1-4) */
    uint8_t         AlternateFunction; /**< GPIO Alternate Function (AF) number (0-15).
                                            E.g., AF1 for TIM1, AF2 for TIM3/TIM4, AF3 for TIM9/TIM10/TIM11.
                                            This value corresponds to the 'y' in GPIO_AFy_TIMx. */
} PWM_Channel_Config_t;


/**
 * @brief Array mapping TRD_Channel_t enumeration to specific hardware configurations.
 *        Each entry defines a unique PWM output.
 *
 * @note  TIM2 and TIM5 are intentionally excluded from this mapping as per requirements,
 *        to be reserved for OS or delay purposes if the MCU lacks dedicated timers.
 *        This mapping aims for unique physical pin assignments for each listed PWM channel
 *        to avoid immediate hardware conflicts. Therefore, TIM10_CH1 and TIM11_CH1,
 *        which share pins (PB8, PB9) with TIM4_CH3 and TIM4_CH4 respectively on STM32F401RC,
 *        are not included in this production-ready map as TIM4 channels are prioritized
 *        due to their higher channel count and common usage.
 */
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // Timer 1 (Advanced-control timer), AF1
    {TIM1, GPIOA, 8,  1, 1}, // TIM1_CH1 on PA8   /* Assumed PWM config */
    {TIM1, GPIOA, 9,  2, 1}, // TIM1_CH2 on PA9   /* Assumed PWM config */
    {TIM1, GPIOA, 10, 3, 1}, // TIM1_CH3 on PA10  /* Assumed PWM config */
    {TIM1, GPIOA, 11, 4, 1}, // TIM1_CH4 on PA11  /* Assumed PWM config */

    // Timer 3 (General-purpose timer), AF2
    {TIM3, GPIOA, 6,  1, 2}, // TIM3_CH1 on PA6   /* Assumed PWM config */
    {TIM3, GPIOA, 7,  2, 2}, // TIM3_CH2 on PA7   /* Assumed PWM config */
    {TIM3, GPIOC, 8,  3, 2}, // TIM3_CH3 on PC8 (PB0 excluded as per requirement) /* Assumed PWM config */
    {TIM3, GPIOB, 1,  4, 2}, // TIM3_CH4 on PB1   /* Assumed PWM config */

    // Timer 4 (General-purpose timer), AF2
    {TIM4, GPIOB, 6,  1, 2}, // TIM4_CH1 on PB6   /* Assumed PWM config */
    {TIM4, GPIOB, 7,  2, 2}, // TIM4_CH2 on PB7   /* Assumed PWM config */
    {TIM4, GPIOB, 8,  3, 2}, // TIM4_CH3 on PB8 (Prioritized over TIM10_CH1) /* Assumed PWM config */
    {TIM4, GPIOB, 9,  4, 2}, // TIM4_CH4 on PB9 (Prioritized over TIM11_CH1) /* Assumed PWM config */

    // Timer 9 (General-purpose timer), AF3
    {TIM9, GPIOA, 2,  1, 3}, // TIM9_CH1 on PA2   /* Assumed PWM config */
    {TIM9, GPIOA, 3,  2, 3}, // TIM9_CH2 on PA3   /* Assumed PWM config */
    // Note: TIM10_CH1 (PB8 AF3) and TIM11_CH1 (PB9 AF3) are not included to avoid
    // direct physical pin conflicts with TIM4_CH3 and TIM4_CH4 respectively.
    // If these channels are required, users must manage pin multiplexing externally.
};


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 *        Configures the timer and GPIOs.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    WDT_Reset(); // Call WDT_Reset at the beginning of the function

    if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel, handle error or return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // 1. Enable Timer Clock
    if (config->TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* PDF Reference */
    } else if (config->TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* PDF Reference */
    } else if (config->TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* PDF Reference */
    } else if (config->TIMx == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* PDF Reference */
    } else if (config->TIMx == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* PDF Reference */
    } else if (config->TIMx == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* PDF Reference */
    } else {
        // Should not happen with current pwm_channel_map
        return; 
    }

    // 2. Enable GPIO Port Clock
    if (config->PortName == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* PDF Reference */
    } else if (config->PortName == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* PDF Reference */
    } else if (config->PortName == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* PDF Reference */
    } else {
        // Should not happen with current pwm_channel_map
        return; 
    }

    // 3. Configure GPIO Pin for Alternate Function (AF)
    uint32_t pin_pos = config->PinNumber;
    uint32_t af_value = config->AlternateFunction;

    // Clear and set MODER bits for Alternate Function mode (MODERy[1:0] = 10b)
    config->PortName->MODER &= ~(0x3UL << (pin_pos * 2)); /* PDF Reference */
    config->PortName->MODER |= (GPIO_MODER_AF_MODE << (pin_pos * 2)); /* PDF Reference */

    // Clear OTYPER bit for Push-pull output type (OTy = 0b).
    // OTYPER bit 0 is for Push-pull, 1 for Open-drain. Clearing ensures push-pull.
    config->PortName->OTYPER &= ~(GPIO_OTYPER_OPENDRAIN << pin_pos); /* PDF Reference - Corrected logic */

    // Clear and set OSPEEDR bits for Very High speed (OSPEEDRy[1:0] = 11b)
    config->PortName->OSPEEDR &= ~(0x3UL << (pin_pos * 2)); /* PDF Reference */
    config->PortName->OSPEEDR |= (GPIO_OSPEEDR_VERY_HIGH << (pin_pos * 2)); /* PDF Reference */

    // Clear PUPDR bits for No pull-up/pull-down (PUPDRy[1:0] = 00b)
    // Clearing both bits (0x3UL) sets them to 00b, which is no pull.
    config->PortName->PUPDR &= ~(0x3UL << (pin_pos * 2)); /* PDF Reference */

    // Configure Alternate Function register (AFRL for pins 0-7, AFRH for pins 8-15)
    if (pin_pos < 8) { // AFRL register
        config->PortName->AFRL &= ~(0xFUL << (pin_pos * 4)); /* PDF Reference */
        config->PortName->AFRL |= (af_value << (pin_pos * 4)); /* PDF Reference */
    } else { // AFRH register
        config->PortName->AFRH &= ~(0xFUL << ((pin_pos - 8) * 4)); /* PDF Reference */
        config->PortName->AFRH |= (af_value << ((pin_pos - 8) * 4)); /* PDF Reference */
    }

    // 4. Configure Timer for PWM Mode
    // Disable counter before configuring
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // Configure Auto-reload preload enable (ARPE = 1)
    config->TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference */

    // Select Edge-aligned mode (CMS = 00) and Upcounting mode (DIR = 0)
    // Clear CMS bits (5:6) and DIR bit (4) to ensure up-counting, edge-aligned mode.
    config->TIMx->CR1 &= ~(TIM_CR1_CMS_MASK | TIM_CR1_DIR); /* PDF Reference - Corrected */

    // Select PWM Mode 1 (OCxM = 110)
    // Enable output compare preload (OCxPE = 1)
    // Enable output compare fast enable (OCxFE = 1) - Crucial for fast PWM response. /* Added */
    // Clear CCxS bits to ensure channel is configured as output (CCxS = 00)
    uint32_t ccmr_val = 0;
    uint32_t ccmr_clear_mask = 0;

    if (config->ChannelNumber == 1) {
        ccmr_val = (TIM_CCMR_OC_MODE_PWM1 << 4) | TIM_CCMR_OCxPE | TIM_CCMR_OCxFE; /* PDF Reference - Corrected */
        ccmr_clear_mask = (0x7UL << 4) | (0x3UL << 0) | TIM_CCMR_OCxPE | TIM_CCMR_OCxFE; // Clear OC1M, CC1S, OC1PE, OC1FE
        config->TIMx->CCMR1 &= ~ccmr_clear_mask;
        config->TIMx->CCMR1 |= ccmr_val;
    } else if (config->ChannelNumber == 2) {
        ccmr_val = (TIM_CCMR_OC_MODE_PWM1 << 12) | (TIM_CCMR_OCxPE << 8) | (TIM_CCMR_OCxFE << 8); /* PDF Reference - Corrected */
        ccmr_clear_mask = (0x7UL << 12) | (0x3UL << 8) | (TIM_CCMR_OCxPE << 8) | (TIM_CCMR_OCxFE << 8); // Clear OC2M, CC2S, OC2PE, OC2FE
        config->TIMx->CCMR1 &= ~ccmr_clear_mask;
        config->TIMx->CCMR1 |= ccmr_val;
    } else if (config->ChannelNumber == 3) { // TIM1, TIM3, TIM4
        ccmr_val = (TIM_CCMR_OC_MODE_PWM1 << 4) | TIM_CCMR_OCxPE | TIM_CCMR_OCxFE; /* PDF Reference - Corrected */
        ccmr_clear_mask = (0x7UL << 4) | (0x3UL << 0) | TIM_CCMR_OCxPE | TIM_CCMR_OCxFE; // Clear OC3M, CC3S, OC3PE, OC3FE
        config->TIMx->CCMR2 &= ~ccmr_clear_mask;
        config->TIMx->CCMR2 |= ccmr_val;
    } else if (config->ChannelNumber == 4) { // TIM1, TIM3, TIM4
        ccmr_val = (TIM_CCMR_OC_MODE_PWM1 << 12) | (TIM_CCMR_OCxPE << 8) | (TIM_CCMR_OCxFE << 8); /* PDF Reference - Corrected */
        ccmr_clear_mask = (0x7UL << 12) | (0x3UL << 8) | (TIM_CCMR_OCxPE << 8) | (TIM_CCMR_OCxFE << 8); // Clear OC4M, CC4S, OC4PE, OC4FE
        config->TIMx->CCMR2 &= ~ccmr_clear_mask;
        config->TIMx->CCMR2 |= ccmr_val;
    }

    // Generate an update event to load preload registers (UG = 1)
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */

    // Enable the Capture/Compare Output for the selected channel (CCxE = 1)
    // Set output polarity to active high (CCxP = 0)
    uint32_t ccer_mask = 0x3UL << ((config->ChannelNumber - 1) * 4); // CCxE and CCxP bits
    uint32_t ccer_val_enable = TIM_CCER_CCxE << ((config->ChannelNumber - 1) * 4); // Set CCxE
    config->TIMx->CCER &= ~ccer_mask; // Clear existing bits (CCxE and CCxP)
    config->TIMx->CCER |= ccer_val_enable; // Set new bits for enable and default active high /* PDF Reference */

    // For Advanced-control Timer (TIM1), enable Main Output (MOE)
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    WDT_Reset(); // Call WDT_Reset at the beginning of the function

    if (TRD_Channel >= TRD_CHANNEL_COUNT || frequency == 0 || duty > 100) {
        // Invalid channel or parameters, handle error
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Calculate ARR and PSC values
    // Timer_CLK_Frequency = TIMER_CLOCK_FREQUENCY_HZ (e.g., 84MHz) /* PDF Reference */
    // ARR = (Timer_CLK_Frequency / (Prescaler + 1) / Frequency) - 1
    // We want to maximize resolution, so start with minimum Prescaler (0)
    tlong timer_clock = TIMER_CLOCK_FREQUENCY_HZ;
    tlong period_counts = timer_clock / frequency;

    uint16_t prescaler = 0;
    uint16_t arr = (uint16_t)(period_counts - 1); // For 16-bit timers

    // If target period exceeds 16-bit, increase prescaler
    if (period_counts > 0xFFFF) { // Check if ARR + 1 exceeds 16-bit max (65536 counts)
        // Calculate minimum prescaler needed
        prescaler = (uint16_t)((period_counts / 0xFFFF) + ( (period_counts % 0xFFFF != 0) ? 1 : 0) - 1); /* PDF Reference */
        // Ensure prescaler does not exceed 16-bit max
        if (prescaler > 0xFFFF) { 
            prescaler = 0xFFFF; /* PDF Reference */
        }
        // Recalculate ARR with the determined prescaler
        arr = (uint16_t)((timer_clock / (prescaler + 1) / frequency) - 1); /* PDF Reference */
        if (arr > 0xFFFF) arr = 0xFFFF; // Should not happen if initial check was correct
    }

    // Ensure calculated ARR is at least 1 (period_counts >= 2)
    // An ARR of 0 means a period of 1. A period of 1 is valid, but setting ARR=1 ensures at least 2 counts (0 to 1).
    if (arr == 0) arr = 1;

    // Set Prescaler (PSC) and Auto-Reload Register (ARR)
    config->TIMx->PSC = prescaler; /* PDF Reference */
    config->TIMx->ARR = arr;       /* PDF Reference */

    // Calculate Capture/Compare Register (CCRx) value
    // CCRx = (ARR + 1) * Duty_Cycle / 100
    // Ensure accurate integer division by converting to tlong for calculation
    uint16_t ccr_val = (uint16_t)(((tlong)(arr + 1) * duty) / 100); /* PDF Reference */
    // Ensure CCRx is within valid range (0 to ARR)
    if (ccr_val > arr) ccr_val = arr;

    // Set CCRx based on channel number
    if (config->ChannelNumber == 1) {
        config->TIMx->CCR1 = ccr_val; /* PDF Reference */
    } else if (config->ChannelNumber == 2) {
        config->TIMx->CCR2 = ccr_val; /* PDF Reference */
    } else if (config->ChannelNumber == 3) {
        config->TIMx->CCR3 = ccr_val; /* PDF Reference */
    } else if (config->ChannelNumber == 4) {
        config->TIMx->CCR4 = ccr_val; /* PDF Reference */
    }

    // Generate an update event to apply new PSC/ARR/CCRx values immediately
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    WDT_Reset(); // Call WDT_Reset at the beginning of the function

    if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel, handle error or return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the counter (CEN bit in CR1)
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference */

    // For Advanced-control Timer (TIM1), ensure Main Output is enabled
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    WDT_Reset(); // Call WDT_Reset at the beginning of the function

    if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel, handle error or return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the Capture/Compare Output for the selected channel (CCxE = 0)
    uint32_t ccer_mask = TIM_CCER_CCxE << ((config->ChannelNumber - 1) * 4);
    config->TIMx->CCER &= ~ccer_mask; /* PDF Reference */

    // Set duty cycle to 0 to ensure output is low (or inactive level)
    if (config->ChannelNumber == 1) {
        config->TIMx->CCR1 = 0; /* PDF Reference */
    } else if (config->ChannelNumber == 2) {
        config->TIMx->CCR2 = 0; /* PDF Reference */
    } else if (config->ChannelNumber == 3) {
        config->TIMx->CCR3 = 0; /* PDF Reference */
    } else if (config->ChannelNumber == 4) {
        config->TIMx->CCR4 = 0; /* PDF Reference */
    }

    // For Advanced-control Timer (TIM1), disabling Main Output (MOE) here
    // should only be done if no other channels on TIM1 are active, or if the
    // intention is to completely disable the timer's output stage.
    // As per requirement to stop "specified channel", only disabling CCxE is sufficient
    // without affecting other channels sharing the same TIM1 instance.
    // If a full TIM1 output stage shutdown is desired when the last channel stops:
    // if (config->TIMx == TIM1 && /* no other channels on TIM1 are active */) {
    //     config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
    // }

    // Optionally stop the counter if no other channels on this timer are active
    // This is more of a timer-level stop than a channel-level stop, keep separate.
    // config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all defined PWM channels and ensures their outputs are off,
 *        then disables the clock to the respective timers and GPIO ports.
 */
void PWM_PowerOff(void) {
    WDT_Reset(); // Call WDT_Reset at the beginning of the function

    // Iterate through all possible PWM channels and disable them
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_COUNT; i++) {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];

        // Disable the counter (CEN bit in CR1) for the current timer.
        // This is done per channel, which might be redundant for timers with multiple active channels,
        // but harmless as the timer will be fully disabled later by clock gate.
        config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

        // Disable the Capture/Compare Output for the selected channel (CCxE = 0)
        uint32_t ccer_mask = TIM_CCER_CCxE << ((config->ChannelNumber - 1) * 4);
        config->TIMx->CCER &= ~ccer_mask; /* PDF Reference */

        // Set duty cycle to 0 to ensure output is low (or inactive level)
        if (config->ChannelNumber == 1) {
            config->TIMx->CCR1 = 0; /* PDF Reference */
        } else if (config->ChannelNumber == 2) {
            config->TIMx->CCR2 = 0; /* PDF Reference */
        } else if (config->ChannelNumber == 3) {
            config->TIMx->CCR3 = 0; /* PDF Reference */
        } else if (config->ChannelNumber == 4) {
            config->TIMx->CCR4 = 0; /* PDF Reference */
        }

        // For Advanced-control Timer (TIM1), disable Main Output (MOE)
        if (config->TIMx == TIM1) {
            config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
        }
    }

    // Disable all used Timer Clocks
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); /* PDF Reference */
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN); /* PDF Reference */

    // Disable all used GPIO Port Clocks
    // Note: This disables GPIO clock for ALL pins on the port, not just PWM.
    // If other GPIOs on the same port are in use, this should be handled carefully
    // by a higher-level power management unit or selective clock disablement.
    // For 'all PWM peripherals and outputs', this is a direct interpretation.
    RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN); /* PDF Reference */
}