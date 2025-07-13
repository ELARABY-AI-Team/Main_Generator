/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include <stdint.h> // For uint32_t, uint8_t

// Note: Timers TIM10 and TIM11 are reserved for potential OS or delay purposes
// based on the requirement to reserve at least 2 timers + channels if no dedicated ones are available.
// These timers are not used for PWM generation in this implementation.

// Need to define this based on the target board/system clock configuration
// Assuming timers on APB1 have clock 84MHz and timers on APB2 have clock 168MHz
// (This is a common configuration for STM32F401 running at 84MHz system clock due to APB prescaler > 1)
#define TIM_APB1_CLOCK 84000000UL // Assumed timer clock for APB1 timers /* Assumed clock frequency - please verify */
#define TIM_APB2_CLOCK 168000000UL // Assumed timer clock for APB2 timers /* Assumed clock frequency - please verify */


// Placeholder for GPIO and Timer base addresses and RCC peripheral clock enable registers
// These base addresses are typically found in the device header files (e.g., stm32f4xx.h)
// but are defined here based on the RM memory map for bare-metal access derived from PDF context.
#define GPIOA_BASE 0x40020000UL // From RM Sec 2.3 /* Derived from PDF context (Memory Map) */
#define GPIOB_BASE 0x40020400UL // From RM Sec 2.3 /* Derived from PDF context (Memory Map) */
#define GPIOC_BASE 0x40020800UL // From RM Sec 2.3 /* Derived from PDF context (Memory Map) */
// ... Add other GPIO ports if needed based on channel assignments

#define TIM1_BASE  0x40010000UL // From RM Sec 2.3 (APB2) /* Derived from PDF context (Memory Map) */
#define TIM2_BASE  0x40000000UL // From RM Sec 2.3 (APB1) /* Derived from PDF context (Memory Map) */
#define TIM3_BASE  0x40000400UL // From RM Sec 2.3 (APB1) /* Derived from PDF context (Memory Map) */
#define TIM4_BASE  0x40000800UL // From RM Sec 2.3 (APB1) /* Derived from PDF context (Memory Map) */
#define TIM5_BASE  0x40000C00UL // From RM Sec 2.3 (APB1) /* Derived from PDF context (Memory Map) */
#define TIM9_BASE  0x40014000UL // From RM Sec 2.3 (APB2) /* Derived from PDF context (Memory Map) */
// TIM10_BASE and TIM11_BASE are defined but reserved /* Derived from PDF context (Memory Map) */
#define TIM10_BASE 0x40014400UL
#define TIM11_BASE 0x40014800UL


// RCC Peripheral Clock Enable Registers (Assumed locations based on RM structure)
// RCC registers are critical for enabling clocks but not detailed in the provided PDF sections for GPIO/TIM.
// This is a necessary assumption for a functional PWM implementation.
#define RCC_BASE       0x40023800UL // Assumed RCC base address /* Assumed RCC base address - please verify */
#define RCC_AHB1ENR_R *(volatile uint32_t *)(RCC_BASE + 0x30UL) // AHB1 Peripheral Clock Enable Register offset /* Assumed RCC register offset - please verify */
#define RCC_APB1ENR_R *(volatile uint32_t *)(RCC_BASE + 0x40UL) // APB1 Peripheral Clock Enable Register offset /* Assumed RCC register offset - please verify */
#define RCC_APB2ENR_R *(volatile uint32_t *)(RCC_BASE + 0x44UL) // APB2 Peripheral Clock Enable Register offset /* Assumed RCC register offset - please verify */


// GPIO Register Offsets (from provided PDF Section 8.4)
#define GPIO_MODER_OFFSET 0x00UL // GPIO port mode register /* PDF Reference */
#define GPIO_OTYPER_OFFSET 0x04UL // GPIO port output type register /* PDF Reference */
#define GPIO_OSPEEDR_OFFSET 0x08UL // GPIO port output speed register /* PDF Reference */
#define GPIO_PUPDR_OFFSET 0x0CUL // GPIO port pull-up/pull-down register /* PDF Reference */
#define GPIO_AFRL_OFFSET 0x20UL // GPIO alternate function low register /* PDF Reference */
#define GPIO_AFRH_OFFSET 0x24UL // GPIO alternate function high register /* PDF Reference */


// Timer Register Offsets (from provided PDF Sections 12.4, 13.4, 14.4, 14.5)
// These offsets are consistent across the General and Advanced timers in the provided text.
#define TIM_CR1_OFFSET   0x00UL // TIMx control register 1 /* PDF Reference */
#define TIM_CR2_OFFSET   0x04UL // TIMx control register 2 /* PDF Reference */
#define TIM_SMCR_OFFSET  0x08UL // TIMx slave mode control register /* PDF Reference */
#define TIM_DIER_OFFSET  0x0CUL // TIMx DMA/interrupt enable register /* PDF Reference */
#define TIM_SR_OFFSET    0x10UL // TIMx status register /* PDF Reference */
#define TIM_EGR_OFFSET   0x14UL // TIMx event generation register /* PDF Reference */
#define TIM_CCMR1_OFFSET 0x18UL // TIMx capture/compare mode register 1 /* PDF Reference */
#define TIM_CCMR2_OFFSET 0x1CUL // TIMx capture/compare mode register 2 /* PDF Reference */
#define TIM_CCER_OFFSET  0x20UL // TIMx capture/compare enable register /* PDF Reference */
#define TIM_CNT_OFFSET   0x24UL // TIMx counter /* PDF Reference */
#define TIM_PSC_OFFSET   0x28UL // TIMx prescaler /* PDF Reference */
#define TIM_ARR_OFFSET   0x2CUL // TIMx auto-reload register /* PDF Reference */
#define TIM_RCR_OFFSET   0x30UL // TIMx repetition counter register /* PDF Reference */
#define TIM_CCR1_OFFSET  0x34UL // TIMx capture/compare register 1 /* PDF Reference */
#define TIM_CCR2_OFFSET  0x38UL // TIMx capture/compare register 2 /* PDF Reference */
#define TIM_CCR3_OFFSET  0x3CUL // TIMx capture/compare register 3 /* PDF Reference */
#define TIM_CCR4_OFFSET  0x40UL // TIMx capture/compare register 4 /* PDF Reference */
#define TIM_BDTR_OFFSET  0x44UL // TIM1 break and dead-time register /* PDF Reference */
#define TIM_DCR_OFFSET   0x48UL // TIMx DMA control register /* PDF Reference */
#define TIM_DMAR_OFFSET  0x4CUL // TIMx DMA address for full transfer /* PDF Reference */
// TIM2_OR_OFFSET 0x50, TIM5_OR_OFFSET 0x50, TIM11_OR_OFFSET 0x50 - not used for PWM /* PDF Reference */


// GPIO Alternate Function Values (from provided PDF Figure 17 implies mapping)
// These are AF numbers, specific pin mappings are assumptions.
#define GPIO_AF_TIM1_2  0x01UL // AF1 for TIM1/TIM2 /* PDF Reference (Figure 17) */
#define GPIO_AF_TIM3_5  0x02UL // AF2 for TIM3/TIM4/TIM5 /* PDF Reference (Figure 17) */
#define GPIO_AF_TIM9_11 0x03UL // AF3 for TIM9/TIM10/TIM11 /* PDF Reference (Figure 17) */


// Timer Channel Definitions (derived from register access in PDF)
#define TIM_CHANNEL_1 1UL
#define TIM_CHANNEL_2 2UL
#define TIM_CHANNEL_3 3UL
#define TIM_CHANNEL_4 4UL

// Structure to hold configuration for each channel
typedef struct
{
    uint32_t tim_base;
    uint32_t tim_clock_freq; // Clock frequency for this specific timer instance
    uint8_t tim_channel;
    uint32_t gpio_base;
    uint8_t gpio_pin;
    uint8_t gpio_af;
    // Indicates if this timer has 32-bit counter/ARR/CCRx registers.
    // TIM2, TIM5 are 32-bit. TIM1, TIM3, TIM4, TIM9 are 16-bit.
    uint8_t is_32bit_timer; // 1 for 32-bit, 0 for 16-bit
} PWM_Channel_Config_t;

// Array holding the configuration for each channel
// Note: Indices match TRD_Channel_t enum values (except 0 is invalid)
// Reserved TIM10 and TIM11 for OS/delay - not used for PWM.
// Using common GPIOs for demonstration based on typical mappings and available AFs.
// TIM1_CH1 on PA8 (AF1)
// TIM2_CH1 on PA0 (AF1), TIM2_CH2 on PA1 (AF1), TIM2_CH3 on PA2 (AF1), TIM2_CH4 on PA3 (AF1)
// TIM3_CH1 on PA6 (AF2), TIM3_CH2 on PA7 (AF2), TIM3_CH3 on PB0 (AF2), TIM3_CH4 on PB1 (AF2)
// TIM4_CH1 on PB6 (AF2), TIM4_CH2 on PB7 (AF2), TIM4_CH3 on PB8 (AF2), TIM4_CH4 on PB9 (AF2)
// TIM5_CH1 on PA0 (AF2) - Note: PA0 is also TIM2_CH1, requires application logic to avoid conflict.
// TIM5_CH2 on PA1 (AF2) - Note: PA1 is also TIM2_CH2.
// TIM5_CH3 on PA2 (AF2) - Note: PA2 is also TIM2_CH3 and TIM9_CH1.
// TIM5_CH4 on PA3 (AF2) - Note: PA3 is also TIM2_CH4 and TIM9_CH2.
// TIM9_CH1 on PA2 (AF3) - Note: PA2 is also TIM2_CH3 and TIM5_CH3.
// TIM9_CH2 on PA3 (AF3) - Note: PA3 is also TIM2_CH4 and TIM5_CH4.
static const PWM_Channel_Config_t Channel_Configs[TRD_PWM_CHANNEL_COUNT] =
{
    { 0, 0, 0, 0, 0, 0, 0 }, // TRD_PWM_CHANNEL_INVALID

    // TIM1 Channels (APB2 Timer - assumed 168MHz clock, 16-bit ARR/CCRx)
    { TIM1_BASE, TIM_APB2_CLOCK, TIM_CHANNEL_1, GPIOA_BASE, 8, GPIO_AF_TIM1_2, 0 }, /* Assumed PWM config - please verify */

    // TIM2 Channels (APB1 Timer - assumed 84MHz clock, 32-bit ARR/CCRx)
    { TIM2_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_1, GPIOA_BASE, 0, GPIO_AF_TIM1_2, 1 }, /* Assumed PWM config - please verify */
    { TIM2_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_2, GPIOA_BASE, 1, GPIO_AF_TIM1_2, 1 }, /* Assumed PWM config - please verify */
    { TIM2_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_3, GPIOA_BASE, 2, GPIO_AF_TIM1_2, 1 }, /* Assumed PWM config - please verify */
    { TIM2_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_4, GPIOA_BASE, 3, GPIO_AF_TIM1_2, 1 }, /* Assumed PWM config - please verify */

    // TIM3 Channels (APB1 Timer - assumed 84MHz clock, 16-bit ARR/CCRx)
    { TIM3_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_1, GPIOA_BASE, 6, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM3_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_2, GPIOA_BASE, 7, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM3_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_3, GPIOB_BASE, 0, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM3_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_4, GPIOB_BASE, 1, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */

    // TIM4 Channels (APB1 Timer - assumed 84MHz clock, 16-bit ARR/CCRx)
    { TIM4_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_1, GPIOB_BASE, 6, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM4_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_2, GPIOB_BASE, 7, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM4_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_3, GPIOB_BASE, 8, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */
    { TIM4_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_4, GPIOB_BASE, 9, GPIO_AF_TIM3_5, 0 }, /* Assumed PWM config - please verify */

    // TIM5 Channels (APB1 Timer - assumed 84MHz clock, 32-bit ARR/CCRx)
    { TIM5_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_1, GPIOA_BASE, 0, GPIO_AF_TIM3_5, 1 }, /* Assumed PWM config - please verify */
    { TIM5_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_2, GPIOA_BASE, 1, GPIO_AF_TIM3_5, 1 }, /* Assumed PWM config - please verify */
    { TIM5_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_3, GPIOA_BASE, 2, GPIO_AF_TIM3_5, 1 }, /* Assumed PWM config - please verify */
    { TIM5_BASE, TIM_APB1_CLOCK, TIM_CHANNEL_4, GPIOA_BASE, 3, GPIO_AF_TIM3_5, 1 }, /* Assumed PWM config - please verify */

    // TIM9 Channels (APB2 Timer - assumed 168MHz clock, 16-bit ARR/CCRx)
    { TIM9_BASE, TIM_APB2_CLOCK, TIM_CHANNEL_1, GPIOA_BASE, 2, GPIO_AF_TIM9_11, 0 }, /* Assumed PWM config - please verify */
    { TIM9_BASE, TIM_APB2_CLOCK, TIM_CHANNEL_2, GPIOA_BASE, 3, GPIO_AF_TIM9_11, 0 }, /* Assumed PWM config - please verify */
};

// Pointers to access GPIO registers based on base address
// Derived structure from register map and offsets in provided PDF Section 8.4.
typedef struct
{
    volatile uint32_t MODER;    // Offset 0x00 /* PDF Reference */
    volatile uint32_t OTYPER;   // Offset 0x04 /* PDF Reference */
    volatile uint32_t OSPEEDR;  // Offset 0x08 /* PDF Reference */
    volatile uint32_t PUPDR;    // Offset 0x0C /* PDF Reference */
    volatile uint32_t IDR;      // Offset 0x10 /* PDF Reference */
    volatile uint32_t ODR;      // Offset 0x14 /* PDF Reference */
    volatile uint32_t BSRR;     // Offset 0x18 /* PDF Reference */
    volatile uint32_t LCKR;     // Offset 0x1C /* PDF Reference */
    volatile uint32_t AFRL;     // Offset 0x20 /* PDF Reference */
    volatile uint32_t AFRH;     // Offset 0x24 /* PDF Reference */
} GPIO_TypeDef;

#define GPIO(BASE) ((GPIO_TypeDef *)(BASE)) /* Derived from PDF context (Register Map) */

// Pointers to access Timer registers based on base address
// Derived structure from register map and offsets in provided PDF Sections 12.4, 13.4, 14.4, 14.5.
// Note: Presence of registers like CR2, SMCR, RCR, CCR2-4, BDTR varies by timer (see PDF sections).
// Accessing reserved offsets will result in bus errors or unexpected behavior.
// but access should be guarded based on the specific timer type via config->tim_base.
typedef struct
{
    volatile uint32_t CR1;    // Offset 0x00 /* PDF Reference */
    volatile uint32_t CR2;    // Offset 0x04 (Reserved for TIM10/11) /* PDF Reference */
    volatile uint32_t SMCR;   // Offset 0x08 (Reserved for TIM10/11) /* PDF Reference */
    volatile uint32_t DIER;   // Offset 0x0C /* PDF Reference */
    volatile uint32_t SR;     // Offset 0x10 /* PDF Reference */
    volatile uint32_t EGR;    // Offset 0x14 /* PDF Reference */
    volatile uint32_t CCMR1;  // Offset 0x18 /* PDF Reference */
    volatile uint32_t CCMR2;  // Offset 0x1C (Reserved for TIM10/11) /* PDF Reference */
    volatile uint32_t CCER;   // Offset 0x20 /* PDF Reference */
    volatile uint32_t CNT;    // Offset 0x24 (Width varies by timer) /* PDF Reference */
    volatile uint32_t PSC;    // Offset 0x28 /* PDF Reference */
    volatile uint32_t ARR;    // Offset 0x2C (Width varies by timer) /* PDF Reference */
    volatile uint32_t RCR;    // Offset 0x30 (Reserved for TIM2-5, TIM9-11) /* PDF Reference */
    volatile uint32_t CCR1;   // Offset 0x34 (Width varies by timer) /* PDF Reference */
    volatile uint32_t CCR2;   // Offset 0x38 (Reserved for TIM10/11) /* PDF Reference */
    volatile uint32_t CCR3;   // Offset 0x3C (Reserved for TIM9-11) /* PDF Reference */
    volatile uint32_t CCR4;   // Offset 0x40 (Reserved for TIM9-11) /* PDF Reference */
    volatile uint32_t BDTR;   // Offset 0x44 (Reserved for TIM2-5, TIM9-11) /* PDF Reference */
    volatile uint32_t DCR;    // Offset 0x48 /* PDF Reference */
    volatile uint32_t DMAR;   // Offset 0x4C /* PDF Reference */
    // Note: OR register at 0x50 is only for TIM2, TIM5, TIM11 according to PDF. Not included in general struct.
} TIM_TypeDef;

#define TIM(BASE) ((TIM_TypeDef *)(BASE)) /* Derived from PDF context (Register Map) */


// Helper function to get configuration for a given channel
static const PWM_Channel_Config_t *Get_Channel_Config(TRD_Channel_t channel)
{
    if (channel >= TRD_PWM_CHANNEL_COUNT || channel == TRD_PWM_CHANNEL_INVALID)
    {
        return NULL; // Invalid channel
    }
    return &Channel_Configs[channel];
}

// Helper function to enable GPIO clock (Requires RCC knowledge not in provided PDF sections)
static void Enable_GPIO_Clock(uint32_t gpio_base)
{
    // Assumed RCC AHB1 enable register based on common STM32F4 structure.
    // This information is outside the scope of the provided PDF content for GPIO/TIM.
    /* Assumed RCC clock enabling - please verify */
    if (gpio_base == GPIOA_BASE)
    {
        RCC_AHB1ENR_R |= (1UL << 0); // Enable GPIOA clock
    }
    else if (gpio_base == GPIOB_BASE)
    {
        RCC_AHB1ENR_R |= (1UL << 1); // Enable GPIOB clock
    }
    else if (gpio_base == GPIOC_BASE)
    {
        RCC_AHB1ENR_R |= (1UL << 2); // Enable GPIOC clock
    }
    // Add other GPIO ports (D, E, H) if used in Channel_Configs
    // Note: GPIOH0 and GPIOH1 are special, but not used in the example configs above.
    // PDF Section 8.1 mentions GPIOx_MODER, etc for x=A..E and H.
}

// Helper function to enable Timer clock (Requires RCC knowledge not in provided PDF sections)
static void Enable_Timer_Clock(uint32_t tim_base)
{
    // Assumed RCC APB1/APB2 enable registers based on common STM32F4 structure.
    // This information is outside the scope of the provided PDF content for GPIO/TIM.
    /* Assumed RCC clock enabling - please verify */
    if (tim_base >= TIM2_BASE && tim_base <= TIM5_BASE)
    {
        // APB1 Timers TIM2, TIM3, TIM4, TIM5
        uint32_t tim_index = (tim_base - TIM2_BASE) / 0x400; // Calculate index (0-3)
        RCC_APB1ENR_R |= (1UL << tim_index); // Enable TIMx clock
    }
    else if (tim_base == TIM1_BASE)
    {
        // APB2 Timer TIM1
        RCC_APB2ENR_R |= (1UL << 0); // Enable TIM1 clock
    }
    else if (tim_base == TIM9_BASE)
    {
        // APB2 Timer TIM9
        RCC_APB2ENR_R |= (1UL << 16); // Enable TIM9 clock
    }
    // TIM10 and TIM11 are reserved and clocks are not enabled here.
}


/**Functions ===========================================================================*/

/**
 * @brief Initialize the PWM hardware and configure the timer and GPIOs for the given channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = Get_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        // Invalid channel requested. Handle or ignore as per application requirements.
        return;
    }

    GPIO_TypeDef *GPIOx = GPIO(config->gpio_base);
    TIM_TypeDef *TIMx = TIM(config->tim_base);
    uint8_t pin = config->gpio_pin;
    uint8_t channel = config->tim_channel;

    // 1. Enable clock for GPIO port
    Enable_GPIO_Clock(config->gpio_base); /* Assumed RCC clock enabling - please verify */

    // 2. Configure GPIO pin for Alternate Function (AF) mode
    // Clear mode bits for the pin (2 bits per pin) /* PDF Reference (GPIOx_MODER) */
    GPIOx->MODER &= ~(3UL << (pin * 2));
    // Set mode bits to Alternate Function (10) /* PDF Reference (GPIOx_MODER) */
    GPIOx->MODER |= (2UL << (pin * 2));

    // 3. Configure GPIO pin output type to Push-Pull (0) /* PDF Reference (GPIOx_OTYPER) */
    GPIOx->OTYPER &= ~(1UL << pin);

    // 4. Configure GPIO pin output speed (e.g., High speed 10) /* PDF Reference (GPIOx_OSPEEDR) */
    // Clear speed bits for the pin (2 bits per pin) /* PDF Reference (GPIOx_OSPEEDR) */
    GPIOx->OSPEEDR &= ~(3UL << (pin * 2));
    // Set speed bits to High speed (10) /* PDF Reference (GPIOx_OSPEEDR) */
    GPIOx->OSPEEDR |= (2UL << (pin * 2));

    // 5. Configure GPIO pin pull-up/pull-down to No pull-up/down (00) /* PDF Reference (GPIOx_PUPDR) */
    GPIOx->PUPDR &= ~(3UL << (pin * 2));

    // 6. Select the Alternate Function for the pin
    // Pins 0-7 use AFRL, Pins 8-15 use AFRH (4 bits per pin) /* PDF Reference (GPIOx_AFRL, GPIOx_AFRH) */
    if (pin < 8)
    {
        // Clear AF bits for the pin /* PDF Reference */
        GPIOx->AFRL &= ~(0xFUL << (pin * 4));
        // Set AF bits /* PDF Reference */
        GPIOx->AFRL |= ((uint32_t)config->gpio_af << (pin * 4));
    }
    else
    {
        // Clear AF bits for the pin /* PDF Reference */
        GPIOx->AFRH &= ~(0xFUL << ((pin - 8) * 4));
        // Set AF bits /* PDF Reference */
        GPIOx->AFRH |= ((uint32_t)config->gpio_af << ((pin - 8) * 4));
    }

    // 7. Enable clock for Timer
    Enable_Timer_Clock(config->tim_base); /* Assumed RCC clock enabling - please verify */

    // 8. Configure Timer Time-base (Upcounting, No clock division)
    // Disable counter while configuring
    TIMx->CR1 &= ~(1UL << 0); // CEN = 0 /* PDF Reference (TIMx_CR1 Bit 0) */

    // Set clock division to tDTS = tCK_INT (00) /* PDF Reference (TIMx_CR1 Bits 9:8) */
    TIMx->CR1 &= ~(3UL << 8); // CKD[1:0] = 00

    // Set counter direction to Upcounting (DIR=0, CMS=00 for edge-aligned)
    // DIR and CMS bits are not applicable/reserved for TIM9-11 (upcounting only)
    if (config->tim_base == TIM1_BASE || (config->tim_base >= TIM2_BASE && config->tim_base <= TIM5_BASE))
    {
        TIMx->CR1 &= ~(1UL << 4); // DIR = 0 (Upcounting) /* PDF Reference (TIMx_CR1 Bit 4) */
        TIMx->CR1 &= ~(3UL << 5); // CMS[1:0] = 00 (Edge-aligned mode) /* PDF Reference (TIMx_CR1 Bits 6:5) */
    }
    // For TIM9-11, DIR/CMS are fixed for upcounting edge-aligned mode /* PDF Reference */

    // Enable auto-reload preload (ARPE=1) /* PDF Reference (TIMx_CR1 Bit 7) */
    TIMx->CR1 |= (1UL << 7);

    // Set Update Request Source to only counter overflow/underflow (URS=1)
    // This prevents software UG or slave mode trigger from setting UIF, only counter rollovers.
    TIMx->CR1 |= (1UL << 2); // URS = 1 /* PDF Reference (TIMx_CR1 Bit 2) */


    // 9. Configure Timer Channel for PWM output
    volatile uint32_t *ccmr_reg = NULL;
    uint8_t ccmr_shift = 0; // 0 for CH1/3, 1 for CH2/4

    switch (channel)
    {
        case TIM_CHANNEL_1:
        case TIM_CHANNEL_2:
            // TIMx_CCMR1 handles CH1 and CH2 /* PDF Reference (TIMx_CCMR1) */
            ccmr_reg = &TIMx->CCMR1;
            ccmr_shift = (channel == TIM_CHANNEL_1) ? 0 : 1; // 0 for CH1, 1 for CH2
            break;
        case TIM_CHANNEL_3:
        case TIM_CHANNEL_4:
            // TIMx_CCMR2 handles CH3 and CH4 (Not present on TIM9-11) /* PDF Reference (TIMx_CCMR2) */
             if (config->tim_base != TIM9_BASE && config->tim_base != TIM10_BASE && config->tim_base != TIM11_BASE)
             {
                 ccmr_reg = &TIMx->CCMR2;
                 ccmr_shift = (channel == TIM_CHANNEL_3) ? 0 : 1; // 0 for CH3, 1 for CH4
             } else {
                 // Invalid channel for this timer, exit
                 return;
             }
            break;
        default:
            // Invalid channel number, exit
            return;
    }

    // Clear CCxS bits (configure as output) /* PDF Reference (TIMx_CCMRx CCxS Bits) */
    // CCxS are at bits [1:0] for channel 1 & 3, and [9:8] for channel 2 & 4
    *ccmr_reg &= ~(3UL << (ccmr_shift * 8));

    // Set output compare mode to PWM Mode 1 (110) /* PDF Reference (TIMx_CCMRx OCxM Bits) */
    // OCxM are at bits [6:4] for channel 1 & 3, and [14:12] for channel 2 & 4
    *ccmr_reg &= ~(7UL << (ccmr_shift * 8 + 4)); // Clear OCxM bits
    *ccmr_reg |= (6UL << (ccmr_shift * 8 + 4));  // Set OCxM to 110 (PWM Mode 1)

    // Enable output compare preload (OCxPE = 1) /* PDF Reference (TIMx_CCMRx OCxPE Bits) */
    // OCxPE are at bits [3] for channel 1 & 3, and [11] for channel 2 & 4
    *ccmr_reg |= (1UL << (ccmr_shift * 8 + 3));

    // 10. Configure output polarity and enable capture/compare output
    uint8_t ccer_shift = (channel - 1); // 0 for CH1, 1 for CH2, 2 for CH3, 3 for CH4

    // Set output polarity to active high (CCxP = 0) /* PDF Reference (TIMx_CCER CCxP Bits) */
    // CCxP are at bits [1] for CH1, [5] for CH2, [9] for CH3, [13] for CH4
    TIMx->CCER &= ~(1UL << (ccer_shift * 4 + 1)); // Clear CCxP bit

    // Output will be enabled in PWM_Start by setting CCxE bit /* PDF Reference (TIMx_CCER CCxE Bits) */
    // CCxE are at bits [0] for CH1, [4] for CH2, [8] for CH3, [12] for CH4

    // 11. For TIM1 only: Configure BDTR (Break and Dead-Time Register)
    // Main Output Enable (MOE) is required for TIM1 outputs to work.
    // Dead time and break features are not strictly needed for basic PWM,
    // but MOE must be set. It is set in PWM_Start.
    // BDTR is only present on TIM1. /* PDF Reference (TIMx_BDTR) */

    // 12. Generate an update event (UG) to load all preload registers (ARR, PSC, CCRx)
    // This ensures the initial configuration is loaded into the active registers.
    TIMx->EGR |= (1UL << 0); // UG = 1 /* PDF Reference (TIMx_EGR Bit 0) */

    // Counter starts disabled (CEN=0) and output is disabled (CCxE=0 and/or MOE=0 for TIM1),
    // they will be enabled in PWM_Start.
}

/**
 * @brief Set the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PWM_Channel_Config_t *config = Get_Channel_Config(TRD_Channel);
    if (config == NULL || frequency == 0)
    {
        // Invalid channel or frequency. Handle as needed.
        return;
    }

    TIM_TypeDef *TIMx = TIM(config->tim_base);
    uint32_t timer_clock = config->tim_clock_freq;
    uint8_t channel = config->tim_channel;
    uint8_t is_32bit = config->is_32bit_timer;

    // Calculate PSC and ARR to achieve the desired frequency.
    // Timer Frequency (CK_CNT) = Timer_Clock / (PSC + 1)
    // PWM Frequency = Timer Frequency / (ARR + 1)
    // frequency = (Timer_Clock / (PSC + 1)) / (ARR + 1)
    // (PSC + 1) * (ARR + 1) = Timer_Clock / frequency

    // We need to find PSC and ARR such that their product + 1 equals Timer_Clock / frequency.
    // To maximize resolution (larger ARR), minimize PSC.
    // Smallest PSC is 0. Try PSC = 0.
    uint32_t total_counts = timer_clock / frequency; // Total counts per period
    uint32_t psc_val = 0;
    uint32_t arr_val;

    uint32_t max_arr = is_32bit ? 0xFFFFFFFFUL : 0xFFFFUL; // Max value for ARR register /* PDF Reference (TIMx_ARR) */
    uint32_t max_psc = 0xFFFFUL; // Max value for PSC register /* PDF Reference (TIMx_PSC) */

    // If total counts fit in ARR with PSC=0, use that
    if (total_counts > 0 && total_counts - 1 <= max_arr)
    {
        psc_val = 0;
        arr_val = total_counts - 1;
    }
    else if (total_counts > 0)
    {
        // If total counts are too large for ARR with PSC=0, calculate PSC
        // Try to keep ARR reasonably large. A common approach is to calculate PSC
        // such that the resulting ARR is within the 16-bit range, ideally not too small.
        // (PSC + 1) = ceil(total_counts / (max_arr + 1))
        // We need to ensure PSC doesn't exceed max_psc.
        psc_val = (total_counts + max_arr) / (max_arr + 1);

        if (psc_val > max_psc)
        {
             // Frequency is too low to be generated by this timer even with max PSC.
             // Or calculation resulted in overflow.
             // Set to minimum frequency possible or handle error.
             // For this example, we'll clamp to the minimum possible frequency with max PSC.
             psc_val = max_psc;
             arr_val = (timer_clock / (psc_val + 1)) / frequency - 1; // Recalculate with max_psc
             if (arr_val > max_arr) arr_val = max_arr; // Clamp ARR
        } else {
             arr_val = (timer_clock / (psc_val + 1)) / frequency - 1; // Calculate ARR
        }
        // Refine arr_val slightly if integer division resulted in error
         if ((psc_val + 1) * (arr_val + 1) > timer_clock / frequency + 1 && arr_val > 0) arr_val--;

    } else {
        // Frequency is higher than timer clock or invalid. Set to max frequency (PSC=0, ARR=0)
        psc_val = 0;
        arr_val = 0;
        frequency = timer_clock; // Actual frequency will be timer_clock
    }


    // Calculate CCR value for duty cycle.
    // Duty cycle is percentage (0-100). Pulse width = period * (duty / 100.0).
    // CCRx value determines the edge. For PWM Mode 1 (output high while CNT < CCRx):
    // - 0% duty: CCRx = 0. Output is low when CNT is 0, remains low.
    // - 100% duty: CCRx >= ARR + 1. Output is high when CNT < ARR+1 (always true for CNT <= ARR).
    // - Duty %: CCRx = floor((ARR + 1) * duty / 100.0)
    uint32_t ccr_val = 0;
    uint64_t period_counts = arr_val + 1;

    if (duty > 0 && duty < 100)
    {
        // Calculate pulse width in counts. Use 64-bit for intermediate multiplication to avoid overflow.
        uint64_t pulse_counts = (period_counts * duty) / 100;
        ccr_val = (uint32_t)pulse_counts;
         // For PWM Mode 1, output is high as long as CNT < CCRx.
         // If pulse_counts is X, we want output high for X counts. This requires CCRx = X.
         // When CNT goes from X-1 to X, the condition CNT < CCRx becomes false (if CCRx=X).
         // If CCRx is based on 0-indexed count (0 to ARR), then CCRx counts from 0 up to ARR.
         // CNT < CCRx means high for counts 0, 1, ..., CCRx-1. This is CCRx counts.
         // So CCRx should be equal to pulse_counts.
    }
    else if (duty == 100)
    {
        // For 100% duty, output is always high. Set CCRx >= ARR + 1.
        // PDF states if CCRx >= ARR, OCxRef is held at '1' in PWM mode 1 upcounting. /* PDF Reference (TIMx_CCMRx OCxM=110 description) */
        ccr_val = arr_val + 1; // Set to ARR + 1 to ensure CCRx > ARR
    }
    // If duty is 0, ccr_val remains 0, resulting in 0% duty cycle. /* PDF Reference (TIMx_CCMRx OCxM=110 description) */


    // Update Timer registers (PSC, ARR, CCRx)
    TIMx->PSC = psc_val; /* PDF Reference (TIMx_PSC) */
    TIMx->ARR = arr_val; /* PDF Reference (TIMx_ARR) */

    // Update the correct CCR register based on the channel number
    // Direct 32-bit write is safe even for 16-bit registers.
    switch (channel)
    {
        case TIM_CHANNEL_1:
            TIMx->CCR1 = ccr_val; /* PDF Reference (TIMx_CCR1) */
            break;
        case TIM_CHANNEL_2:
             // TIM10/11 don't have CCR2 /* PDF Reference (TIMx_CCR2) */
            if (config->tim_base != TIM10_BASE && config->tim_base != TIM11_BASE)
            {
                 TIMx->CCR2 = ccr_val; /* PDF Reference (TIMx_CCR2) */
            }
            break;
        case TIM_CHANNEL_3:
             // TIM9-11 don't have CCR3 /* PDF Reference (TIMx_CCR3) */
             if (config->tim_base != TIM9_BASE && config->tim_base != TIM10_BASE && config->tim_base != TIM11_BASE)
             {
                TIMx->CCR3 = ccr_val; /* PDF Reference (TIMx_CCR3) */
             }
            break;
        case TIM_CHANNEL_4:
            // TIM9-11 don't have CCR4 /* PDF Reference (TIMx_CCR4) */
             if (config->tim_base != TIM9_BASE && config->tim_base != TIM10_BASE && config->tim_base != TIM11_BASE)
             {
                TIMx->CCR4 = ccr_val; /* PDF Reference (TIMx_CCR4) */
             }
            break;
        default:
            // Invalid channel number - should not happen with valid TRD_Channel_t
            break;
    }

    // Generate update event (UG) to load new PSC, ARR, and CCR values from preload registers
    // This is important when using preload registers (ARPE, OCxPE are set in Init)
    // URS=1 prevents this UG from setting the UIF flag.
    TIMx->EGR |= (1UL << 0); // UG = 1 /* PDF Reference (TIMx_EGR Bit 0) */
}

/**
 * @brief Enable and start PWM signal generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = Get_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    TIM_TypeDef *TIMx = TIM(config->tim_base);
    uint8_t channel = config->tim_channel;
    uint8_t ccer_shift = (channel - 1); // 0 for CH1, 1 for CH2, 2 for CH3, 3 for CH4

    // For TIM1, enable main output (MOE) /* PDF Reference (TIMx_BDTR Bit 15) */
    // BDTR is only present on TIM1.
    if (config->tim_base == TIM1_BASE)
    {
        TIMx->BDTR |= (1UL << 15); // MOE = 1
    }

    // Enable the specific channel output (CCxE) /* PDF Reference (TIMx_CCER CCxE Bits) */
    // CCxE are at bits [0] for CH1, [4] for CH2, [8] for CH3, [12] for CH4
    TIMx->CCER |= (1UL << (ccer_shift * 4));

    // Enable the timer counter (CEN) /* PDF Reference (TIMx_CR1 Bit 0) */
    // If the counter is already running (e.g., for other channels), this is harmless.
    TIMx->CR1 |= (1UL << 0);
}

/**
 * @brief Stop the PWM signal output on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = Get_Channel_Config(TRD_Channel);
    if (config == NULL)
    {
        return; // Invalid channel
    }

    TIM_TypeDef *TIMx = TIM(config->tim_base);
    uint8_t channel = config->tim_channel;
    uint8_t ccer_shift = (channel - 1); // 0 for CH1, 1 for CH2, 2 for CH3, 3 for CH4

    // Disable the specific channel output (CCxE) /* PDF Reference (TIMx_CCER CCxE Bits) */
    // CCxE are at bits [0] for CH1, [4] for CH2, [8] for CH3, [12] for CH4
    TIMx->CCER &= ~(1UL << (ccer_shift * 4));

    // Note: For TIM1, the Main Output Enable (MOE) bit in BDTR controls all outputs.
    // To stop only a single channel on TIM1 without affecting others, only CCxE should be cleared.
    // If this were the *last* active channel on TIM1, you might also clear MOE,
    // but this function only stops the specified channel. A higher-level module
    // could track active channels per timer instance.
    // Stopping the timer counter (CEN) here would stop all channels on the timer,
    // which is usually not desired for a per-channel stop function.
}

/**
 * @brief Disable all PWM peripherals and outputs to reduce power consumption.
 *
 * This function attempts to disable outputs, stop timers, and disable clocks
 * for all timers and GPIOs used for PWM channels defined in TRD_Channel_t.
 * Reserved Timers TIM10 and TIM11 are not affected as per requirement.
 *
 * Note: Disabling clocks in RCC registers requires knowledge not explicitly
 * provided in the isolated PDF sections. Assumed RCC register access is used.
 * Resetting GPIO modes to Input Floating is based on the PDF GPIO introduction.
 */
void PWM_PowerOff(void)
{
    // Array of timer bases used for PWM channels defined
    uint32_t pwm_timer_bases[] = {
        TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM9_BASE
    };
    uint32_t num_pwm_timers = sizeof(pwm_timer_bases) / sizeof(pwm_timer_bases[0]);

    // Disable outputs and stop counters for all PWM timers
    for (uint32_t i = 0; i < num_pwm_timers; ++i)
    {
        uint32_t tim_base = pwm_timer_bases[i];
        TIM_TypeDef *TIMx = TIM(tim_base);

        // Disable all Capture/Compare outputs for this timer
        // CCER controls individual channel enables (CCxE bits).
        // For TIM1, MOE in BDTR controls main output enable.
        TIMx->CCER = 0x00000000UL; /* PDF Reference (TIMx_CCER) */
        if (tim_base == TIM1_BASE)
        {
            TIMx->BDTR &= ~(1UL << 15); // MOE = 0 /* PDF Reference (TIMx_BDTR Bit 15) */
        }

        // Disable the timer counter /* PDF Reference (TIMx_CR1 Bit 0) */
        TIMx->CR1 &= ~(1UL << 0); // CEN = 0
    }

    // Disable clocks for Timers used for PWM (requires RCC knowledge not in PDF)
    /* Assumed RCC clock disabling - please verify */
    RCC_APB1ENR_R &= ~((1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 3)); // Disable TIM2, TIM3, TIM4, TIM5 clocks
    RCC_APB2ENR_R &= ~((1UL << 0) | (1UL << 16)); // Disable TIM1, TIM9 clocks

    // Reset GPIO pins used for PWM to their default Input Floating state.
    // PDF Section 8.3.1 states alternate functions are inactive and I/O ports are
    // configured in input floating mode after reset. Setting MODER to 00 achieves this.
    /* PDF Reference (GPIOx_MODER) */
    /* Assumed GPIO reset state based on PDF */
    for (int i = 1; i < TRD_PWM_CHANNEL_COUNT; ++i)
    {
         const PWM_Channel_Config_t *config = &Channel_Configs[i];
         GPIO_TypeDef *GPIOx = GPIO(config->gpio_base);
         uint8_t pin = config->gpio_pin;

         // Reset mode bits to Input (00)
         GPIOx->MODER &= ~(3UL << (pin * 2));

         // Other GPIO settings (OTYPER, OSPEEDR, PUPDR, AFRL/AFRH) could also be reset,
         // but MODER=00 is sufficient to revert to input floating and disable AF output.
    }

    // Note: Disabling GPIO clocks is complex as pins might be shared.
    // It's generally safer to reset the pin mode rather than disabling the entire port clock,
    // unless it's guaranteed no other peripheral is using pins on that port.
    // A complete power off might disable GPIO clocks if no pins on the port are needed,
    // but this requires tracking pin usage across the whole system, which is outside
    // the scope of this module.
    // For this module's configured pins, resetting the mode is sufficient to stop PWM output.
}

```