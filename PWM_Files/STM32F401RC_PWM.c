/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include <stddef.h> // For NULL
#include <stdint.h> // For uint32_t, uint8_t
#include <math.h>   // For roundf (optional, if more precise rounding desired)

// Placeholder for SystemCoreClock (usually provided by CMSIS)
// For this example, assuming a common value for F401RC at max speed
#ifndef SystemCoreClock
#define SystemCoreClock 84000000UL /* Assumed SystemCoreClock for calculations */
#endif

// Placeholder for APB bus frequencies (depend on RCC configuration)
// For STM32F401RC, APB1 max is 42MHz, APB2 max is 84MHz.
// Assuming APB1 prescaler of 2 and APB2 prescaler of 1 for calculations.
#define APB1_CLK_FREQ (SystemCoreClock / 2) /* Assumed APB1 clock frequency */
#define APB2_CLK_FREQ (SystemCoreClock / 1) /* Assumed APB2 clock frequency */

#define MAX_TIM1_3_4_9_10_11_ARR  (0xFFFFUL)     // 16-bit timers max auto-reload value
#define MAX_TIM2_5_ARR            (0xFFFFFFFFUL) // 32-bit timers max auto-reload value

// Type definitions (assuming tlong and tbyte are aliases for standard integer types)
// If not defined globally, uncomment these:
// typedef uint32_t tlong;
// typedef uint8_t  tbyte;


/**
 * @brief Defines the configuration for a single PWM channel.
 */
typedef struct {
    TIM_TypeDef *TIMx;                 /**< Pointer to the Timer peripheral base address (e.g., TIM1, TIM3) */
    uint8_t ChannelNumber;             /**< PWM Channel number (1, 2, 3, or 4) */
    GPIO_TypeDef *PortName;            /**< Pointer to the GPIO port base address (e.g., GPIOA, GPIOB) */
    uint8_t PinNumber;                 /**< Pin number on the GPIO port (0-15) */
    uint8_t AlternateFunctionNumber;   /**< Alternate Function number for the pin (AF1-AF15) */
} PWM_Channel_Config_t;

/**
 * @brief Array mapping TRD_Channel_t enum to specific hardware configurations.
 *
 * This array defines the timer, channel, GPIO port, pin, and alternate function
 * for each available PWM output on the STM32F401RC.
 *
 * Note on Reserved Timers: TIM2 and TIM9 are reserved for other potential
 * system functionalities such as system tick or precise input capture,
 * and are therefore excluded from this PWM driver implementation.
 */
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TIM1 (APB2 Clock - AF1) - Advanced-control timer, 16-bit counter
    // Uses Alternate Function 1 (AF1) for all channels.
    {TIM1, 1, GPIOA, 8, 1},  // TIM1_CH1 on PA8   /* Assumed PWM config */
    {TIM1, 2, GPIOA, 9, 1},  // TIM1_CH2 on PA9   /* Assumed PWM config */
    {TIM1, 3, GPIOA, 10, 1}, // TIM1_CH3 on PA10  /* Assumed PWM config */
    {TIM1, 4, GPIOA, 11, 1}, // TIM1_CH4 on PA11  /* Assumed PWM config */

    // TIM3 (APB1 Clock - AF2) - General-purpose timer, 16-bit counter
    // Prioritizing Port A, then Port B, then Port C for unique mapping.
    // Uses Alternate Function 2 (AF2) for all channels.
    {TIM3, 1, GPIOA, 6, 2},  // TIM3_CH1 on PA6   /* Assumed PWM config */
    {TIM3, 2, GPIOA, 7, 2},  // TIM3_CH2 on PA7   /* Assumed PWM config */
    {TIM3, 3, GPIOB, 0, 2},  // TIM3_CH3 on PB0   /* Assumed PWM config */
    {TIM3, 4, GPIOB, 1, 2},  // TIM3_CH4 on PB1   /* Assumed PWM config */

    // TIM4 (APB1 Clock - AF2) - General-purpose timer, 16-bit counter
    // Prioritizing Port B, then Port D for unique mapping.
    // Uses Alternate Function 2 (AF2) for all channels.
    {TIM4, 1, GPIOB, 6, 2},  // TIM4_CH1 on PB6   /* Assumed PWM config */
    {TIM4, 2, GPIOB, 7, 2},  // TIM4_CH2 on PB7   /* Assumed PWM config */
    {TIM4, 3, GPIOB, 8, 2},  // TIM4_CH3 on PB8   /* Assumed PWM config */
    {TIM4, 4, GPIOB, 9, 2},  // TIM4_CH4 on PB9   /* Assumed PWM config */

    // TIM5 (APB1 Clock - AF2) - General-purpose timer, 32-bit counter
    // All channels typically on Port A for F401.
    // Pin 0 is included as common for STM32 general-purpose timers.
    // Uses Alternate Function 2 (AF2) for all channels.
    {TIM5, 1, GPIOA, 0, 2},  // TIM5_CH1 on PA0   /* Assumed PWM config - pin 0 is often multi-functional */
    {TIM5, 2, GPIOA, 1, 2},  // TIM5_CH2 on PA1   /* Assumed PWM config */
    {TIM5, 3, GPIOA, 2, 2},  // TIM5_CH3 on PA2   /* Assumed PWM config */
    {TIM5, 4, GPIOA, 3, 2},  // TIM5_CH4 on PA3   /* Assumed PWM config */

    // TIM10 (APB2 Clock - AF3) - General-purpose timer, 16-bit counter
    // Only has one channel (CH1).
    // Uses Alternate Function 3 (AF3).
    {TIM10, 1, GPIOB, 8, 3}, // TIM10_CH1 on PB8 (shared with TIM4_CH3) /* Assumed PWM config */

    // TIM11 (APB2 Clock - AF3) - General-purpose timer, 16-bit counter
    // Only has one channel (CH1).
    // Uses Alternate Function 3 (AF3).
    {TIM11, 1, GPIOB, 9, 3}  // TIM11_CH1 on PB9 (shared with TIM4_CH4) /* Assumed PWM config */
};

/**
 * @brief Helper function to get the AHB1ENR bit for a given GPIO Port.
 * @param PortName Pointer to the GPIO_TypeDef structure for the port.
 * @return The corresponding bitmask for enabling the GPIO clock in RCC_AHB1ENR.
 */
static uint32_t GetGpioClockEnableBit(GPIO_TypeDef *PortName) {
    if (PortName == GPIOA)      return RCC_AHB1ENR_GPIOAEN;
    else if (PortName == GPIOB) return RCC_AHB1ENR_GPIOBEN;
    else if (PortName == GPIOC) return RCC_AHB1ENR_GPIOCEN;
    else if (PortName == GPIOD) return RCC_AHB1ENR_GPIODEN;
    else if (PortName == GPIOE) return RCC_AHB1ENR_GPIOEEN;
    else if (PortName == GPIOH) return RCC_AHB1ENR_GPIOHEN;
    return 0; // Invalid port, though should not occur with valid configs
}

/**
 * @brief Helper function to get the Timer Clock Frequency based on Timer Instance.
 * @param TIMx Pointer to the TIM_TypeDef structure for the timer.
 * @return The clock frequency in Hz, or 0 for invalid timer.
 */
static uint32_t GetTimerClockFrequency(TIM_TypeDef *TIMx) {
    // According to RM0368 Rev 5, Sections 12.1 (TIM1) and 13.1 (TIM2-5) and 14.1 (TIM9-11)
    // TIM1, TIM9, TIM10, TIM11 are on APB2 bus.
    // TIM2, TIM3, TIM4, TIM5 are on APB1 bus.
    if (TIMx == TIM1 || TIMx == TIM10 || TIMx == TIM11) { /* PDF Reference - APB2 for TIM1, 10, 11 */
        return APB2_CLK_FREQ;
    } else if (TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) { /* PDF Reference - APB1 for TIM3, 4, 5 */
        return APB1_CLK_FREQ;
    }
    // Reserved timers (TIM2, TIM9) and invalid instances return 0
    return 0; 
}

/**
 * @brief Helper function to enable the clock for a given Timer instance.
 * @param TIMx Pointer to the TIM_TypeDef structure for the timer.
 */
static void EnableTimerClock(TIM_TypeDef *TIMx) {
    if (TIMx == TIM1)      RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   /* PDF Reference */
    else if (TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   /* PDF Reference */
    else if (TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   /* PDF Reference */
    else if (TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;   /* PDF Reference */
    else if (TIMx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* PDF Reference */
    else if (TIMx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* PDF Reference */
    // Reserved timers (TIM2, TIM9) are intentionally not handled here for PWM.
}

/**
 * @brief Helper function to disable the clock for a given Timer instance.
 * @param TIMx Pointer to the TIM_TypeDef structure for the timer.
 */
static void DisableTimerClock(TIM_TypeDef *TIMx) {
    if (TIMx == TIM1)      RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
    else if (TIMx == TIM3) RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
    else if (TIMx == TIM4) RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
    else if (TIMx == TIM5) RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
    else if (TIMx == TIM10) RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
    // FIX: TIM11 is on APB2, not APB1.
    else if (TIMx == TIM11) RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; 
}


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_NUM_CHANNELS) {
        // Invalid channel, return or handle error
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    GPIO_TypeDef *gpio_port = config->PortName;
    uint8_t pin_number = config->PinNumber;
    TIM_TypeDef *tim = config->TIMx;
    uint8_t channel = config->ChannelNumber;
    uint8_t af_number = config->AlternateFunctionNumber;

    // 1. Enable GPIO Clock
    RCC->AHB1ENR |= GetGpioClockEnableBit(gpio_port); /* PDF Reference - AHB1ENR for GPIO */

    // 2. Configure GPIO Pin for Alternate Function
    // Clear and Set MODER bits for Alternate Function mode (10)
    // Assuming GPIO_MODER_AF is defined as 0x2UL in STM32F401RC_PWM.h or CMSIS
    gpio_port->MODER &= ~(3UL << GPIO_MODER_MODERy_POS(pin_number));    /* PDF Reference - GPIOx_MODER */
    gpio_port->MODER |= (GPIO_MODER_AF << GPIO_MODER_MODERy_POS(pin_number)); /* PDF Reference - GPIOx_MODER */

    // Configure OTYPER for Push-Pull output (0)
    // Assuming GPIO_OTYPER_PUSHPULL is defined as 0x0UL in STM32F401RC_PWM.h or CMSIS
    gpio_port->OTYPER &= ~(1UL << GPIO_OTYPER_OTy_POS(pin_number));   /* PDF Reference - GPIOx_OTYPER */
    gpio_port->OTYPER |= (GPIO_OTYPER_PUSHPULL << GPIO_OTYPER_OTy_POS(pin_number)); /* PDF Reference - GPIOx_OTYPER */

    // Configure OSPEEDR for High Speed (10)
    // Assuming GPIO_OSPEEDR_HIGH_SPEED is defined as 0x2UL in STM32F401RC_PWM.h or CMSIS
    gpio_port->OSPEEDR &= ~(3UL << GPIO_OSPEEDR_OSPEEDRy_POS(pin_number)); /* PDF Reference - GPIOx_OSPEEDR */
    gpio_port->OSPEEDR |= (GPIO_OSPEEDR_HIGH_SPEED << GPIO_OSPEEDR_OSPEEDRy_POS(pin_number)); /* PDF Reference - GPIOx_OSPEEDR */

    // Configure PUPDR for No Pull-up/Pull-down (00)
    // Assuming GPIO_PUPDR_NOPULL is defined as 0x0UL in STM32F401RC_PWM.h or CMSIS
    gpio_port->PUPDR &= ~(3UL << GPIO_PUPDR_PUPDRy_POS(pin_number));   /* PDF Reference - GPIOx_PUPDR */
    gpio_port->PUPDR |= (GPIO_PUPDR_NOPULL << GPIO_PUPDR_PUPDRy_POS(pin_number)); /* PDF Reference - GPIOx_PUPDR */

    // Configure Alternate Function (AFRL for pins 0-7, AFRH for pins 8-15)
    // Assuming GPIO_AFR_AFRy_MASK is defined as 0xFUL and GPIO_AFR_AFRy_POS handles (pin_number % 8) * 4
    if (pin_number < 8) {
        gpio_port->AFRL &= ~(GPIO_AFR_AFRy_MASK << GPIO_AFR_AFRy_POS(pin_number)); /* PDF Reference - GPIOx_AFRL */
        gpio_port->AFRL |= (af_number << GPIO_AFR_AFRy_POS(pin_number));            /* PDF Reference - GPIOx_AFRL */
    } else {
        gpio_port->AFRH &= ~(GPIO_AFR_AFRy_MASK << GPIO_AFR_AFRy_POS(pin_number)); /* PDF Reference - GPIOx_AFRH */
        gpio_port->AFRH |= (af_number << GPIO_AFR_AFRy_POS(pin_number));            /* PDF Reference - GPIOx_AFRH */
    }

    // 3. Enable Timer Clock
    EnableTimerClock(tim);

    // 4. Timer Configuration
    // Disable counter before configuration
    tim->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1 */

    // Set Update Disable (UDIS) to 0 (UEV enabled), URS to 1 (only overflow/underflow generates UEV)
    tim->CR1 &= ~(TIM_CR1_UDIS | TIM_CR1_URS); /* Clear both UDIS and URS */
    tim->CR1 |= TIM_CR1_URS; /* Set URS to 1 */ /* PDF Reference - TIMx_CR1 */

    // Set auto-reload preload enable (ARPE) for buffered ARR
    tim->CR1 |= TIM_CR1_ARPE; /* PDF Reference - TIMx_CR1 */

    // Set clock division (CKD) to 00 (tDTS=tCK_INT)
    tim->CR1 &= ~TIM_CR1_CKD; /* PDF Reference - TIMx_CR1 */

    // Configure PWM mode 1 (110) and enable preload for CCRx
    // Set Output Compare Preload Enable (OCxPE)
    // Set Capture/Compare Selection (CCxS) to output (00)
    // Assuming TIM_OCM_PWM1 is defined as 0x6UL
    switch (channel) {
        case 1:
            tim->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M); /* PDF Reference - TIMx_CCMR1 */
            tim->CCMR1 |= (TIM_OCM_PWM1 << TIM_CCMR1_OC1M_Pos); /* PDF Reference - TIMx_CCMR1 */
            tim->CCMR1 |= TIM_CCMR1_OC1PE; /* PDF Reference - TIMx_CCMR1 */
            tim->CCER &= ~(TIM_CCER_CC1P); // Clear polarity bit for active high (0)
            tim->CCER |= TIM_CCER_CC1E;     // Enable output /* PDF Reference - TIMx_CCER */
            break;
        case 2:
            tim->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M); /* PDF Reference - TIMx_CCMR1 */
            tim->CCMR1 |= (TIM_OCM_PWM1 << TIM_CCMR1_OC2M_Pos); /* PDF Reference - TIMx_CCMR1 */
            tim->CCMR1 |= TIM_CCMR1_OC2PE; /* PDF Reference - TIMx_CCMR1 */
            tim->CCER &= ~(TIM_CCER_CC2P); // Clear polarity bit for active high (0)
            tim->CCER |= TIM_CCER_CC2E;     // Enable output /* PDF Reference - TIMx_CCER */
            break;
        case 3:
            // TIM9/10/11 do not have CCMR2 or more than 2 channels (or 1 channel for 10/11)
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M); /* PDF Reference - TIMx_CCMR2 */
                tim->CCMR2 |= (TIM_OCM_PWM1 << TIM_CCMR2_OC3M_Pos); /* PDF Reference - TIMx_CCMR2 */
                tim->CCMR2 |= TIM_CCMR2_OC3PE; /* PDF Reference - TIMx_CCMR2 */
                tim->CCER &= ~(TIM_CCER_CC3P); // Clear polarity bit for active high (0)
                tim->CCER |= TIM_CCER_CC3E;     // Enable output /* PDF Reference - TIMx_CCER */
            }
            break;
        case 4:
            // TIM9/10/11 do not have CCMR2 or more than 2 channels (or 1 channel for 10/11)
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M); /* PDF Reference - TIMx_CCMR2 */
                tim->CCMR2 |= (TIM_OCM_PWM1 << TIM_CCMR2_OC4M_Pos); /* PDF Reference - TIMx_CCMR2 */
                tim->CCMR2 |= TIM_CCMR2_OC4PE; /* PDF Reference - TIMx_CCMR2 */
                tim->CCER &= ~(TIM_CCER_CC4P); // Clear polarity bit for active high (0)
                tim->CCER |= TIM_CCER_CC4E;     // Enable output /* PDF Reference - TIMx_CCER */
            }
            break;
        default:
            // Should not happen as channel is validated by TRD_Channel_t mapping
            break;
    }

    // Generate an update event to load all preload registers (PSC, ARR, CCRx)
    tim->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR */

    // For Advanced-control timers (like TIM1), enable main output
    if (tim == TIM1) {
        tim->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR (MOE) */
    }
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The specific PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, uint32_t frequency, uint8_t duty) {
    if (TRD_Channel >= TRD_NUM_CHANNELS || duty > 100) {
        // Invalid channel or duty cycle, return or handle error
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *tim = config->TIMx;
    uint8_t channel = config->ChannelNumber;

    uint32_t timer_clk_freq = GetTimerClockFrequency(tim);
    if (timer_clk_freq == 0 || frequency == 0) {
        return; // Invalid timer clock or frequency
    }

    float total_cycles_float = (float)timer_clk_freq / (float)frequency;
    uint32_t prescaler = 0;
    uint32_t autoreload = 0;

    // Determine max ARR based on timer type (16-bit or 32-bit)
    uint32_t max_arr = (tim == TIM2 || tim == TIM5) ? MAX_TIM2_5_ARR : MAX_TIM1_3_4_9_10_11_ARR;

    // Calculate prescaler and auto-reload value
    if (total_cycles_float > (float)max_arr + 1.0f) {
        // If the desired period (total_cycles) exceeds what ARR can hold, apply prescaling.
        // Calculate minimum prescaler to fit autoreload within max_arr.
        prescaler = (uint32_t)(total_cycles_float / (max_arr + 1.0f));
        
        // Adjust prescaler up if there's a remainder, to ensure total_cycles_float can be achieved.
        // Example: if total_cycles_float = 65537 and max_arr = 65535, prescaler would be 1.
        // Then (prescaler + 1) * (max_arr + 1) = 2 * 65536 = 131072.
        // This is a common way to ensure (PSC+1)*(ARR+1) >= total_cycles_float.
        if (((uint32_t)total_cycles_float % (max_arr + 1UL)) != 0) {
             prescaler++; // Increment prescaler if (total_cycles / (max_arr + 1)) has a remainder
        }

        // Prescaler register is always 16-bit
        if (prescaler > 0xFFFFUL) prescaler = 0xFFFFUL; /* PDF Reference - PSC register is 16-bit */
        
        // Recalculate autoreload with the chosen prescaler
        autoreload = (uint32_t)roundf(total_cycles_float / (prescaler + 1.0f)) - 1;

    } else {
        // No prescaling needed, ARR can hold the full period.
        prescaler = 0;
        autoreload = (uint32_t)roundf(total_cycles_float) - 1;
    }

    // Ensure autoreload is not negative (min ARR is 0 for period 1)
    if (autoreload > max_arr) autoreload = max_arr; // Should not happen with correct calculations, but as a safeguard.
    // If target frequency is extremely high (period_float < 1), autoreload might be 0 or -1.
    // An ARR of 0 results in a period of 1 clock cycle.
    // Ensure it's not actually negative due to floating point truncation if value is < 1.0
    if (total_cycles_float < 1.0f / (prescaler + 1.0f)) autoreload = 0;


    // Apply calculated values to timer registers
    tim->PSC = prescaler; /* PDF Reference - TIMx_PSC */
    tim->ARR = autoreload; /* PDF Reference - TIMx_ARR */

    // Calculate duty cycle value based on the new ARR (autoreload + 1 is the full period)
    uint32_t ccr_val;
    if (duty == 0) {
        ccr_val = 0;
    } else if (duty == 100) {
        ccr_val = autoreload + 1; // For 100% duty, CCRx should be ARR + 1 (period length)
    } else {
        ccr_val = (uint32_t)(((float)duty / 100.0f) * (autoreload + 1.0f));
    }
    
    // Ensure CCR value does not exceed ARR+1 (full period) and is not negative
    if (ccr_val > (autoreload + 1)) ccr_val = autoreload + 1;
    if (ccr_val < 0) ccr_val = 0; // Should logically not happen with positive duty, but defensive programming

    // Set Capture/Compare Register (CCRx)
    switch (channel) {
        case 1:
            tim->CCR1 = ccr_val; /* PDF Reference - TIMx_CCR1 */
            break;
        case 2:
            tim->CCR2 = ccr_val; /* PDF Reference - TIMx_CCR2 */
            break;
        case 3:
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCR3 = ccr_val; /* PDF Reference - TIMx_CCR3 */
            }
            break;
        case 4:
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCR4 = ccr_val; /* PDF Reference - TIMx_CCR4 */
            }
            break;
        default:
            // Should not happen with valid TRD_Channel_t
            break;
    }

    // Generate an update event to immediately apply the new PSC, ARR, and CCRx values
    // This loads the buffered registers into active registers.
    tim->EGR |= TIM_EGR_UG; /* PDF Reference - TIMx_EGR */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The specific PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_NUM_CHANNELS) {
        // Invalid channel, return or handle error
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *tim = config->TIMx;

    // Enable the counter (CEN bit)
    tim->CR1 |= TIM_CR1_CEN; /* PDF Reference - TIMx_CR1 */

    // For Advanced-control timers (like TIM1), ensure main output is enabled (MOE bit)
    if (tim == TIM1) {
        tim->BDTR |= TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR (MOE) */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The specific PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_NUM_CHANNELS) {
        // Invalid channel, return or handle error
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *tim = config->TIMx;
    uint8_t channel = config->ChannelNumber;

    // Disable the counter (CEN bit)
    tim->CR1 &= ~TIM_CR1_CEN; /* PDF Reference - TIMx_CR1 */

    // For Advanced-control timers (like TIM1), disable main output (MOE bit)
    if (tim == TIM1) {
        tim->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference - TIMx_BDTR (MOE) */
    }

    // Optionally, force the output pin to a low state or disable the output driver
    // This is done by clearing the CCxE bit in CCER.
    switch (channel) {
        case 1: tim->CCER &= ~TIM_CCER_CC1E; break; /* PDF Reference - TIMx_CCER */
        case 2: tim->CCER &= ~TIM_CCER_CC2E; break; /* PDF Reference - TIMx_CCER */
        case 3:
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCER &= ~TIM_CCER_CC3E; /* PDF Reference - TIMx_CCER */
            }
            break;
        case 4:
            if (tim == TIM1 || tim == TIM3 || tim == TIM4 || tim == TIM5) {
                tim->CCER &= ~TIM_CCER_CC4E; /* PDF Reference - TIMx_CCER */
            }
            break;
        default: break;
    }
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 */
void PWM_PowerOff(void) {
    // Iterate through all configured PWM channels and stop them
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_NUM_CHANNELS; i++) {
        PWM_Stop(i);
    }

    // Disable clock for all timers used in this implementation
    DisableTimerClock(TIM1);
    DisableTimerClock(TIM3);
    DisableTimerClock(TIM4);
    DisableTimerClock(TIM5);
    DisableTimerClock(TIM10);
    DisableTimerClock(TIM11);

    // Note: GPIO pins are left in their last configured state (Alternate Function).
    // For maximum power saving, they could be reconfigured to analog mode or input
    // floating, but this might interfere with other functionalities sharing those pins.
    // This implementation prioritizes keeping their alternate function configuration
    // for quick re-initialization if needed, while disabling active timer output.
    // Full power-off for GPIOs would require iterating through and re-configuring
    // each GPIO pin used in the pwm_channel_map, which is typically handled by
    // a higher-level power management module or reset.
}