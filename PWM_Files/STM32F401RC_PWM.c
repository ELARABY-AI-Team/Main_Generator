/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : STM32F410RC PWM bare-metal driver implementation.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F410RC_PWM.h"

/** Assumed definitions from "STM32F410RC_PWM.h"
 * These are necessary for the bare-metal implementation using register access.
 * In a real project, these would come from CMSIS headers (stm32f4xx.h) or custom definitions.
 *
 * typedef struct {
 *     TIM_TypeDef *TIMx;           // Pointer to the Timer peripheral (e.g., TIM3)
 *     uint32_t Channel;            // Timer channel number (e.g., 1, 2, 3, 4)
 *     GPIO_TypeDef *GPIOx;         // Pointer to the GPIO port (e.g., GPIOB)
 *     uint32_t GPIO_Pin;           // GPIO pin number (0-15)
 *     uint32_t GPIO_AF;            // GPIO Alternate Function mapping (e.g., GPIO_AF2_TIM3)
 *     uint32_t RCC_APB_Timer_ENR;  // RCC register (APB1ENR or APB2ENR)
 *     uint32_t RCC_APB_Timer_Mask; // Mask for the Timer clock enable bit
 *     uint32_t RCC_AHB_GPIO_ENR;   // RCC register (AHB1ENR)
 *     uint32_t RCC_AHB_GPIO_Mask;  // Mask for the GPIO clock enable bit
 *     uint8_t APB_Timer_Bus;       // 1 for APB1, 2 for APB2 (for clock calculation)
 * } TRD_Channel_t;
 *
 * Assumed global variable/define for system clock frequency:
 * extern uint32_t SystemCoreClock; // Typically provided by CMSIS startup code
 *
 * Assumed register base addresses and bit definitions:
 * TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef structs and associated bit masks
 * (e.g., TIM_CR1_CEN, TIM_CCMR1_OC1M, TIM_CCER_CC1E, GPIO_MODER_MODER0, RCC_APB1ENR_TIM3EN, etc.)
 * from standard STM32F4 header files.
 *
 * Assumed typedefs:
 * typedef unsigned long tlong; // Mapping to uint32_t
 * typedef unsigned char tbyte; // Mapping to uint8_t
 */

// Define a helper function to get the timer clock frequency
static uint32_t Get_Timer_Clock_Freq(uint8_t apb_bus) {
    uint32_t pclk_freq = 0;
    uint32_t apb_prescaler = 0;
    uint32_t apb_presc_bits = 0;

    // Assumes SystemCoreClock is correctly set (e.g., by CMSIS startup)
    uint32_t hclk_freq = SystemCoreClock;

    if (apb_bus == 1) { // APB1
        apb_presc_bits = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07;
    } else if (apb_bus == 2) { // APB2
        apb_presc_bits = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07;
    } else {
        return 0; // Invalid bus
    }

    // Calculate APB prescaler value (from RM)
    if (apb_presc_bits < 4) {
        apb_prescaler = 1;
    } else {
        apb_prescaler = (1 << (apb_presc_bits - 3));
    }

    pclk_freq = hclk_freq / apb_prescaler;

    // Timer clock is PCLK if APB prescaler is 1, otherwise it's 2*PCLK
    if (apb_prescaler == 1) {
        return pclk_freq;
    } else {
        return pclk_freq * 2;
    }
}

/**Functions ===========================================================================*/

/*************************************************************************************************
* Function Name  : PWM_Init
* Description    : Initialize the PWM hardware and configure the timer and GPIOs for the given channel.
* Parameter      : TRD_Channel_t TRD_Channel - Structure containing channel configuration
* Return Value   : None
*************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel) {
    // Enable GPIO clock
    // Need to check which AHB register the GPIO is on for F410RC. AHB1 is common.
    // Assumed TRD_Channel.RCC_AHB_GPIO_ENR is the correct RCC AHB enable register pointer.
    *(volatile uint32_t *)TRD_Channel.RCC_AHB_GPIO_ENR |= TRD_Channel.RCC_AHB_GPIO_Mask;

    // Enable Timer clock
    // Assumed TRD_Channel.RCC_APB_Timer_ENR is the correct RCC APB enable register pointer.
    *(volatile uint32_t *)TRD_Channel.RCC_APB_Timer_ENR |= TRD_Channel.RCC_APB_Timer_Mask;

    // Configure GPIO pin for Alternate Function
    // Clear MODER bits for the pin (2 bits per pin)
    TRD_Channel.GPIOx->MODER &= ~(0x3UL << (TRD_Channel.GPIO_Pin * 2));
    // Set MODER bits for Alternate Function mode (0b10)
    TRD_Channel.GPIOx->MODER |= (0x2UL << (TRD_Channel.GPIO_Pin * 2));

    // Configure GPIO pin Output Type to Push-Pull (0b00)
    TRD_Channel.GPIOx->OTYPER &= ~(0x1UL << TRD_Channel.GPIO_Pin);

    // Configure GPIO pin Output Speed (e.g., High Speed 0b10)
    TRD_Channel.GPIOx->OSPEEDR &= ~(0x3UL << (TRD_Channel.GPIO_Pin * 2));
    TRD_Channel.GPIOx->OSPEEDR |= (0x2UL << (TRD_Channel.GPIO_Pin * 2)); // High Speed

    // Configure GPIO pin Pull-up/Pull-down to No Pull (0b00)
    TRD_Channel.GPIOx->PUPDR &= ~(0x3UL << (TRD_Channel.GPIO_Pin * 2));

    // Configure GPIO pin Alternate Function mapping
    // AFR[0] for pins 0-7, AFR[1] for pins 8-15 (4 bits per pin)
    TRD_Channel.GPIOx->AFR[(TRD_Channel.GPIO_Pin >> 3)] &= ~(0xFUL << ((TRD_Channel.GPIO_Pin & 0x07) * 4));
    TRD_Channel.GPIOx->AFR[(TRD_Channel.GPIO_Pin >> 3)] |= (TRD_Channel.GPIO_AF << ((TRD_Channel.GPIO_Pin & 0x07) * 4));

    // Configure Timer for PWM Mode 1
    // Clear OCxM and OCxPE bits initially
    // Set PWM mode 1 (110) and Output Compare Preload Enable (OCxPE)
    if (TRD_Channel.Channel == 1) {
        TRD_Channel.TIMx->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
        TRD_Channel.TIMx->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM Mode 1
        TRD_Channel.TIMx->CCMR1 |= TIM_CCMR1_OC1PE; // Output Compare Preload Enable
    } else if (TRD_Channel.Channel == 2) {
        TRD_Channel.TIMx->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
        TRD_Channel.TIMx->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM Mode 1
        TRD_Channel.TIMx->CCMR1 |= TIM_CCMR1_OC2PE; // Output Compare Preload Enable
    } else if (TRD_Channel.Channel == 3) {
        TRD_Channel.TIMx->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE);
        TRD_Channel.TIMx->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2); // PWM Mode 1
        TRD_Channel.TIMx->CCMR2 |= TIM_CCMR2_OC3PE; // Output Compare Preload Enable
    } else if (TRD_Channel.Channel == 4) {
        TRD_Channel.TIMx->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE);
        TRD_Channel.TIMx->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2); // PWM Mode 1
        TRD_Channel.TIMx->CCMR2 |= TIM_CCMR2_OC4PE; // Output Compare Preload Enable
    }
    // Note: Add error handling for invalid channel if necessary

    // Configure Capture/Compare Enable Register (CCER)
    // Enable the output for the channel, set polarity low (default)
    if (TRD_Channel.Channel == 1) {
        TRD_Channel.TIMx->CCER &= ~TIM_CCER_CC1P; // Polarity low
        TRD_Channel.TIMx->CCER |= TIM_CCER_CC1E;  // Output enable
    } else if (TRD_Channel.Channel == 2) {
        TRD_Channel.TIMx->CCER &= ~TIM_CCER_CC2P; // Polarity low
        TRD_Channel.TIMx->CCER |= TIM_CCER_CC2E;  // Output enable
    } else if (TRD_Channel.Channel == 3) {
        TRD_Channel.TIMx->CCER &= ~TIM_CCER_CC3P; // Polarity low
        TRD_Channel.TIMx->CCER |= TIM_CCER_CC3E;  // Output enable
    } else if (TRD_Channel.Channel == 4) {
        TRD_Channel.TIMx->CCER &= ~TIM_CCER_CC4P; // Polarity low
        TRD_Channel.TIMx->CCER |= TIM_CCER_CC4E;  // Output enable
    }

    // Enable Auto-Reload Preload
    TRD_Channel.TIMx->CR1 |= TIM_CR1_ARPE;

    // Set initial PSC, ARR, CCRx to 0
    TRD_Channel.TIMx->PSC = 0;
    TRD_Channel.TIMx->ARR = 0; // Will be set by PWM_Set_Freq
    if (TRD_Channel.Channel == 1) {
        TRD_Channel.TIMx->CCR1 = 0;
    } else if (TRD_Channel.Channel == 2) {
        TRD_Channel.TIMx->CCR2 = 0;
    } else if (TRD_Channel.Channel == 3) {
        TRD_Channel.TIMx->CCR3 = 0;
    } else if (TRD_Channel.Channel == 4) {
        TRD_Channel.TIMx->CCR4 = 0;
    }

    // Generate an update event to load the PSC, ARR, and CCMRx settings
    TRD_Channel.TIMx->EGR |= TIM_EGR_UG;
}

/*************************************************************************************************
* Function Name  : PWM_Set_Freq
* Description    : Set the desired PWM frequency and duty cycle for the selected channel.
* Parameter      : TRD_Channel_t TRD_Channel - Structure containing channel configuration
* Parameter      : tlong frequency - Desired frequency in Hz
* Parameter      : tbyte duty - Desired duty cycle in percentage (0-100)
* Return Value   : None
*************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    uint32_t timer_clock;
    uint32_t total_cycles;
    uint32_t prescaler;
    uint32_t period;
    uint32_t pulse;

    // Get the timer clock frequency
    timer_clock = Get_Timer_Clock_Freq(TRD_Channel.APB_Timer_Bus);

    // Ensure duty cycle is within 0-100 range
    if (duty > 100) {
        duty = 100;
    }

    // Calculate total timer counts per period: Timer_clock / frequency
    // Handle case where frequency is 0 or too high
    if (frequency == 0 || timer_clock / frequency == 0) {
        // Cannot achieve desired frequency. Set to 0% duty or minimum possible freq.
        // Setting ARR to max value and PSC to 0 gives min frequency with highest resolution.
        prescaler = 0;
        period = 0xFFFF; // Max for 16-bit timer. F410 has 16 and 32 bit timers. Using 16-bit max as safe default.
                         // For 32-bit timers (TIM2, TIM5), this could be 0xFFFFFFFF.
                         // A production driver might need to handle timer type explicitly.
        total_cycles = period + 1; // Use the max period value
         // if frequency is requested 0, maybe just turn it off? The spec doesn't say.
         // Let's set to lowest possible frequency with max resolution by setting ARR=Max, PSC=0.
         // If timer is 32-bit, use 0xFFFFFFFF. Let's use 0xFFFF for general case.
    } else {
         total_cycles = timer_clock / frequency;

         // Find suitable prescaler and period values
         // Aim to maximize period for better resolution while keeping total_cycles constant
         // We want (PSC + 1) * (ARR + 1) = total_cycles
         // Let's iterate prescaler from 0 until (ARR + 1) fits in 16 bits (or 32 bits if necessary)
         // Or, prioritize maximum ARR for better resolution:
         // ARR = (total_cycles / (PSC + 1)) - 1. Maximize ARR means minimize PSC.
         // Smallest PSC = 0. Largest ARR = total_cycles - 1. Check if this ARR fits.
         prescaler = 0;
         period = total_cycles - 1;

         // If the calculated period is too large for a 16-bit timer (0xFFFF = 65535)
         // and we assume 16-bit timers or want compatibility, increase prescaler.
         // Note: STM32F410 has TIM2/5 (32-bit) and TIM3/9/10/11 (16-bit).
         // This generic code assumes 16-bit unless total_cycles exceeds 0xFFFF.
         // A robust driver would use the timer width info from TRD_Channel_t.
         if (total_cycles > 0xFFFF + 1) { // If total_cycles is larger than 16-bit range + 1
             // Calculate required prescaler to bring ARR back into range
             prescaler = (total_cycles / (0xFFFF + 1));
             // Recalculate period with the new prescaler
             period = (total_cycles / (prescaler + 1)) - 1;
         }
         // Ensure minimum period for 1% duty cycle resolution
         // total_cycles must be at least 100 for 1% resolution if duty is 0-100%
         if (total_cycles < 100 && frequency != 0) {
            // Cannot achieve 1% resolution at this frequency.
            // Use minimum period of 99 (for 100 counts, 0 to 99) to allow 0-100 mapping.
            // Recalculate prescaler based on minimum period.
            period = 99;
            prescaler = (total_cycles / (period + 1));
            // Update total_cycles based on the minimum period and calculated prescaler
            total_cycles = (prescaler + 1) * (period + 1);
         }

         // Ensure calculated period fits into timer register width (assuming 16-bit unless 32-bit needed)
         // If TRD_Channel.TIMx is TIM2 or TIM5, period can be 32-bit.
         // For robustness without knowing timer type, cap at 16-bit max if needed,
         // or rely on total_cycles > 0xFFFF check above handling larger values.
         // Let's trust the calculation above to produce valid PSC/ARR up to 32-bit if total_cycles allows.

         // Check for potential overflow or division by zero if frequency is extremely high
         if ((prescaler + 1) * (period + 1) == 0) {
              prescaler = 0;
              period = 0;
              total_cycles = 1; // Prevent division by zero in pulse calculation
         }
    }


    // Calculate pulse value for duty cycle
    // Pulse = (Duty * (Period + 1)) / 100
    pulse = (duty * (period + 1)) / 100;

    // Update Timer registers
    // The timer should be disabled or an update event generated after changing PSC/ARR.
    // Since ARPE is enabled, changes take effect at the next update event.
    // We generate UG to apply changes immediately.
    TRD_Channel.TIMx->PSC = prescaler;
    TRD_Channel.TIMx->ARR = period; // ARR value is (total counts - 1)

    // Set the Capture/Compare Register value
    if (TRD_Channel.Channel == 1) {
        TRD_Channel.TIMx->CCR1 = pulse;
    } else if (TRD_Channel.Channel == 2) {
        TRD_Channel.TIMx->CCR2 = pulse;
    } else if (TRD_Channel.Channel == 3) {
        TRD_Channel.TIMx->CCR3 = pulse;
    } else if (TRD_Channel.Channel == 4) {
        TRD_Channel.TIMx->CCR4 = pulse;
    }
    // Note: Add error handling for invalid channel if necessary

    // Generate Update Event to load the new PSC, ARR, and CCR values immediately
    TRD_Channel.TIMx->EGR |= TIM_EGR_UG;

    // Optional: Wait for update flag to be set and clear it, if immediate update is critical
    // while ((TRD_Channel.TIMx->SR & TIM_SR_UIF) == 0);
    // TRD_Channel.TIMx->SR &= ~TIM_SR_UIF;
}

/*************************************************************************************************
* Function Name  : PWM_Start
* Description    : Enable and start PWM signal generation on the specified channel.
* Parameter      : TRD_Channel_t TRD_Channel - Structure containing channel configuration
* Return Value   : None
*************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel) {
    // Enable the timer counter
    TRD_Channel.TIMx->CR1 |= TIM_CR1_CEN;

    // Note: For advanced timers (TIM1/8, not on F410RC base model),
    // BDTR register's MOE (Master Output Enable) bit would also need to be set.
}

/*************************************************************************************************
* Function Name  : PWM_Stop
* Description    : Stop the PWM signal output on the specified channel.
* Parameter      : TRD_Channel_t TRD_Channel - Structure containing channel configuration
* Return Value   : None
*************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    // Disable the timer counter
    TRD_Channel.TIMx->CR1 &= ~TIM_CR1_CEN;

    // Optional: Set CCRx to 0 to force the output low immediately
    // This depends on desired behavior when stopped.
    if (TRD_Channel.Channel == 1) {
        TRD_Channel.TIMx->CCR1 = 0;
    } else if (TRD_Channel.Channel == 2) {
        TRD_Channel.TIMx->CCR2 = 0;
    } else if (TRD_Channel.Channel == 3) {
        TRD_Channel.TIMx->CCR3 = 0;
    } else if (TRD_Channel.Channel == 4) {
        TRD_Channel.TIMx->CCR4 = 0;
    }
    // Generate update event to apply CCRx=0 immediately
     TRD_Channel.TIMx->EGR |= TIM_EGR_UG;

    // Note: For advanced timers, clearing MOE in BDTR might also be desired.
}

/*************************************************************************************************
* Function Name  : PWM_PowerOff
* Description    : Disable all PWM peripherals and outputs to reduce power consumption.
* Parameter      : None
* Return Value   : None
*************************************************************************************************/
void PWM_PowerOff(void) {
    // Disable clocks for all Timers capable of PWM on STM32F410RC
    // TIM2, TIM3, TIM5 are on APB1
    // TIM9, TIM10, TIM11 are on APB2

    // Disable APB1 Timer clocks
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN);

    // Disable APB2 Timer clocks
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN);

    // Note: Disabling GPIO clocks is generally not done in peripheral power-off
    // functions as GPIOs might be shared. The system-level power management
    // or the application should handle GPIO power states if necessary.

    // Note: Timer configuration registers retain their values even after clock disable.
    // Counter value and prescaler are reset upon clock enable.
}