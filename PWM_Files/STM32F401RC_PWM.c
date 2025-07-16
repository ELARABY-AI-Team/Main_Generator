/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM driver for STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/**
 * @brief Global constant defining the assumed internal clock frequency for Timers on APB1.
 *        STM32F401RC APB1 bus can run up to 42MHz. If APB1 prescaler is > 1,
 *        timers on APB1 get a clock frequency of 2 * PCLK1 (e.g., 2 * 42MHz = 84MHz).
 *        This value is critical for accurate frequency calculations.
 *        Assumed PCLK1 = 42MHz, APB1 Prescaler > 1.
 */
#define TIM_APB1_CLOCK_FREQ    (84000000UL) /* Assumed Timer Clock (2x PCLK1) */

/**
 * @brief Global constant defining the assumed internal clock frequency for Timers on APB2.
 *        STM32F401RC APB2 bus can run up to 84MHz. If APB2 prescaler is > 1,
 *        timers on APB2 get a clock frequency of 2 * PCLK2 (e.g., 2 * 84MHz = 168MHz).
 *        This value is critical for accurate frequency calculations.
 *        Assumed PCLK2 = 84MHz, APB2 Prescaler > 1.
 */
#define TIM_APB2_CLOCK_FREQ    (168000000UL) /* Assumed Timer Clock (2x PCLK2) */

// Constant for duty cycle and frequency validation limits
#define PWM_DUTY_MIN            (0U)
#define PWM_DUTY_MAX            (100U)
#define PWM_FREQUENCY_MIN       (1UL)
#define PWM_FREQUENCY_MAX       (1000000UL) // Up to 1 MHz, adjust as needed based on clock/resolution

// Timer type definitions for internal logic
#define TIMER_TYPE_16BIT        (0U)
#define TIMER_TYPE_32BIT        (1U)
#define TIMER_TYPE_ADVANCED     (2U) // For TIM1 specific registers like BDTR

/**
 * @brief Array of configurations for each defined PWM channel.
 *        This mapping links the TRD_Channel_t enum to specific hardware registers and properties.
 *        TIM2 and TIM5 are reserved as per requirements and not included in this array for PWM use.
 *        - TIM_APB1_CLOCK_FREQ for TIM3, TIM4.
 *        - TIM_APB2_CLOCK_FREQ for TIM1, TIM9, TIM10, TIM11.
 */
static const TRD_Channel_Config_t PWM_Channel_Configs[] = {
    // TIM1 Channels (Advanced Timer, APB2)
    {TIM1_BASE, GPIOA_BASE, 8, 1, 1, (1UL << 0), (1UL << 0), TIMER_TYPE_ADVANCED}, // TRD_PWM_TIMER1_CH1 -> PA8 (AF1) /* Assumed PWM config - please verify */
    {TIM1_BASE, GPIOA_BASE, 9, 2, 1, (1UL << 0), (1UL << 0), TIMER_TYPE_ADVANCED}, // TRD_PWM_TIMER1_CH2 -> PA9 (AF1) /* Assumed PWM config - please verify */
    {TIM1_BASE, GPIOA_BASE, 10, 3, 1, (1UL << 0), (1UL << 0), TIMER_TYPE_ADVANCED},// TRD_PWM_TIMER1_CH3 -> PA10 (AF1) /* Assumed PWM config - please verify */
    {TIM1_BASE, GPIOA_BASE, 11, 4, 1, (1UL << 0), (1UL << 0), TIMER_TYPE_ADVANCED},// TRD_PWM_TIMER1_CH4 -> PA11 (AF1) /* Assumed PWM config - please verify */

    // TIM3 Channels (General Purpose, 16-bit, APB1)
    {TIM3_BASE, GPIOA_BASE, 6, 1, 2, (1UL << 0), (1UL << 1), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER3_CH1 -> PA6 (AF2) /* Assumed PWM config - please verify */
    {TIM3_BASE, GPIOA_BASE, 7, 2, 2, (1UL << 0), (1UL << 1), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER3_CH2 -> PA7 (AF2) /* Assumed PWM config - please verify */
    {TIM3_BASE, GPIOB_BASE, 0, 3, 2, (1UL << 1), (1UL << 1), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER3_CH3 -> PB0 (AF2) /* Assumed PWM config - please verify */
    {TIM3_BASE, GPIOB_BASE, 1, 4, 2, (1UL << 1), (1UL << 1), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER3_CH4 -> PB1 (AF2) /* Assumed PWM config - please verify */

    // TIM4 Channels (General Purpose, 16-bit, APB1)
    {TIM4_BASE, GPIOB_BASE, 6, 1, 2, (1UL << 1), (1UL << 2), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER4_CH1 -> PB6 (AF2) /* Assumed PWM config - please verify */
    {TIM4_BASE, GPIOB_BASE, 7, 2, 2, (1UL << 1), (1UL << 2), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER4_CH2 -> PB7 (AF2) /* Assumed PWM config - please verify */
    {TIM4_BASE, GPIOB_BASE, 8, 3, 2, (1UL << 1), (1UL << 2), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER4_CH3 -> PB8 (AF2) /* Assumed PWM config - please verify */
    {TIM4_BASE, GPIOB_BASE, 9, 4, 2, (1UL << 1), (1UL << 2), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER4_CH4 -> PB9 (AF2) /* Assumed PWM config - please verify */

    // TIM9 Channels (General Purpose, 16-bit, APB2)
    {TIM9_BASE, GPIOA_BASE, 2, 1, 3, (1UL << 0), (1UL << 16), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER9_CH1 -> PA2 (AF3) /* Assumed PWM config - please verify */
    {TIM9_BASE, GPIOA_BASE, 3, 2, 3, (1UL << 0), (1UL << 16), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER9_CH2 -> PA3 (AF3) /* Assumed PWM config - please verify */

    // TIM10 Channel (General Purpose, 16-bit, APB2)
    {TIM10_BASE, GPIOB_BASE, 8, 1, 3, (1UL << 1), (1UL << 17), TIMER_TYPE_16BIT}, // TRD_PWM_TIMER10_CH1 -> PB8 (AF3) /* Assumed PWM config - please verify */
                                                                                   // Note: PB8 is also used by TIM4_CH3 (AF2). Only one Alternate Function can be active at a time for a given pin.

    // TIM11 Channel (General Purpose, 16-bit, APB2)
    {TIM11_BASE, GPIOB_BASE, 9, 1, 3, (1UL << 1), (1UL << 18), TIMER_TYPE_16BIT}  // TRD_PWM_TIMER11_CH1 -> PB9 (AF3) /* Assumed PWM config - please verify */
                                                                                   // Note: PB9 is also used by TIM4_CH4 (AF2). Only one Alternate Function can be active at a time for a given pin.
};

/*
 * Timer Reservation Note:
 * As per requirements, Timers 2 and 5 are reserved and excluded from PWM implementation.
 * These timers can be utilized for other system functionalities such as OS ticks,
 * delay functions, or other general-purpose timing requirements to avoid conflicts.
 *
 * Timer 2 is a 32-bit general-purpose timer on APB1.
 * Timer 5 is a 32-bit general-purpose timer on APB1.
 */

/***********************************************************************************************************************
* Function Prototypes (Internal Helper Functions)
***********************************************************************************************************************/
static void PWM_GPIO_Init(const TRD_Channel_Config_t *config);
static void PWM_Timer_Clock_Enable(const TRD_Channel_Config_t *config);
static unsigned long Get_Timer_Input_Clock(const TRD_Channel_Config_t *config);
static tlong* Get_CCR_Register_Pointer(volatile tlong *TIM_Base, tbyte channel);

/***********************************************************************************************************************
* Functions ===========================================================================
***********************************************************************************************************************/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The specific PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        // Invalid channel, handle error or return
        return;
    }

    const TRD_Channel_Config_t *config = &PWM_Channel_Configs[TRD_Channel];

    // 1. Enable Timer Clock
    PWM_Timer_Clock_Enable(config);

    // 2. Initialize GPIO for Alternate Function
    PWM_GPIO_Init(config);

    // 3. Configure Timer Time-base (Prescaler, Auto-reload, Counter Mode)
    // Disable counter before configuration
    TIM_CR1(config->TIMx_Base) &= ~TIM_CR1_CEN; /* PDF Reference */

    // Set counter mode to Upcounting (CMS=00, DIR=0)
    TIM_CR1(config->TIMx_Base) &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR); /* PDF Reference */
    TIM_CR1(config->TIMx_Base) |= TIM_CR1_ARPE; // Enable auto-reload preload enable /* PDF Reference */

    // Set prescaler and ARR to default/reset values. They will be configured later by PWM_Set_Freq.
    // Setting a default ARR > 0 is important to prevent blocking the counter.
    if (config->TimerType == TIMER_TYPE_32BIT) {
         TIM_ARR(config->TIMx_Base) = 0xFFFFFFFFUL; // Max 32-bit /* PDF Reference */
    } else {
         TIM_ARR(config->TIMx_Base) = 0xFFFFUL;     // Max 16-bit /* PDF Reference */
    }
    TIM_PSC(config->TIMx_Base) = 0; /* PDF Reference */

    // 4. Configure Capture/Compare Channel for PWM Output
    volatile tlong *ccmr_reg = (config->TIM_Channel <= 2) ? TIM_CCMR1(config->TIMx_Base) : TIM_CCMR2(config->TIMx_Base);
    tbyte ccmr_pos_offset = ((config->TIM_Channel - 1) % 2) * 8; // 0 for CH1/3, 8 for CH2/4

    // Clear CCxS bits (select output mode)
    *ccmr_reg &= ~(TIM_CCMR1_CC1S_Msk << ccmr_pos_offset); /* PDF Reference */

    // Set Output Compare Mode to PWM Mode 1 (OCxM=110)
    *ccmr_reg &= ~(TIM_CCMR1_OC1M_Msk << ccmr_pos_offset); /* PDF Reference */
    *ccmr_reg |= (6UL << (TIM_CCMR1_OC1M_Pos + ccmr_pos_offset)); /* PDF Reference */ // 6 (110) for PWM Mode 1

    // Enable Output Compare Preload Enable (OCxPE=1)
    *ccmr_reg |= (1UL << (TIM_CCMR1_OC1PE_Pos + ccmr_pos_offset)); /* PDF Reference */

    // 5. Configure Output Enable Register (CCER)
    volatile tlong *ccer_reg = TIM_CCER(config->TIMx_Base);
    tbyte ccer_pos_offset = (config->TIM_Channel - 1) * 4; // 0 for CH1, 4 for CH2, 8 for CH3, 12 for CH4

    // Enable Capture/Compare Output (CCxE=1) and set polarity (CCxP=0, active high)
    *ccer_reg &= ~(TIM_CCER_CC1P << ccer_pos_offset); // Set polarity to active high (0) /* PDF Reference */
    *ccer_reg |= (TIM_CCER_CC1E << ccer_pos_offset);  // Enable output /* PDF Reference */

    // For TIM1 (Advanced Timer), enable Main Output Enable (MOE) bit in BDTR
    if (config->TimerType == TIMER_TYPE_ADVANCED) {
        TIM_BDTR(config->TIMx_Base) |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // 6. Generate an Update event to load all preload registers (ARPE, OCxPE) /* PDF Reference */
    TIM_EGR(config->TIMx_Base) |= TIM_EGR_UG; /* PDF Reference */

    // Optionally clear update flag after generation
    TIM_SR(config->TIMx_Base) &= ~(1UL << 0); // Clear UIF flag /* PDF Reference */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT || frequency < PWM_FREQUENCY_MIN || duty > PWM_DUTY_MAX) {
        // Input validation failed
        return;
    }

    const TRD_Channel_Config_t *config = &PWM_Channel_Configs[TRD_Channel];
    unsigned long ulTimerClock = Get_Timer_Input_Clock(config);

    // Determine max ARR value based on timer type
    unsigned long ulARR_Max;
    if (config->TimerType == TIMER_TYPE_32BIT) {
        ulARR_Max = 0xFFFFFFFFUL; // 32-bit timer
    } else {
        ulARR_Max = 0xFFFFUL;     // 16-bit timer
    }

    // Calculate Prescaler and ARR values
    unsigned long ulPrescaler = 0;
    unsigned long ulARR = 0;
    unsigned long ulPulse = 0;

    // The formula for frequency is Timer_Clock / ((PSC + 1) * (ARR + 1))
    // We want to maximize ARR for better resolution, so first try with minimum PSC (PSC = 0).
    ulARR = (ulTimerClock / frequency) - 1;

    if (ulARR > ulARR_Max) {
        // If ARR exceeds max value, calculate prescaler
        ulPrescaler = (ulTimerClock / (frequency * (ulARR_Max + 1))) + 1; // +1 to ensure division doesn't truncate too much
        
        // Ensure prescaler does not exceed 16-bit max
        if (ulPrescaler > 0xFFFFUL) {
            ulPrescaler = 0xFFFFUL; /* PDF Reference */
        }
        ulARR = (ulTimerClock / (frequency * (ulPrescaler + 1))) - 1;
    }
    
    // Ensure ARR does not exceed max value
    if (ulARR > ulARR_Max) {
        ulARR = ulARR_Max;
    }

    // Calculate Pulse value based on duty cycle
    ulPulse = (ulARR + 1) * duty / 100;

    // Apply the calculated values to the timer registers
    // Ensure counter is disabled before writing PSC and ARR for robustness, then re-enable
    TIM_CR1(config->TIMx_Base) &= ~TIM_CR1_CEN; /* PDF Reference */

    TIM_PSC(config->TIMx_Base) = ulPrescaler; /* PDF Reference */
    TIM_ARR(config->TIMx_Base) = ulARR;       /* PDF Reference */

    // Update the Capture Compare Register
    volatile tlong *ccr_reg_ptr = Get_CCR_Register_Pointer(config->TIMx_Base, config->TIM_Channel);
    if (ccr_reg_ptr != (tlong*)0) {
        *ccr_reg_ptr = ulPulse; /* PDF Reference */
    }

    // Generate an Update event to load new prescaler, ARR, and CCR values into shadow registers
    TIM_EGR(config->TIMx_Base) |= TIM_EGR_UG; /* PDF Reference */
    // Clear update interrupt flag just in case
    TIM_SR(config->TIMx_Base) &= ~(1UL << 0); /* PDF Reference */

    // Re-enable the counter (if it was previously enabled)
    // This function assumes PWM_Start will be called to fully enable.
    // For now, re-enable if it was enabled. However, PWM_Start handles the final enable.
    // For production-ready, it's safer for Set_Freq to not implicitly start.
    // If we want it to apply immediately even if running, remove CEN disable/enable.
    // But generating UG means counter is reset, so it effectively restarts.
    // Let's ensure CEN is reset and then re-enabled if it was running.
    // Simpler: Just rely on PWM_Start/Stop for CEN control.
    // The UG bit sets CEN automatically if in Trigger mode (SMS=110), but usually not in normal PWM.
    // For production, if system relies on continuous PWM, ensure to call PWM_Start after Set_Freq.
    // For this context, we will not automatically re-enable CEN here, it is controlled by PWM_Start.
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return;
    }

    const TRD_Channel_Config_t *config = &PWM_Channel_Configs[TRD_Channel];

    // Enable the counter
    TIM_CR1(config->TIMx_Base) |= TIM_CR1_CEN; /* PDF Reference */

    // For TIM1 (Advanced Timer), enable Main Output Enable (MOE) bit in BDTR
    if (config->TimerType == TIMER_TYPE_ADVANCED) {
        TIM_BDTR(config->TIMx_Base) |= TIM_BDTR_MOE; /* PDF Reference */
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return;
    }

    const TRD_Channel_Config_t *config = &PWM_Channel_Configs[TRD_Channel];

    // Disable the counter
    TIM_CR1(config->TIMx_Base) &= ~TIM_CR1_CEN; /* PDF Reference */

    // For TIM1 (Advanced Timer), disable Main Output Enable (MOE) bit in BDTR
    if (config->TimerType == TIMER_TYPE_ADVANCED) {
        TIM_BDTR(config->TIMx_Base) &= ~TIM_BDTR_MOE; /* PDF Reference */
    }

    // Optionally set duty cycle to 0 to ensure output is low (or high depending on polarity)
    // This avoids floating or undefined state, but actual output depends on GPIO configuration
    // (e.g., if set to AF, the timer is responsible for outputting low/high based on MOE)
    volatile tlong *ccr_reg_ptr = Get_CCR_Register_Pointer(config->TIMx_Base, config->TIM_Channel);
    if (ccr_reg_ptr != (tlong*)0) {
        *ccr_reg_ptr = 0; // Set duty to 0 /* PDF Reference */
    }
    // Generate an Update event to apply CCRx = 0 immediately
    TIM_EGR(config->TIMx_Base) |= TIM_EGR_UG; /* PDF Reference */
    TIM_SR(config->TIMx_Base) &= ~(1UL << 0); /* PDF Reference */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function resets all timers and GPIOs configured for PWM.
 *        Note: This is a destructive operation that will reset other functionalities
 *        if the same peripherals/pins are used for non-PWM purposes.
 *        It assumes the reserved timers (TIM2, TIM5) are not affected.
 */
void PWM_PowerOff(void) {
    // Loop through all possible PWM channels and disable their peripherals
    for (tlong i = 0; i < TRD_PWM_CHANNEL_COUNT; i++) {
        const TRD_Channel_Config_t *config = &PWM_Channel_Configs[i];

        // 1. Disable the Timer Counter
        TIM_CR1(config->TIMx_Base) &= ~TIM_CR1_CEN; /* PDF Reference */

        // 2. Disable Main Output Enable for TIM1
        if (config->TimerType == TIMER_TYPE_ADVANCED) {
            TIM_BDTR(config->TIMx_Base) &= ~TIM_BDTR_MOE; /* PDF Reference */
        }

        // 3. Reset GPIO pin to Input Floating mode and clear AF
        unsigned int moder_shift = config->GPIO_Pin * 2;
        unsigned int afr_shift = (config->GPIO_Pin % 8) * 4;
        volatile tlong *gpio_afr_reg = (config->GPIO_Pin < 8) ? GPIOX_AFRL(config->GPIOx_Base) : GPIOX_AFRH(config->GPIOx_Base);

        // Clear AF selection
        *gpio_afr_reg &= ~(0xFUL << afr_shift); /* PDF Reference */

        // Set MODER to Input (00)
        GPIOX_MODER(config->GPIOx_Base) &= ~(0x3UL << moder_shift); /* PDF Reference */

        // Clear OTYPER, OSPEEDR, PUPDR to reset state (Push-pull, Low speed, No pull)
        GPIOX_OTYPER(config->GPIOx_Base) &= ~(1UL << config->GPIO_Pin);   /* PDF Reference */
        GPIOX_OSPEEDR(config->GPIOx_Base) &= ~(0x3UL << moder_shift); /* PDF Reference */
        GPIOX_PUPDR(config->GPIOx_Base) &= ~(0x3UL << moder_shift);   /* PDF Reference */

        // 4. Disable Timer peripheral clock
        if (config->TIMx_Base == TIM1_BASE || config->TIMx_Base == TIM9_BASE || 
            config->TIMx_Base == TIM10_BASE || config->TIMx_Base == TIM11_BASE) {
            // APB2 Timer
            RCC_APB2ENR &= ~config->RCC_TIM_EN_Bit; /* Assumed Clock Control - please verify */
            RCC_APB2RSTR |= config->RCC_TIM_EN_Bit; // Assert reset /* Assumed Reset Control - please verify */
            RCC_APB2RSTR &= ~config->RCC_TIM_EN_Bit; // Release reset /* Assumed Reset Control - please verify */
        } else {
            // APB1 Timer (TIM3, TIM4)
            RCC_APB1ENR &= ~config->RCC_TIM_EN_Bit; /* Assumed Clock Control - please verify */
            RCC_APB1RSTR |= config->RCC_TIM_EN_Bit; // Assert reset /* Assumed Reset Control - please verify */
            RCC_APB1RSTR &= ~config->RCC_TIM_EN_Bit; // Release reset /* Assumed Reset Control - please verify */
        }

        // 5. Disable GPIO peripheral clock - careful, only if this GPIO is no longer used by ANY active peripheral
        // For robustness in a shared environment, it's better to keep GPIO clock enabled
        // unless a global GPIO PowerOff is required and all pins are known to be idle.
        // For this task, we will disable only if no other active PWM channel uses the same port.
        // but a warning for real-world scenarios is important.
        // As disabling GPIO clocks might affect other peripherals using the same port,
        // it's generally avoided in a mixed-peripheral power-off.
        // Keeping GPIO clocks enabled unless explicitly designed for full system shutdown.
        // For current scope: not disabling GPIO clocks at this level.
    }
}

/***********************************************************************************************************************
* Internal Helper Functions
***********************************************************************************************************************/

/**
 * @brief Initializes GPIO pin for Alternate Function mode for a specific PWM channel.
 * @param config Pointer to the channel configuration.
 */
static void PWM_GPIO_Init(const TRD_Channel_Config_t *config) {
    // 1. Enable GPIO Port Clock (e.g., RCC_AHB1ENR_GPIOAEN)
    if (config->GPIOx_Base == GPIOA_BASE) {
        RCC_AHB1ENR |= (1UL << 0); // GPIOA clock enable /* PDF Reference */
    } else if (config->GPIOx_Base == GPIOB_BASE) {
        RCC_AHB1ENR |= (1UL << 1); // GPIOB clock enable /* PDF Reference */
    } else if (config->GPIOx_Base == GPIOC_BASE) {
        RCC_AHB1ENR |= (1UL << 2); // GPIOC clock enable /* PDF Reference */
    }
    // Add other GPIO ports if used and enabled here

    // Ensure clock is stable (dummy read or small delay)
    volatile tlong dummy_read = RCC_AHB1ENR;
    (void)dummy_read; // Suppress unused variable warning

    unsigned int moder_shift = config->GPIO_Pin * 2;
    unsigned int afr_shift = (config->GPIO_Pin % 8) * 4; // Shift for AFRL (pins 0-7) or AFRH (pins 8-15)

    // 2. Configure GPIO pin mode to Alternate Function mode (MODER = 10)
    GPIOX_MODER(config->GPIOx_Base) &= ~(0x3UL << moder_shift); /* PDF Reference */ // Clear bits
    GPIOX_MODER(config->GPIOx_Base) |= (0x2UL << moder_shift);  /* PDF Reference */ // Set to Alternate Function (10)

    // 3. Configure GPIO output type to Push-Pull (OTYPER = 0)
    GPIOX_OTYPER(config->GPIOx_Base) &= ~(1UL << config->GPIO_Pin); /* PDF Reference */ // Set to Push-Pull (0)

    // 4. Configure GPIO output speed to High Speed (OSPEEDR = 10)
    GPIOX_OSPEEDR(config->GPIOx_Base) &= ~(0x3UL << moder_shift); /* PDF Reference */ // Clear bits
    GPIOX_OSPEEDR(config->GPIOx_Base) |= (0x2UL << moder_shift);  /* PDF Reference */ // Set to High Speed (10)

    // 5. Configure GPIO Pull-up/Pull-down to No Pull-up/Pull-down (PUPDR = 00)
    GPIOX_PUPDR(config->GPIOx_Base) &= ~(0x3UL << moder_shift); /* PDF Reference */ // Clear bits (sets to No Pull-up/Pull-down)

    // 6. Configure GPIO Alternate Function selection
    volatile tlong *gpio_afr_reg = (config->GPIO_Pin < 8) ? GPIOX_AFRL(config->GPIOx_Base) : GPIOX_AFRH(config->GPIOx_Base);
    
    *gpio_afr_reg &= ~(0xFUL << afr_shift);                   // Clear AF bits (4 bits per pin) /* PDF Reference */
    *gpio_afr_reg |= ((tlong)config->AF_Value << afr_shift); // Set AF value /* PDF Reference */
}

/**
 * @brief Enables the clock for the specified Timer peripheral.
 * @param config Pointer to the channel configuration.
 */
static void PWM_Timer_Clock_Enable(const TRD_Channel_Config_t *config) {
    if (config->TIMx_Base == TIM1_BASE || config->TIMx_Base == TIM9_BASE ||
        config->TIMx_Base == TIM10_BASE || config->TIMx_Base == TIM11_BASE) {
        // These timers are on APB2 bus
        RCC_APB2ENR |= config->RCC_TIM_EN_Bit; /* Assumed Clock Control - please verify */
        // Ensure clock is stable (dummy read)
        volatile tlong dummy_read = RCC_APB2ENR;
        (void)dummy_read; // Suppress unused variable warning
    } else {
        // These timers are on APB1 bus (TIM3, TIM4)
        RCC_APB1ENR |= config->RCC_TIM_EN_Bit; /* Assumed Clock Control - please verify */
        // Ensure clock is stable (dummy read)
        volatile tlong dummy_read = RCC_APB1ENR;
        (void)dummy_read; // Suppress unused variable warning
    }
}

/**
 * @brief Gets the input clock frequency for the specified timer.
 *        This relies on the global constants TIM_APB1_CLOCK_FREQ and TIM_APB2_CLOCK_FREQ.
 * @param config Pointer to the channel configuration.
 * @return The timer input clock frequency in Hz.
 */
static unsigned long Get_Timer_Input_Clock(const TRD_Channel_Config_t *config) {
    if (config->TIMx_Base == TIM1_BASE || config->TIMx_Base == TIM9_BASE ||
        config->TIMx_Base == TIM10_BASE || config->TIMx_Base == TIM11_BASE) {
        return TIM_APB2_CLOCK_FREQ;
    } else {
        return TIM_APB1_CLOCK_FREQ;
    }
}

/**
 * @brief Returns a pointer to the correct Capture/Compare Register (CCR) based on the channel.
 * @param TIM_Base The base address of the timer.
 * @param channel The channel number (1-4).
 * @return Pointer to the CCR register, or NULL if invalid channel.
 */
static tlong* Get_CCR_Register_Pointer(volatile tlong *TIM_Base, tbyte channel) {
    switch (channel) {
        case 1: return TIM_CCR1(TIM_Base);
        case 2: return TIM_CCR2(TIM_Base);
        case 3: return TIM_CCR3(TIM_Base);
        case 4: return TIM_CCR4(TIM_Base);
        default: return (tlong*)0; // Invalid channel
    }
}