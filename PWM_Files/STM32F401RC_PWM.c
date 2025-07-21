/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file provides the production-ready implementation for PWM functionalities
*                  on the STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/*
 * @brief Assumed system clock configuration for timer frequency calculation.
 *        This implementation assumes a System Clock (SYSCLK) of 84 MHz.
 *        It also assumes APB1 Prescaler = 2 and APB2 Prescaler = 1.
 *        Under these assumptions:
 *        - PCLK1 (APB1 peripheral clock) = 42 MHz.
 *        - PCLK2 (APB2 peripheral clock) = 84 MHz.
 *        For timers connected to APB1 (TIM2, TIM3, TIM4, TIM5), if APB1 prescaler is > 1,
 *        the timer clock is PCLK1 * 2 = 84 MHz.
 *        For timers connected to APB2 (TIM1, TIM9, TIM10, TIM11), if APB2 prescaler is > 1,
 *        the timer clock is PCLK2 * 2. If APB2 prescaler is 1, timer clock is PCLK2 = 84 MHz.
 *        Given the assumed prescalers, all timers used here (TIM3, TIM4, TIM5, TIM9, TIM10, TIM11)
 *        will effectively run at 84 MHz.
 */
#define SYSTEM_TIMER_CLOCK_FREQ_HZ      84000000UL


/*
 * @brief Internal typedefs for common data types.
 *        Assumed to be defined in a global types header or by the compiler for standard types.
 */
typedef uint32_t tlong;
typedef uint8_t tbyte;

/*
 * @brief Structure to define the configuration for a single PWM channel.
 */
typedef struct
{
    TIM_TypeDef*    TIMx;                  /**< Pointer to the Timer peripheral (e.g., TIM3) */
    uint8_t         ChannelNumber;         /**< Timer channel number (1, 2, 3, or 4) */
    GPIO_TypeDef*   PortName;              /**< Pointer to the GPIO port (e.g., GPIOA) */
    uint8_t         PinNumber;             /**< GPIO pin number (0-15) */
    uint8_t         AlternateFunctionNumber; /**< Alternate function number for the GPIO pin */
} PWM_Channel_Config_t;


/*
 * @brief Array mapping logical PWM channels (TRD_Channel_t) to specific hardware configurations.
 *        This array defines the timer, channel, GPIO port, pin, and alternate function for each
 *        available PWM output.
 *
 * @note  Timers TIM1 and TIM2 are intentionally excluded and reserved for potential
 *        OS / system timekeeping / delay functionalities as per requirements.
 * @note  Pin 0 for any port is excluded to avoid potential conflicts or for specific board
 *        design considerations unless explicitly confirmed as PWM-capable and required.
 * @note  Alternate function numbers (AF) are specific to STM32F401RC and are taken from
 *        the device datasheet (e.g., AF2 for TIM3/4/5, AF3 for TIM9/10/11).
 * @note  All `GPIO_TypeDef*` values like `GPIOA`, `GPIOB`, etc., are assumed to be defined
 *        in `stm32f401xc.h` or similar device-specific header included via
 *        `STM32F401RC_PWM.h`.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    /* TIM3 Channels - AF2 */
    { TIM3, 1, GPIOA, 6,  2 }, /* TIM3_CH1 on PA6 */
    { TIM3, 2, GPIOA, 7,  2 }, /* TIM3_CH2 on PA7 */
    { TIM3, 4, GPIOB, 1,  2 }, /* TIM3_CH4 on PB1 (PB0 excluded) */

    /* TIM4 Channels - AF2 */
    { TIM4, 1, GPIOB, 6,  2 }, /* TIM4_CH1 on PB6 */
    { TIM4, 2, GPIOB, 7,  2 }, /* TIM4_CH2 on PB7 */
    { TIM4, 3, GPIOB, 8,  2 }, /* TIM4_CH3 on PB8 */
    { TIM4, 4, GPIOB, 9,  2 }, /* TIM4_CH4 on PB9 */

    /* TIM5 Channels - AF2 */
    { TIM5, 2, GPIOA, 1,  2 }, /* TIM5_CH2 on PA1 (PA0 excluded) */
    { TIM5, 3, GPIOA, 2,  2 }, /* TIM5_CH3 on PA2 */
    { TIM5, 4, GPIOA, 3,  2 }, /* TIM5_CH4 on PA3 */

    /* TIM9 Channels - AF3 */
    { TIM9, 1, GPIOE, 5,  3 }, /* TIM9_CH1 on PE5 */
    { TIM9, 2, GPIOE, 6,  3 }, /* TIM9_CH2 on PE6 */

    /* TIM10 Channels - AF3 */
    { TIM10, 1, GPIOB, 8,  3 }, /* TIM10_CH1 on PB8 (Note: PB8 can also be TIM4_CH3, ensure no runtime conflict if both are used without careful management) */

    /* TIM11 Channels - AF3 */
    { TIM11, 1, GPIOB, 9,  3 }  /* TIM11_CH1 on PB9 (Note: PB9 can also be TIM4_CH4, ensure no runtime conflict if both are used without careful management) */
};


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 *        This includes enabling clocks for the timer and GPIO, configuring
 *        the GPIO pin for alternate function, and setting up the timer in PWM mode.
 * @param TRD_Channel The logical PWM channel to initialize (index into pwm_channel_map).
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT)
    {
        /* Invalid channel, do nothing */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    GPIO_TypeDef* PortName = config->PortName;
    uint8_t PinNumber = config->PinNumber;
    uint8_t AF_Number = config->AlternateFunctionNumber;
    uint8_t ChannelNumber = config->ChannelNumber;

    /* 1. Enable Clocks for GPIO and Timer */
    /* Enable GPIO Port Clock (matches HAL's __HAL_RCC_GPIOx_CLK_ENABLE() functionality) */
    if (PortName == GPIOA)
    {
        RCC->AHB1ENR |= (1UL << 0); /* GPIOAEN */
    }
    else if (PortName == GPIOB)
    {
        RCC->AHB1ENR |= (1UL << 1); /* GPIOBEN */
    }
    else if (PortName == GPIOC)
    {
        RCC->AHB1ENR |= (1UL << 2); /* GPIOCEN */
    }
    else if (PortName == GPIOD)
    {
        RCC->AHB1ENR |= (1UL << 3); /* GPIODEN */
    }
    else if (PortName == GPIOE)
    {
        RCC->AHB1ENR |= (1UL << 4); /* GPIOEEN */
    }

    /* Enable Timer Clock (matches HAL's __HAL_RCC_TIMx_CLK_ENABLE() functionality) */
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
    {
        /* APB2 Timers */
        if (TIMx == TIM1)  RCC->APB2ENR |= (1UL << 0);  /* TIM1EN */
        if (TIMx == TIM9)  RCC->APB2ENR |= (1UL << 18); /* TIM9EN */
        if (TIMx == TIM10) RCC->APB2ENR |= (1UL << 17); /* TIM10EN */
        if (TIMx == TIM11) RCC->APB2ENR |= (1UL << 16); /* TIM11EN */
    }
    else if (TIMx == TIM2 || TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5)
    {
        /* APB1 Timers */
        if (TIMx == TIM2) RCC->APB1ENR |= (1UL << 0);  /* TIM2EN */
        if (TIMx == TIM3) RCC->APB1ENR |= (1UL << 1);  /* TIM3EN */
        if (TIMx == TIM4) RCC->APB1ENR |= (1UL << 2);  /* TIM4EN */
        if (TIMx == TIM5) RCC->APB1ENR |= (1UL << 3);  /* TIM5EN */
    }

    /* 2. Configure GPIO Pin for Alternate Function (matches HAL_GPIO_Init functionality) */
    /* Clear and Set MODER for Alternate Function Mode (AF) */
    PortName->MODER &= ~(3UL << (PinNumber * 2));
    PortName->MODER |= (2UL << (PinNumber * 2)); /* 10 for AF mode */

    /* Clear and Set OTYPER for Push-Pull Output Type */
    PortName->OTYPER &= ~(1UL << PinNumber); /* 0 for Push-Pull */

    /* Clear and Set OSPEEDR for High Speed */
    PortName->OSPEEDR &= ~(3UL << (PinNumber * 2));
    PortName->OSPEEDR |= (3UL << (PinNumber * 2)); /* 11 for High Speed */

    /* Clear and Set PUPDR for No Pull-up/Pull-down */
    PortName->PUPDR &= ~(3UL << (PinNumber * 2)); /* 00 for No Pull-up/Pull-down */

    /* Configure Alternate Function Low/High Register (AFRL/AFRH) */
    if (PinNumber < 8)
    {
        PortName->AFRL &= ~(0xFUL << (PinNumber * 4)); /* Clear 4 bits for AF */
        PortName->AFRL |= (AF_Number << (PinNumber * 4)); /* Set AF value */
    }
    else
    {
        PortName->AFRH &= ~(0xFUL << ((PinNumber - 8) * 4)); /* Clear 4 bits for AF */
        PortName->AFRH |= (AF_Number << ((PinNumber - 8) * 4)); /* Set AF value */
    }

    /* 3. Configure Timer for PWM Mode */
    /* Disable timer to configure it - good practice, similar to HAL state transitions */
    TIMx->CR1 &= ~TIM_CR1_CEN;

    /* Set PWM Mode 1 (output compare mode: OCxM = 110) */
    /* Enable output compare preload (OCxPE = 1) */
    switch (ChannelNumber)
    {
        case 1:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
            TIMx->CCMR1 |= (6UL << TIM_CCMR1_OC1M_Pos); /* PWM Mode 1 (matches TIM_OCMODE_PWM1) */
            TIMx->CCMR1 |= TIM_CCMR1_OC1PE; /* Output compare preload enable (matches OCxPE=1) */
            // Note: HAL also sets OCFastMode here, but for basic PWM, preload is usually sufficient.
            break;
        case 2:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
            TIMx->CCMR1 |= (6UL << TIM_CCMR1_OC2M_Pos); /* PWM Mode 1 */
            TIMx->CCMR1 |= TIM_CCMR1_OC2PE; /* Output compare preload enable */
            break;
        case 3:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
            TIMx->CCMR2 |= (6UL << TIM_CCMR2_OC3M_Pos); /* PWM Mode 1 */
            TIMx->CCMR2 |= TIM_CCMR2_OC3PE; /* Output compare preload enable */
            break;
        case 4:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
            TIMx->CCMR2 |= (6UL << TIM_CCMR2_OC4M_Pos); /* PWM Mode 1 */
            TIMx->CCMR2 |= TIM_CCMR2_OC4PE; /* Output compare preload enable */
            break;
        default:
            break; /* Should not happen with validation above */
    }

    /* Enable capture/compare output and set output polarity to active high */
    // This is equivalent to HAL's TIM_CCxChannelCmd(TIMx, Channel, TIM_CCx_ENABLE) and setting OCPolarity
    switch (ChannelNumber)
    {
        case 1:
            TIMx->CCER |= TIM_CCER_CC1E; /* Capture/Compare 1 output enable */
            TIMx->CCER &= ~TIM_CCER_CC1P; /* OC1 polarity: active high (matches TIM_OCPOLARITY_HIGH) */
            break;
        case 2:
            TIMx->CCER |= TIM_CCER_CC2E; /* Capture/Compare 2 output enable */
            TIMx->CCER &= ~TIM_CCER_CC2P; /* OC2 polarity: active high */
            break;
        case 3:
            TIMx->CCER |= TIM_CCER_CC3E; /* Capture/Compare 3 output enable */
            TIMx->CCER &= ~TIM_CCER_CC3P; /* OC3 polarity: active high */
            break;
        case 4:
            TIMx->CCER |= TIM_CCER_CC4E; /* Capture/Compare 4 output enable */
            TIMx->CCER &= ~TIM_CCER_CC4P; /* OC4 polarity: active high */
            break;
        default:
            break;
    }

    /* Enable auto-reload preload (ARPE) - matches HAL's AutoReloadPreload setting */
    TIMx->CR1 |= TIM_CR1_ARPE;

    /* Set default values (e.g., 1kHz, 50% duty cycle) if not explicitly set by Set_Freq */
    /* Set initial PSC and ARR for good resolution (e.g., PSC=0, ARR=Max) */
    TIMx->PSC = 0;
    TIMx->ARR = 0xFFFF; // Max period for good initial resolution

    /* Generate an update event to load the Prescaler and ARR values (matches HAL's TIM_EGR_UG) */
    TIMx->EGR |= TIM_EGR_UG;

    /* Clear the update interrupt flag (optional, but good practice, also done in HAL's IRQ handler) */
    TIMx->SR &= ~TIM_SR_UIF;

    /* Do not set Main Output Enable (MOE) here, `PWM_Start` will handle it for advanced timers.
     * This aligns with HAL's pattern where MOE is enabled/disabled with Start/Stop functions. */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The logical PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT || duty > 100 || frequency == 0)
    {
        /* Invalid channel or parameters */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t ChannelNumber = config->ChannelNumber;

    uint32_t timer_clock = SYSTEM_TIMER_CLOCK_FREQ_HZ;
    uint32_t period_counts;
    uint16_t prescaler_val;
    uint16_t arr_val;
    uint32_t ccr_val;

    /* Calculate ARR and Prescaler */
    /* Ensure the period_counts doesn't exceed 32-bit before division by frequency */
    period_counts = timer_clock / frequency;

    if (period_counts > 65535UL) /* If period exceeds 16-bit max, use prescaler */
    {
        /* Calculate prescaler to bring period_counts within 16-bit range */
        prescaler_val = (uint16_t)((period_counts / 65535UL) + 1);
        arr_val = (uint16_t)((period_counts / prescaler_val) - 1);
    }
    else /* No prescaler needed */
    {
        prescaler_val = 0;
        arr_val = (uint16_t)(period_counts - 1);
    }

    /* Set Prescaler and Auto-Reload Register */
    TIMx->PSC = prescaler_val;
    TIMx->ARR = arr_val;

    /* Calculate Capture/Compare Register value for duty cycle */
    /* CCRx = (ARR + 1) * duty / 100. This calculation is standard for PWM duty cycle. */
    ccr_val = ((uint32_t)arr_val + 1) * duty / 100;

    /* Set Capture/Compare Register */
    switch (ChannelNumber)
    {
        case 1:
            TIMx->CCR1 = ccr_val;
            break;
        case 2:
            TIMx->CCR2 = ccr_val;
            break;
        case 3:
            TIMx->CCR3 = ccr_val;
            break;
        case 4:
            TIMx->CCR4 = ccr_val;
            break;
        default:
            break;
    }

    /* Generate an update event to load the new Prescaler, ARR, and CCR values (matches HAL's TIM_EGR_UG) */
    TIMx->EGR |= TIM_EGR_UG;
    /* Clear the update interrupt flag (optional) */
    TIMx->SR &= ~TIM_SR_UIF;
}

/**
 * @brief Enables and starts the PWM signal generation on the specified channel.
 * @param TRD_Channel The logical PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT)
    {
        /* Invalid channel */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    /* For advanced timers (TIM1), enable Main Output (MOE) as part of start sequence.
     * This matches HAL's __HAL_TIM_MOE_ENABLE for break-enabled instances. */
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    /* Enable the counter (matches HAL's __HAL_TIM_ENABLE) */
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The logical PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT)
    {
        /* Invalid channel */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    /* Disable the counter (matches HAL's __HAL_TIM_DISABLE) */
    TIMx->CR1 &= ~TIM_CR1_CEN;

    /* For advanced timers (TIM1), disable Main Output (MOE) as part of stop sequence.
     * This matches HAL's __HAL_TIM_MOE_DISABLE. */
    if (TIMx == TIM1)
    {
        TIMx->BDTR &= ~TIM_BDTR_MOE;
    }
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function iterates through all defined PWM channels, stops their timers,
 *        disables their clocks, and reconfigures their GPIO pins to a low-power state (Analog).
 */
void PWM_PowerOff(void)
{
    /* Iterate through all defined PWM channels */
    for (TRD_Channel_t i = (TRD_Channel_t)0; i < TRD_PWM_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        TIM_TypeDef* TIMx = config->TIMx;
        GPIO_TypeDef* PortName = config->PortName;
        uint8_t PinNumber = config->PinNumber;

        /* 1. Stop the specific timer channel (calls PWM_Stop which disables counter and MOE) */
        PWM_Stop(i);

        /* 2. Configure GPIO pin back to Analog mode (low power) - matches a typical GPIO de-init state */
        /* Set MODER to Analog mode (11) */
        PortName->MODER |= (3UL << (PinNumber * 2)); // Set bits to 11 for Analog mode
        
        /* Reset OTYPER to Push-Pull (0) - default or low power */
        PortName->OTYPER &= ~(1UL << PinNumber);

        /* Reset OSPEEDR to Low speed (00) */
        PortName->OSPEEDR &= ~(3UL << (PinNumber * 2));

        /* Reset PUPDR to No Pull-up/Pull-down (00) */
        PortName->PUPDR &= ~(3UL << (PinNumber * 2));

        /* Clear AFRL/AFRH bits for this pin to remove alternate function mapping */
        if (PinNumber < 8)
        {
            PortName->AFRL &= ~(0xFUL << (PinNumber * 4));
        }
        else
        {
            PortName->AFRH &= ~(0xFUL << ((PinNumber - 8) * 4));
        }
    }

    /* 3. Disable clocks for all used Timer peripherals */
    /* Note: Only disable if no other modules are using them.
     * For a full power-off, assume no other modules need these specific timers.
     * This directly manipulates RCC registers, similar to HAL's __HAL_RCC_TIMx_CLK_DISABLE() */
    RCC->APB1ENR &= ~( (1UL << 1) | /* TIM3EN */
                       (1UL << 2) | /* TIM4EN */
                       (1UL << 3) );/* TIM5EN */

    RCC->APB2ENR &= ~( (1UL << 18) | /* TIM9EN */
                       (1UL << 17) | /* TIM10EN */
                       (1UL << 16) );/* TIM11EN */

    /* 4. Disable clocks for all used GPIO Ports (only if no other modules use them) */
    /* This requires a more sophisticated tracking mechanism to avoid turning off
     * ports that might be used by other peripherals. For a blanket 'power off',
     * we disable all ports used by PWM.
     * This directly manipulates RCC registers, similar to HAL's __HAL_RCC_GPIOx_CLK_DISABLE() */
    RCC->AHB1ENR &= ~( (1UL << 0) | /* GPIOAEN */
                       (1UL << 1) | /* GPIOBEN */
                       (1UL << 2) | /* GPIOCEN */
                       (1UL << 3) | /* GPIODEN */
                       (1UL << 4) );/* GPIOEEN */
}