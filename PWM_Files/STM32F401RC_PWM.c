/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/* Include necessary CMSIS/device headers for register access */
/* Note: This assumes standard STM32 device headers like stm32f4xx.h are available */
/* and provide definitions for peripheral base addresses (TIMx, GPIOx, RCC) and register structures. */
/* The register names used below strictly follow the provided RM0368 PDF snippet. */
#include "stm32f4xx.h"

/* Assuming SystemCoreClock is defined elsewhere (e.g., system_stm32f4xx.c) */
extern uint32_t SystemCoreClock;

/* Assumed Timer Clock Frequencies - based on common STM32F401RC clock tree setup */
/* These need to be verified against the actual project's clock configuration. */
/* TIM1, TIM9, TIM10, TIM11 are on APB2. TIM2, TIM3, TIM4, TIM5 are on APB1. */
/* Assumed APB1 prescaler = 2, APB2 prescaler = 1 or 2 (max freq = SystemCoreClock). */
/* If APBx prescaler > 1, timer clock is 2 * APBx clock. */
/* Assuming APB1 prescaler is 2 (PCLK1 = HCLK/2) and APB2 prescaler is 1 (PCLK2 = HCLK). */
/* Therefore, TIMCLK1/9/10/11 = 2 * PCLK2, TIMCLK2/3/4/5 = 2 * PCLK1 */
/* If APB1 prescaler is > 1 (e.g., 2), timer clock is 2 * APB1 clock */
/* If APB2 prescaler is > 1 (e.g., 2), timer clock is 2 * APB2 clock */
/* If APB prescaler is 1, timer clock is APB clock */

#define GET_TIM_CLOCK_FREQ(TIMx) \
    ((TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) ? \
     (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos == 0 ? SystemCoreClock : SystemCoreClock * 2 : \
     (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos == 0 ? SystemCoreClock : SystemCoreClock * 2) \
     /* Assumed clock config logic - please verify against actual project's clock tree */


/** Reserved Timers and Channels ===========================================================================*/
/* TIM1 and TIM2 are reserved for potential OS/delay purposes or complex control requiring 32-bit timers */
/* Their channels are explicitly excluded from the pwm_channel_map below. */

/** PWM Channel Mapping ===============================================================================*/
/*
 * Production-ready configuration array for mapping logical PWM channels
 * to physical Timer, Channel, GPIO Port, Pin, and Alternate Function.
 *
 * Each entry format:
 * { TIMx, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber }
 *
 * Only valid PWM-capable pins for STM32F401RC are included, excluding pin 0.
 * Assumed mappings are based on common STM32F4 pin configurations.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    /* Timer Channels Available for PWM: TIM3, TIM4, TIM5, TIM9, TIM10, TIM11 */
    /* Reserved Timers: TIM1, TIM2 */
    /* Pin 0 excluded per requirements */

    /* TIM3 Channels (16-bit timer) */
    { TIM3, 1, GPIOA, 6, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 2, GPIOA, 7, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 3, GPIOB, 0, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */
    { TIM3, 4, GPIOB, 1, GPIO_AF_TIM3 },  /* Assumed PWM config - please verify */

    /* TIM4 Channels (16-bit timer) */
    { TIM4, 1, GPIOB, 6, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 2, GPIOB, 7, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 3, GPIOB, 8, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */
    { TIM4, 4, GPIOB, 9, GPIO_AF_TIM4 },  /* Assumed PWM config - please verify */

    /* TIM5 Channels (32-bit timer) - Excluding CH1 (PA0) due to pin 0 exclusion rule */
    { TIM5, 2, GPIOA, 1, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */
    { TIM5, 3, GPIOA, 2, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */
    { TIM5, 4, GPIOA, 3, GPIO_AF_TIM5 },  /* Assumed PWM config - please verify */

    /* TIM9 Channels (16-bit timer) */
    { TIM9, 1, GPIOA, 2, GPIO_AF_TIM9 },  /* Assumed PWM config - please verify */
    { TIM9, 2, GPIOA, 3, GPIO_AF_TIM9 },  /* Assumed PWM config - please verify */

    /* TIM10 Channel (16-bit timer) */
    { TIM10, 1, GPIOB, 8, GPIO_AF_TIM10 }, /* Assumed PWM config - please verify */

    /* TIM11 Channel (16-bit timer) */
    { TIM11, 1, GPIOB, 9, GPIO_AF_TIM11 }  /* Assumed PWM config - please verify */
};

/* Define the size of the map for bounds checking */
#define PWM_CHANNEL_COUNT (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))

/** Local Helper Functions =========================================================================*/

/**
 * @brief Enables the clock for the given GPIO port.
 * @param Port: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 */
static void PWM_Enable_GPIO_Clock(GPIO_TypeDef* Port)
{
    if (Port == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOC)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOD)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOE)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOH)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    /* Add wait for clock ready if necessary (e.g., DSB instruction) */
    __DSB(); /* Assumed best practice for clock enabling */
}

/**
 * @brief Disables the clock for the given GPIO port.
 * @param Port: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 */
static void PWM_Disable_GPIO_Clock(GPIO_TypeDef* Port)
{
    if (Port == GPIOA)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOB)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOC)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOD)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOE)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    else if (Port == GPIOH)
    {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN; /* PDF Reference (General GPIO structure) - Assumed RCC reg name */
    }
    __DSB(); /* Assumed best practice for clock disabling */
}


/**
 * @brief Enables the clock for the given Timer peripheral.
 * @param TIMx: Pointer to the Timer peripheral (e.g., TIM3, TIM4).
 */
static void PWM_Enable_TIM_Clock(TIM_TypeDef* TIMx)
{
    if (TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM5)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM10)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM11)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* Assumed RCC reg name */
    }
     /* Reserved Timers TIM1, TIM2 are not enabled here */
    __DSB(); /* Assumed best practice for clock enabling */
}

/**
 * @brief Disables the clock for the given Timer peripheral.
 * @param TIMx: Pointer to the Timer peripheral (e.g., TIM3, TIM4).
 */
static void PWM_Disable_TIM_Clock(TIM_TypeDef* TIMx)
{
    if (TIMx == TIM3)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM4)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM5)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM9)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM10)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN; /* Assumed RCC reg name */
    }
    else if (TIMx == TIM11)
    {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; /* Assumed RCC reg name */
    }
     /* Reserved Timers TIM1, TIM2 are not disabled here */
    __DSB(); /* Assumed best practice for clock disabling */
}


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel: The logical PWM channel to initialize. Corresponds to the index in pwm_channel_map.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        /* Invalid channel */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    GPIO_TypeDef* Port = config->Port;
    uint8_t Pin = config->PinNumber;
    uint8_t Channel = config->ChannelNumber;
    uint8_t AF = config->AlternateFunctionNumber;

    /* 1. Enable clocks for GPIO and Timer */
    PWM_Enable_GPIO_Clock(Port);
    PWM_Enable_TIM_Clock(TIMx);

    /* 2. Configure GPIO pin for Alternate Function */
    /* Set pin mode to Alternate Function (MODER = 10) */
    Port->MODER &= ~(0x03 << (Pin * 2)); /* PDF Reference (GPIOx_MODER) */
    Port->MODER |= (0x02 << (Pin * 2));  /* PDF Reference (GPIOx_MODER) */

    /* Set output type to Push-Pull (OTYPER = 0) */
    Port->OTYPER &= ~(0x01 << Pin); /* PDF Reference (GPIOx_OTYPER) */
    /* Output speed (OSPEEDR) - setting High speed */
    Port->OSPEEDR &= ~(0x03 << (Pin * 2)); /* PDF Reference (GPIOx_OSPEEDR) */
    Port->OSPEEDR |= (0x02 << (Pin * 2));  /* PDF Reference (GPIOx_OSPEEDR) - High speed (10) */
    /* Pull-up/Pull-down (PUPDR) - setting No pull-up/pull-down */
    Port->PUPDR &= ~(0x03 << (Pin * 2)); /* PDF Reference (GPIOx_PUPDR) */

    /* Set Alternate Function selection */
    if (Pin < 8)
    {
        Port->AFRL &= ~(0x0F << (Pin * 4)); /* PDF Reference (GPIOx_AFRL) */
        Port->AFRL |= (AF << (Pin * 4));    /* PDF Reference (GPIOx_AFRL) */
    }
    else
    {
        Port->AFRH &= ~(0x0F << ((Pin - 8) * 4)); /* PDF Reference (GPIOx_AFRH) */
        Port->AFRH |= (AF << ((Pin - 8) * 4));    /* PDF Reference (GPIOx_AFRH) */
    }

    /* 3. Configure Timer for PWM */

    /* Disable the timer counter before configuration */
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference (TIMx_CR1) */

    /* Configure Time-Base Unit (Prescaler, Auto-Reload) */
    /* Set default PSC and ARR for a initial period. Will be updated by PWM_Set_Freq */
    TIMx->PSC = 0; /* PDF Reference (TIMx_PSC) - Initial Prescaler, will calculate later */
    if (TIMx == TIM2 || TIMx == TIM5) /* 32-bit timers */
    {
         TIMx->ARR = 0xFFFFFFFF; /* PDF Reference (TIMx_ARR) - Max period initially */
    }
    else /* 16-bit timers */
    {
         TIMx->ARR = 0xFFFF; /* PDF Reference (TIMx_ARR) - Max period initially */
    }
    /* Set ARPE bit to enable auto-reload preload buffer */
    TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference (TIMx_CR1) */

    /* Configure Output Compare Mode (PWM Mode 1) */
    volatile uint32_t* ccmr_reg;
    uint8_t ccmr_shift;

    if (Channel == 1 || Channel == 2)
    {
        ccmr_reg = &TIMx->CCMR1; /* PDF Reference (TIMx_CCMR1) */
        ccmr_shift = (Channel == 1) ? 0 : 8;
    }
    else /* Channel 3 or 4 */
    {
        ccmr_reg = &TIMx->CCMR2; /* PDF Reference (TIMx_CCMR2 for TIM1/2/3/4/5/9) */
        ccmr_shift = (Channel == 3) ? 0 : 8;
    }

    /* Configure channel as output (CCxS = 00) */
    *ccmr_reg &= ~(TIM_CCMR1_CC1S_Msk << ccmr_shift); /* PDF Reference (TIMx_CCMRx CCxS) */

    /* Select PWM Mode 1 (OCxM = 110) */
    *ccmr_reg &= ~(TIM_CCMR1_OC1M_Msk << ccmr_shift); /* PDF Reference (TIMx_CCMRx OCxM) */
    *ccmr_reg |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) << ccmr_shift; /* PDF Reference (TIMx_CCMRx OCxM=110) */

    /* Enable output compare preload (OCxPE = 1) */
    *ccmr_reg |= (TIM_CCMR1_OC1PE << ccmr_shift); /* PDF Reference (TIMx_CCMRx OCxPE) */

    /* Set initial Duty Cycle to 0% */
    volatile uint32_t* ccr_reg;
     if (Channel == 1) ccr_reg = &TIMx->CCR1; /* PDF Reference (TIMx_CCR1) */
     else if (Channel == 2) ccr_reg = &TIMx->CCR2; /* PDF Reference (TIMx_CCR2) */
     else if (Channel == 3) ccr_reg = &TIMx->CCR3; /* PDF Reference (TIMx_CCR3) */
     else ccr_reg = &TIMx->CCR4; /* PDF Reference (TIMx_CCR4) */

    *ccr_reg = 0; /* Set initial duty cycle to 0% */

    /* Configure Capture/Compare Enable Register (CCER) */
    /* Set output polarity to active high (CCxP = 0) */
    TIMx->CCER &= ~(TIM_CCER_CC1P << ((Channel - 1) * 4)); /* PDF Reference (TIMx_CCER CCxP) */

    /* Enable the capture/compare output (CCxE = 1) */
    TIMx->CCER |= (TIM_CCER_CC1E << ((Channel - 1) * 4)); /* PDF Reference (TIMx_CCER CCxE) */


    /* TIM1 specific configuration (if used) for Main Output Enable */
    if (TIMx == TIM1)
    {
        /* TIM1 needs BDTR configuration for main output enable */
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference (TIMx_BDTR MOE) - Assumed TIM1 is an advanced timer requiring MOE */
    }

    /* Generate an update event to load the Prescaler, ARR, and CCRx values into the shadow registers */
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference (TIMx_EGR) */

    /* Clear the update flag */
    TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference (TIMx_SR) */

    /* The counter will be started in PWM_Start() */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The logical PWM channel.
 * @param frequency: The desired frequency in Hz (tlong type).
 * @param duty: The desired duty cycle in percentage (0-100) (tbyte type).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT || frequency == 0 || duty > 100)
    {
        /* Invalid channel, frequency, or duty */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t Pin = config->PinNumber; /* Not strictly needed for calculation, but good to have */
    uint8_t Channel = config->ChannelNumber;

    uint32_t tim_clock = GET_TIM_CLOCK_FREQ(TIMx); /* Assumed clock config logic */
    uint32_t period = tim_clock / frequency; // Total timer counts per period

    uint32_t psc = 0;
    uint32_t arr = 0;
    uint32_t max_arr;

    if (TIMx == TIM2 || TIMx == TIM5) /* 32-bit timers */
    {
        max_arr = 0xFFFFFFFF;
    }
    else /* 16-bit timers */
    {
        max_arr = 0xFFFF;
    }

    /* Find appropriate PSC and ARR */
    /* Iterate through PSC values to find a valid ARR */
    for (psc = 0; psc <= 0xFFFF; psc++) /* PSC is 16-bit */
    {
        arr = period / (psc + 1);
        if (arr <= max_arr)
        {
            // Found a valid (PSC, ARR) pair
            break;
        }
    }

    if (psc > 0xFFFF || arr > max_arr)
    {
        /* Could not find a valid (PSC, ARR) combination for the desired frequency */
        /* This frequency is too low for the given timer clock and max ARR */
        /* Handle error (e.g., set to a default, indicate failure) */
        // For production code, this might need a more robust error handling mechanism
        return;
    }

    /* Calculate CCRx value for duty cycle */
    uint32_t ccr_value;
    if (duty == 0)
    {
        ccr_value = 0; // 0% duty cycle
    }
    else if (duty == 100)
    {
        ccr_value = arr + 1; // 100% duty cycle in PWM Mode 1 (refers to PDF section 12.3.10/13.3.9/14.3.9)
                             // Specifically, for upcounting, OCxREF is high as long as CNT < CCRx.
                             // If CCRx > ARR, it's always high. ARR+1 is > ARR.
    }
    else
    {
        // CCRx = (Duty / 100.0) * (ARR + 1)
        ccr_value = (uint32_t)(((uint64_t)duty * (arr + 1)) / 100);
        // Ensure CCR value does not exceed ARR for standard PWM pulse generation
        if (ccr_value > arr) ccr_value = arr;
    }

    /* Apply new configuration */
    TIMx->PSC = psc; /* PDF Reference (TIMx_PSC) */
    TIMx->ARR = arr; /* PDF Reference (TIMx_ARR) */

    /* Set the capture/compare value for the specific channel */
    volatile uint32_t* ccr_reg;
    if (Channel == 1) ccr_reg = &TIMx->CCR1; /* PDF Reference (TIMx_CCR1) */
    else if (Channel == 2) ccr_reg = &TIMx->CCR2; /* PDF Reference (TIMx_CCR2) */
    else if (Channel == 3) ccr_reg = &TIMx->CCR3; /* PDF Reference (TIMx_CCR3) */
    else ccr_reg = &TIMx->CCR4; /* PDF Reference (TIMx_CCR4) */

    *ccr_reg = ccr_value; /* PDF Reference (TIMx_CCRx) */

    /* Generate update event to load the buffers */
    /* UDIS should be 0 for this to work (default state) */
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference (TIMx_EGR) */
}

/**
 * @brief Enable and start PWM signal generation on the specified channel.
 * @param TRD_Channel: The logical PWM channel.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        /* Invalid channel */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t Channel = config->ChannelNumber;

    /* Enable the capture/compare output */
    TIMx->CCER |= (TIM_CCER_CC1E << ((Channel - 1) * 4)); /* PDF Reference (TIMx_CCER CCxE) */

    /* Enable main output (for TIM1) */
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference (TIMx_BDTR MOE) */
    }

    /* Enable the counter */
    TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference (TIMx_CR1) */
}

/**
 * @brief Stop the PWM signal output on the specified channel.
 * @param TRD_Channel: The logical PWM channel.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= PWM_CHANNEL_COUNT)
    {
        /* Invalid channel */
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t Channel = config->ChannelNumber;

    /* Disable the capture/compare output */
    TIMx->CCER &= ~(TIM_CCER_CC1E << ((Channel - 1) * 4)); /* PDF Reference (TIMx_CCER CCxE) */

    /* Disable main output (for TIM1) */
     if (TIMx == TIM1)
    {
        TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference (TIMx_BDTR MOE) */
    }

    /* Optionally stop the counter if this is the only active channel on this timer */
    /* can be handled at a higher level or in PWM_PowerOff. */
}

/**
 * @brief Disable all PWM peripherals and outputs to reduce power consumption.
 */
void PWM_PowerOff(void)
{
    /* Iterate through the map and disable each configured channel and its timer/port */
    for (TRD_Channel_t i = 0; i < PWM_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        TIM_TypeDef* TIMx = config->TIMx;
        GPIO_TypeDef* Port = config->Port;
        uint8_t Pin = config->PinNumber;
        uint8_t Channel = config->ChannelNumber;

        /* Disable the capture/compare output for this channel */
        TIMx->CCER &= ~(TIM_CCER_CC1E << ((Channel - 1) * 4)); /* PDF Reference (TIMx_CCER CCxE) */

        /* If TIM1, disable main output */
        if (TIMx == TIM1)
        {
             TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference (TIMx_BDTR MOE) */
        }

        /* Disable the timer counter */
        TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference (TIMx_CR1) */

        /* Configure GPIO pin back to Input mode (MODER = 00) */
        Port->MODER &= ~(0x03 << (Pin * 2)); /* PDF Reference (GPIOx_MODER) - Input (00) is reset state */

        /* No need to explicitly clear AFRL/AFRH bits as mode is no longer AF */
        /* No need to explicitly clear OTYPER, OSPEEDR, PUPDR as mode is Input (settings are ignored in Input mode) */
    }

    /* Disable clocks for used timers and ports */
    /* This part needs to track unique timers/ports used in the map */
    /* Simple approach: iterate through map, add used timers/ports to a list/mask, then disable */

    uint32_t used_tim_apb1_mask = 0;
    uint32_t used_tim_apb2_mask = 0;
    uint32_t used_gpio_ahb1_mask = 0;

    for (TRD_Channel_t i = 0; i < PWM_CHANNEL_COUNT; i++)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];

        /* Track used timers */
        if (config->TIMx == TIM3) used_tim_apb1_mask |= RCC_APB1ENR_TIM3EN;
        else if (config->TIMx == TIM4) used_tim_apb1_mask |= RCC_APB1ENR_TIM4EN;
        else if (config->TIMx == TIM5) used_tim_apb1_mask |= RCC_APB1ENR_TIM5EN;
        else if (config->TIMx == TIM9) used_tim_apb2_mask |= RCC_APB2ENR_TIM9EN;
        else if (config->TIMx == TIM10) used_tim_apb2_mask |= RCC_APB2ENR_TIM10EN;
        else if (config->TIMx == TIM11) used_tim_apb2_mask |= RCC_APB2ENR_TIM11EN;
        /* Reserved TIM1, TIM2 are not in the map, so their clocks are not disabled here */
        /* If they were enabled elsewhere, they remain enabled */

        /* Track used GPIO ports */
        if (config->Port == GPIOA) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIOAEN;
        else if (config->Port == GPIOB) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIOBEN;
        else if (config->Port == GPIOC) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIOCEN;
        else if (config->Port == GPIOD) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIODEN;
        else if (config->Port == GPIOE) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIOEEN;
        else if (config->Port == GPIOH) used_gpio_ahb1_mask |= RCC_AHB1ENR_GPIOHEN;
    }

    /* Disable the tracked clocks */
    RCC->APB1ENR &= ~used_tim_apb1_mask; /* Assumed RCC reg name */
    RCC->APB2ENR &= ~used_tim_apb2_mask; /* Assumed RCC reg name */
    RCC->AHB1ENR &= ~used_gpio_ahb1_mask; /* Assumed RCC reg name */

    __DSB(); /* Assumed best practice for clock disabling */
}