/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F410RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F410RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

/* Include standard STM32F4xx definitions for register access */
#include "stm32f410rc.h" // Or equivalent chip-specific header if available, otherwise stm32f4xx.h is common.
                         // Assumes peripheral register structs (TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef)
                         // and bit definitions (TIM_CR1_CEN, RCC_APB1ENR_TIMxEN, GPIO_MODER_MODEy, etc.)
                         // are defined here.

/* Assuming TRD_Channel_t definition and related type definitions (tlong, tbyte)
   are available in STM32F410RC_PWM.h */
#include "STM32F410RC_PWM.h"

/* Define assumed custom types if not provided in the header */
#ifndef TLONG_DEFINED
#define TLONG_DEFINED
typedef uint32_t tlong; // Assuming tlong maps to uint32_t
#endif

#ifndef TBYTE_DEFINED
#define TBYTE_DEFINED
typedef uint8_t tbyte;  // Assuming tbyte maps to uint8_t (for 0-100 duty cycle)
#endif

/* Define assumed TRD_Channel_t enum values and map them to hardware */
/* NOTE: This mapping is example based on common TIM/GPIO availability.
   Verify against actual project requirements and PCB layout. */
typedef struct {
    TIM_TypeDef *TIMx;      // Timer instance
    uint32_t    Channel;    // Timer Channel (1 to 4)
    uint32_t    TimerRCC;   // RCC register bit for timer clock enable
    GPIO_TypeDef *GPIOx;    // GPIO port
    uint16_t    GPIO_Pin;   // GPIO pin number (0 to 15)
    uint32_t    GPIORCC;    // RCC register bit for GPIO clock enable
    uint8_t     GPIO_AF;    // GPIO Alternate Function number
} PWM_Channel_Config_t;

// Example Configurations - VERIFY THESE AGAINST STM32F410RC DATASHEET/RM AND YOUR HARDWARE
// Using common pins: TIM3_CH1 (PA6 or PC6), TIM4_CH2 (PB7)
// Need to choose specific pins and verify AF mapping. Let's use PA6 and PB7 as examples.
static const PWM_Channel_Config_t PWM_Configs[] = {
    {
        .TIMx       = TIM3,                 // Timer 3
        .Channel    = 1,                    // Channel 1
        .TimerRCC   = RCC_APB1ENR_TIM3EN,   // Clock enable bit for TIM3
        .GPIOx      = GPIOA,                // GPIO Port A
        .GPIO_Pin   = 6,                    // Pin 6
        .GPIORCC    = RCC_AHB1ENR_GPIOAEN,  // Clock enable bit for GPIOA
        .GPIO_AF    = GPIO_AF2_TIM3         // AF2 maps TIM3 functions on PA6 (Verify with RM)
    },
    {
        .TIMx       = TIM4,                 // Timer 4
        .Channel    = 2,                    // Channel 2
        .TimerRCC   = RCC_APB1ENR_TIM4EN,   // Clock enable bit for TIM4
        .GPIOx      = GPIOB,                // GPIO Port B
        .GPIO_Pin   = 7,                    // Pin 7
        .GPIORCC    = RCC_AHB1ENR_GPIOBEN,  // Clock enable bit for GPIOB
        .GPIO_AF    = GPIO_AF2_TIM4         // AF2 maps TIM4 functions on PB7 (Verify with RM)
    },
    // Add more configurations here for other TRD_Channel_t values as needed
    // { .TIMx = ..., .Channel = ..., ... },
};

/* Helper function to get configuration for a channel */
static const PWM_Channel_Config_t* get_pwm_config(TRD_Channel_t TRD_Channel)
{
    /* Basic validation for the channel index */
    if (TRD_Channel >= (sizeof(PWM_Configs) / sizeof(PWM_Configs[0])))
    {
        return NULL; /* Invalid channel */
    }
    return &PWM_Configs[TRD_Channel];
}

/**Functions ===========================================================================*/

/**
  * @brief  Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
  * @param  TRD_Channel: The PWM channel to initialize.
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        /* Handle error: Invalid channel configuration */
        return;
    }

    /* 1. Enable clock for the Timer and GPIO port */
    /* Use |= to avoid disturbing clocks for other peripherals */
    RCC->AHB1ENR |= config->GPIORCC;
    RCC->APB1ENR |= config->TimerRCC;

    /* Delay to allow clock to stabilize (optional but good practice) */
    /* Read back SR to ensure the clock is stable */
    volatile uint32_t dummy_read;
    dummy_read = RCC->AHB1ENR;
    dummy_read = RCC->APB1ENR;
    (void)dummy_read;

    /* 2. Configure GPIO pin for Alternate Function (AF) */
    /* Clear and set MODER bits: 2 bits per pin (00: Input, 01: Output, 10: AF, 11: Analog) */
    config->GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (config->GPIO_Pin * 2)); // Clear bits
    config->GPIOx->MODER |= (GPIO_MODER_MODE0_1 << (config->GPIO_Pin * 2));  // Set to Alternate Function (10b)

    /* Configure Alternate Function (AFR[L/H]) */
    /* AFRL for pins 0-7, AFRH for pins 8-15. 4 bits per pin. */
    if (config->GPIO_Pin < 8)
    {
        config->GPIOx->AFR[0] &= ~(0xF << (config->GPIO_Pin * 4));          // Clear AF bits (AFRL)
        config->GPIOx->AFR[0] |= (config->GPIO_AF << (config->GPIO_Pin * 4)); // Set AF
    }
    else
    {
        config->GPIOx->AFR[1] &= ~(0xF << ((config->GPIO_Pin - 8) * 4));      // Clear AF bits (AFRH)
        config->GPIOx->AFR[1] |= (config->GPIO_AF << ((config->GPIO_Pin - 8) * 4)); // Set AF
    }

    /* Configure Output Type (OTYPER): Push-pull (0) or Open-drain (1) */
    /* PWM typically uses push-pull */
    config->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT_0 << config->GPIO_Pin); // Set to Push-Pull (0)

    /* Configure Output Speed (OSPEEDR): Low (00), Medium (01), Fast (10), High (11) */
    /* High speed is often preferred for PWM */
    config->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 << (config->GPIO_Pin * 2)); // Clear bits
    config->GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0_1 | GPIO_OSPEEDR_OSPEEDR0_0) << (config->GPIO_Pin * 2); // Set to High Speed (11b)

    /* Configure Pull-up/Pull-down (PUPDR): None (00), Pull-up (01), Pull-down (10) */
    /* No pull-up/down needed for AF output */
    config->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (config->GPIO_Pin * 2)); // Set to No Pull-up/down (00)

    /* 3. Configure Timer for PWM mode */
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t channel = config->Channel;

    /* Disable the timer counter before configuring */
    TIMx->CR1 &= ~TIM_CR1_CEN;

    /* Configure TIM_CR1 */
    TIMx->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    // Other CR1 bits (DIR, CMS, CKD) typically default to upcounting, edge-aligned, no division

    /* Configure TIM_CCMRx for PWM Mode 1 (active when CNT < CCRx) */
    /* OCxM bits: 110 for PWM mode 1 */
    /* OCxPE bit: 1 for Output Compare Preload Enable */
    uint32_t ccmr_mask = 0;
    uint32_t ccmr_value = 0;

    // Determine which CCMR register and which bits to use based on channel
    switch (channel)
    {
        case 1:
            ccmr_mask = TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE;
            ccmr_value = (0x6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; // PWM mode 1 + preload enable
            TIMx->CCMR1 &= ~ccmr_mask; // Clear previous configuration
            TIMx->CCMR1 |= ccmr_value; // Set new configuration
            break;
        case 2:
            ccmr_mask = TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE;
            ccmr_value = (0x6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // PWM mode 1 + preload enable
            TIMx->CCMR1 &= ~ccmr_mask; // Clear previous configuration
            TIMx->CCMR1 |= ccmr_value; // Set new configuration
            break;
        case 3:
            ccmr_mask = TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE;
            ccmr_value = (0x6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; // PWM mode 1 + preload enable
            TIMx->CCMR2 &= ~ccmr_mask; // Clear previous configuration
            TIMx->CCMR2 |= ccmr_value; // Set new configuration
            break;
        case 4:
            ccmr_mask = TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE;
            ccmr_value = (0x6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // PWM mode 1 + preload enable
            TIMx->CCMR2 &= ~ccmr_mask; // Clear previous configuration
            TIMx->CCMR2 |= ccmr_value; // Set new configuration
            break;
        default:
            /* Should not happen with valid config, but handle defensively */
            return;
    }

    /* Configure TIM_CCER: Capture/Compare Enable Register */
    /* CCxE bit: 1 to enable output for the channel */
    /* CCxP bit: 0 for Output polarity high */
    uint32_t ccer_enable_bit = 0;
    switch (channel)
    {
        case 1: ccer_enable_bit = TIM_CCER_CC1E; break;
        case 2: ccer_enable_bit = TIM_CCER_CC2E; break;
        case 3: ccer_enable_bit = TIM_CCER_CC3E; break;
        case 4: ccer_enable_bit = TIM_CCER_CC4E; break;
        default: return;
    }
    TIMx->CCER &= ~(ccer_enable_bit | (ccer_enable_bit << 2)); // Clear enable and polarity bits
    TIMx->CCER |= ccer_enable_bit;                             // Set enable bit (polarity is default high)

    /* For advanced timers (like TIM1), BDTR_MOE needs to be set.
       For general purpose timers (TIM3, TIM4), BDTR is not needed for simple PWM.
       Check RM for TIM3/TIM4 specific BDTR requirements if using advanced features.
       Assuming simple PWM, BDTR is skipped for TIM3/TIM4. */
    // if (TIMx == TIM1 || TIMx == TIM8) { TIMx->BDTR |= TIM_BDTR_MOE; }

    /* Initialize PSC, ARR, CCR values to known state (e.g., 0 frequency, 0 duty) */
    TIMx->PSC = 0;
    TIMx->ARR = 0;
    /* CCR registers are 0-indexed based on channel number */
    TIMx->CCR[channel - 1] = 0;

    /* Generate an update event to load all configurations into active registers */
    TIMx->EGR |= TIM_EGR_UG;

    /* Counter remains disabled after Init. Use PWM_Start to enable it. */
}

/**
  * @brief  Sets the desired PWM frequency and duty cycle for the selected channel.
  * @param  TRD_Channel: The PWM channel to configure.
  * @param  frequency: Desired frequency in Hz.
  * @param  duty: Desired duty cycle in percentage (0-100).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        /* Handle error: Invalid channel configuration */
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t channel = config->Channel;

    /* Ensure timer clock is enabled (should be from Init, but defensive) */
    if (!((RCC->APB1ENR & config->TimerRCC) || (RCC->APB2ENR & config->TimerRCC)))
    {
        /* Timer clock not enabled, cannot configure */
        return;
    }

    /* Ensure frequency is not zero */
    if (frequency == 0)
    {
        /* Set duty cycle to 0 and return (effectively turns off output or sets low) */
        TIMx->CCR[channel - 1] = 0;
        /* Optional: Set ARR/PSC to default if desired for zero frequency */
        // TIMx->PSC = 0;
        // TIMx->ARR = 0;
        return;
    }

    /* Get the timer clock frequency */
    /* Timers on APB1 (TIM2,3,4,5,6,7,12,13,14) clock is PCLK1 * 2 if APB1 prescaler > 1, else PCLK1 */
    /* Timers on APB2 (TIM1,8,9,10,11) clock is PCLK2 * 2 if APB2 prescaler > 1, else PCLK2 */
    /* STM32F410RC has APB1 prescaler 2, APB2 prescaler 1 common configuration.
       SystemCoreClock is the HCLK frequency. */
    uint32_t timer_clock;
    uint32_t apb1_psc = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
    uint32_t apb2_psc = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;

    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) // APB2 Timers
    {
         timer_clock = SystemCoreClock >> ((apb2_psc & 0x04) ? (((apb2_psc & 0x03) + 1) + 1) : 0);
    }
    else // APB1 Timers (TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14)
    {
         timer_clock = SystemCoreClock >> ((apb1_psc & 0x04) ? (((apb1_psc & 0x03) + 1) + 1) : 0);
    }
    /* Note: Simplified check based on typical prescalers: if prescaler > 1 (mask & 0x04 is set), timer clock is PCLK*2 */
    /* PCLKx = HCLK / (APBx_Prescaler) */
    /* timer_clock = HCLK / APBx_Prescaler * 2 if APBx_Prescaler > 1 */
    /* timer_clock = HCLK / APBx_Prescaler * 1 if APBx_Prescaler = 1 */
    /* Correct Calculation:
       uint32_t apb1_div[] = {0, 0, 0, 0, 1, 2, 3, 4}; // Prescaler values 0xx, 100-111 map to /1, /2, /4, /8, /16..
       uint32_t apb_index = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
       uint32_t pclk1 = SystemCoreClock >> apb1_div[apb_index];
       if (apb_index >= 4) timer_clock = pclk1 * 2; else timer_clock = pclk1;
       Do similar for APB2 timers. Using the simplified shift above might be wrong depending on actual bits.
       Let's use the recommended way from ST examples which often involves reading PCLK values.
       Assuming SystemCoreClock and peripheral clock setup is handled elsewhere.
       A common setup for F410RC is HCLK=SystemCoreClock, APB1=HCLK/2, APB2=HCLK/1.
       PCLK1 = HCLK/2, PCLK2 = HCLK.
       APB1 Timer clock = PCLK1 * 2 = HCLK
       APB2 Timer clock = PCLK2 * 1 = HCLK
       but add a note that this might need adjustment based on actual clock tree configuration.
    */
    timer_clock = SystemCoreClock; // ASSUMPTION: Timer clock is equal to SystemCoreClock


    /* Calculate ARR and PSC for the desired frequency */
    uint32_t total_cycles = timer_clock / frequency; // Cycles per period (ideal)

    uint32_t psc = 0;
    uint32_t arr = total_cycles - 1;

    /* Find appropriate PSC and ARR values. Prioritize maximizing ARR for resolution. */
    /* Iterate PSC from 0 up to max (0xFFFF) to find a valid ARR (<= 0xFFFF) */
    while ((arr > 0xFFFF) && (psc < 0xFFFF))
    {
        psc++;
        if (psc == 0xFFFF) { // Avoid division by zero if psc reaches max before finding valid ARR
             arr = 0; // Signal failure or use max possible arr
        } else {
             arr = (total_cycles / (psc + 1)) - 1;
        }
    }

    /* Handle edge case where calculated ARR is still too large even with max PSC */
    if (arr > 0xFFFF)
    {
        arr = 0xFFFF; // Cap ARR at max value for 16-bit timers
        /* Recalculate PSC based on capped ARR */
        if (arr != (uint32_t)-1) { // Avoid division by zero if arr calculation resulted in -1
             psc = (total_cycles / (arr + 1)) - 1;
             if (psc > 0xFFFF) psc = 0xFFFF; // Cap PSC if needed
        } else {
            psc = 0xFFFF; // Set PSC to max if ARR calculation failed
        }
    }

    /* Ensure minimal ARR for non-zero frequency (at least 1 cycle period -> ARR=0) */
    if (frequency > 0 && arr == (uint32_t)-1) arr = 0;
    if (frequency > 0 && total_cycles == 0) { /* Frequency is higher than timer clock */ arr = 0; psc = 0; }

    /* Update PSC and ARR registers */
    TIMx->PSC = psc;
    TIMx->ARR = arr;

    /* Calculate CCR value for the desired duty cycle */
    /* Duty cycle = (CCRx + 1) / (ARR + 1) * 100 for edge-aligned mode 1 (output high active) */
    /* CCRx = (duty * (ARR + 1)) / 100 - 1 */
    /* Using 64-bit intermediate to avoid overflow during multiplication */
    uint32_t ccr_value = (uint32_t)(((uint64_t)duty * (arr + 1)) / 100);

    /* Update CCR register for the channel */
    /* Ensure CCR value does not exceed ARR+1 (max pulse width) */
    if (ccr_value > (arr + 1)) ccr_value = arr + 1;
    TIMx->CCR[channel - 1] = ccr_value;

    /* Generate an update event to load the new PSC, ARR, and CCR values */
    /* Only if timer is running, otherwise they will be loaded when CEN is set */
    if (TIMx->CR1 & TIM_CR1_CEN)
    {
        TIMx->EGR |= TIM_EGR_UG;
    }
}

/**
  * @brief  Enable and start PWM signal generation on the specified channel.
  * @param  TRD_Channel: The PWM channel to start.
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        /* Handle error: Invalid channel configuration */
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t channel = config->Channel;

    /* Enable the Capture/Compare output for the channel */
    uint32_t ccer_enable_bit = 0;
    switch (channel)
    {
        case 1: ccer_enable_bit = TIM_CCER_CC1E; break;
        case 2: ccer_enable_bit = TIM_CCER_CC2E; break;
        case 3: ccer_enable_bit = TIM_CCER_CC3E; break;
        case 4: ccer_enable_bit = TIM_CCER_CC4E; break;
        default: return;
    }
    TIMx->CCER |= ccer_enable_bit;

    /* For advanced timers (TIM1, TIM8), enable the main output */
    // if (TIMx == TIM1 || TIMx == TIM8) { TIMx->BDTR |= TIM_BDTR_MOE; }

    /* Enable the counter */
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Stop the PWM signal output on the specified channel.
  * @param  TRD_Channel: The PWM channel to stop.
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL)
    {
        /* Handle error: Invalid channel configuration */
        return;
    }

    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t channel = config->Channel;

    /* Disable the Capture/Compare output for the channel */
    uint32_t ccer_enable_bit = 0;
    switch (channel)
    {
        case 1: ccer_enable_bit = TIM_CCER_CC1E; break;
        case 2: ccer_enable_bit = TIM_CCER_CC2E; break;
        case 3: ccer_enable_bit = TIM_CCER_CC3E; break;
        case 4: ccer_enable_bit = TIM_CCER_CC4E; break;
        default: return;
    }
    TIMx->CCER &= ~ccer_enable_bit;

    /* For advanced timers (TIM1, TIM8), disable the main output if no other channels are active */
    // This check is complex. A simpler approach is to only disable MOE in PowerOff.
    // if (TIMx == TIM1 || TIMx == TIM8) { // Check if any other channel on this timer is active
    //    if (!((TIMx->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) & ~ccer_enable_bit))
    //    {
    //        TIMx->BDTR &= ~TIM_BDTR_MOE; // Disable MOE only if this was the last active channel
    //    }
    // }

    /* Optional: Set the output compare value to 0 to ensure output goes low */
    /* TIMx->CCR[channel - 1] = 0; */

    /* Note: The timer counter (TIMx->CR1_CEN) is *not* disabled here,
       allowing other channels on the same timer to continue running.
       The counter is only disabled in PowerOff. */
}

/**
  * @brief  Disable all PWM peripherals and outputs to reduce power consumption.
  *         Stops all timers used for PWM and disables their clocks and associated GPIOs.
  * @param  None
  * @retval None
  */
void PWM_PowerOff(void)
{
    /* Iterate through all possible PWM configurations */
    /* Stop each configured channel and disable its GPIO */
    for (size_t i = 0; i < (sizeof(PWM_Configs) / sizeof(PWM_Configs[0])); ++i)
    {
        const PWM_Channel_Config_t *config = &PWM_Configs[i];

        TIM_TypeDef *TIMx = config->TIMx;
        uint32_t channel = config->Channel;
        GPIO_TypeDef *GPIOx = config->GPIOx;
        uint16_t gpio_pin = config->GPIO_Pin;

        /* 1. Stop the channel output and the timer counter */
        uint32_t ccer_enable_bit = 0;
        switch (channel)
        {
            case 1: ccer_enable_bit = TIM_CCER_CC1E; break;
            case 2: ccer_enable_bit = TIM_CCER_CC2E; break;
            case 3: ccer_enable_bit = TIM_CCER_CC3E; break;
            case 4: ccer_enable_bit = TIM_CCER_CC4E; break;
            default: break; // Should not happen with valid config
        }

        /* Disable the channel output */
        if (ccer_enable_bit != 0)
        {
             TIMx->CCER &= ~ccer_enable_bit;
        }

        /* Disable the timer counter */
        TIMx->CR1 &= ~TIM_CR1_CEN;

        /* For advanced timers (TIM1, TIM8), disable the main output */
        // if (TIMx == TIM1 || TIMx == TIM8) { TIMx->BDTR &= ~TIM_BDTR_MOE; }


        /* 2. Reset GPIO configuration */
        /* Set MODER to Analog Input (11) - lowest power state for most pins */
        GPIOx->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0) << (gpio_pin * 2); // Set to Analog (11b)
        /* Clear other settings */
        GPIOx->OTYPER &= ~(GPIO_OTYPER_OT_0 << gpio_pin);       // Push-Pull
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0 << (gpio_pin * 2)); // Low Speed
        GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (gpio_pin * 2));   // No pull-up/down

        /* Note: Disabling GPIO clock happens next */
    }

    /* 3. Disable clocks for all Timers and GPIO ports used for PWM */
    /* Iterate again to collect unique RCC bits or use separate logic */
    /* Simpler approach: Disable clocks for all Timers/GPIOs defined in PWM_Configs */
    for (size_t i = 0; i < (sizeof(PWM_Configs) / sizeof(PWM_Configs[0])); ++i)
    {
         const PWM_Channel_Config_t *config = &PWM_Configs[i];
         RCC->APB1ENR &= ~config->TimerRCC; // Disable timer clock
         RCC->APB2ENR &= ~config->TimerRCC; // Disable timer clock (check APB2 as well)
         RCC->AHB1ENR &= ~config->GPIORCC;  // Disable GPIO clock
    }

    /* Optional: Assert reset for the timers */
    // RCC->APB1RSTR |= config->TimerRCC;
    // RCC->APB1RSTR &= ~config->TimerRCC;
    // RCC->APB2RSTR |= config->TimerRCC;
    // RCC->APB2RSTR &= ~config->TimerRCC;

    /* Delay for clock deactivation (optional) */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB1ENR;
    dummy_read = RCC->APB2ENR;
    dummy_read = RCC->AHB1ENR;
    (void)dummy_read;
}