/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Implementation for PWM generation on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h" // Assumes this header defines TRD_Channel_t and potentially other necessary macros/structures

// Include standard STM32 peripheral access headers - these are typically provided by the vendor or CMSIS
// For bare-metal, we rely on register definitions usually found in CMSIS headers like stm32f4xx.h
#include "stm32f4xx.h"

/***********************************************************************************************************************
* PRIVATE MACROS
***********************************************************************************************************************/
// Define the assumed Timer clock frequency.
// STM32F401 timers connected to APB1 (TIM2, TIM3, TIM4, TIM5) and APB2 (TIM1, TIM9, TIM10, TIM11)
// run at PCLKx * 2 if the APBx prescaler is > 1, and PCLKx if the prescaler is 1.
// For this implementation, we assume a system clock where APB1 and APB2 timer clocks are 84MHz.
// In a production system, this value should be dynamically determined or based on a known clock configuration.
#define PWM_TIMER_CLOCK_FREQ    84000000UL // Example: 84 MHz

// Define the maximum value for the 16-bit timer registers (ARR, CCR)
#define PWM_MAX_TIMER_VAL       65535UL

/***********************************************************************************************************************
* PRIVATE DATA STRUCTURES
***********************************************************************************************************************/
// Structure to map the TRD_Channel_t enum to specific hardware resources
typedef struct
{
    TIM_TypeDef     *TIMx;      // Pointer to the Timer instance
    uint32_t        Channel;    // Timer channel (e.g., TIM_CHANNEL_1)
    GPIO_TypeDef    *GPIOx;     // Pointer to the GPIO port
    uint16_t        GPIO_Pin;   // GPIO pin number (e.g., GPIO_PIN_6)
    uint8_t         GPIO_AF;    // GPIO Alternate Function number
    uint32_t        RCC_APB_ENR; // RCC APB Enable Register (APB1ENR or APB2ENR)
    uint32_t        RCC_APB_ENR_MASK; // Mask for the specific Timer clock enable bit
    uint32_t        RCC_AHB_ENR; // RCC AHB Enable Register (AHB1ENR)
    uint32_t        RCC_AHB_ENR_MASK; // Mask for the specific GPIO port clock enable bit
} PWM_Channel_Config_t;

// Array mapping TRD_Channel_t to hardware configurations.
// Populate this array based on the desired PWM output pins and the STM32F401RC datasheet/RM.
// NOTE: TIM9 and TIM10 channels are intentionally excluded as per the requirement to reserve them.
static const PWM_Channel_Config_t PWM_Channel_Map[] =
{
    // Example Mappings (User needs to verify these against their hardware design and RM AF tables)
    // Channel ID            | Timer | Ch | Port | Pin     | AF | RCC APB Reg  | APB Mask      | RCC AHB Reg | AHB Mask
    { /* TRD_PWM_CHANNEL_1 */  TIM3,   TIM_CHANNEL_1, GPIOA, GPIO_PIN_6, GPIO_AF_TIM3, RCC->APB1ENR, RCC_APB1ENR_TIM3EN, RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN },
    { /* TRD_PWM_CHANNEL_2 */  TIM3,   TIM_CHANNEL_2, GPIOA, GPIO_PIN_7, GPIO_AF_TIM3, RCC->APB1ENR, RCC_APB1ENR_TIM3EN, RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN },
    { /* TRD_PWM_CHANNEL_3 */  TIM4,   TIM_CHANNEL_3, GPIOB, GPIO_PIN_8, GPIO_AF_TIM4, RCC->APB1ENR, RCC_APB1ENR_TIM4EN, RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN },
    { /* TRD_PWM_CHANNEL_4 */  TIM1,   TIM_CHANNEL_1, GPIOA, GPIO_PIN_8, GPIO_AF_TIM1, RCC->APB2ENR, RCC_APB2ENR_TIM1EN, RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN },
    { /* TRD_PWM_CHANNEL_5 */  TIM2,   TIM_CHANNEL_2, GPIOA, GPIO_PIN_1, GPIO_AF_TIM2, RCC->APB1ENR, RCC_APB1ENR_TIM2EN, RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN },
    { /* TRD_PWM_CHANNEL_6 */  TIM11,  TIM_CHANNEL_1, GPIOB, GPIO_PIN_9, GPIO_AF_TIM11,RCC->APB2ENR, RCC_APB2ENR_TIM11EN,RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN },
    // Add more mappings as needed based on your TRD_Channel_t definition in the header file
};

// Keep track of initialized channels to avoid re-initialization
static bool s_InitializedChannels[sizeof(PWM_Channel_Map) / sizeof(PWM_Channel_Config_t)] = {false};

// Keep track of active timers to disable clocks correctly in PowerOff
static uint32_t s_ActiveTimersAPB1Mask = 0;
static uint32_t s_ActiveTimersAPB2Mask = 0;
static uint32_t s_ActiveGPIOsMask = 0;


/***********************************************************************************************************************
* PRIVATE FUNCTION PROTOTYPES
***********************************************************************************************************************/
// Helper function to get the configuration for a given channel ID
static const PWM_Channel_Config_t* prv_GetChannelConfig(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* PRIVATE FUNCTIONS
***********************************************************************************************************************/

/**
  * @brief  Helper function to get the configuration for a given channel ID.
  * @param  TRD_Channel: The PWM channel ID (enum from header).
  * @retval Pointer to the configuration structure, or NULL if invalid channel.
  */
static const PWM_Channel_Config_t* prv_GetChannelConfig(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel < (sizeof(PWM_Channel_Map) / sizeof(PWM_Channel_Config_t)))
    {
        return &PWM_Channel_Map[TRD_Channel];
    }
    return NULL; // Invalid channel
}


/**Functions ===========================================================================*/

/**
  * @brief  Initializes the PWM hardware for the given channel.
  * @param  TRD_Channel: The PWM channel ID (enum from header).
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = prv_GetChannelConfig(TRD_Channel);

    if (config == NULL)
    {
        // Handle error: Invalid channel
        return;
    }

    // Check if channel is already initialized
    if (s_InitializedChannels[TRD_Channel])
    {
        return; // Already initialized
    }

    // 1. Enable GPIO clock
    // Use the register pointer and mask from the config
    // Note: This assumes RCC->AHB1ENR is the only relevant AHB register for GPIO clocks
    RCC->AHB1ENR |= config->RCC_AHB_ENR_MASK;
    // Wait for GPIO clock to be ready (optional but good practice)
    (void)RCC->AHB1ENR;

    s_ActiveGPIOsMask |= config->RCC_AHB_ENR_MASK; // Mark GPIO clock as active

    // 2. Configure GPIO pin for Alternate Function
    uint32_t pin_pos = 0;
    // Determine pin position (0-15) to configure MODER, OTYPER, OSPEEDR, PUPDR
    // Note: GPIO_Pin is a bitmask (e.g., GPIO_PIN_6 is 0x0040). Need to find the bit position.
    if (config->GPIO_Pin > 0)
    {
        while (!((config->GPIO_Pin >> pin_pos) & 0x1))
        {
            pin_pos++;
        }
    }

    // Set pin mode to Alternate Function (AF)
    config->GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (pin_pos * 2)); // Clear current mode
    config->GPIOx->MODER |= (GPIO_MODER_MODER0_1 << (pin_pos * 2)); // Set to AF mode (10)

    // Set output type to Push-Pull
    config->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin_pos); // Clear current type (Push-Pull is 0)

    // Set output speed (Optional: High speed)
    config->GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0 << (pin_pos * 2)); // Set to High Speed (11)

    // Set pull-up/pull-down to No Pull-Up/Down
    config->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin_pos * 2)); // Clear current pull config

    // Configure the Alternate Function register (AFRL or AFRH)
    if (pin_pos < 8) // AFRL (Pins 0-7)
    {
        config->GPIOx->AFR[0] &= ~(GPIO_AFRL_AFRL0 << (pin_pos * 4)); // Clear current AF
        config->GPIOx->AFR[0] |= (config->GPIO_AF << (pin_pos * 4)); // Set new AF
    }
    else // AFRH (Pins 8-15)
    {
        config->GPIOx->AFR[1] &= ~(GPIO_AFRH_AFRH0 << ((pin_pos - 8) * 4)); // Clear current AF
        config->GPIOx->AFR[1] |= (config->GPIO_AF << ((pin_pos - 8) * 4)); // Set new AF
    }


    // 3. Enable Timer clock
    // Use the register pointer and mask from the config
    if (config->RCC_APB_ENR == RCC->APB1ENR)
    {
        RCC->APB1ENR |= config->RCC_APB_ENR_MASK;
        s_ActiveTimersAPB1Mask |= config->RCC_APB_ENR_MASK; // Mark Timer clock as active
        (void)RCC->APB1ENR; // Wait for clock to be ready
    }
    else if (config->RCC_APB_ENR == RCC->APB2ENR)
    {
        RCC->APB2ENR |= config->RCC_APB_ENR_MASK;
        s_ActiveTimersAPB2Mask |= config->RCC_APB_ENR_MASK; // Mark Timer clock as active
        (void)RCC->APB2ENR; // Wait for clock to be ready
    }


    // 4. Configure Timer for PWM mode
    // Stop the timer before configuration
    config->TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set a default Prescaler and Auto-Reload Register (Period)
    // These will be updated later by PWM_Set_Freq, but needed for initial setup
    // A common default is max period with min prescaler, giving lowest freq initially.
    config->TIMx->PSC = 0;
    config->TIMx->ARR = PWM_MAX_TIMER_VAL;

    // Configure Output Compare mode for the specific channel
    // We need to select the correct CCMR register (CCMR1 or CCMR2) based on the channel
    // and then configure the OCxM bits for PWM Mode 1 (110) and OCxPE bit for preload enable.
    uint32_t ccmr_mask;
    uint32_t ccmr_mode;
    volatile uint32_t *ccmr_reg;

    switch (config->Channel)
    {
        case TIM_CHANNEL_1:
            ccmr_reg = &config->TIMx->CCMR1;
            ccmr_mask = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE;
            ccmr_mode = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM Mode 1
            break;
        case TIM_CHANNEL_2:
            ccmr_reg = &config->TIMx->CCMR1;
            ccmr_mask = TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE;
            ccmr_mode = (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // PWM Mode 1
            break;
        case TIM_CHANNEL_3:
            ccmr_reg = &config->TIMx->CCMR2;
            ccmr_mask = TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE;
            ccmr_mode = (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2); // PWM Mode 1
            break;
        case TIM_CHANNEL_4:
            ccmr_reg = &config->TIMx->CCMR2;
            ccmr_mask = TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE;
            ccmr_mode = (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2); // PWM Mode 1
            break;
        default:
            return; // Invalid channel number for Timer configuration
    }

    // Clear existing mode bits for the channel
    *ccmr_reg &= ~ccmr_mask;
    // Set PWM Mode 1 and enable output compare preload (OCxPE)
    *ccmr_reg |= ccmr_mode | (ccmr_mask & (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE));

    // Enable the capture/compare output (CCxE bit)
    config->TIMx->CCER |= (TIM_CCER_CC1E << ((config->Channel - 1) * 4)); // CC1E=Bit0, CC2E=Bit4, CC3E=Bit8, CC4E=Bit12

    // For TIM1 (Advanced Control Timer), enable Main Output Enable (MOE) bit in BDTR register
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Set a default Capture/Compare value (e.g., 0% duty cycle initially)
    // This will be updated by PWM_Set_Freq
    switch (config->Channel)
    {
        case TIM_CHANNEL_1: config->TIMx->CCR1 = 0; break;
        case TIM_CHANNEL_2: config->TIMx->CCR2 = 0; break;
        case TIM_CHANNEL_3: config->TIMx->CCR3 = 0; break;
        case TIM_CHANNEL_4: config->TIMx->CCR4 = 0; break;
        default: break; // Should not happen due to switch above, but good practice
    }

    // Generate an update event to load the prescaler and ARR values into the active registers
    config->TIMx->EGR |= TIM_EGR_UG;
    // Clear the update interrupt flag (UIF) if it was set by UG
    config->TIMx->SR &= ~TIM_SR_UIF;

    s_InitializedChannels[TRD_Channel] = true; // Mark channel as initialized
}

/**
  * @brief  Sets the desired PWM frequency and duty cycle for the selected channel.
  * @param  TRD_Channel: The PWM channel ID (enum from header).
  * @param  frequency: The desired frequency in Hz (tlong).
  * @param  duty: The desired duty cycle in percentage (0-100) (tbyte).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    const PWM_Channel_Config_t *config = prv_GetChannelConfig(TRD_Channel);

    if (config == NULL || !s_InitializedChannels[TRD_Channel])
    {
        // Handle error: Invalid or uninitialized channel
        return;
    }

    if (frequency == 0 || duty > 100)
    {
        // Handle error: Invalid frequency or duty cycle
        return;
    }

    // Calculate Timer Period (ARR) and Prescaler (PSC)
    // Timer_Clock = (PSC + 1) * (ARR + 1) * frequency
    // (PSC + 1) * (ARR + 1) = Timer_Clock / frequency
    uint64_t timer_ticks = PWM_TIMER_CLOCK_FREQ / frequency; // Use 64-bit for intermediate calculation

    uint32_t psc = 0;
    uint32_t arr = 0;

    // Strategy: Find smallest PSC such that ARR <= PWM_MAX_TIMER_VAL (65535)
    // (ARR + 1) = timer_ticks / (PSC + 1)
    // We need timer_ticks / (PSC + 1) <= PWM_MAX_TIMER_VAL + 1
    // (PSC + 1) >= timer_ticks / (PWM_MAX_TIMER_VAL + 1)
    uint64_t psc_plus_1_min = (timer_ticks + PWM_MAX_TIMER_VAL) / (PWM_MAX_TIMER_VAL + 1);

    if (psc_plus_1_min == 0) // Avoid division by zero if timer_ticks is very small
    {
        psc_plus_1_min = 1;
    }

    // Check if the required PSC+1 exceeds the maximum possible value (65536)
    if (psc_plus_1_min > (PWM_MAX_TIMER_VAL + 1))
    {
        // Frequency is too low to be generated with this timer/clock combination
        // or calculation error. Set to minimum possible frequency (max ARR, max PSC).
        psc = PWM_MAX_TIMER_VAL;
        arr = PWM_MAX_TIMER_VAL;
    }
    else
    {
        psc = (uint32_t)psc_plus_1_min - 1;
        arr = (uint32_t)(timer_ticks / (psc + 1)) - 1;

        // Double check calculated values
        if (arr > PWM_MAX_TIMER_VAL || psc > PWM_MAX_TIMER_VAL)
        {
             // Fallback or error handling: Set to max period/prescaler or stop PWM?
             // For now, set to max possible values.
            psc = PWM_MAX_TIMER_VAL;
            arr = PWM_MAX_TIMER_VAL;
        }
    }


    // Calculate Capture/Compare value (Duty Cycle)
    // CCRx = (ARR + 1) * Duty / 100
    uint32_t ccr_val = (uint32_t)(((uint64_t)(arr + 1) * duty) / 100);

    // Update Timer registers (Prescaler, ARR, and CCR)
    // It's recommended to update these while the timer is stopped or using preload/update event
    bool was_running = (config->TIMx->CR1 & TIM_CR1_CEN);
    if (was_running)
    {
        // Stop timer temporarily or rely on preload (OCxPE) and UG event
        // Relying on preload and UG is generally smoother
        //config->TIMx->CR1 &= ~TIM_CR1_CEN; // Option 1: Stop
    }


    config->TIMx->PSC = psc;
    config->TIMx->ARR = arr;

    switch (config->Channel)
    {
        case TIM_CHANNEL_1: config->TIMx->CCR1 = ccr_val; break;
        case TIM_CHANNEL_2: config->TIMx->CCR2 = ccr_val; break;
        case TIM_CHANNEL_3: config->TIMx->CCR3 = ccr_val; break;
        case TIM_CHANNEL_4: config->TIMx->CCR4 = ccr_val; break;
        default: break; // Should not happen
    }

    // Generate an update event to load the new PSC, ARR, and CCR values
    config->TIMx->EGR |= TIM_EGR_UG;
    // Clear the update interrupt flag (UIF) if it was set by UG
    config->TIMx->SR &= ~TIM_SR_UIF;

    if (was_running)
    {
        // config->TIMx->CR1 |= TIM_CR1_CEN; // Option 1: Restart if stopped
    }
}

/**
  * @brief  Enables and starts PWM signal generation on the specified channel.
  * @param  TRD_Channel: The PWM channel ID (enum from header).
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = prv_GetChannelConfig(TRD_Channel);

    if (config == NULL || !s_InitializedChannels[TRD_Channel])
    {
        // Handle error: Invalid or uninitialized channel
        return;
    }

    // Enable the timer counter
    config->TIMx->CR1 |= TIM_CR1_CEN;

    // For TIM1, ensure the output is enabled (MOE)
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

/**
  * @brief  Stops the PWM signal output on the specified channel.
  * @param  TRD_Channel: The PWM channel ID (enum from header).
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    const PWM_Channel_Config_t *config = prv_GetChannelConfig(TRD_Channel);

    if (config == NULL || !s_InitializedChannels[TRD_Channel])
    {
        // Handle error: Invalid or uninitialized channel
        return;
    }

    // Stop the timer counter
    config->TIMx->CR1 &= ~TIM_CR1_CEN;

    // Optionally, set the corresponding output compare register to 0 or ARR+1
    // to ensure the output pin goes low (0% duty cycle) or high (100% duty cycle, depending on mode).
    // Setting to 0 for 0% is common for stopping.
    switch (config->Channel)
    {
        case TIM_CHANNEL_1: config->TIMx->CCR1 = 0; break;
        case TIM_CHANNEL_2: config->TIMx->CCR2 = 0; break;
        case TIM_CHANNEL_3: config->TIMx->CCR3 = 0; break;
        case TIM_CHANNEL_4: config->TIMx->CCR4 = 0; break;
        default: break;
    }

    // Generate an update event to load the CCR value immediately
    config->TIMx->EGR |= TIM_EGR_UG;
    config->TIMx->SR &= ~TIM_SR_UIF;

    // For TIM1, disable Main Output Enable (MOE) if desired, though stopping the counter is usually sufficient.
    // Leaving MOE enabled won't hurt if the counter is stopped.
    // if (config->TIMx == TIM1)
    // {
    //     config->TIMx->BDTR &= ~TIM_BDTR_MOE;
    // }
}

/**
  * @brief  Disables all PWM peripherals and outputs to reduce power consumption.
  * @param  None
  * @retval None
  */
void PWM_PowerOff(void)
{
    // Iterate through all possible channels to disable individual timers/GPIOs
    for (size_t i = 0; i < (sizeof(PWM_Channel_Map) / sizeof(PWM_Channel_Config_t)); ++i)
    {
        const PWM_Channel_Config_t *config = &PWM_Channel_Map[i];

        if (s_InitializedChannels[i])
        {
            // Stop the timer counter
            config->TIMx->CR1 &= ~TIM_CR1_CEN;

            // For TIM1, disable Main Output Enable (MOE)
            if (config->TIMx == TIM1)
            {
                config->TIMx->BDTR &= ~TIM_BDTR_MOE;
            }

            // Disable the Capture/Compare output enable (CCxE) for this channel
            config->TIMx->CCER &= ~(TIM_CCER_CC1E << ((config->Channel - 1) * 4));

            // Set the GPIO pin back to a safe low-power state (e.g., Analog Input)
            uint32_t pin_pos = 0;
             if (config->GPIO_Pin > 0)
            {
                while (!((config->GPIO_Pin >> pin_pos) & 0x1))
                {
                    pin_pos++;
                }
            }
            // Set pin mode to Analog (11)
            config->GPIOx->MODER |= (GPIO_MODER_MODER0 << (pin_pos * 2)); // Set to Analog mode (11)
            // Clear alternate function config
             if (pin_pos < 8) config->GPIOx->AFR[0] &= ~(GPIO_AFRL_AFRL0 << (pin_pos * 4));
             else             config->GPIOx->AFR[1] &= ~(GPIO_AFRH_AFRH0 << ((pin_pos - 8) * 4));

            // Optionally disable specific timer clock if it's no longer needed by any channel.
            // A simple approach here is to disable all clocks that were *ever* turned on for PWM.
        }
        // Reset the initialization flag
        s_InitializedChannels[i] = false;
    }

    // Disable all Timer clocks that were marked as active
    RCC->APB1ENR &= ~s_ActiveTimersAPB1Mask;
    RCC->APB2ENR &= ~s_ActiveTimersAPB2Mask;

    // Disable all GPIO clocks that were marked as active
    RCC->AHB1ENR &= ~s_ActiveGPIOsMask;

    // Reset active masks
    s_ActiveTimersAPB1Mask = 0;
    s_ActiveTimersAPB2Mask = 0;
    s_ActiveGPIOsMask = 0;
}

/***********************************************************************************************************************
* END OF FILE
***********************************************************************************************************************/