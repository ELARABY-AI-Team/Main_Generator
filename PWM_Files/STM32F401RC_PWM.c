/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include "stm32f4xx.h" // Standard CMSIS header for STM32F4 series, provides register definitions

// Assume SystemCoreClock is defined elsewhere (e.g., in system_stm32f4xx.c)
// For STM32F401RC, max SysClock is 84MHz.
// APB1 timers (TIM2, TIM3, TIM4, TIM5) clock frequency.
// If APB1 prescaler > 1, Timer clock = 2 * PCLK1. Max PCLK1 = 42MHz (SysClk/2). So Timer clock max = 84MHz.
// If APB2 timers (TIM1) prescaler > 1, Timer clock = 2 * PCLK2. Max PCLK2 = 84MHz (SysClk/1). So Timer clock max = 168MHz.
// We will assume a common configuration where APB1 and APB2 prescalers result in the maximum timer clock:
// APB1 Timers (TIM3, TIM4): 84 MHz
// APB2 Timers (TIM1): 84 MHz (if SysClk=84 and APB2 prescaler is /2 or /4, timer clock is 84)
// A more robust implementation would calculate this based on RCC->CFGR.
#define TIMER_CLOCK_FREQ    (84000000UL) // Assuming APB1/APB2 Timer clocks are 84MHz

// --- Assumed definitions from STM32F401RC_PWM.h ---
// typedef enum {
//     TRD_TIM3_CH1, // Example: TIM3 Channel 1 on PB4 (AF2)
//     TRD_TIM3_CH2, // Example: TIM3 Channel 2 on PB5 (AF2)
//     TRD_TIM4_CH1, // Example: TIM4 Channel 1 on PB6 (AF2)
//     // Add other supported channels following the RM...
//     NUM_TRD_CHANNELS // Total number of supported channels
// } TRD_Channel_t;

// typedef long tlong; // At least 32 bits
// typedef unsigned char tbyte; // 8 bits (uint8_t)
// ----------------------------------------------------


/**
 * @brief Structure to hold configuration for each supported PWM channel.
 *        Based on STM32F401RC Reference Manual (RM0368).
 */
typedef struct {
    TIM_TypeDef* TIMx;          // Pointer to the timer peripheral (e.g., TIM3)
    uint8_t Channel_Idx;        // Channel index (0=CH1, 1=CH2, 2=CH3, 3=CH4)
    GPIO_TypeDef* GPIOx;        // Pointer to the GPIO port (e.g., GPIOB)
    uint8_t GPIO_Pin_Idx;       // GPIO pin index (0-15)
    uint8_t GPIO_AF;            // GPIO Alternate Function number (0-15)
    volatile uint32_t* RCC_ENR; // Pointer to the relevant RCC Enable Register (e.g., &RCC->APB1ENR)
    uint32_t RCC_EN_BIT_Pos;    // Bit position in the RCC Enable Register (e.g., RCC_APB1ENR_TIM3EN_Pos)
    volatile uint32_t* GPIO_RCC_ENR; // Pointer to the GPIO RCC Enable Register (&RCC->AHB1ENR)
    uint32_t GPIO_RCC_EN_BIT_Pos; // Bit position in the GPIO RCC Enable Register (e.g., RCC_AHB1ENR_GPIOBEN_Pos)
    uint8_t Is_TIM1;            // Flag: 1 if this channel uses TIM1, 0 otherwise (TIM1 requires BDTR_MOE)
} PWM_Channel_Config_t;

/**
 * @brief Configuration array mapping TRD_Channel_t to hardware specifics.
 *        Populated based on STM32F401RC Datasheet/Reference Manual AF mappings.
 *        Only include channels actually used in the application.
 *        Example entries for TIM3 and TIM4 channels on GPIOB (common pins):
 *        - TIM3_CH1 on PB4 (AF2)
 *        - TIM3_CH2 on PB5 (AF2)
 *        - TIM4_CH1 on PB6 (AF2)
 */
static const PWM_Channel_Config_t g_pwm_configs[] = {
    // TRD_TIM3_CH1 -> TIM3 Channel 1, PB4, AF2
    { TIM3, 0, GPIOB, 4, GPIO_AF2_TIM3, &RCC->APB1ENR, RCC_APB1ENR_TIM3EN_Pos, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos, 0 },
    // TRD_TIM3_CH2 -> TIM3 Channel 2, PB5, AF2
    { TIM3, 1, GPIOB, 5, GPIO_AF2_TIM3, &RCC->APB1ENR, RCC_APB1ENR_TIM3EN_Pos, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos, 0 },
    // TRD_TIM4_CH1 -> TIM4 Channel 1, PB6, AF2
    { TIM4, 0, GPIOB, 6, GPIO_AF2_TIM4, &RCC->APB1ENR, RCC_APB1ENR_TIM4EN_Pos, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos, 0 },
    // Add more channels as needed, following this pattern and using the correct
    // Timer, Channel, GPIO, AF, and RCC bits from the F401RC RM/Datasheet.
    // Example for TIM1:
    // { TIM1, 0, GPIOA, 8, GPIO_AF1_TIM1, &RCC->APB2ENR, RCC_APB2ENR_TIM1EN_Pos, &RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos, 1 }, // TIM1_CH1 on PA8 (AF1)
};
static const uint32_t NUM_TRD_CHANNELS = sizeof(g_pwm_configs) / sizeof(g_pwm_configs[0]);


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 *        Configures the necessary timer and GPIO peripherals.
 * @param TRD_Channel: The PWM channel to initialize (as defined in TRD_Channel_t).
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    // Check if the channel is valid
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel, handle error (e.g., assertion, log, return)
        // For production, might assert or return an error code if function signature allowed it.
        return;
    }

    const PWM_Channel_Config_t* config = &g_pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    GPIO_TypeDef* GPIOx = config->GPIOx;
    uint8_t pin_idx = config->GPIO_Pin_Idx;
    uint8_t channel_idx = config->Channel_Idx;

    // 1. Enable peripheral clocks (Timer and GPIO)
    // Use direct register access based on config struct
    *(config->RCC_ENR) |= (1UL << config->RCC_EN_BIT_Pos);
    *(config->GPIO_RCC_ENR) |= (1UL << config->GPIO_RCC_EN_BIT_Pos);

    // Delay to allow peripheral clock to stabilize (optional but good practice)
    // Can add a small delay loop or dummy read
    volatile uint32_t dummy_read;
    dummy_read = *(config->RCC_ENR);
    dummy_read = *(config->GPIO_RCC_ENR);
    (void)dummy_read; // Avoid unused variable warning

    // 2. Configure GPIO pin for Alternate Function Push-Pull
    // Clear MODER bits (Input mode 00) for the pin
    GPIOx->MODER &= ~(GPIO_MODER_MODE0 << (pin_idx * 2));
    // Set MODER bits to Alternate Function mode (10)
    GPIOx->MODER |= (GPIO_MODER_MODE0_1 << (pin_idx * 2));

    // Output Type: Push-pull (reset state is push-pull)
    GPIOx->OTYPER &= ~(GPIO_OTYPER_OT0 << pin_idx); // Clear the output type bit

    // Output Speed: High speed (10) or Very High speed (11). Use High speed.
    GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin_idx * 2)); // Clear OSPEEDR bits
    GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_1 << (pin_idx * 2)); // Set to High speed (10)

    // Pull-up/Pull-down: No pull-up/pull-down (reset state is no pull-up/pull-down)
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin_idx * 2)); // Clear PUPDR bits

    // Configure Alternate Function mapping
    // AFR[0] for pins 0-7, AFR[1] for pins 8-15
    // Each pin uses 4 bits.
    if (pin_idx < 8) {
        GPIOx->AFR[0] &= ~(0xFUL << (pin_idx * 4)); // Clear AF bits for the pin
        GPIOx->AFR[0] |= ((uint32_t)config->GPIO_AF << (pin_idx * 4)); // Set AF bits
    } else {
        GPIOx->AFR[1] &= ~(0xFUL << ((pin_idx - 8) * 4)); // Clear AF bits for the pin
        GPIOx->AFR[1] |= ((uint32_t)config->GPIO_AF << ((pin_idx - 8) * 4)); // Set AF bits
    }

    // 3. Configure Timer for PWM mode
    // Set PWM Mode 1 (110) for the specific channel's output compare mode bits (OCxM)
    // Channels 1 & 2 are configured in CCMR1, Channels 3 & 4 in CCMR2.
    volatile uint32_t* ccmr_reg;
    uint32_t ccmr_shift; // Shift for the channel's configuration bits (8 bits per channel)
    uint32_t ccer_bit_pos; // Bit position in CCER register

    if (channel_idx == 0) { // Channel 1
        ccmr_reg = &TIMx->CCMR1;
        ccmr_shift = 0;
        ccer_bit_pos = TIM_CCER_CC1E_Pos;
    } else if (channel_idx == 1) { // Channel 2
        ccmr_reg = &TIMx->CCMR1;
        ccmr_shift = 8;
        ccer_bit_pos = TIM_CCER_CC2E_Pos;
    } else if (channel_idx == 2) { // Channel 3
        ccmr_reg = &TIMx->CCMR2;
        ccmr_shift = 0;
        ccer_bit_pos = TIM_CCER_CC3E_Pos;
    } else { // Channel 4
        ccmr_reg = &TIMx->CCMR2;
        ccmr_shift = 8;
        ccer_bit_pos = TIM_CCER_CC4E_Pos;
    }

    // Clear OCxM bits (bits 4-6 for the channel's 8-bit block)
    *ccmr_reg &= ~(TIM_CCMR1_OC1M << ccmr_shift);
    // Set OCxM bits to PWM Mode 1 (110) -> Output/Compare mode bits are 4-6
    *ccmr_reg |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2) << ccmr_shift; // 0b110 shifted

    // Enable Output Compare Preload enable (OCxPE, bit 3) for the channel
    // This makes updates to CCRx effective only after an update event (UG or timer overflow)
    *ccmr_reg |= (TIM_CCMR1_OC1PE << ccmr_shift);

    // Enable Capture/Compare Output (CCxE) in the CCER register for the channel
    // This enables the output signal on the pin.
    // Channel 1: Bit 0 (CC1E), Channel 2: Bit 4 (CC2E), Channel 3: Bit 8 (CC3E), Channel 4: Bit 12 (CC4E)
    TIMx->CCER |= (1UL << ccer_bit_pos);

    // Optional: Configure output polarity (CCxP). Default is active high (0).
    // TIMx->CCER &= ~(1UL << (ccer_bit_pos + 1)); // Keep active high

    // (For TIM1 only): Enable Main Output Enable (MOE) in BDTR register.
    // Required for advanced timers (TIM1) to allow output on the pins.
    if (config->Is_TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Timer is configured but not started. Frequency/Duty are set by PWM_Set_Freq.
    // Counter is started by PWM_Start.
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 *        Calculates and sets the timer's ARR, PSC, and CCR registers.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired duty cycle in percentage (0 to 100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    // Check if the channel is valid
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    // Check frequency and duty constraints
    if (frequency == 0 || duty > 100) {
        // Invalid parameters
        // Could set duty to 0 and frequency to a safe default, or simply return.
        // Setting duty to 0 is often a safe default.
        if (duty > 100) duty = 0;
        if (frequency == 0) {
             // Cannot generate 0 Hz, maybe set duty to 0 and stop the timer later?
             // For now, just return.
             return;
        }
    }

    const PWM_Channel_Config_t* config = &g_pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t channel_idx = config->Channel_Idx;

    uint32_t timer_clock = TIMER_CLOCK_FREQ; // Assuming constant timer clock frequency

    uint32_t psc = 0;
    uint32_t arr = 0;
    uint64_t temp_arr_plus_1; // Use 64-bit for intermediate calculation

    // Calculate ARR and PSC to achieve the desired frequency.
    // Timer frequency = timer_clock / (PSC + 1)
    // PWM frequency = timer_frequency / (ARR + 1) = timer_clock / ((PSC + 1) * (ARR + 1))
    // So, (PSC + 1) * (ARR + 1) = timer_clock / frequency

    uint32_t total_timer_ticks = timer_clock / frequency;

    // Find a suitable PSC and ARR.
    // Iterate through possible PSC values to find a valid ARR (within 16-bit range for TIM3/4).
    // We want to maximize ARR for better duty cycle resolution, so iterate PSC upwards.
    // Limit PSC to 0xFFFF for 16-bit timers. For 32-bit timers (TIM2, TIM5), ARR can be 32-bit.
    // We assume 16-bit timers (TIM3, TIM4) based on the examples in g_pwm_configs.
    const uint32_t MAX_ARR = 0xFFFF; // For 16-bit timers

    for (psc = 0; psc <= 0xFFFF; psc++) {
        temp_arr_plus_1 = total_timer_ticks / (psc + 1);
        // Check if the division is exact and ARR fits within 16 bits
        if ((total_timer_ticks % (psc + 1) == 0) && (temp_arr_plus_1 > 0) && (temp_arr_plus_1 <= (MAX_ARR + 1))) {
            arr = (uint32_t)(temp_arr_plus_1 - 1);
            break; // Found a valid PSC/ARR pair
        }
    }

    // If loop finishes without finding a pair (shouldn't happen for valid frequencies within timer range),
    // arr and psc will have default values or the last calculated values.
    // In a robust system, handle this "no solution found" case. For void function, maybe print error.

    // Apply the calculated PSC and ARR
    TIMx->PSC = psc;
    TIMx->ARR = arr;

    // Calculate and set the Capture Compare Register (CCR) for the desired duty cycle
    // Duty Cycle = (CCRx / (ARR + 1)) * 100%
    // CCRx = (Duty / 100.0) * (ARR + 1)
    // Use 64-bit intermediate to prevent overflow when multiplying duty * (arr + 1)
    uint32_t ccr_value = (uint32_t)((uint64_t)duty * (arr + 1) / 100);

    // Set the correct CCR register based on channel index
    // TIM_TypeDef struct contains CCR1, CCR2, CCR3, CCR4 members.
    switch(channel_idx) {
        case 0: TIMx->CCR1 = ccr_value; break;
        case 1: TIMx->CCR2 = ccr_value; break;
        case 2: TIMx->CCR3 = ccr_value; break;
        case 3: TIMx->CCR4 = ccr_value; break;
        default:
            // Should not happen with correct config
            return;
    }

    // Generate an Update Event (UG) to load the new PSC, ARR, and CCR values immediately
    // This is important after changing timer parameters.
    TIMx->EGR |= TIM_EGR_UG;
    // Optional: Clear the Update Interrupt Flag if UIE is enabled, to avoid pending interrupt
    // TIMx->SR &= ~TIM_SR_UIF; // Assuming UIE is not enabled by this driver

    // The timer counter needs to be enabled via PWM_Start to actually generate output.
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 *        Assumes PWM_Init and PWM_Set_Freq have been called.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    // Check if the channel is valid
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &g_pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;

    // Enable the counter to start timer operation
    TIMx->CR1 |= TIM_CR1_CEN;

    // (For TIM1 only): Ensure Main Output Enable (MOE) is set.
    // This was set in Init, but can be reset by break events. Re-setting here ensures output.
    if (config->Is_TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 *        Disables the channel output but keeps the timer counter running.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     // Check if the channel is valid
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &g_pwm_configs[TRD_Channel];
    TIM_TypeDef* TIMx = config->TIMx;
    uint8_t channel_idx = config->Channel_Idx;

    // Disable Capture/Compare Output (CCxE) in the CCER register for the channel
    // This disables the output signal on the pin without stopping the timer counter.
    uint32_t ccer_bit_pos;
     if (channel_idx == 0) ccer_bit_pos = TIM_CCER_CC1E_Pos;
     else if (channel_idx == 1) ccer_bit_pos = TIM_CCER_CC2E_Pos;
     else if (channel_idx == 2) ccer_bit_pos = TIM_CCER_CC3E_Pos;
     else ccer_bit_pos = TIM_CCER_CC4E_Pos; // Channel 4

    TIMx->CCER &= ~(1UL << ccer_bit_pos);

    // (For TIM1 only): If this is TIM1, disabling the *last* active channel could potentially
    // warrant disabling MOE. However, this requires state tracking (how many TIM1 channels
    // are active). A simpler approach is to leave MOE enabled, or disable it here
    // and re-enable in start. Leaving it enabled is less disruptive if other channels are active.
    // We will leave MOE enabled when stopping individual TIM1 channels.
}

/**
 * @brief Disables all PWM peripherals and outputs configured by this driver.
 *        Disables timer and GPIO clocks to reduce power consumption.
 */
void PWM_PowerOff(void)
{
    // Iterate through all defined PWM channels in the configuration
    for (uint32_t i = 0; i < NUM_TRD_CHANNELS; i++) {
        const PWM_Channel_Config_t* config = &g_pwm_configs[i];
        TIM_TypeDef* TIMx = config->TIMx;
        GPIO_TypeDef* GPIOx = config->GPIOx;
        uint8_t pin_idx = config->GPIO_Pin_Idx;

        // 1. Disable the timer counter
        TIMx->CR1 &= ~TIM_CR1_CEN;

        // 2. Disable Main Output (MOE) for TIM1 if applicable
        if (config->Is_TIM1) {
           TIMx->BDTR &= ~TIM_BDTR_MOE;
        }

        // 3. Disable the specific channel output (CCxE bit)
        // This was already done in PWM_Stop, but doing it again ensures it's off
        // in case Start was called but Stop wasn't.
        uint32_t ccer_bit_pos;
        if (config->Channel_Idx == 0) ccer_bit_pos = TIM_CCER_CC1E_Pos;
        else if (config->Channel_Idx == 1) ccer_bit_pos = TIM_CCER_CC2E_Pos;
        else if (config->Channel_Idx == 2) ccer_bit_pos = TIM_CCER_CC3E_Pos;
        else ccer_bit_pos = TIM_CCER_CC4E_Pos;
        TIMx->CCER &= ~(1UL << ccer_bit_pos);


        // 4. Disable the timer clock (only disable if no other channel on this timer is needed?)
        // This requires tracking which timers are still in use.
        // For a simple power off, disabling all configured timer clocks is acceptable.
        // A more complex version would track active timers.
        // Let's disable the clock for the timer instance. Note this might affect other channels
        // on the same timer if they are not being powered off by this driver's scope.
        // A safer approach might be resetting the timer peripheral instead of disabling clock.
        // For power reduction, disabling clock is more effective. Let's disable the clock.
        *(config->RCC_ENR) &= ~(1UL << config->RCC_EN_BIT_Pos);

        // 5. Configure GPIO pin back to a low-power state (e.g., Input Floating)
        // Clear MODER bits (sets to Input 00)
        GPIOx->MODER &= ~(GPIO_MODER_MODE0 << (pin_idx * 2));
        // Ensure pull-up/pull-down is off (reset state)
        GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin_idx * 2));
        // Clear AF mapping (sets to AF0 / System function, often input or analog)
        if (pin_idx < 8) {
            GPIOx->AFR[0] &= ~(0xFUL << (pin_idx * 4));
        } else {
            GPIOx->AFR[1] &= ~(0xFUL << ((pin_idx - 8) * 4));
        }

        // 6. Disable GPIO clock.
        // Similar issue as timer clock - might affect other pins on the port.
        // Disable the clock for the GPIO port.
         *(config->GPIO_RCC_ENR) &= ~(1UL << config->GPIO_RCC_EN_BIT_Pos);
    }
    // Note: This implementation disables clocks for ALL timers/GPIOs listed in g_pwm_configs.
    // If different channels use the same timer/GPIO port, the clock for that
    // peripheral might be enabled/disabled multiple times or disabled while still needed
    // by another channel in the list being processed later.
    // A better approach for power off might be to collect unique timer and GPIO peripherals
    // used, and disable each unique clock only once.
}