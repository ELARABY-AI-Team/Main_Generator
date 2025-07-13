/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready implementation for PWM generation on STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include <stdint.h>

// Note: Assuming bare-metal register definitions (e.g., TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef)
// are available via includes within "STM32F401RC_PWM.h" or a related CMSIS header.
// The register names used match those described in the provided RM0368 PDF excerpt.

// Note: The STM32F401RC has TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11.
// Timers TIM2 and TIM5 are 32-bit general-purpose timers.
// Timers TIM3 and TIM4 are 16-bit general-purpose timers.
// Timers TIM9, TIM10, TIM11 are 16-bit general-purpose timers.
// TIM1 is a 16-bit advanced-control timer.
// Per requirements, reserving at least 2 timers with their channels for other potential uses (OS, delay).
// We reserve TIM2 (32-bit) and TIM5 (32-bit) for this purpose.
// This leaves TIM1, TIM3, TIM4, TIM9, TIM10, TIM11 available for PWM.
// Based on common STM32F401RC pinouts (and *assuming* standard AF mappings, as detailed mapping tables were not provided),
// we select a subset of available PWM channels from TIM1, TIM3, TIM4, TIM9, TIM10, and TIM11.
// We strictly avoid using GPIO pins with pin number 0, as the provided PDF excerpt does not explicitly
// confirm their PWM timer function capability on specific channels.
// We prioritize pins >= 1.

// Structure to map a TRD_Channel_t to specific hardware
typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin; // Pin number 0-15
    TIM_TypeDef *timer_inst;
    uint8_t timer_channel; // Timer channel number (1-4 for TIM1/3/4, 1 for TIM10/11, 1/2 for TIM9)
    uint8_t gpio_af; // Alternate function number
    uint32_t rcc_gpio_mask; // RCC AHB1ENR bit for GPIO port clock enable
    uint32_t rcc_timer_mask; // RCC APB1ENR/APB2ENR bit for Timer clock enable
    // Note: RCC register addresses (AHB1ENR, APB1ENR, APB2ENR) are assumed available
    // via includes and standard peripheral register structures.
} pwm_channel_map_t;

// Map TRD_Channel_t enum to specific hardware timers and GPIO pins
// This mapping is based on common STM32F401RC pinouts and assumed AF values,
// as the precise AF mapping table was not provided in the RM excerpt.
// Avoids pin 0.
static const pwm_channel_map_t pwm_channel_map[] = {
    // TIM1 Channels (APB2 bus, assumed 84MHz timer clock)
    { GPIOA, 8,  TIM1, 1, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN }, // TRD_CHANNEL_0 -> PA8 (TIM1_CH1, AF1) /* Assumed PWM config - please verify */
    { GPIOA, 9,  TIM1, 2, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN }, // TRD_CHANNEL_1 -> PA9 (TIM1_CH2, AF1) /* Assumed PWM config - please verify */
    { GPIOA, 10, TIM1, 3, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN }, // TRD_CHANNEL_2 -> PA10 (TIM1_CH3, AF1) /* Assumed PWM config - please verify */
    { GPIOA, 11, TIM1, 4, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN }, // TRD_CHANNEL_3 -> PA11 (TIM1_CH4, AF1) /* Assumed PWM config - please verify */

    // TIM3 Channels (APB1 bus, assumed 84MHz timer clock after x2 scaling)
    { GPIOC, 6,  TIM3, 1, GPIO_AF_TIM3,  RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN }, // TRD_CHANNEL_4 -> PC6 (TIM3_CH1, AF2) /* Assumed PWM config - please verify */
    { GPIOC, 7,  TIM3, 2, GPIO_AF_TIM3,  RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN }, // TRD_CHANNEL_5 -> PC7 (TIM3_CH2, AF2) /* Assumed PWM config - please verify */
    { GPIOC, 8,  TIM3, 3, GPIO_AF_TIM3,  RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN }, // TRD_CHANNEL_6 -> PC8 (TIM3_CH3, AF2) /* Assumed PWM config - please verify */
    { GPIOC, 9,  TIM3, 4, GPIO_AF_TIM3,  RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN }, // TRD_CHANNEL_7 -> PC9 (TIM3_CH4, AF2) /* Assumed PWM config - please verify */

    // TIM4 Channels (APB1 bus, assumed 84MHz timer clock after x2 scaling)
    { GPIOB, 6,  TIM4, 1, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN }, // TRD_CHANNEL_8 -> PB6 (TIM4_CH1, AF2) /* Assumed PWM config - please verify */
    { GPIOB, 7,  TIM4, 2, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN }, // TRD_CHANNEL_9 -> PB7 (TIM4_CH2, AF2) /* Assumed PWM config - please verify */
    { GPIOB, 8,  TIM4, 3, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN }, // TRD_CHANNEL_10 -> PB8 (TIM4_CH3, AF2) /* Assumed PWM config - please verify */
    { GPIOB, 9,  TIM4, 4, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN }, // TRD_CHANNEL_11 -> PB9 (TIM4_CH4, AF2) /* Assumed PWM config - please verify */

    // TIM9 Channels (APB2 bus, assumed 84MHz timer clock)
    { GPIOA, 2,  TIM9, 1, GPIO_AF_TIM9,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM9EN }, // TRD_CHANNEL_12 -> PA2 (TIM9_CH1, AF3) /* Assumed PWM config - please verify */
    { GPIOA, 3,  TIM9, 2, GPIO_AF_TIM9,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM9EN }, // TRD_CHANNEL_13 -> PA3 (TIM9_CH2, AF3) /* Assumed PWM config - please verify */

    // TIM10 Channel (APB2 bus, assumed 84MHz timer clock)
    { GPIOB, 8, TIM10, 1, GPIO_AF_TIM10, RCC_AHB1ENR_GPIOBEN, RCC_APB2ENR_TIM10EN }, // TRD_CHANNEL_14 -> PB8 (TIM10_CH1, AF3) /* Assumed PWM config - please verify */

    // TIM11 Channel (APB2 bus, assumed 84MHz timer clock)
    { GPIOB, 9, TIM11, 1, GPIO_AF_TIM11, RCC_AHB1ENR_GPIOBEN, RCC_APB2ENR_TIM11EN }  // TRD_CHANNEL_15 -> PB9 (TIM11_CH1, AF3) /* Assumed PWM config - please verify */
};
// Note: TRD_CHANNEL_COUNT should match the number of entries in the map.
#define TRD_CHANNEL_COUNT (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))

// Assumed Timer clock frequency (f_TIM).
// On STM32F401, timers connected to APB1 (TIM3, TIM4) run at HCLK/2 if APB1 prescaler <= 1, or HCLK if APB1 prescaler > 1. Max APB1 is 42MHz.
// Timers connected to APB2 (TIM1, TIM9, TIM10, TIM11) run at HCLK/2 if APB2 prescaler <= 1, or HCLK if APB2 prescaler > 1. Max APB2 is 84MHz.
// If APBx prescaler > 1, the timer clock is twice the APBx clock.
// Assuming HCLK = 84MHz, APB1 Prescaler = 2 (42MHz APB1), APB2 Prescaler = 1 (84MHz APB2).
// Timer clocks would be: TIM3/4 -> 42MHz * 2 = 84MHz. TIM1/9/10/11 -> 84MHz * 1 = 84MHz.
static const tlong F_TIM_CLOCK_HZ = 84000000; /* Assumed Timer clock frequency (f_TIM) = 84MHz - please verify */

static uint8_t pwm_initialized[TRD_CHANNEL_COUNT] = {0}; // To track which channels are initialized

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Invalid channel
        return;
    }

    const pwm_channel_map_t *channel_info = &pwm_channel_map[TRD_Channel];
    GPIO_TypeDef *GPIOx = channel_info->gpio_port;
    uint16_t gpio_pin = channel_info->gpio_pin;
    TIM_TypeDef *TIMx = channel_info->timer_inst;
    uint8_t timer_channel = channel_info->timer_channel;
    uint8_t gpio_af = channel_info->gpio_af;
    uint16_t ccer_ccxe_bit = (1 << (timer_channel * 4)); // CCxE is bit 4*channel_number (1-based) in CCER
    uint16_t ccer_ccxp_bit = (1 << (timer_channel * 4 + 1)); // CCxP is bit 4*channel_number + 1

    // 1. Enable GPIO clock
    RCC->AHB1ENR |= channel_info->rcc_gpio_mask; /* Assumed RCC clock enable bit */

    // 2. Enable Timer clock
    // Check if timer is on APB1 or APB2 bus (based on which RCC enable register bit is used)
    if (channel_info->rcc_timer_mask & 0x0000FFFF) // APB1 bits are 15:0
    {
        RCC->APB1ENR |= channel_info->rcc_timer_mask; /* Assumed RCC clock enable bit */
    }
    else // APB2 bits are 31:16
    {
        RCC->APB2ENR |= channel_info->rcc_timer_mask; /* Assumed RCC clock enable bit */
    }

    // Wait for clocks to stabilize (optional, but good practice)
    // A simple delay loop or checking register bits might be needed in production code,
    // but for basic initialization, a few NOPs or just assuming clock is stable is common.

    // 3. Configure GPIO pin for Alternate Function
    // MODER: Alternate function mode (10)
    GPIOx->MODER &= ~(0b11 << (gpio_pin * 2)); /* PDF Reference for MODER bits 2y:2y+1 */
    GPIOx->MODER |= (0b10 << (gpio_pin * 2)); /* PDF Reference for MODER bits 2y:2y+1 */

    // OTYPER: Push-pull output type (0)
    GPIOx->OTYPER &= ~(1 << gpio_pin); /* PDF Reference for OTYPER bit y */

    // OSPEEDR: High speed (10) - adjust as needed
    GPIOx->OSPEEDR &= ~(0b11 << (gpio_pin * 2)); /* PDF Reference for OSPEEDR bits 2y:2y+1 */
    GPIOx->OSPEEDR |= (0b10 << (gpio_pin * 2)); /* PDF Reference for OSPEEDR bits 2y:2y+1 */

    // PUPDR: No pull-up, pull-down (00)
    GPIOx->PUPDR &= ~(0b11 << (gpio_pin * 2)); /* PDF Reference for PUPDR bits 2y:2y+1 */

    // AFRL/AFRH: Select the Alternate Function number
    if (gpio_pin < 8)
    {
        // Configure AFRL (pins 0-7)
        GPIOx->AFRL &= ~(0b1111 << (gpio_pin * 4)); /* PDF Reference for AFRLy bits */
        GPIOx->AFRL |= (gpio_af << (gpio_pin * 4)); /* PDF Reference for AFRLy bits */
    }
    else
    {
        // Configure AFRH (pins 8-15)
        GPIOx->AFRH &= ~(0b1111 << ((gpio_pin - 8) * 4)); /* PDF Reference for AFRHy bits */
        GPIOx->AFRH |= (gpio_af << ((gpio_pin - 8) * 4)); /* PDF Reference for AFRHy bits */
    }

    // 4. Configure Timer for PWM
    // Disable the timer before configuring
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference for CEN bit */

    // Configure Time-base: Upcounting mode, Edge-aligned, Auto-reload preload enable
    TIMx->CR1 &= ~TIM_CR1_DIR; // Upcounting (0) - Note: TIM9/10/11 are upcounters only. /* PDF Reference for DIR bit */
    TIMx->CR1 &= ~TIM_CR1_CMS; // Edge-aligned mode (00) /* PDF Reference for CMS bits */
    TIMx->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable (1) /* PDF Reference for ARPE bit */
    TIMx->CR1 &= ~TIM_CR1_CKD; // CKD = 00, tDTS = tCK_INT /* PDF Reference for CKD bits */

    // Configure PWM channel mode
    uint32_t ccmr_offset = (timer_channel - 1) / 2; // CCMR1 for channels 1 & 2, CCMR2 for channels 3 & 4
    uint32_t ccmr_shift = ((timer_channel - 1) % 2) * 8; // Shift by 0 for CH1/CH3, 8 for CH2/CH4

    // Clear existing mode and preload/fast enable bits
    // Mask: OCxCE (15/7), OCxM (14:12 / 6:4), OCxPE (11/3), OCxFE (10/2), CCxS (9:8 / 1:0)
    // We are configuring as output (CCxS=00) for PWM.
    // For OCx modes, the bits are 4,5,6 for channel 1 and 12,13,14 for channel 2 in CCMR1.
    // For OCx modes, the bits are 4,5,6 for channel 3 and 12,13,14 for channel 4 in CCMR2.
    uint32_t ccmr_mask = (0b11111111 << ccmr_shift); // Mask for OCxCE, OCxM, OCxPE, OCxFE, CCxS bits (if applicable)
     if (timer_channel % 2 != 0) { // Channel 1 or 3 (odd)
        // Clear bits 7:0 for channel 1/3 in CCMR1/2
         if (ccmr_offset == 0) TIMx->CCMR1 &= ~0x00FF;
         else TIMx->CCMR2 &= ~0x00FF;
     } else { // Channel 2 or 4 (even)
        // Clear bits 15:8 for channel 2/4 in CCMR1/2
         if (ccmr_offset == 0) TIMx->CCMR1 &= ~0xFF00;
         else TIMx->CCMR2 &= ~0xFF00;
     }


    // Set PWM Mode 1 (110) and Output Compare Preload Enable (1)
    uint32_t ccmr_value = (0b110 << 4) | (1 << 3); // OCxM = 110, OCxPE = 1
    if (timer_channel % 2 == 0) { // Adjust shift for channels 2 and 4 (bits 15:8)
         ccmr_value <<= 8;
    }


    if (ccmr_offset == 0)
    {
        TIMx->CCMR1 |= ccmr_value; /* PDF Reference for OCxM, OCxPE bits */
    }
    else
    {
        TIMx->CCMR2 |= ccmr_value; /* PDF Reference for OCxM, OCxPE bits */
    }

    // Set output polarity active high (0) - adjust if active low is needed
    TIMx->CCER &= ~ccer_ccxp_bit; /* PDF Reference for CCxP bit */
    // Disable CCxE initially - will be enabled in PWM_Start
    TIMx->CCER &= ~ccer_ccxe_bit; /* PDF Reference for CCxE bit */
    // Note: CCxNE bits only exist on TIM1 (Advanced Control). Ignored for other timers per PDF description.
    if (TIMx == TIM1) {
         TIMx->CCER &= ~(ccer_ccxe_bit << 1); // Clear CCxNE bit (bit 4*channel_number + 2)
    }

    // Initialize PSC and ARR to default values (e.g., max period)
    // Frequency and duty cycle are set in PWM_Set_Freq
    TIMx->PSC = 0; // Default prescaler value /* PDF Reference for PSC register */
    // Note: ARR is 16-bit for all available timers on this device.
    TIMx->ARR = 0xFFFF; // Default auto-reload value (max period) /* PDF Reference for ARR register */

    // Generate an update event to load the prescaler and auto-reload registers
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference for UG bit */
    // Clear the update flag (UIF) after generating update (optional, but clean)
    TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference for UIF bit */


    pwm_initialized[TRD_Channel] = 1; // Mark channel as initialized
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || !pwm_initialized[TRD_Channel])
    {
        // Invalid channel or not initialized
        return;
    }

    if (frequency == 0)
    {
        // Cannot generate PWM with 0 frequency
        // Consider stopping the channel or forcing output low?
        PWM_Stop(TRD_Channel); // Stop the signal
        return;
    }

    if (duty > 100)
    {
        duty = 100; // Clamp duty cycle
    }

    const pwm_channel_map_t *channel_info = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_info->timer_inst;
    uint8_t timer_channel = channel_info->timer_channel;

    // Calculate PSC and ARR for the desired frequency
    // f_PWM = f_TIM_CLOCK / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = f_TIM_CLOCK / frequency

    tlong total_ticks = F_TIM_CLOCK_HZ / frequency;

    // Simple calculation: Find smallest PSC >= 0 that keeps ARR <= 0xFFFF
    // Iterate through possible PSC values starting from 0
    uint16_t psc_value = 0;
    uint16_t arr_value = 0;

    // Find suitable PSC and ARR. Max PSC is 0xFFFF. Max ARR is 0xFFFF for these timers.
    // Let's choose PSC such that ARR is reasonably large for duty cycle resolution.
    // Target (PSC+1)*(ARR+1) = total_ticks.
    // If we want ARR to be close to 0xFFFF, PSC+1 = total_ticks / (0xFFFF + 1)
    tlong psc_plus_1 = total_ticks / (0xFFFF + 1);
    if (total_ticks % (0xFFFF + 1) != 0) {
        psc_plus_1++; // Round up if needed
    }

    if (psc_plus_1 == 0) {
        psc_value = 0;
    } else {
        psc_value = psc_plus_1 - 1;
    }

    // Ensure PSC does not exceed 16-bit max
    if (psc_value > 0xFFFF) {
        psc_value = 0xFFFF;
        // Recalculate ARR based on max PSC
         arr_value = (F_TIM_CLOCK_HZ / frequency / (psc_value + 1)) - 1;
         if (arr_value > 0xFFFF) arr_value = 0xFFFF; // Should not happen if psc_value is derived from total_ticks / (0xFFFF+1)
    } else {
       // Calculate ARR based on chosen PSC
       arr_value = (F_TIM_CLOCK_HZ / frequency / (psc_value + 1)) - 1;
       // Handle potential floating point inaccuracies or very low frequencies
       if (arr_value > 0xFFFF) {
            arr_value = 0xFFFF;
            // Recalculate PSC if ARR maxed out
            psc_value = (F_TIM_CLOCK_HZ / frequency / (arr_value + 1)) - 1;
             if (psc_value > 0xFFFF) psc_value = 0xFFFF; // Should not happen
       } else if (arr_value == 0xFFFF && total_ticks % (psc_value + 1) != 0 && psc_value < 0xFFFF) {
           // If ARR is exactly 0xFFFF and the division wasn't perfect, increment PSC
           psc_value++;
            if (psc_value > 0xFFFF) psc_value = 0xFFFF; // Should not happen
            arr_value = (F_TIM_CLOCK_HZ / frequency / (psc_value + 1)) - 1;
            if (arr_value > 0xFFFF) arr_value = 0xFFFF; // Should not happen
       }
    }

     // Ensure ARR is not negative in case of frequency too high
    if (arr_value > 0xFFFF) { // Check for underflow from -1
         arr_value = 0; // Smallest possible period
         psc_value = 0; // Smallest possible prescaler
    }

    // Calculate the capture/compare value for the duty cycle
    tlong duty_ticks = (tlong)(arr_value + 1) * duty / 100;
    // Ensure duty_ticks does not exceed ARR (can happen if duty=100% and calculation rounds up)
    if (duty_ticks > arr_value + 1) duty_ticks = arr_value + 1; // Max value is ARR+1 for 100% duty
    if (duty_ticks == arr_value + 1) duty_ticks = arr_value + 1; // 100% duty cycle means CCR >= ARR
    if (duty_ticks > 0 && duty == 0) duty_ticks = 0; // Edge case for 0%
    if (duty_ticks < 0) duty_ticks = 0;

    // Set the timer registers
    // Note: Assumes timer is not running or relies on preload registers.
    // For robustness, disable timer briefly if not using preload, but we enabled ARPE/OCxPE in Init.
    // Preload registers are updated at the next update event (UG or overflow).
    // We generate UG below to apply immediately.
    TIMx->PSC = psc_value; /* PDF Reference for PSC register */
    TIMx->ARR = arr_value; /* PDF Reference for ARR register */

    // Set the CCRx register based on the channel number
    // CCR1: TIMx->CCR1, CCR2: TIMx->CCR2, etc.
    volatile uint32_t *ccr_reg;
    switch (timer_channel) {
        case 1: ccr_reg = &TIMx->CCR1; break; /* PDF Reference for CCR1 register */
        case 2: ccr_reg = &TIMx->CCR2; break; /* PDF Reference for CCR2 register */
        case 3:
            // TIM1, TIM3, TIM4 have CH3. TIM9/10/11 do not have CH3 in the provided text.
            if (TIMx == TIM1 || TIMx == TIM3 || TIMx == TIM4) {
                ccr_reg = &TIMx->CCR3; break; /* PDF Reference for CCR3 register */
            } else return; // Invalid channel for this timer
        case 4:
             // TIM1, TIM3, TIM4 have CH4. TIM9/10/11 do not have CH4 in the provided text.
            if (TIMx == TIM1 || TIMx == TIM3 || TIMx == TIM4) {
                ccr_reg = &TIMx->CCR4; break; /* PDF Reference for CCR4 register */
            } else return; // Invalid channel for this timer
        default: return; // Invalid channel number
    }
    *ccr_reg = duty_ticks; /* PDF Reference for CCRx register */


    // Generate an update event to load the new PSC, ARR, and CCRx values into the active registers
    // This is needed because ARPE and OCxPE are enabled.
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference for UG bit */
    // Clear the update flag (UIF) after generating update (optional)
    TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference for UIF bit */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || !pwm_initialized[TRD_Channel])
    {
        // Invalid channel or not initialized
        return;
    }

    const pwm_channel_map_t *channel_info = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_info->timer_inst;
    uint8_t timer_channel = channel_info->timer_channel;
    uint16_t ccer_ccxe_bit = (1 << (timer_channel * 4)); // CCxE is bit 4*channel_number (1-based) in CCER

    // Enable the Capture/Compare output for the channel
    TIMx->CCER |= ccer_ccxe_bit; /* PDF Reference for CCxE bit */

    // Enable the Main Output (MOE) for TIM1 if it's an advanced timer.
    // MOE is in BDTR register, specific to advanced timers (TIM1).
    if (TIMx == TIM1)
    {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference for MOE bit */
    }

    // Enable the counter. This starts the timer and PWM generation.
    TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference for CEN bit */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= TRD_CHANNEL_COUNT || !pwm_initialized[TRD_Channel])
    {
        // Invalid channel or not initialized
        return;
    }

    const pwm_channel_map_t *channel_info = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_info->timer_inst;
    uint8_t timer_channel = channel_info->timer_channel;
    uint16_t ccer_ccxe_bit = (1 << (timer_channel * 4)); // CCxE is bit 4*channel_number (1-based) in CCER

    // Disable the Capture/Compare output for the channel
    TIMx->CCER &= ~ccer_ccxe_bit; /* PDF Reference for CCxE bit */

    // Optional: Force the output pin to a known state (e.g., low)
    // Find the correct CCMR register and mask based on channel number
    uint32_t ccmr_offset = (timer_channel - 1) / 2;
    uint32_t ccmr_shift = ((timer_channel - 1) % 2) * 8; // Shift for channel 1/3 (0) or 2/4 (8)
    uint32_t ocm_mask = (0b111 << (ccmr_shift + 4)); // Mask for OCxM bits (bits 6:4 or 14:12)

    // Set OCxM to "Force inactive level" (100)
    uint32_t force_inactive_value = (0b100 << (ccmr_shift + 4)); /* PDF Reference for OCxM bits (Force inactive) */

    if (ccmr_offset == 0) {
        TIMx->CCMR1 &= ~ocm_mask;
        TIMx->CCMR1 |= force_inactive_value;
    } else { // CCMR2 for channels 3 and 4
         TIMx->CCMR2 &= ~ocm_mask;
         TIMx->CCMR2 |= force_inactive_value;
    }

    // Generate an update event to force the output state immediately
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference for UG bit */
    // Clear the update flag (UIF)
    TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference for UIF bit */


    // Note: We don't disable the timer counter (CEN) here, only the specific channel output (CCxE).
    // This allows the timer to continue running, which might be necessary for synchronization
    // or other features if multiple channels on the same timer are used or if this timer is a master.
    // The counter will only be stopped in PWM_PowerOff if it's the last active channel on that timer.

}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        Stops all active PWM channels and disables associated clocks.
 */
void PWM_PowerOff(void)
{
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_COUNT; ++i)
    {
        if (pwm_initialized[i])
        {
            // Stop the individual channel
            // PWM_Stop function already disables the CCxE bit and forces output low.
            // It does NOT stop the timer counter itself.
             PWM_Stop(i);

            // Now, check if this timer is still needed by any other initialized channel.
            // If not, disable the timer clock and potentially the GPIO port clock.
            const pwm_channel_map_t *current_channel_info = &pwm_channel_map[i];
            TIM_TypeDef *TIMx = current_channel_info->timer_inst;
            GPIO_TypeDef *GPIOx = current_channel_info->gpio_port;

            uint8_t timer_still_needed = 0;
            uint8_t gpio_still_needed = 0;

            for (TRD_Channel_t j = 0; j < TRD_CHANNEL_COUNT; ++j)
            {
                if (pwm_initialized[j])
                {
                    // Check if another initialized channel uses the same timer
                    if (pwm_channel_map[j].timer_inst == TIMx)
                    {
                         // Check if any channel output on this timer is still enabled
                         // Need to re-read CCER for this timer instance
                         uint16_t ccer_val = TIMx->CCER;
                         // Mask bits for channels 1, 2, 3, 4 CCxE
                         uint16_t active_channels_mask = TIM_CCER_CC1E | TIM_CCER_CC2E;
                         if (TIMx == TIM1 || TIMx == TIM3 || TIMx == TIM4) { // TIM1, TIM3, TIM4 have channels 3 & 4
                             active_channels_mask |= TIM_CCER_CC3E | TIM_CCER_CC4E;
                         }

                         if (ccer_val & active_channels_mask)
                         {
                              timer_still_needed = 1; // At least one channel on this timer is still outputting
                         }

                         // Check if the timer counter is still enabled
                         if (TIMx->CR1 & TIM_CR1_CEN) {
                              timer_still_needed = 1; // Timer counter is still running
                         }

                    }

                    // Check if another initialized channel uses the same GPIO port
                    if (pwm_channel_map[j].gpio_port == GPIOx)
                    {
                        // Simple check: if *any* other initialized channel uses this port, keep its clock on.
                        // More advanced check would verify if any pin on this port is still configured as AF.
                         gpio_still_needed = 1;
                    }

                     if (timer_still_needed && gpio_still_needed) break; // Found dependents, no need to check further
                }
            }

            // If the timer is no longer needed, disable its clock and stop the counter
            if (!timer_still_needed) {
                TIMx->CR1 &= ~TIM_CR1_CEN; // Disable the counter /* PDF Reference for CEN bit */
                 if (current_channel_info->rcc_timer_mask & 0x0000FFFF) {
                    RCC->APB1ENR &= ~current_channel_info->rcc_timer_mask; /* Assumed RCC clock disable bit */
                 } else {
                    RCC->APB2ENR &= ~current_channel_info->rcc_timer_mask; /* Assumed RCC clock disable bit */
                 }
            }

            // If the GPIO port is no longer needed (assuming no other AF/GPIO use), disable its clock
            // Note: This check is basic. In a real system, other peripherals might use the same GPIO port.
            // A more robust approach would require tracking GPIO pin usage globally.
            // For this exercise, we assume if no initialized PWM channel uses the port, it can be disabled.
             if (!gpio_still_needed) {
                 RCC->AHB1ENR &= ~current_channel_info->rcc_gpio_mask; /* Assumed RCC clock disable bit */
             }


            pwm_initialized[i] = 0; // Mark channel as uninitialized
        }
    }
}