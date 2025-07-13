/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

// Assume SystemCoreClock or similar define exists elsewhere, providing the main system clock frequency in Hz.
// The timer clock frequency depends on the APB prescalers. For STM32F4, APB1 timers are typically
// clocked at APB1 clock if APB1 prescaler is 1, or twice the APB1 clock if APB1 prescaler is > 1.
// Similarly for APB2 timers. This information is NOT provided in the given PDF excerpt.
// We will assume a fixed timer clock frequency for all timers for calculation simplicity,
// and note that this assumption requires configuration elsewhere (e.g., RCC setup).
// This constant should ideally come from the header or a configuration file based on the actual clock setup.
#define PWM_TIMER_CLOCK_FREQ_HZ 84000000UL // Assumed timer clock frequency - please verify

// Definition of Timer and GPIO configurations for PWM channels.
// This mapping is based on typical STM32F401RC alternate functions but
// is NOT derived from the provided PDF content, as the specific AF map
// table is not included in the excerpt.
// Timers TIM2 and TIM3 are reserved for potential OS/Delay use as per requirements.
// Available timers for PWM: TIM1, TIM4, TIM5, TIM9, TIM10, TIM11.
// GPIO Pin 0 is avoided unless its PWM capability is explicitly confirmed in the PDF (it is not).
typedef struct {
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin_mask; // Pin number as a bit mask (e.g., 1 << 8 for pin 8)
    TIM_TypeDef *timer;
    uint8_t timer_channel; // 1, 2, 3, or 4
    uint8_t alternate_function; // AF value (0-15)
    uint32_t rcc_gpio_en_mask; // Mask for RCC->AHB1ENR to enable GPIO port clock
    uint32_t rcc_timer_en_mask; // Mask for RCC->APBxENR to enable Timer clock
    uint8_t timer_bits; // 16 or 32
} PWM_Config_t;

// Assume symbolic names like GPIOA, TIM1, RCC_AHB1ENR_GPIOAEN, GPIO_AF1_TIM1 etc.
// are defined in "STM32F401RC_PWM.h" or equivalent CMSIS/HAL headers included by it.
// Using bit masks for gpio_pin_mask (e.g., (1 << 8) for PA8) might be more bare-metal friendly,
// assuming GPIO_PIN_x macros are not available. Let's use bit masks for clarity with PDF bit descriptions.
#define GPIO_PIN_TO_MASK(pin_number) (1U << (pin_number))

// Map TRD_Channel_t enum to specific hardware configurations
static const PWM_Config_t pwm_channel_map[] = {
    // TIM1 Channels (APB2, 16-bit) - Typically on GPIOA, GPIOB, some others. Using common ones > pin 0.
    { GPIOA, GPIO_PIN_TO_MASK(8),  TIM1, 1, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, 16 }, // TRD_CHANNEL_1 -> TIM1_CH1 on PA8 (AF1) /* Assumed PWM config - please verify */
    { GPIOA, GPIO_PIN_TO_MASK(9),  TIM1, 2, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, 16 }, // TRD_CHANNEL_2 -> TIM1_CH2 on PA9 (AF1) /* Assumed PWM config - please verify */
    { GPIOA, GPIO_PIN_TO_MASK(10), TIM1, 3, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, 16 }, // TRD_CHANNEL_3 -> TIM1_CH3 on PA10 (AF1) /* Assumed PWM config - please verify */
    { GPIOA, GPIO_PIN_TO_MASK(11), TIM1, 4, GPIO_AF_TIM1,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, 16 }, // TRD_CHANNEL_4 -> TIM1_CH4 on PA11 (AF1) /* Assumed PWM config - please verify */

    // TIM4 Channels (APB1, 16-bit) - Typically on GPIOB, some others. Using common ones > pin 0.
    { GPIOB, GPIO_PIN_TO_MASK(6),  TIM4, 1, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, 16 }, // TRD_CHANNEL_5 -> TIM4_CH1 on PB6 (AF2) /* Assumed PWM config - please verify */
    { GPIOB, GPIO_PIN_TO_MASK(7),  TIM4, 2, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, 16 }, // TRD_CHANNEL_6 -> TIM4_CH2 on PB7 (AF2) /* Assumed PWM config - please verify */
    { GPIOB, GPIO_PIN_TO_MASK(8),  TIM4, 3, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, 16 }, // TRD_CHANNEL_7 -> TIM4_CH3 on PB8 (AF2) /* Assumed PWM config - please verify */
    { GPIOB, GPIO_PIN_TO_MASK(9),  TIM4, 4, GPIO_AF_TIM4,  RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, 16 }, // TRD_CHANNEL_8 -> TIM4_CH4 on PB9 (AF2) /* Assumed PWM config - please verify */

    // TIM5 Channels (APB1, 32-bit) - Typically on GPIOA, GPIOC. Avoid PA0/PC0 (pin 0).
    { GPIOA, GPIO_PIN_TO_MASK(1),  TIM5, 2, GPIO_AF_TIM5,  RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, 32 }, // TRD_CHANNEL_9 -> TIM5_CH2 on PA1 (AF2) /* Assumed PWM config - please verify */
    { GPIOA, GPIO_PIN_TO_MASK(2),  TIM5, 3, GPIO_AF_TIM5,  RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, 32 }, // TRD_CHANNEL_10 -> TIM5_CH3 on PA2 (AF2) /* Assumed PWM config - please verify */
    { GPIOA, GPIO_PIN_TO_MASK(3),  TIM5, 4, GPIO_AF_TIM5,  RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, 32 }, // TRD_CHANNEL_11 -> TIM5_CH4 on PA3 (AF2) /* Assumed PWM config - please verify */

    // TIM9 Channels (APB2, 16-bit) - Typically on GPIOA.
    { GPIOA, GPIO_PIN_TO_MASK(2),  TIM9, 1, GPIO_AF_TIM9,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM9EN, 16 }, // TRD_CHANNEL_12 -> TIM9_CH1 on PA2 (AF3) /* Assumed PWM config - please verify */ // Note: PA2 is also TIM5_CH3 (AF2). AF needs to be set correctly.
    { GPIOA, GPIO_PIN_TO_MASK(3),  TIM9, 2, GPIO_AF_TIM9,  RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM9EN, 16 }, // TRD_CHANNEL_13 -> TIM9_CH2 on PA3 (AF3) /* Assumed PWM config - please verify */ // Note: PA3 is also TIM5_CH4 (AF2). AF needs to be set correctly.

    // TIM10 Channel (APB2, 16-bit) - Typically on GPIOB.
    { GPIOB, GPIO_PIN_TO_MASK(8),  TIM10, 1, GPIO_AF_TIM10, RCC_AHB1ENR_GPIOBEN, RCC_APB2ENR_TIM10EN, 16 }, // TRD_CHANNEL_14 -> TIM10_CH1 on PB8 (AF3) /* Assumed PWM config - please verify */ // Note: PB8 is also TIM4_CH3 (AF2).

    // TIM11 Channel (APB2, 16-bit) - Typically on GPIOB.
    { GPIOB, GPIO_PIN_TO_MASK(9),  TIM11, 1, GPIO_AF_TIM11, RCC_AHB1ENR_GPIOBEN, RCC_APB2ENR_TIM11EN, 16 }, // TRD_CHANNEL_15 -> TIM11_CH1 on PB9 (AF3) /* Assumed PWM config - please verify */ // Note: PB9 is also TIM4_CH4 (AF2).
};

#define PWM_CHANNEL_COUNT (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))

/**
 * @brief Get the PWM configuration for a given TRD channel.
 * @param TRD_Channel The TRD channel identifier.
 * @return Pointer to the PWM_Config_t structure, or NULL if channel is invalid.
 */
static const PWM_Config_t* get_pwm_config(TRD_Channel_t TRD_Channel) {
    // Ensure TRD_Channel_t maps to a valid index
    if (TRD_Channel >= PWM_CHANNEL_COUNT) {
        return NULL; // Invalid channel
    }
    return &pwm_channel_map[TRD_Channel];
}

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel The channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    const PWM_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL) {
        // Handle error: Invalid channel
        return;
    }

    // 1. Enable GPIO clock
    // Using assumed RCC_AHB1ENR base address and masks from header
    RCC->AHB1ENR |= config->rcc_gpio_en_mask; /* Assumed RCC_AHB1ENR register & bit masks */

    // 2. Configure GPIO pin for Alternate Function
    uint32_t pin_number = 0;
    // Find the pin number from the mask
    for (uint32_t i = 0; i < 16; i++) {
        if ((config->gpio_pin_mask >> i) & 1) {
            pin_number = i;
            break;
        }
    }

    // Configure MODER to Alternate function mode (10)
    // PDF Reference: Table 24, Table 27 (GPIOx_MODER), Section 8.4.1
    config->gpio_port->MODER &= ~(0x03U << (pin_number * 2)); // Clear bits
    config->gpio_port->MODER |= (0x02U << (pin_number * 2)); // Set to Alternate function mode (10) /* PDF Reference */

    // Configure OTYPER to Push-pull (0)
    // PDF Reference: Table 24, Table 27 (GPIOx_OTYPER), Section 8.4.2
    config->gpio_port->OTYPER &= ~(0x01U << pin_number); // Set to Push-pull (0) /* PDF Reference */

    // Configure OSPEEDR (e.g., High speed 10)
    // PDF Reference: Table 24, Table 27 (GPIOx_OSPEEDR), Section 8.4.3
    config->gpio_port->OSPEEDR &= ~(0x03U << (pin_number * 2)); // Clear bits
    config->gpio_port->OSPEEDR |= (0x02U << (pin_number * 2)); // Set to High speed (10) /* PDF Reference */

    // Configure PUPDR to No pull-up, pull-down (00)
    // PDF Reference: Table 24, Table 27 (GPIOx_PUPDR), Section 8.4.4
    config->gpio_port->PUPDR &= ~(0x03U << (pin_number * 2)); // Set to No pull-up, pull-down (00) /* PDF Reference */

    // Configure AFRL/AFRH for the specific alternate function
    // PDF Reference: Table 27, Section 8.4.9 (AFRL), Section 8.4.10 (AFRH)
    if (pin_number < 8) {
        // Configure AFRL for pins 0-7
        config->gpio_port->AFRL &= ~(0x0FU << (pin_number * 4)); // Clear 4 bits
        config->gpio_port->AFRL |= (config->alternate_function << (pin_number * 4)); // Set AF value /* PDF Reference */
    } else {
        // Configure AFRH for pins 8-15
        config->gpio_port->AFRH &= ~(0x0FU << ((pin_number - 8) * 4)); // Clear 4 bits
        config->gpio_port->AFRH |= (config->alternate_function << ((pin_number - 8) * 4)); // Set AF value /* PDF Reference */
    }

    // 3. Enable Timer clock
    // Using assumed RCC_APBxENR base addresses and masks from header
    if (config->timer == TIM1 || config->timer == TIM9 || config->timer == TIM10 || config->timer == TIM11) {
        // Assume TIM1, TIM9-11 are on APB2
        RCC->APB2ENR |= config->rcc_timer_en_mask; /* Assumed RCC_APB2ENR register & bit mask */
    } else { // TIM2, TIM3, TIM4, TIM5
        // Assume TIM2-5 are on APB1
        RCC->APB1ENR |= config->rcc_timer_en_mask; /* Assumed RCC_APB1ENR register & bit mask */
    }

    // 4. Configure Timer Time-base
    // PDF Reference: TIMx_CR1 (12.4.1, 13.4.1, 14.4.1, 14.5.1)
    config->timer->CR1 &= ~TIM_CR1_CEN; // Disable counter during configuration /* PDF Reference */
    config->timer->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); // Set to Edge-aligned mode (00), Upcounting (0) /* PDF Reference */
    config->timer->CR1 |= TIM_CR1_ARPE; // Enable auto-reload preload /* PDF Reference */
    config->timer->PSC = 0; // Set prescaler to 0 initially (maximum timer clock) /* PDF Reference */

    // Determine max ARR based on timer bits
    uint32_t max_arr = (config->timer_bits == 32) ? 0xFFFFFFFFU : 0xFFFFU;
    config->timer->ARR = max_arr; // Set ARR to max value initially (minimum frequency) /* PDF Reference */

    // 5. Configure Timer Channel for PWM
    // PDF Reference: TIMx_CCMR1/CCMR2 (12.4.7/12.4.8, 13.4.7/13.4.8, 14.4.6/14.5.5)
    // PDF Reference: TIMx_CCER (12.4.9, 13.4.9, 14.4.7, 14.5.6)
    volatile uint32_t *ccmr_reg;
    uint32_t ccmr_shift;
    volatile uint32_t *ccr_reg;
    uint32_t ccer_shift;

    // Determine CCMR register, bit shift, CCR register, and CCER bit shift based on channel number
    switch (config->timer_channel) {
        case 1:
            ccmr_reg = &(config->timer->CCMR1);
            ccmr_shift = 0; // Bits 7:0 for channel 1 in CCMR1
            ccr_reg = &(config->timer->CCR1); /* PDF Reference */
            ccer_shift = 0; // Bit 0 for CC1E, Bit 1 for CC1P, etc.
            break;
        case 2:
            ccmr_reg = &(config->timer->CCMR1);
            ccmr_shift = 8; // Bits 15:8 for channel 2 in CCMR1
            ccr_reg = &(config->timer->CCR2); /* PDF Reference */
            ccer_shift = 4; // Bit 4 for CC2E, Bit 5 for CC2P, etc.
            break;
        case 3:
            ccmr_reg = &(config->timer->CCMR2);
            ccmr_shift = 0; // Bits 7:0 for channel 3 in CCMR2
            ccr_reg = &(config->timer->CCR3); /* PDF Reference */
            ccer_shift = 8; // Bit 8 for CC3E, Bit 9 for CC3P, etc.
            break;
        case 4:
            ccmr_reg = &(config->timer->CCMR2);
            ccmr_shift = 8; // Bits 15:8 for channel 4 in CCMR2
            ccr_reg = &(config->timer->CCR4); /* PDF Reference */
            ccer_shift = 12; // Bit 12 for CC4E, Bit 13 for CC4P, etc.
            break;
        default:
            // Should not happen with valid channels in map
            return;
    }

    // Clear CCxS bits (Input/Output selection) - Set to Output (00)
    // PDF Reference: CCMRx, CCxS (12.4.7/12.4.8, 13.4.7/13.4.8, 14.4.6/14.5.5)
    *ccmr_reg &= ~(0x03U << ccmr_shift); // Clear CCxS bits /* PDF Reference */

    // Configure OCxM to PWM mode 1 (110) and enable OCxPE (Preload enable)
    // PDF Reference: CCMRx, OCxM, OCxPE (12.4.7/12.4.8, 13.4.7/13.4.8, 14.4.6/14.5.5)
    *ccmr_reg &= ~(0x07U << (ccmr_shift + 4)); // Clear OCxM bits (bits 4-6 relative to shift) /* PDF Reference */
    *ccmr_reg |= (0x06U << (ccmr_shift + 4)); // Set OCxM to PWM mode 1 (110) /* PDF Reference */
    *ccmr_reg |= (0x01U << (ccmr_shift + 3)); // Set OCxPE (bit 3 relative to shift) /* PDF Reference */

    // Configure CCER to set output polarity (CCxP) and disable output initially (CCxE)
    // PDF Reference: CCER, CCxE, CCxP (12.4.9, 13.4.9, 14.4.7, 14.5.6)
    config->timer->CCER &= ~((TIM_CCER_CCxE | TIM_CCER_CCxP) << ccer_shift); // Clear CCxE and CCxP bits /* PDF Reference */
    // config->timer->CCER |= (0x00U << ccer_shift); // CCxE=0 (Output disabled), CCxP=0 (Active high) - already cleared above

    // For TIM1 (Advanced Timer), enable Main Output Enable (MOE) in BDTR
    // PDF Reference: TIM1_BDTR (12.4.18) vs General Purpose Timers
    if (config->timer == TIM1) {
        // BDTR exists for TIM1, BDTR offset is 0x44 (PDF 12.4.18)
        // MOE bit is bit 15 (PDF 12.4.18)
         config->timer->BDTR |= TIM_BDTR_MOE; /* PDF Reference */ // Enable main output (BDTR is 32-bit)
    }

    // 6. Generate an update event to load preload registers with initial values
    // PDF Reference: TIMx_EGR (12.4.6, 13.4.6, 14.4.5, 14.5.4)
    config->timer->EGR |= TIM_EGR_UG; /* PDF Reference */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    const PWM_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL || frequency == 0 || duty > 100) {
        // Handle error: Invalid channel, frequency, or duty cycle
        return;
    }

    // Get timer clock frequency (Assumed constant based on setup outside this driver)
    uint32_t timer_clock = PWM_TIMER_CLOCK_FREQ_HZ; // Assumed constant - please verify

    // Calculate the total number of timer clock cycles per PWM period
    uint32_t period_cycles = timer_clock / frequency;

    uint32_t psc = 0;
    uint32_t arr = 0;
    uint32_t max_arr = (config->timer_bits == 32) ? 0xFFFFFFFFU : 0xFFFFU;

    // Find suitable PSC and ARR values
    // Iterate through PSC to find the smallest value that results in ARR within the timer's capacity
    // Start PSC from 0
    for (psc = 0; psc <= 0xFFFFU; psc++) { // PSC is 16-bit (PDF 12.4.11, 13.4.11, 14.4.9, 14.5.8)
        // Check if period_cycles is divisible by (psc + 1) to get an integer ARR
        if (period_cycles % (psc + 1) == 0) {
            arr = (period_cycles / (psc + 1)) - 1;
            // Check if the resulting ARR fits within the timer's capacity
            if (arr <= max_arr) {
                // Found a valid PSC and ARR combination
                break;
            }
        }
        // If period_cycles is not perfectly divisible, check the floor value of ARR
        // This might slightly deviate from the exact frequency
        arr = (period_cycles / (psc + 1)) - 1;
         if (arr <= max_arr) {
             // Found a valid PSC and ARR combination (with potential frequency inaccuracy)
             // For production, perfect division or closest approximation should be considered.
             break;
         }

        // If psc reaches max and no valid ARR found, period_cycles is too large.
        if (psc == 0xFFFFU) {
             // Handle error: Cannot achieve desired frequency with available timer clock
             return; // Frequency too low or timer clock too high
        }
    }

    // Set PSC and ARR registers
    // PDF Reference: TIMx_PSC (12.4.11, 13.4.11, 14.4.9, 14.5.8), TIMx_ARR (12.4.12, 13.4.12, 14.4.10, 14.5.9)
    config->timer->PSC = psc; /* PDF Reference */
    config->timer->ARR = arr; /* PDF Reference */

    // Calculate the Capture/Compare value (pulse length) based on duty cycle
    // PDF Reference: Section 12.3.10, 13.3.9, 14.3.9 (PWM mode description)
    // Formula: Pulse = (ARR + 1) * duty / 100
    uint32_t pulse_cycles = (period_cycles * duty) / 100;

    // Handle 0% and 100% duty cycle based on PDF description (CCRx=0 for 0%, CCRx > ARR for 100%)
    // PDF Reference: Section 12.3.10, 13.3.9 (mentions CCRx > ARR for 100% in upcounting mode 1)
    // For safety/robustness, set CCRx=0 for 0% and CCRx=ARR+1 for 100%
    if (duty == 0) {
        pulse_cycles = 0;
    } else if (duty == 100) {
        pulse_cycles = arr + 1; // Set CCRx >= ARR+1 for 100% duty in upcounting PWM1
    } else {
         // For duty cycles between 1 and 99, CCRx is the number of active counts (pulse_cycles)
         // In upcounting mode 1, OCxREF is high when CNT < CCRx.
         // So, to be high for `pulse_cycles`, CCRx should be `pulse_cycles`.
         // The counter counts from 0 to ARR. A count of `k` means `k+1` cycles have passed (0, 1, ..., k).
         // If CCRx is set to `N`, the output is high for counts 0, 1, ..., N-1. This is N cycles.
         // So, `CCRx` should be set to `pulse_cycles`.
        pulse_cycles = (period_cycles * duty) / 100;
    }

     // Ensure pulse_cycles does not exceed ARR + 1 for 100% (or max timer value)
     if (pulse_cycles > arr + 1) {
        pulse_cycles = arr + 1;
     }


    // Set CCRx register
    // PDF Reference: TIMx_CCR1-4 (12.4.14-12.4.17, 13.4.13-13.4.16, 14.4.11/14.4.12/14.5.10)
    volatile uint32_t *ccr_reg;
     switch (config->timer_channel) {
        case 1: ccr_reg = &(config->timer->CCR1); break; /* PDF Reference */
        case 2: ccr_reg = &(config->timer->CCR2); break; /* PDF Reference */
        case 3: ccr_reg = &(config->timer->CCR3); break; /* PDF Reference */
        case 4: ccr_reg = &(config->timer->CCR4); break; /* PDF Reference */
        default: return; // Should not happen
     }
    *ccr_reg = pulse_cycles; /* PDF Reference */

    // Generate an update event to load the new PSC, ARR, and CCRx values
    // PDF Reference: TIMx_EGR (12.4.6, 13.4.6, 14.4.5, 14.5.4)
    // This is needed if timer is running and preload is enabled (which we configured in Init)
    config->timer->EGR |= TIM_EGR_UG; /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    const PWM_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL) {
        // Handle error: Invalid channel
        return;
    }

    // Determine CCER bit shift based on channel number
    uint32_t ccer_shift;
     switch (config->timer_channel) {
        case 1: ccer_shift = 0; break;
        case 2: ccer_shift = 4; break;
        case 3: ccer_shift = 8; break;
        case 4: ccer_shift = 12; break;
        default: return; // Should not happen
     }

    // Enable the output pin for the channel
    // PDF Reference: CCER, CCxE (12.4.9, 13.4.9, 14.4.7, 14.5.6)
    config->timer->CCER |= (TIM_CCER_CCxE << ccer_shift); /* PDF Reference */

    // For TIM1 (Advanced Timer), ensure Main Output Enable (MOE) is set
    // PDF Reference: TIM1_BDTR (12.4.18)
    if (config->timer == TIM1) {
         config->timer->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Enable the timer counter
    // PDF Reference: CR1, CEN (12.4.1, 13.4.1, 14.4.1, 14.5.1)
    config->timer->CR1 |= TIM_CR1_CEN; /* PDF Reference */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    const PWM_Config_t *config = get_pwm_config(TRD_Channel);
    if (config == NULL) {
        // Handle error: Invalid channel
        return;
    }

    // Determine CCER bit shift based on channel number
    uint32_t ccer_shift;
     switch (config->timer_channel) {
        case 1: ccer_shift = 0; break;
        case 2: ccer_shift = 4; break;
        case 3: ccer_shift = 8; break;
        case 4: ccer_shift = 12; break;
        default: return; // Should not happen
     }

    // Disable the output pin for the channel
    // PDF Reference: CCER, CCxE (12.4.9, 13.4.9, 14.4.7, 14.5.6) - Setting CCxE=0 disables output.
    config->timer->CCER &= ~(TIM_CCER_CCxE << ccer_shift); /* PDF Reference */

    // Note: We do not disable the timer counter (CEN) or TIM1's MOE here,
    // as these affect all channels on the same timer. This function only
    // stops the output for the specified channel.
}

/**
 * @brief Disables all PWM peripherals and outputs.
 * This function stops all configured PWM channels and disables the associated
 * timers and GPIO clocks to reduce power consumption.
 */
void PWM_PowerOff(void) {
    // Keep track of which timers/GPIO ports have been processed to disable clocks only once
    uint32_t timers_enabled_masks_apb1 = 0;
    uint32_t timers_enabled_masks_apb2 = 0;
    uint32_t gpios_enabled_masks = 0;

    // Iterate through all configured PWM channels
    for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        const PWM_Config_t *config = &pwm_channel_map[i];

        // Disable the output pin for the channel
        // PDF Reference: CCER, CCxE (12.4.9, 13.4.9, 14.4.7, 14.5.6)
        uint32_t ccer_shift;
        switch (config->timer_channel) {
           case 1: ccer_shift = 0; break;
           case 2: ccer_shift = 4; break;
           case 3: ccer_shift = 8; break;
           case 4: ccer_shift = 12; break;
           default: continue; // Should not happen
        }
        config->timer->CCER &= ~(TIM_CCER_CCxE << ccer_shift); /* PDF Reference */

        // For TIM1 (Advanced Timer), clear Main Output Enable (MOE)
        // PDF Reference: TIM1_BDTR (12.4.18)
        if (config->timer == TIM1) {
             config->timer->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
        }

        // Disable the timer counter
        // PDF Reference: CR1, CEN (12.4.1, 13.4.1, 14.4.1, 14.5.1)
        config->timer->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

        // Configure the GPIO pin back to Input mode (reset state is Input Floating per PDF 8.3.1)
        // PDF Reference: MODER (Table 24, Table 27, Section 8.4.1)
        uint32_t pin_number = 0;
        for (uint32_t p = 0; p < 16; p++) {
            if ((config->gpio_pin_mask >> p) & 1) {
                pin_number = p;
                break;
            }
        }
        // Set MODER to Input (00)
        config->gpio_port->MODER &= ~(0x03U << (pin_number * 2)); // Set to Input (00) /* PDF Reference */

        // Store the RCC enable masks to disable clocks later
        gpios_enabled_masks |= config->rcc_gpio_en_mask;
        if (config->timer == TIM1 || config->timer == TIM9 || config->timer == TIM10 || config->timer == TIM11) {
             timers_enabled_masks_apb2 |= config->rcc_timer_en_mask;
        } else { // TIM2, TIM3, TIM4, TIM5
             timers_enabled_masks_apb1 |= config->rcc_timer_en_mask;
        }
    }

    // Disable the timer clocks in RCC (disable only those that were used)
    // Using assumed RCC_APBxENR base addresses and masks from header
    RCC->APB1ENR &= ~timers_enabled_masks_apb1; /* Assumed RCC_APB1ENR register */
    RCC->APB2ENR &= ~timers_enabled_masks_apb2; /* Assumed RCC_APB2ENR register */

    // Note: Disabling GPIO clocks here might affect other peripherals using those GPIO ports.
    // A more robust implementation might track GPIO usage per peripheral.
    // we disable clocks for the GPIO ports used by the configured PWM channels.
    // Using assumed RCC_AHB1ENR base address and masks from header
    RCC->AHB1ENR &= ~gpios_enabled_masks; /* Assumed RCC_AHB1ENR register */
}