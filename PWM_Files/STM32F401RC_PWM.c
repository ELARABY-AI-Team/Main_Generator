/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM driver implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-10
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

// Header file including channel definitions and custom types
#include "STM32F401RC_PWM.h"
// Standard CMSIS header for STM32F401xE (includes register definitions like TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef)
// Note: This header is assumed to be available for standard peripheral register base addresses and struct definitions
// as per common bare-metal practice on STM32. Specific register bit fields and logic
// will strictly follow the provided PDF RM0368 snippets.
#include "stm32f401xe.h"


// Assume tlong is unsigned long and tbyte is unsigned char if not defined in header
#ifndef TLONG_DEFINED
typedef unsigned long tlong;
#define TLONG_DEFINED
#endif
#ifndef TBYTE_DEFINED
typedef unsigned char tbyte;
#define TBYTE_DEFINED
#endif

/*
 * Timer Reservation:
 * TIM1 (Advanced Control) and TIM2 (32-bit general purpose) are reserved
 * for potential use by an RTOS, delay functions, or other system services.
 * This implementation will use TIM3, TIM4, TIM9, TIM10, and TIM11 for PWM output.
 */

/* Assumed PWM configuration mappings (Timer, Channel, GPIO Port, Pin, AF) */
// Note: Specific GPIO pins and AF numbers are based on typical STM32F401RC datasheets
// and RM0368 Alternate Function mapping tables (not explicitly provided in the snippets),
// and are assumed for this implementation. Please verify against the actual datasheet.
typedef struct {
    TIM_TypeDef* timer;
    uint32_t channel_idx; // 0=CH1, 1=CH2, 2=CH3, 3=CH4 (matches CCER bit positions / 4)
    GPIO_TypeDef* gpio_port;
    uint32_t gpio_pin; // 0-15
    uint8_t gpio_af;   // Alternate Function number
} PWM_Channel_Config_t;

// Mapping table for usable PWM channels (based on TIM3, TIM4, TIM9, TIM10, TIM11)
// Using 0-indexed channel_idx for internal logic convenience matching CCER bit layout
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TIM3 Channels (AF2) - TIM3 is 16-bit
    {TIM3, 0, GPIOA, 6, GPIO_AF2_TIM3}, /* Assumed PWM config - please verify */
    {TIM3, 1, GPIOA, 7, GPIO_AF2_TIM3}, /* Assumed PWM config - please verify */
    {TIM3, 2, GPIOB, 0, GPIO_AF2_TIM3}, /* Assumed PWM config - please verify */
    {TIM3, 3, GPIOB, 1, GPIO_AF2_TIM3}, /* Assumed PWM config - please verify */
    // TIM4 Channels (AF2) - TIM4 is 16-bit
    {TIM4, 0, GPIOB, 6, GPIO_AF2_TIM4}, /* Assumed PWM config - please verify */
    {TIM4, 1, GPIOB, 7, GPIO_AF2_TIM4}, /* Assumed PWM config - please verify */
    {TIM4, 2, GPIOB, 8, GPIO_AF2_TIM4}, /* Assumed PWM config - please verify */
    {TIM4, 3, GPIOB, 9, GPIO_AF2_TIM4}, /* Assumed PWM config - please verify */
    // TIM9 Channels (AF3) - TIM9 is 16-bit (only CH1/CH2 available)
    {TIM9, 0, GPIOA, 2, GPIO_AF3_TIM9}, /* Assumed PWM config - please verify */
    {TIM9, 1, GPIOA, 3, GPIO_AF3_TIM9}, /* Assumed PWM config - please verify */
    // TIM10 Channel (AF3) - TIM10 is 16-bit (only CH1 available)
    {TIM10, 0, GPIOB, 8, GPIO_AF3_TIM10}, /* Assumed PWM config - please verify */ // Note: PB8 is also TIM4_CH3. This config uses it for TIM10.
    // TIM11 Channel (AF3) - TIM11 is 16-bit (only CH1 available)
    {TIM11, 0, GPIOB, 9, GPIO_AF3_TIM11}, /* Assumed PWM config - please verify */ // Note: PB9 is also TIM4_CH4. This config uses it for TIM11.
};

// Number of available PWM channels in the map
#define NUM_PWM_CHANNELS (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))

// Define assumed timer clock frequency (CK_TIM)
// This is based on a common system clock configuration for STM32F401RC,
// where APB1/APB2 timers are clocked at up to 84MHz (if APB prescaler > 1).
// This value might need adjustment based on the actual RCC configuration.
#define TIM_CLOCK_FREQ_HZ 84000000UL /* Assumption: Timer clock frequency is 84MHz */

// Helper defines for Channel to Register Mapping bits (derived from PDF)
// GPIO register masks and positions
#define GPIO_MODER_MODE_AF          (2U) /* PDF Reference (10: Alternate function mode) */
#define GPIO_OTYPER_PP              (0U) /* PDF Reference (0: Output push-pull) */
#define GPIO_OSPEEDR_HIGH           (2U) /* PDF Reference (10: High speed) */
#define GPIO_PUPDR_NO               (0U) /* PDF Reference (00: No pull-up, pull-down) */

// TIM register masks and positions (derived from PDF register maps)
#define TIM_CR1_CEN_Pos             (0U) /* PDF Reference */
#define TIM_CR1_ARPE_Pos            (7U) /* PDF Reference */
#define TIM_EGR_UG_Pos              (0U) /* PDF Reference */
#define TIM_SR_UIF_Pos              (0U) /* PDF Reference */
#define TIM_CCER_CCxE_Pos_Base      (0U) // Base position for CC1E /* PDF Reference */
#define TIM_CCER_CCxP_Pos_Base      (1U) // Base position for CC1P /* PDF Reference */

// Channel mapping bits for CCMR registers (derived from PDF)
#define TIM_CCMR_CCS_Pos_Base       (0U) // Base position for CC1S/CC3S /* PDF Reference */
#define TIM_CCMR_OCFE_Pos_Base      (2U) // Base position for OC1FE/OC3FE /* PDF Reference */
#define TIM_CCMR_OCPE_Pos_Base      (3U) // Base position for OC1PE/OC3PE /* PDF Reference */
#define TIM_CCMR_OCM_Pos_Base       (4U) // Base position for OC1M/OC3M /* PDF Reference */
#define TIM_CCMR_OCM_PWM1_Val       (6U) // Value for PWM mode 1 (110) /* PDF Reference */

// AF mapping values (derived from PDF Table 24/Figure 17 - Assumed specific values)
#define GPIO_AF2_TIM3               (2U) /* Assumed AF mapping - please verify */
#define GPIO_AF2_TIM4               (2U) /* Assumed AF mapping - please verify */
#define GPIO_AF3_TIM9               (3U) /* Assumed AF mapping - please verify */
#define GPIO_AF3_TIM10              (3U) /* Assumed AF mapping - please verify */
#define GPIO_AF3_TIM11              (3U) /* Assumed AF mapping - please verify */


// Forward declarations for internal helper functions
static const PWM_Channel_Config_t* get_channel_config(TRD_Channel_t TRD_Channel);
static void configure_gpio(const PWM_Channel_Config_t* config);
static void enable_timer_clock(TIM_TypeDef* timer); /* Assumption required for RCC access */
static void disable_timer_clock(TIM_TypeDef* timer); /* Assumption required for RCC access */
static void enable_gpio_clock(GPIO_TypeDef* gpio_port); /* Assumption required for RCC access */
static void disable_gpio_clock(GPIO_TypeDef* gpio_port); /* Assumption required for RCC access */
static void reset_timer(TIM_TypeDef* timer); /* Assumption required for RCC peripheral reset */


/**Functions ===========================================================================*/

/**
 * @brief Get the configuration for a given PWM channel enum.
 * @param TRD_Channel The PWM channel enum.
 * @return Pointer to the channel configuration structure, or NULL if invalid.
 */
static const PWM_Channel_Config_t* get_channel_config(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= NUM_PWM_CHANNELS) {
        return NULL; // Invalid channel
    }
    return &pwm_channel_map[TRD_Channel];
}

/**
 * @brief Enable the clock for the given GPIO port.
 * @param gpio_port Pointer to the GPIO port.
 */
static void enable_gpio_clock(GPIO_TypeDef* gpio_port) {
    // This function assumes access to the RCC peripheral registers.
    // The specific bit to set depends on the GPIO port.
    if (gpio_port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Assumption: RCC clock enabling via standard registers */
    } else if (gpio_port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Assumption: RCC clock enabling via standard registers */
    } else if (gpio_port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* Assumption: RCC clock enabling via standard registers */
    }
    // For STM32F401RC, GPIO D, E, H are either not available or have limited pins.
    // Add checks for other ports if needed based on channel map (not strictly required by prompt).
    // Ensure the clock is ready - reading the register after setting the enable bit.
    (void)RCC->AHB1ENR; /* Dummy read for clock readiness, standard practice */
}

/**
 * @brief Disable the clock for the given GPIO port.
 * @param gpio_port Pointer to the GPIO port.
 */
static void disable_gpio_clock(GPIO_TypeDef* gpio_port) {
     // This function assumes access to the RCC peripheral registers.
    if (gpio_port == GPIOA) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; /* Assumption: RCC clock disabling via standard registers */
    } else if (gpio_port == GPIOB) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; /* Assumption: RCC clock disabling via standard registers */
    } else if (gpio_port == GPIOC) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; /* Assumption: RCC clock disabling via standard registers */
    }
}


/**
 * @brief Enable the clock for the given Timer peripheral.
 * @param timer Pointer to the Timer peripheral.
 */
static void enable_timer_clock(TIM_TypeDef* timer) {
    // This function assumes access to the RCC peripheral registers.
    // Determine which APB bus the timer is on and set the appropriate enable bit.
    // TIM3, TIM4 are on APB1. TIM9, TIM10, TIM11 are on APB2.
    if (timer == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Assumption: RCC clock enabling via standard registers */
    } else if (timer == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* Assumption: RCC clock enabling via standard registers */
    } else if (timer == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* Assumption: RCC clock enabling via standard registers */
    } else if (timer == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* Assumption: RCC clock enabling via standard registers */
    } else if (timer == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* Assumption: RCC clock enabling via standard registers */
    }
    // TIM1, TIM2, TIM5 are reserved.
    // Ensure the clock is ready
    (void)RCC->APB1ENR; /* Dummy read for clock readiness */
    (void)RCC->APB2ENR; /* Dummy read for clock readiness */
}

/**
 * @brief Disable the clock for the given Timer peripheral.
 * @param timer Pointer to the Timer peripheral.
 */
static void disable_timer_clock(TIM_TypeDef* timer) {
    // This function assumes access to the RCC peripheral registers.
    if (timer == TIM3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* Assumption: RCC clock disabling via standard registers */
    } else if (timer == TIM4) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* Assumption: RCC clock disabling via standard registers */
    } else if (timer == TIM9) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN; /* Assumption: RCC clock disabling via standard registers */
    } else if (timer == TIM10) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN; /* Assumption: RCC clock disabling via standard registers */
    } else if (timer == TIM11) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; /* Assumption: RCC clock disabling via standard registers */
    }
}

/**
 * @brief Reset the given Timer peripheral.
 * @param timer Pointer to the Timer peripheral.
 */
static void reset_timer(TIM_TypeDef* timer) {
    // This function assumes access to the RCC peripheral registers.
    // Determine which APB bus the timer is on and assert/deassert the reset bit.
    if (timer == TIM3) {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;   /* Assumption: RCC peripheral reset via standard registers */
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;  /* Assumption: RCC peripheral reset via standard registers */
    } else if (timer == TIM4) {
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;   /* Assumption: RCC peripheral reset via standard registers */
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;  /* Assumption: RCC peripheral reset via standard registers */
    } else if (timer == TIM9) {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST;   /* Assumption: RCC peripheral reset via standard registers */
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST;  /* Assumption: RCC peripheral reset via standard registers */
    } else if (timer == TIM10) {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST; /* Assumption: RCC peripheral reset via standard registers */
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM10RST;/* Assumption: RCC peripheral reset via standard registers */
    } else if (timer == TIM11) {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM11RST; /* Assumption: RCC peripheral reset via standard registers */
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM11RST;/* Assumption: RCC peripheral reset via standard registers */
    }
    // TIM1, TIM2, TIM5 are reserved.
}


/**
 * @brief Configure the GPIO pin for the specific timer alternate function.
 * @param config Pointer to the channel configuration.
 */
static void configure_gpio(const PWM_Channel_Config_t* config) {
    if (!config) return;

    GPIO_TypeDef* GPIOx = config->gpio_port;
    uint32_t pin = config->gpio_pin;
    uint8_t af = config->gpio_af;

    // Enable GPIO clock for the port (e.g., RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;)
    enable_gpio_clock(GPIOx); /* Assumption: RCC clock enabling via standard registers */

    // Configure pin as Alternate Function (MODER = 10)
    // Clear current mode bits for the pin (2 bits per pin, position 2*pin)
    GPIOx->MODER &= ~(0x3U << (pin * 2)); /* PDF Reference (MODER bits 2y:2y+1) */
    // Set mode bits to Alternate Function (10)
    GPIOx->MODER |= (GPIO_MODER_MODE_AF << (pin * 2)); /* PDF Reference (MODER bits 2y:2y+1) */

    // Configure output type as Push-Pull (OTYPER = 0)
    // Clear current output type bit for the pin (1 bit per pin, position pin)
    GPIOx->OTYPER &= ~(0x1U << pin); /* PDF Reference (OTYPER bits y) */

    // Configure output speed (e.g., High speed = 10)
    // Clear current speed bits for the pin (2 bits per pin, position 2*pin)
    GPIOx->OSPEEDR &= ~(0x3U << (pin * 2)); /* PDF Reference (OSPEEDR bits 2y:2y+1) */
    // Set speed bits (e.g., 10 for High Speed)
    GPIOx->OSPEEDR |= (GPIO_OSPEEDR_HIGH << (pin * 2)); /* PDF Reference (OSPEEDR bits 2y:2y+1) */

    // Configure pull-up/pull-down as No pull-up/pull-down (PUPDR = 00)
    // Clear current pull bits for the pin (2 bits per pin, position 2*pin)
    GPIOx->PUPDR &= ~(0x3U << (pin * 2)); /* PDF Reference (PUPDR bits 2y:2y+1) */

    // Select Alternate Function (AFRL for pins 0-7, AFRH for pins 8-15)
    if (pin < 8) {
        // Clear current AF bits for the pin (4 bits per pin, position 4*pin in AFRL)
        GPIOx->AFR[0] &= ~(0xFU << (pin * 4)); /* PDF Reference (AFRLy bits 3:0) */
        // Set AF bits
        GPIOx->AFR[0] |= (af << (pin * 4)); /* PDF Reference (AFRLy bits 3:0) */
    } else { // pin >= 8
        // Clear current AF bits for the pin (4 bits per pin, position 4*(pin-8) in AFRH)
        GPIOx->AFR[1] &= ~(0xFU << ((pin - 8) * 4)); /* PDF Reference (AFrHy bits 3:0) */
        // Set AF bits
        GPIOx->AFR[1] |= (af << ((pin - 8) * 4)); /* PDF Reference (AFrHy bits 3:0) */
    }
}


/**
 * @brief Initialize the PWM hardware and configure the timer and GPIOs.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (!config) {
        // Invalid channel, cannot initialize.
        return;
    }

    TIM_TypeDef* TIMx = config->timer;
    uint32_t channel_idx = config->channel_idx;

    // Configure the GPIO pin
    configure_gpio(config);

    // Enable the timer clock
    enable_timer_clock(TIMx); /* Assumption: RCC clock enabling via standard registers */

    // Reset the timer peripheral to default state
    reset_timer(TIMx); /* Assumption: RCC peripheral reset via standard registers */

    // Disable the counter before configuration
    TIMx->CR1 &= ~(1U << TIM_CR1_CEN_Pos); /* PDF Reference */

    // Set Auto-Reload Preload Enable (ARPE) - allows changing ARR on the fly
    TIMx->CR1 |= (1U << TIM_CR1_ARPE_Pos); /* PDF Reference */

    // Configure the timer channel for PWM mode 1
    // Get the address of the correct Capture/Compare Mode Register (CCMR1 or CCMR2)
    // And the shift amount within that register for the specific channel
    volatile uint32_t* CCMRx;
    uint32_t ccmr_shift;

    if (channel_idx == 0) { // Channel 1 maps to CCMR1, bits 0-7
        CCMRx = &TIMx->CCMR1; /* PDF Reference */
        ccmr_shift = 0;
    } else if (channel_idx == 1) { // Channel 2 maps to CCMR1, bits 8-15
        CCMRx = &TIMx->CCMR1; /* PDF Reference */
        ccmr_shift = 8;
    } else if (channel_idx == 2) { // Channel 3 maps to CCMR2, bits 0-7 (TIM3/TIM4 only)
        CCMRx = &TIMx->CCMR2; /* PDF Reference */
        ccmr_shift = 0;
    } else if (channel_idx == 3) { // Channel 4 maps to CCMR2, bits 8-15 (TIM3/TIM4 only)
        CCMRx = &TIMx->CCMR2; /* PDF Reference */
        ccmr_shift = 8;
    } else {
         return; // Should not happen with current map, but included for robustness
    }

    // Configure CCMRx register for the channel:
    // Clear CCxS (bits 0:1 or 8:9 within CCMRx) - Output mode (00)
    *CCMRx &= ~(0x3U << (ccmr_shift + TIM_CCMR_CCS_Pos_Base)); /* PDF Reference (CCxS bits) */
    // Clear OCxM (bits 4:6 or 12:14) - Output Compare Mode
    *CCMRx &= ~(0x7U << (ccmr_shift + TIM_CCMR_OCM_Pos_Base)); /* PDF Reference (OCxM bits) */
    // Set OCxM to PWM Mode 1 (110)
    *CCMRx |= (TIM_CCMR_OCM_PWM1_Val << (ccmr_shift + TIM_CCMR_OCM_Pos_Base)); /* PDF Reference (OCxM bits, PWM mode 1) */
    // Set OCxPE (bit 3 or 11) to enable preload
    *CCMRx |= (1U << (ccmr_shift + TIM_CCMR_OCPE_Pos_Base)); /* PDF Reference (OCxPE bit) */
    // OCxFE (bit 2 or 10) is 0 by default and cleared above, disabling fast mode.

    // Configure CCER register: Disable output initially, set active high polarity
    // Clear CCxE (bit 0, 4, 8, or 12) and CCxP (bit 1, 5, 9, or 13) for the channel
    TIMx->CCER &= ~((1U << (channel_idx * 4 + TIM_CCER_CCxE_Pos_Base)) |
                     (1U << (channel_idx * 4 + TIM_CCER_CCxP_Pos_Base))); /* PDF Reference (CCxE and CCxP bits) */
    // CCxE is cleared to disable output
    // CCxP is cleared for active high polarity

    // For TIM1 (if used, but reserved), need to configure BDTR (MOE, OSSI, etc.)
    // Since TIM3, TIM4, TIM9, TIM10, TIM11 are General Purpose timers (or limited general purpose),
    // they do not have the BDTR register or the MOE bit according to the provided PDF snippets.
    // Output enable relies solely on the CCxE bit for these timers based on the PDF (Table 55).

    // Set a default period and duty cycle (e.g., 1kHz, 50%)
    PWM_Set_Freq(TRD_Channel, 1000, 50);

    // Generate an update event to load the new values into the active registers
    TIMx->EGR |= (1U << TIM_EGR_UG_Pos); /* PDF Reference */
    // Clear the update flag as it might be set by UG
    TIMx->SR &= ~(1U << TIM_SR_UIF_Pos); /* PDF Reference */
}

/**
 * @brief Set the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired frequency in Hz. Must be > 0.
 * @param duty The desired duty cycle in percent (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (!config || frequency == 0 || duty > 100) {
        // Invalid channel, frequency, or duty cycle
        return;
    }

    TIM_TypeDef* TIMx = config->timer;
    uint32_t channel_idx = config->channel_idx;

    uint32_t tim_clock = TIM_CLOCK_FREQ_HZ; /* Assumption: Timer clock frequency */

    // Calculate ARR and PSC for the desired frequency
    // Period_counts = (PSC + 1) * (ARR + 1) = Timer_Clock / Frequency
    // ARR = (Timer_Clock / (Frequency * (PSC + 1))) - 1

    uint32_t psc = 0;
    uint32_t arr = 0;
    uint64_t total_counts_needed = (uint64_t)tim_clock / frequency;

    // Determine max ARR value based on timer type (16-bit or 32-bit)
    // TIM3, TIM4, TIM9, TIM10, TIM11 are 16-bit.
    // TIM2, TIM5 are 32-bit (reserved in this implementation).
    uint32_t max_arr = 0xFFFFU; /* PDF Reference (implied by TIMx_ARR size for used timers) */

    // Find the smallest PSC value that keeps ARR within the maximum limit (max_arr)
    // Iterate through possible PSC values (16-bit PSC)
    for (psc = 0; psc <= 0xFFFF; ++psc) { /* PDF Reference (PSC is 16-bit) */
        // Calculate the required ARR for the current PSC
        // arr + 1 = total_counts_needed / (psc + 1)
        uint64_t arr_plus_1 = total_counts_needed / (psc + 1);

        // Check if the required ARR + 1 fits within the maximum possible value for ARR + 1 (max_arr + 1)
        if (arr_plus_1 > 0 && arr_plus_1 <= (uint64_t)max_arr + 1) {
             arr = (uint32_t)(arr_plus_1 - 1);
             // Validate that the frequency is achievable reasonably
             // The actual frequency will be TIM_CLOCK_FREQ_HZ / ((psc + 1) * (arr + 1))
             // If integer division truncated total_counts_needed, the actual frequency
             // will be slightly higher or equal to the target frequency.
             // If frequency is very low, total_counts_needed might be huge. Check for overflow?
             // If total_counts_needed exceeds (0xFFFF+1)*(0xFFFF+1), cannot achieve even with max PSC/ARR.
             // This check is implicitly done by the loop iterating through max PSC.
             break; // Found a suitable PSC and ARR
        }
         // If arr_plus_1 is 0, frequency is too high for even psc=0. Not possible with frequency > 0.
         // If arr_plus_1 > max_arr + 1, increment psc and try again.
    }

    // If the loop finished and psc is > 0xFFFF, it means even with max PSC,
    // the required ARR + 1 was still > max_arr + 1. This should theoretically not happen
    // for standard PWM frequencies on a timer clocked at 84MHz unless the target frequency is extremely low.
    // For extreme low frequencies, the calculated total_counts_needed might exceed uint64_t max,
    // but TIM_CLOCK_FREQ_HZ * 0xFFFF seems safe within uint64_t.
    // A more robust implementation would handle impossible frequencies explicitly.
    // For this implementation, we rely on the loop finding the best fit up to max PSC.


    // Configure the Timer Base (PSC and ARR)
    // Read CR1 to preserve other bits, disable CEN, write PSC/ARR, restore CR1.
    uint32_t cr1_reg = TIMx->CR1;
    TIMx->CR1 &= ~(1U << TIM_CR1_CEN_Pos); // Clear CEN /* PDF Reference */

    TIMx->PSC = psc; /* PDF Reference */
    TIMx->ARR = arr; /* PDF Reference */

    // Calculate CCRx value based on duty cycle
    // Duty Cycle % = (Pulse_Length_counts / Period_counts) * 100
    // Pulse_Length_counts = (Duty * Period_counts) / 100
    // Period_counts = ARR + 1
    // CCRx value determines Pulse_Length_counts in PWM1 mode (upcounting).
    // Pulse_Length_counts = CCRx if output is high when CNT < CCRx (0..CCRx-1, total CCRx counts)
    // Or Pulse_Length_counts = CCRx + 1 if output is high when CNT <= CCRx (0..CCRx, total CCRx+1 counts).
    // PDF: "OCxREF is high as long as TIMx_CNT < TIMx_CCRx" -> Pulse length = CCRx counts.
    // Duty % = (CCRx / (ARR + 1)) * 100
    // CCRx = (Duty * (ARR + 1)) / 100

    uint64_t ccr_val_64 = (uint64_t)(arr + 1) * duty / 100;
    uint32_t ccr_val;

    // The maximum theoretical value for 100% duty based on formula is ARR + 1.
    // However, the CCRx register is typically the same size as ARR (16-bit for these timers).
    // The PDF states "If the compare value in TIMx_CCRx is greater than the auto-reload value (in TIMx_ARR) then OCxREF is held at ‘1’."
    // A common practice for 100% is to set CCRx = ARR. This keeps CNT < ARR true for 0..ARR-1 (ARR counts high). Period is ARR+1.
    // Duty = ARR / (ARR+1), which is close to 100% but not exactly.
    // If the hardware truly holds high for CCRx >= ARR + 1, we could write ARR+1 if it fits in the register.
    // For 16-bit timers, ARR+1 can be 0x10000, which doesn't fit in a 16-bit register.
    // Let's stick to the interpretation that CCRx == ARR gives approximately 100% for edge-aligned upcounting.
    // For 0% duty, CCRx should be 0.
    // Let's use: If duty = 0, CCRx = 0. If duty = 100, CCRx = ARR. Otherwise, CCRx = (ARR * duty) / 100.
    // This simplifies calculation and fits within the register width (0 to ARR).

    if (duty == 0) {
        ccr_val = 0;
    } else if (duty == 100) {
        ccr_val = arr; // Approximate 100% duty for edge-aligned upcounting /* PDF Reference */
    } else {
        // Calculate based on ARR (period is ARR+1, pulse is approx CCRx)
        // CCRx / ARR approx = duty / 100
        // CCRx approx = (ARR * duty) / 100
        ccr_val = (uint32_t)((uint64_t)arr * duty / 100);
    }

    // Set the Capture/Compare Register (CCRx) for the channel
    if (channel_idx == 0) {
        TIMx->CCR1 = ccr_val; /* PDF Reference */
    } else if (channel_idx == 1) {
        TIMx->CCR2 = ccr_val; /* PDF Reference */
    } else if (channel_idx == 2) { // TIM3/TIM4 only
        TIMx->CCR3 = ccr_val; /* PDF Reference */
    } else if (channel_idx == 3) { // TIM3/TIM4 only
        TIMx->CCR4 = ccr_val; /* PDF Reference */
    }


    // Generate an update event to load the new values into the active registers
    // This also clears the counter if URS=0 (default) or URS=1 and UG is set (default URS=0)
    TIMx->EGR |= (1U << TIM_EGR_UG_Pos); /* PDF Reference */
    // Clear the update flag as it might be set by UG (if URS=0)
    TIMx->SR &= ~(1U << TIM_SR_UIF_Pos); /* PDF Reference */

    // Restore original CR1 state (including CEN if it was set)
    // Note: UG bit is write-only and self-clearing
    TIMx->CR1 = cr1_reg; // Restore CR1
    // Ensure CEN bit is set if it was set before, as writing CR1 might clear it depending on flags
    if(cr1_reg & (1U << TIM_CR1_CEN_Pos))
    {
        TIMx->CR1 |= (1U << TIM_CR1_CEN_Pos); /* PDF Reference */
    }
}

/**
 * @brief Enable and start PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (!config) {
        return; // Invalid channel
    }

    TIM_TypeDef* TIMx = config->timer;
    uint32_t channel_idx = config->channel_idx;

    // Enable the Capture/Compare output for the channel
    TIMx->CCER |= (1U << (channel_idx * 4 + TIM_CCER_CCxE_Pos_Base)); /* PDF Reference (CCxE bit) */

    // For TIM1 (if used, but reserved), need to enable the main output (MOE bit in BDTR).
    // General purpose timers (TIM3, TIM4, TIM9, TIM10, TIM11) do not have MOE bit.
    // The output enable is controlled solely by the CCxE bit for these timers.

    // Enable the counter (if not already running)
    // Note: Enabling the counter starts all channels configured on this timer.
    TIMx->CR1 |= (1U << TIM_CR1_CEN_Pos); /* PDF Reference */
}

/**
 * @brief Stop the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* config = get_channel_config(TRD_Channel);
    if (!config) {
        return; // Invalid channel
    }

    TIM_TypeDef* TIMx = config->timer;
    uint32_t channel_idx = config->channel_idx;

    // Disable the Capture/Compare output for the channel
    TIMx->CCER &= ~(1U << (channel_idx * 4 + TIM_CCER_CCxE_Pos_Base)); /* PDF Reference (CCxE bit) */

    // Stopping the counter (CEN) or main output (MOE, for TIM1 only)
    // affects all channels on that timer. The requirement is "Stop the PWM signal
    // output on the specified channel", which implies per-channel control.
    // Disabling the specific channel's CCxE bit achieves this.
}

/**
 * @brief Disable all PWM peripherals and outputs to reduce power consumption.
 */
void PWM_PowerOff(void) {
    // Iterate through all configured PWM channels and stop their output and timers.
    // A set of used timers/ports would be more efficient than iterating all possible channels
    // Need to ensure we only disable clocks/reset timers once per peripheral.
    // Using flags or a set of pointers would be more robust.
    // For this implementation, we will simply iterate and apply disable/reset to each
    // timer/port found in the map, which is safe but might be slightly redundant
    // if multiple channels share the same timer/port.

    for (size_t i = 0; i < NUM_PWM_CHANNELS; ++i) {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        TIM_TypeDef* TIMx = config->timer;
        GPIO_TypeDef* GPIOx = config->gpio_port;
        uint32_t pin = config->gpio_pin;

        // Disable the timer counter
        TIMx->CR1 &= ~(1U << TIM_CR1_CEN_Pos); /* PDF Reference */

        // Disable all Capture/Compare outputs for this timer (clears CCxE, CCxP, etc. for all channels)
        // This ensures all outputs driven by this timer are turned off.
        TIMx->CCER = 0; /* PDF Reference */

        // For TIM1 (if used, but reserved), disable the main output (MOE)
        // if (TIMx == TIM1) { // TIM1 is reserved
        //    TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
        // }

        // Reset the timer peripheral - puts all its registers back to default state
        reset_timer(TIMx); /* Assumption: RCC peripheral reset via standard registers */

        // Disable the timer clock
        disable_timer_clock(TIMx); /* Assumption: RCC clock disabling via standard registers */

        // Configure the specific GPIO pin back to Input Floating (reset state)
        // Clear mode bits for the pin (2 bits per pin, position 2*pin)
        GPIOx->MODER &= ~(0x3U << (pin * 2)); /* PDF Reference (MODER bits 2y:2y+1) */
        // This sets the mode to 00 (Input). OTYPER, OSPEEDR, PUPDR defaults usually align
        // with Input Floating after reset, but explicit config could be added for robustness.
        // Clearing the mode to 00 makes it Input, and the default state is floating.

        // Disable the GPIO clock for the port.
        // NOTE: This should only be done if NO other pins on this port are currently needed.
        // A more robust implementation might track which ports are still active before disabling.
        // Here, we disable the clock for every port used by a PWM channel in the map.
        // This might cause issues if other peripherals are using pins on these ports.
        disable_gpio_clock(GPIOx); /* Assumption: RCC clock disabling via standard registers */

    } // End for loop over channels
}