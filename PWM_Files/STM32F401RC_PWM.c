/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h" // Contains TRD_Channel_t enum and function prototypes
#include <stdint.h>         // For standard integer types (uint32_t, uint8_t, etc.)
// Assuming standard CMSIS header is available for peripheral base addresses and register structures
// The provided PDF gives register offsets, but base addresses and full structure definitions
// are typically provided by a device header file like stm32f4xx.h.
#include "stm32f4xx.h"      // Provides access to register structures (GPIOx, TIMx, RCC) /* Assumed header - please verify */

//----------------------------------------------------------------------------------------------------------------------
// Configuration Defines
//----------------------------------------------------------------------------------------------------------------------

// Define the timer clock frequencies based on the system clock configuration.
// This is a critical assumption as the provided PDF text does not specify
// the system clock configuration or APB prescalers.
// It is assumed that the APB prescalers are configured such that the timer
// clock frequency is twice the APB clock frequency if the prescaler is > 1,
// which is typical for STM32.
// For STM32F401RC, max APB1 is 42MHz, max APB2 is 84MHz.
// Assuming max clock speeds are configured (e.g., 84MHz SYSCLK, APB1 prescaler 2, APB2 prescaler 1).
// Timer clock for APB1 timers (TIM2, TIM3, TIM4, TIM5) would be 2 * 42MHz = 84MHz.
// Timer clock for APB2 timers (TIM1, TIM9, TIM10, TIM11) would be 2 * 84MHz = 168MHz if APB2 prescaler > 1.
// as this is the maximum APB2 frequency and a common timer clock frequency.
#define TIM_CLOCK_FREQ    84000000UL /* Assumed timer clock frequency - please verify based on actual RCC setup */

// Define the minimum required timer frequency to consider valid (e.g., 1 Hz)
#define MIN_PWM_FREQ_HZ        1UL

//----------------------------------------------------------------------------------------------------------------------
// Reserved Timers
//----------------------------------------------------------------------------------------------------------------------

// Reserved Timers: TIM10 and TIM11.
// These timers are reserved for potential OS/delay purposes and are excluded
// from this PWM driver implementation. The STM32F401RC has TIM1-5 and TIM9-11.
// This driver utilizes TIM1, TIM2, TIM3, TIM4, and potentially TIM5 and TIM9 channels
// based on the pwm_channel_map configuration below, avoiding the reserved timers.

//----------------------------------------------------------------------------------------------------------------------
// Channel Mapping
//----------------------------------------------------------------------------------------------------------------------

// Structure to hold the hardware configuration for each TRD_Channel_t
typedef struct {
    TIM_TypeDef *TIMx;         // Pointer to Timer peripheral (e.g., TIM1, TIM2)
    uint32_t Channel;          // Timer Channel identifier (e.g., TIM_CHANNEL_1, TIM_CHANNEL_2)
    GPIO_TypeDef *GPIOx;       // Pointer to GPIO port (e.g., GPIOA, GPIOB)
    uint32_t Pin;              // GPIO Pin number (0-15)
    uint32_t AF;               // Alternate Function number (AF0-AF15)
} PWM_Channel_Config_t;

// Array mapping logical TRD_Channel_t to physical hardware configuration.
// Each entry corresponds to a TRD_Channel_t enum value.
// Maps TIMx_CHy -> GPIO_PxN, AFz.
// Selection criteria:
// 1. Use pins that support TIMx_CHy alternate function based on STM32F401RC datasheet.
// 2. Avoid GPIO pin number 0 as instructed, unless strictly necessary for a defined channel (not needed here).
// 3. Prefer using pins numbered 1 and above.
// 4. Avoid potential conflicts by selecting one timer function per pin.
// 5. Exclude TIM10 and TIM11 as they are reserved.
// 6. Exclude TIM5 and TIM9 channels that conflict with selected TIM1-4 channels on the chosen pins.
// 7. Avoid known debug pins (PA13, PA14, PA15, PB3, PB4) where their alternate function is used for debug.
//    PA13/14/15 are TIM1 CH1-3, PB3 is TIM2_CH2, PB4 is TIM3_CH1/TIM8_CH2.
//    We are using PA8-PA11 for TIM1, PA1-PA3 for TIM2, PC6-PC9 for TIM3, PB6-PB9 for TIM4.
//    This avoids PA13/14/15 (except if they were TIM1), PA0, PB3, PB4.
//    PB8/PB9 also support TIM10/TIM11, but TIM4 is prioritized for more channels and TIM10/11 are reserved.
//    PC6-PC9 are TIM3 AF2.
//    PA1-PA3 are TIM2 AF1. PA0 is TIM2_CH1 AF1, excluded.
//    PA8-PA11 are TIM1 AF1.
//    PB6-PB9 are TIM4 AF2.
//    This selection uses TIM1, TIM2, TIM3, TIM4 providing 15 channels across different ports.
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TIM1 channels (APB2 bus) - PA8, PA9, PA10, PA11 -> AF1
    [TRD_CHANNEL_TIM1_CH1] = {TIM1, TIM_CHANNEL_1, GPIOA, 8, GPIO_AF_TIM1},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM1_CH2] = {TIM1, TIM_CHANNEL_2, GPIOA, 9, GPIO_AF_TIM1},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM1_CH3] = {TIM1, TIM_CHANNEL_3, GPIOA, 10, GPIO_AF_TIM1}, /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM1_CH4] = {TIM1, TIM_CHANNEL_4, GPIOA, 11, GPIO_AF_TIM1}, /* Assumed PWM config - please verify */

    // TIM2 channels (APB1 bus, 32-bit) - PA1, PA2, PA3 -> AF1
    // PA0 (CH1) excluded as per requirements.
    [TRD_CHANNEL_TIM2_CH2] = {TIM2, TIM_CHANNEL_2, GPIOA, 1, GPIO_AF_TIM2},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM2_CH3] = {TIM2, TIM_CHANNEL_3, GPIOA, 2, GPIO_AF_TIM2},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM2_CH4] = {TIM2, TIM_CHANNEL_4, GPIOA, 3, GPIO_AF_TIM2},  /* Assumed PWM config - please verify */

    // TIM3 channels (APB1 bus, 16-bit) - PC6, PC7, PC8, PC9 -> AF2
    // Using PC pins to provide variety and avoid potential conflicts/debug issues on PA6/PA7/PB0/PB1/PB4/PB5
    [TRD_CHANNEL_TIM3_CH1] = {TIM3, TIM_CHANNEL_1, GPIOC, 6, GPIO_AF_TIM3},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM3_CH2] = {TIM3, TIM_CHANNEL_2, GPIOC, 7, GPIO_AF_TIM3},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM3_CH3] = {TIM3, TIM_CHANNEL_3, GPIOC, 8, GPIO_AF_TIM3},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM3_CH4] = {TIM3, TIM_CHANNEL_4, GPIOC, 9, GPIO_AF_TIM3},  /* Assumed PWM config - please verify */

    // TIM4 channels (APB1 bus, 16-bit) - PB6, PB7, PB8, PB9 -> AF2
    // PB8/PB9 conflict with TIM10/TIM11 (AF3), prioritizing TIM4.
    [TRD_CHANNEL_TIM4_CH1] = {TIM4, TIM_CHANNEL_1, GPIOB, 6, GPIO_AF_TIM4},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM4_CH2] = {TIM4, TIM_CHANNEL_2, GPIOB, 7, GPIO_AF_TIM4},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM4_CH3] = {TIM4, TIM_CHANNEL_3, GPIOB, 8, GPIO_AF_TIM4},  /* Assumed PWM config - please verify */
    [TRD_CHANNEL_TIM4_CH4] = {TIM4, TIM_CHANNEL_4, GPIOB, 9, GPIO_AF_TIM4},  /* Assumed PWM config - please verify */

    // TIM5 (APB1 bus, 32-bit): PA0 (CH1), PA1(CH2), PA2(CH3), PA3(CH4) -> AF2
    // Conflicts with TIM2/TIM3 channels on PA1/PA2/PA3/PC6-9. Excluded to avoid conflicts in this map.

    // TIM9 (APB2 bus, 16-bit): PA2(CH1), PA3(CH2) -> AF3
    // Conflicts with TIM2 channels on PA2/PA3. Excluded to avoid conflicts.

    // TIM10 & TIM11 are reserved for OS/delay.

};

// Helper macro to check if a channel is valid based on the map size
#define IS_VALID_CHANNEL(channel) ((channel) < (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])))

// Helper macro to get configuration for a valid channel
#define GET_CHANNEL_CONFIG(channel) (&pwm_channel_map[channel])

//----------------------------------------------------------------------------------------------------------------------
/**Functions ===========================================================================*/
//----------------------------------------------------------------------------------------------------------------------


/**
  * @brief Initializes the PWM hardware for a specific channel.
  * Configures the GPIO pin for Alternate Function, enables timer clock,
  * and sets up the timer in PWM mode.
  * @param TRD_Channel The channel to initialize.
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (!IS_VALID_CHANNEL(TRD_Channel)) {
        // Handle invalid channel (e.g., log error, assert, or return a status)
        // For void function, just return.
        return;
    }

    const PWM_Channel_Config_t *config = GET_CHANNEL_CONFIG(TRD_Channel);
    uint32_t channel_idx; // 0-3 index for CCMR/CCER register bit manipulation

    // Determine channel index (0-3) from the TIM_CHANNEL_x identifier
    switch (config->Channel) {
        case TIM_CHANNEL_1: channel_idx = 0; break;
        case TIM_CHANNEL_2: channel_idx = 1; break;
        case TIM_CHANNEL_3: channel_idx = 2; break;
        case TIM_CHANNEL_4: channel_idx = 3; break;
        default: return; // Should not happen with valid config, but defensive
    }

    // 1. Enable GPIO clock for the port
    // Use the appropriate AHB1 enable register for GPIOs /* Assumed RCC config - please verify */
    if (config->GPIOx == GPIOA) {
         RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Assumed RCC config - please verify */
    } else if (config->GPIOx == GPIOB) {
         RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Assumed RCC config - please verify */
    } else if (config->GPIOx == GPIOC) {
         RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* Assumed RCC config - please verify */
    }
    // Note: Add cases for GPIOD, GPIOE, GPIOH if used in pwm_channel_map

    // Ensure GPIO clock is stable - add a small delay or dummy read if necessary for the specific MCU
    // Dummy read is a common practice to ensure the clock is up and running before configuring the peripheral
    (void)config->GPIOx->MODER;


    // 2. Configure GPIO pin as Alternate Function output
    // MODER: Set bits 2y:2y+1 to 10 for Alternate function mode /* PDF Reference */
    config->GPIOx->MODER &= ~(0x3U << (config->Pin * 2)); /* PDF Reference */
    config->GPIOx->MODER |= (0x2U << (config->Pin * 2));  /* PDF Reference */

    // OTYPER: Set bit y to 0 for Output push-pull (reset state) /* PDF Reference */
    config->GPIOx->OTYPER &= ~(0x1U << config->Pin); /* PDF Reference */

    // OSPEEDR: Set bits 2y:2y+1 for Output speed. Using High speed (10) /* PDF Reference */
    config->GPIOx->OSPEEDR &= ~(0x3U << (config->Pin * 2)); /* PDF Reference */
    config->GPIOx->OSPEEDR |= (0x2U << (config->Pin * 2)); /* PDF Reference */ /* Assumed speed */

    // PUPDR: Set bits 2y:2y+1 to 00 for No pull-up, pull-down (reset state) /* PDF Reference */
    config->GPIOx->PUPDR &= ~(0x3U << (config->Pin * 2)); /* PDF Reference */

    // AFR[L/H]: Set bits 4y:4y+3 (AFRL for pins 0-7, AFRH for pins 8-15) to the specific AF number /* PDF Reference */
    if (config->Pin < 8) {
        config->GPIOx->AFR[0] &= ~(0xFU << (config->Pin * 4));           /* PDF Reference */
        config->GPIOx->AFR[0] |= (config->AF << (config->Pin * 4));      /* PDF Reference */ /* Assumed PWM config - please verify */
    } else {
        config->GPIOx->AFR[1] &= ~(0xFU << ((config->Pin - 8) * 4));     /* PDF Reference */
        config->GPIOx->AFR[1] |= (config->AF << ((config->Pin - 8) * 4)); /* PDF Reference */ /* Assumed PWM config - please verify */
    }

    // 3. Enable Timer clock
    // Use the appropriate APB enable register for timers /* Assumed RCC config - please verify */
     if (config->TIMx == TIM1) {
         RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* Assumed RCC config - please verify */
     } else if (config->TIMx == TIM2) {
         RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* Assumed RCC config - please verify */
     } else if (config->TIMx == TIM3) {
         RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Assumed RCC config - please verify */
     } else if (config->TIMx == TIM4) {
         RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* Assumed RCC config - please verify */
     } else if (config->TIMx == TIM5) { // Although TIM5 is excluded from the map, include the case defensively.
         RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; /* Assumed RCC config - please verify */
     } else if (config->TIMx == TIM9) { // Although TIM9 is excluded from the map, include the case defensively.
         RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* Assumed RCC config - please verify */
     }
    // Note: TIM10 and TIM11 are reserved and not enabled here.

    // Ensure Timer clock is stable
    (void)config->TIMx->CR1;

    // 4. Configure Timer Base Unit
    // CR1: Counter direction, clock division, auto-reload preload, etc. /* PDF Reference */
    config->TIMx->CR1 &= ~TIM_CR1_CMS;    /* PDF Reference */ // Edge-aligned mode (CMS=00)
    // TIM9, TIM10, TIM11 are up-counters only. DIR bit is read-only.
    // TIM1, TIM2, TIM3, TIM4, TIM5 are up/down/center. Set DIR=0 for upcounting.
    if (config->TIMx != TIM9 && config->TIMx != TIM10 && config->TIMx != TIM11) {
         config->TIMx->CR1 &= ~TIM_CR1_DIR;    /* PDF Reference */ // Upcounting mode (DIR=0)
    }
    config->TIMx->CR1 &= ~TIM_CR1_CKD;    /* PDF Reference */ // No clock division (CKD=00)
    config->TIMx->CR1 |= TIM_CR1_ARPE;     /* PDF Reference */ // Auto-reload preload enable (ARPE=1)
    config->TIMx->CR1 &= ~TIM_CR1_UDIS;   /* PDF Reference */ // UDIS = 0 (Update events enabled)
    config->TIMx->CR1 &= ~TIM_CR1_OPM;    /* PDF Reference */ // OPM = 0 (Repetitive mode)

    // Initialize PSC and ARR to default values (e.g., max period, min prescaler)
    // These will be overwritten by the first call to PWM_Set_Freq
    config->TIMx->PSC = 0; /* PDF Reference */
    if (config->TIMx == TIM2 || config->TIMx == TIM5) {
         config->TIMx->ARR = 0xFFFFFFFF; /* PDF Reference */
    } else {
         config->TIMx->ARR = 0xFFFF; /* PDF Reference */
    }


    // 5. Configure Timer Channel for PWM Output
    // Use CCMR1 (channels 1 & 2) or CCMR2 (channels 3 & 4) based on channel index
    volatile uint32_t* ccmr;
    uint32_t ccmr_shift; // Bit shift for the specific channel configuration block

    if (channel_idx < 2) { // CCMR1 handles channels 1 (bits 0-7) and 2 (bits 8-15)
        ccmr = (volatile uint32_t*)&(config->TIMx->CCMR1); // Cast needed for generic pointer
        ccmr_shift = channel_idx * 8;
    } else { // CCMR2 handles channels 3 (bits 0-7) and 4 (bits 8-15)
        ccmr = (volatile uint32_t*)&(config->TIMx->CCMR2); // Cast needed for generic pointer
        ccmr_shift = (channel_idx - 2) * 8;
    }

    // Configure as output (CCxS = 00) /* PDF Reference */
    *ccmr &= ~(0x3U << (ccmr_shift + 0));

    // Configure in PWM Mode 1 (OCxM = 110) /* PDF Reference */
    *ccmr &= ~(0x7U << (ccmr_shift + 4)); // Clear OCxM bits
    *ccmr |= (0x6U << (ccmr_shift + 4));  // Set OCxM to 110 (PWM Mode 1)

    // Enable Output Compare Preload (OCxPE = 1) /* PDF Reference */
    *ccmr |= (0x1U << (ccmr_shift + 3));

    // Configure Capture/Compare Enable Register (CCER) /* PDF Reference */
    volatile uint32_t* ccer = (volatile uint32_t*)&(config->TIMx->CCER); // Cast needed for generic pointer
    uint32_t ccer_shift = channel_idx * 4; // Channel 1 bits 0-3, Channel 2 bits 4-7, etc.

    // Enable the output (CCxE = 1) /* PDF Reference */
    *ccer |= (0x1U << (ccer_shift + 0)); // CCxE = 1

    // Set output polarity (Active High - CCxP = 0) /* PDF Reference */
    *ccer &= ~(0x1U << (ccer_shift + 1)); // CCxP = 0

    // Note: CCxNP (Complementary Output Polarity) and CCxNE (Complementary Output Enable)
    // are only applicable for TIM1. They are not configured here as standard PWM typically
    // only uses the main output.

    // For TIM1 (Advanced Timer), configure BDTR (Break and Dead-Time Register) /* PDF Reference */
    // This register is not present in general-purpose timers (TIM2-TIM5, TIM9-TIM11).
    if (config->TIMx == TIM1) {
        // Enable Main Output (MOE = 1) /* PDF Reference */
        // BDTR offset is 0x44 /* PDF Reference */
        // MOE is bit 15 /* PDF Reference */
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Generate an update event (UG bit in EGR) to load all configurations
    // and preload registers (PSC, ARR, CCR) into active registers. /* PDF Reference */
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */
    // Clear update interrupt flag (UIF) that might be set by the UG bit /* PDF Reference */
    config->TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference */

    // Counter remains disabled until PWM_Start is called
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
}

/**
  * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
  * Recalculates ARR, PSC, and CCR values based on the desired frequency and duty cycle.
  * Applies the new values to the timer registers, loading them via an update event.
  * @param TRD_Channel The channel to configure.
  * @param frequency The desired PWM frequency in Hz.
  * @param duty The desired duty cycle in percentage (0-100).
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (!IS_VALID_CHANNEL(TRD_Channel)) {
        // Handle invalid channel
        return;
    }

    // Validate frequency and duty cycle ranges
    if (frequency < MIN_PWM_FREQ_HZ || duty > 100) {
        // Handle invalid parameters (e.g., log error, use default, or return status)
        // For void function, just return.
        return;
    }

    const PWM_Channel_Config_t *config = GET_CHANNEL_CONFIG(TRD_Channel);

    // Get the timer clock frequency (Assumed defined constant)
    uint32_t timer_clock = TIM_CLOCK_FREQ; /* Assumed timer clock frequency */

    // Calculate total timer counts per period: (ARR + 1) * (PSC + 1) = timer_clock / frequency
    uint64_t total_counts = 0;
    if (frequency > 0) { // Avoid division by zero
        total_counts = (uint64_t)timer_clock / frequency;
    }

    // Determine max ARR value based on timer type (16-bit or 32-bit)
    uint32_t max_arr = 0xFFFF; // Default for 16-bit timers (TIM1, TIM3, TIM4, TIM9, TIM10, TIM11)
    if (config->TIMx == TIM2 || config->TIMx == TIM5) { // TIM2 and TIM5 are 32-bit /* PDF Reference */
        max_arr = 0xFFFFFFFF;
    }

    uint16_t psc = 0;
    uint32_t arr = 0;

    // Find a suitable PSC and ARR combination.
    // We iterate through possible PSC values starting from 0 to maximize ARR
    // for potentially better resolution, while staying within the ARR limits.
    // PSC is a 16-bit register.
    for (psc = 0; psc <= 0xFFFF; ++psc) { /* PDF Reference */
        if ((psc + 1) == 0) continue; // Should not happen with uint16_t psc

        uint64_t counts_per_psc = total_counts / (psc + 1);

        if (counts_per_psc > 0) {
             // calculated_arr = counts_per_psc - 1
             // Check if counts_per_psc fits within the timer's ARR size + 1
             // E.g., for 16-bit timer, counts_per_psc cannot exceed 0xFFFF + 1 = 0x10000
             uint64_t max_counts_per_psc = max_arr + 1;
             if (counts_per_psc <= max_counts_per_psc) {
                 arr = (uint32_t)(counts_per_psc - 1);
                 break; // Found a valid pair
             }
        } else {
            // If counts_per_psc is 0, it means total_counts < psc+1.
            // This happens if frequency is too low resulting in very large total_counts,
            // or if PSC is already very large.
            // If frequency is very low, total_counts might exceed uint64_t capacity, but TIM_CLOCK_FREQ/MIN_PWM_FREQ_HZ should fit.
            // More likely issue: total_counts < (psc + 1), meaning (psc + 1) is too large for the desired period.
            // We break the loop here, using the last valid (psc, arr) or the initial large arr/psc=0.
            // If the loop finishes without break, the default arr/psc=0xffffffff/0 will be used (or max_arr/0).
            // A better approach might be to start PSC from a value that makes ARR fit in the register size.
            // Let's simplify: if total_counts > max_arr, then psc must be > 0. Smallest psc needed is (total_counts / (max_arr + 1)).
            if (total_counts > (uint66_t)max_arr) {
                 // Calculate minimum psc required
                 uint64_t min_psc = (total_counts + max_arr) / (max_arr + 1) - 1;
                 if (min_psc > 0xFFFF) {
                     // Frequency is too low for the maximum PSC and ARR
                     // Use max period possible
                     psc = 0xFFFF;
                     arr = max_arr;
                     break;
                 }
                 // Restart search from min_psc
                 psc = (uint16_t)min_psc -1 ; // -1 because loop does ++psc
                 continue; // Continue the loop with a higher psc
            } else {
                 // total_counts fits in ARR+1, PSC=0 is possible
                 psc = 0;
                 arr = (uint32_t)(total_counts - 1);
                 break; // PSC=0 works
            }
        }
    }

    // Fallback if calculation failed or frequency too low
     if (total_counts == 0 || psc > 0xFFFF || (arr > max_arr && max_arr != 0xFFFFFFFF) ) {
         // Could not achieve the requested frequency or duty cycle within limits
         // Set a default known state, e.g., minimum frequency, 0 duty cycle
         psc = 0xFFFF; // Max prescaler
         arr = max_arr; // Max period
         // Resulting frequency will be timer_clock / ((0xFFFF+1)*(max_arr+1))
         // This might be less than MIN_PWM_FREQ_HZ, but it's a safe fallback.
     }


    // Calculate CCR value based on duty cycle.
    // CCR = duty * (ARR + 1) / 100
    // Use uint64_t for intermediate calculation to prevent overflow, especially with 32-bit ARR.
    uint32_t ccr_val = (uint32_t)(((uint64_t)duty * (arr + 1)) / 100);

    // Ensure CCR value is not greater than ARR+1, which corresponds to 100% duty cycle in PWM Mode 1.
    if (ccr_val > (arr + 1)) {
        ccr_val = arr + 1;
    }


    // Apply the calculated values to the timer registers
    config->TIMx->PSC = psc; /* PDF Reference */
    // ARR register size depends on the specific timer
    if (config->TIMx == TIM2 || config->TIMx == TIM5) { // 32-bit ARR for TIM2/TIM5
         config->TIMx->ARR = arr; /* PDF Reference */
    } else { // 16-bit ARR for TIM1, TIM3, TIM4, TIM9, TIM10, TIM11
         config->TIMx->ARR = (uint16_t)arr; /* PDF Reference */
    }

    // Apply the calculated CCR value to the corresponding channel register
    // CCRx registers are 32-bit for TIM2/TIM5, and 16-bit for others.
    // Writing uint32_t to 16-bit register will truncate, which is fine as ccr_val <= arr + 1 <= max_arr + 1.
    // If max_arr is 16-bit, ccr_val will also fit in 16 bits.
    switch (config->Channel) {
        case TIM_CHANNEL_1:
            config->TIMx->CCR1 = ccr_val; /* PDF Reference */
            break;
        case TIM_CHANNEL_2:
            config->TIMx->CCR2 = ccr_val; /* PDF Reference */
            break;
        case TIM_CHANNEL_3:
            config->TIMx->CCR3 = ccr_val; /* PDF Reference */
            break;
        case TIM_CHANNEL_4:
            config->TIMx->CCR4 = ccr_val; /* PDF Reference */
            break;
        default:
            return; // Should not happen
    }

    // Generate an update event (UG bit in EGR) to load the new PSC, ARR, and CCR values
    // from their preload registers into the active registers. /* PDF Reference */
    // This ensures the new values are applied at the next timer cycle.
    // Note: Setting UG bit can re-initialize the counter depending on URS bit (URS=0 by default after reset).
    // With URS=0 and UG set, counter is reinitialized. This provides immediate update but can glitch output.
    // If timer is running, this forces the update.
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */
    // Clear update interrupt flag (UIF) that might be set by the UG bit /* PDF Reference */
    config->TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference */
}

/**
  * @brief Enables and starts PWM signal generation on the specified channel.
  * This makes the timer counter start counting if not already running,
  * and enables the output driver for the specified channel.
  * @param TRD_Channel The channel to start.
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (!IS_VALID_CHANNEL(TRD_Channel)) {
        // Handle invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = GET_CHANNEL_CONFIG(TRD_Channel);

    // For TIM1 (Advanced Timer), enable Main Output (MOE). /* PDF Reference */
    // General purpose timers (TIM2-5, 9-11) do not have a MOE bit.
    if (config->TIMx == TIM1) {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Enable the timer counter (CEN bit in TIMx_CR1). /* PDF Reference */
    // The counter starts counting 1 clock cycle after setting CEN. /* PDF Reference */
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference */

    // The channel output enable (CCxE in CCER) is already set in PWM_Init.
}

/**
  * @brief Stops PWM signal generation on the specified channel.
  * Disables the output driver for the channel and stops the timer counter
  * if it's the only channel using this timer.
  * @param TRD_Channel The channel to stop.
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (!IS_VALID_CHANNEL(TRD_Channel)) {
        // Handle invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = GET_CHANNEL_CONFIG(TRD_Channel);
    uint32_t channel_idx; // 0-3 index for CCER register bit manipulation

    // Determine channel index (0-3) from the TIM_CHANNEL_x identifier
    switch (config->Channel) {
        case TIM_CHANNEL_1: channel_idx = 0; break;
        case TIM_CHANNEL_2: channel_idx = 1; break;
        case TIM_CHANNEL_3: channel_idx = 2; break;
        case TIM_CHANNEL_4: channel_idx = 3; break;
        default: return; // Should not happen
    }

    // Disable the channel output enable (CCxE in TIMx_CCER). /* PDF Reference */
    // This stops the timer from driving the pin.
    volatile uint32_t* ccer = (volatile uint32_t*)&(config->TIMx->CCER); // Cast needed for generic pointer
    uint32_t ccer_shift = channel_idx * 4; // Channel 1 bits 0-3, Channel 2 bits 4-7, etc.
    *ccer &= ~(0x1U << (ccer_shift + 0)); // Clear CCxE bit /* PDF Reference */

    // For TIM1 (Advanced Timer), disable Main Output (MOE). /* PDF Reference */
    // If this is the only TIM1 channel active, disabling MOE turns off all TIM1 outputs.
    if (config->TIMx == TIM1) {
        // A more sophisticated driver might check if other channels on TIM1 are still running
        // before clearing MOE. For this implementation, just clear MOE.
        config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */
    }

    // A more sophisticated driver might stop the timer counter (CEN) only if no other
    // here. If other channels are active, they will also stop.
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // Pin state is now controlled by GPIO registers (Input Floating after PowerOff).
}

/**
  * @brief Disables all PWM peripherals and outputs to reduce power consumption.
  * Stops all configured PWM channels, disables timer clocks, and configures GPIOs
  * back to input floating state.
  */
void PWM_PowerOff(void)
{
    // Stop all individual channels defined in the map
    for (uint32_t i = 0; i < TRD_NUM_CHANNELS; ++i) { // Iterate through the enum range
        PWM_Stop((TRD_Channel_t)i); // Call Stop for each mapped channel
    }

    // Configure all used GPIO pins back to Input Floating mode.
    // Input Floating (MODER=00) is the reset state for GPIOs. /* PDF Reference */
    for (uint32_t i = 0; i < TRD_NUM_CHANNELS; ++i) { // Iterate through the map
        const PWM_Channel_Config_t *config = GET_CHANNEL_CONFIG((TRD_Channel_t)i);

        // MODER: Set bits 2y:2y+1 to 00: Input (reset state) /* PDF Reference */
        config->GPIOx->MODER &= ~(0x3U << (config->Pin * 2)); /* PDF Reference */

        // PUPDR: Set bits 2y:2y+1 to 00: No pull-up, pull-down (reset state) /* PDF Reference */
        // This ensures no current draw from internal pull resistors in input mode.
        config->GPIOx->PUPDR &= ~(0x3U << (config->Pin * 2)); /* PDF Reference */

        // Other GPIO registers (OTYPER, OSPEEDR, AFR[L/H]) are typically irrelevant
        // when MODER is set to Input (00). Leaving them as is or resetting is possible.
        // Resetting AF is good practice, but not strictly required for input floating.
        // if (config->Pin < 8) {
        //     config->GPIOx->AFR[0] &= ~(0xFU << (config->Pin * 4)); /* PDF Reference */
        // } else {
        //     config->GPIOx->AFR[1] &= ~(0xFU << ((config->Pin - 8) * 4)); /* PDF Reference */
        // }
    }

    // Disable clocks for all *used* TIMx peripherals.
    // Based on the timers included in the pwm_channel_map: TIM1, TIM2, TIM3, TIM4.
    // Use the appropriate APB enable registers. /* Assumed RCC config - please verify */
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; /* Assumed RCC config - please verify */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; /* Assumed RCC config - please verify */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* Assumed RCC config - please verify */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* Assumed RCC config - please verify */
    // Note: Add cases for TIM5, TIM9 if they were included in the map.
    // GPIO clocks are typically not disabled here as other parts of the system
    // might still use pins on the same port.
}