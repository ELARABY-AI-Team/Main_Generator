/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : STM32F401RC PWM Driver Implementation
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

// Assuming tlong and tbyte are defined elsewhere, e.g., in stdint.h or a custom types header
// For standalone compilation and demonstration purposes, define them here if not provided by the included header:
#ifndef TLONG_TBYTE_DEFINED
#define TLONG_TBYTE_DEFINED
typedef unsigned long tlong;
typedef unsigned char tbyte;
#endif // TLONG_TBYTE_DEFINED

#include "STM32F401RC_PWM.h" // Include the user's header file.
                              // This header is assumed to define:
                              // 1. The TRD_Channel_t enum with specific channel names.
                              // 2. The TRD_CHANNEL_COUNT define, representing the total number of configured channels.
                              // The order of enumerators in TRD_Channel_t is assumed to match the index
                              // in the PWM_CHANNELS array defined below.

// --- Peripheral Base Addresses ---
// Assumed Peripheral Base Addresses for GPIO and TIM peripherals on STM32F401RC.
// These must be verified against the device datasheet or standard header files (e.g., stm32f4xx.h).
#define GPIOA_BASE        (0x40020000U) /* Assumed GPIO Base Address - please verify */
#define GPIOB_BASE        (0x40020400U) /* Assumed GPIO Base Address - please verify */
#define GPIOC_BASE        (0x40020800U) /* Assumed GPIO Base Address - please verify */
// Add other GPIO ports (D, E, H) if needed for specific channels from the datasheet

#define TIM1_BASE         (0x40010000U) /* Assumed TIM Base Address - please verify */ // APB2
// #define TIM2_BASE      (0x40000000U) // Reserved (APB1) - Not used in this implementation
// #define TIM3_BASE      (0x40000400U) // Reserved (APB1) - Not used in this implementation
#define TIM4_BASE         (0x40000800U) /* Assumed TIM Base Address - please verify */ // APB1
#define TIM5_BASE         (0x40000C00U) /* Assumed TIM Base Address - please verify */ // APB1
#define TIM9_BASE         (0x40014000U) /* Assumed TIM Base Address - please verify */ // APB2
#define TIM10_BASE        (0x40014400U) /* Assumed TIM Base Address - please verify */ // APB2
#define TIM11_BASE        (0x40014800U) /* Assumed TIM Base Address - please verify */ // APB2


// --- Register Offsets ---
// GPIO Register Offsets (from PDF Section 8.4, Table 27)
#define GPIO_MODER_OFFSET (0x00U) /* PDF Reference */
#define GPIO_OTYPER_OFFSET (0x04U) /* PDF Reference */
#define GPIO_OSPEEDR_OFFSET (0x08U) /* PDF Reference */
#define GPIO_PUPDR_OFFSET (0x0CU) /* PDF Reference */
#define GPIO_AFRL_OFFSET  (0x20U) /* PDF Reference */
#define GPIO_AFRH_OFFSET  (0x24U) /* PDF Reference */

// TIM Register Offsets (from PDF Sections 12.4, 13.4, 14.4/14.5 Register Maps)
// These offsets are relative to the respective TIMx_BASE address.
// Common Offsets across TIM1, TIM2-5, TIM9-11 (where applicable)
#define TIM_CR1_OFFSET    (0x00U) /* PDF Reference */
#define TIM_DIER_OFFSET   (0x0CU) /* PDF Reference */
#define TIM_SR_OFFSET     (0x10U) /* PDF Reference */
#define TIM_EGR_OFFSET    (0x14U) /* PDF Reference */
#define TIM_CCMR1_OFFSET  (0x18U) /* PDF Reference */
#define TIM_CCMR2_OFFSET  (0x1CU) /* PDF Reference */ // Note: Only present on TIM1, TIM2-5
#define TIM_CCER_OFFSET   (0x20U) /* PDF Reference */
#define TIM_CNT_OFFSET    (0x24U) /* PDF Reference */
#define TIM_PSC_OFFSET    (0x28U) /* PDF Reference */
#define TIM_ARR_OFFSET    (0x2CU) /* PDF Reference */
#define TIM_CCR1_OFFSET   (0x34U) /* PDF Reference */
#define TIM_CCR2_OFFSET   (0x38U) /* PDF Reference */ // Note: Only present on TIM1, TIM2-5, TIM9
#define TIM_CCR3_OFFSET   (0x3CU) /* PDF Reference */ // Note: Only present on TIM1, TIM2-5
#define TIM_CCR4_OFFSET   (0x40U) /* PDF Reference */ // Note: Only present on TIM1, TIM2-5

// TIM1 Specific Offset (Advanced Timer)
#define TIM1_BDTR_OFFSET  (0x44U) /* PDF Reference */

// --- Assumed Timer Clock Frequency ---
// The actual timer clock frequency (CK_PSC, which is the input to the prescaler)
// depends on the system clock (SYSCLK) and the APB bus prescalers configured in RCC.
// For STM32F401RC, TIM1, TIM9-11 are on APB2. TIM2-5 are on APB1.
// If the APB prescaler (APB1 or APB2) is configured to divide by 2 or more, the timer
// clock frequency is typically twice the APB clock frequency. If the prescaler is 1,
// it's the same as the APB clock frequency.
// We assume a common configuration where the timer input clock is 84MHz.
// This is a critical assumption and must match the actual clock configuration in RCC.
#define TIM_CLOCK_FREQ_HZ 84000000UL /* Assumed Timer Input Clock Frequency - please verify */

// --- TRD_Channel_t Definition and Supported Channels Configuration ---
// Define a structure to hold the configuration details for each potential PWM channel.
// This structure helps in mapping a TRD_Channel_t enum value to the specific
// timer, channel, GPIO port, pin, and Alternate Function settings.
typedef struct {
    unsigned int timer_base;        // Base address of the timer peripheral
    unsigned char timer_channel_idx; // Channel index (1, 2, 3, or 4) used for register access
    unsigned int gpio_port_base;    // Base address of the GPIO port
    unsigned char gpio_pin;         // Pin number on the GPIO port (0-15)
    unsigned char gpio_af;          // Alternate Function number for the pin
    unsigned char timer_is_16bit;   // 1 if the timer counter/ARR/CCRx are 16-bit, 0 if 32-bit
    unsigned char timer_is_advanced;// 1 if TIM1 (has BDTR), 0 otherwise
} TRD_Channel_Definition_t;

// Define the array of supported PWM channels and their configurations.
// The index in this array corresponds to the enumerator value in TRD_Channel_t.
// Assumed Pin/AF mappings are based on typical STM32F401RC datasheet configurations.
// These mappings are critical and MUST be verified against the specific device datasheet.
static const TRD_Channel_Definition_t PWM_CHANNELS[] = {
    // Reserved Timers: TIM2 and TIM3 are explicitly NOT included here to fulfill the reservation requirement.
    // TIM6 and TIM7 are basic timers not typically used for complex PWM and are not covered by the provided PDF context.
    {TIM1_BASE,  1, GPIOA_BASE, 8,  1, 1, 1},  // TRD_CHANNEL_TIM1_CH1: TIM1_CH1 on PA8, AF1 /* Assumed PWM config - please verify */
    {TIM1_BASE,  4, GPIOB_BASE, 0,  1, 1, 1},  // TRD_CHANNEL_TIM1_CH4: TIM1_CH4 on PB0, AF1 /* Assumed PWM config - please verify */
    {TIM4_BASE,  1, GPIOB_BASE, 6,  2, 1, 0},  // TRD_CHANNEL_TIM4_CH1: TIM4_CH1 on PB6, AF2 /* Assumed PWM config - please verify */
    {TIM4_BASE,  2, GPIOB_BASE, 7,  2, 1, 0},  // TRD_CHANNEL_TIM4_CH2: TIM4_CH2 on PB7, AF2 /* Assumed PWM config - please verify */
    {TIM5_BASE,  1, GPIOA_BASE, 0,  2, 0, 0},  // TRD_CHANNEL_TIM5_CH1: TIM5_CH1 on PA0, AF2 /* Assumed PWM config - please verify */
    {TIM9_BASE,  1, GPIOA_BASE, 2,  3, 1, 0},  // TRD_CHANNEL_TIM9_CH1: TIM9_CH1 on PA2, AF3 /* Assumed PWM config - please verify */
    {TIM9_BASE,  2, GPIOA_BASE, 3,  3, 1, 0},  // TRD_CHANNEL_TIM9_CH2: TIM9_CH2 on PA3, AF3 /* Assumed PWM config - please verify */
    {TIM10_BASE, 1, GPIOB_BASE, 8,  3, 1, 0},  // TRD_CHANNEL_TIM10_CH1: TIM10_CH1 on PB8, AF3 /* Assumed PWM config - please verify */
    {TIM11_BASE, 1, GPIOB_BASE, 9,  3, 1, 0},  // TRD_CHANNEL_TIM11_CH1: TIM11_CH1 on PB9, AF3 /* Assumed PWM config - please verify */
    // Ensure TRD_CHANNEL_COUNT in STM32F401RC_PWM.h matches the number of entries above.
};


// --- Timer Reservation Explanation ---
/*
 * Timer Reservation:
 * As per the requirements, at least 2 timers capable of PWM generation are reserved
 * for potential use by an operating system (OS) or delay functions.
 * Based on the provided PDF documentation (RM0368 sections covering TIM1, TIM2-5, TIM9-11),
 * these timers are described as having PWM capabilities on STM32F401 devices.
 * TIM2 (32-bit General Purpose) and TIM3 (16-bit General Purpose) have been arbitrarily
 * selected for this reservation and are therefore explicitly excluded from the list
 * of usable PWM channels (PWM_CHANNELS array) in this implementation.
 * TIM6 and TIM7 are Basic timers often used for OS/delay, but their register definitions
 * and capabilities are not included in the provided PDF context, thus they cannot be
 * configured or explicitly reserved within the constraints of this implementation based
 * strictly on the PDF content.
 */


/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * Configures the timer and GPIOs for PWM output in Edge-aligned Upcounting mode.
 * Assumes GPIO and Timer peripheral clocks have already been enabled via RCC.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel index provided.
        return;
    }

    const TRD_Channel_Definition_t* channel_def = &PWM_CHANNELS[TRD_Channel];

    // --- Configure GPIO Pin for Alternate Function ---
    // Assumption: GPIO clock for the relevant port is already enabled (e.g., in SystemInit or board initialization).
    // In production code requiring bare-metal RCC control, enable clock here:
    // #define RCC_AHB1ENR_OFFSET (0x30U) // Assuming offset for AHB1 enable register
    // #define RCC_APB2ENR_OFFSET (0x44U) // Assuming offset for APB2 enable register
    // ... enable clock for channel_def->gpio_port_base ...

    volatile unsigned int* moder_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_MODER_OFFSET); /* PDF Reference */
    volatile unsigned int* otyper_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_OTYPER_OFFSET); /* PDF Reference */
    volatile unsigned int* ospeedr_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_OSPEEDR_OFFSET); /* PDF Reference */
    volatile unsigned int* pupdr_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_PUPDR_OFFSET); /* PDF Reference */
    volatile unsigned int* afrl_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_AFRL_OFFSET); /* PDF Reference */
    volatile unsigned int* afrh_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_AFRH_OFFSET); /* PDF Reference */

    unsigned char pin = channel_def->gpio_pin;
    unsigned char af = channel_def->gpio_af;

    // Set pin mode to Alternate Function (10) for the selected pin.
    *moder_reg = (*moder_reg & ~(3U << (pin * 2))) | (2U << (pin * 2)); /* PDF Reference - MODER bits (2y:2y+1) */

    // Set output type to Push-Pull (0) for the selected pin.
    *otyper_reg &= ~(1U << pin); /* PDF Reference - OTy bit */

    // Set output speed to High speed (10) for the selected pin.
    // Using High speed as a common choice for PWM output.
    *ospeedr_reg = (*ospeedr_reg & ~(3U << (pin * 2))) | (2U << (pin * 2)); /* PDF Reference - OSPEEDRy bits (2y:2y+1) */

    // Set pull-up/pull-down to No pull-up, pull-down (00) for the selected pin.
    *pupdr_reg &= ~(3U << (pin * 2)); /* PDF Reference - PUPDRy bits (2y:2y+1) */

    // Select the Alternate Function (AF) number for the pin.
    // AFRL for pins 0-7, AFRH for pins 8-15.
    if (pin < 8) {
        *afrl_reg = (*afrl_reg & ~(0xFU << (pin * 4))) | (af << (pin * 4)); /* PDF Reference - AFRLy bits (y=0..7, bits 4y:4y+3) */
    } else {
        *afrh_reg = (*afrh_reg & ~(0xFU << ((pin - 8) * 4))) | (af << ((pin - 8) * 4)); /* PDF Reference - AFRHy bits (y=8..15, bits 4(y-8):4(y-8)+3) */
    }
    /* Assumed Pin/AF mapping based on STM32F401RC datasheet - please verify */


    // --- Configure Timer for PWM ---
    // Assumption: Timer clock for the relevant peripheral is already enabled (e.g., in SystemInit or board initialization).
    // In production code requiring bare-metal RCC control, enable clock here:
    // ... enable clock for channel_def->timer_base ...

    volatile unsigned int* cr1_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CR1_OFFSET); /* PDF Reference */
    volatile unsigned int* ccer_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCER_OFFSET); /* PDF Reference */

    // Stop the timer counter (CEN = 0) to configure it safely.
    *cr1_reg &= ~(1U << 0); /* PDF Reference - CEN bit */

    // Configure Time-base Unit Control Register 1 (TIMx_CR1)
    // Set counter mode to Upcounting (DIR=0, CMS=00)
    // Set clock division to tDTS = tCK_INT (CKD=00)
    // Set Update Request Source (URS=0): Any event (overflow, UG, trigger) generates update interrupt/DMA (if enabled).
    // Set Update Disable (UDIS=0): Update event generation is enabled.
    *cr1_reg &= ~((3U << 5) | (1U << 4) | (3U << 8) | (1U << 2) | (1U << 1)); /* PDF Reference - CMS[1:0], DIR, CKD[1:0], URS, UDIS bits */
    // Enable Auto-Reload Preload (ARPE = 1): Makes ARR buffered, updated at UEV.
    *cr1_reg |= (1U << 7); /* PDF Reference - ARPE bit */


    // Configure Capture/Compare Mode Register (TIMx_CCMR1 or TIMx_CCMR2)
    unsigned char channel_idx = channel_def->timer_channel_idx;
    volatile unsigned int* ccmr_reg; // Pointer will point to either CCMR1 or CCMR2
    unsigned int ccmr_channel_pos; // Bit position within CCMR register for this channel's config block

    // Determine which CCMR register and the position of the channel's bits (CCxS, OCxM, etc.)
    if (channel_idx == 1) {
        ccmr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCMR1_OFFSET); /* PDF Reference */
        ccmr_channel_pos = 0; // Channel 1 uses bits [7:0] in CCMR1 (CC1S, OC1M, etc.)
    } else if (channel_idx == 2) {
         // TIM10/11 only have channel 1 as per PDF register map (Table 61).
         if (channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return; // Channel 2 not available on TIM10/11
        ccmr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCMR1_OFFSET); /* PDF Reference */
        ccmr_channel_pos = 8; // Channel 2 uses bits [15:8] in CCMR1 (CC2S, OC2M, etc.)
    } else if (channel_idx == 3) {
         // TIM9/10/11 do not have CCMR2 as per PDF register maps (Tables 59, 61).
         if (channel_def->timer_base == TIM9_BASE || channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return; // Channel 3 not available on TIM9/10/11
        ccmr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCMR2_OFFSET); /* PDF Reference */
        ccmr_channel_pos = 0; // Channel 3 uses bits [7:0] in CCMR2 (CC3S, OC3M, etc.)
    } else if (channel_idx == 4) {
         // TIM9/10/11 do not have CCMR2 as per PDF register maps (Tables 59, 61).
         if (channel_def->timer_base == TIM9_BASE || channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return; // Channel 4 not available on TIM9/10/11
        ccmr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCMR2_OFFSET); /* PDF Reference */
        ccmr_channel_pos = 8; // Channel 4 uses bits [15:8] in CCMR2 (CC4S, OC4M, etc.)
    } else {
        return; // Invalid channel index (should not be reached with proper TRD_Channel_t)
    }

    // Configure the channel as Output (CCxS = 00)
    // This also makes the corresponding TIMx_CCRx register writable for output compare mode.
    *ccmr_reg &= ~(3U << ccmr_channel_pos); /* PDF Reference - CCxS bits */

    // Configure Output Compare mode to PWM Mode 1 (OCxM = 110)
    // In upcounting, channel is active as long as TIMx_CNT < TIMx_CCRx else inactive.
    // Clear OCxM bits, then set them to 110 (binary)
    *ccmr_reg = (*ccmr_reg & ~(7U << (ccmr_channel_pos + 4))) | (6U << (ccmr_channel_pos + 4)); /* PDF Reference - OCxM bits (bits 4-6 or 12-14) */

    // Enable Output Compare Preload (OCxPE = 1)
    // Enables buffering of TIMx_CCRx register, updated at UEV.
    *ccmr_reg |= (1U << (ccmr_channel_pos + 3)); /* PDF Reference - OCxPE bit (bit 3 or 11) */

    // Disable Output Compare Fast Enable (OCxFE = 0, default)
    // Fast enable is not typically needed for standard PWM.
    *ccmr_reg &= ~(1U << (ccmr_channel_pos + 2)); /* PDF Reference - OCxFE bit (bit 2 or 10) */

    // Disable Output Compare Clear Enable (OCxCE = 0, default)
    // OCxCE is only available for channels 1 and 4 on TIM1, and channels 1, 2, 3, 4 on TIM2-5.
    // Not available on TIM9-11 according to PDF CCMR descriptions.
    if ((channel_def->timer_is_advanced && (channel_idx == 1 || channel_idx == 4)) ||
        (!channel_def->timer_is_advanced && channel_def->timer_base != TIM9_BASE && channel_def->timer_base != TIM10_BASE && channel_def->timer_base != TIM11_BASE)) {
         // OC1CE is bit 7 in CCMR1, OC4CE is bit 15 in CCMR2.
         unsigned int occe_bit_pos = (channel_idx == 1) ? 7 : ((channel_idx == 4) ? 15 : 0xFF); // 0xFF is invalid sentinel
         if (occe_bit_pos != 0xFF) {
             *ccmr_reg &= ~(1U << occe_bit_pos); /* PDF Reference - OCxCE bit (bit 7 or 15) */ // Clear OCxCE
         }
    }


    // Configure Capture/Compare Enable Register (TIMx_CCER)
    unsigned int ccer_channel_bit_base = (channel_idx - 1) * 4; // Bit position for the channel's CCxE, CCxP, etc. bits
    // Clear CCxE (Output Enable) and CCxP (Output Polarity) bits.
    // Defaulting to CCxE=0 (Output Disabled), CCxP=0 (Active high polarity assumed).
    *ccer_reg &= ~((1U << ccer_channel_bit_base) | (1U << (ccer_channel_bit_base + 1))); /* PDF Reference - CCxE, CCxP bits */
    // Set CCxP to 0 for Active High polarity output.
    // To use Active Low, set this bit: *ccer_reg |= (1U << (ccer_channel_bit_base + 1)); /* PDF Reference - CCxP bit */
    /* Assumed PWM config - Active High Polarity - please verify */

    // CCxNE (Complementary Output Enable) and CCxNP (Complementary Output Polarity)
    // are only available for channels 1, 2, 3 on TIM1 (Advanced timer).
    // They are not used in basic PWM setup for a single output pin.
    if (channel_def->timer_is_advanced && channel_idx <= 3) { // TIM1 channels 1, 2, 3 have complementary outputs
         *ccer_reg &= ~((1U << (ccer_channel_bit_base + 2)) | (1U << (ccer_channel_bit_base + 3))); /* PDF Reference - CCxNE, CCxNP bits */
    }


    // For advanced timers (TIM1), configure Break and Dead-time Register (BDTR)
    // BDTR controls Main Output Enable (MOE) and Break/Dead-time features.
    if (channel_def->timer_is_advanced) {
        volatile unsigned int* bdtr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM1_BDTR_OFFSET); /* PDF Reference */
        // Clear MOE, AOE, BKP, BKE, OSSR, OSSI, LOCK bits.
        // This disables the main output, break features, and unlocks the BDTR register.
        *bdtr_reg &= ~((1U << 15) | (1U << 14) | (1U << 13) | (1U << 12) | (3U << 10) | (3U << 8)); /* PDF Reference - MOE, AOE, BKP, BKE, OSSR, OSSI, LOCK bits */
         // Clear DTG bits (Dead time = 0).
         *bdtr_reg &= ~(0xFFU << 0); /* PDF Reference - DTG bits */
        // MOE bit will be set in the PWM_Start function to enable outputs.
    }

    // Generate an update event (UG bit in EGR)
    // This forces the loading of all buffered registers (PSC, ARR, CCRx)
    // and re-initializes the counter and prescaler (if UDIS=0 and URS=0/1).
    volatile unsigned int* egr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_EGR_OFFSET); /* PDF Reference */
    *egr_reg |= (1U << 0); /* PDF Reference - UG bit */

    // Optional: Wait for the update flag (UIF) to be set and clear it.
    // This confirms that the update has completed and the initial configuration
    // (including the initial PSC, ARR, CCRx values if set before Init) is loaded.
    // This wait is only necessary if URS=0 (default) and UDIS=0 (default).
    volatile unsigned int* sr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_SR_OFFSET); /* PDF Reference */
    // Check if UIF is set (bit 0 in SR), loop while not set
    while (!(*sr_reg & (1U << 0))); /* PDF Reference - UIF bit */
    // Clear the update flag by writing 0 to it
    *sr_reg &= ~(1U << 0); /* PDF Reference - UIF bit (cleared by writing 0) */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * Recalculates and updates timer registers (PSC, ARR, CCRx) based on the
 * assumed timer clock frequency (TIM_CLOCK_FREQ_HZ).
 * A frequency of 0 or duty > 100 will be ignored.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency The desired frequency in Hz (tlong). Must be > 0.
 * @param duty The desired duty cycle in percent (0-100) (tbyte).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    if (TRD_Channel >= TRD_CHANNEL_COUNT || frequency == 0 || duty > 100) {
        // Invalid channel index, frequency, or duty cycle.
        return;
    }

    const TRD_Channel_Definition_t* channel_def = &PWM_CHANNELS[TRD_Channel];

    volatile unsigned int* psc_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_PSC_OFFSET); /* PDF Reference */
    volatile unsigned int* arr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_ARR_OFFSET); /* PDF Reference */
    volatile unsigned int* ccr_reg; // Pointer will point to the correct CCR register /* PDF Reference */

    unsigned long max_timer_value; // Maximum value for the timer's counter/ARR/CCRx registers
    if (channel_def->timer_is_16bit) {
        max_timer_value = 0xFFFF; // 16-bit maximum value /* PDF Reference */
    } else { // 32-bit timer (TIM5 used here)
        max_timer_value = 0xFFFFFFFF; // 32-bit maximum value /* PDF Reference */
    }

    // Calculate the total number of timer ticks required per period.
    // Period = Timer_Clock_Frequency / PWM_Frequency
    // Use unsigned long long for intermediate calculation to avoid overflow,
    // especially if TIM_CLOCK_FREQ_HZ is high and frequency is low.
    unsigned long long total_period_ticks_ull = (unsigned long long)TIM_CLOCK_FREQ_HZ / frequency;

    // Check if the required period (in ticks) is achievable by the timer.
    // The maximum achievable period is (0xFFFF + 1) * (0xFFFF + 1) for a 16-bit timer with 16-bit PSC,
    // and (0xFFFFFFFF + 1) * (0xFFFF + 1) for a 32-bit timer with 16-bit PSC.
    // A simpler check: is the required period larger than the max ARR+1 even with PSC = 0?
    if (total_period_ticks_ull == 0 || total_period_ticks_ull > (unsigned long long)(max_timer_value + 1) * (0xFFFF + 1) ) {
         // Frequency is too high or too low to be generated by this timer configuration.
         // Handle error (e.g., print warning, set default, or just return).
         // Returning without changing registers keeps the previous setting.
         return;
    }

    unsigned long total_period_ticks = (unsigned long)total_period_ticks_ull;


    // Find a suitable Prescaler (PSC) and Auto-Reload Register (ARR) value.
    // We aim for the smallest PSC value that results in ARR <= max_timer_value.
    // Smaller PSC generally provides better frequency and duty cycle resolution.
    unsigned long psc = 0;
    unsigned long arr = 0;
    unsigned long max_psc = 0xFFFF; // Max value for PSC register /* PDF Reference */
    unsigned char found_valid_pair = 0;

    // Iterate through possible PSC values from 0 upwards.
    for (psc = 0; psc <= max_psc; ++psc) {
        if ((psc + 1) == 0) continue; // Should not happen with 16-bit psc

        // Calculate the required ARR value for the current PSC.
        // ARR + 1 = total_period_ticks / (PSC + 1)
        unsigned long current_arr_plus_1 = total_period_ticks / (psc + 1);

        // Check if the calculated ARR+1 is within the timer's ARR range (1 to max_timer_value + 1)
        if (current_arr_plus_1 > 0 && current_arr_plus_1 <= (max_timer_value + 1)) {
            arr = current_arr_plus_1 - 1;
            found_valid_pair = 1;
            // Prioritize smaller PSC values for better resolution. Break on the first valid find.
            break;
        }
        // If current_arr_plus_1 is 0, it means frequency * (psc+1) > TIM_CLOCK_FREQ_HZ. Try larger PSC.
        // If current_arr_plus_1 > max_timer_value + 1, it means (psc+1) is too small. Try larger PSC.
    }

    if (!found_valid_pair) {
         // Should not happen if total_period_ticks_ull was checked against max theoretical period.
         // If reached, frequency is somehow unachievable within the timer's limits.
         return;
    }

    // psc and arr are now determined and represent a valid configuration.


    // Set Prescaler and Auto-Reload Register values.
    // Note: PSC is 16-bit. ARR is 16 or 32-bit depending on the timer.
    *psc_reg = psc; /* PDF Reference - PSC register */

    if (channel_def->timer_is_16bit) {
        *((volatile unsigned short *)arr_reg) = (unsigned short)arr; /* PDF Reference - ARR register (16-bit access) */
    } else { // 32-bit timer (TIM5)
        *arr_reg = (unsigned int)arr; /* PDF Reference - ARR register (32-bit access) */
    }


    // Calculate and Set Capture/Compare Register (CCRx) value based on duty cycle.
    // In PWM Mode 1 (upcounting), the output is high when CNT < CCRx.
    // Period = ARR + 1 ticks. Pulse Width = CCRx ticks. Duty Cycle = CCRx / (ARR + 1).
    // CCRx = Duty * (ARR + 1) / 100.
    unsigned long ccr_value;
    unsigned long effective_period_ticks = arr + 1; // The actual period in ticks based on chosen ARR

    if (duty == 0) {
        ccr_value = 0; // 0% duty cycle means the output is always low (CCRx=0 results in CNT >= CCRx always being true when CNT starts from 0)
    } else if (duty == 100) {
        // 100% duty cycle means the output is always high.
        // According to PDF definition (Section 12.3.10 etc.), OCxREF is held at '1' if TIMx_CCRx >= TIMx_ARR+1.
        // Setting CCRx to ARR + 1 achieves this, provided the register can hold the value.
        // If ARR is already the maximum value (e.g., 0xFFFF for 16-bit timer), ARR+1 will overflow.
        // In that case, setting CCRx to the maximum possible value (e.g., 0xFFFF) will likely achieve 100%.
        // Following the implied logic from PDF figures showing CCRx=ARR for 100% duty when ARR=8,
        // setting CCRx = ARR might be intended, although it results in (ARR)/(ARR+1) duty.
        // Let's implement the definition: CCRx >= ARR + 1 for continuous high.
        // We'll set CCRx = ARR + 1, but cap it at the maximum register value if it exceeds it.
        unsigned long target_ccr_for_100 = effective_period_ticks; // This is ARR + 1
         if (target_ccr_for_100 > max_timer_value) {
             ccr_value = max_timer_value; // Cap if ARR+1 exceeds register limit
         } else {
             ccr_value = target_ccr_for_100;
         }
        /* Follows PDF Definition (CCRx >= ARR+1 for high), capped by register size */

    } else { // duty > 0 and < 100
        // Calculate the required CCR value based on the proportional duty cycle.
        // CCRx = (duty * (ARR + 1)) / 100
        unsigned long long calculated_ccr_ull = (unsigned long long)duty * effective_period_ticks;
        calculated_ccr_ull /= 100;
        ccr_value = (unsigned long)calculated_ccr_ull;

        // Ensure calculated CCR is within valid bounds for non-zero/100% duty: [1, ARR].
        // A calculated ccr_value of 0 here implies the duty cycle is less than what 1 tick represents (less than 100/period %).
        // In this case, set minimum pulse width to 1 tick (CCRx=1).
        if (ccr_value == 0) ccr_value = 1; // Minimum pulse width is 1 tick (corresponds to CCRx = 1 in upcounting starting from 0)
        // Ensure CCRx does not exceed ARR for duty < 100.
        if (ccr_value > arr) ccr_value = arr; // Defensive cap. Should not happen with duty < 100 if calculations are correct.
    }


    // Get the correct Capture/Compare Register (TIMx_CCRx) address based on channel index.
    // Offsets are relative to TIMx_BASE.
    switch (channel_def->timer_channel_idx) {
        case 1: ccr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCR1_OFFSET); /* PDF Reference */ break;
        case 2:
            // CCR2 not available on TIM10/11 (per PDF Table 61)
            if (channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return;
            ccr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCR2_OFFSET); /* PDF Reference */
            break;
        case 3:
            // CCR3 not available on TIM9/10/11 (per PDF Tables 59, 61)
            if (channel_def->timer_base == TIM9_BASE || channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return;
            ccr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCR3_OFFSET); /* PDF Reference */
            break;
        case 4:
            // CCR4 not available on TIM9/10/11 (per PDF Tables 59, 61)
             if (channel_def->timer_base == TIM9_BASE || channel_def->timer_base == TIM10_BASE || channel_def->timer_base == TIM11_BASE) return;
            ccr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCR4_OFFSET); /* PDF Reference */
            break;
        default: return; // Should not happen with valid TRD_Channel_t and channel availability checks
    }

    // Set Capture/Compare Register value.
    // CCR registers are 16-bit or 32-bit depending on the timer (TIM5 is 32-bit here).
    // Cast the calculated ccr_value to the appropriate width before writing to the register.
    if (channel_def->timer_is_16bit) {
         *((volatile unsigned short *)ccr_reg) = (unsigned short)ccr_value; /* PDF Reference - CCR register (16-bit access) */
    } else { // 32-bit timer (TIM5)
         *ccr_reg = (unsigned int)ccr_value; /* PDF Reference - CCR register (32-bit access) */
    }


    // Generate an update event (UG bit in EGR) to load the new PSC, ARR, and CCR values
    // from their preload registers into the active registers.
    // This ensures the changes take effect at the end of the current period (due to ARPE/OCxPE).
    // For immediate update, this UG bit is essential after writing to preload registers.
    volatile unsigned int* egr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_EGR_OFFSET); /* PDF Reference */
    *egr_reg |= (1U << 0); /* PDF Reference - UG bit */

    // Optional: Wait for the update flag (UIF) to be set and clear it.
    // This guarantees that the register update triggered by UG has completed.
    // This wait is valid if URS=0 (default) and UDIS=0 (default), as UG will set UIF.
    volatile unsigned int* sr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_SR_OFFSET); /* PDF Reference */
    // Check if UIF is set (bit 0 in SR), loop while not set
    while (!(*sr_reg & (1U << 0))); /* PDF Reference - UIF bit */
    // Clear the update flag by writing 0 to it.
    *sr_reg &= ~(1U << 0); /* PDF Reference - UIF bit (cleared by writing 0) */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * This involves enabling the specific output channel and starting the timer counter.
 * For TIM1 (Advanced timer), it also enables the main output via the BDTR register.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
     if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel index.
        return;
    }

    const TRD_Channel_Definition_t* channel_def = &PWM_CHANNELS[TRD_Channel];

    volatile unsigned int* cr1_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CR1_OFFSET); /* PDF Reference */
    volatile unsigned int* ccer_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCER_OFFSET); /* PDF Reference */

    // Enable the specific Output Channel (CCxE = 1) in the CCER register.
    // This connects the timer's internal output signal to the GPIO pin.
    unsigned int ccer_channel_bit_base = (channel_def->timer_channel_idx - 1) * 4; // Base bit position for this channel in CCER
    *ccer_reg |= (1U << ccer_channel_bit_base); /* PDF Reference - CCxE bit (bit 0, 4, 8, or 12) */

    // For advanced timers (TIM1), enable the Main Output (MOE = 1) in the BDTR register.
    // This bit globally enables outputs for all channels on TIM1.
    // General purpose timers do not have this bit.
    if (channel_def->timer_is_advanced) {
        volatile unsigned int* bdtr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM1_BDTR_OFFSET); /* PDF Reference */
        *bdtr_reg |= (1U << 15); /* PDF Reference - MOE bit */
    }

    // Start the Timer Counter (CEN = 1) in the CR1 register.
    // This begins the timer counting sequence.
    // Note: Starting the timer affects all channels configured on that specific timer instance.
    // This implementation assumes the timer is primarily used for these PWM outputs.
    *cr1_reg |= (1U << 0); /* PDF Reference - CEN bit */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * Disables the output for the specific channel by clearing its enable bit.
 * The timer counter continues running, which can be useful for quick restarts or
 * synchronizing multiple channels on the same timer.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
     if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel index.
        return;
    }

    const TRD_Channel_Definition_t* channel_def = &PWM_CHANNELS[TRD_Channel];

    volatile unsigned int* ccer_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCER_OFFSET); /* PDF Reference */

    // Disable the specific Output Channel (CCxE = 0) in the CCER register.
    unsigned int ccer_channel_bit_base = (channel_def->timer_channel_idx - 1) * 4; // Base bit position for this channel in CCER
    *ccer_reg &= ~(1U << ccer_channel_bit_base); /* PDF Reference - CCxE bit (bit 0, 4, 8, or 12) */

    // Note: For TIM1 (Advanced timer), the MOE bit in BDTR also controls outputs.
    // Clearing CCxE for a specific channel is sufficient to stop that channel's output,
    // even if MOE is still set. Clearing MOE stops *all* TIM1 outputs.
    // This implementation chooses to disable the individual channel output (CCxE).
}

/**
 * @brief Disables all configured PWM peripherals and outputs to reduce power consumption.
 * Stops all timers used by the configured PWM channels and disables their outputs.
 * Also resets the associated GPIO pins to their input floating reset state.
 * Note: This function does NOT disable the peripheral clocks (requires RCC access not covered by the PDF context).
 */
void PWM_PowerOff(void) {
    // Iterate through all defined PWM channels to disable them.
    // This approach assumes that each timer instance used by these channels
    // is primarily dedicated to these PWM outputs. If timers are shared for
    // other purposes (e.g., input capture on another channel), this needs refinement.
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_COUNT; ++i) {
        const TRD_Channel_Definition_t* channel_def = &PWM_CHANNELS[i];

        volatile unsigned int* cr1_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CR1_OFFSET); /* PDF Reference */
        volatile unsigned int* ccer_reg = (volatile unsigned int *)(channel_def->timer_base + TIM_CCER_OFFSET); /* PDF Reference */

        // Stop the Timer Counter (CEN = 0) for this timer instance.
        *cr1_reg &= ~(1U << 0); /* PDF Reference - CEN bit */

        // Disable the specific Output Channel (CCxE = 0) in the CCER register.
        unsigned int ccer_channel_bit_base = (channel_def->timer_channel_idx - 1) * 4; // Base bit position for this channel in CCER
        *ccer_reg &= ~(1U << ccer_channel_bit_base); /* PDF Reference - CCxE bit (bit 0, 4, 8, or 12) */

        // For TIM1 (Advanced), disable the Main Output (MOE = 0) in the BDTR register.
        // This bit globally disables outputs for all channels on TIM1.
        if (channel_def->timer_is_advanced) {
             volatile unsigned int* bdtr_reg = (volatile unsigned int *)(channel_def->timer_base + TIM1_BDTR_OFFSET); /* PDF Reference */
            *bdtr_reg &= ~(1U << 15); /* PDF Reference - MOE bit */
        }

        // Reset the associated GPIO pin mode to Input Floating (00).
        // This is the default state after reset and helps reduce power consumption.
        volatile unsigned int* moder_reg = (volatile unsigned int *)(channel_def->gpio_port_base + GPIO_MODER_OFFSET); /* PDF Reference */
        unsigned char pin = channel_def->gpio_pin;
        *moder_reg &= ~(3U << (pin * 2)); /* PDF Reference - MODER bits (2y:2y+1), set to 00 */

        // Note: To fully power off, the corresponding TIMxEN and GPIOxEN bits
        // in the RCC enable registers (e.g., RCC_APB1ENR, RCC_APB2ENR, RCC_AHB1ENR)
        // would need to be cleared. This level of RCC control is outside the scope
        // of the provided PDF context for timer and GPIO registers.
    }
}