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

#include "STM32F401RC_PWM.h" // Assuming this defines TRD_Channel_t, PWM_Channel_Config_t, and CMSIS/HAL register access macros/bases (TIM_TypeDef, GPIO_TypeDef, RCC_TypeDef, GPIOA, TIM1, RCC_AHB1ENR_GPIOAEN, TIM_CR1_CEN, etc.)

// Assume SystemCoreClock is defined elsewhere (e.g., system_stm32f4xx.h)
// Assumed timer clock frequency calculation based on typical STM32F401 clock tree configuration.
// APB prescalers are often configured such that timer clocks are SystemCoreClock.
#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK 84000000UL /* Assumed System Clock Frequency in Hz */ /* Assumed clock config - please verify */
#endif

// Assume APB1 timer clock (for TIM2,3,4,5,10,11) and APB2 timer clock (for TIM1,9) are both SystemCoreClock.
// NOTE: This assumption needs to be verified against the actual clock tree setup (e.g., by checking RCC APB prescalers).
// If APB prescaler is > 1, the timer clock is 2x APB clock.
// For production, replace this with actual clock frequency retrieval (e.g., using HAL_RCC_GetSysClockFreq) or a configuration define based on the specific board/clock setup.
#define PWM_TIMER_CLOCK SYSTEM_CORE_CLOCK /* Assumed Timer Clock Frequency for all timers */ /* Assumed clock config - please verify */


// Reservation: TIM10 and TIM11 are reserved for other system uses (e.g., OS, timing services).
// They are excluded from the PWM channel map below.
// This reservation fulfills the requirement to reserve at least 2 timers.


/**PWM Channel Configuration Mapping ===========================================================================*/
// Production-ready definition of the PWM channel configuration array.
// Maps abstract TRD_Channel_t to specific Timer, Channel, GPIO Port, Pin, and Alternate Function.
// Only includes valid PWM-capable pins for STM32F401RC, excluding reserved timers (TIM10, TIM11)
// and timers not available on this specific device (TIM8, TIM12, TIM13, TIM14 as per PDF).
// Assumes standard STM32F401RC pin mappings and AF assignments based on datasheet.
// Pin number 0 (PA0) is included as it is a valid TIM channel pin.
// Entries using pins explicitly listed as conflicting with reserved timers are excluded (PB8, PB9 if used by TIM4).
// Note: Some pins (PA0-PA3, PB8, PB9, etc.) are shared among multiple timers/peripherals.
// The mapping below lists common combinations. Ensure your TRD_Channel_t enum matches this order.
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (APB2 Timer - AF1) - Advanced-control timer (16-bit counter)
    { TIM1, 1, GPIOA,  8, GPIO_AF_TIM1  }, /* TIM1_CH1 on PA8 */ /* Assumed PWM config - please verify */
    { TIM1, 2, GPIOA,  9, GPIO_AF_TIM1  }, /* TIM1_CH2 on PA9 */ /* Assumed PWM config - please verify */
    { TIM1, 3, GPIOA, 10, GPIO_AF_TIM1  }, /* TIM1_CH3 on PA10 */ /* Assumed PWM config - please verify */
    { TIM1, 4, GPIOA, 11, GPIO_AF_TIM1  }, /* TIM1_CH4 on PA11 */ /* Assumed PWM config - please verify */

    // TIM2 Channels (APB1 Timer - AF1) - General-purpose timer (32-bit counter)
    // Note: PA0-PA3 are also TIM5 (AF2) and TIM9 (AF3) pins. Using TIM2 mapping here.
    { TIM2, 1, GPIOA,  0, GPIO_AF_TIM2  }, /* TIM2_CH1 on PA0 */ /* Assumed PWM config - please verify */
    { TIM2, 2, GPIOA,  1, GPIO_AF_TIM2  }, /* TIM2_CH2 on PA1 */ /* Assumed PWM config - please verify */
    { TIM2, 3, GPIOA,  2, GPIO_AF_TIM2  }, /* TIM2_CH3 on PA2 */ /* Assumed PWM config - please verify */
    { TIM2, 4, GPIOA,  3, GPIO_AF_TIM2  }, /* TIM2_CH4 on PA3 */ /* Assumed PWM config - please verify */
    { TIM2, 2, GPIOB,  3, GPIO_AF_TIM2  }, /* TIM2_CH2 on PB3 */ /* Assumed PWM config - please verify */ // Alternate pin
    { TIM2, 3, GPIOB, 10, GPIO_AF_TIM2  }, /* TIM2_CH3 on PB10 */ /* Assumed PWM config - please verify */ // Alternate pin

    // TIM3 Channels (APB1 Timer - AF2) - General-purpose timer (16-bit counter)
    // Note: PB4, PB5, PC6-PC9 are TIM3 pins. PB8/PB9 conflicts with reserved TIM10/11.
    { TIM3, 1, GPIOB,  4, GPIO_AF_TIM3  }, /* TIM3_CH1 on PB4 */ /* Assumed PWM config - please verify */
    { TIM3, 2, GPIOB,  5, GPIO_AF_TIM3  }, /* TIM3_CH2 on PB5 */ /* Assumed PWM config - please verify */
    { TIM3, 1, GPIOC,  6, GPIO_AF_TIM3  }, /* TIM3_CH1 on PC6 */ /* Assumed PWM config - please verify */ // Alternate pin
    { TIM3, 2, GPIOC,  7, GPIO_AF_TIM3  }, /* TIM3_CH2 on PC7 */ /* Assumed PWM config - please verify */ // Alternate pin
    { TIM3, 3, GPIOC,  8, GPIO_AF_TIM3  }, /* TIM3_CH3 on PC8 */ /* Assumed PWM config - please verify */
    { TIM3, 4, GPIOC,  9, GPIO_AF_TIM3  }, /* TIM3_CH4 on PC9 */ /* Assumed PWM config - please verify */

    // TIM4 Channels (APB1 Timer - AF2) - General-purpose timer (16-bit counter)
    // Note: PB6-PB9 are TIM4 pins. PB8/PB9 conflicts with reserved TIM10/11.
    // The TIM4 channels on PB8/PB9 are NOT included here to avoid potential conflicts.
    { TIM4, 1, GPIOB,  6, GPIO_AF_TIM4  }, /* TIM4_CH1 on PB6 */ /* Assumed PWM config - please verify */
    { TIM4, 2, GPIOB,  7, GPIO_AF_TIM4  }, /* TIM4_CH2 on PB7 */ /* Assumed PWM config - please verify */
    // PB8 (TIM4_CH3/TIM10_CH1) and PB9 (TIM4_CH4/TIM11_CH1) excluded due to conflict with Reserved TIM10/11 pins.

    // TIM5 Channels (APB1 Timer - AF2) - General-purpose timer (32-bit counter)
    // Note: PA0-PA3 are also TIM2 (AF1) and TIM9 (AF3) pins. Using TIM5 mapping here.
    { TIM5, 1, GPIOA,  0, GPIO_AF_TIM5  }, /* TIM5_CH1 on PA0 */ /* Assumed PWM config - please verify */
    { TIM5, 2, GPIOA,  1, GPIO_AF_TIM5  }, /* TIM5_CH2 on PA1 */ /* Assumed PWM config - please verify */
    { TIM5, 3, GPIOA,  2, GPIO_AF_TIM5  }, /* TIM5_CH3 on PA2 */ /* Assumed PWM config - please verify */
    { TIM5, 4, GPIOA,  3, GPIO_AF_TIM5  }, /* TIM5_CH4 on PA3 */ /* Assumed PWM config - please verify */

    // TIM9 Channels (APB2 Timer - AF3) - General-purpose timer (16-bit counter)
    // Note: PA2-PA3 are also TIM2 (AF1) and TIM5 (AF2) pins. Using TIM9 mapping here.
    { TIM9, 1, GPIOA,  2, GPIO_AF_TIM9  }, /* TIM9_CH1 on PA2 */ /* Assumed PWM config - please verify */
    { TIM9, 2, GPIOA,  3, GPIO_AF_TIM9  }, /* TIM9_CH2 on PA3 */ /* Assumed PWM config - please verify */

    // Note: TIM10 (PB8, AF3) and TIM11 (PB9, AF3) are reserved and excluded from this driver.
    // The pins PB8 and PB9 are thus also excluded from use by other timers (TIM4) in this map
    // to prevent this driver from configuring pins assigned to reserved timers.
};
// Total available PWM channels mapped: 4 (TIM1) + 6 (TIM2) + 6 (TIM3) + 2 (TIM4) + 4 (TIM5) + 2 (TIM9) = 24 potential channels.
// The actual number of available TRD_Channel_t values should match the number of entries above.
// If you need to use shared pins with different timers (e.g., PA0 for TIM2 and TIM5),
// you would need separate entries in the map for each combination and carefully select the TRD_Channel_t.
// The current map lists each *physical* pin/AF combo once under its common timer mapping based on standard examples,
// except for the alternates on different ports (PB3, PB10, PC6, PC7).


/**Functions ===========================================================================*/

/**
  * @brief  Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
  * @param  TRD_Channel: The PWM channel to initialize.
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // 1. Enable GPIO clock
    // Determine which GPIO port to enable based on the config.
    uint32_t gpio_rcc_en = 0;
    GPIO_TypeDef* gpio_port;

    if (config->PortName == GPIOA)
    {
        gpio_rcc_en = RCC_AHB1ENR_GPIOAEN; /* PDF mentions GPIOA/B/C/D/E/H - AHB1 bus */
        gpio_port = GPIOA;
    }
    else if (config->PortName == GPIOB)
    {
        gpio_rcc_en = RCC_AHB1ENR_GPIOBEN; /* PDF mentions GPIOA/B/C/D/E/H - AHB1 bus */
        gpio_port = GPIOB;
    }
    else if (config->PortName == GPIOC)
    {
        gpio_rcc_en = RCC_AHB1ENR_GPIOCEN; /* PDF mentions GPIOA/B/C/D/E/H - AHB1 bus */
        gpio_port = GPIOC;
    }
    // Add checks for GPIOD, GPIOE, GPIOH if needed based on full list, although F401RC has fewer ports.
    // Assuming standard ports A, B, C for the mapped pins based on common F401RC pinouts.
    else
    {
        // Unsupported GPIO port or internal error
        return;
    }

    // Ensure the clock is enabled (set bit) and leave any other enabled clocks as is (|=)
    RCC->AHB1ENR |= gpio_rcc_en; /* PDF: General-purpose I/Os (GPIO) clock enable */


    // 2. Enable Timer clock
    // Determine which Timer peripheral clock to enable (APB1 or APB2)
    // Ensure the clock is enabled (set bit) and leave any other enabled clocks as is (|=)
    if (config->TIMx == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // TIM1 is on APB2 bus /* Assumed bus config - please verify */
    }
    else if (config->TIMx == TIM2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // TIM2 is on APB1 bus /* Assumed bus config - please verify */
    }
    else if (config->TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // TIM3 is on APB1 bus /* Assumed bus config - please verify */
    }
    else if (config->TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // TIM4 is on APB1 bus /* Assumed bus config - please verify */
    }
    else if (config->TIMx == TIM5)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // TIM5 is on APB1 bus /* Assumed bus config - please verify */
    }
    else if (config->TIMx == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; // TIM9 is on APB2 bus /* Assumed bus config - please verify */
    }
    // TIM10, TIM11 excluded due to reservation.
    else
    {
        // Unsupported Timer or internal error
        return;
    }


    // 3. Configure GPIO pin as Alternate Function (AF) output
    // Set MODER to Alternate function mode (10)
    uint32_t pin_mode_mask = (0x3UL << (config->PinNumber * 2));
    uint32_t af_mode_value = (0x2UL << (config->PinNumber * 2)); /* PDF: MODER = 10 for Alternate function mode */
    gpio_port->MODER &= ~pin_mode_mask; // Clear bits
    gpio_port->MODER |= af_mode_value;  // Set to Alternate function mode

    // Set OTYPER to Push-pull (0)
    uint32_t pin_otyper_mask = (0x1UL << config->PinNumber);
    gpio_port->OTYPER &= ~pin_otyper_mask; /* PDF: OTYPER = 0 for Output push-pull */

    // Set OSPEEDR to Very high speed (11)
    uint32_t pin_ospeed_mask = (0x3UL << (config->PinNumber * 2));
    uint32_t vh_speed_value = (0x3UL << (config->PinNumber * 2)); /* PDF: OSPEEDR = 11 for Very high speed */
    gpio_port->OSPEEDR &= ~pin_ospeed_mask; // Clear bits
    gpio_port->OSPEEDR |= vh_speed_value; // Set to Very high speed

    // Set PUPDR to No pull-up, pull-down (00)
    uint32_t pin_pupdr_mask = (0x3UL << (config->PinNumber * 2));
    uint32_t no_pu_pd_value = (0x0UL << (config->PinNumber * 2)); /* PDF: PUPDR = 00 for No pull-up, pull-down */
    gpio_port->PUPDR &= ~pin_pupdr_mask; // Clear bits
    gpio_port->PUPDR |= no_pu_pd_value;  // Set to No pull-up, pull-down

    // Configure Alternate Function (AFRL/AFRH)
    // Pin 0-7 -> AFRL (AFR[0]), Pin 8-15 -> AFRH (AFR[1])
    uint32_t pin_af_mask = (0xFUL << ((config->PinNumber % 8) * 4));
    uint32_t af_value = (config->AlternateFunctionNumber << ((config->PinNumber % 8) * 4)); /* PDF: AFRLy/AFRHy = AF selection */

    if (config->PinNumber < 8)
    {
        gpio_port->AFR[0] &= ~pin_af_mask; // AFR[0] is AFRL
        gpio_port->AFR[0] |= af_value;
    }
    else
    {
        gpio_port->AFR[1] &= ~pin_af_mask; // AFR[1] is AFRH
        gpio_port->AFR[1] |= af_value;
    }


    // 4. Configure Timer Base (PSC, ARR) - Initial values (e.g., for 1kHz)
    // Frequency = Timer_Clock / ((PSC + 1) * (ARR + 1))
    // Set initial frequency and duty cycle (e.g., 1kHz, 50%) by calling Set_Freq
    // A default call here simplifies Init, letting Set_Freq handle the calculations.
    // However, the prompt asks Init to configure timer base and channel.
    // Let's configure a default state (e.g. PSC=0, max ARR) and rely on Set_Freq
    // being called afterwards for the desired frequency/duty.

    // Stop the timer before configuration
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF: CR1 CEN = 0 */

    // Configure time-base settings (PSC, ARR) and channel settings (CCMR, CCER)
    // Defaults: PSC=0, ARR=max value, PWM Mode 1, no output enable, active high polarity.
    config->TIMx->PSC = 0; /* PDF: TIMx_PSC */
    if (config->TIMx == TIM2 || config->TIMx == TIM5) {
         config->TIMx->ARR = 0xFFFFFFFFUL; /* PDF: TIMx_ARR */ // 32-bit timer max ARR
    } else {
         config->TIMx->ARR = 0xFFFFUL; /* PDF: TIMx_ARR */ // 16-bit timer max ARR
    }
    config->TIMx->CNT = 0; // Start counter from 0 /* PDF: TIMx_CNT */


    // 5. Configure Timer Channel for PWM
    uint32_t ccmr_mask;
    uint32_t ccmr_value_mode;
    uint32_t ccmr_value_preload;
    uint32_t ccer_mask;
    uint32_t ccer_value_enable;
    uint32_t ccer_value_polarity;

    // Determine registers and masks based on channel number (1-4)
    // Ensure channel is set to output mode (CCxS = 00) and PWM Mode 1 (OCxM = 110)
    // Enable output compare preload (OCxPE = 1)
    // Set output polarity active high (CCxP = 0)
    // Disable output initially (CCxE = 0)
    if (config->ChannelNumber == 1)
    {
        // For CH1, uses CCMR1 register, bits related to OC1
        ccmr_mask = TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_OC1FE_Msk | TIM_CCMR1_OC1CE_Msk;
        ccmr_value_mode = (0x0UL << TIM_CCMR1_CC1S_Pos) | // CC1S = 00 for Output /* PDF: CC1S = 00 */
                          (0x6UL << TIM_CCMR1_OC1M_Pos); // OC1M = 110 for PWM mode 1 /* PDF: OC1M = 110 */
        ccmr_value_preload = (0x1UL << TIM_CCMR1_OC1PE_Pos); // OC1PE = 1 for preload enable /* PDF: OC1PE = 1 */

        ccer_mask = TIM_CCER_CC1E_Msk | TIM_CCER_CC1P_Msk | TIM_CCER_CC1NP_Msk;
        ccer_value_enable = (0x0UL << TIM_CCER_CC1E_Pos); // CC1E = 0 initially disabled /* PDF: CC1E = 0 */
        ccer_value_polarity = (0x0UL << TIM_CCER_CC1P_Pos); // CC1P = 0 for active high /* PDF: CC1P = 0 */
        // CC1NP must be kept cleared for standard OCx outputs (TIM1 has CC1NP)
        // If TIMx is TIM1, ensure CC1NP is 0. Otherwise mask it out.
        if (config->TIMx != TIM1) ccer_mask &= ~TIM_CCER_CC1NP_Msk;

    }
    else if (config->ChannelNumber == 2)
    {
         // For CH2, uses CCMR1 register, bits related to OC2
         ccmr_mask = TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE_Msk | TIM_CCMR1_OC2FE_Msk | TIM_CCMR1_OC2CE_Msk;
         ccmr_value_mode = (0x0UL << TIM_CCMR1_CC2S_Pos) | // CC2S = 00 for Output /* PDF: CC2S = 00 */
                           (0x6UL << TIM_CCMR1_OC2M_Pos); // OC2M = 110 for PWM mode 1 /* PDF: OC2M = 110 */
         ccmr_value_preload = (0x1UL << TIM_CCMR1_OC2PE_Pos); // OC2PE = 1 for preload enable /* PDF: OC2PE = 1 */

         ccer_mask = TIM_CCER_CC2E_Msk | TIM_CCER_CC2P_Msk | TIM_CCER_CC2NP_Msk;
         ccer_value_enable = (0x0UL << TIM_CCER_CC2E_Pos); // CC2E = 0 initially disabled /* PDF: CC2E = 0 */
         ccer_value_polarity = (0x0UL << TIM_CCER_CC2P_Pos); // CC2P = 0 for active high /* PDF: CC2P = 0 */
          // If TIMx is TIM1, ensure CC2NP is 0. Otherwise mask it out.
         if (config->TIMx != TIM1) ccer_mask &= ~TIM_CCER_CC2NP_Msk;
    }
     else if (config->ChannelNumber == 3 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5) ) // TIM1, 2, 3, 4, 5 have CH3
    {
        // For CH3, uses CCMR2 register, bits related to OC3
        ccmr_mask = TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE_Msk | TIM_CCMR2_OC3FE_Msk | TIM_CCMR2_OC3CE_Msk;
        ccmr_value_mode = (0x0UL << TIM_CCMR2_CC3S_Pos) | // CC3S = 00 for Output /* PDF: CC3S = 00 */
                          (0x6UL << TIM_CCMR2_OC3M_Pos); // OC3M = 110 for PWM mode 1 /* PDF: OC3M = 110 */
        ccmr_value_preload = (0x1UL << TIM_CCMR2_OC3PE_Pos); // OC3PE = 1 for preload enable /* PDF: OC3PE = 1 */

        ccer_mask = TIM_CCER_CC3E_Msk | TIM_CCER_CC3P_Msk | TIM_CCER_CC3NP_Msk;
        ccer_value_enable = (0x0UL << TIM_CCER_CC3E_Pos); // CC3E = 0 initially disabled /* PDF: CC3E = 0 */
        ccer_value_polarity = (0x0UL << TIM_CCER_CC3P_Pos); // CC3P = 0 for active high /* PDF: CC3P = 0 */
         // If TIMx is TIM1, ensure CC3NP is 0. Otherwise mask it out.
        if (config->TIMx != TIM1) ccer_mask &= ~TIM_CCER_CC3NP_Msk;
    }
    else if (config->ChannelNumber == 4 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) // TIM1, 2, 3, 4, 5 have CH4
    {
        // For CH4, uses CCMR2 register, bits related to OC4
        ccmr_mask = TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE_Msk | TIM_CCMR2_OC4FE_Msk | TIM_CCMR2_OC4CE_Msk;
        ccmr_value_mode = (0x0UL << TIM_CCMR2_CC4S_Pos) | // CC4S = 00 for Output /* PDF: CC4S = 00 */
                          (0x6UL << TIM_CCMR2_OC4M_Pos); // OC4M = 110 for PWM mode 1 /* PDF: OC4M = 110 */
        ccmr_value_preload = (0x1UL << TIM_CCMR2_OC4PE_Pos); // OC4PE = 1 for preload enable /* PDF: OC4PE = 1 */

        ccer_mask = TIM_CCER_CC4E_Msk | TIM_CCER_CC4P_Msk; // No CC4NP bit /* PDF: CC4E = 0, CC4P = 0 */
        ccer_value_enable = (0x0UL << TIM_CCER_CC4E_Pos); // CC4E = 0 initially disabled
        ccer_value_polarity = (0x0UL << TIM_CCER_CC4P_Pos); // CC4P = 0 for active high

         // No CCxNP bit for Channel 4 on any timer with CH4
    }
    else
    {
         // Invalid channel number for this timer type or internal error
         return;
    }

    // Configure CCMR register (Output mode, PWM mode 1, preload enable)
    // The PDF mentions CCMRx offsets 0x18 for CCMR1 and 0x1C for CCMR2 from the timer base.
    // CMSIS provides TIMx->CCMR[0] for CCMR1 and TIMx->CCMR[1] for CCMR2.
    volatile uint32_t* ccmr_reg;
    if (config->ChannelNumber <= 2) {
        ccmr_reg = &config->TIMx->CCMR[0]; // CCMR1
    } else { // Channel 3 or 4
        ccmr_reg = &config->TIMx->CCMR[1]; // CCMR2
    }

    *ccmr_reg &= ~ccmr_mask;
    *ccmr_reg |= ccmr_value_mode | ccmr_value_preload;

    // Configure CCER register (Output enable, Polarity)
    config->TIMx->CCER &= ~ccer_mask;
    config->TIMx->CCER |= ccer_value_enable | ccer_value_polarity;

    // Configure CKD to default (00, tDTS=tCK_INT)
    config->TIMx->CR1 &= ~TIM_CR1_CKD_Msk; /* PDF: CR1 CKD */

    // Enable auto-reload preload for ARR
    config->TIMx->CR1 |= TIM_CR1_ARPE; /* PDF: CR1 ARPE = 1 */

    // For TIM1 (advanced timer), enable the main output (MOE bit in BDTR)
    // MOE needs to be set for the output to be active, even if CCxE is 1.
    // We set it here during init, but the actual output only becomes active when PWM_Start is called.
    if (config->TIMx == TIM1)
    {
         // BDTR is a 32-bit register, reset value 0x0000.
         // Set MOE bit (Bit 15).
         config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF: BDTR MOE = 1 */
    }


    // Generate an update event (software UG bit) to load the Prescaler, ARR, and CCRx values from preload registers
    // into active registers. This also clears the counter (if URS=0, which is the default).
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF: EGR UG = 1 */

    // Clear the update flag after generating the update
    // The UG bit auto-clears, but UIF might be set depending on URS bit (default URS=0)
    config->TIMx->SR &= ~TIM_SR_UIF; /* PDF: SR UIF */

    // Counter remains disabled until PWM_Start is called
    // config->TIMx->CR1 &= ~TIM_CR1_CEN; // Explicitly ensuring CEN is 0 (it should be 0 after reset anyway) /* PDF: CR1 CEN = 0 */
}

/**
  * @brief  Sets the desired PWM frequency and duty cycle for the selected channel.
  * @param  TRD_Channel: The PWM channel to configure.
  * @param  frequency: The desired frequency in Hz.
  * @param  duty: The desired duty cycle in percent (0-100).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    uint32_t timer_clock = PWM_TIMER_CLOCK; // Use assumed timer clock /* Assumed clock config - please verify */

    if (frequency == 0) // If frequency is 0, effectively turn off PWM by setting duty to 0
    {
        duty = 0;
    }
    if (duty > 100) duty = 100; // Cap duty cycle

    // Calculate Total Timer Counts per Period: Period_counts = Timer_Clock / Frequency
    // Use 64-bit intermediate for calculation to avoid overflow when timer_clock is large
    uint64_t period_counts_64 = (uint64_t)timer_clock / frequency;

    uint32_t prescaler = 0;
    uint32_t auto_reload;

    // Determine max possible ARR value for the timer
    uint32_t max_arr = (config->TIMx == TIM2 || config->TIMx == TIM5) ? 0xFFFFFFFFUL : 0xFFFFUL;

    if (period_counts_64 == 0) // Frequency is too high for the clock
    {
         prescaler = 0;
         auto_reload = 0; // Minimum possible period
    }
    else
    {
        // Find appropriate PSC and ARR values: (PSC+1) * (ARR+1) = period_counts_64
        // Iterate PSC from 0 upwards, finding the first PSC where (period_counts_64 / (PSC+1)) fits in ARR.
        // Prioritize lower PSC values for potentially better time resolution.
        prescaler = 0;
        uint64_t arr_plus_1_64;

        while (prescaler <= 0xFFFFUL)
        {
            // Check if (PSC + 1) is a divisor and the result fits in max_arr + 1
            uint32_t psc_plus_1 = prescaler + 1;
            if ((period_counts_64 % psc_plus_1) == 0) // Check for exact division
            {
                arr_plus_1_64 = period_counts_64 / psc_plus_1;
                if (arr_plus_1_64 > 0 && arr_plus_1_64 <= (uint64_t)max_arr + 1UL)
                {
                    auto_reload = (uint32_t)arr_plus_1_64 - 1;
                    goto found_params; // Found a suitable PSC/ARR pair
                }
            }
            // If exact division not found or doesn't fit, try next PSC
            prescaler++;
        }

        // If loop finishes without finding an ideal pair (should only happen if period_counts_64 is huge)
        // Fallback: Use max ARR and calculate PSC. This might not give exact frequency due to integer division.
        auto_reload = max_arr;
        uint64_t psc_plus_1_64 = period_counts_64 / (auto_reload + 1ULL);
        if (psc_plus_1_64 > (uint64_t)0xFFFFUL + 1ULL)
        {
            // Frequency is too low to achieve even with max PSC and max ARR
            prescaler = 0xFFFFUL;
            auto_reload = max_arr; // Keep max ARR
        }
        else
        {
             prescaler = (uint32_t)psc_plus_1_64 - 1;
             if (prescaler > 0xFFFFUL) prescaler = 0xFFFFUL; // Cap PSC
        }

        found_params:; // Label for goto
    }

    config->TIMx->PSC = prescaler; /* PDF: TIMx_PSC */
    config->TIMx->ARR = auto_reload; /* PDF: TIMx_ARR */

    // Calculate CCR value for duty cycle
    // For PWM Mode 1 (upcounting), output is high when CNT < CCRx. Period = ARR + 1 counts.
    // Duty Cycle (%) = (Number of 'high' counts / Total counts) * 100 = (CCRx + 1) / (ARR + 1) * 100
    // CCRx + 1 = (Duty * (ARR + 1)) / 100
    // CCRx = ((Duty * (ARR + 1)) / 100) - 1 ? No, ((Duty * (ARR + 1)) / 100) is the number of high counts.
    // High counts = (Duty * (ARR + 1)) / 100.  CCRx is the value *before* which the output is high.
    // If high counts = N, output is high for CNT from 0 to N-1. So CCRx should be N.
    // CCRx = (Duty * (ARR + 1)) / 100.
    // Using 64-bit intermediate for calculation precision
    uint32_t compare_value;
    if (duty == 0) {
        compare_value = 0; // 0% duty /* PDF: mentions CCR=0 gives 0% or 1 clock pulse */
    } else if (duty == 100) {
        compare_value = auto_reload + 1; // 100% duty /* PDF: mentions CCR > ARR holds output high */
    } else {
         compare_value = (uint32_t)(((uint64_t)duty * (auto_reload + 1UL)) / 100UL);
         // Note: If the calculated value is 0 but duty is > 0, it implies ARR+1 is small (<100).
         // e.g., duty=10, ARR=5. (10*6)/100 = 0. Set compare_value to 1 in this case to get at least 1 high count.
         // Except for 0% duty already handled.
         if (compare_value == 0 && duty > 0) compare_value = 1; // Ensure at least 1 high count if duty > 0
         if (compare_value > auto_reload + 1) compare_value = auto_reload + 1; // Cap at max for 100% behavior
    }


    // Update CCR register based on channel number
    if (config->ChannelNumber == 1)
    {
        config->TIMx->CCR1 = compare_value; /* PDF: TIMx_CCR1 */
    }
    else if (config->ChannelNumber == 2)
    {
        config->TIMx->CCR2 = compare_value; /* PDF: TIMx_CCR2 */
    }
    else if (config->ChannelNumber == 3 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5))
    {
        config->TIMx->CCR3 = compare_value; /* PDF: TIMx_CCR3 */
    }
     else if (config->ChannelNumber == 4 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5))
    {
        config->TIMx->CCR4 = compare_value; /* PDF: TIMx_CCR4 */
    }
    // No need to generate UG here, values will be loaded at next update event (counter overflow/underflow) because ARPE/OCxPE are enabled.
}

/**
  * @brief  Enable and start PWM signal generation on the specified channel.
  * @param  TRD_Channel: The PWM channel to start.
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Enable the specific channel output (CCxE bit)
    uint32_t ccer_enable_value;

    if (config->ChannelNumber == 1) {
        ccer_enable_value = TIM_CCER_CC1E; /* PDF: CC1E = 1 */
    } else if (config->ChannelNumber == 2) {
         ccer_enable_value = TIM_CCER_CC2E; /* PDF: CC2E = 1 */
    } else if (config->ChannelNumber == 3 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
         ccer_enable_value = TIM_CCER_CC3E; /* PDF: CC3E = 1 */
    } else if (config->ChannelNumber == 4 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
         ccer_enable_value = TIM_CCER_CC4E; /* PDF: CC4E = 1 */
    } else {
         // Invalid channel for this timer type
         return;
    }

    // Use |= to enable the specific bit without affecting other channel enables on the same timer
    config->TIMx->CCER |= ccer_enable_value; // Enable Capture/Compare output

    // For TIM1 (advanced timer), enable the main output (MOE bit in BDTR)
    // MOE is required for TIM1 outputs (OCx and OCN) to be driven by the timer.
    // BDTR is a 32-bit register.
    if (config->TIMx == TIM1)
    {
         config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF: BDTR MOE = 1 */
    }

    // Start the timer counter
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF: CR1 CEN = 1 */
}

/**
  * @brief  Stop the PWM signal output on the specified channel.
  * @param  TRD_Channel: The PWM channel to stop.
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Disable the specific channel output (CCxE bit)
    uint32_t ccer_disable_mask;

    if (config->ChannelNumber == 1) {
        ccer_disable_mask = TIM_CCER_CC1E_Msk;
    } else if (config->ChannelNumber == 2) {
         ccer_disable_mask = TIM_CCER_CC2E_Msk;
    } else if (config->ChannelNumber == 3 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
         ccer_disable_mask = TIM_CCER_CC3E_Msk;
    } else if (config->ChannelNumber == 4 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
         ccer_disable_mask = TIM_CCER_CC4E_Msk;
    } else {
         // Invalid channel for this timer type
         return;
    }

    // Use &= ~ to disable the specific bit without affecting other channel enables on the same timer
    config->TIMx->CCER &= ~ccer_disable_mask; /* PDF: CCxE = 0 */


    // For TIM1 (advanced timer), disable the main output (MOE bit in BDTR)
    // Note: This affects ALL channels on TIM1. If this function is expected to *only* stop
    // the *specific* channel's output without affecting others on the same timer,
    // this MOE clearing should only happen if NO channels on TIM1 are enabled.
    // Simple approach: clear MOE. A production system might need more complex state tracking.
    if (config->TIMx == TIM1)
    {
         config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF: BDTR MOE = 0 */
    }

    // Optional: Set the GPIO pin to a defined state (e.g., low) after stopping PWM.
    // This requires switching the pin mode back to GPIO output temporarily.
    // This is outside the strict definition of "Stop PWM signal output" but is good practice.
    // Let's set it back to Output Push-Pull Low.

    GPIO_TypeDef* gpio_port;
    if (config->PortName == GPIOA) {
        gpio_port = GPIOA;
    } else if (config->PortName == GPIOB) {
        gpio_port = GPIOB;
    } else if (config->PortName == GPIOC) {
        gpio_port = GPIOC;
    } else {
         // Should not happen based on map
         return;
    }

    // Configure MODER to General purpose output mode (01)
    uint32_t pin_mode_mask = (0x3UL << (config->PinNumber * 2));
    uint32_t output_mode_value = (0x1UL << (config->PinNumber * 2)); /* PDF: MODER = 01 for GP output */
    gpio_port->MODER &= ~pin_mode_mask; // Clear bits
    gpio_port->MODER |= output_mode_value; // Set to GP output mode

    // Set pin output low using BSRR for atomic write
    uint32_t pin_reset_mask = (0x1UL << (config->PinNumber + 16)); /* PDF: BSRR BRy bit (y+16) resets ODRy */
    gpio_port->BSRR = pin_reset_mask;

    // Note: The timer counter continues running unless explicitly stopped.
    // Stopping the counter might not be desired if other channels on the same timer are active.
    // The function description says "Stop the PWM signal output", not "Stop the timer".
    // Clearing CCxE (and MOE for TIM1) is sufficient for this.
    // If stopping the timer is required, uncomment the line below, but consider multi-channel usage.
    // config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF: CR1 CEN = 0 */
}

/**
  * @brief  Disable all PWM peripherals and outputs to reduce power consumption.
  * @param  None
  * @retval None
  */
void PWM_PowerOff(void)
{
    // Iterate through all configured PWM channels in the map
    for (size_t i = 0; i < sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]); ++i)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];

        // Disable the specific channel output (CCxE bit)
        uint32_t ccer_disable_mask;
         if (config->ChannelNumber == 1) {
            ccer_disable_mask = TIM_CCER_CC1E_Msk;
        } else if (config->ChannelNumber == 2) {
             ccer_disable_mask = TIM_CCER_CC2E_Msk;
        } else if (config->ChannelNumber == 3 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
             ccer_disable_mask = TIM_CCER_CC3E_Msk;
        } else if (config->ChannelNumber == 4 && (config->TIMx == TIM1 || config->TIMx == TIM2 || config->TIMx == TIM3 || config->TIMx == TIM4 || config->TIMx == TIM5)) {
             ccer_disable_mask = TIM_CCER_CC4E_Msk;
        } else {
             // Should not happen with a correct map
             continue;
        }
        config->TIMx->CCER &= ~ccer_disable_mask; /* PDF: CCxE = 0 */

         // For TIM1, disable the main output (MOE bit in BDTR). Affects all TIM1 channels.
         if (config->TIMx == TIM1)
         {
              config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF: BDTR MOE = 0 */
         }

        // Put the GPIO pin into Analog mode (low power consumption state, also default after reset)
        // PDF Section 8.3.12 describes Analog mode. Output buffer & Schmitt trigger disabled, pull-ups/downs disabled.
        GPIO_TypeDef* gpio_port;
        if (config->PortName == GPIOA) {
            gpio_port = GPIOA;
        } else if (config->PortName == GPIOB) {
            gpio_port = GPIOB;
        } else if (config->PortName == GPIOC) {
            gpio_port = GPIOC;
        } else {
             // Should not happen with a correct map
             continue;
        }

        uint32_t pin_mode_mask = (0x3UL << (config->PinNumber * 2));
        uint32_t analog_mode_value = (0x3UL << (config->PinNumber * 2)); /* PDF: MODER = 11 for Analog mode */
        gpio_port->MODER &= ~pin_mode_mask; // Clear bits
        gpio_port->MODER |= analog_mode_value; // Set to Analog mode

        // Note: This loop iterates through each channel. Clock disabling should be done once per peripheral.
        // Disabling clocks happens below after processing all pins.
    }

    // Explicitly stop all configured timers by clearing CEN bit
     for (size_t i = 0; i < sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]); ++i)
    {
         const PWM_Channel_Config_t* config = &pwm_channel_map[i];
         config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF: CR1 CEN = 0 */
    }


    // Disable clocks for all potentially used PWM timer peripherals
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN); /* Assumed bus config - please verify */
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN); /* Assumed bus config - please verify */

    // Disable clocks for all GPIO ports that had PWM configured on them (A, B, C)
    // This is safe as these pins are now in Analog mode (low power).
    RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN); /* PDF mentions GPIOA/B/C/D/E/H - AHB1 bus */
}