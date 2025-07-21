/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include "stm32f401xe.h" // Standard STM32F401XE CMSIS definitions for registers
#include <stdint.h>      // Required for uint32_t and uint8_t
#include <math.h>        // Required for floating point operations (e.g., in PWM_Set_Freq)

// Assumed System Clock and Timer Frequencies:
// The reference manual (RM0368) does not specify the SystemCoreClock or APB clock frequencies.
// Based on common STM32F401RC configurations:
// HCLK (System Clock) = 84 MHz
// APB1 Prescaler = 2 (so PCLK1 = 42 MHz). For timers on APB1, if APB1 prescaler > 1, the timer clock is 2 * PCLK1.
// APB2 Prescaler = 1 (so PCLK2 = 84 MHz). For timers on APB2, if APB2 prescaler = 1, the timer clock is PCLK2.
// Therefore, for all timers used in this implementation (TIM1, TIM3, TIM4, TIM9, TIM10, TIM11),
// the effective input clock frequency to the timer's prescaler is assumed to be 84 MHz.
#define TIM_CLK_FREQ_HZ 84000000UL /* Assumed timer input clock frequency (84 MHz for F401) */

// Helper macros for GPIO configuration based on RM0368, Table 24 "Port bit configuration table"
#define GPIO_MODE_INPUT       0x00U /* PDF Reference: 00: Input (reset state) */
#define GPIO_MODE_OUTPUT      0x01U /* PDF Reference: 01: General purpose output mode */
#define GPIO_MODE_AF          0x02U /* PDF Reference: 10: Alternate function mode */
#define GPIO_MODE_ANALOG      0x03U /* PDF Reference: 11: Analog mode */

#define GPIO_OTYPE_PP         0x00U // Push-pull /* PDF Reference: 0: Output push-pull (reset state) */
#define GPIO_OTYPE_OD         0x01U // Open-drain /* PDF Reference: 1: Output open-drain */

#define GPIO_OSPEED_LOW       0x00U /* PDF Reference: 00: Low speed */
#define GPIO_OSPEED_MEDIUM    0x01U /* PDF Reference: 01: Medium speed */
#define GPIO_OSPEED_HIGH      0x02U /* PDF Reference: 10: High speed */
#define GPIO_OSPEED_VERY_HIGH 0x03U /* PDF Reference: 11: Very high speed */

#define GPIO_PUPD_NONE        0x00U /* PDF Reference: 00: No pull-up, pull-down */
#define GPIO_PUPD_PU          0x01U /* PDF Reference: 01: Pull-up */
#define GPIO_PUPD_PD          0x02U /* PDF Reference: 10: Pull-down */

// Helper macros for Alternate Function definitions
// These are typical AF values for STM32F4xx series, as the provided PDF (RM0368)
// describes *how* to set AFs but not *which* AF corresponds to which TIM on specific pins.
#define GPIO_AF_TIM1  ((uint8_t)0x01U) // AF1
#define GPIO_AF_TIM2  ((uint8_t)0x01U) // AF1 (Reserved for OS/Delay)
#define GPIO_AF_TIM3  ((uint8_t)0x02U) // AF2
#define GPIO_AF_TIM4  ((uint8_t)0x02U) // AF2
#define GPIO_AF_TIM5  ((uint8_t)0x02U) // AF2 (Reserved for OS/Delay)
#define GPIO_AF_TIM9  ((uint8_t)0x03U) // AF3
#define GPIO_AF_TIM10 ((uint8_t)0x03U) // AF3
#define GPIO_AF_TIM11 ((uint8_t)0x03U) // AF3


// Define PWM_Channel_Config_t structure
typedef struct {
    TIM_TypeDef *TIMx;
    uint8_t ChannelNumber; // 1 to 4 for most timers, 1 for TIM10/11
    GPIO_TypeDef *PortName;
    uint8_t PinNumber;
    uint8_t AlternateFunctionNumber;
} PWM_Channel_Config_t;

// Reservation Note:
// TIM2 and TIM5 are explicitly reserved for potential use by an RTOS or other system delay
// functionalities (e.g., SysTick, high-resolution timers).
// Therefore, they are intentionally excluded from the pwm_channel_map.
// This adheres to the requirement to reserve at least two timers for OS/delay purposes.

static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (APB2 Timer, located on GPIOA)
    // Common mappings for STM32F401RC (64-pin package typically has A, B, C partially)
    // PA8 is TIM1_CH1
    {TIM1, 1, GPIOA, 8, GPIO_AF_TIM1}, /* Assumed PWM config - AF1 for TIM1 */
    // PA9 is TIM1_CH2
    {TIM1, 2, GPIOA, 9, GPIO_AF_TIM1}, /* Assumed PWM config - AF1 for TIM1 */
    // PA10 is TIM1_CH3
    {TIM1, 3, GPIOA, 10, GPIO_AF_TIM1}, /* Assumed PWM config - AF1 for TIM1 */

    // TIM3 Channels (APB1 Timer, can be on GPIOA, GPIOB, GPIOC)
    // PA6 is TIM3_CH1
    {TIM3, 1, GPIOA, 6, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PA7 is TIM3_CH2
    {TIM3, 2, GPIOA, 7, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PB0 is TIM3_CH3
    {TIM3, 3, GPIOB, 0, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3, Pin 0 allowed by instruction */
    // PB1 is TIM3_CH4
    {TIM3, 4, GPIOB, 1, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PC6 is TIM3_CH1 (alternative for PA6)
    {TIM3, 1, GPIOC, 6, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PC7 is TIM3_CH2 (alternative for PA7)
    {TIM3, 2, GPIOC, 7, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PC8 is TIM3_CH3 (alternative for PB0)
    {TIM3, 3, GPIOC, 8, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */
    // PC9 is TIM3_CH4 (alternative for PB1)
    {TIM3, 4, GPIOC, 9, GPIO_AF_TIM3}, /* Assumed PWM config - AF2 for TIM3 */

    // TIM4 Channels (APB1 Timer, located on GPIOB)
    // PB6 is TIM4_CH1
    {TIM4, 1, GPIOB, 6, GPIO_AF_TIM4}, /* Assumed PWM config - AF2 for TIM4 */
    // PB7 is TIM4_CH2
    {TIM4, 2, GPIOB, 7, GPIO_AF_TIM4}, /* Assumed PWM config - AF2 for TIM4 */
    // PB8 is TIM4_CH3
    {TIM4, 3, GPIOB, 8, GPIO_AF_TIM4}, /* Assumed PWM config - AF2 for TIM4 */
    // PB9 is TIM4_CH4
    {TIM4, 4, GPIOB, 9, GPIO_AF_TIM4}, /* Assumed PWM config - AF2 for TIM4 */

    // TIM9 Channels (APB2 Timer, located on GPIOA)
    // PA2 is TIM9_CH1
    {TIM9, 1, GPIOA, 2, GPIO_AF_TIM9}, /* Assumed PWM config - AF3 for TIM9 */
    // PA3 is TIM9_CH2
    {TIM9, 2, GPIOA, 3, GPIO_AF_TIM9}, /* Assumed PWM config - AF3 for TIM9 */

    // TIM10 Channel (APB2 Timer, can be on GPIOB)
    // PB8 is TIM10_CH1 (PB8 also supports TIM4_CH3 on AF2, which is a valid remapping for different AFs)
    {TIM10, 1, GPIOB, 8, GPIO_AF_TIM10}, /* Assumed PWM config - AF3 for TIM10 */

    // TIM11 Channel (APB2 Timer, can be on GPIOB)
    // PB9 is TIM11_CH1 (PB9 also supports TIM4_CH4 on AF2, which is a valid remapping for different AFs)
    {TIM11, 1, GPIOB, 9, GPIO_AF_TIM11} /* Assumed PWM config - AF3 for TIM11 */
};

/**Functions ===========================================================================*/

/**
 * @brief  Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param  TRD_Channel: The PWM channel to initialize.
 * @retval None
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    // Validate channel input based on the size of the pwm_channel_map array
    if (TRD_Channel >= (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])))
    {
        // Invalid channel, return without action
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // 1. Enable GPIO Peripheral Clock
    // (RCC registers are assumed as per common STM32 practices, direct register access like HAL)
    if (config->PortName == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    }
    else if (config->PortName == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
    }
    else if (config->PortName == GPIOC)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 
    }
    // For STM32F401RC (64-pin package), GPIOD and GPIOE are typically not available.

    // 2. Configure GPIO Pin for Alternate Function (AF) Mode
    // MODER: Set pin mode to Alternate Function (10)
    config->PortName->MODER &= ~(GPIO_MODER_MODER0_Msk << (config->PinNumber * 2)); /* PDF Reference: Bits 2y:2y+1 MODERy[1:0] */
    config->PortName->MODER |= (GPIO_MODE_AF << (config->PinNumber * 2));             /* PDF Reference: 10: Alternate function mode */

    // OTYPER: Set output type to Push-Pull (0) - common for PWM
    config->PortName->OTYPER &= ~(GPIO_OTYPER_OT0_Msk << config->PinNumber);         /* PDF Reference: Bits 15:0 OTy */
    config->PortName->OTYPER |= (GPIO_OTYPE_PP << config->PinNumber);                 /* PDF Reference: 0: Output push-pull (reset state) */

    // OSPEEDR: Set output speed to Very High Speed (11) - recommended for high-frequency PWM
    config->PortName->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk << (config->PinNumber * 2)); /* PDF Reference: Bits 2y:2y+1 OSPEEDRy[1:0] */
    config->PortName->OSPEEDR |= (GPIO_OSPEED_VERY_HIGH << (config->PinNumber * 2)); /* PDF Reference: 11: Very high speed */

    // PUPDR: Set to No Pull-up/Pull-down (00)
    config->PortName->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk << (config->PinNumber * 2)); /* PDF Reference: Bits 2y:2y+1 PUPDRy[1:0] */
    config->PortName->PUPDR |= (GPIO_PUPD_NONE << (config->PinNumber * 2));           /* PDF Reference: 00: No pull-up, pull-down */

    // AFRL/AFRH: Set Alternate Function Number for the pin
    if (config->PinNumber < 8)
    {
        // For pins 0-7, use AFRL (Alternate Function Low Register)
        config->PortName->AFR[0] &= ~(0xFU << (config->PinNumber * 4)); /* PDF Reference: Bits 31:0 AFRLy */
        config->PortName->AFR[0] |= (config->AlternateFunctionNumber << (config->PinNumber * 4)); /* PDF Reference: AFRLy selection */
    }
    else
    {
        // For pins 8-15, use AFRH (Alternate Function High Register)
        config->PortName->AFR[1] &= ~(0xFU << ((config->PinNumber - 8) * 4)); /* PDF Reference: Bits 31:0 AFRHy */
        config->PortName->AFR[1] |= (config->AlternateFunctionNumber << ((config->PinNumber - 8) * 4)); /* PDF Reference: AFRHy selection */
    }

    // 3. Enable Timer Peripheral Clock
    // (RCC registers are assumed as per common STM32 practices, direct register access like HAL)
    if (config->TIMx == TIM1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; 
    }
    else if (config->TIMx == TIM3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
    }
    else if (config->TIMx == TIM4)
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; 
    }
    else if (config->TIMx == TIM9)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; 
    }
    else if (config->TIMx == TIM10)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; 
    }
    else if (config->TIMx == TIM11)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; 
    }
    
    // 4. Configure Timer Base and PWM Mode
    // Disable counter before configuration to prevent unexpected behavior
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: Bit 0 CEN */
    
    // Configure CR1: Set Counter Mode (Up-counting, default), Clock Division (no division, default), and enable ARPE.
    // This approach aligns with HAL's TIM_Base_SetConfig which modifies specific CR1 fields.
    // Clear Counter Mode (CMS) and Direction (DIR) bits for Up-counting (00)
    config->TIMx->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); /* HAL equivalent: `tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); tmpcr1 |= Structure->CounterMode;` (where Structure->CounterMode is 0 for Up) */
    // Clear Clock Division (CKD) bits
    config->TIMx->CR1 &= ~TIM_CR1_CKD_Msk;             /* HAL equivalent: `tmpcr1 &= ~TIM_CR1_CKD;` */
    // Set Clock Division to no division (00) - this is redundant if CKD bits are already cleared by previous line.
    config->TIMx->CR1 |= (0x00U << TIM_CR1_CKD_Pos);   /* HAL equivalent: `tmpcr1 |= (uint32_t)Structure->ClockDivision;` (where Structure->ClockDivision is 0 for no division) */
    // Enable auto-reload preload (ARPE) for buffered ARR register
    config->TIMx->CR1 |= TIM_CR1_ARPE;                 /* PDF Reference: Bit 7 ARPE. HAL equivalent: `MODIFY_REG(tmpcr1, TIM_CR1_ARPE, Structure->AutoReloadPreload);` */

    // Configure Capture/Compare Mode Register (CCMR) for PWM Mode 1
    // The TIMx_CCMR1 handles channels 1 and 2. TIMx_CCMR2 handles channels 3 and 4.
    volatile uint32_t* ccmr_reg = NULL;
    uint8_t ccmr_channel_pos_offset = 0; // Offset for the specific channel's bits within the CCMR register

    if (config->ChannelNumber == 1)
    {
        ccmr_reg = (volatile uint32_t*)&(config->TIMx->CCMR1);
        ccmr_channel_pos_offset = 0; // CC1S and OC1M start at bit 0 and 4
    }
    else if (config->ChannelNumber == 2)
    {
        ccmr_reg = (volatile uint32_t*)&(config->TIMx->CCMR1);
        ccmr_channel_pos_offset = 8; // CC2S and OC2M start at bit 8 and 12
    }
    else if (config->ChannelNumber == 3)
    {
        ccmr_reg = (volatile uint32_t*)&(config->TIMx->CCMR2);
        ccmr_channel_pos_offset = 0; // CC3S and OC3M start at bit 0 and 4
    }
    else if (config->ChannelNumber == 4)
    {
        ccmr_reg = (volatile uint32_t*)&(config->TIMx->CCMR2);
        ccmr_channel_pos_offset = 8; // CC4S and OC4M start at bit 8 and 12
    }
    else
    {
        // Invalid channel number (should be caught by TRD_Channel validation)
        return;
    }

    // Clear CCxS bits to configure channel as output (00)
    *ccmr_reg &= ~((0x3U << (ccmr_channel_pos_offset + 0))); /* PDF Reference: CCxS */
    // Set OCxM to PWM mode 1 (110)
    *ccmr_reg &= ~((0x7U << (ccmr_channel_pos_offset + 4))); /* PDF Reference: OCxM */
    *ccmr_reg |=  ((0x6U << (ccmr_channel_pos_offset + 4))); /* PDF Reference: 110: PWM mode 1 */
    // Enable output compare preload enable (OCxPE) for buffered CCRx register
    *ccmr_reg |=  (0x1U << (ccmr_channel_pos_offset + 3));   /* PDF Reference: Bit OCxPE */
    // Configure Output Fast mode: Disable (0) by default for normal PWM operation. 
    // This matches HAL's typical PWM configuration if not explicitly set to Fast mode.
    // OCxFE bit is at position 2 for channel 1/3, and 10 for channel 2/4. This is (ccmr_channel_pos_offset + 2).
    *ccmr_reg &= ~(0x1U << (ccmr_channel_pos_offset + 2)); /* Clear OCxFE bit to disable Fast Mode (0) */

    // Configure Capture/Compare Enable Register (CCER)
    // CCxE bit position is (ChannelNumber-1) * 4
    uint8_t ccer_channel_pos_offset = (config->ChannelNumber - 1) * 4;

    // Clear CCxP (polarity) and CCxE (enable) bits for the specific channel.
    // Default polarity for PWM Mode 1 is High (CCxP = 0), which is achieved by clearing this bit.
    // CCxE will be set in PWM_Start.
    config->TIMx->CCER &= ~((TIM_CCER_CC1P_Msk | TIM_CCER_CC1E_Msk) << ccer_channel_pos_offset); /* PDF Reference: CC1P, CC1E */

    // For Advanced Timers (TIM1), enable Main Output Enable (MOE) bit in BDTR
    // This bit must be set for the output to be enabled for TIM1.
    // HAL also sets this in Start, but it's safe to set it here during initialization as well.
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: Bit 15 MOE */
    }

    // Generate an update event (UG) to load the preload registers (ARR, PSC, CCRx)
    // into the active registers before starting the counter.
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference: Bit 0 UG */
    
    // Clear UIF flag after update event generation (writing 0 to clear in status register)
    config->TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference: Bit 0 UIF */
}

/**
 * @brief  Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param  TRD_Channel: The PWM channel to configure.
 * @param  frequency: The desired PWM frequency in Hz.
 * @param  duty: The desired duty cycle in percentage (0-100).
 * @retval None
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, uint32_t frequency, uint8_t duty)
{
    // Validate input parameters
    if (TRD_Channel >= (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])) || frequency == 0 || duty > 100)
    {
        // Invalid channel or parameters, return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Calculate ARR and PSC to achieve the desired frequency and duty cycle.
    // Timer clock (TIM_CLK_FREQ_HZ) is assumed to be 84MHz.
    // Period = (ARR + 1) * (PSC + 1) / TIM_CLK_FREQ_HZ
    // For 16-bit timers (all used in this map), max ARR and PSC value is 65535 (0xFFFF).
    
    uint32_t period_counts = TIM_CLK_FREQ_HZ / frequency; 
    uint32_t psc_value = 0;
    uint32_t arr_value = period_counts - 1;

    // Adjust PSC if initial ARR exceeds the 16-bit range or if it's too large for a desired resolution
    // Try to keep PSC as small as possible for higher resolution, but large enough for ARR to fit.
    while (arr_value > 0xFFFFUL && psc_value < 0xFFFFUL)
    {
        psc_value++;
        // Recalculate ARR based on new PSC
        arr_value = (TIM_CLK_FREQ_HZ / (frequency * (psc_value + 1))) - 1;
    }

    // Check if the desired frequency can be achieved within 16-bit limits for ARR and PSC
    if (arr_value > 0xFFFFUL || psc_value > 0xFFFFUL || arr_value < 1) // ARR must be at least 1 for a signal
    {
        // Cannot achieve the desired frequency with current timer configuration limits, or frequency too high/low
        return; 
    }

    // Stop timer counter to ensure safe modification of ARR and PSC
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: Bit 0 CEN */

    // Set Prescaler (PSC) and Auto-Reload Register (ARR) values
    config->TIMx->PSC = (uint16_t)psc_value; /* PDF Reference: PSC[15:0] */
    config->TIMx->ARR = (uint16_t)arr_value; /* PDF Reference: ARR[15:0] */

    // Calculate Compare Value (CCRx) based on duty cycle.
    // Duty cycle in PWM Mode 1: Active as long as CNT < CCRx.
    // 0% duty: CCRx = 0. Output held low.
    // 100% duty: CCRx = ARR + 1. Output held high. (as per PDF "If compare value > ARR, OCxREF held at 1")
    // (Duty / 100) = (CCRx / (ARR + 1))
    uint32_t ccr_value;
    if (duty == 0)
    {
        ccr_value = 0;
    }
    else if (duty == 100)
    {
        ccr_value = arr_value + 1; // For 100% duty cycle, set CCRx > ARR to keep output high
    }
    else
    {
        // General duty cycle calculation: CCRx = (duty / 100) * (ARR + 1)
        ccr_value = (uint32_t)(((float)duty / 100.0F) * (arr_value + 1));
        // Ensure CCRx does not exceed ARR if not 100% duty to stay within the normal period bounds
        // This implicitly handles potential floating point inaccuracies ensuring CCRx <= ARR for duty < 100%
        if (ccr_value > arr_value) {
            ccr_value = arr_value; 
        }
    }
    
    // Set Capture/Compare Register (CCRx) value
    switch (config->ChannelNumber)
    {
        case 1: config->TIMx->CCR1 = (uint16_t)ccr_value; break; /* PDF Reference: CCR1[15:0] */
        case 2: config->TIMx->CCR2 = (uint16_t)ccr_value; break; /* PDF Reference: CCR2[15:0] */
        case 3: config->TIMx->CCR3 = (uint16_t)ccr_value; break; /* PDF Reference: CCR3[15:0] */
        case 4: config->TIMx->CCR4 = (uint16_t)ccr_value; break; /* PDF Reference: CCR4[15:0] */
        default: break; // Should not happen due to TRD_Channel validation
    }

    // Generate an update event (UG) to immediately apply the new ARR, PSC, and CCRx values
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference: Bit 0 UG */
    // Clear UIF flag after update event generation (writing 0 to clear in status register)
    config->TIMx->SR &= ~TIM_SR_UIF; /* PDF Reference: Bit 0 UIF */

    // Re-enable timer counter if it was running before this function call (or if it should start after config)
    // For production, often re-enabled by PWM_Start explicitly. For convenience, re-enable here.
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference: Bit 0 CEN */
}

/**
 * @brief  Enables and starts PWM signal generation on the specified channel.
 * @param  TRD_Channel: The PWM channel to start.
 * @retval None
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    // Validate channel input
    if (TRD_Channel >= (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])))
    {
        // Invalid channel, return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the specific Capture/Compare Channel Output (CCxE)
    uint8_t ccer_channel_pos_offset = (config->ChannelNumber - 1) * 4;
    config->TIMx->CCER |= (0x01U << (TIM_CCER_CC1E_Pos + ccer_channel_pos_offset)); /* PDF Reference: CC1E. HAL equivalent: TIM_CCxChannelCmd(TIMx, Channel, TIM_CCx_ENABLE); */

    // For Advanced Control Timers (like TIM1), Main Output Enable (MOE) must be set.
    // This affects all channels of TIM1.
    if (config->TIMx == TIM1)
    {
        config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: Bit 15 MOE. HAL equivalent: __HAL_TIM_MOE_ENABLE(htim); */
    }

    // Enable the timer counter (CEN)
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference: Bit 0 CEN. HAL equivalent: __HAL_TIM_ENABLE(htim); */
}

/**
 * @brief  Stops the PWM signal output on the specified channel.
 * @param  TRD_Channel: The PWM channel to stop.
 * @retval None
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    // Validate channel input
    if (TRD_Channel >= (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])))
    {
        // Invalid channel, return
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the specific Capture/Compare Channel Output (CCxE)
    // This will stop the PWM signal on the specific pin, but the timer counter will keep running.
    // This is correct for multi-channel timers where other channels might still be active.
    uint8_t ccer_channel_pos_offset = (config->ChannelNumber - 1) * 4;
    config->TIMx->CCER &= ~((0x01U << (TIM_CCER_CC1E_Pos + ccer_channel_pos_offset))); /* PDF Reference: CC1E. HAL equivalent: TIM_CCxChannelCmd(TIMx, Channel, TIM_CCx_DISABLE); */

    // For Advanced Timers (like TIM1), if no other channels are active, MOE can be cleared.
    // This function is per-channel, so only clearing CCxE is appropriate here.
    // Global MOE and CEN disabling are handled by PWM_PowerOff.
}

/**
 * @brief  Disables all PWM peripherals and outputs to reduce power consumption.
 * @param  None
 * @retval None
 */
void PWM_PowerOff(void)
{
    // Iterate through all defined PWM channels in the map
    for (uint32_t i = 0; i < (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])); i++)
    {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];

        // 1. Disable the specific Capture/Compare Channel Output (CCxE)
        uint8_t ccer_channel_pos_offset = (config->ChannelNumber - 1) * 4;
        config->TIMx->CCER &= ~((0x01U << (TIM_CCER_CC1E_Pos + ccer_channel_pos_offset))); /* PDF Reference: CC1E */
        
        // 2. For Advanced Control Timers (TIM1), disable Main Output Enable (MOE)
        if (config->TIMx == TIM1)
        {
            config->TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference: Bit 15 MOE. HAL equivalent: __HAL_TIM_MOE_DISABLE(htim); */
        }
        
        // 3. Disable the Timer Counter (CEN)
        config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: Bit 0 CEN. HAL equivalent: __HAL_TIM_DISABLE(htim); */
        
        // 4. Reconfigure GPIO pin to Input Floating mode (default reset state for GPIOs).
        // This is done to minimize power consumption and release the pin from alternate function control.
        // MODER: Set to Input mode (00)
        config->PortName->MODER &= ~(GPIO_MODER_MODER0_Msk << (config->PinNumber * 2)); /* PDF Reference: Bits 2y:2y+1 MODERy[1:0] */
        config->PortName->MODER |= (GPIO_MODE_INPUT << (config->PinNumber * 2));         /* PDF Reference: 00: Input (reset state) */
        
        // Other GPIO registers (OTYPER, OSPEEDR, PUPDR, AFRL/AFRH) are typically reset
        // by MCU reset to their default safe states (e.g., Push-Pull, Low Speed, No Pull-up/down, AF0).
        // Explicitly clearing them here is generally redundant if MODER is set to Input,
        // as their settings become irrelevant for input pins.
        // For absolute power minimization, setting PUPD to None is good practice.
        config->PortName->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk << (config->PinNumber * 2)); /* PDF Reference: Bits 2y:2y+1 PUPDRy[1:0] */
    }

    // 5. Disable all used Timer Peripheral Clocks in RCC.
    // (RCC registers are assumed as per common STM32 practices, direct register access like HAL)
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN); 
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN); 
    
    // GPIO clocks are usually left enabled as GPIOs might be used by other non-PWM functions.
    // For a complete system power-off, more extensive RCC clock gating for GPIOs and other
    // unused peripherals would be required, but this function focuses on PWM-specific components.
}