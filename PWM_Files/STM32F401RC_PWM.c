/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Bare-metal PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-20
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "pwm.h"
#include "stm32f401xe.h" // CMSIS device header for STM32F401RC registers

// Assuming these type definitions are available from a central header like "types.h"
// If not, define them here or in pwm.h
typedef uint32_t tlong;
typedef uint8_t tbyte;

// TRD_Channel_t (from pwm.h)
// typedef enum {
//     TRD_CHANNEL_PWM1 = 0,
//     TRD_CHANNEL_PWM2,
//     TRD_CHANNEL_PWM3,
//     NUM_TRD_PWM_CHANNELS
// } TRD_Channel_t;

// Define System Clock Frequency for STM32F401RC
// Typical HCLK is 84MHz. APB1 prescaler /2 (PCLK1=42MHz), APB2 prescaler /1 (PCLK2=84MHz).
// For TIM2/3/4/5 (APB1 timers), clock is 2*PCLK1 = 84MHz.
// For TIM1/9/10/11 (APB2 timers), clock is PCLK2 = 84MHz.
// So, all general-purpose timers in this configuration receive an 84MHz clock.
#define SYSTEM_TIMER_CLOCK_HZ 84000000UL

// Define standard GPIO Pin numbers if not available from stm32f401xe.h / CMSIS
#ifndef GPIO_PIN_0
#define GPIO_PIN_0                 ((uint16_t)0x0001U)
#define GPIO_PIN_1                 ((uint16_t)0x0002U)
#define GPIO_PIN_2                 ((uint16_t)0x0004U)
#define GPIO_PIN_3                 ((uint16_t)0x0008U)
#define GPIO_PIN_4                 ((uint16_t)0x0010U)
#define GPIO_PIN_5                 ((uint16_t)0x0020U)
#define GPIO_PIN_6                 ((uint16_t)0x0040U)
#define GPIO_PIN_7                 ((uint16_t)0x0080U)
#define GPIO_PIN_8                 ((uint16_t)0x0100U)
#define GPIO_PIN_9                 ((uint16_t)0x0200U)
#define GPIO_PIN_10                ((uint16_t)0x0400U)
#define GPIO_PIN_11                ((uint16_t)0x0800U)
#define GPIO_PIN_12                ((uint16_t)0x1000U)
#define GPIO_PIN_13                ((uint16_t)0x2000U)
#define GPIO_PIN_14                ((uint16_t)0x4000U)
#define GPIO_PIN_15                ((uint16_t)0x8000U)
#endif

// Define Alternate Function numbers (specific to STM32F401RC)
#define GPIO_AF1_TIM1              ((uint8_t)0x01U)
#define GPIO_AF1_TIM2              ((uint8_t)0x01U)
#define GPIO_AF2_TIM3              ((uint8_t)0x02U)
#define GPIO_AF2_TIM4              ((uint8_t)0x02U)
#define GPIO_AF2_TIM5              ((uint8_t)0x02U)
#define GPIO_AF3_TIM9              ((uint8_t)0x03U)
#define GPIO_AF3_TIM10             ((uint8_t)0x03U)
#define GPIO_AF3_TIM11             ((uint8_t)0x03U)

// Define TIM Channel numbers (used by HAL, useful for consistency)
#define TIM_CHANNEL_1             ((uint32_t)0x00000000U)
#define TIM_CHANNEL_2             ((uint32_t)0x00000004U)
#define TIM_CHANNEL_3             ((uint32_t)0x00000008U)
#define TIM_CHANNEL_4             ((uint32_t)0x0000000CU)

// Define register bit masks directly for bare-metal access
// RCC AHB1ENR
#define RCC_AHB1ENR_GPIOAEN_Pos     (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk     (0x1U << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN_Pos     (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk     (0x1U << RCC_AHB1ENR_GPIOBEN_Pos)

// RCC APB1ENR
#define RCC_APB1ENR_TIM2EN_Pos      (0U)
#define RCC_APB1ENR_TIM2EN_Msk      (0x1U << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM3EN_Pos      (1U)
#define RCC_APB1ENR_TIM3EN_Msk      (0x1U << RCC_APB1ENR_TIM3EN_Pos)
#define RCC_APB1ENR_TIM4EN_Pos      (2U)
#define RCC_APB1ENR_TIM4EN_Msk      (0x1U << RCC_APB1ENR_TIM4EN_Pos)
#define RCC_APB1ENR_TIM5EN_Pos      (3U)
#define RCC_APB1ENR_TIM5EN_Msk      (0x1U << RCC_APB1ENR_TIM5EN_Pos)

// RCC APB2ENR
#define RCC_APB2ENR_TIM1EN_Pos      (0U)
#define RCC_APB2ENR_TIM1EN_Msk      (0x1U << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_TIM9EN_Pos      (16U)
#define RCC_APB2ENR_TIM9EN_Msk      (0x1U << RCC_APB2ENR_TIM9EN_Pos)
#define RCC_APB2ENR_TIM10EN_Pos     (17U)
#define RCC_APB2ENR_TIM10EN_Msk     (0x1U << RCC_APB2ENR_TIM10EN_Pos)
#define RCC_APB2ENR_TIM11EN_Pos     (18U)
#define RCC_APB2ENR_TIM11EN_Msk     (0x1U << RCC_APB2ENR_TIM11EN_Pos)

// GPIO MODER, OTYPER, OSPEEDR, PUPDR, AFRL/AFRH
#define GPIO_MODER_MODE_Pos(pin)    ((pin) * 2U)
#define GPIO_MODER_MODE_Msk(pin)    (0x3U << GPIO_MODER_MODE_Pos(pin))
#define GPIO_MODER_AF_Msk(pin)      (0x2U << GPIO_MODER_MODE_Pos(pin)) // Alternate function mode

#define GPIO_OTYPER_OT_Pos(pin)     (pin)
#define GPIO_OTYPER_OT_Msk(pin)     (0x1U << GPIO_OTYPER_OT_Pos(pin))
#define GPIO_OTYPER_PP_Msk(pin)     (0x0U << GPIO_OTYPER_OT_Pos(pin)) // Push-pull

#define GPIO_OSPEEDR_OSPEED_Pos(pin) ((pin) * 2U)
#define GPIO_OSPEEDR_OSPEED_Msk(pin) (0x3U << GPIO_OSPEEDR_OSPEED_Pos(pin))
#define GPIO_OSPEEDR_VHIGH_Msk(pin)  (0x3U << GPIO_OSPEEDR_OSPEED_Pos(pin)) // Very high speed

#define GPIO_PUPDR_PUPD_Pos(pin)    ((pin) * 2U)
#define GPIO_PUPDR_PUPD_Msk(pin)    (0x3U << GPIO_PUPDR_PUPD_Pos(pin))
#define GPIO_PUPDR_NOPULL_Msk(pin)  (0x0U << GPIO_PUPDR_PUPD_Pos(pin)) // No pull-up/pull-down

#define GPIO_AFR_AF_Msk             (0xFU)

// TIM CR1 Register bits
#define TIM_CR1_CEN_Pos             (0U)
#define TIM_CR1_CEN_Msk             (0x1U << TIM_CR1_CEN_Pos)
#define TIM_CR1_ARPE_Pos            (7U)
#define TIM_CR1_ARPE_Msk            (0x1U << TIM_CR1_ARPE_Pos)
#define TIM_CR1_DIR_Pos             (4U)
#define TIM_CR1_DIR_Msk             (0x1U << TIM_CR1_DIR_Pos)
#define TIM_CR1_CMS_Pos             (5U)
#define TIM_CR1_CMS_Msk             (0x3U << TIM_CR1_CMS_Pos)

// TIM CCMR1 Register bits (for Channels 1 and 2)
#define TIM_CCMR1_OC1M_Pos          (4U)
#define TIM_CCMR1_OC1M_Msk          (0x7U << TIM_CCMR1_OC1M_Pos)
#define TIM_CCMR1_OC1M_PWM1         (0x6U << TIM_CCMR1_OC1M_Pos) // PWM mode 1
#define TIM_CCMR1_OC1PE_Pos         (3U)
#define TIM_CCMR1_OC1PE_Msk         (0x1U << TIM_CCMR1_OC1PE_Pos)

#define TIM_CCMR1_OC2M_Pos          (12U)
#define TIM_CCMR1_OC2M_Msk          (0x7U << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2M_PWM1         (0x6U << TIM_CCMR1_OC2M_Pos) // PWM mode 1
#define TIM_CCMR1_OC2PE_Pos         (11U)
#define TIM_CCMR1_OC2PE_Msk         (0x1U << TIM_CCMR1_OC2PE_Pos)

// TIM CCMR2 Register bits (for Channels 3 and 4)
#define TIM_CCMR2_OC3M_Pos          (4U)
#define TIM_CCMR2_OC3M_Msk          (0x7U << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3M_PWM1         (0x6U << TIM_CCMR2_OC3M_Pos) // PWM mode 1
#define TIM_CCMR2_OC3PE_Pos         (3U)
#define TIM_CCMR2_OC3PE_Msk         (0x1U << TIM_CCMR2_OC3PE_Pos)

#define TIM_CCMR2_OC4M_Pos          (12U)
#define TIM_CCMR2_OC4M_Msk          (0x7U << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4M_PWM1         (0x6U << TIM_CCMR2_OC4M_Pos) // PWM mode 1
#define TIM_CCMR2_OC4PE_Pos         (11U)
#define TIM_CCMR2_OC4PE_Msk         (0x1U << TIM_CCMR2_OC4PE_Pos)

// TIM CCER Register bits (Channel Enable/Polarity)
#define TIM_CCER_CC1E_Pos           (0U)
#define TIM_CCER_CC1P_Pos           (1U)
#define TIM_CCER_CC2E_Pos           (4U)
#define TIM_CCER_CC2P_Pos           (5U)
#define TIM_CCER_CC3E_Pos           (8U)
#define TIM_CCER_CC3P_Pos           (9U)
#define TIM_CCER_CC4E_Pos           (12U)
#define TIM_CCER_CC4P_Pos           (13U)

// TIM EGR Register bits (Event Generation)
#define TIM_EGR_UG_Pos              (0U)
#define TIM_EGR_UG_Msk              (0x1U << TIM_EGR_UG_Pos)

// TIM BDTR Register bits (Break and Dead-Time Register - for advanced timers like TIM1)
#define TIM_BDTR_MOE_Pos            (15U)
#define TIM_BDTR_MOE_Msk            (0x1U << TIM_BDTR_MOE_Pos)


// Internal structure to map logical channels to hardware configurations
typedef struct {
    TIM_TypeDef *TIMx;
    uint32_t ChannelNumber; // TIM_CHANNEL_1, TIM_CHANNEL_2, etc.
    GPIO_TypeDef *PortName;
    uint16_t PinNumber;     // GPIO_PIN_0, GPIO_PIN_1, etc.
    uint8_t AF_Number;      // AF1, AF2, etc.
} PWM_Channel_Config_t;

// Array mapping TRD_Channel_t to specific STM32F401RC PWM hardware
// Ensure this matches the TRD_Channel_t enum in pwm.h
static const PWM_Channel_Config_t pwm_channel_map[NUM_TRD_PWM_CHANNELS] = {
    [TRD_CHANNEL_PWM1] = {
        .TIMx = TIM2,
        .ChannelNumber = TIM_CHANNEL_1,
        .PortName = GPIOA,
        .PinNumber = GPIO_PIN_0,
        .AF_Number = GPIO_AF1_TIM2,
    },
    [TRD_CHANNEL_PWM2] = {
        .TIMx = TIM3,
        .ChannelNumber = TIM_CHANNEL_1,
        .PortName = GPIOA,
        .PinNumber = GPIO_PIN_6,
        .AF_Number = GPIO_AF2_TIM3,
    },
    [TRD_CHANNEL_PWM3] = {
        .TIMx = TIM4,
        .ChannelNumber = TIM_CHANNEL_2,
        .PortName = GPIOB,
        .PinNumber = GPIO_PIN_7,
        .AF_Number = GPIO_AF2_TIM4,
    }
};

/**
 * @brief Helper function to get the pin index (0-15) from GPIO_PIN_X define.
 * @param pin The GPIO_PIN_X value (e.g., GPIO_PIN_0).
 * @retval The pin index (0-15).
 */
static uint8_t get_pin_index(uint16_t pin) {
    uint8_t index = 0;
    // Iterate to find the bit position
    while (((pin >> index) & 0x1) == 0 && index < 16) {
        index++;
    }
    return index;
}

/**
 * @brief Helper function to enable the clock for a given GPIO port.
 * @param GPIOx Pointer to the GPIO port (e.g., GPIOA, GPIOB).
 * @retval None
 */
static void enable_gpio_clock(GPIO_TypeDef *GPIOx) {
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;
    } else if (GPIOx == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk;
    }
    // Add other GPIO ports if used by pwm_channel_map
    // Ensure the clock is stable
    (void)RCC->AHB1ENR;
}

/**
 * @brief Helper function to disable the clock for a given GPIO port.
 * @param GPIOx Pointer to the GPIO port (e.g., GPIOA, GPIOB).
 * @retval None
 */
static void disable_gpio_clock(GPIO_TypeDef *GPIOx) {
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN_Msk;
    } else if (GPIOx == GPIOB) {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN_Msk;
    }
}

/**
 * @brief Helper function to enable the clock for a given TIM peripheral.
 * @param TIMx Pointer to the TIM peripheral (e.g., TIM1, TIM2).
 * @retval None
 */
static void enable_tim_clock(TIM_TypeDef *TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk;
    } else if (TIMx == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;
    } else if (TIMx == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN_Msk;
    } else if (TIMx == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN_Msk;
    } else if (TIMx == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN_Msk;
    } else if (TIMx == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN_Msk;
    } else if (TIMx == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN_Msk;
    } else if (TIMx == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN_Msk;
    }
    // Ensure the clock is stable
    (void)RCC->APB1ENR; // Read to ensure clock is enabled
    (void)RCC->APB2ENR;
}

/**
 * @brief Helper function to disable the clock for a given TIM peripheral.
 * @param TIMx Pointer to the TIM peripheral (e.g., TIM1, TIM2).
 * @retval None
 */
static void disable_tim_clock(TIM_TypeDef *TIMx) {
    if (TIMx == TIM1) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN_Msk;
    } else if (TIMx == TIM2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN_Msk;
    } else if (TIMx == TIM3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN_Msk;
    } else if (TIMx == TIM4) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN_Msk;
    } else if (TIMx == TIM5) {
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN_Msk;
    } else if (TIMx == TIM9) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN_Msk;
    } else if (TIMx == TIM10) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN_Msk;
    } else if (TIMx == TIM11) {
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN_Msk;
    }
}

/**
 * @brief Configures a specific TIM channel for PWM mode.
 * @param TIMx Pointer to the TIM peripheral.
 * @param Channel The TIM channel to configure (e.g., TIM_CHANNEL_1).
 * @param PulseValue The initial compare value for the channel (duty cycle).
 * @retval None
 */
static void TIM_PWM_SetConfig(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t PulseValue) {
    uint32_t tmpccmrx;
    uint32_t ccer_pos; // Position of CCxE bit in CCER

    // Determine the CCER enable bit position for the channel
    if (Channel == TIM_CHANNEL_1) {
        ccer_pos = TIM_CCER_CC1E_Pos;
    } else if (Channel == TIM_CHANNEL_2) {
        ccer_pos = TIM_CCER_CC2E_Pos;
    } else if (Channel == TIM_CHANNEL_3) {
        ccer_pos = TIM_CCER_CC3E_Pos;
    } else { // TIM_CHANNEL_4
        ccer_pos = TIM_CCER_CC4E_Pos;
    }

    // Disable the Channel output before configuration
    TIMx->CCER &= ~(1U << ccer_pos);

    if (Channel == TIM_CHANNEL_1) {
        tmpccmrx = TIMx->CCMR1;
        // Clear Output Compare Mode (OC1M) and Capture/Compare Selection (CC1S) bits
        tmpccmrx &= ~(TIM_CCMR1_OC1M_Msk | (0x3U << 0));
        // Set PWM mode 1 (0b110) and enable output compare preload (OC1PE)
        tmpccmrx |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE_Msk;
        TIMx->CCMR1 = tmpccmrx;
        TIMx->CCR1 = PulseValue;
        // Set Output Compare Polarity to Active High (clear CC1P bit)
        TIMx->CCER &= ~TIM_CCER_CC1P_Msk;
    } else if (Channel == TIM_CHANNEL_2) {
        tmpccmrx = TIMx->CCMR1;
        // Clear OC2M and CC2S bits
        tmpccmrx &= ~(TIM_CCMR1_OC2M_Msk | (0x3U << 8));
        // Set PWM mode 1 (0b110) and enable output compare preload (OC2PE)
        tmpccmrx |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE_Msk;
        TIMx->CCMR1 = tmpccmrx;
        TIMx->CCR2 = PulseValue;
        // Set Output Compare Polarity to Active High (clear CC2P bit)
        TIMx->CCER &= ~TIM_CCER_CC2P_Msk;
    } else if (Channel == TIM_CHANNEL_3) {
        tmpccmrx = TIMx->CCMR2;
        // Clear OC3M and CC3S bits
        tmpccmrx &= ~(TIM_CCMR2_OC3M_Msk | (0x3U << 0));
        // Set PWM mode 1 (0b110) and enable output compare preload (OC3PE)
        tmpccmrx |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE_Msk;
        TIMx->CCMR2 = tmpccmrx;
        TIMx->CCR3 = PulseValue;
        // Set Output Compare Polarity to Active High (clear CC3P bit)
        TIMx->CCER &= ~TIM_CCER_CC3P_Msk;
    } else { // TIM_CHANNEL_4
        tmpccmrx = TIMx->CCMR2;
        // Clear OC4M and CC4S bits
        tmpccmrx &= ~(TIM_CCMR2_OC4M_Msk | (0x3U << 8));
        // Set PWM mode 1 (0b110) and enable output compare preload (OC4PE)
        tmpccmrx |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE_Msk;
        TIMx->CCMR2 = tmpccmrx;
        TIMx->CCR4 = PulseValue;
        // Set Output Compare Polarity to Active High (clear CC4P bit)
        TIMx->CCER &= ~TIM_CCER_CC4P_Msk;
    }
}

/***********************************************************************************************************************
* Function Name  : PWM_Init
* Description    : Initializes the PWM peripheral for a specific channel.
* Arguments      : TRD_Channel_t TRD_Channel - Logical PWM channel identifier.
* Returns        : None
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= NUM_TRD_PWM_CHANNELS) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    GPIO_TypeDef *GPIOx = config->PortName;
    uint16_t PinNumber = config->PinNumber;
    uint8_t PinIndex = get_pin_index(PinNumber);
    uint8_t AF_Number = config->AF_Number;
    uint32_t ChannelNumber = config->ChannelNumber;

    // 1. Enable GPIO clock
    enable_gpio_clock(GPIOx);

    // 2. Configure GPIO pin for Alternate Function
    // Clear MODER bits for the pin and set to Alternate Function (10)
    GPIOx->MODER &= ~GPIO_MODER_MODE_Msk(PinIndex);
    GPIOx->MODER |= GPIO_MODER_AF_Msk(PinIndex);

    // Configure Output type to Push-pull (0)
    GPIOx->OTYPER &= ~GPIO_OTYPER_OT_Msk(PinIndex);
    GPIOx->OTYPER |= GPIO_OTYPER_PP_Msk(PinIndex);

    // Configure Output speed to Very High (11)
    GPIOx->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED_Msk(PinIndex);
    GPIOx->OSPEEDR |= GPIO_OSPEEDR_VHIGH_Msk(PinIndex);

    // Configure Pull-up/Pull-down to No Pull-up/Pull-down (00)
    GPIOx->PUPDR &= ~GPIO_PUPDR_PUPD_Msk(PinIndex);
    GPIOx->PUPDR |= GPIO_PUPDR_NOPULL_Msk(PinIndex);

    // Configure Alternate Function (AFR[0] for Pin 0-7, AFR[1] for Pin 8-15)
    if (PinIndex < 8) {
        GPIOx->AFR[0] &= ~(GPIO_AFR_AF_Msk << (PinIndex * 4));
        GPIOx->AFR[0] |= (AF_Number << (PinIndex * 4));
    } else {
        GPIOx->AFR[1] &= ~(GPIO_AFR_AF_Msk << ((PinIndex - 8) * 4));
        GPIOx->AFR[1] |= (AF_Number << ((PinIndex - 8) * 4));
    }

    // 3. Enable TIM clock
    enable_tim_clock(TIMx);

    // 4. Configure TIM base (CR1, PSC, ARR)
    // Disable counter before configuration
    TIMx->CR1 &= ~TIM_CR1_CEN_Msk;

    // Clear Counter Mode (CMS) and Direction (DIR) bits for Up-counting
    TIMx->CR1 &= ~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk);

    // Set auto-reload preload enable (ARPE)
    TIMx->CR1 |= TIM_CR1_ARPE_Msk;

    // Default PSC and ARR (will be updated by PWM_Set_Freq)
    TIMx->PSC = 0;
    TIMx->ARR = 0xFFFF; // Set a reasonable default max period for 16-bit timers

    // 5. Configure PWM mode for the channel
    // PWM Mode 1 (active high), initial pulse value 0 (0% duty cycle)
    TIM_PWM_SetConfig(TIMx, ChannelNumber, 0);

    // For advanced timers (TIM1), enable Main Output (MOE bit in BDTR)
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE_Msk;
    }

    // Generate an update event to load the Prescaler and the auto-reload value immediately
    TIMx->EGR |= TIM_EGR_UG_Msk;
}

/***********************************************************************************************************************
* Function Name  : PWM_Set_Freq
* Description    : Sets the frequency and duty cycle for a specific PWM channel.
* Arguments      : TRD_Channel_t TRD_Channel - Logical PWM channel identifier.
*                  tlong frequency - Desired PWM frequency in Hz.
*                  tbyte duty - Desired duty cycle (0-255, where 255 is 100%).
* Returns        : None
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    if (TRD_Channel >= NUM_TRD_PWM_CHANNELS || frequency == 0) {
        return; // Invalid channel or frequency
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;

    uint32_t prescaler = 0;
    uint32_t period = 0;

    // Calculate Prescaler (PSC) and Auto-Reload Register (ARR)
    // Timer_Clock_Freq = SYSTEM_TIMER_CLOCK_HZ / (PSC + 1)
    // PWM_Frequency = Timer_Clock_Freq / (ARR + 1)
    // So, ARR = (Timer_Clock_Freq / PWM_Frequency) - 1
    // We try to keep PSC as small as possible for higher resolution,
    // but ensure ARR fits in 16-bit for 16-bit timers or 32-bit for 32-bit timers.

    uint32_t max_period = (TIMx == TIM2 || TIMx == TIM5) ? 0xFFFFFFFFUL : 0xFFFFUL;

    for (prescaler = 0; prescaler <= 0xFFFFUL; prescaler++) { // PSC is 16-bit
        if (SYSTEM_TIMER_CLOCK_HZ / (prescaler + 1) < frequency) {
            continue; // Timer clock after prescaler is too low
        }
        period = (SYSTEM_TIMER_CLOCK_HZ / (prescaler + 1)) / frequency;
        if (period > 0) { // Period must be at least 1
            period--; // ARR = period - 1
            if (period <= max_period) {
                break; // Found suitable values
            }
        }
    }

    if (prescaler > 0xFFFFUL || period > max_period) {
        // Could not find suitable PSC/ARR values for the desired frequency
        // This might occur if the frequency is too high/low for the timer capabilities
        return;
    }

    // Set Prescaler and Auto-Reload Register
    TIMx->PSC = prescaler;
    TIMx->ARR = period;

    // Calculate Capture Compare Register (Pulse)
    // Pulse = (Period + 1) * duty / 255. For PWM Mode 1, active output when CNT < CCRx.
    // If duty = 255 (100%), Pulse should be equal to or greater than Period to ensure 100% duty cycle.
    // If duty = 0, Pulse should be 0.
    uint32_t pulse_value = (period * duty) / 255;
    if (duty == 255) { // Ensure 100% duty cycle means output is always high
        pulse_value = period + 1;
    }


    // Update the Capture Compare Register for the specific channel
    if (ChannelNumber == TIM_CHANNEL_1) {
        TIMx->CCR1 = pulse_value;
    } else if (ChannelNumber == TIM_CHANNEL_2) {
        TIMx->CCR2 = pulse_value;
    } else if (ChannelNumber == TIM_CHANNEL_3) {
        TIMx->CCR3 = pulse_value;
    } else { // TIM_CHANNEL_4
        TIMx->CCR4 = pulse_value;
    }

    // Generate an update event to load the new PSC, ARR, and CCR values immediately
    TIMx->EGR |= TIM_EGR_UG_Msk;
}

/***********************************************************************************************************************
* Function Name  : PWM_Start
* Description    : Starts PWM generation on a specific channel.
* Arguments      : TRD_Channel_t TRD_Channel - Logical PWM channel identifier.
* Returns        : None
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= NUM_TRD_PWM_CHANNELS) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;
    uint32_t ccer_pos; // Position of CCxE bit in CCER

    // Determine the CCER enable bit position for the channel
    if (ChannelNumber == TIM_CHANNEL_1) {
        ccer_pos = TIM_CCER_CC1E_Pos;
    } else if (ChannelNumber == TIM_CHANNEL_2) {
        ccer_pos = TIM_CCER_CC2E_Pos;
    } else if (ChannelNumber == TIM_CHANNEL_3) {
        ccer_pos = TIM_CCER_CC3E_Pos;
    } else { // TIM_CHANNEL_4
        ccer_pos = TIM_CCER_CC4E_Pos;
    }

    // Enable the Capture Compare channel output
    TIMx->CCER |= (1U << ccer_pos);

    // For advanced timers (TIM1), enable Main Output Enable (MOE)
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE_Msk;
    }

    // Enable the Counter
    TIMx->CR1 |= TIM_CR1_CEN_Msk;
}

/***********************************************************************************************************************
* Function Name  : PWM_Stop
* Description    : Stops PWM generation on a specific channel.
* Arguments      : TRD_Channel_t TRD_Channel - Logical PWM channel identifier.
* Returns        : None
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= NUM_TRD_PWM_CHANNELS) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;
    uint32_t ccer_pos; // Position of CCxE bit in CCER

    // Determine the CCER enable bit position for the channel
    if (ChannelNumber == TIM_CHANNEL_1) {
        ccer_pos = TIM_CCER_CC1E_Pos;
    } else if (ChannelNumber == TIM_CHANNEL_2) {
        ccer_pos = TIM_CCER_CC2E_Pos;
    } else if (ChannelNumber == TIM_CHANNEL_3) {
        ccer_pos = TIM_CCER_CC3E_Pos;
    } else { // TIM_CHANNEL_4
        ccer_pos = TIM_CCER_CC4E_Pos;
    }

    // Disable the Capture Compare channel output
    TIMx->CCER &= ~(1U << ccer_pos);

    // For advanced timers (TIM1), disable Main Output Enable (MOE)
    if (TIMx == TIM1) {
        TIMx->BDTR &= ~TIM_BDTR_MOE_Msk;
    }

    // Disable the Counter
    // Note: This disables the entire timer. If multiple channels are used on the same timer,
    // a more complex state management would be needed to only stop the timer when all channels are disabled.
    // For this bare-metal exercise, stopping a channel implies stopping its timer.
    TIMx->CR1 &= ~TIM_CR1_CEN_Msk;
}

/***********************************************************************************************************************
* Function Name  : PWM_PowerOff
* Description    : Powers off all initialized PWM peripherals.
* Arguments      : None
* Returns        : None
***********************************************************************************************************************/
void PWM_PowerOff(void) {
    for (int i = 0; i < NUM_TRD_PWM_CHANNELS; i++) {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];
        TIM_TypeDef *TIMx = config->TIMx;
        GPIO_TypeDef *GPIOx = config->PortName;
        uint16_t PinNumber = config->PinNumber;
        uint8_t PinIndex = get_pin_index(PinNumber);
        uint32_t ChannelNumber = config->ChannelNumber;
        uint32_t ccer_pos;

        // Determine the CCER enable bit position for the channel
        if (ChannelNumber == TIM_CHANNEL_1) {
            ccer_pos = TIM_CCER_CC1E_Pos;
        } else if (ChannelNumber == TIM_CHANNEL_2) {
            ccer_pos = TIM_CCER_CC2E_Pos;
        } else if (ChannelNumber == TIM_CHANNEL_3) {
            ccer_pos = TIM_CCER_CC3E_Pos;
        } else { // TIM_CHANNEL_4
            ccer_pos = TIM_CCER_CC4E_Pos;
        }

        // 1. Stop the PWM channel (if active)
        // Disable the Capture Compare channel output
        TIMx->CCER &= ~(1U << ccer_pos);

        // For advanced timers (TIM1), disable Main Output Enable (MOE)
        if (TIMx == TIM1) {
            TIMx->BDTR &= ~TIM_BDTR_MOE_Msk;
        }

        // Disable the Counter
        TIMx->CR1 &= ~TIM_CR1_CEN_Msk;

        // 2. Disable TIM peripheral clock
        disable_tim_clock(TIMx);

        // 3. Reset GPIO pin to default input state (analog input or floating input)
        // MODER: Input mode (00)
        GPIOx->MODER &= ~GPIO_MODER_MODE_Msk(PinIndex);
        GPIOx->MODER |= (0x0U << GPIO_MODER_MODE_Pos(PinIndex));

        // OTYPER: Push-pull (default for most pins)
        GPIOx->OTYPER &= ~GPIO_OTYPER_OT_Msk(PinIndex);
        GPIOx->OTYPER |= GPIO_OTYPER_PP_Msk(PinIndex); // Set to push-pull (default state)

        // OSPEEDR: Low speed (00)
        GPIOx->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED_Msk(PinIndex);
        GPIOx->OSPEEDR |= (0x0U << GPIO_OSPEEDR_OSPEED_Pos(PinIndex));

        // PUPDR: No pull-up/pull-down (00)
        GPIOx->PUPDR &= ~GPIO_PUPDR_PUPD_Msk(PinIndex);
        GPIOx->PUPDR |= GPIO_PUPDR_NOPULL_Msk(PinIndex);

        // AFR: Clear Alternate Function (set to AF0, which is system function or general-purpose input)
        if (PinIndex < 8) {
            GPIOx->AFR[0] &= ~(GPIO_AFR_AF_Msk << (PinIndex * 4));
        } else {
            GPIOx->AFR[1] &= ~(GPIO_AFR_AF_Msk << ((PinIndex - 8) * 4));
        }

        // 4. Disable GPIO clock (this will truly power down the port, saving more power)
        disable_gpio_clock(GPIOx);
    }
}