/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM implementation for STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-17
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "pwm.h" // Assumed to contain TRD_Channel_t

typedef unsigned long tlong;
typedef unsigned char tbyte;

// Minimal bare-metal register definitions for STM32F401RC
// These definitions are extracted from common STM32F4xx device headers and reference manuals
// but are self-contained here as per instructions to avoid HAL dependencies.

// Peripheral Base Addresses
#define PERIPH_BASE           (0x40000000UL)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)

// RCC Base Address
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)

// GPIO Base Addresses
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)

// TIM Base Addresses
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)

// Structure definitions for direct register access
typedef struct
{
  volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

typedef struct
{
  volatile uint32_t CR1;          /*!< TIM control register 1,              Address offset: 0x00 */
  volatile uint32_t CR2;          /*!< TIM control register 2,              Address offset: 0x04 */
  volatile uint32_t SMCR;         /*!< TIM slave mode control register,     Address offset: 0x08 */
  volatile uint32_t DIER;         /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  volatile uint32_t SR;           /*!< TIM status register,                 Address offset: 0x10 */
  volatile uint32_t EGR;          /*!< TIM event generation register,       Address offset: 0x14 */
  volatile uint32_t CCMR1;        /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  volatile uint32_t CCMR2;        /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  volatile uint32_t CCER;         /*!< TIM capture/compare enable register, Address offset: 0x20 */
  volatile uint32_t CNT;          /*!< TIM counter register,                Address offset: 0x24 */
  volatile uint32_t PSC;          /*!< TIM prescaler register,              Address offset: 0x28 */
  volatile uint32_t ARR;          /*!< TIM auto-reload register,            Address offset: 0x2C */
  volatile uint32_t RCR;          /*!< TIM repetition counter register,     Address offset: 0x30 */
  volatile uint32_t CCR1;         /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  volatile uint32_t CCR2;         /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  volatile uint32_t CCR3;         /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  volatile uint32_t CCR4;         /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  volatile uint32_t BDTR;         /*!< TIM break and dead-time register,    Address offset: 0x44 */
  volatile uint32_t DCR;          /*!< TIM DMA control register,            Address offset: 0x48 */
  volatile uint32_t DMAR;         /*!< TIM DMA actual address register,     Address offset: 0x4C */
} TIM_TypeDef;

typedef struct
{
  volatile uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  volatile uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  volatile uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                 */
  volatile uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                            */
  volatile uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register,                   Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock enable register,                   Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock enable register,                   Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                 */
  volatile uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                            */
  volatile uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                 */
  volatile uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                            */
  volatile uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  volatile uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                            */
  volatile uint32_t SSCGR;         /*!< RCC Spread spectrum clock generation register,               Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  volatile uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;


// Pointers to peripherals
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)

// RCC clock enable bits
#define RCC_AHB1ENR_GPIOAEN   (1UL << 0U)
#define RCC_AHB1ENR_GPIOBEN   (1UL << 1U)
#define RCC_AHB1ENR_GPIOCEN   (1UL << 2U)

#define RCC_APB1ENR_TIM2EN    (1UL << 0U)
#define RCC_APB1ENR_TIM3EN    (1UL << 1U)
#define RCC_APB1ENR_TIM4EN    (1UL << 2U)

#define RCC_APB2ENR_TIM1EN    (1UL << 0U)

// GPIO Register Bit definitions (related to MODER, OTYPER, OSPEEDR, PUPDR, AFR)
#define GPIO_MODE_INPUT       0x00UL
#define GPIO_MODE_OUTPUT      0x01UL
#define GPIO_MODE_AF          0x02UL
#define GPIO_MODE_ANALOG      0x03UL

#define GPIO_SPEED_LOW        0x00UL
#define GPIO_SPEED_MEDIUM     0x01UL
#define GPIO_SPEED_HIGH       0x02UL
#define GPIO_SPEED_VERY_HIGH  0x03UL

#define GPIO_PUPD_NOPULL      0x00UL
#define GPIO_PUPD_PU          0x01UL
#define GPIO_PUPD_PD          0x02UL

// GPIO Alternate Function definitions for TIM on STM32F401RC (common ones)
#define GPIO_AF1_TIM1           0x01UL
#define GPIO_AF1_TIM2           0x01UL
#define GPIO_AF2_TIM3           0x02UL
#define GPIO_AF2_TIM4           0x02UL

// TIM Register Bit definitions
#define TIM_CR1_CEN               (1UL << 0U)   // Counter Enable
#define TIM_CR1_ARPE              (1UL << 7U)   // Auto-Reload Preload Enable
#define TIM_CR1_CKD               (0x3UL << 8U) // Clock Division (00: t_DTS=t_CK_INT)
#define TIM_CR1_CMS               (0x3UL << 5U) // Center-aligned mode selection (00: Edge-aligned mode)
#define TIM_CR1_DIR               (1UL << 4U)   // Direction (0: upcounter)

#define TIM_EGR_UG                (1UL << 0U)   // Update Generation

// Capture/Compare Mode Register 1 (CCMR1) for CH1/CH2
// OC1M (Output Compare 1 Mode): Bits 6:4
#define TIM_CCMR1_OC1M_Pos        (4U)
#define TIM_CCMR1_OC1M            (0x7UL << TIM_CCMR1_OC1M_Pos)
#define TIM_OCMODE_PWM1           (0x6UL << TIM_CCMR1_OC1M_Pos) // PWM mode 1

// OC1PE (Output Compare 1 Preload Enable): Bit 3
#define TIM_CCMR1_OC1PE           (1UL << 3U)

// OC1FE (Output Compare 1 Fast Enable): Bit 2
#define TIM_CCMR1_OC1FE           (1UL << 2U)

// CC1S (Capture/Compare 1 Selection): Bits 1:0 (00 for Output Compare)
#define TIM_CCMR1_CC1S            (0x3UL << 0U)

// CCMR1 for CH2 (shifted by 8 for OC2M, OC2PE, OC2FE, CC2S)
#define TIM_CCMR1_OC2M_Pos        (4U + 8U)
#define TIM_CCMR1_OC2M            (0x7UL << TIM_CCMR1_OC2M_Pos)
#define TIM_CCMR1_OC2PE           (1UL << (3U + 8U))
#define TIM_CCMR1_OC2FE           (1UL << (2U + 8U))
#define TIM_CCMR1_CC2S            (0x3UL << (0U + 8U))


// Capture/Compare Mode Register 2 (CCMR2) for CH3/CH4 (similar to CCMR1)
#define TIM_CCMR2_OC3M_Pos        (4U)
#define TIM_CCMR2_OC3M            (0x7UL << TIM_CCMR2_OC3M_Pos)
#define TIM_CCMR2_OC3PE           (1UL << 3U)
#define TIM_CCMR2_OC3FE           (1UL << 2U)
#define TIM_CCMR2_CC3S            (0x3UL << 0U)

#define TIM_CCMR2_OC4M_Pos        (4U + 8U)
#define TIM_CCMR2_OC4M            (0x7UL << TIM_CCMR2_OC4M_Pos)
#define TIM_CCMR2_OC4PE           (1UL << (3U + 8U))
#define TIM_CCMR2_OC4FE           (1UL << (2U + 8U))
#define TIM_CCMR2_CC4S            (0x3UL << (0U + 8U))


// Capture/Compare Enable Register (CCER) for CCxE, CCxP, CCxNP, CCxNE
#define TIM_CCER_CC1E             (1UL << 0U)   // Capture/Compare 1 output enable
#define TIM_CCER_CC1P             (1UL << 1U)   // Capture/Compare 1 output Polarity (0: high, 1: low)
#define TIM_CCER_CC1NP            (1UL << 3U)   // Capture/Compare 1 complementary output Polarity (for TIM1/8)

#define TIM_CCER_CC2E             (1UL << 4U)
#define TIM_CCER_CC2P             (1UL << 5U)
#define TIM_CCER_CC2NP            (1UL << 7U)

#define TIM_CCER_CC3E             (1UL << 8U)
#define TIM_CCER_CC3P             (1UL << 9U)
#define TIM_CCER_CC3NP            (1UL << 11U)

#define TIM_CCER_CC4E             (1UL << 12U)
#define TIM_CCER_CC4P             (1UL << 13U)
// Note: TIM1_CH4 does not have a complementary output, so CC4NP is not applicable.

// For advanced timers (TIM1): Break and Dead-Time Register (BDTR)
#define TIM_BDTR_MOE              (1UL << 15U)  // Main Output Enable

// TIM Channels (used in HAL for `Channel` parameter in `TIM_CCxChannelCmd` logic)
#define TIM_CHANNEL_1             0x00000000UL
#define TIM_CHANNEL_2             0x00000004UL
#define TIM_CHANNEL_3             0x00000008UL
#define TIM_CHANNEL_4             0x0000000CUL
// The (Channel & 0x1FU) logic in HAL correctly uses the low bits of these defines as shifts for CCER bits.

// Output Polarity Configuration (for `OCPolarity` in Internal_TIM_OC_Config_t)
#define TIM_OCPOLARITY_HIGH       0x00000000UL // Corresponds to clearing CCxP bit
#define TIM_OCPOLARITY_LOW        0x00000001UL // Corresponds to setting CCxP bit (inverted from actual bit value for consistency with HAL)

// Output Fast Mode
#define TIM_OCFAST_DISABLE        0x00000000UL // Corresponds to clearing OCxFE bit

// For advanced timers (TIM1), if complementary channels and idle states were used
#define TIM_OCN_POLARITY_HIGH     0x00000000UL // Corresponds to clearing CCxNP bit
#define TIM_OCN_POLARITY_LOW      0x00000001UL // Corresponds to setting CCxNP bit (inverted from actual bit value)
#define TIM_OCIDLESTATE_LOW       0x00000000UL // Corresponds to clearing OISx bit
#define TIM_OCIDLESTATE_HIGH      0x00000001UL // Corresponds to setting OISx bit
#define TIM_OCNIDLESTATE_LOW      0x00000000UL // Corresponds to clearing OISxN bit
#define TIM_OCNIDLESTATE_HIGH     0x00000001UL // Corresponds to setting OISxN bit

// OIS (Output Idle State) bits in TIMx_CR2
#define TIM_CR2_OIS1              (1UL << 0U)
#define TIM_CR2_OIS2              (1UL << 2U)
#define TIM_CR2_OIS3              (1UL << 4U)
#define TIM_CR2_OIS4              (1UL << 6U)
#define TIM_CR2_OIS1N             (1UL << 1U)
#define TIM_CR2_OIS2N             (1UL << 3U)
#define TIM_CR2_OIS3N             (1UL << 5U)


// Peripheral Clock Frequency (for STM32F401RC, assuming HCLK=84MHz)
// For F401RC, APB1 and APB2 timer clocks are typically 84MHz.
#define TIM_CLOCK_FREQ      84000000UL

// Internal structure for PWM Channel configuration
typedef struct
{
    TIM_TypeDef    *TIMx;
    uint32_t        ChannelNumber;  // TIM_CHANNEL_x constant (e.g., TIM_CHANNEL_1)
    GPIO_TypeDef   *PortName;
    uint32_t        PinNumber;      // GPIO pin number (0-15)
    uint32_t        AFNumber;       // Alternate Function number (e.g., GPIO_AF1_TIM2)
    uint32_t        RCC_TIM_EN_BIT; // Bit mask for enabling TIM clock in RCC_APB1ENR or RCC_APB2ENR
    uint32_t        RCC_GPIO_EN_BIT; // Bit mask for enabling GPIO clock in RCC_AHB1ENR
    uint8_t         IsAdvancedTimer; // Flag: 1 if advanced timer (like TIM1), 0 otherwise
} PWM_Channel_Config_t;

// Array mapping TRD_Channel_t to specific hardware configurations on STM32F401RC
// The specific pins and AFs are based on common STM32F401RC configurations.
const PWM_Channel_Config_t pwm_channel_map[] = {
    [TRD_PWM_CHANNEL_1] = {
        .TIMx = TIM2,
        .ChannelNumber = TIM_CHANNEL_1,
        .PortName = GPIOA,
        .PinNumber = 0, // PA0
        .AFNumber = GPIO_AF1_TIM2,
        .RCC_TIM_EN_BIT = RCC_APB1ENR_TIM2EN,
        .RCC_GPIO_EN_BIT = RCC_AHB1ENR_GPIOAEN,
        .IsAdvancedTimer = 0
    },
    [TRD_PWM_CHANNEL_2] = {
        .TIMx = TIM3,
        .ChannelNumber = TIM_CHANNEL_2,
        .PortName = GPIOA,
        .PinNumber = 7, // PA7
        .AFNumber = GPIO_AF2_TIM3,
        .RCC_TIM_EN_BIT = RCC_APB1ENR_TIM3EN,
        .RCC_GPIO_EN_BIT = RCC_AHB1ENR_GPIOAEN,
        .IsAdvancedTimer = 0
    },
    [TRD_PWM_CHANNEL_3] = {
        .TIMx = TIM4,
        .ChannelNumber = TIM_CHANNEL_3,
        .PortName = GPIOB,
        .PinNumber = 8, // PB8
        .AFNumber = GPIO_AF2_TIM4,
        .RCC_TIM_EN_BIT = RCC_APB1ENR_TIM4EN,
        .RCC_GPIO_EN_BIT = RCC_AHB1ENR_GPIOBEN,
        .IsAdvancedTimer = 0
    },
    [TRD_PWM_CHANNEL_4] = {
        .TIMx = TIM1,
        .ChannelNumber = TIM_CHANNEL_1,
        .PortName = GPIOA,
        .PinNumber = 8, // PA8
        .AFNumber = GPIO_AF1_TIM1,
        .RCC_TIM_EN_BIT = RCC_APB2ENR_TIM1EN,
        .RCC_GPIO_EN_BIT = RCC_AHB1ENR_GPIOAEN,
        .IsAdvancedTimer = 1 // TIM1 is an advanced timer
    }
};

// Internal structure to pass configuration similar to HAL_TIM_OC_InitTypeDef
// This struct is not exposed externally.
typedef struct {
  uint32_t OCMode;
  uint32_t Pulse;
  uint32_t OCPolarity;
  uint32_t OCFastMode;
  // Parameters below are relevant for advanced timers (like TIM1) with complementary outputs
  uint32_t OCNPolarity;  // Output Compare N Polarity
  uint32_t OCIdleState;  // Output Compare Idle State
  uint32_t OCNIdleState; // Output Compare N Idle State
  uint8_t IsAdvancedTimer; // Flag to indicate if the current timer is an advanced timer
} Internal_TIM_OC_Config_t;


/* Private function prototypes */
static void TIM_CCxChannelCmd_BareMetal(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);
static void TIM_OC_SetConfig_BareMetal(TIM_TypeDef *TIMx, uint32_t Channel, Internal_TIM_OC_Config_t *OC_Config);
static void GPIO_Config_AF_Output(GPIO_TypeDef *GPIOx, uint32_t pin_idx, uint32_t af_num);
static void GPIO_Config_Input_NoPull(GPIO_TypeDef *GPIOx, uint32_t pin_idx);


/**
  * @brief  Enables or disables the TIM Capture Compare Channel x.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel specifies the TIM Channel (e.g., TIM_CHANNEL_1)
  * @param  ChannelState specifies the TIM Channel CCxE bit new state.
  *          This parameter can be: 1 (ENABLE) or 0 (DISABLE).
  * @retval None
  */
static void TIM_CCxChannelCmd_BareMetal(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState)
{
  // Calculate the bit position for the specific channel's enable bit (CCxE) in CCER
  uint32_t tmp_cce_bit = (TIM_CCER_CC1E << (Channel & 0x1FU)); 

  // Clear the CCxE Bit
  TIMx->CCER &= ~tmp_cce_bit;

  // Set or reset the CCxE Bit based on ChannelState
  if (ChannelState) {
      TIMx->CCER |= tmp_cce_bit;
  }
}

/**
  * @brief  Configure a TIM Output Compare/PWM channel directly using register access.
  *         This function encapsulates the logic from HAL's TIM_OCx_SetConfig.
  * @param  TIMx The TIM peripheral instance.
  * @param  Channel The TIM channel to configure (e.g., TIM_CHANNEL_1).
  * @param  OC_Config Pointer to Internal_TIM_OC_Config_t structure.
  * @retval None
  */
static void TIM_OC_SetConfig_BareMetal(TIM_TypeDef *TIMx, uint32_t Channel, Internal_TIM_OC_Config_t *OC_Config)
{
  uint32_t tmpccmrx = 0;
  uint32_t tmpccer = TIMx->CCER;
  uint32_t tmpcr2 = TIMx->CR2;

  // Disable the channel's output enable bit first, as per HAL logic.
  TIM_CCxChannelCmd_BareMetal(TIMx, Channel, 0); // 0 means disable

  // Select appropriate CCMR register based on channel (CCMR1 for CH1/CH2, CCMR2 for CH3/CH4)
  volatile uint32_t *ccmr_reg;
  uint32_t oc_mode_mask;
  uint32_t oc_mode_shift;
  uint32_t cc_selection_mask;
  uint32_t ocpe_bit;
  uint32_t ocfe_bit;

  if ((Channel == TIM_CHANNEL_1) || (Channel == TIM_CHANNEL_2)) {
      ccmr_reg = &TIMx->CCMR1;
  } else { // TIM_CHANNEL_3, TIM_CHANNEL_4
      ccmr_reg = &TIMx->CCMR2;
  }

  // Determine bit masks and shifts based on channel number within the CCMR register (first or second channel group)
  if ((Channel == TIM_CHANNEL_1) || (Channel == TIM_CHANNEL_3)) { // First channel in CCMRx register
      oc_mode_mask = TIM_CCMR1_OC1M; 
      cc_selection_mask = TIM_CCMR1_CC1S; 
      oc_mode_shift = TIM_CCMR1_OC1M_Pos; // OC1M_Pos also applies to OC3M_Pos
      ocpe_bit = TIM_CCMR1_OC1PE;
      ocfe_bit = TIM_CCMR1_OC1FE;
  } else { // TIM_CHANNEL_2 or TIM_CHANNEL_4 (Second channel in CCMRx register)
      oc_mode_mask = TIM_CCMR1_OC2M; 
      cc_selection_mask = TIM_CCMR1_CC2S; 
      oc_mode_shift = TIM_CCMR1_OC2M_Pos; // OC2M_Pos also applies to OC4M_Pos
      ocpe_bit = TIM_CCMR1_OC2PE;
      ocfe_bit = TIM_CCMR1_OC2FE;
  }

  // Read current CCMR value
  tmpccmrx = *ccmr_reg;

  // Clear OC Mode and Capture/Compare Selection bits.
  // Setting CCxS to 00 configures the channel as output compare.
  tmpccmrx &= ~(oc_mode_mask | cc_selection_mask);
  // Set OC Mode (e.g., PWM1)
  tmpccmrx |= OC_Config->OCMode;

  // Set Preload Enable bit
  tmpccmrx |= ocpe_bit;

  // Configure Output Fast mode (cleared then set)
  tmpccmrx &= ~ocfe_bit; // Clear fast mode enable bit
  if (OC_Config->OCFastMode != TIM_OCFAST_DISABLE) { // If fast mode is desired (not supported by this basic implementation)
      tmpccmrx |= ocfe_bit;
  }

  // Write updated CCMR value
  *ccmr_reg = tmpccmrx;

  // Configure Output Polarity in CCER
  uint32_t ccp_bit = (TIM_CCER_CC1P << (Channel & 0x1FU));
  tmpccer &= ~ccp_bit; // Clear current polarity (default to high active)
  if (OC_Config->OCPolarity == TIM_OCPOLARITY_LOW) {
      tmpccer |= ccp_bit; // Set for low polarity if requested
  }

  // Handle complementary channel polarity if it's an advanced timer with N-channels
  if (OC_Config->IsAdvancedTimer) {
    uint32_t ccnp_bit = (TIM_CCER_CC1NP << (Channel & 0x1FU));
    tmpccer &= ~ccnp_bit; // Clear current N-polarity
    if (OC_Config->OCNPolarity == TIM_OCN_POLARITY_LOW) {
        tmpccer |= ccnp_bit; // Set for low N-polarity
    }
    // Also clear the CCxNE bit (complementary output enable) for consistency with common usage
    // It will be enabled with TIM_CCxChannelCmd_BareMetal for the N channel if needed externally.
    tmpccer &= ~(1UL << ((Channel & 0x1FU) + 2)); // CC1NE = CC1E + 2 bits.

    // Handle Output Idle States (OISx, OISxN) for advanced timers (TIM1) in CR2
    uint32_t ois_bit = 0;
    uint32_t oisn_bit = 0;

    // Determine OIS bits based on Channel number
    if (Channel == TIM_CHANNEL_1) {
        ois_bit = TIM_CR2_OIS1;
        oisn_bit = TIM_CR2_OIS1N;
    } else if (Channel == TIM_CHANNEL_2) {
        ois_bit = TIM_CR2_OIS2;
        oisn_bit = TIM_CR2_OIS2N;
    } else if (Channel == TIM_CHANNEL_3) {
        ois_bit = TIM_CR2_OIS3;
        oisn_bit = TIM_CR2_OIS3N;
    } else if (Channel == TIM_CHANNEL_4) {
        ois_bit = TIM_CR2_OIS4;
        oisn_bit = 0; // TIM1_CH4 does not have a complementary output in F401RC
    }

    tmpcr2 &= ~(ois_bit | oisn_bit); // Clear both OIS and OISN bits

    if (OC_Config->OCIdleState == TIM_OCIDLESTATE_HIGH) {
        tmpcr2 |= ois_bit;
    }
    if (OC_Config->OCNIdleState == TIM_OCNIDLESTATE_HIGH) {
        tmpcr2 |= oisn_bit;
    }
  }
  
  // Write to TIMx CR2
  TIMx->CR2 = tmpcr2;

  // Set the Capture Compare Register value (Pulse)
  if (Channel == TIM_CHANNEL_1)      { TIMx->CCR1 = OC_Config->Pulse; }
  else if (Channel == TIM_CHANNEL_2) { TIMx->CCR2 = OC_Config->Pulse; }
  else if (Channel == TIM_CHANNEL_3) { TIMx->CCR3 = OC_Config->Pulse; }
  else if (Channel == TIM_CHANNEL_4) { TIMx->CCR4 = OC_Config->Pulse; }

  // Write to TIMx CCER
  TIMx->CCER = tmpccer;
}


/**
  * @brief Configures GPIO pin for Alternate Function Output (Push-Pull, High Speed).
  * @param GPIOx Pointer to the GPIO peripheral (e.g., GPIOA).
  * @param pin_idx The pin number (0-15).
  * @param af_num The Alternate Function number (0-15).
  * @retval None
  */
static void GPIO_Config_AF_Output(GPIO_TypeDef *GPIOx, uint32_t pin_idx, uint32_t af_num)
{
    // Set pin mode to Alternate Function (10)
    GPIOx->MODER &= ~(3UL << (pin_idx * 2)); // Clear current mode bits
    GPIOx->MODER |= (GPIO_MODE_AF << (pin_idx * 2));

    // Set output type to Push-Pull (0)
    GPIOx->OTYPER &= ~(1UL << pin_idx); // Clear for Push-Pull

    // Set output speed to High (10)
    GPIOx->OSPEEDR &= ~(3UL << (pin_idx * 2)); // Clear current speed bits
    GPIOx->OSPEEDR |= (GPIO_SPEED_HIGH << (pin_idx * 2));

    // Set pull-up/pull-down to No Pull (00)
    GPIOx->PUPDR &= ~(3UL << (pin_idx * 2)); // Clear current pull bits

    // Set Alternate Function
    if (pin_idx < 8) { // For AFRL (pins 0-7)
        GPIOx->AFR[0] &= ~(0xFUL << (pin_idx * 4)); // Clear AF bits
        GPIOx->AFR[0] |= (af_num << (pin_idx * 4));
    } else { // For AFRH (pins 8-15)
        GPIOx->AFR[1] &= ~(0xFUL << ((pin_idx - 8) * 4)); // Clear AF bits
        GPIOx->AFR[1] |= (af_num << ((pin_idx - 8) * 4));
    }
}

/**
  * @brief Configures GPIO pin to Input mode with No Pull-up/down.
  * @param GPIOx Pointer to the GPIO peripheral (e.g., GPIOA).
  * @param pin_idx The pin number (0-15).
  * @retval None
  */
static void GPIO_Config_Input_NoPull(GPIO_TypeDef *GPIOx, uint32_t pin_idx)
{
    // Set pin mode to Input (00)
    GPIOx->MODER &= ~(3UL << (pin_idx * 2)); // Clear bits for input mode

    // Set pull-up/pull-down to No Pull (00)
    GPIOx->PUPDR &= ~(3UL << (pin_idx * 2)); // Clear bits for no pull
}


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes the PWM for a specific channel.
  * @param  TRD_Channel The logical PWM channel to initialize.
  * @retval None
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    GPIO_TypeDef *PortName = config->PortName;
    uint32_t PinNumber = config->PinNumber;
    uint32_t AFNumber = config->AFNumber;
    uint32_t ChannelNumber = config->ChannelNumber;

    // 1. Enable peripheral clocks (TIMx and GPIOx)
    // Check if TIMx is on APB2 or APB1
    if ((uint32_t)TIMx >= APB2PERIPH_BASE) { 
        RCC->APB2ENR |= config->RCC_TIM_EN_BIT;
    } else { // APB1 timer
        RCC->APB1ENR |= config->RCC_TIM_EN_BIT;
    }
    RCC->AHB1ENR |= config->RCC_GPIO_EN_BIT;

    // 2. Configure GPIO pin for Alternate Function PWM output
    GPIO_Config_AF_Output(PortName, PinNumber, AFNumber);

    // 3. Configure TIM Base (Prescaler and Period)
    // Reset TIM Control Register 1 (CR1) parameters first (CMS, DIR, CKD)
    TIMx->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR | TIM_CR1_CKD);
    // Set Auto-Reload Preload Enable (ARPE)
    TIMx->CR1 |= TIM_CR1_ARPE;

    // Set initial values (these will be overridden by PWM_Set_Freq for active use)
    uint32_t default_period = 99; // For 100 steps of duty cycle (ARR counts from 0 to ARR)
    uint32_t default_frequency = 1000; // 1 kHz
    // Calculate a default prescaler for the default frequency
    uint32_t default_prescaler = (TIM_CLOCK_FREQ / (default_frequency * (default_period + 1))) - 1;
    if (default_prescaler > 0xFFFFUL) default_prescaler = 0xFFFFUL; // Cap at max
    uint32_t default_duty_pulse = (default_period + 1) * 50 / 100; // 50% duty

    TIMx->ARR = default_period;
    TIMx->PSC = default_prescaler;

    // Generate an update event to reload the Prescaler and ARR values immediately
    TIMx->EGR = TIM_EGR_UG;

    // 4. Configure TIM PWM Channel specific settings
    Internal_TIM_OC_Config_t oc_config = {0}; // Initialize all members to 0
    oc_config.OCMode = TIM_OCMODE_PWM1;      // Standard PWM Mode 1
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH; // Active High output
    oc_config.Pulse = default_duty_pulse;    // Initial pulse value
    oc_config.OCFastMode = TIM_OCFAST_DISABLE; // Fast mode disabled by default

    // Propagate advanced timer flag for internal logic
    oc_config.IsAdvancedTimer = config->IsAdvancedTimer;
    // Set default values for complementary channels and idle states, relevant only for advanced timers
    oc_config.OCNPolarity = TIM_OCN_POLARITY_HIGH;
    oc_config.OCIdleState = TIM_OCIDLESTATE_LOW;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_LOW;

    TIM_OC_SetConfig_BareMetal(TIMx, ChannelNumber, &oc_config);

    // 5. If it's an advanced timer (like TIM1), enable Main Output
    if (config->IsAdvancedTimer) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }
}

/**
  * @brief  Sets the PWM frequency and duty cycle for a specific channel.
  * @param  TRD_Channel The logical PWM channel.
  * @param  frequency The desired PWM frequency in Hz.
  * @param  duty The desired duty cycle in percentage (0-100).
  * @retval None
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return; // Invalid channel
    }
    if (frequency == 0) { // If frequency is zero, stop the PWM
        PWM_Stop(TRD_Channel);
        return;
    }
    if (duty > 100) duty = 100; // Clamp duty cycle to max 100%

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;

    // Period is fixed to 99 for 100 steps duty cycle resolution (0-99 for ARR, so Period + 1 = 100)
    uint32_t period = 99; // ARR value
    uint32_t prescaler;
    uint32_t pulse;

    // Calculate prescaler: prescaler = (TIM_CLOCK_FREQ / (frequency * (period + 1))) - 1
    tlong freq_period_product = (tlong)frequency * (period + 1);
    if (freq_period_product == 0) { // Defensive check, should not happen if frequency != 0
        PWM_Stop(TRD_Channel);
        return;
    }
    prescaler = (TIM_CLOCK_FREQ / freq_period_product) - 1;

    // Max prescaler value for 16-bit timers is 0xFFFF (65535)
    if (prescaler > 0xFFFFUL) {
        prescaler = 0xFFFFUL; 
    }

    // Calculate pulse based on duty cycle: Pulse = (Period + 1) * duty / 100
    pulse = ((period + 1) * duty) / 100;
    // Ensure pulse does not exceed the (period + 1) to avoid unexpected behavior for 100% duty
    if (pulse > (period + 1)) {
        pulse = (period + 1);
    }
    
    // Update TIM registers: Prescaler and Capture Compare Register
    TIMx->PSC = prescaler;

    if (ChannelNumber == TIM_CHANNEL_1)      { TIMx->CCR1 = pulse; }
    else if (ChannelNumber == TIM_CHANNEL_2) { TIMx->CCR2 = pulse; }
    else if (ChannelNumber == TIM_CHANNEL_3) { TIMx->CCR3 = pulse; }
    else if (ChannelNumber == TIM_CHANNEL_4) { TIMx->CCR4 = pulse; }

    // Generate an update event to immediately apply the new prescaler and duty cycle.
    // This is necessary because Auto-Reload Preload (ARPE) is enabled.
    TIMx->EGR = TIM_EGR_UG;
}

/**
  * @brief  Starts the PWM signal generation for a specific channel.
  * @param  TRD_Channel The logical PWM channel to start.
  * @retval None
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;

    // Enable the Capture compare channel output
    TIM_CCxChannelCmd_BareMetal(TIMx, ChannelNumber, 1); // 1 means ENABLE

    // If it's an advanced timer (TIM1), enable the main output (MOE bit in BDTR)
    if (config->IsAdvancedTimer) {
        TIMx->BDTR |= TIM_BDTR_MOE;
    }

    // Enable the Peripheral (TIM Counter Enable bit in CR1)
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Stops the PWM signal generation for a specific channel.
  * @param  TRD_Channel The logical PWM channel to stop.
  * @retval None
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_PWM_CHANNEL_COUNT) {
        return; // Invalid channel
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    uint32_t ChannelNumber = config->ChannelNumber;

    // Disable the Capture compare channel output
    TIM_CCxChannelCmd_BareMetal(TIMx, ChannelNumber, 0); // 0 means DISABLE

    // If it's an advanced timer (TIM1), disable the main output
    if (config->IsAdvancedTimer) {
        TIMx->BDTR &= ~TIM_BDTR_MOE;
    }

    // Disable the Peripheral (TIM Counter)
    TIMx->CR1 &= ~TIM_CR1_CEN;

    // Set CCR to 0 to ensure the output is forced low immediately
    if (ChannelNumber == TIM_CHANNEL_1)      { TIMx->CCR1 = 0; }
    else if (ChannelNumber == TIM_CHANNEL_2) { TIMx->CCR2 = 0; }
    else if (ChannelNumber == TIM_CHANNEL_3) { TIMx->CCR3 = 0; }
    else if (ChannelNumber == TIM_CHANNEL_4) { TIMx->CCR4 = 0; }

    // Generate an update event to apply the CCR=0 immediately
    TIMx->EGR = TIM_EGR_UG;
}

/**
  * @brief  Powers off all PWM channels and releases associated resources.
  *         This function stops all configured PWM channels, sets their GPIO pins
  *         to input mode (safe state), and disables their respective peripheral clocks.
  * @param  None
  * @retval None
  */
void PWM_PowerOff(void)
{
    for (int i = 0; i < TRD_PWM_CHANNEL_COUNT; i++) {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];
        TIM_TypeDef *TIMx = config->TIMx;
        GPIO_TypeDef *PortName = config->PortName;
        uint32_t PinNumber = config->PinNumber;

        // 1. Stop the PWM channel gracefully (disables timer and output)
        PWM_Stop((TRD_Channel_t)i);

        // 2. De-configure GPIO pin to a safe state (Input mode, No Pull-up/down)
        GPIO_Config_Input_NoPull(PortName, PinNumber);

        // 3. Disable peripheral clocks for the TIM and GPIO modules
        // Check if TIMx is on APB2 or APB1
        if ((uint32_t)TIMx >= APB2PERIPH_BASE) { 
            RCC->APB2ENR &= ~config->RCC_TIM_EN_BIT;
        } else { // APB1 timer
            RCC->APB1ENR &= ~config->RCC_TIM_EN_BIT;
        }
        
        // Disable GPIO clock. This assumes that no other peripheral or
        // component relies on this specific GPIO port's clock.
        // For a full system power-off, this is usually acceptable.
        RCC->AHB1ENR &= ~config->RCC_GPIO_EN_BIT;
    }
}