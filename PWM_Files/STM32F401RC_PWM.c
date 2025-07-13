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

#include "STM32F401RC_PWM.h"

// ====================================================================================================================
// PRIVATE DEFINITIONS
// ====================================================================================================================

// Note: Peripheral base addresses, RCC registers/bits, and GPIO AF values are
// NOT explicitly provided in the included PDF excerpts for RM0368.
// The definitions below are ASSUMED based on standard STM32F4xx microcontroller
// memory maps and peripherals. These MUST BE VERIFIED against the specific
// STM32F401RC datasheet and full reference manual.

// Peripheral base addresses (Assumed)
#define GPIOA_BASE (0x40020000UL) /* Assumed base address for GPIOA */
#define GPIOB_BASE (0x40020400UL) /* Assumed base address for GPIOB */
#define GPIOC_BASE (0x40020800UL) /* Assumed base address for GPIOC */
#define GPIOD_BASE (0x40020C00UL) /* Assumed base address for GPIOD */
// Add other GPIO ports if needed based on pin mappings

#define TIM1_BASE   (0x40010000UL) /* Assumed base address for TIM1 */
#define TIM2_BASE   (0x40000000UL) /* Assumed base address for TIM2 */
#define TIM3_BASE   (0x40000400UL) /* Assumed base address for TIM3 */
#define TIM4_BASE   (0x40000800UL) /* Assumed base address for TIM4 */
#define TIM5_BASE   (0x40000C00UL) /* Assumed base address for TIM5 */
#define TIM9_BASE   (0x40014000UL) /* Assumed base address for TIM9 */
#define TIM10_BASE  (0x40014400UL) /* Assumed base address for TIM10 */
#define TIM11_BASE  (0x40014800UL) /* Assumed base address for TIM11 */

// RCC base address and register offsets/bits (Assumed)
#define RCC_BASE             (0x40023800UL) /* Assumed base address for RCC */
#define RCC_AHB1ENR_OFFSET   (0x30UL)       /* Assumed offset for AHB1 Enable Register */
#define RCC_APB1ENR_OFFSET   (0x3ACL)       /* Assumed offset for APB1 Enable Register */
#define RCC_APB2ENR_OFFSET   (0x34UL)       /* Assumed offset for APB2 Enable Register */

#define RCC_AHB1ENR (*((volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET))) /* Assumed RCC AHB1 Enable Register */
#define RCC_APB1ENR (*((volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET))) /* Assumed RCC APB1 Enable Register */
#define RCC_APB2ENR (*((volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET))) /* Assumed RCC APB2 Enable Register */

// RCC GPIO Enable bits (Assumed)
#define RCC_AHB1ENR_GPIOAEN (1U << 0U) /* Assumed RCC GPIOA Enable Bit */
#define RCC_AHB1ENR_GPIOBEN (1U << 1U) /* Assumed RCC GPIOB Enable Bit */
#define RCC_AHB1ENR_GPIOCEN (1U << 2U) /* Assumed RCC GPIOC Enable Bit */
#define RCC_AHB1ENR_GPIODEN (1U << 3U) /* Assumed RCC GPIOD Enable Bit */
// Add other GPIO enable bits if ports are used

// RCC TIM Enable bits (Assumed)
#define RCC_APB2ENR_TIM1EN  (1U << 0U)  /* Assumed RCC TIM1 Enable Bit (APB2) */
#define RCC_APB1ENR_TIM2EN  (1U << 0U)  /* Assumed RCC TIM2 Enable Bit (APB1) */
#define RCC_APB1ENR_TIM3EN  (1U << 1U)  /* Assumed RCC TIM3 Enable Bit (APB1) */
#define RCC_APB1ENR_TIM4EN  (1U << 2U)  /* Assumed RCC TIM4 Enable Bit (APB1) */
#define RCC_APB1ENR_TIM5EN  (1U << 3U)  /* Assumed RCC TIM5 Enable Bit (APB1) */
#define RCC_APB2ENR_TIM9EN  (1U << 16U) /* Assumed RCC TIM9 Enable Bit (APB2) */
#define RCC_APB2ENR_TIM10EN (1U << 17U) /* Assumed RCC TIM10 Enable Bit (APB2) */
#define RCC_APB2ENR_TIM11EN (1U << 18U) /* Assumed RCC TIM11 Enable Bit (APB2) */

// Define Timer Clock Frequencies - NOT INCLUDED IN PDF TEXT, ASSUMED MAX SPEED
// Based on typical F401RC clock tree where timer clock is 2 * APB clock if APB prescaler > 1.
// Assuming SYSCLK = 84MHz, APB1 Prescaler > 1 (APB1 clock = 42MHz), APB2 Prescaler > 1 (APB2 clock = 84MHz).
// Timer clock is then 2 * APB clock = 84MHz for both APB1 and APB2 timers in this scenario.
#define TIM_CLOCK_APB1 (84000000UL) /* Assumed Timer clock frequency for TIM2-5 */
#define TIM_CLOCK_APB2 (84000000UL) /* Assumed Timer clock frequency for TIM1, TIM9-11 */


// Structs for register access - offsets based on PDF RM0368 sections
// GPIO struct definition - NOT INCLUDED IN PDF TEXT, created from register offsets
typedef struct
{
  volatile uint32_t MODER;    /* GPIO port mode register,               Address offset: 0x00 */ /* PDF Reference */
  volatile uint32_t OTYPER;   /* GPIO port output type register,        Address offset: 0x04 */ /* PDF Reference */
  volatile uint32_t OSPEEDR;  /* GPIO port output speed register,       Address offset: 0x08 */ /* PDF Reference */
  volatile uint32_t PUPDR;    /* GPIO port pull-up/pull-down register,  Address offset: 0x0C */ /* PDF Reference */
  volatile uint32_t IDR;      /* GPIO port input data register,         Address offset: 0x10 */ /* PDF Reference */
  volatile uint32_t ODR;      /* GPIO port output data register,        Address offset: 0x14 */ /* PDF Reference */
  volatile uint32_t BSRR;     /* GPIO port bit set/reset register,      Address offset: 0x18 */ /* PDF Reference */
  volatile uint32_t LCKR;     /* GPIO port configuration lock register, Address offset: 0x1C */ /* PDF Reference */
  volatile uint32_t AFRL;     /* GPIO alternate function low register,  Address offset: 0x20 */ /* PDF Reference */
  volatile uint32_t AFRH;     /* GPIO alternate function high register, Address offset: 0x24 */ /* PDF Reference */
} GPIO_TypeDef;

// TIM struct definition - NOT INCLUDED IN PDF TEXT, created from register offsets
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1,              Address offset: 0x00 */ /* PDF Reference */
  volatile uint32_t CR2;      /* TIM control register 2,              Address offset: 0x04 */ /* PDF Reference */
  volatile uint32_t SMCR;     /* TIM slave mode control register,     Address offset: 0x08 */ /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register,   Address offset: 0x0C */ /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register,                 Address offset: 0x10 */ /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register,       Address offset: 0x14 */ /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, Address offset: 0x18 */ /* PDF Reference */
  volatile uint32_t CCMR2;    /* TIM capture/compare mode register 2, Address offset: 0x1C */ // Not on TIM9/10/11
  volatile uint32_t CCER;     /* TIM capture/compare enable register, Address offset: 0x20 */ /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter,                         Address offset: 0x24 */ /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler,                       Address offset: 0x28 */ /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register,            Address offset: 0x2C */ /* PDF Reference */
  volatile uint32_t RCR;      /* TIM repetition counter register,     Address offset: 0x30 */ // TIM1 only /* PDF Reference */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1,      Address offset: 0x34 */ /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2,      Address offset: 0x38 */ /* PDF Reference */
  volatile uint32_t CCR3;     /* TIM capture/compare register 3,      Address offset: 0x3C */ // Not on TIM9/10/11 /* PDF Reference */
  volatile uint32_t CCR4;     /* TIM capture/compare register 4,      Address offset: 0x40 */ // Not on TIM9/10/11 /* PDF Reference */
  volatile uint32_t BDTR;     /* TIM break and dead-time register,    Address offset: 0x44 */ // TIM1 only /* PDF Reference */
  volatile uint32_t DCR;      /* TIM DMA control register,            Address offset: 0x48 */ /* PDF Reference */
  volatile uint32_t DMAR;     /* TIM DMA address for full transfer,   Address offset: 0x4C */ /* PDF Reference */
  volatile uint32_t OR;       /* TIM option register,                 Address offset: 0x50 */ // TIM2, TIM5, TIM11 only /* PDF Reference */
} TIM_TypeDef;


// Define Timer instances based on assumed base addresses
#define TIM1 ((TIM_TypeDef *) TIM1_BASE)
#define TIM2 ((TIM_TypeDef *) TIM2_BASE)
#define TIM3 ((TIM_TypeDef *) TIM3_BASE)
#define TIM4 ((TIM_TypeDef *) TIM4_BASE)
// TIM5 is reserved for OS/delay functions - Do NOT define for PWM use
#define TIM9 ((TIM_TypeDef *) TIM9_BASE)
#define TIM10 ((TIM_TypeDef *) TIM10_BASE)
// TIM11 is reserved for OS/delay functions - Do NOT define for PWM use

// Define GPIO Port instances based on assumed base addresses
#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *) GPIOD_BASE)
// Add other GPIO ports if needed


// Bit masks and positions based on PDF RM0368 descriptions (for used bits)
// GPIO
#define GPIO_MODER_MODEy_Pos(y) (2U * (y)) /* PDF Reference */
#define GPIO_MODER_MODEy_Msk(y) (0x3U << GPIO_MODER_MODEy_Pos(y))
#define GPIO_MODER_AF_MODE (2U) /* 10: Alternate function mode */ /* PDF Reference */
#define GPIO_MODER_INPUT_MODE (0U) /* 00: Input (reset state) */ /* PDF Reference */

#define GPIO_OTYPER_OTy_Pos(y) (y) /* PDF Reference */
#define GPIO_OTYPER_OTy_Msk(y) (0x1U << GPIO_OTYPER_OTy_Pos(y))
#define GPIO_OTYPER_PP_MODE (0U) /* 0: Output push-pull */ /* PDF Reference */

#define GPIO_OSPEEDR_OSPEEDRy_Pos(y) (2U * (y)) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDRy_Msk(y) (0x3U << GPIO_OSPEEDR_OSPEEDRy_Pos(y))
#define GPIO_OSPEEDR_HIGH_SPEED (2U) /* 10: High speed */ /* PDF Reference */

#define GPIO_PUPDR_PUPDRy_Pos(y) (2U * (y)) /* PDF Reference */
#define GPIO_PUPDR_PUPDRy_Msk(y) (0x3U << GPIO_PUPDR_PUPDRy_Pos(y))
#define GPIO_PUPDR_NO_PUPD (0U) /* 00: No pull-up, pull-down */ /* PDF Reference */

#define GPIO_AFRL_AFRLy_Pos(y) (4U * (y)) // y = 0..7 /* PDF Reference */
#define GPIO_AFRL_AFRLy_Msk(y) (0xFU << GPIO_AFRL_AFRLy_Pos(y))

#define GPIO_AFRH_AFRHy_Pos(y) (4U * ((y) - 8U)) // y = 8..15 /* PDF Reference */
#define GPIO_AFRH_AFRHy_Msk(y) (0xFU << GPIO_AFRH_AFRHy_Pos(y))


// TIM CR1
#define TIM_CR1_CEN_Pos   (0U)  /* PDF Reference */
#define TIM_CR1_CEN_Msk   (0x1U << TIM_CR1_CEN_Pos) /*!< CEN */
#define TIM_CR1_CEN       TIM_CR1_CEN_Msk
#define TIM_CR1_UDIS_Pos  (1U)  /* PDF Reference */
#define TIM_CR1_UDIS_Msk  (0x1U << TIM_CR1_UDIS_Pos) /*!< UDIS */
#define TIM_CR1_UDIS      TIM_CR1_UDIS_Msk
#define TIM_CR1_URS_Pos   (2U)  /* PDF Reference */
#define TIM_CR1_URS_Msk   (0x1U << TIM_CR1_URS_Pos) /*!< URS */
#define TIM_CR1_URS       TIM_CR1_URS_Msk
#define TIM_CR1_OPM_Pos   (3U)  /* PDF Reference */
#define TIM_CR1_OPM_Msk   (0x1U << TIM_CR1_OPM_Pos) /*!< OPM */
#define TIM_CR1_OPM       TIM_CR1_OPM_Msk
#define TIM_CR1_DIR_Pos   (4U)  /* PDF Reference */
#define TIM_CR1_DIR_Msk   (0x1U << TIM_CR1_DIR_Pos) /*!< DIR */
#define TIM_CR1_DIR       TIM_CR1_DIR_Msk
#define TIM_CR1_CMS_Pos   (5U)  /* PDF Reference */
#define TIM_CR1_CMS_Msk   (0x3U << TIM_CR1_CMS_Pos) /*!< CMS[1:0] */
#define TIM_CR1_CMS       TIM_CR1_CMS_Msk
#define TIM_CR1_ARPE_Pos  (7U)  /* PDF Reference */
#define TIM_CR1_ARPE_Msk  (0x1U << TIM_CR1_ARPE_Pos) /*!< ARPE */
#define TIM_CR1_ARPE      TIM_CR1_ARPE_Msk
#define TIM_CR1_CKD_Pos   (8U)  /* PDF Reference */
#define TIM_CR1_CKD_Msk   (0x3U << TIM_CR1_CKD_Pos) /*!< CKD[1:0] */
#define TIM_CR1_CKD       TIM_CR1_CKD_Msk

// TIM BDTR (TIM1 only)
#define TIM_BDTR_MOE_Pos    (15U) /* PDF Reference */
#define TIM_BDTR_MOE_Msk    (0x1U << TIM_BDTR_MOE_Pos) /*!< MOE */
#define TIM_BDTR_MOE        TIM_BDTR_MOE_Msk

// TIM EGR
#define TIM_EGR_UG_Pos    (0U) /* PDF Reference */
#define TIM_EGR_UG_Msk    (0x1U << TIM_EGR_UG_Pos) /*!< UG */
#define TIM_EGR_UG        TIM_EGR_UG_Msk

// TIM CCMR1 / CCMR2 (Output Compare mode)
// Note: These registers are union-like in function based on CCxS bits,
//       but accessed as 32-bit words per GPIO section recommendation.
//       Bit definitions are specific to the channel number (1, 2, 3, 4).

// CCMR1 bits (Channels 1 and 2)
#define TIM_CCMR1_CC1S_Pos    (0U) /* PDF Reference */
#define TIM_CCMR1_OC1PE_Pos   (3U) /* PDF Reference */
#define TIM_CCMR1_OC1PE_Msk   (0x1U << TIM_CCMR1_OC1PE_Pos) /*!< OC1PE */
#define TIM_CCMR1_OC1PE       TIM_CCMR1_OC1PE_Msk
#define TIM_CCMR1_OC1M_Pos    (4U) /* PDF Reference */
#define TIM_CCMR1_OC1M_Msk    (0x7U << TIM_CCMR1_OC1M_Pos) /*!< OC1M[2:0] */
#define TIM_CCMR1_OC1M_PWM1   (0x6U << TIM_CCMR1_OC1M_Pos) /* 110: PWM mode 1 */ /* PDF Reference */

#define TIM_CCMR1_CC2S_Pos    (8U) /* PDF Reference */
#define TIM_CCMR1_OC2PE_Pos   (11U) /* PDF Reference */
#define TIM_CCMR1_OC2PE_Msk   (0x1U << TIM_CCMR1_OC2PE_Pos) /*!< OC2PE */
#define TIM_CCMR1_OC2PE       TIM_CCMR1_OC2PE_Msk
#define TIM_CCMR1_OC2M_Pos    (12U) /* PDF Reference */
#define TIM_CCMR1_OC2M_Msk    (0x7U << TIM_CCMR1_OC2M_Pos) /*!< OC2M[2:0] */
#define TIM_CCMR1_OC2M_PWM1   (0x6U << TIM_CCMR1_OC2M_Pos) /* 110: PWM mode 1 */ /* PDF Reference */

// CCMR2 bits (Channels 3 and 4) - Only for TIM1, TIM2, TIM3, TIM4
#define TIM_CCMR2_CC3S_Pos    (0U) /* PDF Reference */
#define TIM_CCMR2_OC3PE_Pos   (3U) /* PDF Reference */
#define TIM_CCMR2_OC3PE_Msk   (0x1U << TIM_CCMR2_OC3PE_Pos) /*!< OC3PE */
#define TIM_CCMR2_OC3PE       TIM_CCMR2_OC3PE_Msk
#define TIM_CCMR2_OC3M_Pos    (4U) /* PDF Reference */
#define TIM_CCMR2_OC3M_Msk    (0x7U << TIM_CCMR2_OC3M_Pos) /*!< OC3M[2:0] */
#define TIM_CCMR2_OC3M_PWM1   (0x6U << TIM_CCMR2_OC3M_Pos) /* 110: PWM mode 1 */ /* PDF Reference */

#define TIM_CCMR2_CC4S_Pos    (8U) /* PDF Reference */
#define TIM_CCMR2_OC4PE_Pos   (11U) /* PDF Reference */
#define TIM_CCMR2_OC4PE_Msk   (0x1U << TIM_CCMR2_OC4PE_Pos) /*!< OC4PE */
#define TIM_CCMR2_OC4PE       TIM_CCMR2_OC4PE_Msk
#define TIM_CCMR2_OC4M_Pos    (12U) /* PDF Reference */
#define TIM_CCMR2_OC4M_Msk    (0x7U << TIM_CCMR2_OC4M_Pos) /*!< OC4M[2:0] */
#define TIM_CCMR2_OC4M_PWM1   (0x6U << TIM_CCMR2_OC4M_Pos) /* 110: PWM mode 1 */ /* PDF Reference */

// TIM CCER
#define TIM_CCER_CC1E_Pos   (0U) /* PDF Reference */
#define TIM_CCER_CC1E_Msk   (0x1U << TIM_CCER_CC1E_Pos) /*!< CC1E */
#define TIM_CCER_CC1E       TIM_CCER_CC1E_Msk
#define TIM_CCER_CC1P_Pos   (1U) /* PDF Reference */
#define TIM_CCER_CC1P_Msk   (0x1U << TIM_CCER_CC1P_Pos) /*!< CC1P */
#define TIM_CCER_CC1P       TIM_CCER_CC1P_Msk // Polarity bit

#define TIM_CCER_CC2E_Pos   (4U) /* PDF Reference */
#define TIM_CCER_CC2E_Msk   (0x1U << TIM_CCER_CC2E_Pos) /*!< CC2E */
#define TIM_CCER_CC2E       TIM_CCER_CC2E_Msk
#define TIM_CCER_CC2P_Pos   (5U) /* PDF Reference */
#define TIM_CCER_CC2P_Msk   (0x1U << TIM_CCER_CC2P_Pos) /*!< CC2P */
#define TIM_CCER_CC2P       TIM_CCER_CC2P_Msk

#define TIM_CCER_CC3E_Pos   (8U) /* PDF Reference */
#define TIM_CCER_CC3E_Msk   (0x1U << TIM_CCER_CC3E_Pos) /*!< CC3E */
#define TIM_CCER_CC3E       TIM_CCER_CC3E_Msk
#define TIM_CCER_CC3P_Pos   (9U) /* PDF Reference */
#define TIM_CCER_CC3P_Msk   (0x1U << TIM_CCER_CC3P_Pos) /*!< CC3P */
#define TIM_CCER_CC3P       TIM_CCER_CC3P_Msk

#define TIM_CCER_CC4E_Pos   (12U) /* PDF Reference */
#define TIM_CCER_CC4E_Msk   (0x1U << TIM_CCER_CC4E_Pos) /*!< CC4E */
#define TIM_CCER_CC4E       TIM_CCER_CC4E_Msk
#define TIM_CCER_CC4P_Pos   (13U) /* PDF Reference */
#define TIM_CCER_CC4P_Msk   (0x1U << TIM_CCER_CC4P_Pos) /*!< CC4P */
#define TIM_CCER_CC4P       TIM_CCER_CC4P_Msk

// ====================================================================================================================
// PRIVATE DATA
// ====================================================================================================================

// Structure to map TRD_Channel_t to Timer, Channel, and GPIO
// GPIO and AF mappings are placeholders based on common STM32F4 practices
// AND REQUIRE VERIFICATION against the specific STM32F401RC datasheet Alternate Function table.
// Pins starting from 1 are preferred where multiple options exist, and pin 0 is avoided.
typedef struct {
    TIM_TypeDef *TIM;           /* Pointer to the TIM peripheral */
    uint32_t TIM_CLK_EN_BIT;    /* RCC clock enable bit for the TIM */
    volatile uint32_t *TIM_CLK_EN_REG; /* Pointer to the RCC clock enable register (APB1 or APB2) */
    uint8_t Channel;            /* Timer channel number (1, 2, 3, or 4) */
    GPIO_TypeDef *GPIO_Port;    /* Pointer to the GPIO peripheral */
    uint32_t GPIO_CLK_EN_BIT;   /* RCC clock enable bit for the GPIO port */
    uint8_t GPIO_Pin;           /* GPIO pin number (0-15) */
    uint8_t GPIO_AF;            /* GPIO Alternate Function number */ // Placeholder - need datasheet AF table

} PWM_Channel_Map_t;

// Mapping array from TRD_Channel_t to hardware resources
// WARNING: GPIO Port, Pin, and AF values are ASSUMED based on common STM32F4 mappings
//          and MUST BE VERIFIED against the STM32F401RC datasheet Alternate Function table.
// Pin 0 is avoided based on requirements unless explicitly necessary and verified via datasheet.
//
// Timers TIM5 and TIM11 are reserved for potential OS/delay functions as per requirement.
// These reserved timers/channels are excluded from this PWM implementation.
static const PWM_Channel_Map_t pwm_channel_map[TRD_CHANNEL_COUNT] = {
    // TIM1 channels (APB2 timer, 16-bit counter, Advanced Control)
    { TIM1, RCC_APB2ENR_TIM1EN, &RCC_APB2ENR, 1, GPIOA, RCC_AHB1ENR_GPIOAEN, 8,  1 }, /* Assumed PWM config - please verify TIM1_CH1 -> PA8, AF1 */
    { TIM1, RCC_APB2ENR_TIM1EN, &RCC_APB2ENR, 2, GPIOA, RCC_AHB1ENR_GPIOAEN, 9,  1 }, /* Assumed PWM config - please verify TIM1_CH2 -> PA9, AF1 */
    { TIM1, RCC_APB2ENR_TIM1EN, &RCC_APB2ENR, 3, GPIOA, RCC_AHB1ENR_GPIOAEN, 10, 1 }, /* Assumed PWM config - please verify TIM1_CH3 -> PA10, AF1 */
    { TIM1, RCC_APB2ENR_TIM1EN, &RCC_APB2ENR, 4, GPIOA, RCC_AHB1ENR_GPIOAEN, 11, 1 }, /* Assumed PWM config - please verify TIM1_CH4 -> PA11, AF1 */

    // TIM2 channels (APB1 timer, 32-bit counter, General Purpose)
    { TIM2, RCC_APB1ENR_TIM2EN, &RCC_APB1ENR, 1, GPIOA, RCC_AHB1ENR_GPIOAEN, 1,  1 }, /* Assumed PWM config - please verify TIM2_CH1 -> PA1, AF1 */
    { TIM2, RCC_APB1ENR_TIM2EN, &RCC_APB1ENR, 2, GPIOB, RCC_AHB1ENR_GPIOBEN, 3,  1 }, /* Assumed PWM config - please verify TIM2_CH2 -> PB3, AF1 */
    { TIM2, RCC_APB1ENR_TIM2EN, &RCC_APB1ENR, 3, GPIOB, RCC_AHB1ENR_GPIOBEN, 10, 1 }, /* Assumed PWM config - please verify TIM2_CH3 -> PB10, AF1 */
    { TIM2, RCC_APB1ENR_TIM2EN, &RCC_APB1ENR, 4, GPIOB, RCC_AHB1ENR_GPIOBEN, 11, 1 }, /* Assumed PWM config - please verify TIM2_CH4 -> PB11, AF1 */

    // TIM3 channels (APB1 timer, 16-bit counter, General Purpose)
    { TIM3, RCC_APB1ENR_TIM3EN, &RCC_APB1ENR, 1, GPIOA, RCC_AHB1ENR_GPIOAEN, 6,  2 }, /* Assumed PWM config - please verify TIM3_CH1 -> PA6, AF2 */
    { TIM3, RCC_APB1ENR_TIM3EN, &RCC_APB1ENR, 2, GPIOA, RCC_AHB1ENR_GPIOAEN, 7,  2 }, /* Assumed PWM config - please verify TIM3_CH2 -> PA7, AF2 */
    { TIM3, RCC_APB1ENR_TIM3EN, &RCC_APB1ENR, 3, GPIOC, RCC_AHB1ENR_GPIOCEN, 8,  2 }, /* Assumed PWM config - please verify TIM3_CH3 -> PC8, AF2 */
    { TIM3, RCC_APB1ENR_TIM3EN, &RCC_APB1ENR, 4, GPIOC, RCC_AHB1ENR_GPIOCEN, 9,  2 }, /* Assumed PWM config - please verify TIM3_CH4 -> PC9, AF2 */

    // TIM4 channels (APB1 timer, 16-bit counter, General Purpose)
    { TIM4, RCC_APB1ENR_TIM4EN, &RCC_APB1ENR, 1, GPIOB, RCC_AHB1ENR_GPIOBEN, 6,  2 }, /* Assumed PWM config - please verify TIM4_CH1 -> PB6, AF2 */
    { TIM4, RCC_APB1ENR_TIM4EN, &RCC_APB1ENR, 2, GPIOB, RCC_AHB1ENR_GPIOBEN, 7,  2 }, /* Assumed PWM config - please verify TIM4_CH2 -> PB7, AF2 */
    { TIM4, RCC_APB1ENR_TIM4EN, &RCC_APB1ENR, 3, GPIOD, RCC_AHB1ENR_GPIODEN, 14, 2 }, /* Assumed PWM config - please verify TIM4_CH3 -> PD14, AF2 */
    { TIM4, RCC_APB1ENR_TIM4EN, &RCC_APB1ENR, 4, GPIOD, RCC_AHB1ENR_GPIODEN, 15, 2 }, /* Assumed PWM config - please verify TIM4_CH4 -> PD15, AF2 */

    // TIM9 channels (APB2 timer, 16-bit counter, General Purpose)
    { TIM9, RCC_APB2ENR_TIM9EN, &RCC_APB2ENR, 1, GPIOA, RCC_AHB1ENR_GPIOAEN, 2,  3 }, /* Assumed PWM config - please verify TIM9_CH1 -> PA2, AF3 */
    { TIM9, RCC_APB2ENR_TIM9EN, &RCC_APB2ENR, 2, GPIOA, RCC_AHB1ENR_GPIOAEN, 3,  3 }, /* Assumed PWM config - please verify TIM9_CH2 -> PA3, AF3 */

    // TIM10 channels (APB2 timer, 16-bit counter, General Purpose)
    { TIM10, RCC_APB2ENR_TIM10EN, &RCC_APB2ENR, 1, GPIOB, RCC_AHB1ENR_GPIOBEN, 8, 3 }  /* Assumed PWM config - please verify TIM10_CH1 -> PB8, AF3 */
};

// Flag to track which channels have been initialized
static uint8_t initialized_channels[TRD_CHANNEL_COUNT] = {0};

// Helper function to get timer clock frequency
static uint32_t get_timer_clock_freq(TIM_TypeDef *TIMx) {
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10) {
        return TIM_CLOCK_APB2; /* Assumed Timer clock frequency for TIM1, TIM9-11 */
    } else if (TIMx == TIM2 || TIMx == TIM3 || TIMx == TIM4) {
        return TIM_CLOCK_APB1; /* Assumed Timer clock frequency for TIM2-5 */
    }
    // Reserved timers or invalid instance
    return 0;
}

// Helper function to get the maximum ARR value for a timer
static uint32_t get_timer_max_arr(TIM_TypeDef *TIMx) {
     if (TIMx == TIM2) { // TIM2 is 32-bit
        return 0xFFFFFFFFUL; /* PDF Reference for TIM2 ARR size */
    } else { // TIM1, TIM3, TIM4, TIM9, TIM10 are 16-bit
        return 0xFFFFUL; /* PDF Reference for 16-bit TIM ARR size */
    }
}

// Helper function to enable/disable GPIO clock
static void set_gpio_clock(GPIO_TypeDef *GPIOx, uint8_t enable) {
    if (GPIOx == GPIOA) {
        if (enable) RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Assumed RCC GPIOA Enable Bit */
        else RCC_AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
    } else if (GPIOx == GPIOB) {
        if (enable) RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Assumed RCC GPIOB Enable Bit */
        else RCC_AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
    } else if (GPIOx == GPIOC) {
         if (enable) RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* Assumed RCC GPIOC Enable Bit */
        else RCC_AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
    } else if (GPIOx == GPIOD) {
        if (enable) RCC_AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* Assumed RCC GPIOD Enable Bit */
        else RCC_AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
    }
    // Add other ports if used in pwm_channel_map
}

// ====================================================================================================================
// /**Functions ===========================================================================*/
// ====================================================================================================================

/***********************************************************************************************************************
* Function Name: PWM_Init
* Description  : Initialize the PWM hardware and configure the timer and GPIOs for the given channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to initialize.
* Return Value : None
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Map_t *channel_map = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_map->TIM;
    GPIO_TypeDef *GPIOx = channel_map->GPIO_Port;
    uint8_t pin = channel_map->GPIO_Pin;
    uint8_t channel_num = channel_map->Channel; // 1-based channel number

    // 1. Enable GPIO clock
    set_gpio_clock(GPIOx, 1);

    // 2. Configure GPIO pin for Alternate Function
    // Clear MODE bits for the pin
    GPIOx->MODER &= ~(GPIO_MODER_MODEy_Msk(pin)); /* PDF Reference: MODER */
    // Set MODE bits to Alternate Function (10)
    GPIOx->MODER |= GPIO_MODER_MODEy(pin, GPIO_MODER_AF_MODE); /* PDF Reference: MODER */

    // 3. Select the Alternate Function for the pin
    // Note: AF mapping values are placeholders, VERIFY with datasheet
    if (pin < 8) {
        // Configure AFRL for pins 0-7
        GPIOx->AFRL &= ~(GPIO_AFRL_AFRLy_Msk(pin)); /* PDF Reference: AFRL */
        GPIOx->AFRL |= GPIO_AFRL_AFRLy(pin, channel_map->GPIO_AF); /* PDF Reference: AFRL */
        /* Assumed PWM config - please verify AF value */
    } else {
        // Configure AFRH for pins 8-15
        GPIOx->AFRH &= ~(GPIO_AFRH_AFRHy_Msk(pin)); /* PDF Reference: AFRH */
        GPIOx->AFRH |= GPIO_AFRH_AFRHy(pin, channel_map->GPIO_AF); /* PDF Reference: AFRH */
        /* Assumed PWM config - please verify AF value */
    }

    // 4. Configure GPIO output type as Push-Pull (0)
    GPIOx->OTYPER &= ~(GPIO_OTYPER_OTy_Msk(pin)); /* PDF Reference: OTYPER */
    GPIOx->OTYPER |= GPIO_OTYPER_OTy(pin, GPIO_OTYPER_PP_MODE); /* PDF Reference: OTYPER */

    // 5. Configure GPIO output speed as High speed (10)
    GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDRy_Msk(pin)); /* PDF Reference: OSPEEDR */
    GPIOx->OSPEEDR |= GPIO_OSPEEDR_OSPEEDRy(pin, GPIO_OSPEEDR_HIGH_SPEED); /* PDF Reference: OSPEEDR */

    // 6. Configure GPIO pull-up/pull-down as No PU/PD (00)
    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk(pin)); /* PDF Reference: PUPDR */
    GPIOx->PUPDR |= GPIO_PUPDR_PUPDRy(pin, GPIO_PUPDR_NO_PUPD); /* PDF Reference: PUPDR */

    // 7. Enable Timer clock
    *(channel_map->TIM_CLK_EN_REG) |= channel_map->TIM_CLK_EN_BIT; /* Assumed RCC TIM Enable Bit */

    // Delay to allow timer clock to stabilize (optional but good practice)
    // A simple read back can suffice
    (void)*(channel_map->TIM_CLK_EN_REG);

    // 8. Stop the timer
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: CR1 */

    // 9. Configure Time-Base unit (Prescaler and Auto-Reload)
    // Default to a common frequency and duty cycle during init
    // The user can call PWM_Set_Freq after init to set the desired values.
    tlong default_freq = 1000; // 1 kHz
    tbyte default_duty = 50;   // 50%

    uint32_t timer_clock = get_timer_clock_freq(TIMx); /* Assumed Timer clock frequency */
    uint32_t max_arr = get_timer_max_arr(TIMx);

    uint16_t prescaler = 0;
    uint32_t arr = 0;

    // Calculate initial PSC and ARR
    calculate_timer_params(timer_clock, default_freq, &prescaler, &arr, max_arr);

    // Set Prescaler and ARR
    TIMx->PSC = (uint16_t)prescaler; /* PDF Reference: PSC */
    TIMx->ARR = arr;                 /* PDF Reference: ARR */

    // Configure timer mode: Upcounting, Edge-aligned
    TIMx->CR1 &= ~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk); /* PDF Reference: CR1 */ // Clear DIR and CMS
    TIMx->CR1 |= TIM_CR1_ARPE; // Enable ARR preload buffer /* PDF Reference: CR1 */
    TIMx->CR1 |= TIM_CR1_URS;  // Only counter overflow/underflow generates update /* PDF Reference: CR1 */

    // 10. Configure the Capture/Compare channel for PWM mode
    uint32_t duty_counts = (arr + 1) * default_duty / 100; // Calculate initial CCR value

    // Clear and set channel mode bits (CCxS) to Output (00) and PWM Mode 1 (110)
    // Enable output preload register (OCxPE)
    // Channels 1 and 2 use CCMR1, Channels 3 and 4 use CCMR2
    volatile uint32_t *ccmr_reg;
    uint32_t ccmr_clear_mask_s;
    uint32_t ccmr_clear_mask_m;
    uint32_t ccmr_set_mask_m;
    uint32_t ccmr_set_mask_pe;
    volatile uint32_t *ccr_reg;
    uint32_t ccer_enable_mask;
    uint32_t ccer_polarity_mask;


    switch (channel_num) {
        case 1:
            ccmr_reg = &TIMx->CCMR1; /* PDF Reference: CCMR1 */
            ccmr_clear_mask_s = TIM_CCMR1_CC1S_Msk;
            ccmr_clear_mask_m = TIM_CCMR1_OC1M_Msk;
            ccmr_set_mask_m   = TIM_CCMR1_OC1M_PWM1; /* PDF Reference: OC1M_PWM1 */
            ccmr_set_mask_pe  = TIM_CCMR1_OC1PE; /* PDF Reference: OC1PE */
            ccer_enable_mask  = TIM_CCER_CC1E; /* PDF Reference: CC1E */
            ccer_polarity_mask= TIM_CCER_CC1P; /* PDF Reference: CC1P */
            ccr_reg = &TIMx->CCR1; /* PDF Reference: CCR1 */
            break;
        case 2:
            ccmr_reg = &TIMx->CCMR1; /* PDF Reference: CCMR1 */
            ccmr_clear_mask_s = TIM_CCMR1_CC2S_Msk;
            ccmr_clear_mask_m = TIM_CCMR1_OC2M_Msk;
            ccmr_set_mask_m   = TIM_CCMR1_OC2M_PWM1; /* PDF Reference: OC2M_PWM1 */
            ccmr_set_mask_pe  = TIM_CCMR1_OC2PE; /* PDF Reference: OC2PE */
            ccer_enable_mask  = TIM_CCER_CC2E; /* PDF Reference: CC2E */
            ccer_polarity_mask= TIM_CCER_CC2P; /* PDF Reference: CC2P */
            ccr_reg = &TIMx->CCR2; /* PDF Reference: CCR2 */
            break;
        case 3:
            // TIM9/10 do not have CH3/4
            if (TIMx == TIM9 || TIMx == TIM10) return;
            ccmr_reg = &TIMx->CCMR2; /* PDF Reference: CCMR2 */
            ccmr_clear_mask_s = TIM_CCMR2_CC3S_Msk;
            ccmr_clear_mask_m = TIM_CCMR2_OC3M_Msk;
            ccmr_set_mask_m   = TIM_CCMR2_OC3M_PWM1; /* PDF Reference: OC3M_PWM1 */
            ccmr_set_mask_pe  = TIM_CCMR2_OC3PE; /* PDF Reference: OC3PE */
            ccer_enable_mask  = TIM_CCER_CC3E; /* PDF Reference: CC3E */
            ccer_polarity_mask= TIM_CCER_CC3P; /* PDF Reference: CC3P */
            ccr_reg = &TIMx->CCR3; /* PDF Reference: CCR3 */
            break;
        case 4:
             // TIM9/10 do not have CH3/4
            if (TIMx == TIM9 || TIMx == TIM10) return;
            ccmr_reg = &TIMx->CCMR2; /* PDF Reference: CCMR2 */
            ccmr_clear_mask_s = TIM_CCMR2_CC4S_Msk;
            ccmr_clear_mask_m = TIM_CCMR2_OC4M_Msk;
            ccmr_set_mask_m   = TIM_CCMR2_OC4M_PWM1; /* PDF Reference: OC4M_PWM1 */
            ccmr_set_mask_pe  = TIM_CCMR2_OC4PE; /* PDF Reference: OC4PE */
            ccer_enable_mask  = TIM_CCER_CC4E; /* PDF Reference: CC4E */
            ccer_polarity_mask= TIM_CCER_CC4P; /* PDF Reference: CC4P */
            ccr_reg = &TIMx->CCR4; /* PDF Reference: CCR4 */
            break;
        default:
            // Should not happen based on channel_map, but for safety
            return;
    }

    // Configure Channel as Output (CCxS = 00)
    *ccmr_reg &= ~ccmr_clear_mask_s; /* PDF Reference: CCMRx, CCxS */

    // Configure Output Compare Mode (OCxM = PWM1) and enable Output Compare Preload (OCxPE = 1)
    *ccmr_reg &= ~ccmr_clear_mask_m; /* PDF Reference: CCMRx, OCxM */
    *ccmr_reg |= ccmr_set_mask_m; /* PDF Reference: OCxM_PWM1 */
    *ccmr_reg |= ccmr_set_mask_pe; /* PDF Reference: OCxPE */

    // Set initial Capture Compare Register value (duty cycle)
    *ccr_reg = duty_counts; /* PDF Reference: CCRx */

    // Enable Capture/Compare Output (CCxE = 1)
    TIMx->CCER |= ccer_enable_mask; /* PDF Reference: CCER, CCxE */

    // Set output polarity (CCxP = 0 for active high)
    TIMx->CCER &= ~ccer_polarity_mask; /* PDF Reference: CCER, CCxP */

    // TIM1 specific: Enable Main Output (MOE) in BDTR
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: BDTR, MOE */
    }

    // 11. Generate an Update Event to load all registers from preload to active
    TIMx->EGR |= TIM_EGR_UG; /* PDF Reference: EGR, UG */

    // 12. Mark channel as initialized
    initialized_channels[TRD_Channel] = 1;
}

/***********************************************************************************************************************
* Function Name: PWM_Set_Freq
* Description  : Set the desired PWM frequency and duty cycle for the selected channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to configure.
*              : tlong frequency             - Desired PWM frequency in Hz.
*              : tbyte duty                  - Desired duty cycle in percentage (0-100).
* Return Value : None
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    if (TRD_Channel >= TRD_CHANNEL_COUNT || frequency == 0 || duty > 100) {
        // Invalid channel, frequency, or duty cycle
        return;
    }

    // Ensure the channel has been initialized
    if (initialized_channels[TRD_Channel] == 0) {
        // Channel not initialized, cannot set frequency/duty
        // Could call PWM_Init here, but the requirement is separate functions.
        // User must call PWM_Init first.
        return;
    }

    const PWM_Channel_Map_t *channel_map = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_map->TIM;
    uint8_t channel_num = channel_map->Channel; // 1-based channel number

    uint32_t timer_clock = get_timer_clock_freq(TIMx); /* Assumed Timer clock frequency */
    uint32_t max_arr = get_timer_max_arr(TIMx);

    uint16_t prescaler = 0;
    uint32_t arr = 0;

    // Calculate new PSC and ARR based on the desired frequency
    calculate_timer_params(timer_clock, frequency, &prescaler, &arr, max_arr);

    // Calculate the new Capture Compare Register value based on the new ARR and duty
    uint32_t duty_counts = (arr + 1) * duty / 100; /* PDF Reference for duty cycle logic */

    // Update Timer registers
    // These changes take effect at the next Update Event because ARPE is enabled
    TIMx->PSC = (uint16_t)prescaler; /* PDF Reference: PSC */
    TIMx->ARR = arr;                 /* PDF Reference: ARR */

    volatile uint32_t *ccr_reg;
    switch (channel_num) {
        case 1: ccr_reg = &TIMx->CCR1; break; /* PDF Reference: CCR1 */
        case 2: ccr_reg = &TIMx->CCR2; break; /* PDF Reference: CCR2 */
        case 3: ccr_reg = &TIMx->CCR3; break; /* PDF Reference: CCR3 */
        case 4: ccr_reg = &TIMx->CCR4; break; /* PDF Reference: CCR4 */
        default: return; // Should not happen
    }
    *ccr_reg = duty_counts; /* PDF Reference: CCRx */
}

/***********************************************************************************************************************
* Function Name: PWM_Start
* Description  : Enable and start PWM signal generation on the specified channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to start.
* Return Value : None
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel) {
     if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

    // Ensure the channel has been initialized
    if (initialized_channels[TRD_Channel] == 0) {
        // Channel not initialized, cannot start
        return;
    }

    const PWM_Channel_Map_t *channel_map = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_map->TIM;
    uint8_t channel_num = channel_map->Channel;

    // TIM1 specific: Set Main Output Enable (MOE) in BDTR
    // This is required for TIM1 output to be active, even single channel
    if (TIMx == TIM1) {
        TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference: BDTR, MOE */
    }

    // Enable the Capture/Compare Output (CCxE = 1)
    // This bit needs to be set for the pin to output the timer signal
    uint32_t ccer_enable_mask;
    switch (channel_num) {
        case 1: ccer_enable_mask = TIM_CCER_CC1E; break; /* PDF Reference: CCER, CC1E */
        case 2: ccer_enable_mask = TIM_CCER_CC2E; break; /* PDF Reference: CCER, CC2E */
        case 3: ccer_enable_mask = TIM_CCER_CC3E; break; /* PDF Reference: CCER, CC3E */
        case 4: ccer_enable_mask = TIM_CCER_CC4E; break; /* PDF Reference: CCER, CC4E */
        default: return; // Should not happen
    }
     TIMx->CCER |= ccer_enable_mask; /* PDF Reference: CCER, CCxE */


    // Enable the timer counter (CEN = 1)
    TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference: CR1, CEN */
}

/***********************************************************************************************************************
* Function Name: PWM_Stop
* Description  : Stop the PWM signal output on the specified channel.
* Arguments    : TRD_Channel_t TRD_Channel - The PWM channel to stop.
* Return Value : None
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel) {
     if (TRD_Channel >= TRD_CHANNEL_COUNT) {
        // Invalid channel
        return;
    }

     // No need to check initialization for stopping, just disable if it's running

    const PWM_Channel_Map_t *channel_map = &pwm_channel_map[TRD_Channel];
    TIM_TypeDef *TIMx = channel_map->TIM;
    uint8_t channel_num = channel_map->Channel;

    // Disable the Capture/Compare Output (CCxE = 0)
    // This prevents the timer from controlling the pin
    uint32_t ccer_enable_mask;
    switch (channel_num) {
        case 1: ccer_enable_mask = TIM_CCER_CC1E; break; /* PDF Reference: CCER, CC1E */
        case 2: ccer_enable_mask = TIM_CCER_CC2E; break; /* PDF Reference: CCER, CC2E */
        case 3: ccer_enable_mask = TIM_CCER_CC3E; break; /* PDF Reference: CCER, CC3E */
        case 4: ccer_enable_mask = TIM_CCER_CC4E; break; /* PDF Reference: CCER, CC4E */
        default: return; // Should not happen
    }
    TIMx->CCER &= ~ccer_enable_mask; /* PDF Reference: CCER, CCxE */

    // TIM1 specific: Clear Main Output Enable (MOE) in BDTR
    // If this is the only channel using TIM1 output, clear MOE.
    // A more robust implementation would track MOE state per timer.
    // For this simple implementation, assume stopping *any* TIM1 channel might disable MOE.
    // This might require careful handling if multiple TIM1 channels are used simultaneously.
    if (TIMx == TIM1) {
        TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference: BDTR, MOE */
    }

    // Disable the timer counter (CEN = 0)
    // Note: This stops the *entire* timer peripheral, affecting all its channels.
    // A more granular stop would only disable the specific channel's output.
    // Requirement is "Stop the PWM signal output on the specified channel". Disabling CCxE
    // is the primary way to stop a single channel output. Disabling CEN stops the timer counts.
    // Let's disable CEN as it's also explicitly part of the Start function.
    TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: CR1, CEN */

    // Optionally, reset the counter
    // TIMx->CNT = 0; /* PDF Reference: CNT */
}

/***********************************************************************************************************************
* Function Name: PWM_PowerOff
* Description  : Disable all PWM peripherals and outputs to reduce power consumption.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void PWM_PowerOff(void) {
    // Iterate through all possible PWM channels
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_COUNT; ++i) {
        if (initialized_channels[i]) {
            const PWM_Channel_Map_t *channel_map = &pwm_channel_map[i];
            TIM_TypeDef *TIMx = channel_map->TIM;
            GPIO_TypeDef *GPIOx = channel_map->GPIO_Port;
            uint8_t pin = channel_map->GPIO_Pin;
            uint8_t channel_num = channel_map->Channel;

            // 1. Disable the Capture/Compare Output (CCxE = 0)
            uint32_t ccer_enable_mask;
            switch (channel_num) {
                case 1: ccer_enable_mask = TIM_CCER_CC1E; break; /* PDF Reference: CCER, CC1E */
                case 2: ccer_enable_mask = TIM_CCER_CC2E; break; /* PDF Reference: CCER, CC2E */
                case 3: ccer_enable_mask = TIM_CCER_CC3E; break; /* PDF Reference: CCER, CC3E */
                case 4: ccer_enable_mask = TIM_CCER_CC4E; break; /* PDF Reference: CCER, CC4E */
                default: continue; // Should not happen
            }
            TIMx->CCER &= ~ccer_enable_mask; /* PDF Reference: CCER, CCxE */

            // TIM1 specific: Clear Main Output Enable (MOE) in BDTR
            if (TIMx == TIM1) {
                 TIMx->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference: BDTR, MOE */
            }

            // 2. Disable the timer counter (CEN = 0)
            TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference: CR1, CEN */

            // 3. Set GPIO pin back to Input Floating (reset state)
            // Clear MODE bits for the pin
            GPIOx->MODER &= ~(GPIO_MODER_MODEy_Msk(pin)); /* PDF Reference: MODER */
            // MODE bits reset value is 00 (Input). No need to explicitly set.

            // Also reset other GPIO configs (OTYPER, OSPEEDR, PUPDR, AFRL/AFRH)
            // Reset values are typically 0 for these (Input floating PP Low speed No PU/PD AF0)
            // For robustness, explicitly set them back to likely reset state.
            GPIOx->OTYPER &= ~(GPIO_OTYPER_OTy_Msk(pin)); /* PDF Reference: OTYPER */
            GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDRy_Msk(pin)); /* PDF Reference: OSPEEDR */
            GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDRy_Msk(pin)); /* PDF Reference: PUPDR */

            // Reset AF config (AF0 is often default after reset)
             if (pin < 8) {
                GPIOx->AFRL &= ~(GPIO_AFRL_AFRLy_Msk(pin)); /* PDF Reference: AFRL */
            } else {
                GPIOx->AFRH &= ~(GPIO_AFRH_AFRHy_Msk(pin)); /* PDF Reference: AFRH */
            }

            // 4. Disable Timer clock (only if no other channels on this timer are active - requires tracking)
            // A simpler approach for a full power off is to disable the clock if any channel
            // on that timer was initialized via this module. A static flag per timer could work.
            // For this implementation, let's just assume we can disable the timer clock if ANY
            // channel was initialized through this module. A more complex system might check
            // if the timer is still used by other modules or channels.
            // We don't have state tracking per timer, so iterate through *all* map entries for a timer
            // to see if any are still active before disabling the clock. This is inefficient but safe.

            uint8_t any_channel_initialized_on_this_timer = 0;
            for (TRD_Channel_t j = 0; j < TRD_CHANNEL_COUNT; ++j) {
                if (initialized_channels[j] && pwm_channel_map[j].TIM == TIMx) {
                    any_channel_initialized_on_this_timer = 1;
                    break;
                }
            }

            if (any_channel_initialized_on_this_timer) {
                 // Need to disable the clock carefully to avoid affecting other PWM channels on the same timer
                 // This requires tracking if *all* channels on a given TIM have been stopped.
                 // A simpler power-off just disables everything regardless of state.
                 // Let's refine: PowerOff disables ALL configured PWM peripherals used by *this* module.
                 // A simple way is to just disable the clock unconditionally for each timer *once*.
                 // We need a way to avoid disabling the clock multiple times for the same timer.
                 // A mask for powered off timers.

                uint32_t tim_mask = 0;
                if (TIMx == TIM1) tim_mask = (1U << 0);
                else if (TIMx == TIM2) tim_mask = (1U << 1);
                else if (TIMx == TIM3) tim_mask = (1U << 2);
                else if (TIMx == TIM4) tim_mask = (1U << 3);
                else if (TIMx == TIM9) tim_mask = (1U << 8); // Use different bit positions to avoid conflict with TIM1-4 bits
                else if (TIMx == TIM10) tim_mask = (1U << 9);
                // Reserved TIM5 (bit 4) and TIM11 (bit 10) are not used here

                static uint32_t powered_off_timers_mask = 0; // Tracks timers whose clocks have been disabled by this func

                if (!(powered_off_timers_mask & tim_mask)) {
                     if (channel_map->TIM_CLK_EN_REG == &RCC_APB1ENR) {
                         *channel_map->TIM_CLK_EN_REG &= ~channel_map->TIM_CLK_EN_BIT; /* Assumed RCC TIM Enable Bit */
                     } else if (channel_map->TIM_CLK_EN_REG == &RCC_APB2ENR) {
                         *channel_map->TIM_CLK_EN_REG &= ~channel_map->TIM_CLK_EN_BIT; /* Assumed RCC TIM Enable Bit */
                     }
                     powered_off_timers_mask |= tim_mask; // Mark this timer as powered off
                }

                // Mark the channel as no longer initialized
                initialized_channels[i] = 0;
            }

             // 5. Disable GPIO clock (only if no other channels on this port are active - requires tracking)
             // Similar issue to timer clock. A simple power-off disables all GPIOs used by this module.
             // Track powered off GPIO ports.
             uint32_t gpio_mask = 0;
             if (GPIOx == GPIOA) gpio_mask = (1U << 0);
             else if (GPIOx == GPIOB) gpio_mask = (1U << 1);
             else if (GPIOx == GPIOC) gpio_mask = (1U << 2);
             else if (GPIOx == GPIOD) gpio_mask = (1U << 3);

             static uint32_t powered_off_gpios_mask = 0; // Tracks GPIOs whose clocks have been disabled by this func

            if (!(powered_off_gpios_mask & gpio_mask)) {
                 set_gpio_clock(GPIOx, 0); // Disable clock for the port
                 powered_off_gpios_mask |= gpio_mask; // Mark this GPIO as powered off
            }
        }
    }
}


// Helper function to calculate PSC and ARR for a given frequency
// It tries to find a pair (PSC, ARR) that fits within the timer's limits
// and yields the target frequency. Prioritizes resolution (smaller PSC first).
// If frequency is 0, returns PSC=0, ARR=0.
// If frequency cannot be achieved, may return PSC=0xFFFF or ARR=0 depending on calculation path.
static void calculate_timer_params(uint32_t timer_clock, tlong frequency, uint16_t *prescaler, uint32_t *arr, uint32_t max_arr) {
    if (frequency == 0 || timer_clock == 0) {
        *prescaler = 0;
        *arr = 0;
        return;
    }

    // Calculate the total number of timer counts per period
    uint32_t total_counts_ideal = timer_clock / frequency;

    // Handle case where frequency is too high (total_counts < 1)
    if (total_counts_ideal < 1) {
        *prescaler = 0;
        *arr = 0; // Cannot generate frequency higher than timer_clock
        return;
    }

    // Try PSC = 0 first
    if (total_counts_ideal - 1 <= max_arr) {
        *prescaler = 0;
        *arr = total_counts_ideal - 1;
        return;
    }

    // Iterate PSC from 1 upwards to find a suitable value
    uint16_t psc = 1;
    uint32_t current_arr;

    while (psc <= 0xFFFF) { // PSC is 16-bit
         // Calculate (ARR + 1) for the current PSC
         uint32_t counts_per_arr_plus_1 = total_counts_ideal / (psc + 1);

         if (counts_per_arr_plus_1 > 0) {
             current_arr = counts_per_arr_plus_1 - 1;

             // Check if the calculated ARR fits within the timer's capacity
             if (current_arr <= max_arr) {
                 // Found a valid pair (psc, current_arr)
                 *prescaler = psc;
                 *arr = current_arr;
                 return;
             }
         }
         // If counts_per_arr_plus_1 is 0, it means total_counts_ideal < (psc + 1),
         // which won't work. Increment psc and try again.

         psc++;
    }

    // If the loop finishes, it means no valid (psc, arr) pair was found
    // to achieve the frequency. Return values indicating failure or max possible.
    // For safety, return 0 frequency parameters.
    *prescaler = 0xFFFF; // Or 0
    *arr = 0; // Or max_arr with highest possible psc
}

/* Note: The implementation reserves TIM5 and TIM11 from being used for PWM.
 * This decision is based on the requirement to reserve at least two timers for
 * potential use by an RTOS, delay functions, or other system tasks not handled here.
 * TIM5 (32-bit) and TIM11 (16-bit) are chosen as common candidates for such roles,
 * leaving TIM1, TIM2, TIM3, TIM4, TIM9, and TIM10 available for PWM generation.
 */