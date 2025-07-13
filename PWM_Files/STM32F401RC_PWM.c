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
#include <stdint.h> // For uint32_t, uint8_t, uint64_t

// Assume standard CMSIS definitions for peripheral base addresses and register structures
// The provided PDF excerpt does not contain these definitions.
// Assumed standard STM32F4 memory mapping and peripheral register access.
#define PERIPH_BASE           (0x40000000U) /*!< Peripheral base address */
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000U)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)

// Assumed GPIO Port Base Addresses (from standard STM32F4 memory map)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800U)
// ... other ports not used in example map

// Assumed TIM Base Addresses (from standard STM32F4 memory map)
// TIM2-7 on APB1, TIM1,8-11 on APB2
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000U)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800U)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000U)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400U)

// Assumed RCC Base Address (from standard STM32F4 memory map)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800U)

// Assumed RCC Register Offsets (Not in PDF, based on standard STM32F4)
#define RCC_AHB1ENR_OFFSET    (0x30U)
#define RCC_APB1ENR_OFFSET    (0x40U)
#define RCC_APB2ENR_OFFSET    (0x44U)

// Assumed RCC Register Pointers
#define RCC                 ((RCC_TypeDef *) RCC_BASE)

// Assumed GPIO Register Structures and Pointers (based on PDF offsets)
typedef struct
{
  volatile uint32_t MODER;    /* GPIO port mode register,                 Offset: 0x00 */ /* PDF Reference */
  volatile uint32_t OTYPER;   /* GPIO port output type register,          Offset: 0x04 */ /* PDF Reference */
  volatile uint32_t OSPEEDR;  /* GPIO port output speed register,         Offset: 0x08 */ /* PDF Reference */
  volatile uint32_t PUPDR;    /* GPIO port pull-up/pull-down register,    Offset: 0x0C */ /* PDF Reference */
  volatile uint32_t IDR;      /* GPIO port input data register,           Offset: 0x10 */ /* PDF Reference */
  volatile uint32_t ODR;      /* GPIO port output data register,          Offset: 0x14 */ /* PDF Reference */
  volatile uint32_t BSRR;     /* GPIO port bit set/reset register,        Offset: 0x18 */ /* PDF Reference */
  volatile uint32_t LCKR;     /* GPIO port configuration lock register,   Offset: 0x1C */ /* PDF Reference */
  volatile uint32_t AFRL;     /* GPIO alternate function low register,    Offset: 0x20 */ /* PDF Reference */
  volatile uint32_t AFRH;     /* GPIO alternate function high register,   Offset: 0x24 */ /* PDF Reference */
} GPIO_TypeDef;

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

// Assumed TIM Register Structures and Pointers (based on PDF offsets)
// Generic structure for TIM2-5, TIM9-11 (common registers)
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1,              Offset: 0x00 */ /* PDF Reference */
  volatile uint32_t CR2;      /* TIM control register 2,              Offset: 0x04 */ /* PDF Reference */
  volatile uint32_t SMCR;     /* TIM slave mode control register,     Offset: 0x08 */ /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register,   Offset: 0x0C */ /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register,                 Offset: 0x10 */ /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register,       Offset: 0x14 */ /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, Offset: 0x18 */ /* PDF Reference */
  volatile uint32_t CCMR2;    /* TIM capture/compare mode register 2, Offset: 0x1C */ /* PDF Reference */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, Offset: 0x20 */ /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter register,                Offset: 0x24 */ /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register,              Offset: 0x28 */ /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register,            Offset: 0x2C */ /* PDF Reference */
  uint32_t RESERVED0;        /* Reserved,                            Offset: 0x30 */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1,      Offset: 0x34 */ /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2,      Offset: 0x38 */ /* PDF Reference */
  volatile uint32_t CCR3;     /* TIM capture/compare register 3,      Offset: 0x3C */ /* PDF Reference */
  volatile uint32_t CCR4;     /* TIM capture/compare register 4,      Offset: 0x40 */ /* PDF Reference */
  uint32_t RESERVED1;        /* Reserved,                            Offset: 0x44 */ // BDTR for TIM1
  volatile uint32_t DCR;      /* TIM DMA control register,            Offset: 0x48 */ /* PDF Reference */
  volatile uint32_t DMAR;     /* TIM DMA address for full transfer,   Offset: 0x4C */ /* PDF Reference */
  volatile uint32_t OR;       /* TIM option register,                 Offset: 0x50 */ // Only on TIM2, TIM5, TIM11
} TIM_TypeDef;

// TIM1 structure (includes BDTR)
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1,              Offset: 0x00 */ /* PDF Reference */
  volatile uint32_t CR2;      /* TIM control register 2,              Offset: 0x04 */ /* PDF Reference */
  volatile uint32_t SMCR;     /* TIM slave mode control register,     Offset: 0x08 */ /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register,   Offset: 0x0C */ /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register,                 Offset: 0x10 */ /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register,       Offset: 0x14 */ /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, Offset: 0x18 */ /* PDF Reference */
  volatile uint32_t CCMR2;    /* TIM capture/compare mode register 2, Offset: 0x1C */ /* PDF Reference */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, Offset: 0x20 */ /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter register,                Offset: 0x24 */ /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register,              Offset: 0x28 */ /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register,            Offset: 0x2C */ /* PDF Reference */
  volatile uint32_t RCR;      /* TIM repetition counter register,     Offset: 0x30 */ /* PDF Reference */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1,      Offset: 0x34 */ /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2,      Offset: 0x38 */ /* PDF Reference */
  volatile uint32_t CCR3;     /* TIM capture/compare register 3,      Offset: 0x3C */ /* PDF Reference */
  volatile uint32_t CCR4;     /* TIM capture/compare register 4,      Offset: 0x40 */ /* PDF Reference */
  volatile uint32_t BDTR;     /* TIM break and dead-time register,    Offset: 0x44 */ /* PDF Reference */
  volatile uint32_t DCR;      /* TIM DMA control register,            Offset: 0x48 */ /* PDF Reference */
  volatile uint32_t DMAR;     /* TIM DMA address for full transfer,   Offset: 0x4C */ /* PDF Reference */
} TIM1_TypeDef;

#define TIM1                ((TIM1_TypeDef *) TIM1_BASE)
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)


// Assumed RCC structure (Not in PDF, based on standard STM32F4)
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t AHB1ENR; // Offset 0x30
  volatile uint32_t AHB2ENR;
  volatile uint32_t APB1ENR; // Offset 0x40
  volatile uint32_t APB2ENR; // Offset 0x44
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  volatile uint32_t DCKCFGR;
} RCC_TypeDef;


// Assumed Timer Clock Frequencies (Based on typical STM32F401RC configuration)
// Assuming APBx prescalers are > 1, so TIMCLK = 2 * PCLKx
#define TIMER_CLK_APB1      (84000000UL) /* 42 MHz APB1 * 2 */ // Assumed timer clock
#define TIMER_CLK_APB2      (168000000UL) /* 84 MHz APB2 * 2 */ // Assumed timer clock

// Bit definitions based on PDF
// GPIO MODER
#define GPIO_MODER_MODER0_Pos (0U) /* PDF Reference */
#define GPIO_MODER_MODER0_Msk (0x3U << GPIO_MODER_MODER0_Pos) /* PDF Reference */
#define GPIO_MODER_MODER0_Input (0x0U << GPIO_MODER_MODER0_Pos) /* PDF Reference */
#define GPIO_MODER_MODER0_Output (0x1U << GPIO_MODER_MODER0_Pos) /* PDF Reference */
#define GPIO_MODER_MODER0_AF (0x2U << GPIO_MODER_MODER0_Pos) /* PDF Reference */
#define GPIO_MODER_MODER0_Analog (0x3U << GPIO_MODER_MODER0_Pos) /* PDF Reference */
#define GPIO_MODER_MODER(Pin) (GPIO_MODER_MODER0_Msk << (2*(Pin))) /* PDF Reference */
#define GPIO_MODER_MODER_AF_Set(Pin) (GPIO_MODER_MODER0_AF << (2*(Pin))) /* PDF Reference */

// GPIO OTYPER
#define GPIO_OTYPER_OT0_Pos (0U) /* PDF Reference */
#define GPIO_OTYPER_OT0_Msk (0x1U << GPIO_OTYPER_OT0_Pos) /* PDF Reference */
#define GPIO_OTYPER_OT0_PP (0x0U << GPIO_OTYPER_OT0_Pos) /* PDF Reference */
#define GPIO_OTYPER_OT0_OD (0x1U << GPIO_OTYPER_OT0_Pos) /* PDF Reference */
#define GPIO_OTYPER_OT(Pin) (GPIO_OTYPER_OT0_Msk << (Pin)) /* PDF Reference */
#define GPIO_OTYPER_PP_Set(Pin) (GPIO_OTYPER_OT0_PP << (Pin)) /* PDF Reference */

// GPIO OSPEEDR
#define GPIO_OSPEEDR_OSPEEDR0_Pos (0U) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR0_Msk (0x3U << GPIO_OSPEEDR_OSPEEDR0_Pos) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR0_Low (0x0U << GPIO_OSPEEDR_OSPEEDR0_Pos) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR0_Medium (0x1U << GPIO_OSPEEDR_OSPEEDR0_Pos) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR0_High (0x2U << GPIO_OSPEEDR_OSPEEDR0_Pos) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR0_VeryHigh (0x3U << GPIO_OSPEEDR_OSPEEDR0_Pos) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR(Pin) (GPIO_OSPEEDR_OSPEEDR0_Msk << (2*(Pin))) /* PDF Reference */
#define GPIO_OSPEEDR_High_Set(Pin) (GPIO_OSPEEDR_OSPEEDR0_High << (2*(Pin))) /* PDF Reference */

// GPIO PUPDR
#define GPIO_PUPDR_PUPDR0_Pos (0U) /* PDF Reference */
#define GPIO_PUPDR_PUPDR0_Msk (0x3U << GPIO_PUPDR_PUPDR0_Pos) /* PDF Reference */
#define GPIO_PUPDR_PUPDR0_NoPull (0x0U << GPIO_PUPDR_PUPDR0_Pos) /* PDF Reference */
#define GPIO_PUPDR_PUPDR0_PullUp (0x1U << GPIO_PUPDR_PUPDR0_Pos) /* PDF Reference */
#define GPIO_PUPDR_PUPDR0_PullDown (0x2U << GPIO_PUPDR_PUPDR0_Pos) /* PDF Reference */
#define GPIO_PUPDR_PUPDR(Pin) (GPIO_PUPDR_PUPDR0_Msk << (2*(Pin))) /* PDF Reference */
#define GPIO_PUPDR_NoPull_Set(Pin) (GPIO_PUPDR_PUPDR0_NoPull << (2*(Pin))) /* PDF Reference */

// GPIO AFRL/AFRH
#define GPIO_AFRL_AFRL0_Pos (0U) /* PDF Reference */
#define GPIO_AFRL_AFRL0_Msk (0xFU << GPIO_AFRL_AFRL0_Pos) /* PDF Reference */
#define GPIO_AFRL_AFR(Pin) (GPIO_AFRL_AFRL0_Msk << (4*((Pin) % 8))) /* PDF Reference */
#define GPIO_AFRL_AFR_Set(Pin, AF) (((uint32_t)(AF)) << (4*((Pin) % 8))) /* PDF Reference */

#define GPIO_AFRH_AFRH8_Pos (0U) /* PDF Reference */
#define GPIO_AFRH_AFRH8_Msk (0xFU << GPIO_AFRH_AFRH8_Pos) /* PDF Reference */
#define GPIO_AFRH_AFR(Pin) (GPIO_AFRH_AFRH8_Msk << (4*((Pin) % 8))) /* PDF Reference */
#define GPIO_AFRH_AFR_Set(Pin, AF) (((uint32_t)(AF)) << (4*((Pin) % 8))) /* PDF Reference */


// TIM CR1
#define TIM_CR1_CEN_Pos           (0U) /* PDF Reference */
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos) /* PDF Reference */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk /* PDF Reference */
#define TIM_CR1_UDIS_Pos          (1U) /* PDF Reference */
#define TIM_CR1_UDIS_Msk          (0x1U << TIM_CR1_UDIS_Pos) /* PDF Reference */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk /* PDF Reference */
#define TIM_CR1_URS_Pos           (2U) /* PDF Reference */
#define TIM_CR1_URS_Msk           (0x1U << TIM_CR1_URS_Pos) /* PDF Reference */
#define TIM_CR1_URS               TIM_CR1_URS_Msk /* PDF Reference */
#define TIM_CR1_OPM_Pos           (3U) /* PDF Reference */
#define TIM_CR1_OPM_Msk           (0x1U << TIM_CR1_OPM_Pos) /* PDF Reference */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk /* PDF Reference */
#define TIM_CR1_DIR_Pos           (4U) /* PDF Reference */
#define TIM_CR1_DIR_Msk           (0x1U << TIM_CR1_DIR_Pos) /* PDF Reference */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk /* PDF Reference */
#define TIM_CR1_CMS_Pos           (5U) /* PDF Reference */
#define TIM_CR1_CMS_Msk           (0x3U << TIM_CR1_CMS_Pos) /* PDF Reference */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk /* PDF Reference */
#define TIM_CR1_ARPE_Pos          (7U) /* PDF Reference */
#define TIM_CR1_ARPE_Msk          (0x1U << TIM_CR1_ARPE_Pos) /* PDF Reference */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk /* PDF Reference */
#define TIM_CR1_CKD_Pos           (8U) /* PDF Reference */
#define TIM_CR1_CKD_Msk           (0x3U << TIM_CR1_CKD_Pos) /* PDF Reference */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk /* PDF Reference */

// TIM CR2 (common bits)
#define TIM_CR2_CCDS_Pos          (3U) /* PDF Reference */
#define TIM_CR2_CCDS_Msk          (0x1U << TIM_CR2_CCDS_Pos) /* PDF Reference */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk /* PDF Reference */
#define TIM_CR2_MMS_Pos           (4U) /* PDF Reference */
#define TIM_CR2_MMS_Msk           (0x7U << TIM_CR2_MMS_Pos) /* PDF Reference */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk /* PDF Reference */
#define TIM_CR2_TI1S_Pos          (7U) /* PDF Reference */
#define TIM_CR2_TI1S_Msk          (0x1U << TIM_CR2_TI1S_Pos) /* PDF Reference */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk /* PDF Reference */

// TIM CR2 (TIM1 specific bits)
#define TIM_CR2_CCPC_Pos          (0U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_CCPC_Msk          (0x1U << TIM_CR2_CCPC_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_CCUS_Pos          (2U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_CCUS_Msk          (0x1U << TIM_CR2_CCUS_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1_Pos          (8U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1_Msk          (0x1U << TIM_CR2_OIS1_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1N_Pos         (9U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1N_Msk         (0x1U << TIM_CR2_OIS1N_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2_Pos          (10U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2_Msk          (0x1U << TIM_CR2_OIS2_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2N_Pos         (11U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2N_Msk         (0x1U << TIM_CR2_OIS2N_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3_Pos          (12U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3_Msk          (0x1U << TIM_CR2_OIS3_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3N_Pos         (13U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3N_Msk         (0x1U << TIM_CR2_OIS3N_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS4_Pos          (14U) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS4_Msk          (0x1U << TIM_CR2_OIS4_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk /* PDF Reference */ // TIM1 specific

// TIM SMCR (common bits)
#define TIM_SMCR_SMS_Pos          (0U) /* PDF Reference */
#define TIM_SMCR_SMS_Msk          (0x7U << TIM_SMCR_SMS_Pos) /* PDF Reference */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk /* PDF Reference */
#define TIM_SMCR_TS_Pos           (4U) /* PDF Reference */
#define TIM_SMCR_TS_Msk           (0x7U << TIM_SMCR_TS_Pos) /* PDF Reference */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk /* PDF Reference */
#define TIM_SMCR_MSM_Pos          (7U) /* PDF Reference */
#define TIM_SMCR_MSM_Msk          (0x1U << TIM_SMCR_MSM_Pos) /* PDF Reference */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk /* PDF Reference */
#define TIM_SMCR_ETF_Pos          (8U) /* PDF Reference */
#define TIM_SMCR_ETF_Msk          (0xFU << TIM_SMCR_ETF_Pos) /* PDF Reference */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk /* PDF Reference */
#define TIM_SMCR_ETPS_Pos         (12U) /* PDF Reference */
#define TIM_SMCR_ETPS_Msk         (0x3U << TIM_SMCR_ETPS_Pos) /* PDF Reference */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk /* PDF Reference */
#define TIM_SMCR_ECE_Pos          (14U) /* PDF Reference */
#define TIM_SMCR_ECE_Msk          (0x1U << TIM_SMCR_ECE_Pos) /* PDF Reference */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk /* PDF Reference */
#define TIM_SMCR_ETP_Pos          (15U) /* PDF Reference */
#define TIM_SMCR_ETP_Msk          (0x1U << TIM_SMCR_ETP_Pos) /* PDF Reference */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk /* PDF Reference */

// TIM DIER
#define TIM_DIER_UIE_Pos          (0U) /* PDF Reference */
#define TIM_DIER_UIE_Msk          (0x1U << TIM_DIER_UIE_Pos) /* PDF Reference */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk /* PDF Reference */
#define TIM_DIER_CC1IE_Pos        (1U) /* PDF Reference */
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos) /* PDF Reference */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk /* PDF Reference */
#define TIM_DIER_CC2IE_Pos        (2U) /* PDF Reference */
#define TIM_DIER_CC2IE_Msk        (0x1U << TIM_DIER_CC2IE_Pos) /* PDF Reference */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk /* PDF Reference */
#define TIM_DIER_CC3IE_Pos        (3U) /* PDF Reference */
#define TIM_DIER_CC3IE_Msk        (0x1U << TIM_DIER_CC3IE_Pos) /* PDF Reference */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk /* PDF Reference */
#define TIM_DIER_CC4IE_Pos        (4U) /* PDF Reference */
#define TIM_DIER_CC4IE_Msk        (0x1U << TIM_DIER_CC4IE_Pos) /* PDF Reference */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk /* PDF Reference */
#define TIM_DIER_TIE_Pos          (6U) /* PDF Reference */
#define TIM_DIER_TIE_Msk          (0x1U << TIM_DIER_TIE_Pos) /* PDF Reference */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk /* PDF Reference */
#define TIM_DIER_BIE_Pos          (7U) /* PDF Reference */ // TIM1 specific
#define TIM_DIER_BIE_Msk          (0x1U << TIM_DIER_BIE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_DIER_UDE_Pos          (8U) /* PDF Reference */
#define TIM_DIER_UDE_Msk          (0x1U << TIM_DIER_UDE_Pos) /* PDF Reference */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk /* PDF Reference */
#define TIM_DIER_CC1DE_Pos        (9U) /* PDF Reference */
#define TIM_DIER_CC1DE_Msk        (0x1U << TIM_DIER_CC1DE_Pos) /* PDF Reference */
#define TIM_DIER_CC1DE            TIM_DIER_CC1DE_Msk /* PDF Reference */
#define TIM_DIER_CC2DE_Pos        (10U) /* PDF Reference */
#define TIM_DIER_CC2DE_Msk        (0x1U << TIM_DIER_CC2DE_Pos) /* PDF Reference */
#define TIM_DIER_CC2DE            TIM_DIER_CC2DE_Msk /* PDF Reference */
#define TIM_DIER_CC3DE_Pos        (11U) /* PDF Reference */
#define TIM_DIER_CC3DE_Msk        (0x1U << TIM_DIER_CC3DE_Pos) /* PDF Reference */
#define TIM_DIER_CC3DE            TIM_DIER_CC3DE_Msk /* PDF Reference */
#define TIM_DIER_CC4DE_Pos        (12U) /* PDF Reference */
#define TIM_DIER_CC4DE_Msk        (0x1U << TIM_DIER_CC4DE_Pos) /* PDF Reference */
#define TIM_DIER_CC4DE            TIM_DIER_CC4DE_Msk /* PDF Reference */
#define TIM_DIER_COMDE_Pos        (13U) /* PDF Reference */ // TIM1 specific
#define TIM_DIER_COMDE_Msk        (0x1U << TIM_DIER_COMDE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_DIER_TDE_Pos          (14U) /* PDF Reference */
#define TIM_DIER_TDE_Msk          (0x1U << TIM_DIER_TDE_Pos) /* PDF Reference */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk /* PDF Reference */


// TIM EGR
#define TIM_EGR_UG_Pos            (0U) /* PDF Reference */
#define TIM_EGR_UG_Msk            (0x1U << TIM_EGR_UG_Pos) /* PDF Reference */
#define TIM_EGR_UG                TIM_EGR_UG_Msk /* PDF Reference */
#define TIM_EGR_CC1G_Pos          (1U) /* PDF Reference */
#define TIM_EGR_CC1G_Msk          (0x1U << TIM_EGR_CC1G_Pos) /* PDF Reference */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk /* PDF Reference */
#define TIM_EGR_CC2G_Pos          (2U) /* PDF Reference */
#define TIM_EGR_CC2G_Msk          (0x1U << TIM_EGR_CC2G_Pos) /* PDF Reference */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk /* PDF Reference */
#define TIM_EGR_CC3G_Pos          (3U) /* PDF Reference */
#define TIM_EGR_CC3G_Msk          (0x1U << TIM_EGR_CC3G_Pos) /* PDF Reference */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk /* PDF Reference */
#define TIM_EGR_CC4G_Pos          (4U) /* PDF Reference */
#define TIM_EGR_CC4G_Msk          (0x1U << TIM_EGR_CC4G_Pos) /* PDF Reference */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk /* PDF Reference */
#define TIM_EGR_COMG_Pos          (5U) /* PDF Reference */ // TIM1 specific
#define TIM_EGR_COMG_Msk          (0x1U << TIM_EGR_COMG_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk /* PDF Reference */ // TIM1 specific
#define TIM_EGR_TG_Pos            (6U) /* PDF Reference */
#define TIM_EGR_TG_Msk            (0x1U << TIM_EGR_TG_Pos) /* PDF Reference */
#define TIM_EGR_TG                TIM_EGR_TG_Msk /* PDF Reference */
#define TIM_EGR_BG_Pos            (7U) /* PDF Reference */ // TIM1 specific
#define TIM_EGR_BG_Msk            (0x1U << TIM_EGR_BG_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_EGR_BG                TIM_EGR_BG_Msk /* PDF Reference */ // TIM1 specific


// TIM CCMR1/CCMR2 (Output compare mode)
#define TIM_CCMR1_OC1M_Pos        (4U) /* PDF Reference */
#define TIM_CCMR1_OC1M_Msk        (0x7U << TIM_CCMR1_OC1M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC1M_PWM1       (0x6U << TIM_CCMR1_OC1M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC1M_PWM2       (0x7U << TIM_CCMR1_OC1M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC1PE_Pos       (3U) /* PDF Reference */
#define TIM_CCMR1_OC1PE_Msk       (0x1U << TIM_CCMR1_OC1PE_Pos) /* PDF Reference */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk /* PDF Reference */
#define TIM_CCMR1_OC1FE_Pos       (2U) /* PDF Reference */
#define TIM_CCMR1_OC1FE_Msk       (0x1U << TIM_CCMR1_OC1FE_Pos) /* PDF Reference */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk /* PDF Reference */
#define TIM_CCMR1_CC1S_Pos        (0U) /* PDF Reference */
#define TIM_CCMR1_CC1S_Msk        (0x3U << TIM_CCMR1_CC1S_Pos) /* PDF Reference */
#define TIM_CCMR1_CC1S_Output     (0x0U << TIM_CCMR1_CC1S_Pos) /* PDF Reference */

#define TIM_CCMR1_OC2M_Pos        (12U) /* PDF Reference */
#define TIM_CCMR1_OC2M_Msk        (0x7U << TIM_CCMR1_OC2M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC2M_PWM1       (0x6U << TIM_CCMR1_OC2M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC2M_PWM2       (0x7U << TIM_CCMR1_OC2M_Pos) /* PDF Reference */
#define TIM_CCMR1_OC2PE_Pos       (11U) /* PDF Reference */
#define TIM_CCMR1_OC2PE_Msk       (0x1U << TIM_CCMR1_OC2PE_Pos) /* PDF Reference */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk /* PDF Reference */
#define TIM_CCMR1_OC2FE_Pos       (10U) /* PDF Reference */
#define TIM_CCMR1_OC2FE_Msk       (0x1U << TIM_CCMR1_OC2FE_Pos) /* PDF Reference */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk /* PDF Reference */
#define TIM_CCMR1_CC2S_Pos        (8U) /* PDF Reference */
#define TIM_CCMR1_CC2S_Msk        (0x3U << TIM_CCMR1_CC2S_Pos) /* PDF Reference */
#define TIM_CCMR1_CC2S_Output     (0x0U << TIM_CCMR1_CC2S_Pos) /* PDF Reference */

#define TIM_CCMR2_OC3M_Pos        (4U) /* PDF Reference */
#define TIM_CCMR2_OC3M_Msk        (0x7U << TIM_CCMR2_OC3M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC3M_PWM1       (0x6U << TIM_CCMR2_OC3M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC3M_PWM2       (0x7U << TIM_CCMR2_OC3M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC3PE_Pos       (3U) /* PDF Reference */
#define TIM_CCMR2_OC3PE_Msk       (0x1U << TIM_CCMR2_OC3PE_Pos) /* PDF Reference */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk /* PDF Reference */
#define TIM_CCMR2_OC3FE_Pos       (2U) /* PDF Reference */
#define TIM_CCMR2_OC3FE_Msk       (0x1U << TIM_CCMR2_OC3FE_Pos) /* PDF Reference */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk /* PDF Reference */
#define TIM_CCMR2_CC3S_Pos        (0U) /* PDF Reference */
#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos) /* PDF Reference */
#define TIM_CCMR2_CC3S_Output     (0x0U << TIM_CCMR2_CC3S_Pos) /* PDF Reference */

#define TIM_CCMR2_OC4M_Pos        (12U) /* PDF Reference */
#define TIM_CCMR2_OC4M_Msk        (0x7U << TIM_CCMR2_OC4M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC4M_PWM1       (0x6U << TIM_CCMR2_OC4M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC4M_PWM2       (0x7U << TIM_CCMR2_OC4M_Pos) /* PDF Reference */
#define TIM_CCMR2_OC4PE_Pos       (11U) /* PDF Reference */
#define TIM_CCMR2_OC4PE_Msk       (0x1U << TIM_CCMR2_OC4PE_Pos) /* PDF Reference */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk /* PDF Reference */
#define TIM_CCMR2_OC4FE_Pos       (10U) /* PDF Reference */
#define TIM_CCMR2_OC4FE_Msk       (0x1U << TIM_CCMR2_OC4FE_Pos) /* PDF Reference */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk /* PDF Reference */
#define TIM_CCMR2_CC4S_Pos        (8U) /* PDF Reference */
#define TIM_CCMR2_CC4S_Msk        (0x3U << TIM_CCMR2_CC4S_Pos) /* PDF Reference */
#define TIM_CCMR2_CC4S_Output     (0x0U << TIM_CCMR2_CC4S_Pos) /* PDF Reference */


// TIM CCER
#define TIM_CCER_CC1E_Pos         (0U) /* PDF Reference */
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos) /* PDF Reference */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk /* PDF Reference */
#define TIM_CCER_CC1P_Pos         (1U) /* PDF Reference */
#define TIM_CCER_CC1P_Msk         (0x1U << TIM_CCER_CC1P_Pos) /* PDF Reference */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk /* PDF Reference */
#define TIM_CCER_CC1NE_Pos        (2U) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC1NE_Msk        (0x1U << TIM_CCER_CC1NE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC1NP_Pos        (3U) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC1NP_Msk        (0x1U << TIM_CCER_CC1NP_Pos) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk /* PDF Reference */ // TIM1 specific (or Input for others)

#define TIM_CCER_CC2E_Pos         (4U) /* PDF Reference */
#define TIM_CCER_CC2E_Msk         (0x1U << TIM_CCER_CC2E_Pos) /* PDF Reference */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk /* PDF Reference */
#define TIM_CCER_CC2P_Pos         (5U) /* PDF Reference */
#define TIM_CCER_CC2P_Msk         (0x1U << TIM_CCER_CC2P_Pos) /* PDF Reference */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk /* PDF Reference */
#define TIM_CCER_CC2NE_Pos        (6U) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC2NE_Msk        (0x1U << TIM_CCER_CC2NE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC2NP_Pos        (7U) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC2NP_Msk        (0x1U << TIM_CCER_CC2NP_Pos) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk /* PDF Reference */ // TIM1 specific (or Input for others)

#define TIM_CCER_CC3E_Pos         (8U) /* PDF Reference */
#define TIM_CCER_CC3E_Msk         (0x1U << TIM_CCER_CC3E_Pos) /* PDF Reference */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk /* PDF Reference */
#define TIM_CCER_CC3P_Pos         (9U) /* PDF Reference */
#define TIM_CCER_CC3P_Msk         (0x1U << TIM_CCER_CC3P_Pos) /* PDF Reference */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk /* PDF Reference */
#define TIM_CCER_CC3NE_Pos        (10U) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC3NE_Msk        (0x1U << TIM_CCER_CC3NE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_CCER_CC3NP_Pos        (11U) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC3NP_Msk        (0x1U << TIM_CCER_CC3NP_Pos) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk /* PDF Reference */ // TIM1 specific (or Input for others)

#define TIM_CCER_CC4E_Pos         (12U) /* PDF Reference */
#define TIM_CCER_CC4E_Msk         (0x1U << TIM_CCER_CC4E_Pos) /* PDF Reference */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk /* PDF Reference */
#define TIM_CCER_CC4P_Pos         (13U) /* PDF Reference */
#define TIM_CCER_CC4P_Msk         (0x1U << TIM_CCER_CC4P_Pos) /* PDF Reference */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk /* PDF Reference */
#define TIM_CCER_CC4NP_Pos        (15U) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC4NP_Msk        (0x1U << TIM_CCER_CC4NP_Pos) /* PDF Reference */ // TIM1 specific (or Input for others)
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk /* PDF Reference */ // TIM1 specific (or Input for others)


// TIM BDTR (TIM1 specific)
#define TIM_BDTR_DTG_Pos          (0U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_DTG_Msk          (0xFFU << TIM_BDTR_DTG_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_LOCK_Pos         (8U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_LOCK_Msk         (0x3U << TIM_BDTR_LOCK_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSI_Pos         (10U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSI_Msk         (0x1U << TIM_BDTR_OSSI_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSR_Pos         (11U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSR_Msk         (0x1U << TIM_BDTR_OSSR_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKE_Pos          (12U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKE_Msk          (0x1U << TIM_BDTR_BKE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKP_Pos          (13U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKP_Msk          (0x1U << TIM_BDTR_BKP_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_AOE_Pos          (14U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_AOE_Msk          (0x1U << TIM_BDTR_AOE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_MOE_Pos          (15U) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_MOE_Msk          (0x1U << TIM_BDTR_MOE_Pos) /* PDF Reference */ // TIM1 specific
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk /* PDF Reference */ // TIM1 specific


// RCC AHB1ENR (Not in PDF, assumed from STM32F4)
#define RCC_AHB1ENR_GPIOAEN_Pos   (0U) // Assumed RCC config
#define RCC_AHB1ENR_GPIOAEN_Msk   (0x1U << RCC_AHB1ENR_GPIOAEN_Pos) // Assumed RCC config
#define RCC_AHB1ENR_GPIOBEN_Pos   (1U) // Assumed RCC config
#define RCC_AHB1ENR_GPIOBEN_Msk   (0x1U << RCC_AHB1ENR_GPIOBEN_Pos) // Assumed RCC config
#define RCC_AHB1ENR_GPIOCEN_Pos   (2U) // Assumed RCC config
#define RCC_AHB1ENR_GPIOCEN_Msk   (0x1U << RCC_AHB1ENR_GPIOCEN_Pos) // Assumed RCC config

// RCC APB1ENR (Not in PDF, assumed from STM32F4)
#define RCC_APB1ENR_TIM2EN_Pos    (0U) // Assumed RCC config
#define RCC_APB1ENR_TIM2EN_Msk    (0x1U << RCC_APB1ENR_TIM2EN_Pos) // Assumed RCC config
#define RCC_APB1ENR_TIM3EN_Pos    (1U) // Assumed RCC config
#define RCC_APB1ENR_TIM3EN_Msk    (0x1U << RCC_APB1ENR_TIM3EN_Pos) // Assumed RCC config
#define RCC_APB1ENR_TIM4EN_Pos    (2U) // Assumed RCC config
#define RCC_APB1ENR_TIM4EN_Msk    (0x1U << RCC_APB1ENR_TIM4EN_Pos) // Assumed RCC config
#define RCC_APB1ENR_TIM5EN_Pos    (3U) // Assumed RCC config
#define RCC_APB1ENR_TIM5EN_Msk    (0x1U << RCC_APB1ENR_TIM5EN_Pos) // Assumed RCC config

// RCC APB2ENR (Not in PDF, assumed from STM32F4)
#define RCC_APB2ENR_TIM1EN_Pos    (0U) // Assumed RCC config
#define RCC_APB2ENR_TIM1EN_Msk    (0x1U << RCC_APB2ENR_TIM1EN_Pos) // Assumed RCC config
#define RCC_APB2ENR_TIM9EN_Pos    (16U) // Assumed RCC config
#define RCC_APB2ENR_TIM9EN_Msk    (0x1U << RCC_APB2ENR_TIM9EN_Pos) // Assumed RCC config
#define RCC_APB2ENR_TIM10EN_Pos   (17U) // Assumed RCC config
#define RCC_APB2ENR_TIM10EN_Msk   (0x1U << RCC_APB2ENR_TIM10EN_Pos) // Assumed RCC config
#define RCC_APB2ENR_TIM11EN_Pos   (18U) // Assumed RCC config
#define RCC_APB2ENR_TIM11EN_Msk   (0x1U << RCC_APB2ENR_TIM11EN_Pos) // Assumed RCC config


// Structure to hold PWM channel configuration
typedef struct {
    void* TIMx;                 // Pointer to Timer peripheral (TIM1_TypeDef or TIM_TypeDef)
    uint8_t ChannelNumber;      // Timer Channel (1, 2, 3, or 4)
    GPIO_TypeDef* GPIO_Port;    // Pointer to GPIO port
    uint8_t PinNumber;          // GPIO pin number (0-15)
    uint8_t GPIO_AF;            // GPIO Alternate Function number
    uint32_t TimerClock;        // Clock frequency for this timer
} PWM_Channel_Config_t;


// Enum for specific PWM channels (based on available timers and chosen pins)
// TIM5 and TIM11 are reserved for other purposes (e.g., OS tick, delays).
// TIM2 is 32-bit, TIM3/4/9/10 are 16-bit, TIM1 is 16-bit advanced.
// Choose representative channels avoiding conflicts on the same physical pin in this map.
// Assumed PWM config - please verify
typedef enum {
    TRD_PWM_TIM1_CH1,   // PA8, AF1
    TRD_PWM_TIM1_CH2,   // PA9, AF1
    TRD_PWM_TIM1_CH3,   // PA10, AF1
    TRD_PWM_TIM1_CH4,   // PA11, AF1
    TRD_PWM_TIM2_CH1,   // PA5, AF1 (using PA5 instead of PA0)
    TRD_PWM_TIM2_CH2,   // PA1, AF1
    TRD_PWM_TIM2_CH3,   // PA2, AF1
    TRD_PWM_TIM2_CH4,   // PA3, AF1
    TRD_PWM_TIM3_CH1,   // PA6, AF2
    TRD_PWM_TIM3_CH2,   // PA7, AF2
    TRD_PWM_TIM3_CH3,   // PC8, AF2 (using PC8 instead of PB0)
    TRD_PWM_TIM3_CH4,   // PC9, AF2 (using PC9 instead of PB1)
    TRD_PWM_TIM4_CH1,   // PB6, AF2
    TRD_PWM_TIM4_CH2,   // PB7, AF2
    TRD_PWM_TIM4_CH3,   // PB8, AF2
    TRD_PWM_TIM4_CH4,   // PB9, AF2
    TRD_PWM_TIM9_CH1,   // PA2, AF3 (Note: PA2 is also TIM2_CH3, choose one config per pin in application)
    TRD_PWM_TIM10_CH1,  // PB8, AF3 (Note: PB8 is also TIM4_CH3, choose one config per pin in application)
    NUM_TRD_CHANNELS    // Sentinel for count
} TRD_Channel_t;


// Array mapping TRD_Channel_t enum to actual hardware configurations
// Assumed PWM config - please verify pin mappings and AF values based on your board/datasheet
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TIMx, ChannelNumber, GPIO_Port, PinNumber, GPIO_AF, TimerClock
    { TIM1, 1, GPIOA, 8,  1, TIMER_CLK_APB2 }, // PA8 -> TIM1_CH1 AF1
    { TIM1, 2, GPIOA, 9,  1, TIMER_CLK_APB2 }, // PA9 -> TIM1_CH2 AF1
    { TIM1, 3, GPIOA, 10, 1, TIMER_CLK_APB2 }, // PA10 -> TIM1_CH3 AF1
    { TIM1, 4, GPIOA, 11, 1, TIMER_CLK_APB2 }, // PA11 -> TIM1_CH4 AF1

    { TIM2, 1, GPIOA, 5,  1, TIMER_CLK_APB1 }, // PA5 -> TIM2_CH1 AF1
    { TIM2, 2, GPIOA, 1,  1, TIMER_CLK_APB1 }, // PA1 -> TIM2_CH2 AF1
    { TIM2, 3, GPIOA, 2,  1, TIMER_CLK_APB1 }, // PA2 -> TIM2_CH3 AF1
    { TIM2, 4, GPIOA, 3,  1, TIMER_CLK_APB1 }, // PA3 -> TIM2_CH4 AF1

    { TIM3, 1, GPIOA, 6,  2, TIMER_CLK_APB1 }, // PA6 -> TIM3_CH1 AF2
    { TIM3, 2, GPIOA, 7,  2, TIMER_CLK_APB1 }, // PA7 -> TIM3_CH2 AF2
    { TIM3, 3, GPIOC, 8,  2, TIMER_CLK_APB1 }, // PC8 -> TIM3_CH3 AF2
    { TIM3, 4, GPIOC, 9,  2, TIMER_CLK_APB1 }, // PC9 -> TIM3_CH4 AF2

    { TIM4, 1, GPIOB, 6,  2, TIMER_CLK_APB1 }, // PB6 -> TIM4_CH1 AF2
    { TIM4, 2, GPIOB, 7,  2, TIMER_CLK_APB1 }, // PB7 -> TIM4_CH2 AF2
    { TIM4, 3, GPIOB, 8,  2, TIMER_CLK_APB1 }, // PB8 -> TIM4_CH3 AF2
    { TIM4, 4, GPIOB, 9,  2, TIMER_CLK_APB1 }, // PB9 -> TIM4_CH4 AF2

    { TIM9, 1, GPIOA, 2,  3, TIMER_CLK_APB2 }, // PA2 -> TIM9_CH1 AF3 (Conflicts with TIM2_CH3)
    { TIM10, 1, GPIOB, 8, 3, TIMER_CLK_APB2 }, // PB8 -> TIM10_CH1 AF3 (Conflicts with TIM4_CH3)
};

// Note: Timers TIM5 and TIM11 are reserved for potential operating system
// or other timing-critical functions like delays. They are not used for PWM
// generation in this driver implementation.


/**Functions ===========================================================================*/

/**
  * @brief Initializes the PWM hardware for a specific channel.
  * @param TRD_Channel: The channel to initialize.
  */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    GPIO_TypeDef* gpio_port = config->GPIO_Port;
    uint8_t pin = config->PinNumber;
    void* tim_base = config->TIMx;
    uint8_t channel = config->ChannelNumber;
    uint8_t af = config->GPIO_AF;

    // 1. Enable GPIO clock (Assumed RCC registers, not in PDF)
    if (gpio_port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk; // Assumed RCC config
    } else if (gpio_port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk; // Assumed RCC config
    } else if (gpio_port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN_Msk; // Assumed RCC config
    }
    // Add checks for other ports if needed

    // 2. Configure GPIO pin for Alternate Function
    // Clear MODER bits and set to Alternate Function (10)
    gpio_port->MODER &= ~GPIO_MODER_MODER(pin); /* PDF Reference */
    gpio_port->MODER |= GPIO_MODER_MODER_AF_Set(pin); /* PDF Reference */

    // Configure Output Type: Push-Pull (0)
    gpio_port->OTYPER &= ~GPIO_OTYPER_OT(pin); /* PDF Reference */
    gpio_port->OTYPER |= GPIO_OTYPER_PP_Set(pin); /* PDF Reference */

    // Configure Output Speed: High Speed (10)
    gpio_port->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR(pin); /* PDF Reference */
    gpio_port->OSPEEDR |= GPIO_OSPEEDR_High_Set(pin); /* PDF Reference */

    // Configure Pull-up/Pull-down: No Pull-up/Pull-down (00)
    gpio_port->PUPDR &= ~GPIO_PUPDR_PUPDR(pin); /* PDF Reference */
    gpio_port->PUPDR |= GPIO_PUPDR_NoPull_Set(pin); /* PDF Reference */

    // Configure Alternate Function
    if (pin < 8) {
        gpio_port->AFRL &= ~GPIO_AFRL_AFR(pin); /* PDF Reference */
        gpio_port->AFRL |= GPIO_AFRL_AFR_Set(pin, af); /* PDF Reference */
    } else {
        gpio_port->AFRH &= ~GPIO_AFRH_AFR(pin); /* PDF Reference */
        gpio_port->AFRH |= GPIO_AFRH_AFR_Set(pin, af); /* PDF Reference */
    }

    // 3. Enable Timer clock (Assumed RCC registers, not in PDF)
    if (tim_base == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk; // Assumed RCC config
    } else if (tim_base == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk; // Assumed RCC config
    } else if (tim_base == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN_Msk; // Assumed RCC config
    } else if (tim_base == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN_Msk; // Assumed RCC config
    } else if (tim_base == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN_Msk; // Assumed RCC config
    } else if (tim_base == TIM10) {
         RCC->APB2ENR |= RCC_APB2ENR_TIM10EN_Msk; // Assumed RCC config
    }
    // TIM5, TIM11 explicitly excluded as reserved

    // Delay to allow timer clock to stabilize (optional but good practice)
    volatile uint32_t dummy;
    dummy = RCC->APB1ENR;
    dummy = RCC->APB2ENR;
    dummy = RCC->AHB1ENR;
    (void)dummy; // Avoid unused variable warning


    // 4. Configure Timer Timebase and PWM Mode
    // Stop the timer and disable UEV (UG bit can still generate update)
    ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Reference */
    ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_UDIS_Msk; /* PDF Reference */

    // Configure prescaler and ARR (will be set to default 0xFF/0xFFFF later, updated in Set_Freq)
    // Set upcounting mode (DIR = 0), edge-aligned mode (CMS = 00)
    ((TIM_TypeDef*)tim_base)->CR1 &= ~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk); /* PDF Reference */

    // Enable ARR preload buffer
    ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_ARPE_Msk; /* PDF Reference */

    // Configure PWM mode 1 (110) for the selected channel, enable preload for CCRx
    switch (channel) {
        case 1:
            ((TIM_TypeDef*)tim_base)->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk); /* PDF Reference */
            ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_CC1S_Output; /* PDF Reference */
            ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_OC1M_PWM1; /* PDF Reference */
            ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_OC1PE_Msk; /* PDF Reference */
            break;
        case 2:
            // TIM9/10/11 only have CH1
            if (tim_base != TIM9 && tim_base != TIM10) {
                 ((TIM_TypeDef*)tim_base)->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk); /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_CC2S_Output; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_OC2M_PWM1; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR1 |= TIM_CCMR1_OC2PE_Msk; /* PDF Reference */
            } else {
                 // Invalid channel for this timer type
                 ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_CEN_Msk; // Restore CEN if disabled above
                 ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_UDIS_Msk; // Restore UDIS if disabled above
                 return; // Exit if invalid channel
            }
            break;
        case 3:
             // TIM9/10/11 only have CH1 (and CH2 for TIM9)
            if (tim_base != TIM9 && tim_base != TIM10 && tim_base != TIM1) { // TIM1, TIM2-5 have CH3/4
                 ((TIM_TypeDef*)tim_base)->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk); /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_CC3S_Output; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC3M_PWM1; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC3PE_Msk; /* PDF Reference */
            } else if (tim_base == TIM1) { // Special case for TIM1 (BDTR related)
                 ((TIM1_TypeDef*)tim_base)->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk); /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_CC3S_Output; /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC3M_PWM1; /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC3PE_Msk; /* PDF Reference */
            }
             else {
                 // Invalid channel for this timer type
                 ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_CEN_Msk; // Restore CEN if disabled above
                 ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_UDIS_Msk; // Restore UDIS if disabled above
                 return; // Exit if invalid channel
             }
            break;
        case 4:
             // TIM9/10/11 only have CH1 (and CH2 for TIM9)
            if (tim_base != TIM9 && tim_base != TIM10 && tim_base != TIM1) { // TIM1, TIM2-5 have CH3/4
                 ((TIM_TypeDef*)tim_base)->CCMR2 &= ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk); /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_CC4S_Output; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC4M_PWM1; /* PDF Reference */
                 ((TIM_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC4PE_Msk; /* PDF Reference */
            } else if (tim_base == TIM1) { // Special case for TIM1 (BDTR related)
                 ((TIM1_TypeDef*)tim_base)->CCMR2 &= ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk); /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_CC4S_Output; /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC4M_PWM1; /* PDF Reference */
                 ((TIM1_TypeDef*)tim_base)->CCMR2 |= TIM_CCMR2_OC4PE_Msk; /* PDF Reference */
            }
             else {
                 // Invalid channel for this timer type
                 ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_CEN_Msk; // Restore CEN if disabled above
                 ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_UDIS_Msk; // Restore UDIS if disabled above
                 return; // Exit if invalid channel
             }
            break;
        default:
            // Invalid channel number
            ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_CEN_Msk; // Restore CEN if disabled above
            ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_UDIS_Msk; // Restore UDIS if disabled above
            return; // Exit
    }

    // Set output polarity to active high (CCxP = 0) for the channel
    switch (channel) {
        case 1: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC1P_Msk; break; /* PDF Reference */
        case 2: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC2P_Msk; break; /* PDF Reference */
        case 3: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC3P_Msk; break; /* PDF Reference */
        case 4: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC4P_Msk; break; /* PDF Reference */
        default: break; // Should not happen
    }

    // For TIM1, configure BDTR
    if (tim_base == TIM1) {
        // Enable main output (MOE=1), Clear OSSI/OSSR/LOCK/DTG for basic PWM
        ((TIM1_TypeDef*)tim_base)->BDTR |= TIM_BDTR_MOE_Msk; /* PDF Reference */
        ((TIM1_TypeDef*)tim_base)->BDTR &= ~(TIM_BDTR_OSSI_Msk | TIM_BDTR_OSSR_Msk | TIM_BDTR_LOCK_Msk | TIM_BDTR_DTG_Msk); /* PDF Reference */
         // Note: BDTR configuration is crucial for advanced timers like TIM1
    }

    // Generate an update event to load all configurations from preload registers
    // URS=0 is default after reset, so this UG generates an update event (UEV) and sets UIF
    ((TIM_TypeDef*)tim_base)->EGR |= TIM_EGR_UG_Msk; /* PDF Reference */

    // Re-enable UEV generation if desired (URS is still 0)
    ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_UDIS_Msk; /* PDF Reference */
}


/**
  * @brief Sets the PWM frequency and duty cycle for a specific channel.
  * @param TRD_Channel: The channel to configure.
  * @param frequency: Desired PWM frequency in Hz.
  * @param duty: Desired duty cycle in percentage (0-100).
  */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= NUM_TRD_CHANNELS || frequency == 0) {
        // Invalid channel or frequency
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    void* tim_base = config->TIMx;
    uint8_t channel = config->ChannelNumber;
    uint32_t timer_clock = config->TimerClock;

    // Calculate ARR and Prescaler
    uint32_t period;
    uint32_t psc = 0;
    uint32_t arr = 0;

    // Use 64-bit intermediate to prevent overflow during calculation
    uint64_t temp_period = (uint64_t)timer_clock / frequency;

    // Find a suitable prescaler and ARR combination
    // Prioritize larger period (smaller frequency division by PSC) for better resolution
    // Check timer bit width
    uint32_t max_arr = 0xFFFF; // 16-bit timers
    if (tim_base == TIM2) max_arr = 0xFFFFFFFF; // 32-bit timer

    if (temp_period <= max_arr) {
        psc = 0;
        arr = temp_period - 1;
    } else {
        // Need prescaler
        // Find minimum psc such that (timer_clock / (psc + 1)) / frequency fits in ARR
        // (psc + 1) > timer_clock / (frequency * max_arr)
        // psc > (timer_clock / (frequency * max_arr)) - 1
        // psc = (timer_clock / (frequency * max_arr))
        psc = (uint32_t)((uint64_t)timer_clock / (frequency * (max_arr + 1)));

        // Recalculate period with the found prescaler
        period = timer_clock / (psc + 1) / frequency;
        arr = period - 1;

        // Adjust psc if needed - a larger psc might result in a period that is slightly off
        // due to integer division. Try psc-1 to see if it fits.
        if (psc > 0) {
            uint32_t psc_try = psc - 1;
            uint32_t period_try = timer_clock / (psc_try + 1) / frequency;
            if (period_try <= max_arr) {
                 // If the lower prescaler results in a valid ARR, use it
                 // This gives better frequency accuracy
                 psc = psc_try;
                 arr = period_try - 1;
            }
        }

        // Final check to ensure calculated period is valid
        if (arr > max_arr) {
             // Still too large, or frequency is too low for the timer
             // Or the initial estimate for psc was too low due to integer math.
             // Increment psc until it fits.
             while ((uint64_t)timer_clock / (psc + 1) / frequency > max_arr) {
                  psc++;
             }
             period = timer_clock / (psc + 1) / frequency;
             arr = period - 1;
             if (arr > max_arr) {
                 // Frequency is too low for the timer, even with max prescaler.
                 // Set to minimum possible frequency (max ARR and max PSC)
                 psc = 0xFFFF; // 16-bit prescaler
                 arr = max_arr;
             }
        } else if (period == 0) {
             // Frequency is too high, period is less than 1 timer tick.
             // Set to maximum possible frequency (min ARR = 0, min PSC = 0)
             psc = 0;
             arr = 0; // Smallest non-zero period is 1 (ARR=0), highest frequency.
        }
    }


    // Calculate Capture/Compare Value (CCRx) based on duty cycle and ARR
    uint32_t ccr = 0;
    if (duty == 0) {
        ccr = 0; // 0% duty cycle
    } else if (duty >= 100) {
        // 100% duty cycle: CCRx >= ARR
        // Set CCR to ARR + 1 to ensure it's always >= ARR during upcounting
        ccr = arr + 1;
    } else {
        // CCRx = (Period * Duty) / 100
        // Period = ARR + 1
        ccr = (uint32_t)(((uint64_t)(arr + 1) * duty) / 100);
    }

    // Update timer registers
    // Temporarily disable counter and update disable to ensure PSC/ARR/CCRx are loaded correctly
    // Setting UG will load the new values
    uint32_t cr1_reg = ((TIM_TypeDef*)tim_base)->CR1;
    ((TIM_TypeDef*)tim_base)->CR1 &= ~(TIM_CR1_CEN_Msk | TIM_CR1_UDIS_Msk); /* PDF Reference */ // Disable CEN, Clear UDIS
    ((TIM_TypeDef*)tim_base)->PSC = psc; /* PDF Reference */
    ((TIM_TypeDef*)tim_base)->ARR = arr; /* PDF Reference */

    switch (channel) {
        case 1: ((TIM_TypeDef*)tim_base)->CCR1 = ccr; break; /* PDF Reference */
        case 2: ((TIM_TypeDef*)tim_base)->CCR2 = ccr; break; /* PDF Reference */
        case 3: ((TIM_TypeDef*)tim_base)->CCR3 = ccr; break; /* PDF Reference */
        case 4: ((TIM_TypeDef*)tim_base)->CCR4 = ccr; break; /* PDF Reference */
        default: break; // Should not happen
    }

    // Generate update event to load the new PSC, ARR, and CCRx values
    ((TIM_TypeDef*)tim_base)->EGR |= TIM_EGR_UG_Msk; /* PDF Reference */

    // Restore previous CEN state and clear UDIS if it was set before
    ((TIM_TypeDef*)tim_base)->CR1 = (cr1_reg & ~TIM_CR1_UDIS_Msk); /* PDF Reference */
}


/**
  * @brief Starts PWM signal generation on the specified channel.
  * @param TRD_Channel: The channel to start.
  */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    void* tim_base = config->TIMx;
    uint8_t channel = config->ChannelNumber;

    // Enable the specific channel output
    switch (channel) {
        case 1: ((TIM_TypeDef*)tim_base)->CCER |= TIM_CCER_CC1E_Msk; break; /* PDF Reference */
        case 2: ((TIM_TypeDef*)tim_base)->CCER |= TIM_CCER_CC2E_Msk; break; /* PDF Reference */
        case 3: ((TIM_TypeDef*)tim_base)->CCER |= TIM_CCER_CC3E_Msk; break; /* PDF Reference */
        case 4: ((TIM_TypeDef*)tim_base)->CCER |= TIM_CCER_CC4E_Msk; break; /* PDF Reference */
        default: return; // Should not happen
    }

    // For TIM1, also enable the main output
    if (tim_base == TIM1) {
        ((TIM1_TypeDef*)tim_base)->BDTR |= TIM_BDTR_MOE_Msk; /* PDF Reference */
    }

    // Enable the timer counter
    ((TIM_TypeDef*)tim_base)->CR1 |= TIM_CR1_CEN_Msk; /* PDF Reference */
}


/**
  * @brief Stops PWM signal output on the specified channel.
  * @param TRD_Channel: The channel to stop.
  */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= NUM_TRD_CHANNELS) {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    void* tim_base = config->TIMx;
    uint8_t channel = config->ChannelNumber;

    // Disable the specific channel output
    switch (channel) {
        case 1: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC1E_Msk; break; /* PDF Reference */
        case 2: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC2E_Msk; break; /* PDF Reference */
        case 3: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC3E_Msk; break; /* PDF Reference */
        case 4: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC4E_Msk; break; /* PDF Reference */
        default: return; // Should not happen
    }

    // Note: For TIM1, disabling individual CCxE bits might be sufficient
    // to stop the output on that pin, while keeping other channels/features active.
    // Clearing MOE stops ALL TIM1 outputs and breaks features. We'll keep MOE set
    // and rely on individual channel enables (CCxE).

    // Check if any channels are still enabled for this timer.
    // If not, stopping the counter might be desirable to save power.
    // This requires checking all CCxE bits for the timer.
    // assuming other channels might still be active or it will be stopped by PowerOff.
}


/**
  * @brief Disables all PWM peripherals and outputs.
  * Reduces power consumption by disabling timer clocks.
  */
void PWM_PowerOff(void)
{
    // Stop all timer counters used for PWM
    // Disable all channel outputs

    // Loop through all defined channels and stop them
    for (TRD_Channel_t i = 0; i < NUM_TRD_CHANNELS; ++i) {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        void* tim_base = config->TIMx;
        uint8_t channel = config->ChannelNumber;

        // Disable the specific channel output
        switch (channel) {
            case 1: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC1E_Msk; break; /* PDF Reference */
            case 2: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC2E_Msk; break; /* PDF Reference */
            case 3: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC3E_Msk; break; /* PDF Reference */
            case 4: ((TIM_TypeDef*)tim_base)->CCER &= ~TIM_CCER_CC4E_Msk; break; /* PDF Reference */
            default: break;
        }

        // Disable timer counter
        ((TIM_TypeDef*)tim_base)->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Reference */

        // For TIM1, disable main output
        if (tim_base == TIM1) {
            ((TIM1_TypeDef*)tim_base)->BDTR &= ~TIM_BDTR_MOE_Msk; /* PDF Reference */
        }
    }


    // Disable clocks for all used TIM peripherals (Assumed RCC registers, not in PDF)
    // This will power down the timer blocks.
    RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN_Msk | RCC_APB2ENR_TIM9EN_Msk | RCC_APB2ENR_TIM10EN_Msk); // Assumed RCC config
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN_Msk | RCC_APB1ENR_TIM3EN_Msk | RCC_APB1ENR_TIM4EN_Msk); // Assumed RCC config

    // Note: GPIO clocks are not disabled here, as they might be used by other peripherals.
    // If GPIOs were exclusively used for PWM, their clocks could also be disabled.
}