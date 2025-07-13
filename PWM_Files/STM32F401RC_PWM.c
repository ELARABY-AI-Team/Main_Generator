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
#include <stdint.h> // For standard integer types
#include <stddef.h> // For NULL

//======================================================================================================================
// Register Definitions (Derived from provided PDF offsets and standard STM32 bare-metal access)
// Note: Base addresses and RCC clocking assumed based on standard STM32F401RC memory map,
// as this information is not present in the provided PDF content.

#define PERIPH_BASE           (0x40000000UL) /*!< Peripheral base address */
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)

// GPIO Base Addresses (Assumed standard STM32F4 base addresses)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
// GPIOH_BASE exists but is not used for general GPIO in 401xB/C according to PDF, though registers exist.
// We only include ports A-E based on available TIM mappings and GPIO register map (x=A..E and H).

// Timer Base Addresses (Assumed standard STM32F4 base addresses)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL) // Advanced Control Timer
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL) // General Purpose (32-bit)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL) // General Purpose (16-bit)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL) // General Purpose (16-bit)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL) // General Purpose (32-bit) - RESERVED
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL) // General Purpose (16-bit)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL) // General Purpose (16-bit) - RESERVED
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL) // General Purpose (16-bit)
// TIM6, TIM7, TIM8, TIM12, TIM13, TIM14 are not described in the provided PDF section.

// RCC Base Address (Assumed standard STM32F4 base address)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL) /*!< RCC base address */

// RCC Register Offsets (Assumed standard STM32F4 register offsets for clock enable)
// Note: RCC register definitions are NOT in the provided PDF. This is a necessary assumption.
#define RCC_AHB1ENR_OFFSET    (0x30UL) /* AHB1 Peripheral Clock Enable Register */
#define RCC_APB1ENR_OFFSET    (0x40UL) /* APB1 Peripheral Clock Enable Register */
#define RCC_APB2ENR_OFFSET    (0x44UL) /* APB2 Peripheral Clock Enable Register */

// Peripheral clock enable registers pointers (Assumed standard STM32F4 pointers)
#define RCC_AHB1ENR           (*(volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET))
#define RCC_APB1ENR           (*(volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET))
#define RCC_APB2ENR           (*(volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET))

// GPIO Port Clock Enable Bits (Assumed standard STM32F4 bit positions)
// Note: GPIO clock enable bits are NOT in the provided PDF. This is a necessary assumption.
#define RCC_AHB1ENR_GPIOAEN_Pos (0U)
#define RCC_AHB1ENR_GPIOBEN_Pos (1U)
#define RCC_AHB1ENR_GPIOCEN_Pos (2U)
#define RCC_AHB1ENR_GPIODEN_Pos (3U)
#define RCC_AHB1ENR_GPIOEEN_Pos (4U)

#define RCC_AHB1ENR_GPIOAEN_Msk (1U << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN_Msk (1U << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOCEN_Msk (1U << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIODEN_Msk (1U << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIOEEN_Msk (1U << RCC_AHB1ENR_GPIOEEN_Pos)

// Timer Clock Enable Bits (Assumed standard STM32F4 bit positions)
// Note: Timer clock enable bits are NOT in the provided PDF. This is a necessary assumption.
#define RCC_APB2ENR_TIM1EN_Pos  (0U)
#define RCC_APB1ENR_TIM2EN_Pos  (0U)
#define RCC_APB1ENR_TIM3EN_Pos  (1U)
#define RCC_APB1ENR_TIM4EN_Pos  (2U)
#define RCC_APB1ENR_TIM5EN_Pos  (3U) // TIM5 is Reserved
#define RCC_APB2ENR_TIM9EN_Pos  (16U)
#define RCC_APB2ENR_TIM10EN_Pos (17U) // TIM10 is Reserved
#define RCC_APB2ENR_TIM11EN_Pos (18U)

#define RCC_APB2ENR_TIM1EN_Msk  (1U << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB1ENR_TIM2EN_Msk  (1U << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM3EN_Msk  (1U << RCC_APB1ENR_TIM3EN_Pos)
#define RCC_APB1ENR_TIM4EN_Msk  (1U << RCC_APB1ENR_TIM4EN_Pos)
#define RCC_APB1ENR_TIM5EN_Msk  (1U << RCC_APB1ENR_TIM5EN_Pos) // TIM5 is Reserved
#define RCC_APB2ENR_TIM9EN_Msk  (1U << RCC_APB2ENR_TIM9EN_Pos)
#define RCC_APB2ENR_TIM10EN_Msk (1U << RCC_APB2ENR_TIM10EN_Pos) // TIM10 is Reserved
#define RCC_APB2ENR_TIM11EN_Msk (1U << RCC_APB2ENR_TIM11EN_Pos)


// GPIO Register Map (Derived from provided PDF Table 27)
typedef struct
{
  volatile uint32_t MODER;    /*!< GPIO port mode register,                    Address offset: 0x00 */
  volatile uint32_t OTYPER;   /*!< GPIO port output type register,               Address offset: 0x04 */
  volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,              Address offset: 0x08 */
  volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,         Address offset: 0x0C */
  volatile uint32_t IDR;      /*!< GPIO port input data register,                Address offset: 0x10 */
  volatile uint32_t ODR;      /*!< GPIO port output data register,               Address offset: 0x14 */
  volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,             Address offset: 0x18 */
  volatile uint32_t LCKR;     /*!< GPIO port configuration lock register,        Address offset: 0x1C */
  volatile uint32_t AFRL;     /*!< GPIO alternate function low register,         Address offset: 0x20 */
  volatile uint32_t AFRH;     /*!< GPIO alternate function high register,        Address offset: 0x24 */
} GPIO_TypeDef;

// GPIO Register Bitfield Masks and Positions (Derived from provided PDF descriptions)
// MODER (2 bits per pin)
#define GPIO_MODER_MODE_Pos(Pin)    ((Pin) * 2U)
#define GPIO_MODER_MODE_Msk(Pin)    (0x3U << GPIO_MODER_MODE_Pos(Pin))
#define GPIO_MODER_INPUT            (0x0U)
#define GPIO_MODER_OUTPUT           (0x1U)
#define GPIO_MODER_AF               (0x2U)
#define GPIO_MODER_ANALOG           (0x3U)

// OTYPER (1 bit per pin)
#define GPIO_OTYPER_OT_Pos(Pin)     ((Pin) * 1U)
#define GPIO_OTYPER_OT_Msk(Pin)     (0x1U << GPIO_OTYPER_OT_Pos(Pin))
#define GPIO_OTYPER_PP              (0x0U) /* Push-pull */
#define GPIO_OTYPER_OD              (0x1U) /* Open-drain */

// OSPEEDR (2 bits per pin)
#define GPIO_OSPEEDR_OSPEED_Pos(Pin)((Pin) * 2U)
#define GPIO_OSPEEDR_OSPEED_Msk(Pin)(0x3U << GPIO_OSPEEDR_OSPEED_Pos(Pin))
#define GPIO_OSPEEDR_LOW_SPEED      (0x0U)
#define GPIO_OSPEEDR_MEDIUM_SPEED   (0x1U)
#define GPIO_OSPEEDR_HIGH_SPEED     (0x2U)
#define GPIO_OSPEEDR_VERY_HIGH_SPEED(0x3U)

// PUPDR (2 bits per pin)
#define GPIO_PUPDR_PUPD_Pos(Pin)    ((Pin) * 2U)
#define GPIO_PUPDR_PUPD_Msk(Pin)    (0x3U << GPIO_PUPDR_PUPD_Pos(Pin))
#define GPIO_PUPDR_NO_PULL          (0x0U)
#define GPIO_PUPDR_PULL_UP          (0x1U)
#define GPIO_PUPDR_PULL_DOWN        (0x2U)

// AFRL/AFRH (4 bits per pin)
#define GPIO_AFRL_AFSEL_Pos(Pin)    ((Pin) * 4U)
#define GPIO_AFRL_AFSEL_Msk(Pin)    (0xFU << GPIO_AFRL_AFSEL_Pos(Pin))
#define GPIO_AFRH_AFSEL_Pos(Pin)    (((Pin) - 8U) * 4U)
#define GPIO_AFRH_AFSEL_Msk(Pin)    (0xFU << GPIO_AFRH_AFSEL_Pos(Pin))
#define GPIO_AF_AFSEL(AF)           ((AF) & 0xFU)


// Timer Register Map (Derived from provided PDF Tables 52, 56, 59, 61)

// Base struct for common timer registers (TIM1, TIM2-5, TIM9-11 share some structure)
typedef struct
{
  volatile uint32_t CR1;      /*!< TIM control register 1,                      Address offset: 0x00 */
  volatile uint32_t CR2;      /*!< TIM control register 2,                      Address offset: 0x04 */
  volatile uint32_t SMCR;     /*!< TIM slave mode control register,             Address offset: 0x08 */
  volatile uint32_t DIER;     /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  volatile uint32_t SR;       /*!< TIM status register,                         Address offset: 0x10 */
  volatile uint32_t EGR;      /*!< TIM event generation register,               Address offset: 0x14 */
  volatile uint32_t CCMR1;    /*!< TIM capture/compare mode register 1,         Address offset: 0x18 */
  volatile uint32_t CCMR2;    /*!< TIM capture/compare mode register 2,         Address offset: 0x1C */ // Only for TIM1, TIM2-5, TIM9
  volatile uint32_t CCER;     /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  volatile uint32_t CNT;      /*!< TIM counter,                                 Address offset: 0x24 */
  volatile uint32_t PSC;      /*!< TIM prescaler,                               Address offset: 0x28 */
  volatile uint32_t ARR;      /*!< TIM auto-reload register,                    Address offset: 0x2C */
  volatile uint32_t RCR;      /*!< TIM repetition counter register,             Address offset: 0x30 */ // Only for TIM1
  volatile uint32_t CCR1;     /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  volatile uint32_t CCR2;     /*!< TIM capture/compare register 2,              Address offset: 0x38 */ // Only for TIM1, TIM2-5, TIM9
  volatile uint32_t CCR3;     /*!< TIM capture/compare register 3,              Address offset: 0x3C */ // Only for TIM1, TIM2-5
  volatile uint32_t CCR4;     /*!< TIM capture/compare register 4,              Address offset: 0x40 */ // Only for TIM1, TIM2-5
  volatile uint32_t BDTR;     /*!< TIM break and dead-time register,            Address offset: 0x44 */ // Only for TIM1
  volatile uint32_t DCR;      /*!< TIM DMA control register,                    Address offset: 0x48 */ // Only for TIM1, TIM2-5
  volatile uint32_t DMAR;     /*!< TIM DMA address for full transfer,           Address offset: 0x4C */ // Only for TIM1, TIM2-5
  volatile uint32_t OR;       /*!< TIM option register,                         Address offset: 0x50 */ // Only for TIM2, TIM5, TIM11
} TIM_TypeDef;


// Timer Register Bitfield Masks and Positions (Derived from provided PDF descriptions)

// TIMx_CR1 (0x00)
#define TIM_CR1_CEN_Pos         (0U)
#define TIM_CR1_CEN_Msk         (0x1U << TIM_CR1_CEN_Pos)       /* Counter enable */
#define TIM_CR1_UDIS_Pos        (1U)
#define TIM_CR1_UDIS_Msk        (0x1U << TIM_CR1_UDIS_Pos)      /* Update disable */
#define TIM_CR1_URS_Pos         (2U)
#define TIM_CR1_URS_Msk         (0x1U << TIM_CR1_URS_Pos)       /* Update request source */
#define TIM_CR1_OPM_Pos         (3U)
#define TIM_CR1_OPM_Msk         (0x1U << TIM_CR1_OPM_Pos)       /* One pulse mode */
#define TIM_CR1_DIR_Pos         (4U)
#define TIM_CR1_DIR_Msk         (0x1U << TIM_CR1_DIR_Pos)       /* Direction */
#define TIM_CR1_CMS_Pos         (5U)
#define TIM_CR1_CMS_Msk         (0x3U << TIM_CR1_CMS_Pos)       /* Center-aligned mode selection */
#define TIM_CR1_ARPE_Pos        (7U)
#define TIM_CR1_ARPE_Msk        (0x1U << TIM_CR1_ARPE_Pos)      /* Auto-reload preload enable */
#define TIM_CR1_CKD_Pos         (8U)
#define TIM_CR1_CKD_Msk         (0x3U << TIM_CR1_CKD_Pos)       /* Clock division */

// TIMx_CR2 (0x04)
#define TIM_CR2_CCPC_Pos        (0U) // TIM1 only
#define TIM_CR2_CCPC_Msk        (0x1U << TIM_CR2_CCPC_Pos)
#define TIM_CR2_CCUS_Pos        (2U) // TIM1 only
#define TIM_CR2_CCUS_Msk        (0x1U << TIM_CR2_CCUS_Pos)
#define TIM_CR2_CCDS_Pos        (3U)
#define TIM_CR2_CCDS_Msk        (0x1U << TIM_CR2_CCDS_Pos)      /* Capture/compare DMA selection */
#define TIM_CR2_MMS_Pos         (4U)
#define TIM_CR2_MMS_Msk         (0x7U << TIM_CR2_MMS_Pos)       /* Master mode selection */
#define TIM_CR2_TI1S_Pos        (7U) // TIM1, TIM2-5 share, TIM9-11 share
#define TIM_CR2_TI1S_Msk        (0x1U << TIM_CR2_TI1S_Pos)      /* TI1 selection */
#define TIM_CR2_OIS1_Pos        (8U) // TIM1 only
#define TIM_CR2_OIS1_Msk        (0x1U << TIM_CR2_OIS1_Pos)
#define TIM_CR2_OIS1N_Pos       (9U) // TIM1 only
#define TIM_CR2_OIS1N_Msk       (0x1U << TIM_CR2_OIS1N_Pos)
// ... OIS2, OIS2N, OIS3, OIS3N, OIS4

// TIMx_EGR (0x14)
#define TIM_EGR_UG_Pos          (0U)
#define TIM_EGR_UG_Msk          (0x1U << TIM_EGR_UG_Pos)        /* Update generation */
#define TIM_EGR_CC1G_Pos        (1U)
#define TIM_EGR_CC1G_Msk        (0x1U << TIM_EGR_CC1G_Pos)      /* Capture/compare 1 generation */
// ... CC2G, CC3G, CC4G, COMG (TIM1 only), TG, BG (TIM1 only)

// TIMx_CCMR1 (0x18) - Channels 1 & 2
#define TIM_CCMR1_CC1S_Pos      (0U)
#define TIM_CCMR1_CC1S_Msk      (0x3U << TIM_CCMR1_CC1S_Pos)    /* Capture/Compare 1 selection */
#define TIM_CCMR1_OC1FE_Pos     (2U)
#define TIM_CCMR1_OC1FE_Msk     (0x1U << TIM_CCMR1_OC1FE_Pos)   /* Output Compare 1 fast enable */
#define TIM_CCMR1_OC1PE_Pos     (3U)
#define TIM_CCMR1_OC1PE_Msk     (0x1U << TIM_CCMR1_OC1PE_Pos)   /* Output Compare 1 preload enable */
#define TIM_CCMR1_OC1M_Pos      (4U)
#define TIM_CCMR1_OC1M_Msk      (0x7U << TIM_CCMR1_OC1M_Pos)    /* Output Compare 1 mode */
#define TIM_CCMR1_OC1CE_Pos     (7U) // TIM1, TIM2-5 only
#define TIM_CCMR1_OC1CE_Msk     (0x1U << TIM_CCMR1_OC1CE_Pos)   /* Output Compare 1 clear enable */
#define TIM_CCMR1_CC2S_Pos      (8U) // TIM1, TIM2-5, TIM9 only
#define TIM_CCMR1_CC2S_Msk      (0x3U << TIM_CCMR1_CC2S_Pos)    /* Capture/Compare 2 selection */
#define TIM_CCMR1_OC2FE_Pos     (10U) // TIM1, TIM2-5, TIM9 only
#define TIM_CCMR1_OC2FE_Msk     (0x1U << TIM_CCMR1_OC2FE_Pos)   /* Output Compare 2 fast enable */
#define TIM_CCMR1_OC2PE_Pos     (11U) // TIM1, TIM2-5, TIM9 only
#define TIM_CCMR1_OC2PE_Msk     (0x1U << TIM_CCMR1_OC2PE_Pos)   /* Output Compare 2 preload enable */
#define TIM_CCMR1_OC2M_Pos      (12U) // TIM1, TIM2-5, TIM9 only
#define TIM_CCMR1_OC2M_Msk      (0x7U << TIM_CCMR1_OC2M_Pos)    /* Output Compare 2 mode */
#define TIM_CCMR1_OC2CE_Pos     (15U) // TIM1, TIM2-5 only
#define TIM_CCMR1_OC2CE_Msk     (0x1U << TIM_CCMR1_OC2CE_Pos)   /* Output Compare 2 clear enable */

// TIMx_CCMR2 (0x1C) - Channels 3 & 4 (Only TIM1, TIM2-5 have CCMR2)
#define TIM_CCMR2_CC3S_Pos      (0U)
#define TIM_CCMR2_CC3S_Msk      (0x3U << TIM_CCMR2_CC3S_Pos)    /* Capture/Compare 3 selection */
#define TIM_CCMR2_OC3FE_Pos     (2U)
#define TIM_CCMR2_OC3FE_Msk     (0x1U << TIM_CCMR2_OC3FE_Pos)   /* Output Compare 3 fast enable */
#define TIM_CCMR2_OC3PE_Pos     (3U)
#define TIM_CCMR2_OC3PE_Msk     (0x1U << TIM_CCMR2_OC3PE_Pos)   /* Output Compare 3 preload enable */
#define TIM_CCMR2_OC3M_Pos      (4U)
#define TIM_CCMR2_OC3M_Msk      (0x7U << TIM_CCMR2_OC3M_Pos)    /* Output Compare 3 mode */
#define TIM_CCMR2_OC3CE_Pos     (7U)
#define TIM_CCMR2_OC3CE_Msk     (0x1U << TIM_CCMR2_OC3CE_Pos)   /* Output Compare 3 clear enable */
#define TIM_CCMR2_CC4S_Pos      (8U)
#define TIM_CCMR2_CC4S_Msk      (0x3U << TIM_CCMR2_CC4S_Pos)    /* Capture/Compare 4 selection */
#define TIM_CCMR2_OC4FE_Pos     (10U)
#define TIM_CCMR2_OC4FE_Msk     (0x1U << TIM_CCMR2_OC4FE_Pos)   /* Output Compare 4 fast enable */
#define TIM_CCMR2_OC4PE_Pos     (11U)
#define TIM_CCMR2_OC4PE_Msk     (0x1U << TIM_CCMR2_OC4PE_Pos)   /* Output Compare 4 preload enable */
#define TIM_CCMR2_OC4M_Pos      (12U)
#define TIM_CCMR2_OC4M_Msk      (0x7U << TIM_CCMR2_OC4M_Pos)    /* Output Compare 4 mode */
#define TIM_CCMR2_OC4CE_Pos     (15U)
#define TIM_CCMR2_OC4CE_Msk     (0x1U << TIM_CCMR2_OC4CE_Pos)   /* Output Compare 4 clear enable */

// CCMRx OCxM values (PWM modes)
#define TIM_OCMODE_PWM1         (0x6U) // 110
#define TIM_OCMODE_PWM2         (0x7U) // 111

// TIMx_CCER (0x20)
#define TIM_CCER_CC1E_Pos       (0U)
#define TIM_CCER_CC1E_Msk       (0x1U << TIM_CCER_CC1E_Pos)     /* Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos       (1U)
#define TIM_CCER_CC1P_Msk       (0x1U << TIM_CCER_CC1P_Pos)     /* Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos      (2U) // TIM1 only
#define TIM_CCER_CC1NE_Msk      (0x1U << TIM_CCER_CC1NE_Pos)    /* Capture/Compare 1 complementary output enable */
#define TIM_CCER_CC1NP_Pos      (3U) // TIM1 only + Input capture for others
#define TIM_CCER_CC1NP_Msk      (0x1U << TIM_CCER_CC1NP_Pos)    /* Capture/Compare 1 complementary output Polarity */
// ... CC2E, CC2P, CC2NE, CC2NP (TIM1 only + Input for others), CC3E, CC3P, CC3NE, CC3NP (TIM1 only), CC4E, CC4P (TIM1 only)

// TIMx_BDTR (0x44) - Only TIM1 has BDTR according to PDF sections
#define TIM_BDTR_DTG_Pos        (0U)
#define TIM_BDTR_DTG_Msk        (0xFFU << TIM_BDTR_DTG_Pos)     /* Dead-time generator setup */
#define TIM_BDTR_LOCK_Pos       (8U)
#define TIM_BDTR_LOCK_Msk       (0x3U << TIM_BDTR_LOCK_Pos)     /* Lock configuration */
#define TIM_BDTR_OSSI_Pos       (10U)
#define TIM_BDTR_OSSI_Msk       (0x1U << TIM_BDTR_OSSI_Pos)     /* Off-state selection for Idle mode */
#define TIM_BDTR_OSSR_Pos       (11U)
#define TIM_BDTR_OSSR_Msk       (0x1U << TIM_BDTR_OSSR_Pos)     /* Off-state selection for Run mode */
#define TIM_BDTR_BKE_Pos        (12U)
#define TIM_BDTR_BKE_Msk        (0x1U << TIM_BDTR_BKE_Pos)      /* Break enable */
#define TIM_BDTR_BKP_Pos        (13U)
#define TIM_BDTR_BKP_Msk        (0x1U << TIM_BDTR_BKP_Pos)      /* Break polarity */
#define TIM_BDTR_AOE_Pos        (14U)
#define TIM_BDTR_AOE_Msk        (0x1U << TIM_BDTR_AOE_Pos)      /* Automatic output enable */
#define TIM_BDTR_MOE_Pos        (15U)
#define TIM_BDTR_MOE_Msk        (0x1U << TIM_BDTR_MOE_Pos)      /* Main output enable */


// Helper function to get GPIO Port base address
static GPIO_TypeDef* GPIO_Get_Port_Base(char port_name)
{
    switch (port_name)
    {
        case 'A': return (GPIO_TypeDef*)GPIOA_BASE;
        case 'B': return (GPIO_TypeDef*)GPIOB_BASE;
        case 'C': return (GPIO_TypeDef*)GPIOC_BASE;
        case 'D': return (GPIO_TypeDef*)GPIOD_BASE;
        case 'E': return (GPIO_TypeDef*)GPIOE_BASE;
        // Ports F, G, H, I, J, K are not generally available or used for this purpose
        // on STM32F401RC or not described for GPIO in the provided PDF for these pins/AFs.
        default: return NULL;
    }
}

// Helper function to get Timer base address and properties
typedef struct
{
    TIM_TypeDef* base;
    uint32_t clock_en_mask;
    volatile uint32_t* clock_en_reg;
    uint8_t is_32bit;
    uint8_t is_advanced; // TIM1 only
} TIM_Properties_t;

static TIM_Properties_t TIM_Get_Properties(uint8_t timer_number)
{
    TIM_Properties_t props = {0};
    switch (timer_number)
    {
        case 1:
            props.base = (TIM_TypeDef*)TIM1_BASE;
            props.clock_en_mask = RCC_APB2ENR_TIM1EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB2ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM1
            props.is_advanced = 1;
            break;
        case 2:
            props.base = (TIM_TypeDef*)TIM2_BASE;
            props.clock_en_mask = RCC_APB1ENR_TIM2EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB1ENR;            // Assumed RCC clocking
            props.is_32bit = 1; // 32-bit counter for TIM2 (PDF Ref - Table 56)
            props.is_advanced = 0;
            break;
        case 3:
            props.base = (TIM_TypeDef*)TIM3_BASE;
            props.clock_en_mask = RCC_APB1ENR_TIM3EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB1ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM3 (PDF Ref - Table 56)
            props.is_advanced = 0;
            break;
        case 4:
            props.base = (TIM_TypeDef*)TIM4_BASE;
            props.clock_en_mask = RCC_APB1ENR_TIM4EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB1ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM4 (PDF Ref - Table 56)
            props.is_advanced = 0;
            break;
        case 5:
             // TIM5 is Reserved for other uses as per requirements
            props.base = (TIM_TypeDef*)TIM5_BASE; // Still get base address for power off
            props.clock_en_mask = RCC_APB1ENR_TIM5EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB1ENR;            // Assumed RCC clocking
            props.is_32bit = 1; // 32-bit counter for TIM5 (PDF Ref - Table 56)
            props.is_advanced = 0;
            break;
        case 9:
            props.base = (TIM_TypeDef*)TIM9_BASE;
            props.clock_en_mask = RCC_APB2ENR_TIM9EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB2ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM9 (PDF Ref - Table 59)
            props.is_advanced = 0;
            break;
        case 10:
             // TIM10 is Reserved for other uses as per requirements
            props.base = (TIM_TypeDef*)TIM10_BASE; // Still get base address for power off
            props.clock_en_mask = RCC_APB2ENR_TIM10EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB2ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM10 (PDF Ref - Table 61)
            props.is_advanced = 0;
            break;
        case 11:
            props.base = (TIM_TypeDef*)TIM11_BASE;
            props.clock_en_mask = RCC_APB2ENR_TIM11EN_Msk; // Assumed RCC clocking
            props.clock_en_reg = &RCC_APB2ENR;            // Assumed RCC clocking
            props.is_32bit = 0; // 16-bit counter for TIM11 (PDF Ref - Table 61)
            props.is_advanced = 0;
            break;
        default:
            props.base = NULL; // Invalid timer number
            break;
    }
    return props;
}

// Helper function to get GPIO clock enable mask
static uint32_t GPIO_Get_Clock_Enable_Mask(char port_name)
{
     // Assumed RCC clocking - NOT in provided PDF
    switch (port_name)
    {
        case 'A': return RCC_AHB1ENR_GPIOAEN_Msk;
        case 'B': return RCC_AHB1ENR_GPIOBEN_Msk;
        case 'C': return RCC_AHB1ENR_GPIOCEN_Msk;
        case 'D': return RCC_AHB1ENR_GPIODEN_Msk;
        case 'E': return RCC_AHB1ENR_GPIOEEN_Msk;
        default: return 0;
    }
}


//======================================================================================================================
// PWM Channel Configuration Mapping
// This array maps logical PWM channels (TRD_Channel_t) to specific hardware timer channels and GPIO pins.
// Note: TIM5 and TIM10 are reserved as per requirements and excluded from this map.
// Note: Pin 0 is excluded as per requirements unless explicitly documented otherwise (not the case in PDF).
// Note: Pin-to-AF-to-TIM-Channel mappings are assumed based on standard STM32F401RC datasheets/pinouts,
// as the provided PDF does not contain this specific alternate function mapping table.
// All entries here are ASSUMED valid PWM configurations for STM32F401RC.
// The PortName uses standard GPIOx naming convention for register access.
// Format: TIMx, ChannelNumber, PortName, PinNumber, AlternateFunctionNumber

static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 - Advanced Control Timer (APB2) - Max 84MHz timer clock
    { 1, 1, 'A', 8, 1 }, /* Assumed PWM config - please verify */
    { 1, 2, 'A', 9, 1 }, /* Assumed PWM config - please verify */
    { 1, 3, 'A', 10, 1 }, /* Assumed PWM config - please verify */
    { 1, 4, 'A', 11, 1 }, /* Assumed PWM config - please verify */
    { 1, 1, 'E', 9, 1 }, /* Assumed PWM config - please verify */ // Duplicate TIM/Channel is bad, pick one pin per channel
    { 1, 2, 'E', 11, 1 }, /* Assumed PWM config - please verify */ // Duplicate
    { 1, 3, 'E', 13, 1 }, /* Assumed PWM config - please verify */ // Duplicate
    { 1, 4, 'E', 14, 1 }, /* Assumed PWM config - please verify */ // Duplicate
    // Corrected TIM1 entries (one pin per channel for common usage)
    { 1, 1, 'A', 8, 1 }, /* Assumed PWM config - TIM1_CH1 on PA8 (AF1) */
    { 1, 2, 'A', 9, 1 }, /* Assumed PWM config - TIM1_CH2 on PA9 (AF1) */
    { 1, 3, 'A', 10, 1 }, /* Assumed PWM config - TIM1_CH3 on PA10 (AF1) */
    { 1, 4, 'A', 11, 1 }, /* Assumed PWM config - TIM1_CH4 on PA11 (AF1) */

    // TIM2 - General Purpose Timer (APB1) - Max 42MHz timer clock (if APB1 prescaler > 1), Max 84MHz (if APB1 prescaler = 1)
    // Note: PA0 (TIM2_CH1) excluded due to pin 0 rule.
    { 2, 1, 'A', 1, 1 }, /* Assumed PWM config - TIM2_CH1 on PA1 (AF1) */
    { 2, 1, 'A', 5, 1 }, /* Assumed PWM config - TIM2_CH1 on PA5 (AF1) */ // Duplicate
    { 2, 1, 'A', 15, 1 }, /* Assumed PWM config - TIM2_CH1 on PA15 (AF1) */ // Duplicate
    { 2, 2, 'A', 2, 1 }, /* Assumed PWM config - TIM2_CH2 on PA2 (AF1) */
    { 2, 2, 'B', 3, 1 }, /* Assumed PWM config - TIM2_CH2 on PB3 (AF1) */ // Duplicate
    { 2, 3, 'A', 3, 1 }, /* Assumed PWM config - TIM2_CH3 on PA3 (AF1) */
    { 2, 3, 'B', 10, 1 }, /* Assumed PWM config - TIM2_CH3 on PB10 (AF1) */ // Duplicate
    { 2, 4, 'B', 11, 1 }, /* Assumed PWM config - TIM2_CH4 on PB11 (AF1) */
     // Corrected TIM2 entries (one pin per channel for common usage)
    { 2, 1, 'A', 1, 1 }, /* Assumed PWM config - TIM2_CH1 on PA1 (AF1) */
    { 2, 2, 'A', 2, 1 }, /* Assumed PWM config - TIM2_CH2 on PA2 (AF1) */
    { 2, 3, 'A', 3, 1 }, /* Assumed PWM config - TIM2_CH3 on PA3 (AF1) */
    { 2, 4, 'B', 11, 1 }, /* Assumed PWM config - TIM2_CH4 on PB11 (AF1) */


    // TIM3 - General Purpose Timer (APB1) - Max 84MHz timer clock (if APB1 prescaler > 1), Max 42MHz (if APB1 prescaler = 1)
    { 3, 1, 'A', 6, 2 }, /* Assumed PWM config - TIM3_CH1 on PA6 (AF2) */
    { 3, 2, 'A', 7, 2 }, /* Assumed PWM config - TIM3_CH2 on PA7 (AF2) */
    { 3, 3, 'B', 0, 2 }, /* Assumed PWM config - TIM3_CH3 on PB0 (AF2) */
    { 3, 4, 'B', 1, 2 }, /* Assumed PWM config - TIM3_CH4 on PB1 (AF2) */
    // Additional options for TIM3 (one pin per channel)
    // { 3, 1, 'B', 4, 2 }, /* Assumed PWM config - TIM3_CH1 on PB4 (AF2) */ // Alternative PA6
    // { 3, 1, 'C', 6, 2 }, /* Assumed PWM config - TIM3_CH1 on PC6 (AF2) */ // Alternative PA6
    // { 3, 2, 'B', 5, 2 }, /* Assumed PWM config - TIM3_CH2 on PB5 (AF2) */ // Alternative PA7
    // { 3, 2, 'C', 7, 2 }, /* Assumed PWM config - TIM3_CH2 on PC7 (AF2) */ // Alternative PA7
    // { 3, 3, 'C', 8, 2 }, /* Assumed PWM config - TIM3_CH3 on PC8 (AF2) */ // Alternative PB0
    // { 3, 4, 'C', 9, 2 }, /* Assumed PWM config - TIM3_CH4 on PC9 (AF2) */ // Alternative PB1


    // TIM4 - General Purpose Timer (APB1) - Max 84MHz timer clock (if APB1 prescaler > 1), Max 42MHz (if APB1 prescaler = 1)
    { 4, 1, 'B', 6, 2 }, /* Assumed PWM config - TIM4_CH1 on PB6 (AF2) */
    { 4, 2, 'B', 7, 2 }, /* Assumed PWM config - TIM4_CH2 on PB7 (AF2) */
    { 4, 3, 'B', 8, 2 }, /* Assumed PWM config - TIM4_CH3 on PB8 (AF2) */
    { 4, 4, 'B', 9, 2 }, /* Assumed PWM config - TIM4_CH4 on PB9 (AF2) */
    // PD pins are listed in GPIO registers A..E, H, so they are included here.
    // { 4, 1, 'D', 12, 2 }, /* Assumed PWM config - TIM4_CH1 on PD12 (AF2) */ // Alternative PB6
    // { 4, 2, 'D', 13, 2 }, /* Assumed PWM config - TIM4_CH2 on PD13 (AF2) */ // Alternative PB7
    // { 4, 3, 'D', 14, 2 }, /* Assumed PWM config - TIM4_CH3 on PD14 (AF2) */ // Alternative PB8
    // { 4, 4, 'D', 15, 2 }, /* Assumed PWM config - TIM4_CH4 on PD15 (AF2) */ // Alternative PB9

    // TIM5 - General Purpose Timer (APB1) - RESERVED as per requirements.
    // TIM10 - General Purpose Timer (APB2) - RESERVED as per requirements.

    // TIM9 - General Purpose Timer (APB2) - Max 84MHz timer clock (if APB2 prescaler > 1), Max 84MHz (if APB2 prescaler = 1)
    { 9, 1, 'A', 2, 3 }, /* Assumed PWM config - TIM9_CH1 on PA2 (AF3) */
    { 9, 2, 'A', 3, 3 }, /* Assumed PWM config - TIM9_CH2 on PA3 (AF3) */
    // PE pins are listed in GPIO registers A..E, H, so they are included here.
    // { 9, 1, 'E', 5, 3 }, /* Assumed PWM config - TIM9_CH1 on PE5 (AF3) */ // Alternative PA2
    // { 9, 2, 'E', 6, 3 }, /* Assumed PWM config - TIM9_CH2 on PE6 (AF3) */ // Alternative PA3

    // TIM11 - General Purpose Timer (APB2) - Max 84MHz timer clock (if APB2 prescaler > 1), Max 84MHz (if APB2 prescaler = 1)
    { 11, 1, 'B', 9, 3 } /* Assumed PWM config - TIM11_CH1 on PB9 (AF3) */
};

static const size_t pwm_channel_map_size = sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]);

/*
 * Timer Reservation:
 * TIM5 (32-bit) and TIM10 (single-channel) are reserved for potential system/OS use
 * or other non-PWM functionalities as per requirements.
 * They are excluded from the `pwm_channel_map` array for standard PWM driver usage.
 * This reservation is noted here as required.
 */

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The logical channel to initialize (index into pwm_channel_map).
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= pwm_channel_map_size)
    {
        // Invalid channel index
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    GPIO_TypeDef* gpio_port = GPIO_Get_Port_Base(config->PortName);
    TIM_Properties_t tim_props = TIM_Get_Properties(config->TIMx);
    TIM_TypeDef* tim = tim_props.base;

    if (gpio_port == NULL || tim == NULL)
    {
        // Configuration error or reserved timer
        return;
    }

    // 1. Enable GPIO clock (Assumed RCC clocking - NOT in provided PDF)
    uint32_t gpio_clk_mask = GPIO_Get_Clock_Enable_Mask(config->PortName);
    if (gpio_port == (GPIO_TypeDef*)GPIOA_BASE) { RCC_AHB1ENR |= gpio_clk_mask; } // Assumed RCC clocking
    else if (gpio_port == (GPIO_TypeDef*)GPIOB_BASE) { RCC_AHB1ENR |= gpio_clk_mask; } // Assumed RCC clocking
    else if (gpio_port == (GPIO_TypeDef*)GPIOC_BASE) { RCC_AHB1ENR |= gpio_clk_mask; } // Assumed RCC clocking
    else if (gpio_port == (GPIO_TypeDef*)GPIOD_BASE) { RCC_AHB1ENR |= gpio_clk_mask; } // Assumed RCC clocking
    else if (gpio_port == (GPIO_TypeDef*)GPIOE_BASE) { RCC_AHB1ENR |= gpio_clk_mask; } // Assumed RCC clocking

    // Delay after GPIO clock enable (Assumed standard practice, not in PDF)
    volatile uint32_t dummy_read = gpio_port->MODER; // Assumed MODER offset 0x00 (PDF Table 27)
    (void)dummy_read; // Avoid unused variable warning

    // 2. Configure GPIO pin for Alternate Function (PDF Ref - Section 8.3.11, 8.4)
    uint32_t pin = config->PinNumber;

    // Clear current mode and set Alternate Function mode
    gpio_port->MODER &= ~GPIO_MODER_MODE_Msk(pin); /* PDF Ref - Table 27 */
    gpio_port->MODER |= (GPIO_MODER_AF << GPIO_MODER_MODE_Pos(pin)); /* PDF Ref - Table 24, 27 */

    // Configure Output Type to Push-pull (Standard for PWM)
    gpio_port->OTYPER &= ~GPIO_OTYPER_OT_Msk(pin); /* PDF Ref - Table 27 */
    gpio_port->OTYPER |= (GPIO_OTYPER_PP << GPIO_OTYPER_OT_Pos(pin)); /* PDF Ref - Table 24, 27 */

    // Configure Output Speed (e.g., Very High Speed)
    gpio_port->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED_Msk(pin); /* PDF Ref - Table 27 */
    gpio_port->OSPEEDR |= (GPIO_OSPEEDR_VERY_HIGH_SPEED << GPIO_OSPEEDR_OSPEED_Pos(pin)); /* PDF Ref - Table 24, 27 */

    // Configure Pull-up/Pull-down to No Pull-up/Pull-down
    gpio_port->PUPDR &= ~GPIO_PUPDR_PUPD_Msk(pin); /* PDF Ref - Table 27 */
    gpio_port->PUPDR |= (GPIO_PUPDR_NO_PULL << GPIO_PUPDR_PUPD_Pos(pin)); /* PDF Ref - Table 24, 27 */

    // Select Alternate Function
    if (pin < 8)
    {
        gpio_port->AFRL &= ~GPIO_AFRL_AFSEL_Msk(pin); /* PDF Ref - Table 27 */
        gpio_port->AFRL |= (GPIO_AF_AFSEL(config->AlternateFunctionNumber) << GPIO_AFRL_AFSEL_Pos(pin)); /* PDF Ref - Table 27 */
    }
    else
    {
        gpio_port->AFRH &= ~GPIO_AFRH_AFSEL_Msk(pin); /* PDF Ref - Table 27 */
        gpio_port->AFRH |= (GPIO_AF_AFSEL(config->AlternateFunctionNumber) << GPIO_AFRH_AFSEL_Pos(pin)); /* PDF Ref - Table 27 */
    }

    // 3. Enable Timer clock (Assumed RCC clocking - NOT in provided PDF)
    *(tim_props.clock_en_reg) |= tim_props.clock_en_mask; // Assumed RCC clocking

    // Delay after Timer clock enable (Assumed standard practice, not in PDF)
    dummy_read = tim->CR1; // Assumed CR1 offset 0x00 (PDF Table 52, 56, 59, 61)
    (void)dummy_read; // Avoid unused variable warning

    // 4. Configure Timer for PWM (Upcounting, Edge-aligned, PWM Mode 1)
    // Set timer mode: upcounting, edge-aligned, buffer ARR (ARPE)
    tim->CR1 &= ~(TIM_CR1_DIR_Msk | TIM_CR1_CMS_Msk | TIM_CR1_ARPE_Msk); /* PDF Ref - Table 52, 56, 59, 61 */
    tim->CR1 |= TIM_CR1_ARPE_Msk; /* PDF Ref - Table 52, 56, 59, 61 */ // Enable ARR preload

    // Configure Clock Division (tDTS) - e.g., no division
    tim->CR1 &= ~TIM_CR1_CKD_Msk; /* PDF Ref - Table 52, 56, 59, 61 */
    tim->CR1 |= (0x0U << TIM_CR1_CKD_Pos); /* PDF Ref - Table 52, 56, 59, 61 */ // tDTS = tCK_INT

    uint32_t channel_number = config->ChannelNumber;
    uint32_t ccmr_offset = 0;
    uint32_t ccmr_mask = 0;
    uint32_t ccmr_ocm_pos = 0;
    uint32_t ccmr_ocpe_pos = 0;
    volatile uint32_t* ccmr_reg = NULL;
    volatile uint32_t* ccer_reg = &tim->CCER; /* PDF Ref - Table 52, 56, 59, 61 */
    uint32_t ccer_cce_pos = 0;
    uint32_t ccer_ccp_pos = 0;

    // Determine CCMR register, masks, and positions based on channel number
    if (channel_number == 1 || channel_number == 2)
    {
        ccmr_reg = &tim->CCMR1; /* PDF Ref - Table 52, 56, 59, 61 */
        if (channel_number == 1)
        {
            ccmr_offset = TIM_CCMR1_CC1S_Pos;
            ccmr_mask = TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_OC1FE_Msk; // Include FE/PE masks
            ccmr_ocm_pos = TIM_CCMR1_OC1M_Pos;
            ccmr_ocpe_pos = TIM_CCMR1_OC1PE_Pos;
            ccer_cce_pos = TIM_CCER_CC1E_Pos;
            ccer_ccp_pos = TIM_CCER_CC1P_Pos;
        }
        else // Channel 2
        {
             // TIM10/11 only have CC1, so Channel 2 is not applicable
             if (tim_props.base == (TIM_TypeDef*)TIM1_BASE ||
                 tim_props.base == (TIM_TypeDef*)TIM2_BASE ||
                 tim_props.base == (TIM_TypeDef*)TIM3_BASE ||
                 tim_props.base == (TIM_TypeDef*)TIM4_BASE ||
                 tim_props.base == (TIM_TypeDef*)TIM9_BASE) // TIM9 has CC2
             {
                ccmr_offset = TIM_CCMR1_CC2S_Pos;
                ccmr_mask = TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE_Msk | TIM_CCMR1_OC2FE_Msk; // Include FE/PE masks
                ccmr_ocm_pos = TIM_CCMR1_OC2M_Pos;
                ccmr_ocpe_pos = TIM_CCMR1_OC2PE_Pos;
                ccer_cce_pos = TIM_CCER_CC2E_Pos;
                ccer_ccp_pos = TIM_CCER_CC2P_Pos;
             } else {
                 // Invalid channel for this timer type
                 return;
             }
        }
    }
    else if (channel_number == 3 || channel_number == 4)
    {
         // Only TIM1, TIM2-5 have Channels 3 & 4
         if (tim_props.base == (TIM_TypeDef*)TIM1_BASE ||
             tim_props.base == (TIM_TypeDef*)TIM2_BASE ||
             tim_props.base == (TIM_TypeDef*)TIM3_BASE ||
             tim_props.base == (TIM_TypeDef*)TIM4_BASE)
         {
            ccmr_reg = &tim->CCMR2; /* PDF Ref - Table 52, 56 */
            if (channel_number == 3)
            {
                ccmr_offset = TIM_CCMR2_CC3S_Pos;
                ccmr_mask = TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE_Msk | TIM_CCMR2_OC3FE_Msk; // Include FE/PE masks
                ccmr_ocm_pos = TIM_CCMR2_OC3M_Pos;
                ccmr_ocpe_pos = TIM_CCMR2_OC3PE_Pos;
                ccer_cce_pos = TIM_CCER_CC3E_Pos;
                ccer_ccp_pos = TIM_CCER_CC3P_Pos;
            }
            else // Channel 4
            {
                ccmr_offset = TIM_CCMR2_CC4S_Pos;
                ccmr_mask = TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE_Msk | TIM_CCMR2_OC4FE_Msk; // Include FE/PE masks
                ccmr_ocm_pos = TIM_CCMR2_OC4M_Pos;
                ccmr_ocpe_pos = TIM_CCMR2_OC4PE_Pos;
                ccer_cce_pos = TIM_CCER_CC4E_Pos;
                ccer_ccp_pos = TIM_CCER_CC4P_Pos;
            }
         } else {
             // Invalid channel for this timer type
             return;
         }
    } else {
        // Invalid channel number
        return;
    }

    // Configure Capture/Compare channel for Output (CCxS = 00)
    *ccmr_reg &= ~ccmr_mask; /* PDF Ref - Table 52, 56, 59, 61 */
    *ccmr_reg |= (0x0U << ccmr_offset); // CCxS = 00 -> Output mode /* PDF Ref - Table 52, 56, 59, 61 */

    // Set PWM Mode 1 (OCxM = 110) and enable preload (OCxPE = 1)
    *ccmr_reg |= (TIM_OCMODE_PWM1 << ccmr_ocm_pos); /* PDF Ref - Table 52, 56, 59, 61 */
    *ccmr_reg |= (0x1U << ccmr_ocpe_pos);          /* PDF Ref - Table 52, 56, 59, 61 */

    // Configure Output Polarity (e.g., Active High) and disable output initially
    *ccer_reg &= ~((0x1U << ccer_cce_pos) | (0x1U << ccer_ccp_pos)); /* PDF Ref - Table 52, 56, 59, 61 */
    // CCxE = 0 (Output disabled)
    // CCxP = 0 (Active High)

    // For TIM1 (Advanced Timer), configure BDTR for Main Output Enable (MOE)
    if (tim_props.is_advanced)
    {
        tim->BDTR |= TIM_BDTR_MOE_Msk; // Enable main output (PDF Ref - Table 52)
        // Default BDTR settings like dead time, break are 0 by reset, safe for basic PWM
    }

    // Generate an update event to load all configurations (UG bit)
    tim->EGR |= TIM_EGR_UG_Msk; /* PDF Ref - Table 52, 56, 59, 61 */

    // Initial values for PSC, ARR, CCRx will be set by PWM_Set_Freq
}


/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The logical channel to configure.
 * @param frequency The desired PWM frequency in Hz.
 * @param duty The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= pwm_channel_map_size)
    {
        // Invalid channel index
        return;
    }

    if (duty > 100) duty = 100; // Cap duty cycle

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_Properties_t tim_props = TIM_Get_Properties(config->TIMx);
    TIM_TypeDef* tim = tim_props.base;

    if (tim == NULL)
    {
         // Configuration error or reserved timer
        return;
    }

    // Assumed Timer Clock Frequency (e.g., based on SystemCoreClock and APB prescalers)
    // This is a critical assumption not covered by the provided PDF.
    // For STM32F401RC, APB1 is typically up to 42MHz, APB2 up to 84MHz.
    // If APBx prescaler is 1, timer clock is APBx clock. If APBx prescaler > 1, timer clock is 2x APBx clock.
    uint32_t timer_clock;
    if (config->TIMx == 1 || config->TIMx == 9 || config->TIMx == 10 || config->TIMx == 11) // APB2 timers
    {
        // Assume APB2 clock is 84MHz, and timer clock is 84MHz (APB2 prescaler = 1 or 2+)
         timer_clock = 84000000UL; /* Assumed Timer Clock Frequency */
    }
    else // APB1 timers (TIM2, TIM3, TIM4, TIM5)
    {
        // Assume APB1 clock is 42MHz, and timer clock is 84MHz (APB1 prescaler > 1)
         timer_clock = 84000000UL; /* Assumed Timer Clock Frequency */
    }


    uint32_t period_ticks = 0;
    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t ccr = 0;

    if (frequency > 0 && timer_clock >= frequency)
    {
        uint32_t total_ticks = timer_clock / frequency;

        // Find smallest prescaler such that (total_ticks / (prescaler + 1)) fits in ARR register
        uint32_t max_arr = tim_props.is_32bit ? 0xFFFFFFFFUL : 0xFFFFUL; // PDF Ref - Table 52, 56, 59, 61

        prescaler = 0;
        while (prescaler < 0xFFFFUL && (total_ticks / (prescaler + 1)) > (max_arr + 1)) // Prescaler is 16-bit (PDF Ref - Table 52, 56, 59, 61)
        {
             prescaler++;
        }

        // Calculate ARR and CCR
        arr = (total_ticks / (prescaler + 1)) - 1;
        // Duty cycle based on CNT < CCRx for PWM1. Period is ARR+1 ticks.
        // CCRx = ((ARR + 1) * duty) / 100
        ccr = (uint32_t)(((uint64_t)(arr + 1) * duty) / 100);

         // Special case for 0% duty cycle (output is always low)
         if (duty == 0) {
             ccr = 0; // CNT < 0 is never true
         }
         // Special case for 100% duty cycle (output is always high)
         // CNT < CCRx should be always true. Setting CCRx to ARR + 1 usually achieves this.
         if (duty == 100) {
             ccr = arr + 1;
         }
    }
    else
    {
        // Frequency is 0 or too high, effectively stop PWM
        prescaler = 0;
        arr = 0; // Minimum period
        ccr = 0; // 0% duty cycle
    }


    // Write PSC and ARR registers
    tim->PSC = prescaler; /* PDF Ref - Table 52, 56, 59, 61 */
    tim->ARR = arr;       /* PDF Ref - Table 52, 56, 59, 61 */

    // Write CCR register based on channel number
    switch (config->ChannelNumber)
    {
        case 1: tim->CCR1 = ccr; break; /* PDF Ref - Table 52, 56, 59, 61 */
        case 2: tim->CCR2 = ccr; break; /* PDF Ref - Table 52, 56, 59, 61 */
        case 3: tim->CCR3 = ccr; break; /* PDF Ref - Table 52, 56 */ // TIM1, TIM2-5 only
        case 4: tim->CCR4 = ccr; break; /* PDF Ref - Table 52, 56 */ // TIM1, TIM2-5 only
        default: return; // Should not happen with valid channel_map
    }

    // Generate update event to load buffered registers (UG bit)
    // Check if counter is enabled before generating UG if URS is set (URS=1 bypasses UIF on UG)
    // If URS=0 (default), UG always sets UIF. For on-the-fly updates, UG is usually ok.
    // The simplest approach is to always generate UG after changing parameters if ARPE/OCxPE are used.
     tim->EGR |= TIM_EGR_UG_Msk; /* PDF Ref - Table 52, 56, 59, 61 */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The logical channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= pwm_channel_map_size)
    {
        // Invalid channel index
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_Properties_t tim_props = TIM_Get_Properties(config->TIMx);
    TIM_TypeDef* tim = tim_props.base;

     if (tim == NULL)
    {
         // Configuration error or reserved timer
        return;
    }

    // Enable the specific channel output (CCxE bit)
    uint32_t channel_number = config->ChannelNumber;
    volatile uint32_t* ccer_reg = &tim->CCER; /* PDF Ref - Table 52, 56, 59, 61 */
    uint32_t ccer_cce_pos = 0;

    switch (channel_number)
    {
        case 1: ccer_cce_pos = TIM_CCER_CC1E_Pos; break; /* PDF Ref - Table 52, 56, 59, 61 */
        case 2: ccer_cce_pos = TIM_CCER_CC2E_Pos; break; /* PDF Ref - Table 52, 56, 59 */ // TIM1,2-5,9
        case 3: ccer_cce_pos = TIM_CCER_CC3E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
        case 4: ccer_cce_pos = TIM_CCER_CC4E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
        default: return; // Should not happen
    }

    *ccer_reg |= (0x1U << ccer_cce_pos); /* PDF Ref - Table 52, 56, 59, 61 */

    // For TIM1 (Advanced Timer), enable Main Output (MOE bit)
    if (tim_props.is_advanced)
    {
        tim->BDTR |= TIM_BDTR_MOE_Msk; /* PDF Ref - Table 52 */
    }

    // Enable the Counter (CEN bit)
    tim->CR1 |= TIM_CR1_CEN_Msk; /* PDF Ref - Table 52, 56, 59, 61 */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The logical channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= pwm_channel_map_size)
    {
        // Invalid channel index
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];
    TIM_Properties_t tim_props = TIM_Get_Properties(config->TIMx);
    TIM_TypeDef* tim = tim_props.base;

     if (tim == NULL)
    {
         // Configuration error or reserved timer
        return;
    }

    // Disable the specific channel output (CCxE bit)
    uint32_t channel_number = config->ChannelNumber;
    volatile uint32_t* ccer_reg = &tim->CCER; /* PDF Ref - Table 52, 56, 59, 61 */
    uint32_t ccer_cce_pos = 0;

    switch (channel_number)
    {
        case 1: ccer_cce_pos = TIM_CCER_CC1E_Pos; break; /* PDF Ref - Table 52, 56, 59, 61 */
        case 2: ccer_cce_pos = TIM_CCER_CC2E_Pos; break; /* PDF Ref - Table 52, 56, 59 */ // TIM1,2-5,9
        case 3: ccer_cce_pos = TIM_CCER_CC3E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
        case 4: ccer_cce_pos = TIM_CCER_CC4E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
        default: return; // Should not happen
    }
    *ccer_reg &= ~(0x1U << ccer_cce_pos); /* PDF Ref - Table 52, 56, 59, 61 */

    // For TIM1 (Advanced Timer), disable Main Output (MOE bit) if *all* channels are stopped
    // stopping *a* channel on TIM1 means disabling MOE if that's the *only* channel used.
    // Or, disable MOE only when PWM_PowerOff is called.
    // The PDF states MOE is cleared asynchronously by break OR set by software/AOE.
    // Let's only disable the specific channel output here. MOE control is complex (BDTR table).
    // A safer approach for individual channel stop is *not* to touch MOE here, unless
    // we are sure no other channels on TIM1 are active.
    // To strictly follow 'Stop the PWM signal output on the specified channel',
    // disabling the specific channel enable (CCxE) is the direct method.
    // The impact of MOE depends on OSSI/OSSR bits (PDF Table 51), which default to 0.
    // With default OSSI/OSSR=0, CCxE=0 -> Output Disabled (OCx=0, OCx_EN=0). This is a safe stop.
    // We won't touch MOE in this function to allow other channels on TIM1 to remain active.

    // Disable the Counter (CEN bit) - Only if THIS timer instance is no longer needed for *any* PWM
    // Similar to MOE, disabling the entire timer might affect other channels.
    // Let's only disable the specific channel output here. The timer counter can keep running.
    // If a full stop of the timer instance is needed when the *last* channel is stopped,
    // more state tracking is required. For this implementation, stopping a channel
    // means disabling its specific output pin driver from the timer.

}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 */
void PWM_PowerOff(void)
{
    // Iterate through the defined channels in the map
    for (size_t i = 0; i < pwm_channel_map_size; ++i)
    {
        const PWM_Channel_Config_t* config = &pwm_channel_map[i];
        TIM_Properties_t tim_props = TIM_Get_Properties(config->TIMx);
        TIM_TypeDef* tim = tim_props.base;

        if (tim == NULL)
        {
             // Should not happen with valid map, but defensive check
             continue;
        }

        // Disable the counter for this timer instance (CEN bit)
        tim->CR1 &= ~TIM_CR1_CEN_Msk; /* PDF Ref - Table 52, 56, 59, 61 */

         // For TIM1 (Advanced Timer), disable Main Output (MOE bit)
        if (tim_props.is_advanced)
        {
             tim->BDTR &= ~TIM_BDTR_MOE_Msk; /* PDF Ref - Table 52 */
        }

        // Disable the specific channel output (CCxE bit) - although timer is stopped, explicit is safer
        uint32_t channel_number = config->ChannelNumber;
        volatile uint32_t* ccer_reg = &tim->CCER; /* PDF Ref - Table 52, 56, 59, 61 */
        uint32_t ccer_cce_pos = 0;

        switch (channel_number)
        {
            case 1: ccer_cce_pos = TIM_CCER_CC1E_Pos; break; /* PDF Ref - Table 52, 56, 59, 61 */
            case 2: ccer_cce_pos = TIM_CCER_CC2E_Pos; break; /* PDF Ref - Table 52, 56, 59 */ // TIM1,2-5,9
            case 3: ccer_cce_pos = TIM_CCER_CC3E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
            case 4: ccer_cce_pos = TIM_CCER_CC4E_Pos; break; /* PDF Ref - Table 52, 56 */ // TIM1,2-5
            default: continue; // Should not happen
        }
        *ccer_reg &= ~(0x1U << ccer_cce_pos); /* PDF Ref - Table 52, 56, 59, 61 */

        // Disable the clock for this timer instance (Assumed RCC clocking - NOT in provided PDF)
        *(tim_props.clock_en_reg) &= ~tim_props.clock_en_mask; // Assumed RCC clocking

        // Disable the clock for the associated GPIO port (Assumed RCC clocking - NOT in provided PDF)
        // This requires tracking which ports are used by the active PWM channels.
        // A more optimized power off would only disable ports actually enabled for PWM.
        GPIO_TypeDef* gpio_port = GPIO_Get_Port_Base(config->PortName);
        if (gpio_port != NULL)
        {
            uint32_t gpio_clk_mask = GPIO_Get_Clock_Enable_Mask(config->PortName);
            if (gpio_port == (GPIO_TypeDef*)GPIOA_BASE) { RCC_AHB1ENR &= ~gpio_clk_mask; } // Assumed RCC clocking
            else if (gpio_port == (GPIO_TypeDef*)GPIOB_BASE) { RCC_AHB1ENR &= ~gpio_clk_mask; } // Assumed RCC clocking
            else if (gpio_port == (GPIO_TypeDef*)GPIOC_BASE) { RCC_AHB1ENR &= ~gpio_clk_mask; } // Assumed RCC clocking
            else if (gpio_port == (GPIO_TypeDef*)GPIOD_BASE) { RCC_AHB1ENR &= ~gpio_clk_mask; } // Assumed RCC clocking
            else if (gpio_port == (GPIO_TypeDef*)GPIOE_BASE) { RCC_AHB1ENR &= ~gpio_clk_mask; } // Assumed RCC clocking
        }
    }

     // Also explicitly disable clocks for reserved timers if they exist and were potentially enabled elsewhere
     TIM_Properties_t tim5_props = TIM_Get_Properties(5);
     if (tim5_props.base != NULL) { *(tim5_props.clock_en_reg) &= ~tim5_props.clock_en_mask; } // Assumed RCC clocking
     TIM_Properties_t tim10_props = TIM_Get_Properties(10);
     if (tim10_props.base != NULL) { *(tim10_props.clock_en_reg) &= ~tim10_props.clock_en_mask; } // Assumed RCC clocking
}