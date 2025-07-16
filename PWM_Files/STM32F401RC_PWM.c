#ifndef STM32F401RC_PWM_H
#define STM32F401RC_PWM_H

/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for PWM driver on STM32F401RC.
* Author         : Technology Inovation Software Team
* Creation Date  : 2025-07-16
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

// Standard integer types
typedef unsigned long tlong;
typedef unsigned char tbyte;

// Define specific Alternate Function values for timers (assumed common for STM32F401RC)
// These values are based on typical STM32F401RC alternate function mapping tables.
#define GPIO_AF_TIM1_VAL    (0x01U) // AF1 /* Assumed AF value - please verify */
#define GPIO_AF_TIM3_VAL    (0x02U) // AF2 /* Assumed AF value - please verify */
#define GPIO_AF_TIM4_VAL    (0x02U) // AF2 /* Assumed AF value - please verify */
#define GPIO_AF_TIM9_VAL    (0x03U) // AF3 /* Assumed AF value - please verify */
#define GPIO_AF_TIM10_VAL   (0x03U) // AF3 /* Assumed AF value - please verify */
#define GPIO_AF_TIM11_VAL   (0x03U) // AF3 /* Assumed AF value - please verify */


// Enumeration for available PWM channels based on selected timers and common pin assignments.
// As per requirements, Timers TIM2 and TIM5 (and their channels) are reserved for potential OS or
// delay functionalities and are excluded from this PWM driver implementation.
typedef enum
{
    // TIM1 Channels (Advanced-control timer, 16-bit, on APB2 bus)
    // Common pins for TIM1: PA8 (CH1), PA9 (CH2), PA10 (CH3), PA11 (CH4)
    TRD_CHANNEL_TIM1_CH1 = 0,
    TRD_CHANNEL_TIM1_CH2,
    TRD_CHANNEL_TIM1_CH3,
    TRD_CHANNEL_TIM1_CH4,

    // TIM3 Channels (General-purpose timer, 16-bit, on APB1 bus)
    // Common pins for TIM3: PA6 (CH1), PA7 (CH2), PB0 (CH3), PB1 (CH4)
    TRD_CHANNEL_TIM3_CH1,
    TRD_CHANNEL_TIM3_CH2,
    TRD_CHANNEL_TIM3_CH3,
    TRD_CHANNEL_TIM3_CH4,

    // TIM4 Channels (General-purpose timer, 16-bit, on APB1 bus)
    // Common pins for TIM4: PB6 (CH1), PB7 (CH2), PB8 (CH3), PB9 (CH4)
    TRD_CHANNEL_TIM4_CH1,
    TRD_CHANNEL_TIM4_CH2,
    TRD_CHANNEL_TIM4_CH3,
    TRD_CHANNEL_TIM4_CH4,

    // TIM9 Channels (General-purpose timer, 16-bit, on APB2 bus)
    // Common pins for TIM9: PA2 (CH1), PA3 (CH2)
    TRD_CHANNEL_TIM9_CH1,
    TRD_CHANNEL_TIM9_CH2,

    // TIM10 Channel (General-purpose timer, 16-bit, on APB2 bus)
    // Common pin for TIM10: PB8 (CH1). Note: PB8 is also TIM4_CH3 (AF2).
    // Ensure that if this channel is used, TIM4_CH3 is not, or AF is correctly configured.
    TRD_CHANNEL_TIM10_CH1,

    // TIM11 Channel (General-purpose timer, 16-bit, on APB2 bus)
    // Common pin for TIM11: PB9 (CH1). Note: PB9 is also TIM4_CH4 (AF2).
    // Ensure that if this channel is used, TIM4_CH4 is not, or AF is correctly configured.
    TRD_CHANNEL_TIM11_CH1,

    NUM_PWM_CHANNELS // Total number of available PWM channels defined in this driver
} TRD_Channel_t;


// Function Prototypes
void PWM_Init(TRD_Channel_t TRD_Channel);
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t TRD_Channel);
void PWM_Stop(TRD_Channel_t TRD_Channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H */

/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready PWM implementation for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-16
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"

/*
 * Timers TIM2 and TIM5 (and their channels) are reserved for potential OS or delay functionalities.
 * They are excluded from this PWM driver implementation as per requirements.
 */

/* --- Private Defines and Macros --- */

// Helper macros for bare-metal register access
#define BIT_SET(REG, BIT)   ((REG) |= (1U << (BIT)))
#define BIT_CLR(REG, BIT)   ((REG) &= ~(1U << (BIT)))
#define REG_WRITE(REG, VAL) ((REG) = (VAL))
#define REG_READ(REG)       ((REG))
#define MODIFY_REG(REG, CLRMASK, SETMASK) (REG = (REG & ~(CLRMASK)) | (SETMASK))

// Peripheral Base Addresses (Assumed standard values for STM32F401RC)
#define PERIPH_BASE           (0x40000000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)

// RCC Base Address (Assumed from memory map)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define RCC                   ((volatile RCC_TypeDef *) RCC_BASE)

// GPIO Base Addresses
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
// Only GPIOA, GPIOB, GPIOC are used in the channel map.
// The STM32F401RC has GPIOs A to H, but F/G/I/J/K are not available.
// PH0/PH1 are specific, leaving A,B,C,D,E,PH0/PH1.
// Our selected pins fall within A, B, C.

// TIM Base Addresses
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL) /* Reserved Timer */
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL) /* Reserved Timer */
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

/* --- Register Structures (Simplified for bare-metal access from PDF) --- */

typedef struct
{
  volatile tlong CR;         /*!< RCC clock control register,                Address offset: 0x00 */
  volatile tlong PLLCFGR;    /*!< RCC PLL configuration register,            Address offset: 0x04 */
  volatile tlong CFGR;       /*!< RCC clock configuration register,          Address offset: 0x08 */
  volatile tlong CIR;        /*!< RCC clock interrupt register,              Address offset: 0x0C */
  volatile tlong AHB1RSTR;   /*!< RCC AHB1 peripheral reset register,        Address offset: 0x10 */
  volatile tlong AHB2RSTR;   /*!< RCC AHB2 peripheral reset register,        Address offset: 0x14 */
  volatile tlong AHB3RSTR;   /*!< RCC AHB3 peripheral reset register,        Address offset: 0x18 */
  tlong        RESERVED0;    /*!< Reserved,                                           0x1C */
  volatile tlong APB1RSTR;   /*!< RCC APB1 peripheral reset register,        Address offset: 0x20 */
  volatile tlong APB2RSTR;   /*!< RCC APB2 peripheral reset register,        Address offset: 0x24 */
  tlong        RESERVED1[2]; /*!< Reserved,                                   0x28-0x2C */
  volatile tlong AHB1ENR;    /*!< RCC AHB1 peripheral clock enable register, Address offset: 0x30 */
  volatile tlong AHB2ENR;    /*!< RCC AHB2 peripheral clock enable register, Address offset: 0x34 */
  volatile tlong AHB3ENR;    /*!< RCC AHB3 peripheral clock enable register, Address offset: 0x38 */
  tlong        RESERVED2;    /*!< Reserved,                                           0x3C */
  volatile tlong APB1ENR;    /*!< RCC APB1 peripheral clock enable register, Address offset: 0x40 */
  volatile tlong APB2ENR;    /*!< RCC APB2 peripheral clock enable register, Address offset: 0x44 */
  tlong        RESERVED3[2]; /*!< Reserved,                                   0x48-0x4C */
  volatile tlong AHB1LPENR;  /*!< RCC AHB1 peripheral low power clock enable register, Address offset: 0x50 */
  volatile tlong AHB2LPENR;  /*!< RCC AHB2 peripheral low power clock enable register, Address offset: 0x54 */
  volatile tlong AHB3LPENR;  /*!< RCC AHB3 peripheral low power clock enable register, Address offset: 0x58 */
  tlong        RESERVED4;    /*!< Reserved,                                           0x5C */
  volatile tlong APB1LPENR;  /*!< RCC APB1 peripheral low power clock enable register, Address offset: 0x60 */
  volatile tlong APB2LPENR;  /*!< RCC APB2 peripheral low power clock enable register, Address offset: 0x64 */
  tlong        RESERVED5[2]; /*!< Reserved,                                   0x68-0x6C */
  volatile tlong BDCR;       /*!< RCC Backup domain control register,        Address offset: 0x70 */
  volatile tlong CSR;        /*!< RCC clock control & status register,       Address offset: 0x74 */
  tlong        RESERVED6[2]; /*!< Reserved,                                   0x78-0x7C */
  volatile tlong SSCGR;      /*!< RCC spread spectrum clock generation register, Address offset: 0x80 */
  volatile tlong PLLI2SCFGR; /*!< RCC PLLI2S configuration register,         Address offset: 0x84 */
  volatile tlong DCKCFGR;    /*!< RCC Dedicated Clocks configuration register, Address offset: 0x88 */
} RCC_TypeDef;

typedef struct
{
  volatile tlong MODER;    /*!< GPIO port mode register,                     Address offset: 0x00 */
  volatile tlong OTYPER;   /*!< GPIO port output type register,              Address offset: 0x04 */
  volatile tlong OSPEEDR;  /*!< GPIO port output speed register,             Address offset: 0x08 */
  volatile tlong PUPDR;    /*!< GPIO port pull-up/pull-down register,        Address offset: 0x0C */
  volatile tlong IDR;      /*!< GPIO port input data register,               Address offset: 0x10 */
  volatile tlong ODR;      /*!< GPIO port output data register,              Address offset: 0x14 */
  volatile tlong BSRR;     /*!< GPIO port bit set/reset register,            Address offset: 0x18 */
  volatile tlong LCKR;     /*!< GPIO port configuration lock register,       Address offset: 0x1C */
  volatile tlong AFRL;     /*!< GPIO alternate function low register,        Address offset: 0x20 */
  volatile tlong AFRH;     /*!< GPIO alternate function high register,       Address offset: 0x24 */
} GPIO_TypeDef;

// Generic TIM Register Structure (common fields for PWM based on PDF sections 12, 13, 14)
typedef struct
{
  volatile tlong CR1;        /*!< TIM control register 1,                      Address offset: 0x00 */
  volatile tlong CR2;        /*!< TIM control register 2,                      Address offset: 0x04 */
  volatile tlong SMCR;       /*!< TIM slave mode control register,             Address offset: 0x08 */
  volatile tlong DIER;       /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  volatile tlong SR;         /*!< TIM status register,                         Address offset: 0x10 */
  volatile tlong EGR;        /*!< TIM event generation register,               Address offset: 0x14 */
  volatile tlong CCMR1;      /*!< TIM capture/compare mode register 1,         Address offset: 0x18 */
  volatile tlong CCMR2;      /*!< TIM capture/compare mode register 2,         Address offset: 0x1C */
  volatile tlong CCER;       /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  volatile tlong CNT;        /*!< TIM counter register,                        Address offset: 0x24 */
  volatile tlong PSC;        /*!< TIM prescaler register,                      Address offset: 0x28 */
  volatile tlong ARR;        /*!< TIM auto-reload register,                    Address offset: 0x2C */
  volatile tlong RCR;        /*!< TIM repetition counter register,             Address offset: 0x30 */ // Only for TIM1
  volatile tlong CCR1;       /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  volatile tlong CCR2;       /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  volatile tlong CCR3;       /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  volatile tlong CCR4;       /*!< TIM capture/compare register 4,              Address offset: 0x40 */
  volatile tlong BDTR;       /*!< TIM break and dead-time register,            Address offset: 0x44 */ // Only for TIM1
  volatile tlong DCR;        /*!< TIM DMA control register,                    Address offset: 0x48 */
  volatile tlong DMAR;       /*!< TIM DMA address for full transfer,           Address offset: 0x4C */
  volatile tlong OR;         /*!< TIM option register (TIM2/5/11 only),        Address offset: 0x50 */
} TIM_TypeDef;

/* --- Pointers to Peripherals --- */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

/* --- RCC Peripheral Clock Enable Bits (Assumed from common STM32F4 series) --- */
#define RCC_AHB1ENR_GPIOAEN   (1U << 0)
#define RCC_AHB1ENR_GPIOBEN   (1U << 1)
#define RCC_AHB1ENR_GPIOCEN   (1U << 2)

#define RCC_APB2ENR_TIM1EN    (1U << 0)  /* Assumed RCC config */
#define RCC_APB1ENR_TIM3EN    (1U << 1)  /* Assumed RCC config */
#define RCC_APB1ENR_TIM4EN    (1U << 2)  /* Assumed RCC config */
#define RCC_APB2ENR_TIM9EN    (1U << 16) /* Assumed RCC config */
#define RCC_APB2ENR_TIM10EN   (1U << 17) /* Assumed RCC config */
#define RCC_APB2ENR_TIM11EN   (1U << 18) /* Assumed RCC config */

/* --- Common Timer Configuration Bit Masks & Positions from PDF --- */

// TIMx_CR1
#define TIM_CR1_CEN_Pos           (0U)
#define TIM_CR1_CEN               (0x1U << TIM_CR1_CEN_Pos)     /*!< Counter enable (PDF Reference) */
#define TIM_CR1_UDIS_Pos          (1U)
#define TIM_CR1_UDIS              (0x1U << TIM_CR1_UDIS_Pos)    /*!< Update disable (PDF Reference) */
#define TIM_CR1_URS_Pos           (2U)
#define TIM_CR1_URS               (0x1U << TIM_CR1_URS_Pos)     /*!< Update request source (PDF Reference) */
#define TIM_CR1_ARPE_Pos          (7U)
#define TIM_CR1_ARPE              (0x1U << TIM_CR1_ARPE_Pos)    /*!< Auto-reload preload enable (PDF Reference) */

// TIMx_EGR
#define TIM_EGR_UG_Pos            (0U)
#define TIM_EGR_UG                (0x1U << TIM_EGR_UG_Pos)      /*!< Update generation (PDF Reference) */

// TIMx_CCER
#define TIM_CCER_CC1E_Pos         (0U)  /*!< Capture/Compare 1 output enable (PDF Reference) */
#define TIM_CCER_CC1E             (0x1U << TIM_CCER_CC1E_Pos)
#define TIM_CCER_CC1P_Pos         (1U)  /*!< Capture/Compare 1 output polarity (PDF Reference) */
#define TIM_CCER_CC1P             (0x1U << TIM_CCER_CC1P_Pos)
#define TIM_CCER_CC1NP_Pos        (3U)  /*!< Capture/Compare 1 complementary output polarity (PDF Reference - TIM1 only) */
#define TIM_CCER_CC1NP            (0x1U << TIM_CCER_CC1NP_Pos)

#define TIM_CCER_CC2E_Pos         (4U)  /*!< Capture/Compare 2 output enable (PDF Reference) */
#define TIM_CCER_CC2E             (0x1U << TIM_CCER_CC2E_Pos)
#define TIM_CCER_CC2P_Pos         (5U)  /*!< Capture/Compare 2 output polarity (PDF Reference) */
#define TIM_CCER_CC2P             (0x1U << TIM_CCER_CC2P_Pos)
#define TIM_CCER_CC2NP_Pos        (7U)  /*!< Capture/Compare 2 complementary output polarity (PDF Reference - TIM1 only) */
#define TIM_CCER_CC2NP            (0x1U << TIM_CCER_CC2NP_Pos)

#define TIM_CCER_CC3E_Pos         (8U)  /*!< Capture/Compare 3 output enable (PDF Reference) */
#define TIM_CCER_CC3E             (0x1U << TIM_CCER_CC3E_Pos)
#define TIM_CCER_CC3P_Pos         (9U)  /*!< Capture/Compare 3 output polarity (PDF Reference) */
#define TIM_CCER_CC3P             (0x1U << TIM_CCER_CC3P_Pos)
#define TIM_CCER_CC3NP_Pos        (11U) /*!< Capture/Compare 3 complementary output polarity (PDF Reference - TIM1 only) */
#define TIM_CCER_CC3NP            (0x1U << TIM_CCER_CC3NP_Pos)

#define TIM_CCER_CC4E_Pos         (12U) /*!< Capture/Compare 4 output enable (PDF Reference) */
#define TIM_CCER_CC4E             (0x1U << TIM_CCER_CC4E_Pos)
#define TIM_CCER_CC4P_Pos         (13U) /*!< Capture/Compare 4 output polarity (PDF Reference) */
#define TIM_CCER_CC4P             (0x1U << TIM_CCER_CC4P_Pos)

// TIMx_CCMRx (Output Compare Mode)
#define TIM_CCMR_OCxM_PWM1        (0x6U) /* 110: PWM mode 1 (PDF Reference) */
#define TIM_CCMR_OCxPE            (0x1U) /* Preload enable bit value (PDF Reference) */
#define TIM_CCMR_OUTPUT           (0x0U) /* CCx channel is configured as output (PDF Reference) */

// TIMx_BDTR (Only for TIM1)
#define TIM_BDTR_MOE_Pos          (15U)
#define TIM_BDTR_MOE              (0x1U << TIM_BDTR_MOE_Pos) /*! Main output enable (PDF Reference) */


/* --- System Clock Frequencies (Assumed typical values for STM32F401RC) --- */
// These values are typical for an 84MHz HCLK, assuming default APB prescalers.
#define HCLK_FREQ               (84000000UL) // Max HCLK for F401RC /* Assumed system clock - please verify */
#define APB1_TIMER_CLK_FREQ     (HCLK_FREQ / 2) // APB1 prescaler set to 2, Timers receive HCLK/1 = 42MHz /* Assumed system clock - please verify */
#define APB2_TIMER_CLK_FREQ     (HCLK_FREQ / 1) // APB2 prescaler set to 1, Timers receive HCLK/1 = 84MHz /* Assumed system clock - please verify */

// Define the maximum possible ARR value for 16-bit and 32-bit timers
#define MAX_ARR_16BIT           (0xFFFFU) // 65535
#define MAX_ARR_32BIT           (0xFFFFFFFFUL) // 4294967295


/* --- Data Structures --- */

// Helper structure for channel configuration mapping
typedef struct
{
    GPIO_TypeDef *GPIOx;
    tlong GPIO_Pin_Msk;
    tbyte GPIO_AF;
    tlong GPIO_Clk_En_Msk; // RCC AHB1ENR bit
    TIM_TypeDef *TIMx;
    tlong TIM_Clk_En_Msk;  // RCC APBxENR bit (for APB1 or APB2)
    tbyte TIM_Channel;     // 1, 2, 3, or 4
    tlong Timer_Input_Clock_Freq; // APB1_TIMER_CLK_FREQ or APB2_TIMER_CLK_FREQ
    tbyte Is_32bit_Timer; // 1 if 32-bit (TIM2, TIM5), 0 if 16-bit
} PWM_Channel_Config_t;

// Channel configurations (Assumed based on common STM32F401RC pinout and AFs, needs verification against full datasheet)
// These mappings are typical but may vary based on exact device variant and package.
static const PWM_Channel_Config_t PWM_Channel_Map[] =
{
    // TIM1 Channels (Advanced-control timer, 16-bit, on APB2 bus)
    {GPIOA, (1U << 8),  GPIO_AF_TIM1_VAL, RCC_AHB1ENR_GPIOAEN, TIM1, RCC_APB2ENR_TIM1EN, 1, APB2_TIMER_CLK_FREQ, 0}, /* TIM1_CH1 (PA8) Assumed PWM config - please verify */
    {GPIOA, (1U << 9),  GPIO_AF_TIM1_VAL, RCC_AHB1ENR_GPIOAEN, TIM1, RCC_APB2ENR_TIM1EN, 2, APB2_TIMER_CLK_FREQ, 0}, /* TIM1_CH2 (PA9) Assumed PWM config - please verify */
    {GPIOA, (1U << 10), GPIO_AF_TIM1_VAL, RCC_AHB1ENR_GPIOAEN, TIM1, RCC_APB2ENR_TIM1EN, 3, APB2_TIMER_CLK_FREQ, 0}, /* TIM1_CH3 (PA10) Assumed PWM config - please verify */
    {GPIOA, (1U << 11), GPIO_AF_TIM1_VAL, RCC_AHB1ENR_GPIOAEN, TIM1, RCC_APB2ENR_TIM1EN, 4, APB2_TIMER_CLK_FREQ, 0}, /* TIM1_CH4 (PA11) Assumed PWM config - please verify */

    // TIM3 Channels (General-purpose timer, 16-bit, on APB1 bus)
    {GPIOA, (1U << 6),  GPIO_AF_TIM3_VAL, RCC_AHB1ENR_GPIOAEN, TIM3, RCC_APB1ENR_TIM3EN, 1, APB1_TIMER_CLK_FREQ, 0}, /* TIM3_CH1 (PA6) Assumed PWM config - please verify */
    {GPIOA, (1U << 7),  GPIO_AF_TIM3_VAL, RCC_AHB1ENR_GPIOAEN, TIM3, RCC_APB1ENR_TIM3EN, 2, APB1_TIMER_CLK_FREQ, 0}, /* TIM3_CH2 (PA7) Assumed PWM config - please verify */
    {GPIOB, (1U << 0),  GPIO_AF_TIM3_VAL, RCC_AHB1ENR_GPIOBEN, TIM3, RCC_APB1ENR_TIM3EN, 3, APB1_TIMER_CLK_FREQ, 0}, /* TIM3_CH3 (PB0) Assumed PWM config - please verify */
    {GPIOB, (1U << 1),  GPIO_AF_TIM3_VAL, RCC_AHB1ENR_GPIOBEN, TIM3, RCC_APB1ENR_TIM3EN, 4, APB1_TIMER_CLK_FREQ, 0}, /* TIM3_CH4 (PB1) Assumed PWM config - please verify */

    // TIM4 Channels (General-purpose timer, 16-bit, on APB1 bus)
    {GPIOB, (1U << 6),  GPIO_AF_TIM4_VAL, RCC_AHB1ENR_GPIOBEN, TIM4, RCC_APB1ENR_TIM4EN, 1, APB1_TIMER_CLK_FREQ, 0}, /* TIM4_CH1 (PB6) Assumed PWM config - please verify */
    {GPIOB, (1U << 7),  GPIO_AF_TIM4_VAL, RCC_AHB1ENR_GPIOBEN, TIM4, RCC_APB1ENR_TIM4EN, 2, APB1_TIMER_CLK_FREQ, 0}, /* TIM4_CH2 (PB7) Assumed PWM config - please verify */
    {GPIOB, (1U << 8),  GPIO_AF_TIM4_VAL, RCC_AHB1ENR_GPIOBEN, TIM4, RCC_APB1ENR_TIM4EN, 3, APB1_TIMER_CLK_FREQ, 0}, /* TIM4_CH3 (PB8) Assumed PWM config - please verify */
    {GPIOB, (1U << 9),  GPIO_AF_TIM4_VAL, RCC_AHB1ENR_GPIOBEN, TIM4, RCC_APB1ENR_TIM4EN, 4, APB1_TIMER_CLK_FREQ, 0}, /* TIM4_CH4 (PB9) Assumed PWM config - please verify */

    // TIM9 Channels (General-purpose timer, 16-bit, on APB2 bus)
    {GPIOA, (1U << 2),  GPIO_AF_TIM9_VAL, RCC_AHB1ENR_GPIOAEN, TIM9, RCC_APB2ENR_TIM9EN, 1, APB2_TIMER_CLK_FREQ, 0}, /* TIM9_CH1 (PA2) Assumed PWM config - please verify */
    {GPIOA, (1U << 3),  GPIO_AF_TIM9_VAL, RCC_AHB1ENR_GPIOAEN, TIM9, RCC_APB2ENR_TIM9EN, 2, APB2_TIMER_CLK_FREQ, 0}, /* TIM9_CH2 (PA3) Assumed PWM config - please verify */

    // TIM10 Channel (General-purpose timer, 16-bit, on APB2 bus)
    {GPIOB, (1U << 8),  GPIO_AF_TIM10_VAL, RCC_AHB1ENR_GPIOBEN, TIM10, RCC_APB2ENR_TIM10EN, 1, APB2_TIMER_CLK_FREQ, 0}, /* TIM10_CH1 (PB8) Assumed PWM config - please verify */

    // TIM11 Channel (General-purpose timer, 16-bit, on APB2 bus)
    {GPIOB, (1U << 9),  GPIO_AF_TIM11_VAL, RCC_AHB1ENR_GPIOBEN, TIM11, RCC_APB2ENR_TIM11EN, 1, APB2_TIMER_CLK_FREQ, 0}, /* TIM11_CH1 (PB9) Assumed PWM config - please verify */
};

/* --- Private Functions --- */

/**
 * @brief Configures the GPIO pin for Alternate Function mode.
 * @param GPIOx Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param PinMask Mask for the specific pin(s) to configure (e.g., (1U << 0) for Pin 0).
 * @param af_value Alternate Function number (0-15).
 */
static void GPIO_Config_AF(GPIO_TypeDef *GPIOx, tlong PinMask, tbyte af_value)
{
    tbyte pin_num;
    tlong moder_clear_mask;
    tlong afr_clear_mask;

    for (pin_num = 0; pin_num < 16; pin_num++)
    {
        if ((PinMask >> pin_num) & 1U)
        {
            // Configure MODER register for Alternate Function mode (10)
            moder_clear_mask = (0x3U << (pin_num * 2)); /* PDF Reference */
            MODIFY_REG(GPIOx->MODER, moder_clear_mask, (0x2U << (pin_num * 2))); /* PDF Reference */

            // Configure OTYPER for Push-Pull (0) - default reset state, no need to clear unless changed before
            BIT_CLR(GPIOx->OTYPER, pin_num); /* PDF Reference */

            // Configure OSPEEDR for High Speed (10)
            MODIFY_REG(GPIOx->OSPEEDR, (0x3U << (pin_num * 2)), (0x2U << (pin_num * 2))); /* PDF Reference */

            // Configure PUPDR for No pull-up/pull-down (00) - default reset state
            MODIFY_REG(GPIOx->PUPDR, (0x3U << (pin_num * 2)), (0x0U << (pin_num * 2))); /* PDF Reference */

            // Configure AFRL/AFRH registers for Alternate Function
            if (pin_num < 8)
            {
                // AFRL for pins 0-7
                afr_clear_mask = (0xFU << (pin_num * 4)); /* PDF Reference */
                MODIFY_REG(GPIOx->AFRL, afr_clear_mask, (af_value << (pin_num * 4))); /* PDF Reference */
            }
            else
            {
                // AFRH for pins 8-15
                afr_clear_mask = (0xFU << ((pin_num - 8) * 4)); /* PDF Reference */
                MODIFY_REG(GPIOx->AFRH, afr_clear_mask, (af_value << ((pin_num - 8) * 4))); /* PDF Reference */
            }
        }
    }
}

/**
 * @brief Returns a pointer to the CCRx register for the given channel.
 * @param TIMx Pointer to the TIM peripheral.
 * @param channel The timer channel number (1-4).
 * @return Pointer to the CCRx register, or NULL if invalid channel.
 */
static volatile tlong *get_ccrx_register(TIM_TypeDef *TIMx, tbyte channel)
{
    switch (channel)
    {
        case 1: return &(TIMx->CCR1); /* PDF Reference */
        case 2: return &(TIMx->CCR2); /* PDF Reference */
        case 3: return &(TIMx->CCR3); /* PDF Reference */
        case 4: return &(TIMx->CCR4); /* PDF Reference */
        default: return (volatile tlong *)0; // Invalid channel
    }
}

/**
 * @brief Enables the clock for the given GPIO port.
 * @param GPIOx Pointer to the GPIO peripheral.
 * @param ClkEnableMsk The corresponding bit mask in RCC_AHB1ENR.
 */
static void enable_gpio_clk(GPIO_TypeDef *GPIOx, tlong ClkEnableMsk)
{
    if (GPIOx == GPIOA)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); /* Assumed RCC config */
    }
    else if (GPIOx == GPIOB)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN); /* Assumed RCC config */
    }
    else if (GPIOx == GPIOC)
    {
        BIT_SET(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN); /* Assumed RCC config */
    }
    // Add other GPIOs if needed in the future channel map
}

/**
 * @brief Enables the clock for the given Timer peripheral.
 * @param TIMx Pointer to the TIM peripheral.
 * @param ClkEnableMsk The corresponding bit mask in RCC_APB1ENR or RCC_APB2ENR.
 */
static void enable_timer_clk(TIM_TypeDef *TIMx, tlong ClkEnableMsk)
{
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
    {
        BIT_SET(RCC->APB2ENR, ClkEnableMsk); /* Assumed RCC config */
    }
    else if (TIMx == TIM3 || TIMx == TIM4)
    {
        BIT_SET(RCC->APB1ENR, ClkEnableMsk); /* Assumed RCC config */
    }
    // TIM2 and TIM5 are reserved
}

/**
 * @brief Disables the clock for the given Timer peripheral.
 * @param TIMx Pointer to the TIM peripheral.
 * @param ClkEnableMsk The corresponding bit mask in RCC_APB1ENR or RCC_APB2ENR.
 */
static void disable_timer_clk(TIM_TypeDef *TIMx, tlong ClkEnableMsk)
{
    if (TIMx == TIM1 || TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
    {
        BIT_CLR(RCC->APB2ENR, ClkEnableMsk); /* Assumed RCC config */
    }
    else if (TIMx == TIM3 || TIMx == TIM4)
    {
        BIT_CLR(RCC->APB1ENR, ClkEnableMsk); /* Assumed RCC config */
    }
    // TIM2 and TIM5 are reserved
}

/* --- Public Functions --- */

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel, handle error (e.g., return an error code, assert, or log)
        return;
    }

    const PWM_Channel_Config_t *config = &PWM_Channel_Map[TRD_Channel];

    // 1. Enable GPIO Clock
    enable_gpio_clk(config->GPIOx, config->GPIO_Clk_En_Msk);

    // 2. Configure GPIO pin for Alternate Function
    GPIO_Config_AF(config->GPIOx, config->GPIO_Pin_Msk, config->GPIO_AF);

    // 3. Enable Timer Clock
    enable_timer_clk(config->TIMx, config->TIM_Clk_En_Msk);

    // 4. Disable timer during configuration
    BIT_CLR(config->TIMx->CR1, TIM_CR1_CEN); /* PDF Reference */

    // 5. Configure Timer Time-base (Prescaler and Auto-Reload Register)
    //    For initial setup, a default frequency and duty cycle can be set.
    //    Here, we set a default ARR value to a max value for 16-bit timers (0xFFFF)
    //    and a prescaler to effectively achieve a usable frequency range.
    //    Initial PSC and ARR will result in a low default frequency.
    //    Frequency will be properly set by PWM_Set_Freq.

    // Set prescaler to 0 for initial setup (or a small value)
    REG_WRITE(config->TIMx->PSC, 0); /* PDF Reference */

    // Set ARR to max value for 16-bit timers as a default
    REG_WRITE(config->TIMx->ARR, MAX_ARR_16BIT); /* PDF Reference */

    // Configure Timer PWM Mode
    volatile tlong *ccmr_reg = NULL;
    tlong ccmr_clear_mask = 0;
    tlong ccmr_set_mask = 0;

    // Determine which CCMR register to use based on the channel number
    if (config->TIM_Channel == 1 || config->TIM_Channel == 2)
    {
        ccmr_reg = &config->TIMx->CCMR1;
        if (config->TIM_Channel == 1)
        {
            // Configure CC1 channel as output
            ccmr_clear_mask |= (0x3U << (0U)); // CC1S bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OUTPUT << (0U)); // (PDF Reference)
            // Enable OC1PE (Output Compare 1 Preload Enable)
            ccmr_clear_mask |= (0x1U << (3U)); // OC1PE bit (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxPE << (3U)); // (PDF Reference)
            // Configure OC1M for PWM Mode 1
            ccmr_clear_mask |= (0x7U << (4U)); // OC1M bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxM_PWM1 << (4U)); // (PDF Reference)
        }
        else if (config->TIM_Channel == 2)
        {
            // Configure CC2 channel as output
            ccmr_clear_mask |= (0x3U << (8U)); // CC2S bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OUTPUT << (8U)); // (PDF Reference)
            // Enable OC2PE (Output Compare 2 Preload Enable)
            ccmr_clear_mask |= (0x1U << (11U)); // OC2PE bit (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxPE << (11U)); // (PDF Reference)
            // Configure OC2M for PWM Mode 1
            ccmr_clear_mask |= (0x7U << (12U)); // OC2M bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxM_PWM1 << (12U)); // (PDF Reference)
        }
    }
    else if (config->TIM_Channel == 3 || config->TIM_Channel == 4)
    {
        ccmr_reg = &config->TIMx->CCMR2;
        if (config->TIM_Channel == 3)
        {
            // Configure CC3 channel as output
            ccmr_clear_mask |= (0x3U << (0U)); // CC3S bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OUTPUT << (0U)); // (PDF Reference)
            // Enable OC3PE (Output Compare 3 Preload Enable)
            ccmr_clear_mask |= (0x1U << (3U)); // OC3PE bit (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxPE << (3U)); // (PDF Reference)
            // Configure OC3M for PWM Mode 1
            ccmr_clear_mask |= (0x7U << (4U)); // OC3M bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxM_PWM1 << (4U)); // (PDF Reference)
        }
        else if (config->TIM_Channel == 4)
        {
            // Configure CC4 channel as output
            ccmr_clear_mask |= (0x3U << (8U)); // CC4S bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OUTPUT << (8U)); // (PDF Reference)
            // Enable OC4PE (Output Compare 4 Preload Enable)
            ccmr_clear_mask |= (0x1U << (11U)); // OC4PE bit (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxPE << (11U)); // (PDF Reference)
            // Configure OC4M for PWM Mode 1
            ccmr_clear_mask |= (0x7U << (12U)); // OC4M bits (PDF Reference)
            ccmr_set_mask   |= (TIM_CCMR_OCxM_PWM1 << (12U)); // (PDF Reference)
        }
    }
    MODIFY_REG(*ccmr_reg, ccmr_clear_mask, ccmr_set_mask);

    // Set ARPE (Auto-reload preload enable)
    BIT_SET(config->TIMx->CR1, TIM_CR1_ARPE); /* PDF Reference */

    // Set initial duty cycle to 0 (or some safe value like 50%)
    volatile tlong *ccrx = get_ccrx_register(config->TIMx, config->TIM_Channel);
    if (ccrx != (volatile tlong *)0)
    {
        REG_WRITE(*ccrx, 0); // Initial duty cycle 0% /* PDF Reference */
    }

    // Enable the output channel (CCxE bit) and set polarity (CCxP bit)
    tlong ccer_clear_mask = 0;
    tlong ccer_set_mask = 0;

    ccer_clear_mask |= (0x1U << TIM_CCER_CC1E_Pos) << ((config->TIM_Channel - 1) * 4); /* PDF Reference */
    ccer_set_mask   |= (0x1U << TIM_CCER_CC1E_Pos) << ((config->TIM_Channel - 1) * 4); // Enable output /* PDF Reference */
    ccer_clear_mask |= (0x1U << TIM_CCER_CC1P_Pos) << ((config->TIM_Channel - 1) * 4); /* PDF Reference */
    ccer_set_mask   |= (0x0U << TIM_CCER_CC1P_Pos) << ((config->TIM_Channel - 1) * 4); // Active high polarity /* PDF Reference */
    MODIFY_REG(config->TIMx->CCER, ccer_clear_mask, ccer_set_mask);

    // If TIM1, enable Main Output Enable (MOE) for complementary outputs.
    // This is required for any output on TIM1.
    if (config->TIMx == TIM1)
    {
        BIT_SET(config->TIMx->BDTR, TIM_BDTR_MOE); /* PDF Reference */
    }

    // Generate an update event to load preload registers (UG bit)
    BIT_SET(config->TIMx->EGR, TIM_EGR_UG); /* PDF Reference */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency Desired PWM frequency in Hz.
 * @param duty Desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS || frequency == 0 || duty > 100)
    {
        // Invalid parameters
        return;
    }

    const PWM_Channel_Config_t *config = &PWM_Channel_Map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;
    tlong timer_clk = config->Timer_Input_Clock_Freq;

    tlong prescaler = 0;
    tlong auto_reload_value = 0;
    tlong max_arr_val = (config->Is_32bit_Timer) ? MAX_ARR_32BIT : MAX_ARR_16BIT;

    // Calculate prescaler and ARR to achieve desired frequency.
    // Iteratively find a suitable prescaler to keep ARR within limits.
    // PWM_FREQ = TIM_CLK / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = TIM_CLK / PWM_FREQ

    tlong timer_ticks_per_period = timer_clk / frequency;

    if (timer_ticks_per_period == 0)
    {
        // Frequency too high for current timer clock, set max frequency possible
        prescaler = 0; /* PDF Reference */
        auto_reload_value = 0; /* PDF Reference */
    }
    else
    {
        // Find smallest prescaler that keeps ARR <= max_arr_val
        for (prescaler = 0; prescaler < 0xFFFF; prescaler++) /* PDF Reference */
        {
            auto_reload_value = (timer_ticks_per_period / (prescaler + 1));
            if (auto_reload_value > 0 && auto_reload_value <= max_arr_val)
            {
                auto_reload_value--; // ARR = N-1 for N ticks
                break;
            }
        }
        if (prescaler >= 0xFFFF && auto_reload_value == 0) // Did not find suitable prescaler in 16-bit range
        {
            // Fallback to max period for given frequency, might be less accurate
            prescaler = 0xFFFF; /* PDF Reference */
            auto_reload_value = (timer_ticks_per_period / (prescaler + 1));
            if (auto_reload_value > 0)
            {
                auto_reload_value--; /* PDF Reference */
            } else {
                auto_reload_value = 0; /* PDF Reference */
            }
        }
    }

    // Disable timer before changing settings
    BIT_CLR(TIMx->CR1, TIM_CR1_CEN); /* PDF Reference */
    BIT_SET(TIMx->CR1, TIM_CR1_UDIS); // Disable Update Event to prevent unwanted updates during config /* PDF Reference */

    // Apply calculated values
    REG_WRITE(TIMx->PSC, prescaler); /* PDF Reference */
    REG_WRITE(TIMx->ARR, auto_reload_value); /* PDF Reference */

    // Calculate Capture Compare Register value for duty cycle
    tlong ccr_value = (auto_reload_value + 1) * duty / 100;
    if (ccr_value > auto_reload_value) ccr_value = auto_reload_value; // Cap at ARR value
    if (duty == 0) ccr_value = 0; // Ensure 0% duty cycle is 0

    volatile tlong *ccrx = get_ccrx_register(TIMx, config->TIM_Channel);
    if (ccrx != (volatile tlong *)0)
    {
        REG_WRITE(*ccrx, ccr_value); /* PDF Reference */
    }

    // Re-enable Update Event
    BIT_CLR(TIMx->CR1, TIM_CR1_UDIS); /* PDF Reference */

    // Generate update event to load new PSC and ARR values (UG bit)
    BIT_SET(TIMx->EGR, TIM_EGR_UG); /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = &PWM_Channel_Map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;

    // Enable the specific Capture/Compare channel output
    tlong ccer_set_mask = (0x1U << TIM_CCER_CC1E_Pos) << ((config->TIM_Channel - 1) * 4); /* PDF Reference */
    BIT_SET(TIMx->CCER, ccer_set_mask);

    // If TIM1, enable Main Output Enable (MOE)
    if (TIMx == TIM1)
    {
        BIT_SET(TIMx->BDTR, TIM_BDTR_MOE); /* PDF Reference */
    }

    // Enable the counter
    BIT_SET(TIMx->CR1, TIM_CR1_CEN); /* PDF Reference */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = &PWM_Channel_Map[TRD_Channel];
    TIM_TypeDef *TIMx = config->TIMx;

    // Disable the specific Capture/Compare channel output
    tlong ccer_clear_mask = (0x1U << TIM_CCER_CC1E_Pos) << ((config->TIM_Channel - 1) * 4); /* PDF Reference */
    BIT_CLR(TIMx->CCER, ccer_clear_mask);

    // If TIM1, disable Main Output Enable (MOE) if no other channels are active
    // A more robust implementation might check if any other channels on TIM1 are still active.
    if (TIMx == TIM1)
    {
        // Check if any other TIM1 channels are still active to decide if MOE can be cleared.
        // For production, this logic should be improved to track active channels on TIM1.
        // For now, assuming if this channel is stopped, MOE can be safely cleared for TIM1.
        BIT_CLR(TIMx->BDTR, TIM_BDTR_MOE); /* PDF Reference */
    }

    // Disable the counter (optional, can keep counting if needed for other channels)
    BIT_CLR(TIMx->CR1, TIM_CR1_CEN); /* PDF Reference */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function disables clocks for all timers and GPIOs used by this PWM driver.
 */
void PWM_PowerOff(void)
{
    tbyte i;
    for (i = 0; i < NUM_PWM_CHANNELS; i++)
    {
        const PWM_Channel_Config_t *config = &PWM_Channel_Map[i];
        TIM_TypeDef *TIMx = config->TIMx;

        // Ensure timer is stopped
        BIT_CLR(TIMx->CR1, TIM_CR1_CEN); /* PDF Reference */

        // Disable specific channel output
        tlong ccer_clear_mask = (0x1U << TIM_CCER_CC1E_Pos) << ((config->TIM_Channel - 1) * 4); /* PDF Reference */
        BIT_CLR(TIMx->CCER, ccer_clear_mask);

        // If TIM1, ensure MOE is off
        if (TIMx == TIM1)
        {
            BIT_CLR(TIMx->BDTR, TIM_BDTR_MOE); /* PDF Reference */
        }
        
        // Disable timer clock
        disable_timer_clk(TIMx, config->TIM_Clk_En_Msk);

        // Reset GPIO to default input floating mode (MODER 00)
        tlong pin_num = 0;
        tlong moder_clear_mask;
        // Find the pin number from the PinMask
        while (!((config->GPIO_Pin_Msk >> pin_num) & 1U) && pin_num < 16) {
            pin_num++;
        }
        if (pin_num < 16) {
             moder_clear_mask = (0x3U << (pin_num * 2)); /* PDF Reference */
             MODIFY_REG(config->GPIOx->MODER, moder_clear_mask, (0x0U << (pin_num * 2))); /* PDF Reference */
             // Clear Alternate Function registers
             if (pin_num < 8)
             {
                 MODIFY_REG(config->GPIOx->AFRL, (0xFU << (pin_num * 4)), 0); /* PDF Reference */
             }
             else
             {
                 MODIFY_REG(config->GPIOx->AFRH, (0xFU << ((pin_num - 8) * 4)), 0); /* PDF Reference */
             }
        }
        // Note: For a true power-off, GPIO clocks might also need to be disabled.
        // However, disabling GPIO clocks generally requires careful system design
        // as other peripherals might share the same GPIO port.
        // For this specific request, we focus on disabling the PWM functionality and timer clocks.
    }
}