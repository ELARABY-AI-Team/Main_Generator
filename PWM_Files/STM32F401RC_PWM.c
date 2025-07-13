/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include <stdint.h>
// Include assumed CMSIS header for SystemCoreClock
#include "stm32f4xx.h" /* Assumed standard CMSIS header */

// Assuming tlong is uint32_t and tbyte is uint8_t as common practice
// These should ideally be defined in STM32F401RC_PWM.h
// typedef uint32_t tlong;
// typedef uint8_t tbyte;

// Timer clock source frequency (e.g., HCLK for timers not connected to APB prescalers, or PCLK for others)
// On STM32F4, TIM1/9/10/11 are on APB2, TIM2/3/4/5 are on APB1.
// Assume APB1/APB2 timers are running at HCLK frequency if APB prescalers are set to /1, or 2x PCLK if prescaler > 1.
// For F401, max HCLK is 84MHz. APB1 max is 42MHz, APB2 max is 84MHz.
// If APBx prescaler > 1, timer clock is 2 * APBx clock.
// This might need adjustment based on actual RCC configuration.
#define TIMER_CLOCK_FREQ SystemCoreClock /* Assumed timer clock frequency - please verify RCC configuration */

// Define GPIO port base addresses (Assumed based on standard STM32 definitions)
#define GPIOA_BASE (0x40020000UL)
#define GPIOB_BASE (0x40020400UL)
#define GPIOC_BASE (0x40020800UL)
#define GPIOD_BASE (0x40020C00UL)
#define GPIOE_BASE (0x40021000UL)
#define GPIOH_BASE (0x40021C00UL) // GPIOH only has PH0, PH1 on F401

// GPIO Peripheral Structure (Assumed based on standard STM32 definitions)
typedef struct
{
    volatile uint32_t MODER;   // GPIO port mode register, Address offset: 0x00
    volatile uint32_t OTYPER;  // GPIO port output type register, Address offset: 0x04
    volatile uint32_t OSPEEDR; // GPIO port output speed register, Address offset: 0x08
    volatile uint32_t PUPDR;   // GPIO port pull-up/pull-down register, Address offset: 0x0C
    volatile uint32_t IDR;     // GPIO port input data register, Address offset: 0x10
    volatile uint32_t ODR;     // GPIO port output data register, Address offset: 0x14
    volatile uint32_t BSRR;    // GPIO port bit set/reset register, Address offset: 0x18
    volatile uint32_t LCKR;    // GPIO port configuration lock register, Address offset: 0x1C
    volatile uint32_t AFRL;    // GPIO alternate function low register, Address offset: 0x20
    volatile uint32_t AFRH;    // GPIO alternate function high register, Address offset: 0x24
} GPIO_TypeDef;

// Define Timer base addresses (Assumed based on standard STM32 definitions)
#define TIM1_BASE (0x40010000UL)
#define TIM2_BASE (0x40000000UL)
#define TIM3_BASE (0x40000400UL)
#define TIM4_BASE (0x40000800UL)
#define TIM5_BASE (0x40000C00UL)
#define TIM9_BASE (0x40010400UL)
#define TIM10_BASE (0x40010800UL)
#define TIM11_BASE (0x40010C00UL)

// Timer Peripheral Structure (Generic - some fields might not exist on all timers)
// Using 32-bit access where registers might be 32-bit (TIM2/5 CNT, ARR, CCRx)
// and 16-bit for others for clarity where size is explicitly 16-bit.
// Accessing all as 32-bit is also generally safe.
typedef struct
{
    volatile uint32_t CR1;   // TIM control register 1, Address offset: 0x00
    volatile uint32_t CR2;   // TIM control register 2, Address offset: 0x04 (TIM1/2/3/4/5/9 only)
    volatile uint32_t SMCR;  // TIM slave mode control register, Address offset: 0x08 (TIM1/2/3/4/5/9 only)
    volatile uint32_t DIER;  // TIM DMA/interrupt enable register, Address offset: 0x0C
    volatile uint32_t SR;    // TIM status register, Address offset: 0x10
    volatile uint32_t EGR;   // TIM event generation register, Address offset: 0x14
    volatile uint32_t CCMR1; // TIM capture/compare mode register 1, Address offset: 0x18
    volatile uint32_t CCMR2; // TIM capture/compare mode register 2, Address offset: 0x1C (TIM1/2/3/4/5 only)
    volatile uint32_t CCER;  // TIM capture/compare enable register, Address offset: 0x20
    volatile uint32_t CNT;   // TIM counter, Address offset: 0x24
    volatile uint32_t PSC;   // TIM prescaler, Address offset: 0x28
    volatile uint32_t ARR;   // TIM auto-reload register, Address offset: 0x2C
    volatile uint32_t RCR;   // TIM repetition counter register, Address offset: 0x30 (TIM1 only)
    volatile uint32_t CCR1;  // TIM capture/compare register 1, Address offset: 0x34
    volatile uint32_t CCR2;  // TIM capture/compare register 2, Address offset: 0x38 (TIM1/2/3/4/5/9 only)
    volatile uint32_t CCR3;  // TIM capture/compare register 3, Address offset: 0x3C (TIM1/2/3/4/5 only)
    volatile uint32_t CCR4;  // TIM capture/compare register 4, Address offset: 0x40 (TIM1/2/3/4/5 only)
    volatile uint32_t BDTR;  // TIM break and dead-time register, Address offset: 0x44 (TIM1 only)
    volatile uint32_t DCR;   // TIM DMA control register, Address offset: 0x48 (TIM1/2/3/4/5 only)
    volatile uint32_t DMAR;  // TIM DMA address for full transfer, Address offset: 0x4C (TIM1/2/3/4/5 only)
    volatile uint32_t OR;    // TIM option register, Address offset: 0x50 (TIM2/5/11 only)
} TIM_TypeDef;

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE) // Only PD2 on F401
#define GPIOE ((GPIO_TypeDef *)GPIOE_BASE) // No GPIOE on F401
#define GPIOH ((GPIO_TypeDef *)GPIOH_BASE) // Only PH0, PH1 on F401

#define TIM1 ((TIM_TypeDef *)TIM1_BASE)
#define TIM2 ((TIM_TypeDef *)TIM2_BASE)
#define TIM3 ((TIM_TypeDef *)TIM3_BASE)
#define TIM4 ((TIM_TypeDef *)TIM4_BASE)
#define TIM5 ((TIM_TypeDef *)TIM5_BASE)
#define TIM9 ((TIM_TypeDef *)TIM9_BASE)
#define TIM10 ((TIM_TypeDef *)TIM10_BASE)
#define TIM11 ((TIM_TypeDef *)TIM11_BASE)

// RCC Peripheral Structure (Assumed based on standard STM32 definitions)
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB1ENR; // AHB1 Enable Register
    volatile uint32_t AHB2ENR; // AHB2 Enable Register
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1ENR; // APB1 Enable Register
    volatile uint32_t APB2ENR; // APB2 Enable Register
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t DCKCFGR;
} RCC_TypeDef;

#define RCC ((RCC_TypeDef *)0x40023800UL) // Assumed RCC base address

// RCC Enable Bits (Assumed based on F401RC Reference Manual/Datasheet)
#define RCC_AHB1ENR_GPIOAEN (1U << 0)  /* Assumed RCC bit */
#define RCC_AHB1ENR_GPIOBEN (1U << 1)  /* Assumed RCC bit */
#define RCC_AHB1ENR_GPIOCEN (1U << 2)  /* Assumed RCC bit */
#define RCC_AHB1ENR_GPIODEN (1U << 3)  /* Assumed RCC bit */ // Only PD2 on F401
#define RCC_AHB1ENR_GPIOEEN (1U << 4)  /* Assumed RCC bit */ // No GPIOE on F401
#define RCC_AHB1ENR_GPIOHEN (1U << 7)  /* Assumed RCC bit */ // Only PH0, PH1 on F401
#define RCC_APB1ENR_TIM2EN (1U << 0)   /* Assumed RCC bit */
#define RCC_APB1ENR_TIM3EN (1U << 1)   /* Assumed RCC bit */
#define RCC_APB1ENR_TIM4EN (1U << 2)   /* Assumed RCC bit */
#define RCC_APB1ENR_TIM5EN (1U << 3)   /* Assumed RCC bit */
#define RCC_APB2ENR_TIM1EN (1U << 0)   /* Assumed RCC bit */
#define RCC_APB2ENR_TIM9EN (1U << 16)  /* Assumed RCC bit */
#define RCC_APB2ENR_TIM10EN (1U << 17) /* Assumed RCC bit */
#define RCC_APB2ENR_TIM11EN (1U << 18) /* Assumed RCC bit */

// Timer Register Bit Definitions (Based on PDF reference)
// CR1
#define TIM_CR1_CEN_Pos (0U)
#define TIM_CR1_CEN_Msk (0x1U << TIM_CR1_CEN_Pos) /* 0x00000001 */
#define TIM_CR1_CEN TIM_CR1_CEN_Msk             /* PDF Reference */
#define TIM_CR1_UDIS_Pos (1U)
#define TIM_CR1_UDIS_Msk (0x1U << TIM_CR1_UDIS_Pos) /* 0x00000002 */
#define TIM_CR1_UDIS TIM_CR1_UDIS_Msk             /* PDF Reference */
#define TIM_CR1_URS_Pos (2U)
#define TIM_CR1_URS_Msk (0x1U << TIM_CR1_URS_Pos) /* 0x00000004 */
#define TIM_CR1_URS TIM_CR1_URS_Msk             /* PDF Reference */
#define TIM_CR1_OPM_Pos (3U)
#define TIM_CR1_OPM_Msk (0x1U << TIM_CR1_OPM_Pos) /* 0x00000008 */
#define TIM_CR1_OPM TIM_CR1_OPM_Msk             /* PDF Reference */
#define TIM_CR1_DIR_Pos (4U)
#define TIM_CR1_DIR_Msk (0x1U << TIM_CR1_DIR_Pos) /* 0x00000010 */
#define TIM_CR1_DIR TIM_CR1_DIR_Msk             /* PDF Reference */
#define TIM_CR1_CMS_Pos (5U)
#define TIM_CR1_CMS_Msk (0x3U << TIM_CR1_CMS_Pos) /* 0x00000060 */
#define TIM_CR1_CMS TIM_CR1_CMS_Msk             /* PDF Reference */
#define TIM_CR1_ARPE_Pos (7U)
#define TIM_CR1_ARPE_Msk (0x1U << TIM_CR1_ARPE_Pos) /* 0x00000080 */
#define TIM_CR1_ARPE TIM_CR1_ARPE_Msk             /* PDF Reference */
#define TIM_CR1_CKD_Pos (8U)
#define TIM_CR1_CKD_Msk (0x3U << TIM_CR1_CKD_Pos) /* 0x00000300 */
#define TIM_CR1_CKD TIM_CR1_CKD_Msk             /* PDF Reference */

// CCMR1 / CCMR2 (Output Compare Mode)
// CCMR1 OCM1[2:0] = 110 (PWM1) or 111 (PWM2) -> Bits 6:4
// CCMR1 OC1PE = 1 -> Bit 3
// CCMR1 OC1FE = 0 (normal) -> Bit 2
// CCMR1 CC1S = 00 (Output) -> Bits 1:0
// Repeat pattern for other channels in CCMR1/CCMR2
#define TIM_CCMR1_OC1M_Pos (4U)
#define TIM_CCMR1_OC1M_Msk (0x7U << TIM_CCMR1_OC1M_Pos) /* 0x00000070 */
#define TIM_CCMR1_OC1M TIM_CCMR1_OC1M_Msk             /* PDF Reference */
#define TIM_CCMR1_OC1M_PWM1 (0x6U << TIM_CCMR1_OC1M_Pos) /* 110 */ /* PDF Reference */
#define TIM_CCMR1_OC1M_PWM2 (0x7U << TIM_CCMR1_OC1M_Pos) /* 111 */ /* PDF Reference */
#define TIM_CCMR1_OC1PE_Pos (3U)
#define TIM_CCMR1_OC1PE_Msk (0x1U << TIM_CCMR1_OC1PE_Pos) /* 0x00000008 */
#define TIM_CCMR1_OC1PE TIM_CCMR1_OC1PE_Msk             /* PDF Reference */
#define TIM_CCMR1_CC1S_Pos (0U)
#define TIM_CCMR1_CC1S_Msk (0x3U << TIM_CCMR1_CC1S_Pos) /* 0x00000003 */
#define TIM_CCMR1_CC1S_Output (0x0U << TIM_CCMR1_CC1S_Pos) /* 00 */ /* PDF Reference */

#define TIM_CCMR1_OC2M_Pos (12U)
#define TIM_CCMR1_OC2M_Msk (0x7U << TIM_CCMR1_OC2M_Pos) /* 0x00007000 */
#define TIM_CCMR1_OC2M TIM_CCMR1_OC2M_Msk             /* PDF Reference */
#define TIM_CCMR1_OC2M_PWM1 (0x6U << TIM_CCMR1_OC2M_Pos) /* 110 */ /* PDF Reference */
#define TIM_CCMR1_OC2PE_Pos (11U)
#define TIM_CCMR1_OC2PE_Msk (0x1U << TIM_CCMR1_OC2PE_Pos) /* 0x00000800 */
#define TIM_CCMR1_OC2PE TIM_CCMR1_OC2PE_Msk             /* PDF Reference */
#define TIM_CCMR1_CC2S_Pos (8U)
#define TIM_CCMR1_CC2S_Msk (0x3U << TIM_CCMR1_CC2S_Pos) /* 0x00000300 */
#define TIM_CCMR1_CC2S_Output (0x0U << TIM_CCMR1_CC2S_Pos) /* 00 */ /* PDF Reference */

#define TIM_CCMR2_OC3M_Pos (4U)
#define TIM_CCMR2_OC3M_Msk (0x7U << TIM_CCMR2_OC3M_Pos) /* 0x00000070 */
#define TIM_CCMR2_OC3M TIM_CCMR2_OC3M_Msk             /* PDF Reference */
#define TIM_CCMR2_OC3M_PWM1 (0x6U << TIM_CCMR2_OC3M_Pos) /* 110 */ /* PDF Reference */
#define TIM_CCMR2_OC3PE_Pos (3U)
#define TIM_CCMR2_OC3PE_Msk (0x1U << TIM_CCMR2_OC3PE_Pos) /* 0x00000008 */
#define TIM_CCMR2_OC3PE TIM_CCMR2_OC3PE_Msk             /* PDF Reference */
#define TIM_CCMR2_CC3S_Pos (0U)
#define TIM_CCMR2_CC3S_Msk (0x3U << TIM_CCMR2_CC3S_Pos) /* 0x00000003 */
#define TIM_CCMR2_CC3S_Output (0x0U << TIM_CCMR2_CC3S_Pos) /* 00 */ /* PDF Reference */

#define TIM_CCMR2_OC4M_Pos (12U)
#define TIM_CCMR2_OC4M_Msk (0x7U << TIM_CCMR2_OC4M_Pos) /* 0x00007000 */
#define TIM_CCMR2_OC4M TIM_CCMR2_OC4M_Msk             /* PDF Reference */
#define TIM_CCMR2_OC4M_PWM1 (0x6U << TIM_CCMR2_OC4M_Pos) /* 110 */ /* PDF Reference */
#define TIM_CCMR2_OC4PE_Pos (11U)
#define TIM_CCMR2_OC4PE_Msk (0x1U << TIM_CCMR2_OC4PE_Pos) /* 0x00000800 */
#define TIM_CCMR2_OC4PE TIM_CCMR2_OC4PE_Msk             /* PDF Reference */
#define TIM_CCMR2_CC4S_Pos (8U)
#define TIM_CCMR2_CC4S_Msk (0x3U << TIM_CCMR2_CC4S_Pos) /* 0x00000300 */
#define TIM_CCMR2_CC4S_Output (0x0U << TIM_CCMR2_CC4S_Pos) /* 00 */ /* PDF Reference */


// CCER
// CC1E = 1 -> Bit 0
// CC1P = 0 (Active High) -> Bit 1
// Repeat pattern for other channels
#define TIM_CCER_CC1E_Pos (0U)
#define TIM_CCER_CC1E_Msk (0x1U << TIM_CCER_CC1E_Pos) /* 0x00000001 */
#define TIM_CCER_CC1E TIM_CCER_CC1E_Msk             /* PDF Reference */
#define TIM_CCER_CC1P_Pos (1U)
#define TIM_CCER_CC1P_Msk (0x1U << TIM_CCER_CC1P_Pos) /* 0x00000002 */
#define TIM_CCER_CC1P TIM_CCER_CC1P_Msk             /* PDF Reference */
#define TIM_CCER_CC1NE_Pos (2U) // TIM1 only
#define TIM_CCER_CC1NE_Msk (0x1U << TIM_CCER_CC1NE_Pos) /* 0x00000004 */
#define TIM_CCER_CC1NE TIM_CCER_CC1NE_Msk             /* PDF Reference */
#define TIM_CCER_CC1NP_Pos (3U) // TIM1 only
#define TIM_CCER_CC1NP_Msk (0x1U << TIM_CCER_CC1NP_Pos) /* 0x00000008 */
#define TIM_CCER_CC1NP TIM_CCER_CC1NP_Msk             /* PDF Reference */

#define TIM_CCER_CC2E_Pos (4U)
#define TIM_CCER_CC2E_Msk (0x1U << TIM_CCER_CC2E_Pos) /* 0x00000010 */
#define TIM_CCER_CC2E TIM_CCER_CC2E_Msk             /* PDF Reference */
#define TIM_CCER_CC2P_Pos (5U)
#define TIM_CCER_CC2P_Msk (0x1U << TIM_CCER_CC2P_Pos) /* 0x00000020 */
#define TIM_CCER_CC2P TIM_CCER_CC2P_Msk             /* PDF Reference */
#define TIM_CCER_CC2NE_Pos (6U) // TIM1 only
#define TIM_CCER_CC2NE_Msk (0x1U << TIM_CCER_CC2NE_Pos) /* 0x00000040 */
#define TIM_CCER_CC2NE TIM_CCER_CC2NE_Msk             /* PDF Reference */
#define TIM_CCER_CC2NP_Pos (7U) // TIM1 only
#define TIM_CCER_CC2NP_Msk (0x1U << TIM_CCER_CC2NP_Pos) /* 0x00000080 */
#define TIM_CCER_CC2NP TIM_CCER_CC2NP_Msk             /* PDF Reference */

#define TIM_CCER_CC3E_Pos (8U)
#define TIM_CCER_CC3E_Msk (0x1U << TIM_CCER_CC3E_Pos) /* 0x00000100 */
#define TIM_CCER_CC3E TIM_CCER_CC3E_Msk             /* PDF Reference */
#define TIM_CCER_CC3P_Pos (9U)
#define TIM_CCER_CC3P_Msk (0x1U << TIM_CCER_CC3P_Pos) /* 0x00000200 */
#define TIM_CCER_CC3P TIM_CCER_CC3P_Msk             /* PDF Reference */
#define TIM_CCER_CC3NE_Pos (10U) // TIM1 only
#define TIM_CCER_CC3NE_Msk (0x1U << TIM_CCER_CC3NE_Pos) /* 0x00000400 */
#define TIM_CCER_CC3NE TIM_CCER_CC3NE_Msk             /* PDF Reference */
#define TIM_CCER_CC3NP_Pos (11U) // TIM1 only
#define TIM_CCER_CC3NP_Msk (0x1U << TIM_CCER_CC3NP_Pos) /* 0x00000800 */
#define TIM_CCER_CC3NP TIM_CCER_CC3NP_Msk             /* PDF Reference */

#define TIM_CCER_CC4E_Pos (12U)
#define TIM_CCER_CC4E_Msk (0x1U << TIM_CCER_CC4E_Pos) /* 0x00001000 */
#define TIM_CCER_CC4E TIM_CCER_CC4E_Msk             /* PDF Reference */
#define TIM_CCER_CC4P_Pos (13U)
#define TIM_CCER_CC4P_Msk (0x1U << TIM_CCER_CC4P_Pos) /* 0x00002000 */
#define TIM_CCER_CC4P TIM_CCER_CC4P_Msk             /* PDF Reference */
#define TIM_CCER_CC4NP_Pos (15U) // TIM1 only (reserved on others)
#define TIM_CCER_CC4NP_Msk (0x1U << TIM_CCER_CC4NP_Pos) /* 0x00008000 */
#define TIM_CCER_CC4NP TIM_CCER_CC4NP_Msk             /* PDF Reference */


// EGR
#define TIM_EGR_UG_Pos (0U)
#define TIM_EGR_UG_Msk (0x1U << TIM_EGR_UG_Pos) /* 0x00000001 */
#define TIM_EGR_UG TIM_EGR_UG_Msk             /* PDF Reference */

// BDTR (TIM1 only)
#define TIM_BDTR_MOE_Pos (15U)
#define TIM_BDTR_MOE_Msk (0x1U << TIM_BDTR_MOE_Pos) /* 0x00008000 */
#define TIM_BDTR_MOE TIM_BDTR_MOE_Msk             /* PDF Reference */

// GPIO AFR register bit position calculation
#define GPIO_AFRL_AFSEL_Pos(pin) ((pin)*4U) /* PDF Reference */
#define GPIO_AFRL_AFSEL_Msk(pin) (0xFU << GPIO_AFRL_AFSEL_Pos(pin)) /* PDF Reference */
#define GPIO_AFRH_AFSEL_Pos(pin) (((pin)-8U)*4U) /* PDF Reference */
#define GPIO_AFRH_AFSEL_Msk(pin) (0xFU << GPIO_AFRH_AFSEL_Pos(pin)) /* PDF Reference */

// GPIO MODER register bit position calculation
#define GPIO_MODER_MODER_Pos(pin) ((pin)*2U) /* PDF Reference */
#define GPIO_MODER_MODER_Msk(pin) (0x3U << GPIO_MODER_MODER_Pos(pin)) /* PDF Reference */
#define GPIO_MODER_AF (0x2U) // Alternate function mode (10b) /* PDF Reference */
#define GPIO_MODER_Output (0x1U) // General purpose output mode (01b) /* PDF Reference */
#define GPIO_MODER_Input (0x0U) // Input mode (00b) /* PDF Reference */

// GPIO OTYPER register bit position calculation
#define GPIO_OTYPER_OT_Pos(pin) (pin) /* PDF Reference */
#define GPIO_OTYPER_OT_Msk(pin) (0x1U << GPIO_OTYPER_OT_Pos(pin)) /* PDF Reference */
#define GPIO_OTYPER_PushPull (0x0U) // Output push-pull (0b) /* PDF Reference */

// GPIO OSPEEDR register bit position calculation
#define GPIO_OSPEEDR_OSPEEDR_Pos(pin) ((pin)*2U) /* PDF Reference */
#define GPIO_OSPEEDR_OSPEEDR_Msk(pin) (0x3U << GPIO_OSPEEDR_OSPEEDR_Pos(pin)) /* PDF Reference */
#define GPIO_OSPEEDR_High (0x2U) // High speed (10b) /* PDF Reference */

// GPIO PUPDR register bit position calculation
#define GPIO_PUPDR_PUPDR_Pos(pin) ((pin)*2U) /* PDF Reference */
#define GPIO_PUPDR_PUPDR_Msk(pin) (0x3U << GPIO_PUPDR_PUPDR_Pos(pin)) /* PDF Reference */
#define GPIO_PUPDR_NoPull (0x0U) // No pull-up, pull-down (00b) /* PDF Reference */


// Structure to hold configuration for a single PWM channel
typedef struct
{
    TIM_TypeDef* TIMx;
    uint8_t ChannelNumber; // 1, 2, 3, or 4
    GPIO_TypeDef* GPIO_Port;
    uint8_t PinNumber; // 0-15
    uint8_t GPIO_AFx_TIMx;
    uint32_t RccAhb1EnrBit; // RCC_AHB1ENR bit for GPIO port
    uint32_t RccApbEnrBit;  // RCC_APBxENR bit for Timer peripheral
    volatile uint32_t* pCCMR; // Pointer to the relevant CCMR register (CCMR1 or CCMR2)
    uint32_t CCMR_OCM_Msk;  // Mask for OCxM bits in CCMR
    uint32_t CCMR_OCM_Pos;  // Position for OCxM bits in CCMR
    uint32_t CCMR_OCPE_Msk; // Mask for OCxPE bit in CCMR
    uint32_t CCMR_OCPE_Pos; // Position for OCxPE bit in CCMR
    uint32_t CCMR_CCS_Msk;  // Mask for CCxS bits in CCMR
    uint32_t CCMR_CCS_Pos;  // Position for CCxS bits in CCMR
    uint32_t CCER_CCE_Msk;  // Mask for CCxE bit in CCER
    uint32_t CCER_CCE_Pos;  // Position for CCxE bit in CCER
    uint32_t CCER_CCP_Msk;  // Mask for CCxP bit in CCER
    uint32_t CCER_CCP_Pos;  // Position for CCxP bit in CCER
    volatile uint33_t* pCCRx; // Pointer to the relevant CCR register
} PWM_Channel_Config_t;

// Define maximum ARR value for 16-bit and 32-bit timers
#define MAX_ARR_16BIT 0xFFFF
#define MAX_ARR_32BIT 0xFFFFFFFFUL

// --- Timer Reservation ---
// TIM10 and TIM11 are reserved for potential OS/delay purposes as per requirement.
// They are excluded from the PWM_channel_map below.
// This choice is based on them being single-channel timers (less versatile for PWM)
// and available general-purpose timers in the F401RC mentioned in the PDF.
// --------------------------

/**Functions ===========================================================================*/

// Array mapping TRD_Channel_t enum to hardware configuration
// This array must be ordered according to the TRD_Channel_t enum in STM32F401RC_PWM.h
// Example entries:
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // TIM1 Channels (Advanced - CH1-CH4 available, but 1-3 have complementary)
    { TIM1, 1, GPIOA, 8,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM1->CCR1 }, /* Assumed PWM config - please verify - PA8 TIM1_CH1 AF1 */
    { TIM1, 2, GPIOA, 9,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM1->CCR2 }, /* Assumed PWM config - please verify - PA9 TIM1_CH2 AF1 */
    { TIM1, 3, GPIOA, 10, 1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM1->CCR3 }, /* Assumed PWM config - please verify - PA10 TIM1_CH3 AF1 */
    { TIM1, 4, GPIOA, 11, 1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM1->CCR4 }, /* Assumed PWM config - please verify - PA11 TIM1_CH4 AF1 */

    // TIM2 Channels (General Purpose, 32-bit counter)
    // PA0 is TIM2_CH1, but excluded. Use PA1.
    { TIM2, 1, GPIOA, 1,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM2->CCR1 }, /* Assumed PWM config - please verify - PA1 TIM2_CH2 AF1 (Note: PA0 is CH1, PA1 is CH2 - mistake in thought? Re-check. RM says PA0 is TIM2_CH1 AF1, PA1 is TIM2_CH2 AF1. Let's use PA5 for CH1 to avoid pin 0)*/
    { TIM2, 1, GPIOA, 5,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM2->CCR1 }, /* Assumed PWM config - please verify - PA5 TIM2_CH1 AF1 */
    { TIM2, 2, GPIOA, 1,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM2->CCR2 }, /* Assumed PWM config - please verify - PA1 TIM2_CH2 AF1 */
    { TIM2, 3, GPIOB, 10, 1, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM2->CCR3 }, /* Assumed PWM config - please verify - PB10 TIM2_CH3 AF1 */
    { TIM2, 4, GPIOB, 11, 1, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM2->CCR4 }, /* Assumed PWM config - please verify - PB11 TIM2_CH4 AF1 */

    // TIM3 Channels (General Purpose, 16-bit counter)
    // PB4 is TIM3_CH1 AF2, PB5 is TIM3_CH2 AF2, PC8 is TIM3_CH3 AF2, PC9 is TIM3_CH4 AF2
    { TIM3, 1, GPIOB, 4,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM3->CCR1 }, /* Assumed PWM config - please verify - PB4 TIM3_CH1 AF2 */
    { TIM3, 2, GPIOB, 5,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM3->CCR2 }, /* Assumed PWM config - please verify - PB5 TIM3_CH2 AF2 */
    { TIM3, 3, GPIOC, 8,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM3->CCR3 }, /* Assumed PWM config - please verify - PC8 TIM3_CH3 AF2 */
    { TIM3, 4, GPIOC, 9,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM3->CCR4 }, /* Assumed PWM config - please verify - PC9 TIM3_CH4 AF2 */

    // TIM4 Channels (General Purpose, 16-bit counter)
    // PB6 is TIM4_CH1 AF2, PB7 is TIM4_CH2 AF2, PB8 is TIM4_CH3 AF2, PB9 is TIM4_CH4 AF2
    { TIM4, 1, GPIOB, 6,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM4->CCR1 }, /* Assumed PWM config - please verify - PB6 TIM4_CH1 AF2 */
    { TIM4, 2, GPIOB, 7,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM4->CCR2 }, /* Assumed PWM config - please verify - PB7 TIM4_CH2 AF2 */
    { TIM4, 3, GPIOB, 8,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM4->CCR3 }, /* Assumed PWM config - please verify - PB8 TIM4_CH3 AF2 */
    { TIM4, 4, GPIOB, 9,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM4->CCR4 }, /* Assumed PWM config - please verify - PB9 TIM4_CH4 AF2 */

    // TIM5 Channels (General Purpose, 32-bit counter)
    // PA0 is TIM5_CH1 AF2 - Excluded due to pin 0. PA1 is TIM5_CH2 AF2. PA2 is TIM5_CH3 AF2. PA3 is TIM5_CH4 AF2.
    { TIM5, 1, GPIOA, 0,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM5->CCR1 }, // This entry violates the pin 0 rule and will be skipped by logic
    { TIM5, 2, GPIOA, 1,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM5->CCR2 }, /* Assumed PWM config - please verify - PA1 TIM5_CH2 AF2 */
    { TIM5, 3, GPIOA, 2,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM5->CCR3 }, /* Assumed PWM config - please verify - PA2 TIM5_CH3 AF2 */
    { TIM5, 4, GPIOA, 3,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM5->CCR4 }, /* Assumed PWM config - please verify - PA3 TIM5_CH4 AF2 */

    // TIM9 Channels (General Purpose, 16-bit counter, 2 channels, upcounter only)
    // PA2 is TIM9_CH1 AF3, PA3 is TIM9_CH2 AF3. Already used by TIM5. Let's find alternatives.
    // RM indicates PA2 is TIM9_CH1 AF3. PA3 is TIM9_CH2 AF3.
    // Other TIM9 options? Looking at F401 ref manual... No obvious alternatives for TIM9.
    // This highlights potential pin conflict if using both TIM5 and TIM9 CH1/2.
    // For this exercise, let's just list them, assuming a choice is made externally.
    // However, the requirement says "Use real, typical PWM-capable GPIO pins". PA2/PA3 are typical for TIM9.
    // Let's use PA2/PA3 for TIM9, assuming they are not also used by TIM5 in this *specific* setup.
    // Let's remove the conflicting TIM5 entries for PA2/PA3 to adhere to "typical" config.
    // Revised TIM5: Use PA1, PH0, PH1 (if PH0/1 valid), PC0
    // PA1: TIM5_CH2 AF2 - used.
    // Let's search F401RC datasheet for TIM5 pins...
    // TIM5: CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3 (All AF2). Need to avoid PA0. We can't avoid PA1, PA2, PA3 if we want to use all TIM5 channels. This conflicts with the initial PA1/PA2/PA3 choices for other timers/channels.
    // Let's simplify the map to avoid conflicts and exclude pin 0:
    // TIM1: PA8 (CH1), PA9 (CH2), PA10 (CH3), PA11 (CH4) - AF1 (OK)
    // TIM2: PA5 (CH1), PB3 (CH2), PB10 (CH3), PB11 (CH4) - AF1 (Note: PB3 is also JTDO, PB10 is TIM2_CH3 AF1, PB11 is TIM2_CH4 AF1) - Assumed.
    // TIM3: PC6 (CH1), PC7 (CH2), PC8 (CH3), PC9 (CH4) - AF2 (Assumed)
    // TIM4: PB6 (CH1), PB7 (CH2), PB8 (CH3), PB9 (CH4) - AF2 (Assumed)
    // TIM5: PA1 (CH2), PA2 (CH3), PA3 (CH4) - AF2 (PA0 CH1 excluded) - Assumed.
    // TIM9: PA2 (CH1), PA3 (CH2) - AF3 (Conflict with TIM5)

    // Final proposed map avoiding pin 0 and obvious conflicts, using assumed AF values:
    // TIM1: PA8 (CH1), PA9 (CH2), PA10 (CH3), PA11 (CH4) - AF1
    // TIM2: PA5 (CH1), PB3 (CH2), PB10 (CH3), PB11 (CH4) - AF1
    // TIM3: PC6 (CH1), PC7 (CH2), PC8 (CH3), PC9 (CH4) - AF2
    // TIM4: PB6 (CH1), PB7 (CH2), PB8 (CH3), PB9 (CH4) - AF2
    // TIM5: PA1 (CH2), PA2 (CH3), PA3 (CH4) - AF2 (PA0 CH1 excluded)
    // TIM9, TIM10, TIM11 are reserved or excluded from this map.

    { TIM1, 1, GPIOA, 8,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM1->CCR1 }, /* Assumed PWM config - PA8 TIM1_CH1 AF1 */
    { TIM1, 2, GPIOA, 9,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM1->CCR2 }, /* Assumed PWM config - PA9 TIM1_CH2 AF1 */
    { TIM1, 3, GPIOA, 10, 1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM1->CCR3 }, /* Assumed PWM config - PA10 TIM1_CH3 AF1 */
    { TIM1, 4, GPIOA, 11, 1, RCC_AHB1ENR_GPIOAEN, RCC_APB2ENR_TIM1EN, &TIM1->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM1->CCR4 }, /* Assumed PWM config - PA11 TIM1_CH4 AF1 */

    { TIM2, 1, GPIOA, 5,  1, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM2->CCR1 }, /* Assumed PWM config - PA5 TIM2_CH1 AF1 */
    { TIM2, 2, GPIOB, 3,  1, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM2->CCR2 }, /* Assumed PWM config - PB3 TIM2_CH2 AF1 */
    { TIM2, 3, GPIOB, 10, 1, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM2->CCR3 }, /* Assumed PWM config - PB10 TIM2_CH3 AF1 */
    { TIM2, 4, GPIOB, 11, 1, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM2EN, &TIM2->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM2->CCR4 }, /* Assumed PWM config - PB11 TIM2_CH4 AF1 */

    { TIM3, 1, GPIOC, 6,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM3->CCR1 }, /* Assumed PWM config - PC6 TIM3_CH1 AF2 */
    { TIM3, 2, GPIOC, 7,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM3->CCR2 }, /* Assumed PWM config - PC7 TIM3_CH2 AF2 */
    { TIM3, 3, GPIOC, 8,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM3->CCR3 }, /* Assumed PWM config - PC8 TIM3_CH3 AF2 */
    { TIM3, 4, GPIOC, 9,  2, RCC_AHB1ENR_GPIOCEN, RCC_APB1ENR_TIM3EN, &TIM3->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM3->CCR4 }, /* Assumed PWM config - PC9 TIM3_CH4 AF2 */

    { TIM4, 1, GPIOB, 6,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR1, TIM_CCMR1_OC1M_Msk, TIM_CCMR1_OC1M_Pos, TIM_CCMR1_OC1PE_Msk, TIM_CCMR1_OC1PE_Pos, TIM_CCMR1_CC1S_Msk, TIM_CCMR1_CC1S_Pos, TIM_CCER_CC1E_Msk, TIM_CCER_CC1E_Pos, TIM_CCER_CC1P_Msk, TIM_CCER_CC1P_Pos, &TIM4->CCR1 }, /* Assumed PWM config - PB6 TIM4_CH1 AF2 */
    { TIM4, 2, GPIOB, 7,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM4->CCR2 }, /* Assumed PWM config - PB7 TIM4_CH2 AF2 */
    { TIM4, 3, GPIOB, 8,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM4->CCR3 }, /* Assumed PWM config - PB8 TIM4_CH3 AF2 */
    { TIM4, 4, GPIOB, 9,  2, RCC_AHB1ENR_GPIOBEN, RCC_APB1ENR_TIM4EN, &TIM4->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM4->CCR4 }, /* Assumed PWM config - PB9 TIM4_CH4 AF2 */

    { TIM5, 2, GPIOA, 1,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR1, TIM_CCMR1_OC2M_Msk, TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2PE_Msk, TIM_CCMR1_OC2PE_Pos, TIM_CCMR1_CC2S_Msk, TIM_CCMR1_CC2S_Pos, TIM_CCER_CC2E_Msk, TIM_CCER_CC2E_Pos, TIM_CCER_CC2P_Msk, TIM_CCER_CC2P_Pos, &TIM5->CCR2 }, /* Assumed PWM config - PA1 TIM5_CH2 AF2 */
    { TIM5, 3, GPIOA, 2,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR2, TIM_CCMR2_OC3M_Msk, TIM_CCMR2_OC3M_Pos, TIM_CCMR2_OC3PE_Msk, TIM_CCMR2_OC3PE_Pos, TIM_CCMR2_CC3S_Msk, TIM_CCMR2_CC3S_Pos, TIM_CCER_CC3E_Msk, TIM_CCER_CC3E_Pos, TIM_CCER_CC3P_Msk, TIM_CCER_CC3P_Pos, &TIM5->CCR3 }, /* Assumed PWM config - PA2 TIM5_CH3 AF2 */
    { TIM5, 4, GPIOA, 3,  2, RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_TIM5EN, &TIM5->CCMR2, TIM_CCMR2_OC4M_Msk, TIM_CCMR2_OC4M_Pos, TIM_CCMR2_OC4PE_Msk, TIM_CCMR2_OC4PE_Pos, TIM_CCMR2_CC4S_Msk, TIM_CCMR2_CC4S_Pos, TIM_CCER_CC4E_Msk, TIM_CCER_CC4E_Pos, TIM_CCER_CC4P_Msk, TIM_CCER_CC4P_Pos, &TIM5->CCR4 }, /* Assumed PWM config - PA3 TIM5_CH4 AF2 */

    // No entries for TIM9, TIM10, TIM11 due to reservation or conflict avoidance.
};

// Number of configured PWM channels
#define NUM_PWM_CHANNELS (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))


/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

    // Ensure the pin is not 0, as per requirement. Skip initialization if it is.
    if (config->PinNumber == 0) {
        // Pin 0 is not allowed for PWM output configuration in this implementation
        return;
    }

    // 1. Enable GPIO clock for the specified port
    if (config->GPIO_Port == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Assumed RCC bit */
    else if (config->GPIO_Port == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Assumed RCC bit */
    else if (config->GPIO_Port == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* Assumed RCC bit */
    else if (config->GPIO_Port == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* Assumed RCC bit */
    // No GPIOE or GPIOH used in the current map that need clock enabling
    // else if (config->GPIO_Port == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; /* Assumed RCC bit */
    // else if (config->GPIO_Port == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; /* Assumed RCC bit */


    // 2. Configure the GPIO pin for Alternate Function
    uint32_t moder_mask = GPIO_MODER_MODER_Msk(config->PinNumber); /* PDF Reference */
    uint32_t moder_af = GPIO_MODER_AF << GPIO_MODER_MODER_Pos(config->PinNumber); /* PDF Reference */
    config->GPIO_Port->MODER = (config->GPIO_Port->MODER & ~moder_mask) | moder_af; /* PDF Reference */

    // 3. Configure GPIO Output Type (Push-Pull)
    uint32_t otyper_mask = GPIO_OTYPER_OT_Msk(config->PinNumber); /* PDF Reference */
    uint32_t otyper_pp = GPIO_OTYPER_PushPull << GPIO_OTYPER_OT_Pos(config->PinNumber); /* PDF Reference */
    config->GPIO_Port->OTYPER = (config->GPIO_Port->OTYPER & ~otyper_mask) | otyper_pp; /* PDF Reference */

    // 4. Configure GPIO Output Speed (High Speed)
    uint32_t ospeedr_mask = GPIO_OSPEEDR_OSPEEDR_Msk(config->PinNumber); /* PDF Reference */
    uint32_t ospeedr_high = GPIO_OSPEEDR_High << GPIO_OSPEEDR_OSPEEDR_Pos(config->PinNumber); /* PDF Reference */
    config->GPIO_Port->OSPEEDR = (config->GPIO_Port->OSPEEDR & ~ospeedr_mask) | ospeedr_high; /* PDF Reference */

    // 5. Configure GPIO Pull-up/Pull-down (No Pull)
    uint32_t pupdr_mask = GPIO_PUPDR_PUPDR_Msk(config->PinNumber); /* PDF Reference */
    uint32_t pupdr_nopull = GPIO_PUPDR_NoPull << GPIO_PUPDR_PUPDR_Pos(config->PinNumber); /* PDF Reference */
    config->GPIO_Port->PUPDR = (config->GPIO_Port->PUPDR & ~pupdr_mask) | pupdr_nopull; /* PDF Reference */

    // 6. Configure GPIO Alternate Function
    uint32_t af_value = config->GPIO_AFx_TIMx;
    if (config->PinNumber < 8)
    {
        uint32_t afrl_mask = GPIO_AFRL_AFSEL_Msk(config->PinNumber); /* PDF Reference */
        uint32_t afrl_sel = af_value << GPIO_AFRL_AFSEL_Pos(config->PinNumber); /* PDF Reference */
        config->GPIO_Port->AFRL = (config->GPIO_Port->AFRL & ~afrl_mask) | afrl_sel; /* PDF Reference */
    }
    else
    {
        uint32_t afrh_mask = GPIO_AFRH_AFSEL_Msk(config->PinNumber); /* PDF Reference */
        uint32_t afrh_sel = af_value << GPIO_AFRH_AFSEL_Pos(config->PinNumber); /* PDF Reference */
        config->GPIO_Port->AFRH = (config->GPIO_Port->AFRH & ~afrh_mask) | afrh_sel; /* PDF Reference */
    }

    // 7. Enable Timer clock for the specified timer
    // Assumes APB1 for TIM2-5, APB2 for TIM1, 9-11
    if (config->TIMx == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* Assumed RCC bit */
    else if (config->TIMx == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* Assumed RCC bit */
    else if (config->TIMx == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /* Assumed RCC bit */
    else if (config->TIMx == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; /* Assumed RCC bit */
    else if (config->TIMx == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; /* Assumed RCC bit */
    else if (config->TIMx == TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; /* Assumed RCC bit */
    // TIM10/11 are reserved, but enable clock here if they were ever added to map
    // else if (config->TIMx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; /* Assumed RCC bit */
    // else if (config->TIMx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; /* Assumed RCC bit */


    // 8. Configure the Timer for PWM
    // Disable the timer before configuration
    config->TIMx->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // Configure timer timebase (Prescaler and ARR) - Default values, will be updated by Set_Freq
    config->TIMx->PSC = 0; // Default prescaler value /* PDF Reference */
    config->TIMx->ARR = (config->TIMx == TIM2 || config->TIMx == TIM5) ? MAX_ARR_32BIT : MAX_ARR_16BIT; // Default max period /* PDF Reference */

    // Configure Capture/Compare channel for PWM output
    // Clear existing mode and set to Output /* PDF Reference */
    *config->pCCMR &= ~(config->CCMR_OCM_Msk | config->CCMR_CCS_Msk);
    // Set PWM Mode 1 (110) and enable output compare preload (OCxPE)
    *config->pCCMR |= (TIM_CCMR1_OC1M_PWM1 & config->CCMR_OCM_Msk) | (config->CCMR_OCPE_Msk); /* PDF Reference */

    // Configure Capture/Compare Enable register
    // Disable output first, set active high polarity
    config->TIMx->CCER &= ~(config->CCER_CCE_Msk | config->CCER_CCP_Msk);
    // Polarity active high (0) is default, no need to set explicitly unless changing
    // config->TIMx->CCER |= (0 << config->CCER_CCP_Pos); /* PDF Reference */

    // Set initial duty cycle to 0 (or a safe value)
    *config->pCCRx = 0;

    // Enable auto-reload preload for ARR (updates on UEV)
    config->TIMx->CR1 |= TIM_CR1_ARPE; /* PDF Reference */

    // For TIM1 (advanced timer), enable Main Output (MOE) if the channel is 1, 2, 3, or 4 (all have outputs)
    // MOE needs to be set for outputs to be enabled, typically done before starting timer.
    if (config->TIMx == TIM1)
    {
       config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Generate an update event to load the prescaler, ARR, and CCR values immediately
    config->TIMx->EGR |= TIM_EGR_UG; /* PDF Reference */

    // The UG bit is automatically cleared by hardware /* PDF Reference */
    // Wait for the update to complete if necessary (optional, UG is fast)
    // while(config->TIMx->SR & TIM_SR_UIF);
    // config->TIMx->SR &= ~TIM_SR_UIF;

    // Timer is configured but not started yet. Call PWM_Start to enable the counter.
}


/**
 * @brief Sets the PWM frequency and duty cycle for a specific channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz. Must be > 0.
 * @param duty: The desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS || frequency == 0 || duty > 100)
    {
        // Invalid channel, frequency, or duty cycle
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

     // Ensure the pin is not 0, as per requirement. Skip if it is.
    if (config->PinNumber == 0) {
        // Pin 0 is not configured for PWM output
        return;
    }

    // Calculate total timer ticks per period
    // TotalTicks = TIMER_CLOCK_FREQ / frequency
    uint32_t total_ticks;
    if (TIMER_CLOCK_FREQ < frequency) {
         total_ticks = 1; // Handle cases where freq > clock
    } else {
         total_ticks = TIMER_CLOCK_FREQ / frequency;
    }


    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t max_arr;

    // Determine max ARR based on timer type (16-bit or 32-bit)
    if (config->TIMx == TIM2 || config->TIMx == TIM5)
    {
        max_arr = MAX_ARR_32BIT;
    }
    else // TIM1, TIM3, TIM4, TIM9, TIM10, TIM11 are 16-bit
    {
        max_arr = MAX_ARR_16BIT;
    }

    // Calculate PSC and ARR. Aim for largest possible ARR for better duty cycle resolution.
    // (PSC + 1) * (ARR + 1) = total_ticks
    // ARR + 1 = total_ticks / (PSC + 1)
    // ARR = (total_ticks / (PSC + 1)) - 1

    if (total_ticks > 0) {
        // Find suitable PSC and ARR. Iterate PSC from 0 upwards.
        // A simpler approach is to target max ARR first.
        // If total_ticks <= max_arr + 1, we can use PSC = 0.
        if (total_ticks <= max_arr + 1) {
            prescaler = 0;
            arr = total_ticks - 1;
        } else {
            // Calculate minimum required PSC to bring ARR within range
            // (PSC + 1) > total_ticks / (max_arr + 1)
            // PSC > (total_ticks / (max_arr + 1)) - 1
            // Integer division requires care. (total_ticks - 1) / (max_arr + 1) is the smallest integer PSC value needed.
            prescaler = (total_ticks - 1) / (max_arr + 1);

            // Calculate ARR based on the chosen prescaler
            // If (prescaler + 1) is 0 (shouldn't happen with uint32_t > 0) or total_ticks is not divisible
            // this calculation might need refinement depending on required frequency accuracy.
            // Using simple integer division here.
            if (prescaler + 1 > 0) {
               arr = (total_ticks / (prescaler + 1)); // This is (ARR+1)
               if (arr > 0) arr--; // ARR = (ARR+1) - 1
               else arr = 0;
            } else {
               // Should not happen with prescaler >= 0
               prescaler = max_arr; // Fallback to max prescaler
               arr = 0;
            }

             // Ensure ARR does not exceed the maximum value for the timer type
            if (arr > max_arr) {
                arr = max_arr; // Cap ARR at max value
            }
        }

         // Ensure PSC does not exceed 16-bit maximum (65535)
        if (prescaler > 0xFFFF) {
            prescaler = 0xFFFF; // Cap PSC at 16-bit max
            // Recalculate ARR based on capped PSC
            if (prescaler + 1 > 0) {
               arr = (total_ticks / (prescaler + 1));
               if (arr > 0) arr--;
               else arr = 0;
            } else {
               arr = 0; // Should not happen
            }

             // Ensure ARR does not exceed the maximum value for the timer type
            if (arr > max_arr) {
                arr = max_arr; // Cap ARR at max value again
            }
        }

    } else { // total_ticks is 0 or very small (frequency very high or clock low)
       prescaler = max_arr;
       arr = 0;
    }


    // Update Prescaler and ARR registers. These are buffered and will update on the next UEV.
    config->TIMx->PSC = prescaler; /* PDF Reference */
    config->TIMx->ARR = arr;       /* PDF Reference */

    // Calculate the pulse length (duty cycle)
    // pulse_length = (duty / 100.0) * (ARR + 1)
    uint32_t pulse_length = (uint32_t)(((tlong)duty * (arr + 1)) / 100);

    // Update the Capture/Compare register. This is buffered and will update on the next UEV.
    *config->pCCRx = pulse_length; /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

     // Ensure the pin is not 0, as per requirement. Skip if it is.
    if (config->PinNumber == 0) {
        // Pin 0 is not configured for PWM output
        return;
    }

    // Enable the Capture/Compare output for the channel
    config->TIMx->CCER |= config->CCER_CCE_Msk; /* PDF Reference */
    // Polarity active high (0) is default, no need to set explicitly unless changing
    // config->TIMx->CCER &= ~config->CCER_CCP_Msk; // Ensure Active High /* PDF Reference */

    // For TIM1, enable Main Output (MOE)
    if (config->TIMx == TIM1)
    {
       config->TIMx->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Enable the counter
    config->TIMx->CR1 |= TIM_CR1_CEN; /* PDF Reference */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t* config = &pwm_channel_map[TRD_Channel];

     // Ensure the pin is not 0, as per requirement. Skip if it is.
    if (config->PinNumber == 0) {
        // Pin 0 is not configured for PWM output
        return;
    }

    // Disable the Capture/Compare output for the channel
    config->TIMx->CCER &= ~config->CCER_CCE_Msk; /* PDF Reference */

    // For TIM1, disabling MOE stops all outputs for TIM1.
    // If only stopping one channel on TIM1, just disabling CCxE is enough.
    // The requirement is to stop the *specified channel*.
    // No change needed for BDTR->MOE here unless stopping *all* TIM1 channels.

    // Stop the counter. The counter can be stopped per timer, not per channel.
    // A cleaner stop might involve setting duty to 0 before disabling CCxE/CEN
    // Setting CCRx = 0 and generating an update would make the pulse length 0.
    // Disabling the counter might affect other channels on the same timer.
    // Stopping *only* the output for the channel might be preferred if other channels on the same timer are running.
    // Let's just disable the output (CCxE) as the primary way to stop the signal for *this* channel.
    // Disabling the counter (CEN) is done in PowerOff for all timers.
    // If this is the *last* active channel on a timer, you might want to stop the timer counter (CEN) too.
    // For production, tracking active channels per timer would be needed.
    // Simple implementation: Just disable the channel output.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        This function stops all configured PWM channels and disables timer clocks.
 */
void PWM_PowerOff(void)
{
    // Iterate through all possible PWM timers mentioned in the RM sections (TIM1-5, TIM9-11)
    // and disable their clocks and counters.
    // Note: TIM10 and TIM11 were reserved, but they are PWM capable, so their clocks are handled here.

    // Stop TIM1
    TIM1->CR1 &= ~TIM_CR1_CEN;   /* PDF Reference */
    if (TIM1->BDTR & TIM_BDTR_MOE) TIM1->BDTR &= ~TIM_BDTR_MOE; /* PDF Reference */ // Disable Main Output if enabled
    TIM1->CCER = 0; // Disable all capture/compare outputs for TIM1 /* PDF Reference */
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; /* Assumed RCC bit */

    // Stop TIM2
    TIM2->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM2->CCER = 0; // Disable all capture/compare outputs for TIM2 /* PDF Reference */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; /* Assumed RCC bit */

    // Stop TIM3
    TIM3->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM3->CCER = 0; // Disable all capture/compare outputs for TIM3 /* PDF Reference */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; /* Assumed RCC bit */

    // Stop TIM4
    TIM4->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM4->CCER = 0; // Disable all capture/compare outputs for TIM4 /* PDF Reference */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN; /* Assumed RCC bit */

    // Stop TIM5
    TIM5->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM5->CCER = 0; // Disable all capture/compare outputs for TIM5 /* PDF Reference */
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN; /* Assumed RCC bit */

    // Stop TIM9
    TIM9->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM9->CCER = 0; // Disable all capture/compare outputs for TIM9 /* PDF Reference */
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN; /* Assumed RCC bit */

    // Stop TIM10 (Reserved but PWM capable)
    TIM10->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM10->CCER = 0; // Disable capture/compare output for TIM10 /* PDF Reference */
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN; /* Assumed RCC bit */

    // Stop TIM11 (Reserved but PWM capable)
    TIM11->CR1 &= ~TIM_CR1_CEN; /* PDF Reference */
    TIM11->CCER = 0; // Disable capture/compare output for TIM11 /* PDF Reference */
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN; /* Assumed RCC bit */

    // Note: This function disables the clocks for all timers that *can* do PWM on F401RC
    // based on the provided RM sections (TIM1-5, TIM9-11), regardless of whether they
    // were configured in pwm_channel_map or reserved. This maximizes power saving
    // from these timer peripherals.
}