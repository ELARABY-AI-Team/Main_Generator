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
#include <stdint.h> // For uint32_t, uint8_t etc.

/*
 *==============================================================================
 *                          PERIPHERAL REGISTER DEFINITIONS
 *==============================================================================
 * These definitions are based on the STM32F401RC Reference Manual (RM0368),
 * but the base addresses are standard for STM32F4 series and are assumed
 * as they were not provided in the PDF context.
 * Access via these pointers constitutes bare-metal register access.
 */

/* Assumed peripheral base addresses - please verify based on device headers */
#define PERIPH_BASE           (0x40000000UL)
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)

#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL) /* Not in provided GPIO context, but TIM registers mention A..E, H */
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)

#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL) /* Advanced-control timer */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL) /* General-purpose timer */
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL) /* General-purpose timer */
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL) /* General-purpose timer */
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL) /* General-purpose timer */
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL) /* General-purpose timer */
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL) /* General-purpose timer (Reserved) */
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL) /* General-purpose timer (Reserved) */

/* RCC Registers - assumed structure and bits */
typedef struct
{
  volatile uint32_t CR;         /* Clock control register */
  volatile uint32_t PLLCFGR;    /* PLL configuration register */
  volatile uint32_t CFGR;       /* Clock configuration register */
  volatile uint32_t CIR;        /* Clock interrupt register */
  volatile uint32_t AHB1RSTR;   /* AHB1 peripheral reset register */
  volatile uint32_t AHB2RSTR;   /* AHB2 peripheral reset register */
  volatile uint32_t AHB3RSTR;   /* AHB3 peripheral reset register */
  volatile uint32_t RESERVED0;
  volatile uint32_t APB1RSTR;   /* APB1 peripheral reset register */
  volatile uint32_t APB2RSTR;   /* APB2 peripheral reset register */
  volatile uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;    /* AHB1 peripheral clock enable register */ /* PDF Reference (implicit via GPIO) */
  volatile uint32_t AHB2ENR;    /* AHB2 peripheral clock enable register */
  volatile uint32_t AHB3ENR;    /* AHB3 peripheral clock enable register */
  volatile uint32_t RESERVED2;
  volatile uint32_t APB1ENR;    /* APB1 peripheral clock enable register */ /* PDF Reference (implicit via TIM) */
  volatile uint32_t APB2ENR;    /* APB2 peripheral clock enable register */ /* PDF Reference (implicit via TIM) */
} RCC_TypeDef;

#define RCC               ((RCC_TypeDef *)RCC_BASE)

/* GPIO Registers - structure based on PDF */
typedef struct
{
  volatile uint32_t MODER;    /* GPIO port mode register, offset 0x00 */      /* PDF Reference */
  volatile uint32_t OTYPER;   /* GPIO port output type register, offset 0x04 *//* PDF Reference */
  volatile uint32_t OSPEEDR;  /* GPIO port output speed register, 0x08 */      /* PDF Reference */
  volatile uint32_t PUPDR;    /* GPIO port pull-up/pull-down register, 0x0C *//* PDF Reference */
  volatile uint32_t IDR;      /* GPIO port input data register, 0x10 */        /* PDF Reference */
  volatile uint32_t ODR;      /* GPIO port output data register, 0x14 */       /* PDF Reference */
  volatile uint32_t BSRR;     /* GPIO port bit set/reset register, 0x18 */     /* PDF Reference */
  volatile uint32_t LCKR;     /* GPIO port configuration lock register, 0x1C *//* PDF Reference */
  volatile uint32_t AFRL;     /* GPIO alternate function low register, 0x20 *//* PDF Reference */
  volatile uint32_t AFRH;     /* GPIO alternate function high register, 0x24 *//* PDF Reference */
} GPIO_TypeDef;

#define GPIOA             ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB             ((GPIO_TypeDef *)GPIOB_BASE)
#发电C             ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD             ((GPIO_TypeDef *)GPIOD_BASE) /* Might not have all ports/pins on F401RC */
#define GPIOE             ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOH             ((GPIO_TypeDef *)GPIOH_BASE)


/* TIM Advanced-control Registers (TIM1) - structure based on PDF */
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1, offset 0x00 */         /* PDF Reference */
  volatile uint32_t CR2;      /* TIM control register 2, offset 0x04 */         /* PDF Reference */
  volatile uint32_t SMCR;     /* TIM slave mode control register, 0x08 */       /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register, 0x0C */   /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register, offset 0x10 */            /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register, 0x14 */         /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, 0x18 */   /* PDF Reference */
  volatile uint32_t CCMR2;    /* TIM capture/compare mode register 2, 0x1C */   /* PDF Reference */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, 0x20 */   /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter register, offset 0x24 */           /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register, offset 0x28 */         /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register, offset 0x2C */       /* PDF Reference */
  volatile uint32_t RCR;      /* TIM repetition counter register, 0x30 */       /* PDF Reference */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1, 0x34 */        /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2, 0x38 */        /* PDF Reference */
  volatile uint32_t CCR3;     /* TIM capture/compare register 3, 0x3C */        /* PDF Reference */
  volatile uint32_t CCR4;     /* TIM capture/compare register 4, 0x40 */        /* PDF Reference */
  volatile uint32_t BDTR;     /* TIM break and dead-time register, 0x44 */      /* PDF Reference */
  volatile uint32_t DCR;      /* TIM DMA control register, 0x48 */            /* PDF Reference */
  volatile uint32_t DMAR;     /* TIM DMA address for full transfer, 0x4C */     /* PDF Reference */
} TIM_Advanced_TypeDef;

#define TIM1              ((TIM_Advanced_TypeDef *)TIM1_BASE)


/* TIM General-purpose Registers (TIM2, TIM3, TIM4, TIM5) - structure based on PDF */
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1, offset 0x00 */         /* PDF Reference */
  volatile uint32_t CR2;      /* TIM control register 2, offset 0x04 */         /* PDF Reference */
  volatile uint32_t SMCR;     /* TIM slave mode control register, 0x08 */       /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register, 0x0C */   /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register, offset 0x10 */            /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register, 0x14 */         /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, 0x18 */   /* PDF Reference */
  volatile uint32_t CCMR2;    /* TIM capture/compare mode register 2, 0x1C */   /* PDF Reference */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, 0x20 */   /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter register, offset 0x24 */           /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register, offset 0x28 */         /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register, offset 0x2C */       /* PDF Reference */
  volatile uint32_t RESERVED0; /* Offset 0x30 */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1, 0x34 */        /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2, 0x38 */        /* PDF Reference */
  volatile uint32_t CCR3;     /* TIM capture/compare register 3, 0x3C */        /* PDF Reference */
  volatile uint32_t CCR4;     /* TIM capture/compare register 4, 0x40 */        /* PDF Reference */
  volatile uint32_t RESERVED1; /* Offset 0x44 */
  volatile uint32_t DCR;      /* TIM DMA control register, 0x48 */            /* PDF Reference */
  volatile uint32_t DMAR;     /* TIM DMA address for full transfer, 0x4C */     /* PDF Reference */
  volatile uint32_t OR;       /* TIM Option register, offset 0x50 (TIM2/TIM5 only) */ /* PDF Reference */
} TIM_General_TypeDef;

#define TIM2              ((TIM_General_TypeDef *)TIM2_BASE)
#define TIM3              ((TIM_General_TypeDef *)TIM3_BASE)
#define TIM4              ((TIM_General_TypeDef *)TIM4_BASE)
#define TIM5              ((TIM_General_TypeDef *)TIM5_BASE)


/* TIM General-purpose Registers (TIM9) - structure based on PDF */
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1, offset 0x00 */         /* PDF Reference */
  volatile uint32_t RESERVED0; /* Offset 0x04 */
  volatile uint32_t SMCR;     /* TIM slave mode control register, 0x08 */       /* PDF Reference */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register, 0x0C */   /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register, offset 0x10 */            /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register, 0x14 */         /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, 0x18 */   /* PDF Reference */
  volatile uint32_t RESERVED1; /* Offset 0x1C */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, 0x20 */   /* PDF Reference */
  volatile uint32_t CNT;      /* TIM counter register, offset 0x24 */           /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register, offset 0x28 */         /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register, offset 0x2C */       /* PDF Reference */
  volatile uint32_t RESERVED2; /* Offset 0x30 */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1, 0x34 */        /* PDF Reference */
  volatile uint32_t CCR2;     /* TIM capture/compare register 2, 0x38 */        /* PDF Reference */
  volatile uint32_t RESERVED3[4]; /* Offsets 0x3C - 0x48 */
  volatile uint32_t RESERVED4; /* Offset 0x4C */
} TIM_GP_TypeDef;

#define TIM9              ((TIM_GP_TypeDef *)TIM9_BASE)

/* TIM General-purpose Registers (TIM10, TIM11) - structure based on PDF */
typedef struct
{
  volatile uint32_t CR1;      /* TIM control register 1, offset 0x00 */         /* PDF Reference */
  volatile uint32_t RESERVED0[2]; /* Offsets 0x04 - 0x08 */
  volatile uint32_t DIER;     /* TIM DMA/interrupt enable register, 0x0C */   /* PDF Reference */
  volatile uint32_t SR;       /* TIM status register, offset 0x10 */            /* PDF Reference */
  volatile uint32_t EGR;      /* TIM event generation register, 0x14 */         /* PDF Reference */
  volatile uint32_t CCMR1;    /* TIM capture/compare mode register 1, 0x18 */   /* PDF Reference */
  volatile uint32_t RESERVED1[2]; /* Offsets 0x1C - 0x20 */
  volatile uint32_t CCER;     /* TIM capture/compare enable register, 0x20 */   /* PDF Reference */ // Note: Offset 0x20 matches TIM9 struct. PDF shows CCER at 0x20 for TIM10/11 too.
  volatile uint32_t CNT;      /* TIM counter register, offset 0x24 */           /* PDF Reference */
  volatile uint32_t PSC;      /* TIM prescaler register, offset 0x28 */         /* PDF Reference */
  volatile uint32_t ARR;      /* TIM auto-reload register, offset 0x2C */       /* PDF Reference */
  volatile uint32_t RESERVED2; /* Offset 0x30 */
  volatile uint32_t CCR1;     /* TIM capture/compare register 1, 0x34 */        /* PDF Reference */
  volatile uint32_t RESERVED3[7]; /* Offsets 0x38 - 0x4C */
  volatile uint32_t OR;       /* TIM Option register, offset 0x50 (TIM11 only) */ /* PDF Reference */ // PDF shows TIM11_OR at 0x50
} TIM_Basic_TypeDef;

#define TIM10             ((TIM_Basic_TypeDef *)TIM10_BASE)
#define TIM11             ((TIM_Basic_TypeDef *)TIM11_BASE)


/*
 *==============================================================================
 *                           REGISTER BIT DEFINITIONS
 *==============================================================================
 * Derived from the bitfield descriptions in the provided PDF content.
 */

/* RCC_AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN_Pos   (0U)
#define RCC_AHB1ENR_GPIOBEN_Pos   (1U)
#define RCC_AHB1ENR_GPIOCEN_Pos   (2U)
#define RCC_AHB1ENR_GPIODEN_Pos   (3U)
#define RCC_AHB1ENR_GPIOEEN_Pos   (4U)
#define RCC_AHB1ENR_GPIOHEN_Pos   (7U) /* PH0/PH1 */

#define RCC_AHB1ENR_GPIOAEN       (1U << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN       (1U << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOCEN       (1U << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIODEN       (1U << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIOEEN       (1U << RCC_AHB1ENR_GPIOEEN_Pos)
#define RCC_AHB1ENR_GPIOHEN       (1U << RCC_AHB1ENR_GPIOHEN_Pos)

/* RCC_APB1ENR */
#define RCC_APB1ENR_TIM2EN_Pos    (0U)
#define RCC_APB1ENR_TIM3EN_Pos    (1U)
#define RCC_APB1ENR_TIM4EN_Pos    (2U)
#define RCC_APB1ENR_TIM5EN_Pos    (3U)

#define RCC_APB1ENR_TIM2EN        (1U << RCC_APB1ENR_TIM2EN_Pos)
#define RCC_APB1ENR_TIM3EN        (1U << RCC_APB1ENR_TIM3EN_Pos)
#define RCC_APB1ENR_TIM4EN        (1U << RCC_APB1ENR_TIM4EN_Pos)
#define RCC_APB1ENR_TIM5EN        (1U << RCC_APB1ENR_TIM5EN_Pos)

/* RCC_APB2ENR */
#define RCC_APB2ENR_TIM1EN_Pos    (0U)
#define RCC_APB2ENR_TIM9EN_Pos    (16U)
#define RCC_APB2ENR_TIM10EN_Pos   (17U) /* Reserved Timer */
#define RCC_APB2ENR_TIM11EN_Pos   (18U) /* Reserved Timer */

#define RCC_APB2ENR_TIM1EN        (1U << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_TIM9EN        (1U << RCC_APB2ENR_TIM9EN_Pos)
#define RCC_APB2ENR_TIM10EN       (1U << RCC_APB2ENR_TIM10EN_Pos)
#define RCC_APB2ENR_TIM11EN       (1U << RCC_APB2ENR_TIM11EN_Pos)


/* GPIO_MODER - PDF Reference */
#define GPIO_MODER_MODE_Pos(y)    ((y)*2U)
#define GPIO_MODER_MODE(y)        (0x3U << GPIO_MODER_MODE_Pos(y))
#define GPIO_MODER_MODE_INPUT     (0x0U)
#define GPIO_MODER_MODE_OUTPUT    (0x1U)
#define GPIO_MODER_MODE_AF        (0x2U)
#define GPIO_MODER_MODE_ANALOG    (0x3U)

/* GPIO_OTYPER - PDF Reference */
#define GPIO_OTYPER_OT_Pos(y)     (y)
#define GPIO_OTYPER_OT(y)         (0x1U << GPIO_OTYPER_OT_Pos(y))
#define GPIO_OTYPER_PP            (0x0U) /* Push-pull */
#define GPIO_OTYPER_OD            (0x1U) /* Open-drain */

/* GPIO_OSPEEDR - PDF Reference */
#define GPIO_OSPEEDR_OSPEED_Pos(y) ((y)*2U)
#define GPIO_OSPEEDR_OSPEED(y)    (0x3U << GPIO_OSPEEDR_OSPEED_Pos(y))
#define GPIO_OSPEEDR_LOW          (0x0U)
#define GPIO_OSPEEDR_MEDIUM       (0x1U)
#define GPIO_OSPEEDR_HIGH         (0x2U)
#define GPIO_OSPEEDR_VERY_HIGH    (0x3U)

/* GPIO_PUPDR - PDF Reference */
#define GPIO_PUPDR_PUPD_Pos(y)    ((y)*2U)
#define GPIO_PUPDR_PUPD(y)        (0x3U << GPIO_PUPDR_PUPD_Pos(y))
#define GPIO_PUPDR_NONE           (0x0U)
#define GPIO_PUPDR_PU             (0x1U) /* Pull-up */
#define GPIO_PUPDR_PD             (0x2U) /* Pull-down */

/* GPIO_AFRL/AFRH - PDF Reference */
#define GPIO_AFR_AF_Pos(y)        ((y)*4U)
#define GPIO_AFR_AF(y)            (0xFU << GPIO_AFR_AF_Pos(y))

/* TIM_CR1 - PDF Reference */
#define TIM_CR1_CEN_Pos           (0U)
#define TIM_CR1_UDIS_Pos          (1U)
#define TIM_CR1_URS_Pos           (2U)
#define TIM_CR1_OPM_Pos           (3U) /* OPM for One Pulse Mode */
#define TIM_CR1_DIR_Pos           (4U) /* DIR for Direction (TIM1, TIM2-5) */
#define TIM_CR1_CMS_Pos           (5U) /* CMS for Center Aligned Mode (TIM1, TIM2-5) */
#define TIM_CR1_ARPE_Pos          (7U) /* ARPE for Auto-Reload Preload Enable */
#define TIM_CR1_CKD_Pos           (8U) /* CKD for Clock Division */

#define TIM_CR1_CEN               (1U << TIM_CR1_CEN_Pos)     /* Counter enable */
#define TIM_CR1_UDIS              (1U << TIM_CR1_UDIS_Pos)    /* Update disable */
#define TIM_CR1_URS               (1U << TIM_CR1_URS_Pos)     /* Update request source */
#define TIM_CR1_OPM               (1U << TIM_CR1_OPM_Pos)     /* One pulse mode */
#define TIM_CR1_DIR               (1U << TIM_CR1_DIR_Pos)     /* Direction */
#define TIM_CR1_CMS               (0x3U << TIM_CR1_CMS_Pos)   /* Center-aligned mode selection */
#define TIM_CR1_ARPE              (1U << TIM_CR1_ARPE_Pos)    /* Auto-reload preload enable */
#define TIM_CR1_CKD               (0x3U << TIM_CR1_CKD_Pos)   /* Clock division */

/* TIM_CCMR1 / TIM_CCMR2 - PDF Reference */
/* Output Compare Mode */
#define TIM_CCMR_OC_MODE_Pos(channel)   (((channel-1)%2) * 8U + 4U)
#define TIM_CCMR_OC_PE_Pos(channel)     (((channel-1)%2) * 8U + 3U)
#define TIM_CCMR_OC_FE_Pos(channel)     (((channel-1)%2) * 8U + 2U)
#define TIM_CCMR_CC_SELECTION_Pos(channel) (((channel-1)%2) * 8U + 0U)

#define TIM_CCMR_OC_MODE_Mask     (0x7U)
#define TIM_CCMR_OC_PE            (0x1U)
#define TIM_CCMR_OC_FE            (0x1U)
#define TIM_CCMR_CC_SELECTION_Mask (0x3U)

#define TIM_OCM_PWM1              (0x6U) /* PWM mode 1 (110) - PDF Reference */
#define TIM_OCM_PWM2              (0x7U) /* PWM mode 2 (111) - PDF Reference */
#define TIM_OCM_FORCED_LOW        (0x4U) /* Force inactive level (100) - PDF Reference */
#define TIM_OCM_FORCED_HIGH       (0x5U) /* Force active level (101) - PDF Reference */

/* TIM_CCER - PDF Reference */
#define TIM_CCER_CCE_Pos(channel)       (((channel-1)*4U) + 0U)
#define TIM_CCER_CCP_Pos(channel)       (((channel-1)*4U) + 1U)
#define TIM_CCER_CCNE_Pos(channel)      (((channel-1)*4U) + 2U) /* Complementary enable (TIM1) */
#define TIM_CCER_CCNP_Pos(channel)      (((channel-1)*4U) + 3U) /* Complementary polarity (TIM1) */

#define TIM_CCER_CCxE(channel)          (1U << TIM_CCER_CCE_Pos(channel))
#define TIM_CCER_CCxP(channel)          (1U << TIM_CCER_CCP_Pos(channel))
#define TIM_CCER_CCxNE(channel)         (1U << TIM_CCER_CCNE_Pos(channel))
#define TIM_CCER_CCxNP(channel)         (1U << TIM_CCER_CCNP_Pos(channel))

/* TIM_EGR - PDF Reference */
#define TIM_EGR_UG_Pos            (0U)
#define TIM_EGR_CC1G_Pos          (1U)
#define TIM_EGR_CC2G_Pos          (2U)
#define TIM_EGR_CC3G_Pos          (3U)
#define TIM_EGR_CC4G_Pos          (4U)
#define TIM_EGR_TG_Pos            (6U) /* Trigger Generation (TIM1, TIM2-9) */
#define TIM_EGR_BG_Pos            (7U) /* Break Generation (TIM1) */


#define TIM_EGR_UG                (1U << TIM_EGR_UG_Pos)
#define TIM_EGR_CC1G              (1U << TIM_EGR_CC1G_Pos)
#define TIM_EGR_CC2G              (1U << TIM_EGR_CC2G_Pos)
#define TIM_EGR_CC3G              (1U << TIM_EGR_CC3G_Pos)
#define TIM_EGR_CC4G              (1U << TIM_EGR_CC4G_Pos)
#define TIM_EGR_TG                (1U << TIM_EGR_TG_Pos)
#define TIM_EGR_BG                (1U << TIM_EGR_BG_Pos)

/* TIM_SR - PDF Reference */
#define TIM_SR_UIF_Pos            (0U)
#define TIM_SR_CC1IF_Pos          (1U)
#define TIM_SR_CC2IF_Pos          (2U)
#define TIM_SR_CC3IF_Pos          (3U)
#define TIM_SR_CC4IF_Pos          (4U)
#define TIM_SR_TIF_Pos            (6U) /* Trigger Flag (TIM1, TIM2-9) */
#define TIM_SR_BIF_Pos            (7U) /* Break Flag (TIM1) */

#define TIM_SR_UIF                (1U << TIM_SR_UIF_Pos)
#define TIM_SR_CC1IF              (1U << TIM_SR_CC1IF_Pos)
#define TIM_SR_CC2IF              (1U << TIM_SR_CC2IF_Pos)
#define TIM_SR_CC3IF              (1U << TIM_SR_CC3IF_Pos)
#define TIM_SR_CC4IF              (1U << TIM_SR_CC4IF_Pos)
#define TIM_SR_TIF                (1U << TIM_SR_TIF_Pos)
#define TIM_SR_BIF                (1U << TIM_SR_BIF_Pos)


/* TIM1_BDTR - PDF Reference */
#define TIM_BDTR_DTG_Pos          (0U)
#define TIM_BDTR_LOCK_Pos         (8U)
#define TIM_BDTR_OSSI_Pos         (10U)
#define TIM_BDTR_OSSR_Pos         (11U)
#define TIM_BDTR_BKE_Pos          (12U)
#define TIM_BDTR_BKP_Pos          (13U)
#define TIM_BDTR_AOE_Pos          (14U)
#define TIM_BDTR_MOE_Pos          (15U)

#define TIM_BDTR_DTG_Mask         (0xFFU << TIM_BDTR_DTG_Pos)
#define TIM_BDTR_LOCK_Mask        (0x3U << TIM_BDTR_LOCK_Pos)
#define TIM_BDTR_OSSI             (1U << TIM_BDTR_OSSI_Pos)
#define TIM_BDTR_OSSR             (1U << TIM_BDTR_OSSR_Pos)
#define TIM_BDTR_BKE              (1U << TIM_BDTR_BKE_Pos)
#define TIM_BDTR_BKP              (1U << TIM_BDTR_BKP_Pos)
#define TIM_BDTR_AOE              (1U << TIM_BDTR_AOE_Pos)
#define TIM_BDTR_MOE              (1U << TIM_BDTR_MOE_Pos)


/*
 *==============================================================================
 *                             PWM CHANNEL CONFIGURATION
 *==============================================================================
 */

/**
 * @brief Structure to hold the configuration for a single PWM channel.
 *        This maps a TRD_Channel_t value to specific hardware registers and bits.
 */
typedef struct
{
    void              *TIMx;               /**< Pointer to the Timer peripheral (e.g., TIM1, TIM2) */
    GPIO_TypeDef      *GPIOx;              /**< Pointer to the GPIO port (e.g., GPIOA, GPIOB) */
    uint32_t           TIM_Clk_Enable_Bit; /**< Corresponding bit in RCC to enable Timer clock */
    uint32_t           GPIO_Clk_Enable_Bit;/**< Corresponding bit in RCC to enable GPIO clock */
    uint8_t            Channel;            /**< Timer channel number (1-4) */
    uint8_t            AF;                 /**< GPIO Alternate Function number for this pin/timer */
    uint8_t            Timer_Is_Advanced;  /**< Flag: 1 if TIM1 (Advanced-control), 0 otherwise */
    uint8_t            Timer_Is_16Bit;     /**< Flag: 1 if 16-bit timer (TIM1, 3, 4, 9, 10, 11), 0 if 32-bit (TIM2, 5) */
} PWM_Channel_Config_t;

/**
 * @brief Mapping from TRD_Channel_t to hardware configuration.
 *        This array defines which Timer, Channel, and GPIO pin/AF combo corresponds
 *        to each abstract TRD_Channel_t value.
 *        Assumed mapping based on common STM32F401RC pin assignments and the provided
 *        GPIO and TIM sections of the reference manual, avoiding pin 0.
 */
static const PWM_Channel_Config_t pwm_channel_map[] =
{
    // Example mapping (replace with actual required mapping)
    // { TIMx_BASE, GPIOx_BASE, RCC_TIMxEN_BIT, RCC_GPIOxEN_BIT, Channel, Pin, AF, Is_Advanced, Is_16Bit }

    // TIM1 Channels (Advanced, 16-bit)
    { TIM1, GPIOA, RCC_APB2ENR_TIM1EN, RCC_AHB1ENR_GPIOAEN, 1, 8,  1, 1, 1 }, // PA8  -> TIM1_CH1  /* Assumed PWM config - please verify */
    { TIM1, GPIOA, RCC_APB2ENR_TIM1EN, RCC_AHB1ENR_GPIOAEN, 2, 9,  1, 1, 1 }, // PA9  -> TIM1_CH2  /* Assumed PWM config - please verify */
    { TIM1, GPIOA, RCC_APB2ENR_TIM1EN, RCC_AHB1ENR_GPIOAEN, 3, 10, 1, 1, 1 }, // PA10 -> TIM1_CH3  /* Assumed PWM config - please verify */
    { TIM1, GPIOA, RCC_APB2ENR_TIM1EN, RCC_AHB1ENR_GPIOAEN, 4, 11, 1, 1, 1 }, // PA11 -> TIM1_CH4  /* Assumed PWM config - please verify */

    // TIM2 Channels (General, 32-bit)
    { TIM2, GPIOA, RCC_APB1ENR_TIM2EN, RCC_AHB1ENR_GPIOAEN, 1, 1,  1, 0, 0 }, // PA1  -> TIM2_CH1  /* Assumed PWM config - please verify */ // Avoids PA0
    { TIM2, GPIOA, RCC_APB1ENR_TIM2EN, RCC_AHB1ENR_GPIOAEN, 2, 2,  1, 0, 0 }, // PA2  -> TIM2_CH2  /* Assumed PWM config - please verify */
    { TIM2, GPIOA, RCC_APB1ENR_TIM2EN, RCC_AHB1ENR_GPIOAEN, 3, 3,  1, 0, 0 }, // PA3  -> TIM2_CH3  /* Assumed PWM config - please verify */
    { TIM2, GPIOB, RCC_APB1ENR_TIM2EN, RCC_AHB1ENR_GPIOBEN, 4, 11, 1, 0, 0 }, // PB11 -> TIM2_CH4  /* Assumed PWM config - please verify */

    // TIM3 Channels (General, 16-bit)
    { TIM3, GPIOB, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOBEN, 1, 4,  2, 0, 1 }, // PB4  -> TIM3_CH1  /* Assumed PWM config - please verify */
    { TIM3, GPIOB, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOBEN, 2, 5,  2, 0, 1 }, // PB5  -> TIM3_CH2  /* Assumed PWM config - please verify */
    { TIM3, GPIOC, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOCEN, 3, 8,  2, 0, 1 }, // PC8  -> TIM3_CH3  /* Assumed PWM config - please verify */ // Avoids PB0
    { TIM3, GPIOB, RCC_APB1ENR_TIM3EN, RCC_AHB1ENR_GPIOBEN, 4, 1,  2, 0, 1 }, // PB1  -> TIM3_CH4  /* Assumed PWM config - please verify */ // Avoids PB0

    // TIM4 Channels (General, 16-bit)
    { TIM4, GPIOB, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, 1, 6,  2, 0, 1 }, // PB6  -> TIM4_CH1  /* Assumed PWM config - please verify */
    { TIM4, GPIOB, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, 2, 7,  2, 0, 1 }, // PB7  -> TIM4_CH2  /* Assumed PWM config - please verify */
    { TIM4, GPIOB, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, 3, 8,  2, 0, 1 }, // PB8  -> TIM4_CH3  /* Assumed PWM config - please verify */
    { TIM4, GPIOB, RCC_APB1ENR_TIM4EN, RCC_AHB1ENR_GPIOBEN, 4, 9,  2, 0, 1 }, // PB9  -> TIM4_CH4  /* Assumed PWM config - please verify */

    // TIM5 Channels (General, 32-bit)
    { TIM5, GPIOA, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, 1, 12, 2, 0, 0 }, // PA12 -> TIM5_CH1  /* Assumed PWM config - please verify */ // Avoids PA0
    { TIM5, GPIOA, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOAEN, 2, 15, 2, 0, 0 }, // PA15 -> TIM5_CH2  /* Assumed PWM config - please verify */ // Avoids PA1 reuse with TIM2
    { TIM5, GPIOC, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOCEN, 3, 7,  2, 0, 0 }, // PC7  -> TIM5_CH3  /* Assumed PWM config - please verify */ // Avoids PB0 reuse
    { TIM5, GPIOC, RCC_APB1ENR_TIM5EN, RCC_AHB1ENR_GPIOCEN, 4, 9,  2, 0, 0 }, // PC9  -> TIM5_CH4  /* Assumed PWM config - please verify */ // Avoids PB1 reuse

    // TIM9 Channels (General, 16-bit)
    { TIM9, GPIOE, RCC_APB2ENR_TIM9EN, RCC_AHB1ENR_GPIOEEN, 1, 5,  3, 0, 1 }, // PE5  -> TIM9_CH1  /* Assumed PWM config - please verify */ // Avoids PA2 reuse
    { TIM9, GPIOE, RCC_APB2ENR_TIM9EN, RCC_AHB1ENR_GPIOEEN, 2, 6,  3, 0, 1 }, // PE6  -> TIM9_CH2  /* Assumed PWM config - please verify */ // Avoids PA3 reuse
};

// Note: TIM10 and TIM11 (TIM_Basic_TypeDef) are reserved for other purposes (e.g., OS/delay)
// and are not included in the pwm_channel_map.

#define NUM_PWM_CHANNELS (sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]))

// Flag to track initialized channels
static uint8_t initialized_channels[NUM_PWM_CHANNELS] = {0};

/*
 *==============================================================================
 *                             HELPER FUNCTIONS
 *==============================================================================
 */

/**
 * @brief Get the timer clock frequency based on bus and assumed configuration.
 *        This is a critical assumption and needs to be verified based on the
 *        actual system clock tree setup (SYSCLK, AHBx and APBx prescalers).
 *        Assumed: APBx prescaler is 1 or is >1 and the timer clock multiplier is active.
 *        This means the timer clock is the same as the APBx frequency.
 *        For STM32F401RC, Max APB1/APB2 freq is 84MHz.
 */
static uint32_t get_timer_clock_freq(void *TIMx_Base)
{
    /*
     * Assumed Clock Frequencies:
     * APB1 Timer Clock: 84 MHz (TIM2, TIM3, TIM4, TIM5)
     * APB2 Timer Clock: 84 MHz (TIM1, TIM9, TIM10, TIM11)
     * This assumes the APBx prescaler is 1, or if greater than 1, the
     * timer clock is twice the APBx clock (common configuration).
     * Check RCC->CFGR to verify actual clock setup.
     */
    if (TIMx_Base == TIM1 || TIMx_Base == TIM9 || TIMx_Base == TIM10 || TIMx_Base == TIM11)
    {
        // Timers on APB2 bus
        // Read APB2 prescaler from RCC->CFGR bits 15:13
        // If prescaler is 1, timer clock = PCLK2. If prescaler > 1, timer clock = 2*PCLK2.
        // Assuming PCLK2 is 84MHz (max) or half of timer clock if prescaler > 1.
        return 84000000UL; /* Assumed clock frequency - please verify */
    }
    else if (TIMx_Base == TIM2 || TIMx_Base == TIM3 || TIMx_Base == TIM4 || TIMx_Base == TIM5)
    {
        // Timers on APB1 bus
        // Read APB1 prescaler from RCC->CFGR bits 10:8
        // Similar assumption: timer clock is 84MHz.
         return 84000000UL; /* Assumed clock frequency - please verify */
    }
    return 0; // Should not happen for supported timers
}


/**
 * @brief Calculates ARR and PSC values for a given frequency and duty cycle.
 *        Maximizes ARR for better resolution within the timer's capabilities.
 * @param timer_clock Timer clock frequency.
 * @param frequency Desired PWM frequency.
 * @param is_16bit_timer Flag for 16-bit or 32-bit timer.
 * @param arr Pointer to store the calculated auto-reload value.
 * @param psc Pointer to store the calculated prescaler value.
 * @return 1 if calculation is successful, 0 otherwise.
 */
static uint8_t calculate_timer_params(uint32_t timer_clock, tlong frequency, uint8_t is_16bit_timer, uint32_t *arr, uint16_t *psc)
{
    if (frequency == 0 || timer_clock == 0)
    {
        // Cannot generate a signal with 0 frequency
        return 0;
    }

    uint64_t total_ticks = (uint64_t)timer_clock / frequency; /* PDF Reference (Frequency = fCK_CNT / (ARR+1)) */

    if (total_ticks == 0)
    {
        // Frequency is higher than timer clock
        return 0;
    }

    uint32_t max_arr = is_16bit_timer ? 0xFFFF : 0xFFFFFFFF; /* PDF Reference (ARR register size) */

    // Iterate through possible prescaler values to find a valid ARR
    // Iterate from high PSC values downwards to maximize ARR first, improving resolution.
    // Max PSC is 0xFFFF (16-bit)
    for (uint32_t presc = 0xFFFF; presc > 0; presc--)
    {
        uint64_t arr_plus_1 = total_ticks / (presc + 1);
        if (arr_plus_1 > 0 && (total_ticks % (presc + 1)) == 0)
        {
            uint32_t calculated_arr = arr_plus_1 - 1;
            if (calculated_arr <= max_arr)
            {
                *psc = (uint16_t)presc;
                *arr = calculated_arr;
                return 1; // Found a valid PSC/ARR pair
            }
        }
    }

    // If no solution found by iterating high PSC, try PSC = 0 (max resolution)
    if (total_ticks - 1 <= max_arr)
    {
        *psc = 0;
        *arr = total_ticks - 1;
         return 1;
    }

    return 0; // No valid PSC/ARR pair found
}

/*
 *==============================================================================
 *                          FUNCTIONS
 *==============================================================================
 */

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // 1. Enable GPIO clock
    RCC->AHB1ENR |= config->GPIO_Clk_Enable_Bit; /* PDF Reference (implicit via GPIO section) */

    // 2. Configure GPIO pin for Alternate Function (AF)
    uint32_t moder_shift = GPIO_MODER_MODE_Pos(config->Pin); /* PDF Reference */
    config->GPIOx->MODER &= ~(GPIO_MODER_MODE(config->Pin)); /* Clear mode bits - PDF Reference */
    config->GPIOx->MODER |= (GPIO_MODER_MODE_AF << moder_shift); /* Set mode to Alternate Function (10) - PDF Reference */

    // Configure Output Type: Push-pull (0) - PDF Reference
    uint32_t otyper_shift = GPIO_OTYPER_OT_Pos(config->Pin); /* PDF Reference */
    config->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT(config->Pin)); /* Ensure Push-pull (0) - PDF Reference */

    // Configure Output Speed: High speed (10) - PDF Reference
    uint32_t ospeedr_shift = GPIO_OSPEEDR_OSPEED_Pos(config->Pin); /* PDF Reference */
    config->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED(config->Pin)); /* Clear speed bits - PDF Reference */
    config->GPIOx->OSPEEDR |= (GPIO_OSPEEDR_HIGH << ospeedr_shift); /* Set speed to High (10) - PDF Reference */

    // Configure Pull-up/Pull-down: No pull-up/pull-down (00) - PDF Reference
    uint32_t pupdr_shift = GPIO_PUPDR_PUPD_Pos(config->Pin); /* PDF Reference */
    config->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD(config->Pin)); /* Ensure No PUPD (00) - PDF Reference */

    // Configure Alternate Function selection (AFRL/AFRH)
    uint32_t afr_shift = GPIO_AFR_AF_Pos(config->Pin % 8); /* PDF Reference */
    if (config->Pin < 8)
    {
        config->GPIOx->AFRL &= ~(GPIO_AFR_AF(config->Pin % 8)); /* Clear AF bits (0-7) - PDF Reference */
        config->GPIOx->AFRL |= (config->AF << afr_shift);      /* Set AF - PDF Reference */
    }
    else
    {
        config->GPIOx->AFRH &= ~(GPIO_AFR_AF(config->Pin % 8)); /* Clear AF bits (8-15) - PDF Reference */
        config->GPIOx->AFRH |= (config->AF << afr_shift);      /* Set AF - PDF Reference */
    }

    // 3. Enable Timer clock
    if (config->TIMx == TIM1 || config->TIMx == TIM9 || config->TIMx == TIM10 || config->TIMx == TIM11)
    {
         RCC->APB2ENR |= config->TIM_Clk_Enable_Bit; /* PDF Reference (implicit via TIM sections) */
         // Dummy read to ensure clock is ready
         (void) (RCC->APB2ENR);
    }
    else // TIM2, TIM3, TIM4, TIM5
    {
        RCC->APB1ENR |= config->TIM_Clk_Enable_Bit; /* PDF Reference (implicit via TIM sections) */
        // Dummy read to ensure clock is ready
        (void) (RCC->APB1ENR);
    }


    // 4. Configure Timer for PWM (initial state: stopped, low frequency, 0% duty)
    // Stop the counter first
    uint32_t cr1_reg = ((volatile uint32_t *)config->TIMx)[0]; // Access CR1 via pointer arithmetic
    cr1_reg &= ~TIM_CR1_CEN; /* PDF Reference */
    ((volatile uint32_t *)config->TIMx)[0] = cr1_reg;

    // Configure time-base: Upcounting mode (DIR=0, CMS=00), ARPE=1
    cr1_reg &= ~TIM_CR1_DIR;    /* Clear DIR for Upcounting - PDF Reference */
    cr1_reg &= ~TIM_CR1_CMS;    /* Clear CMS for Edge-aligned mode - PDF Reference */
    cr1_reg |= TIM_CR1_ARPE;    /* Enable ARR preload buffer - PDF Reference */
    cr1_reg &= ~TIM_CR1_CKD;    /* Clear CKD bits - PDF Reference */
    // Let CKD remain 00 for tDTS = tCK_INT (default)

    ((volatile uint32_t *)config->TIMx)[0] = cr1_reg;

    // Configure Output Compare mode for the specific channel
    volatile uint32_t *ccmr_reg;
    uint8_t ccmr_shift;
    uint8_t is_oc_mode = 0;

    // Determine which CCMR register and shift to use
    if (config->Channel == 1 || config->Channel == 2)
    {
        ccmr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x18); // TIMx_CCMR1 offset 0x18 /* PDF Reference */
        ccmr_shift = ((config->Channel - 1) % 2) * 8U; /* PDF Reference */
        // Check if the timer supports this channel as output (TIM9 has CCMR1/CCMR2, TIM10/11 only CCMR1)
        if (config->TIMx == TIM9 || config->TIMx == TIM10 || config->TIMx == TIM11 || config->Channel <=4) // All supported timers have at least CC1
        {
             is_oc_mode = 1; // Assume supported as output
        } else {
             // This configuration is not possible according to the PDF for TIM2-5 Ch3/4 or TIM9 Ch3/4
             // This should not happen with the correct pwm_channel_map, but as a safeguard:
             return;
        }
    }
    else if ((config->Channel == 3 || config->Channel == 4) && config->Timer_Is_Advanced == 0 && config->TIMx != TIM9 && config->TIMx != TIM10 && config->TIMx != TIM11) // TIM2-5 channels 3 & 4
    {
         ccmr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x1C); // TIMx_CCMR2 offset 0x1C /* PDF Reference */
         ccmr_shift = ((config->Channel - 1) % 2) * 8U; /* PDF Reference */
         is_oc_mode = 1; // Assume supported as output
    } else {
         // Channel 3 or 4 not supported for TIM9, TIM10, TIM11 or Advanced TIM (handled above)
         return;
    }


    if (is_oc_mode)
    {
        uint32_t ccmr_val = *ccmr_reg;

        // Set as Output mode (CCxS = 00) - PDF Reference
        ccmr_val &= ~(TIM_CCMR_CC_SELECTION_Mask << ccmr_shift); /* PDF Reference */

        // Configure PWM Mode 1 (OCxM = 110) - PDF Reference
        ccmr_val &= ~(TIM_CCMR_OC_MODE_Mask << ccmr_shift); /* Clear mode bits - PDF Reference */
        ccmr_val |= (TIM_OCM_PWM1 << (ccmr_shift + 4));     /* Set PWM1 mode (110) - PDF Reference */

        // Enable Output Compare Preload enable (OCxPE = 1) - PDF Reference
        ccmr_val |= (TIM_CCMR_OC_PE << (ccmr_shift + 3)); /* PDF Reference */

        // OCxFE (Fast enable) = 0 (default) - PDF Reference

        *ccmr_reg = ccmr_val;

        // Configure Capture/Compare Enable Register (CCER)
        volatile uint32_t *ccer_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x20); // TIMx_CCER offset 0x20 /* PDF Reference */
        uint32_t ccer_val = *ccer_reg;

        // CCxE = 0 (Disable output for now) - PDF Reference
        ccer_val &= ~TIM_CCER_CCxE(config->Channel); /* PDF Reference */

        // CCxP = 0 (Active high polarity) - PDF Reference
        ccer_val &= ~TIM_CCER_CCxP(config->Channel); /* PDF Reference */

        // For TIM1 (Advanced-control), configure complementary output if needed.
        // For simple PWM output, keep complementary output disabled and low.
        if (config->Timer_Is_Advanced)
        {
             ccer_val &= ~TIM_CCER_CCxNE(config->Channel); /* Disable complementary output - PDF Reference */
             ccer_val &= ~TIM_CCER_CCxNP(config->Channel); /* Clear complementary polarity - PDF Reference */

             // Also set BDTR register for TIM1 (Main Output Enable etc.)
             // Set initial BDTR for MOE to be controlled by software/AOE=0
             // Keep dead time, break, OSSI/OSSR at reset (0) for now.
             // Will enable MOE in PWM_Start.
             TIM1->BDTR &= ~TIM_BDTR_MOE; /* Disable Main Output initially - PDF Reference */
             TIM1->BDTR &= ~TIM_BDTR_AOE; /* Disable Automatic Output Enable - PDF Reference */
        }

        *ccer_reg = ccer_val;

        // Set initial PSC, ARR, CCR values (e.g., max period, 0 duty)
        // This ensures a defined state before Set_Freq is called.
        uint32_t timer_clock = get_timer_clock_freq(config->TIMx);
        uint32_t initial_arr = config->Timer_Is_16Bit ? 0xFFFF : 0xFFFFFFFF; /* PDF Reference (ARR max value) */
        uint16_t initial_psc = 0; // Use PSC=0 initially for highest frequency/resolution

        // Check if initial_arr > 0 to prevent division by zero if timer is 32-bit and initial_arr is 0
        if (initial_arr > 0) {
             // Set initial duty cycle to 0 (output low)
             uint32_t initial_ccr = 0; // Setting CCR=0 means CNT<0 is never true, output is low in PWM1
             ((volatile uint32_t *)((uint32_t)config->TIMx + 0x28))[0] = initial_psc; // TIMx_PSC offset 0x28 /* PDF Reference */
             ((volatile uint32_t *)((uint32_t)config->TIMx + 0x2C))[0] = initial_arr; // TIMx_ARR offset 0x2C /* PDF Reference */

             volatile uint32_t *ccr_reg;
             if (config->Channel == 1) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x34); /* CCR1 offset 0x34 - PDF Reference */
             else if (config->Channel == 2) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x38); /* CCR2 offset 0x38 - PDF Reference */
             else if (config->Channel == 3 && !config->Timer_Is_Advanced && config->TIMx != TIM9) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x3C); /* CCR3 offset 0x3C - PDF Reference */
             else if (config->Channel == 4 && !config->Timer_Is_Advanced && config->TIMx != TIM9) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x40); /* CCR4 offset 0x40 - PDF Reference */
             else ccr_reg = 0; // Should not happen with correct map

             if (ccr_reg)
             {
                 *ccr_reg = initial_ccr; /* PDF Reference */
             }
        } else {
            // Timer is 32-bit and initial_arr is 0xffffffff, this should be fine.
            // This check is mainly for 16-bit timers if initial_arr was set to 0.
        }


        // Generate an update event to load all configured values from preload to active registers
        // This also clears the counter and prescaler.
        ((volatile uint32_t *)config->TIMx)[5] |= TIM_EGR_UG; // TIMx_EGR offset 0x14, bit 0 /* PDF Reference */
        // Wait for update flag or check URS bit effect? URS=0 means UG sets UIF.
        // Clear the update flag if it was set
        if ((((volatile uint32_t *)config->TIMx)[4]) & TIM_SR_UIF) // TIMx_SR offset 0x10, bit 0 /* PDF Reference */
        {
             ((volatile uint32_t *)config->TIMx)[4] &= ~TIM_SR_UIF; /* Clear UIF - PDF Reference */
        }

        // Mark channel as initialized
        initialized_channels[TRD_Channel] = 1;
    }
}

/**
 * @brief Sets the PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency Desired PWM frequency in Hz.
 * @param duty Desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
     if (TRD_Channel >= NUM_PWM_CHANNELS || !initialized_channels[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    if (duty > 100)
    {
        duty = 100; // Cap duty cycle
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t timer_clock = get_timer_clock_freq(config->TIMx);
    uint32_t arr;
    uint16_t psc;

    if (!calculate_timer_params(timer_clock, frequency, config->Timer_Is_16Bit, &arr, &psc))
    {
        // Could not find valid parameters for the requested frequency
        // Keep previous settings or set to a safe default (e.g., 0% duty)
        // Setting duty to 0% is handled by the duty calculation below.
        // For frequency that cannot be achieved, maybe disable output?
        // For now, just proceed with potentially inaccurate frequency or keep old ARR/PSC if calculation failed.
        // A robust implementation might return an error code or log a warning.
        // For production, calculation should ideally be done offline or handle errors.
         ((volatile uint32_t *)((uint32_t)config->TIMx + 0x34))[0] = 0; // Set CCR to 0 (0% duty) for this channel if calculation failed
         return;
    }


    // Calculate Capture Compare Register value for the desired duty cycle
    // PWM Mode 1 (active high): Output is high when CNT < CCRx.
    // Duty Cycle = CCRx / (ARR + 1) * 100
    // CCRx = (Duty * (ARR + 1)) / 100
    uint32_t ccr;
    if (duty == 0)
    {
        ccr = 0; // 0% duty cycle /* PDF Reference (implicit via PWM mode description) */
    }
    else if (duty == 100)
    {
        ccr = arr + 1; // 100% duty cycle (or slightly > arr) /* PDF Reference (implicit via PWM mode description) */
        // Setting CCRx > ARR results in OCxREF held high for PWM Mode 1. Setting to ARR+1 works.
    }
    else
    {
        // Use 64-bit intermediate to prevent overflow
        ccr = (uint32_t)(((uint64_t)duty * (arr + 1)) / 100); /* PDF Reference (implicit via PWM mode description) */
    }


    // Update Timer registers
    // Setting PSC and ARR while counter is running can affect the current period.
    // Best practice is usually to stop, reconfigure, then start, or use preload/update.
    // We are using preload (ARPE, OCxPE enabled in Init), so we update preload registers
    // and generate an update event.
    ((volatile uint32_t *)((uint32_t)config->TIMx + 0x28))[0] = psc; // TIMx_PSC offset 0x28 /* PDF Reference */
    ((volatile uint32_t *)((uint32_t)config->TIMx + 0x2C))[0] = arr; // TIMx_ARR offset 0x2C /* PDF Reference */

    volatile uint32_t *ccr_reg;
    if (config->Channel == 1) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x34); /* CCR1 offset 0x34 - PDF Reference */
    else if (config->Channel == 2) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x38); /* CCR2 offset 0x38 - PDF Reference */
    else if (config->Channel == 3 && !config->Timer_Is_Advanced && config->TIMx != TIM9) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x3C); /* CCR3 offset 0x3C - PDF Reference */
    else if (config->Channel == 4 && !config->Timer_Is_Advanced && config->TIMx != TIM9) ccr_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x40); /* CCR4 offset 0x40 - PDF Reference */
    else return; // Should not happen with correct map

    *ccr_reg = ccr; /* PDF Reference */

    // Generate an update event to load the new ARR, PSC, CCR values from preload to active registers
    // This happens at the next period boundary if URS=0 (configured in Init).
    ((volatile uint32_t *)config->TIMx)[5] |= TIM_EGR_UG; // TIMx_EGR offset 0x14, bit 0 /* PDF Reference */
    // Clear the update flag if it was set by UG (URS=0)
    if ((((volatile uint32_t *)config->TIMx)[4]) & TIM_SR_UIF) // TIMx_SR offset 0x10, bit 0 /* PDF Reference */
    {
         ((volatile uint32_t *)config->TIMx)[4] &= ~TIM_SR_UIF; /* Clear UIF - PDF Reference */
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= NUM_PWM_CHANNELS || !initialized_channels[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the specific channel output
    volatile uint32_t *ccer_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x20); // TIMx_CCER offset 0x20 /* PDF Reference */
    *ccer_reg |= TIM_CCER_CCxE(config->Channel); /* PDF Reference */

    // For TIM1 (Advanced-control), enable Main Output
    if (config->Timer_Is_Advanced)
    {
        TIM1->BDTR |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Enable the Timer Counter
    uint32_t cr1_reg = ((volatile uint32_t *)config->TIMx)[0]; // Access CR1
    cr1_reg |= TIM_CR1_CEN; /* PDF Reference */
    ((volatile uint32_t *)config->TIMx)[0] = cr1_reg;
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 *        The timer counter continues running.
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     if (TRD_Channel >= NUM_PWM_CHANNELS || !initialized_channels[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the specific channel output (CCxE)
    volatile uint32_t *ccer_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x20); // TIMx_CCER offset 0x20 /* PDF Reference */
    *ccer_reg &= ~TIM_CCER_CCxE(config->Channel); /* PDF Reference */

    // For TIM1 (Advanced-control), consider disabling MOE if this is the last active channel.
    // However, this function is per-channel stop. A full stop function is needed to clear MOE safely.
    // PWM_PowerOff handles disabling MOE for TIM1.
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        Affects all channels configured via this driver.
 */
void PWM_PowerOff(void)
{
    for (TRD_Channel_t i = 0; i < NUM_PWM_CHANNELS; ++i)
    {
        if (initialized_channels[i])
        {
            const PWM_Channel_Config_t *config = &pwm_channel_map[i];

            // Disable the specific channel output (CCxE)
            volatile uint32_t *ccer_reg = (volatile uint32_t *)((uint32_t)config->TIMx + 0x20); /* TIMx_CCER offset 0x20 - PDF Reference */
            *ccer_reg &= ~TIM_CCER_CCxE(config->Channel); /* PDF Reference */

            // Stop the Timer Counter
            uint32_t cr1_reg = ((volatile uint32_t *)config->TIMx)[0]; // Access CR1
            cr1_reg &= ~TIM_CR1_CEN; /* PDF Reference */
            ((volatile uint32_t *)config->TIMx)[0] = cr1_reg;

            // Reset GPIO pin back to input floating state
            uint32_t moder_shift = GPIO_MODER_MODE_Pos(config->Pin); /* PDF Reference */
            config->GPIOx->MODER &= ~(GPIO_MODER_MODE(config->Pin)); /* Clear mode bits - PDF Reference */
            config->GPIOx->MODER |= (GPIO_MODER_MODE_INPUT << moder_shift); /* Set mode to Input (00) - PDF Reference */

            // Clear Alternate Function selection
             uint32_t afr_shift = GPIO_AFR_AF_Pos(config->Pin % 8); /* PDF Reference */
            if (config->Pin < 8)
            {
                config->GPIOx->AFRL &= ~(GPIO_AFR_AF(config->Pin % 8)); /* Clear AF bits (0-7) - PDF Reference */
            }
            else
            {
                config->GPIOx->AFRH &= ~(GPIO_AFR_AF(config->Pin % 8)); /* Clear AF bits (8-15) - PDF Reference */
            }

            // Optional: Clear OTYPER, OSPEEDR, PUPDR to reset values (usually 0)
            uint32_t otyper_shift = GPIO_OTYPER_OT_Pos(config->Pin); /* PDF Reference */
            config->GPIOx->OTYPER &= ~(GPIO_OTYPER_OT(config->Pin)); /* Ensure Push-pull (0) - PDF Reference */

            uint32_t ospeedr_shift = GPIO_OSPEEDR_OSPEED_Pos(config->Pin); /* PDF Reference */
            config->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED(config->Pin)); /* Clear speed bits - PDF Reference */

            uint32_t pupdr_shift = GPIO_PUPDR_PUPD_Pos(config->Pin); /* PDF Reference */
            config->GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD(config->Pin)); /* Ensure No PUPD (00) - PDF Reference */

            // Clear the initialized flag
            initialized_channels[i] = 0;
        }
    }

    // For TIM1 (Advanced-control), disable Main Output if it was enabled
    if (TIM1->BDTR & TIM_BDTR_MOE) /* Check if MOE is set - PDF Reference */
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE; /* Disable Main Output - PDF Reference */
    }

    // Disable clocks for all used Timers and GPIOs
    // This is done by iterating through the map and disabling clocks if they were enabled.
    // A more robust approach would track enabled peripherals, but for this implementation,
    // simply clearing the relevant bits for timers and GPIOs in the map is sufficient
    // as they are dedicated to PWM in this driver.
    for (TRD_Channel_t i = 0; i < NUM_PWM_CHANNELS; ++i)
    {
        const PWM_Channel_Config_t *config = &pwm_channel_map[i];

        if (config->TIMx == TIM1 || config->TIMx == TIM9) // APB2 timers
        {
            RCC->APB2ENR &= ~config->TIM_Clk_Enable_Bit; /* PDF Reference (implicit via TIM sections) */
        }
        else // APB1 timers
        {
            RCC->APB1ENR &= ~config->TIM_Clk_Enable_Bit; /* PDF Reference (implicit via TIM sections) */
        }
         RCC->AHB1ENR &= ~config->GPIO_Clk_Enable_Bit; /* PDF Reference (implicit via GPIO section) */
    }
    // Need dummy reads to ensure clock disable takes effect? Usually not required for disabling.

    // Note: Reserved timers (TIM10, TIM11) clocks are not managed by this driver.
}