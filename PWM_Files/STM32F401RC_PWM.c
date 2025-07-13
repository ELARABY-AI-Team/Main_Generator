/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : Production-ready implementation for PWM on STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h" // Assumed to define TRD_Channel_t, timer/GPIO base addresses, and relevant register bit definitions

/*
 * Bare-metal register access definitions based on RM0368.
 * These are typically provided by CMSIS headers (like stm32f4xx.h).
 * Assuming basic register access via pointers and bit manipulation.
 */

/* GPIO Peripheral Base Addresses (Assumed to be defined in header) */
#ifndef GPIOA_BASE
#define GPIOA_BASE (0x40020000U) /* GPIOA base address */ // Assumed to be defined in header
#endif
#ifndef GPIOB_BASE
#define GPIOB_BASE (0x40020400U) /* GPIOB base address */ // Assumed to be defined in header
#endif
#ifndef GPIOC_BASE
#define GPIOC_BASE (0x40020800U) /* GPIOC base address */ // Assumed to be defined in header
#endif
/* ... other GPIO ports may be needed based on pwm_channel_map */

/* Timer Peripheral Base Addresses (Assumed to be defined in header) */
#ifndef TIM1_BASE
#define TIM1_BASE (0x40010000U) /* TIM1 base address */ // Assumed to be defined in header
#endif
#ifndef TIM3_BASE
#define TIM3_BASE (0x40000400U) /* TIM3 base address */ // Assumed to be defined in header
#endif
#ifndef TIM4_BASE
#define TIM4_BASE (0x40000800U) /* TIM4 base address */ // Assumed to be defined in header
#endif
#ifndef TIM9_BASE
#define TIM9_BASE (0x40014000U) /* TIM9 base address */ // Assumed to be defined in header
#endif
#ifndef TIM10_BASE
#define TIM10_BASE (0x40014400U) /* TIM10 base address */ // Assumed to be defined in header
#endif
#ifndef TIM11_BASE
#define TIM11_BASE (0x40014800U) /* TIM11 base address */ // Assumed to be defined in header
#endif

/* RCC Peripheral Base Address (Assumed to be defined in header) */
#ifndef RCC_BASE
#define RCC_BASE (0x40023800U) /* RCC base address */ // Assumed to be defined in header
#endif

/* Generic Peripheral Register Definitions (Offsets from Base Address) */
#define GPIO_MODER_OFFSET   0x00U /* GPIO port mode register */ // PDF Reference (RM0368 page 158)
#define GPIO_OTYPER_OFFSET  0x04U /* GPIO port output type register */ // PDF Reference (RM0368 page 158)
#define GPIO_OSPEEDR_OFFSET 0x08U /* GPIO port output speed register */ // PDF Reference (RM0368 page 159)
#define GPIO_PUPDR_OFFSET   0x0CU /* GPIO port pull-up/pull-down register */ // PDF Reference (RM0368 page 159)
#define GPIO_AFRL_OFFSET    0x20U /* GPIO alternate function low register */ // PDF Reference (RM0368 page 162)
#define GPIO_AFRH_OFFSET    0x24U /* GPIO alternate function high register */ // PDF Reference (RM0368 page 163)

#define TIM_CR1_OFFSET      0x00U /* TIM control register 1 */ // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR2_OFFSET      0x04U /* TIM control register 2 */ // PDF Reference (RM0368 page 289, 355, 398, 412)
#define TIM_SMCR_OFFSET     0x08U /* TIM slave mode control register */ // PDF Reference (RM0368 page 292, 356, 399, 412)
#define TIM_DIER_OFFSET     0x0CU /* TIM DMA/interrupt enable register */ // PDF Reference (RM0368 page 294, 358, 400, 413)
#define TIM_SR_OFFSET       0x10U /* TIM status register */ // PDF Reference (RM0368 page 296, 359, 402, 413)
#define TIM_EGR_OFFSET      0x14U /* TIM event generation register */ // PDF Reference (RM0368 page 297, 361, 403, 414)
#define TIM_CCMR1_OFFSET    0x18U /* TIM capture/compare mode register 1 */ // PDF Reference (RM0368 page 299, 362, 404, 415)
#define TIM_CCMR2_OFFSET    0x1CU /* TIM capture/compare mode register 2 */ // PDF Reference (RM0368 page 302, 365, 404, 415) - Note: TIM9/10/11 do not have CCMR2
#define TIM_CCER_OFFSET     0x20U /* TIM capture/compare enable register */ // PDF Reference (RM0368 page 303, 366, 407, 417)
#define TIM_CNT_OFFSET      0x24U /* TIM counter */ // PDF Reference (RM0368 page 307, 368, 408, 418)
#define TIM_PSC_OFFSET      0x28U /* TIM prescaler */ // PDF Reference (RM0368 page 307, 368, 408, 418)
#define TIM_ARR_OFFSET      0x2CU /* TIM auto-reload register */ // PDF Reference (RM0368 page 307, 368, 408, 418)
#define TIM_RCR_OFFSET      0x30U /* TIM repetition counter register (TIM1 only)*/ // PDF Reference (RM0368 page 308)
#define TIM_BDTR_OFFSET     0x44U /* TIM break and dead-time register (TIM1 only)*/ // PDF Reference (RM0368 page 310)
#define TIM_DCR_OFFSET      0x48U /* TIM DMA control register */ // PDF Reference (RM0368 page 312, 371)
#define TIM_DMAR_OFFSET     0x4CU /* TIM DMA address for full transfer */ // PDF Reference (RM0368 page 313, 372)
#define TIM_OR_OFFSET       0x50U /* TIM option register (TIM2,5,11) */ // PDF Reference (RM0368 page 373, 419)


/* RCC Register Offsets */
#define RCC_AHB1ENR_OFFSET  0x30U /* RCC AHB1 peripheral clock enable register */
#define RCC_APB1ENR_OFFSET  0x40U /* RCC APB1 peripheral clock enable register */
#define RCC_APB2ENR_OFFSET  0x44U /* RCC APB2 peripheral clock enable register */

/* Convenience macros for register access */
#define REG32(addr, offset) (*(volatile uint32_t *)((addr) + (offset)))
#define REG16(addr, offset) (*(volatile uint16_t *)((addr) + (offset)))

/*
 * Bit manipulation macros (assuming 32-bit registers unless specified)
 * These are basic and rely on the caller knowing the bit position/mask.
 */
#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)  ((REG) & (BIT))
#define CLEAR_REG(REG)      ((REG) = (0x0))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_BIT(REG, ~(CLEARMASK))) | (SETMASK))))

/*
 * Specific bit masks and positions based on RM0368 (selected relevant bits).
 * These would ideally come from device-specific CMSIS headers.
 */

/* GPIO_MODER */
#define GPIO_MODER_MODERy_Pos(y) ((y) * 2U) // PDF Reference (RM0368 page 158)
#define GPIO_MODER_MODERy_Msk(y) (0x3U << GPIO_MODER_MODERy_Pos(y))
#define GPIO_MODER_MODERy_0(y)   (0x1U << GPIO_MODER_MODERy_Pos(y))
#define GPIO_MODER_MODERy_1(y)   (0x2U << GPIO_MODER_MODERy_Pos(y))
#define GPIO_MODER_INPUT(y)      (0x0U << GPIO_MODER_MODERy_Pos(y))
#define GPIO_MODER_OUTPUT(y)     (0x1U << GPIO_MODER_MODERy_Pos(y))
#define GPIO_MODER_AF(y)         (0x2U << GPIO_MODER_MODERy_Pos(y)) // PDF Reference (RM0368 page 158)
#define GPIO_MODER_ANALOG(y)     (0x3U << GPIO_MODER_MODERy_Pos(y))

/* GPIO_OTYPER */
#define GPIO_OTYPER_OTy_Pos(y) ((y)) // PDF Reference (RM0368 page 158)
#define GPIO_OTYPER_OTy_Msk(y) (0x1U << GPIO_OTYPER_OTy_Pos(y))
#define GPIO_OTYPER_PP(y)        (0x0U << GPIO_OTYPER_OTy_Pos(y)) // PDF Reference (RM0368 page 158)
#define GPIO_OTYPER_OD(y)        (0x1U << GPIO_OTYPER_OTy_Pos(y))

/* GPIO_OSPEEDR */
#define GPIO_OSPEEDR_OSPEEDRy_Pos(y) ((y) * 2U) // PDF Reference (RM0368 page 159)
#define GPIO_OSPEEDR_OSPEEDRy_Msk(y) (0x3U << GPIO_OSPEEDR_OSPEEDRy_Pos(y))
#define GPIO_OSPEEDR_LOW(y)          (0x0U << GPIO_OSPEEDR_OSPEEDRy_Pos(y))
#define GPIO_OSPEEDR_MEDIUM(y)       (0x1U << GPIO_OSPEEDR_OSPEEDRy_Pos(y))
#define GPIO_OSPEEDR_HIGH(y)         (0x2U << GPIO_OSPEEDR_OSPEEDRy_Pos(y)) // PDF Reference (RM0368 page 159)
#define GPIO_OSPEEDR_VHIGH(y)        (0x3U << GPIO_OSPEEDR_OSPEEDRy_Pos(y))

/* GPIO_PUPDR */
#define GPIO_PUPDR_PUPDRy_Pos(y) ((y) * 2U) // PDF Reference (RM0368 page 159)
#define GPIO_PUPDR_PUPDRy_Msk(y) (0x3U << GPIO_PUPDR_PUPDRy_Pos(y))
#define GPIO_PUPDR_NO(y)         (0x0U << GPIO_PUPDR_PUPDRy_Pos(y)) // PDF Reference (RM0368 page 159)
#define GPIO_PUPDR_PU(y)         (0x1U << GPIO_PUPDR_PUPDRy_Pos(y))
#define GPIO_PUPDR_PD(y)         (0x2U << GPIO_PUPDR_PUPDRy_Pos(y))

/* GPIO_AFRL (Pins 0-7) */
#define GPIO_AFRL_AFRLy_Pos(y) ((y) * 4U) // PDF Reference (RM0368 page 162)
#define GPIO_AFRL_AFRLy_Msk(y) (0xFU << GPIO_AFRL_AFRLy_Pos(y))
#define GPIO_AFRL_AFy(y, af)   (((af) & 0xFU) << GPIO_AFRL_AFRLy_Pos(y))

/* GPIO_AFRH (Pins 8-15) */
#define GPIO_AFRH_AFRHy_Pos(y) (((y) - 8U) * 4U) // PDF Reference (RM0368 page 163)
#define GPIO_AFRH_AFRHy_Msk(y) (0xFU << GPIO_AFRH_AFRHy_Pos(y))
#define GPIO_AFRH_AFy(y, af)   (((af) & 0xFU) << GPIO_AFRH_AFRHy_Pos(y))

/* TIM_CR1 */
#define TIM_CR1_CEN_Pos  (0U) // PDF Reference (RM0368 page 289, 354, 398, 413)
#define TIM_CR1_CEN_Msk  (0x1U << TIM_CR1_CEN_Pos)
#define TIM_CR1_UDIS_Pos (1U) // PDF Reference (RM0368 page 289, 354, 398, 413)
#define TIM_CR1_UDIS_Msk (0x1U << TIM_CR1_UDIS_Pos)
#define TIM_CR1_URS_Pos  (2U) // PDF Reference (RM0368 page 289, 354, 398, 413)
#define TIM_CR1_URS_Msk  (0x1U << TIM_CR1_URS_Pos)
#define TIM_CR1_OPM_Pos  (3U) // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR1_OPM_Msk  (0x1U << TIM_CR1_OPM_Pos)
#define TIM_CR1_DIR_Pos  (4U) // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR1_DIR_Msk  (0x1U << TIM_CR1_DIR_Pos)
#define TIM_CR1_CMS_Pos  (5U) // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR1_CMS_Msk  (0x3U << TIM_CR1_CMS_Pos)
#define TIM_CR1_CMS_EDGE_Aligned (0x0U << TIM_CR1_CMS_Pos) // PDF Reference (RM0368 page 288, 353)
#define TIM_CR1_ARPE_Pos (7U) // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR1_ARPE_Msk (0x1U << TIM_CR1_ARPE_Pos)
#define TIM_CR1_CKD_Pos  (8U) // PDF Reference (RM0368 page 288, 353, 398, 412)
#define TIM_CR1_CKD_Msk  (0x3U << TIM_CR1_CKD_Pos)

/* TIM_CR2 */
#define TIM_CR2_CCPC_Pos (0U) // PDF Reference (RM0368 page 291) - TIM1 only? RM says "on channels having a complementary output" which is TIM1
#define TIM_CR2_CCPC_Msk (0x1U << TIM_CR2_CCPC_Pos) // PDF Reference (RM0368 page 291)
#define TIM_CR2_CCUS_Pos (2U) // PDF Reference (RM0368 page 291) - TIM1 only? RM says "on channels having a complementary output" which is TIM1
#define TIM_CR2_CCUS_Msk (0x1U << TIM_CR2_CCUS_Pos) // PDF Reference (RM0368 page 291)
#define TIM_CR2_CCDS_Pos (3U) // PDF Reference (RM0368 page 290, 355) - Note: TIM9/10/11 reserved
#define TIM_CR2_CCDS_Msk (0x1U << TIM_CR2_CCDS_Pos) // PDF Reference (RM0368 page 290, 355)
#define TIM_CR2_MMS_Pos  (4U) // PDF Reference (RM0368 page 290, 355)
#define TIM_CR2_MMS_Msk  (0x7U << TIM_CR2_MMS_Pos) // PDF Reference (RM0368 page 290, 355)
#define TIM_CR2_MMS_UPDATE (0x2U << TIM_CR2_MMS_Pos) // PDF Reference (RM0368 page 290, 355)
#define TIM_CR2_TI1S_Pos (7U) // PDF Reference (RM0368 page 290, 355) - Note: TIM10/11 reserved
#define TIM_CR2_TI1S_Msk (0x1U << TIM_CR2_TI1S_Pos) // PDF Reference (RM0368 page 290, 355)

/* TIM_CCMR1 - Output Compare Mode */
#define TIM_CCMR1_OC1S_Pos (0U) // PDF Reference (RM0368 page 299, 363, 405, 416) - Input Capture Mode
#define TIM_CCMR1_CC1S_Pos (0U) // PDF Reference (RM0368 page 301, 364, 406, 416) - Output Compare Mode alias
#define TIM_CCMR1_CC1S_Msk (0x3U << TIM_CCMR1_CC1S_Pos) // PDF Reference (RM0368 page 301, 364, 406, 416)
#define TIM_CCMR1_CC1S_OUTPUT (0x0U << TIM_CCMR1_CC1S_Pos) // PDF Reference (RM0368 page 301, 364, 406, 416)
#define TIM_CCMR1_OC1FE_Pos (2U) // PDF Reference (RM0368 page 300, 363, 405)
#define TIM_CCMR1_OC1FE_Msk (0x1U << TIM_CCMR1_OC1FE_Pos) // PDF Reference (RM0368 page 300, 363, 405)
#define TIM_CCMR1_OC1PE_Pos (3U) // PDF Reference (RM0368 page 300, 363, 405)
#define TIM_CCMR1_OC1PE_Msk (0x1U << TIM_CCMR1_OC1PE_Pos) // PDF Reference (RM0368 page 300, 363, 405)
#define TIM_CCMR1_OC1M_Pos (4U) // PDF Reference (RM0368 page 300, 363, 405, 416)
#define TIM_CCMR1_OC1M_Msk (0x7U << TIM_CCMR1_OC1M_Pos) // PDF Reference (RM0368 page 300, 363, 405, 416)
#define TIM_CCMR1_OC1M_PWM1 (0x6U << TIM_CCMR1_OC1M_Pos) // PDF Reference (RM0368 page 300, 363, 405, 416)
#define TIM_CCMR1_OC1M_PWM2 (0x7U << TIM_CCMR1_OC1M_Pos) // PDF Reference (RM0368 page 300, 363, 405, 416)
#define TIM_CCMR1_OC1CE_Pos (7U) // PDF Reference (RM0368 page 299, 362) - TIM1-5 only
#define TIM_CCMR1_OC1CE_Msk (0x1U << TIM_CCMR1_OC1CE_Pos) // PDF Reference (RM0368 page 299, 362)

#define TIM_CCMR1_CC2S_Pos (8U) // PDF Reference (RM0368 page 299, 363, 404)
#define TIM_CCMR1_CC2S_Msk (0x3U << TIM_CCMR1_CC2S_Pos) // PDF Reference (RM0368 page 299, 363, 404)
#define TIM_CCMR1_CC2S_OUTPUT (0x0U << TIM_CCMR1_CC2S_Pos) // PDF Reference (RM0368 page 299, 363, 404)
#define TIM_CCMR1_OC2FE_Pos (10U) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2FE_Msk (0x1U << TIM_CCMR1_OC2FE_Pos) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2PE_Pos (11U) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2PE_Msk (0x1U << TIM_CCMR1_OC2PE_Pos) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2M_Pos (12U) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2M_Msk (0x7U << TIM_CCMR1_OC2M_Pos) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2M_PWM1 (0x6U << TIM_CCMR1_OC2M_Pos) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2M_PWM2 (0x7U << TIM_CCMR1_OC2M_Pos) // PDF Reference (RM0368 page 299, 362, 404)
#define TIM_CCMR1_OC2CE_Pos (15U) // PDF Reference (RM0368 page 299, 362) - TIM1-5 only
#define TIM_CCMR1_OC2CE_Msk (0x1U << TIM_CCMR1_OC2CE_Pos) // PDF Reference (RM0368 page 299, 362)

/* TIM_CCMR2 - Output Compare Mode */
#define TIM_CCMR2_CC3S_Pos (0U) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_CC3S_Msk (0x3U << TIM_CCMR2_CC3S_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_CC3S_OUTPUT (0x0U << TIM_CCMR2_CC3S_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3FE_Pos (2U) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3FE_Msk (0x1U << TIM_CCMR2_OC3FE_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3PE_Pos (3U) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3PE_Msk (0x1U << TIM_CCMR2_OC3PE_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3M_Pos (4U) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3M_Msk (0x7U << TIM_CCMR2_OC3M_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3M_PWM1 (0x6U << TIM_CCMR2_OC3M_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3M_PWM2 (0x7U << TIM_CCMR2_OC3M_Pos) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3CE_Pos (7U) // PDF Reference (RM0368 page 302, 365)
#define TIM_CCMR2_OC3CE_Msk (0x1U << TIM_CCMR2_OC3CE_Pos) // PDF Reference (RM0368 page 302, 365)

#define TIM_CCMR2_CC4S_Pos (8U) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_CC4S_Msk (0x3U << TIM_CCMR2_CC4S_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_CC4S_OUTPUT (0x0U << TIM_CCMR2_CC4S_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4FE_Pos (10U) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4FE_Msk (0x1U << TIM_CCMR2_OC4FE_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4PE_Pos (11U) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4PE_Msk (0x1U << TIM_CCMR2_OC4PE_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4M_Pos (12U) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4M_Msk (0x7U << TIM_CCMR2_OC4M_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4M_PWM1 (0x6U << TIM_CCMR2_OC4M_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4M_PWM2 (0x7U << TIM_CCMR2_OC4M_Pos) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4CE_Pos (15U) // PDF Reference (RM0368 page 302, 366)
#define TIM_CCMR2_OC4CE_Msk (0x1U << TIM_CCMR2_OC4CE_Pos) // PDF Reference (RM0368 page 302, 366)

/* TIM_CCER */
#define TIM_CCER_CC1E_Pos   (0U) // PDF Reference (RM0368 page 305, 367, 407, 418)
#define TIM_CCER_CC1E_Msk   (0x1U << TIM_CCER_CC1E_Pos) // PDF Reference (RM0368 page 305, 367, 407, 418)
#define TIM_CCER_CC1P_Pos   (1U) // PDF Reference (RM0368 page 305, 367, 407, 418)
#define TIM_CCER_CC1P_Msk   (0x1U << TIM_CCER_CC1P_Pos) // PDF Reference (RM0368 page 305, 367, 407, 418)
#define TIM_CCER_CC1NE_Pos  (2U) // PDF Reference (RM0368 page 304) - TIM1 only
#define TIM_CCER_CC1NE_Msk  (0x1U << TIM_CCER_CC1NE_Pos) // PDF Reference (RM0368 page 304)
#define TIM_CCER_CC1NP_Pos  (3U) // PDF Reference (RM0368 page 304, 367, 407, 418)
#define TIM_CCER_CC1NP_Msk  (0x1U << TIM_CCER_CC1NP_Pos) // PDF Reference (RM0368 page 304, 367, 407, 418)

#define TIM_CCER_CC2E_Pos   (4U) // PDF Reference (RM0368 page 304, 367, 407)
#define TIM_CCER_CC2E_Msk   (0x1U << TIM_CCER_CC2E_Pos) // PDF Reference (RM0368 page 304, 367, 407)
#define TIM_CCER_CC2P_Pos   (5U) // PDF Reference (RM0368 page 304, 367, 407)
#define TIM_CCER_CC2P_Msk   (0x1U << TIM_CCER_CC2P_Pos) // PDF Reference (RM0368 page 304, 367, 407)
#define TIM_CCER_CC2NE_Pos  (6U) // PDF Reference (RM0368 page 304) - TIM1 only
#define TIM_CCER_CC2NE_Msk  (0x1U << TIM_CCER_CC2NE_Pos) // PDF Reference (RM0368 page 304)
#define TIM_CCER_CC2NP_Pos  (7U) // PDF Reference (RM0368 page 304, 367, 407)
#define TIM_CCER_CC2NP_Msk  (0x1U << TIM_CCER_CC2NP_Pos) // PDF Reference (RM0368 page 304, 367, 407)

#define TIM_CCER_CC3E_Pos   (8U) // PDF Reference (RM0368 page 303, 367) - TIM1-5 only
#define TIM_CCER_CC3E_Msk   (0x1U << TIM_CCER_CC3E_Pos) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC3P_Pos   (9U) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC3P_Msk   (0x1U << TIM_CCER_CC3P_Pos) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC3NE_Pos  (10U) // PDF Reference (RM0368 page 303) - TIM1 only
#define TIM_CCER_CC3NE_Msk  (0x1U << TIM_CCER_CC3NE_Pos) // PDF Reference (RM0368 page 303)
#define TIM_CCER_CC3NP_Pos  (11U) // PDF Reference (RM0368 page 303, 366) - TIM1-5 only
#define TIM_CCER_CC3NP_Msk  (0x1U << TIM_CCER_CC3NP_Pos) // PDF Reference (RM0368 page 303, 366)

#define TIM_CCER_CC4E_Pos   (12U) // PDF Reference (RM0368 page 303, 367) - TIM1-5 only
#define TIM_CCER_CC4E_Msk   (0x1U << TIM_CCER_CC4E_Pos) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC4P_Pos   (13U) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC4P_Msk   (0x1U << TIM_CCER_CC4P_Pos) // PDF Reference (RM0368 page 303, 367)
#define TIM_CCER_CC4NP_Pos  (15U) // PDF Reference (RM0368 page 366) - TIM2-5 only? RM says "CC4NP not implemented in TIM1"
#define TIM_CCER_CC4NP_Msk  (0x1U << TIM_CCER_CC4NP_Pos) // PDF Reference (RM0368 page 366)

/* TIM_EGR */
#define TIM_EGR_UG_Pos    (0U) // PDF Reference (RM0368 page 298, 361, 403, 414)
#define TIM_EGR_UG_Msk    (0x1U << TIM_EGR_UG_Pos) // PDF Reference (RM0368 page 298, 361, 403, 414)
#define TIM_EGR_CC1G_Pos  (1U) // PDF Reference (RM0368 page 298, 361, 403, 414)
#define TIM_EGR_CC1G_Msk  (0x1U << TIM_EGR_CC1G_Pos) // PDF Reference (RM0368 page 298, 361, 403, 414)
#define TIM_EGR_CC2G_Pos  (2U) // PDF Reference (RM0368 page 298, 361, 403)
#define TIM_EGR_CC2G_Msk  (0x1U << TIM_EGR_CC2G_Pos) // PDF Reference (RM0368 page 298, 361, 403)
#define TIM_EGR_CC3G_Pos  (3U) // PDF Reference (RM0368 page 298, 361)
#define TIM_EGR_CC3G_Msk  (0x1U << TIM_EGR_CC3G_Pos) // PDF Reference (RM0368 page 298, 361)
#define TIM_EGR_CC4G_Pos  (4U) // PDF Reference (RM0368 page 298, 361)
#define TIM_EGR_CC4G_Msk  (0x1U << TIM_EGR_CC4G_Pos) // PDF Reference (RM0368 page 298, 361)
#define TIM_EGR_COMG_Pos  (5U) // PDF Reference (RM0368 page 298) - TIM1 only? RM says "channels having a complementary output"
#define TIM_EGR_COMG_Msk  (0x1U << TIM_EGR_COMG_Pos) // PDF Reference (RM0368 page 298)
#define TIM_EGR_TG_Pos    (6U) // PDF Reference (RM0368 page 297, 361, 403)
#define TIM_EGR_TG_Msk    (0x1U << TIM_EGR_TG_Pos) // PDF Reference (RM0368 page 297, 361, 403)
#define TIM_EGR_BG_Pos    (7U) // PDF Reference (RM0368 page 297) - TIM1 only?
#define TIM_EGR_BG_Msk    (0x1U << TIM_EGR_BG_Pos) // PDF Reference (RM0368 page 297)

/* TIM_BDTR (TIM1 only) */
#define TIM_BDTR_DTG_Pos  (0U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_DTG_Msk  (0xFFU << TIM_BDTR_DTG_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_LOCK_Pos (8U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_LOCK_Msk (0x3U << TIM_BDTR_LOCK_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_OSSI_Pos (10U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_OSSI_Msk (0x1U << TIM_BDTR_OSSI_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_OSSR_Pos (11U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_OSSR_Msk (0x1U << TIM_BDTR_OSSR_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_BKE_Pos  (12U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_BKE_Msk  (0x1U << TIM_BDTR_BKE_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_BKP_Pos  (13U) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_BKP_Msk  (0x1U << TIM_BDTR_BKP_Pos) // PDF Reference (RM0368 page 311)
#define TIM_BDTR_AOE_Pos  (14U) // PDF Reference (RM0368 page 310)
#define TIM_BDTR_AOE_Msk  (0x1U << TIM_BDTR_AOE_Pos) // PDF Reference (RM0368 page 310)
#define TIM_BDTR_MOE_Pos  (15U) // PDF Reference (RM0368 page 310)
#define TIM_BDTR_MOE_Msk  (0x1U << TIM_BDTR_MOE_Pos) // PDF Reference (RM0368 page 310)

/* RCC AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN_Pos (0U)
#define RCC_AHB1ENR_GPIOAEN_Msk (0x1U << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOBEN_Pos (1U)
#define RCC_AHB1ENR_GPIOBEN_Msk (0x1U << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOCEN_Pos (2U)
#define RCC_AHB1ENR_GPIOCEN_Msk (0x1U << RCC_AHB1ENR_GPIOCEN_Pos)
/* ... other GPIO enable bits */

/* RCC APB1ENR */
#define RCC_APB1ENR_TIM3EN_Pos (1U)
#define RCC_APB1ENR_TIM3EN_Msk (0x1U << RCC_APB1ENR_TIM3EN_Pos)
#define RCC_APB1ENR_TIM4EN_Pos (2U)
#define RCC_APB1ENR_TIM4EN_Msk (0x1U << RCC_APB1ENR_TIM4EN_Pos)
/* ... other APB1 timer enable bits */

/* RCC APB2ENR */
#define RCC_APB2ENR_TIM1EN_Pos (0U)
#define RCC_APB2ENR_TIM1EN_Msk (0x1U << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_TIM9EN_Pos (16U)
#define RCC_APB2ENR_TIM9EN_Msk (0x1U << RCC_APB2ENR_TIM9EN_Pos)
#define RCC_APB2ENR_TIM10EN_Pos (17U)
#define RCC_APB2ENR_TIM10EN_Msk (0x1U << RCC_APB2ENR_TIM10EN_Pos)
#define RCC_APB2ENR_TIM11EN_Pos (18U)
#define RCC_APB2ENR_TIM11EN_Msk (0x1U << RCC_APB2ENR_TIM11EN_Pos)
/* ... other APB2 timer enable bits */

/*
 * Timer clock frequency assumption.
 * The actual timer clock depends on the APBx prescalers.
 * For STM32F401RC, if APB1/APB2 prescalers are > 1, the timer clock is 2*APB clock.
 * Assuming HCLK = 84MHz, APB1 prescaler = 2 (APB1=42MHz), APB2 prescaler = 1 (APB2=84MHz).
 * Then TIM3, TIM4 clock = 2 * 42MHz = 84MHz.
 * TIM1, TIM9, TIM10, TIM11 clock = 1 * 84MHz = 84MHz.
 */
#define TIMER_CLOCK_FREQ    84000000UL /* Assumed Timer Clock Frequency (Hz) */ /* Assumed PWM config - please verify */

/* Max ARR value for 16-bit timers */
#define MAX_ARR_16BIT       65535U /* PDF Reference (RM0368 page 243, 316, 376) */

/*
 * Timer Reservation:
 * TIM2 and TIM5 are reserved for OS/Delay purposes as per requirements.
 * These timers are excluded from the PWM channel map.
 */

/*
 * PWM Channel Mapping:
 * Maps TRD_Channel_t to specific Timer, Channel, GPIO Port, Pin, and AF.
 * This mapping is based on common STM32F401RC pin assignments and must be verified
 * with the device datasheet and user hardware design.
 * Pins with number 0 are avoided unless commonly used for a specific channel (e.g., PB0).
 */
typedef struct {
    uint32_t TIM_Base;
    uint8_t TIM_Channel; /* 1-4 */
    uint32_t GPIO_Port_Base;
    uint8_t GPIO_Pin; /* 0-15 */
    uint8_t GPIO_AF; /* Alternate Function number */
    uint32_t RCC_APB_ENR_Mask; /* Mask for timer enable bit in RCC APB1ENR or APB2ENR */
    uint32_t RCC_AHB1ENR_Mask; /* Mask for GPIO port enable bit in RCC AHB1ENR */
} PWM_Config_t;

static const PWM_Config_t pwm_channel_map[] = {
    /* Example mappings (verify with datasheet and hardware) */
    /* TIM1 Channels (APB2, High-performance) */
    {TIM1_BASE, 1, GPIOA_BASE, 8, 1, RCC_APB2ENR_TIM1EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk}, /* TIM1_CH1 on PA8 (AF1) */ /* Assumed PWM config - please verify */
    {TIM1_BASE, 2, GPIOA_BASE, 9, 1, RCC_APB2ENR_TIM1EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk}, /* TIM1_CH2 on PA9 (AF1) */ /* Assumed PWM config - please verify */
    {TIM1_BASE, 3, GPIOA_BASE, 10, 1, RCC_APB2ENR_TIM1EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk},/* TIM1_CH3 on PA10 (AF1) */ /* Assumed PWM config - please verify */
    {TIM1_BASE, 4, GPIOA_BASE, 11, 1, RCC_APB2ENR_TIM1EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk},/* TIM1_CH4 on PA11 (AF1) */ /* Assumed PWM config - please verify */

    /* TIM3 Channels (APB1, General-purpose) */
    // Using PC6-PC9 to avoid conflicts with TIM4/TIM10/TIM11 on common pins like PA6/PA7/PB0/PB1
    {TIM3_BASE, 1, GPIOC_BASE, 6, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOCEN_Msk}, /* TIM3_CH1 on PC6 (AF2) */ /* Assumed PWM config - please verify */
    {TIM3_BASE, 2, GPIOC_BASE, 7, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOCEN_Msk}, /* TIM3_CH2 on PC7 (AF2) */ /* Assumed PWM config - please verify */
    {TIM3_BASE, 3, GPIOC_BASE, 8, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOCEN_Msk}, /* TIM3_CH3 on PC8 (AF2) */ /* Assumed PWM config - please verify */
    {TIM3_BASE, 4, GPIOC_BASE, 9, 2, RCC_APB1ENR_TIM3EN_Msk, RCC_AHB1ENR_GPIOCEN_Msk}, /* TIM3_CH4 on PC9 (AF2) */ /* Assumed PWM config - please verify */

    /* TIM4 Channels (APB1, General-purpose) */
    {TIM4_BASE, 1, GPIOB_BASE, 6, 2, RCC_APB1ENR_TIM4EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk}, /* TIM4_CH1 on PB6 (AF2) */ /* Assumed PWM config - please verify */
    {TIM4_BASE, 2, GPIOB_BASE, 7, 2, RCC_APB1ENR_TIM4EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk}, /* TIM4_CH2 on PB7 (AF2) */ /* Assumed PWM config - please verify */
    {TIM4_BASE, 3, GPIOB_BASE, 8, 2, RCC_APB1ENR_TIM4EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk}, /* TIM4_CH3 on PB8 (AF2) */ /* Assumed PWM config - please verify */
    {TIM4_BASE, 4, GPIOB_BASE, 9, 2, RCC_APB1ENR_TIM4EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk}, /* TIM4_CH4 on PB9 (AF2) */ /* Assumed PWM config - please verify */

    /* TIM9 Channels (APB2, General-purpose) */
    {TIM9_BASE, 1, GPIOA_BASE, 2, 3, RCC_APB2ENR_TIM9EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk}, /* TIM9_CH1 on PA2 (AF3) */ /* Assumed PWM config - please verify */
    {TIM9_BASE, 2, GPIOA_BASE, 3, 3, RCC_APB2ENR_TIM9EN_Msk, RCC_AHB1ENR_GPIOAEN_Msk}, /* TIM9_CH2 on PA3 (AF3) */ /* Assumed PWM config - please verify */

    /* TIM10 Channel (APB2, General-purpose, 1 Channel) */
    {TIM10_BASE, 1, GPIOB_BASE, 8, 3, RCC_APB2ENR_TIM10EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk},/* TIM10_CH1 on PB8 (AF3) - Conflict with TIM4_CH3 (AF2) */ /* Assumed PWM config - please verify */

    /* TIM11 Channel (APB2, General-purpose, 1 Channel) */
    {TIM11_BASE, 1, GPIOB_BASE, 9, 3, RCC_APB2ENR_TIM11EN_Msk, RCC_AHB1ENR_GPIOBEN_Msk},/* TIM11_CH1 on PB9 (AF3) - Conflict with TIM4_CH4 (AF2) */ /* Assumed PWM config - please verify */
};

#define PWM_CHANNEL_COUNT (sizeof(pwm_channel_map) / sizeof(PWM_Config_t))

/**Functions ===========================================================================*/

/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel: The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= PWM_CHANNEL_COUNT) {
        /* Invalid channel */
        return;
    }

    const PWM_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable GPIO clock for the port
    // Assuming RCC_AHB1ENR_OFFSET exists for the specific GPIO port
    uint32_t* rcc_ahb1enr = (uint32_t*)(RCC_BASE + RCC_AHB1ENR_OFFSET);
    SET_BIT(*rcc_ahb1enr, config->RCC_AHB1ENR_Mask); /* PDF Reference (RM0368 GPIO intro) */

    // Enable Timer clock
    // Assuming RCC_APB1ENR_OFFSET/RCC_APB2ENR_OFFSET exists for the specific timer
    uint32_t* rcc_apbenr;
    if (config->TIM_Base == TIM1_BASE || config->TIM_Base == TIM9_BASE || config->TIM_Base == TIM10_BASE || config->TIM_Base == TIM11_BASE) {
        rcc_apbenr = (uint32_t*)(RCC_BASE + RCC_APB2ENR_OFFSET);
    } else { // TIM3, TIM4 (TIM2, TIM5 reserved)
        rcc_apbenr = (uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
    }
    SET_BIT(*rcc_apbenr, config->RCC_APB_ENR_Mask); /* PDF Reference (RM0368 page 243, 316, 376, and RCC section - not fully provided) */


    // Configure GPIO pin for Alternate Function (AF) mode
    uint32_t* gpio_moder = (uint32_t*)(config->GPIO_Port_Base + GPIO_MODER_OFFSET);
    MODIFY_REG(*gpio_moder,
               GPIO_MODER_MODERy_Msk(config->GPIO_Pin),
               GPIO_MODER_AF(config->GPIO_Pin)); /* PDF Reference (RM0368 page 158, 149) */

    // Configure GPIO pin Output Type to Push-Pull (PP)
    uint32_t* gpio_otyper = (uint32_t*)(config->GPIO_Port_Base + GPIO_OTYPER_OFFSET);
    MODIFY_REG(*gpio_otyper,
               GPIO_OTYPER_OTy_Msk(config->GPIO_Pin),
               GPIO_OTYPER_PP(config->GPIO_Pin)); /* PDF Reference (RM0368 page 158) */

    // Configure GPIO pin Output Speed (High Speed recommended for PWM)
    uint32_t* gpio_ospeedr = (uint32_t*)(config->GPIO_Port_Base + GPIO_OSPEEDR_OFFSET);
     MODIFY_REG(*gpio_ospeedr,
               GPIO_OSPEEDR_OSPEEDRy_Msk(config->GPIO_Pin),
               GPIO_OSPEEDR_HIGH(config->GPIO_Pin)); /* PDF Reference (RM0368 page 159) */

    // Configure GPIO pin Pull-up/Pull-down to No Pull-up/down
    uint32_t* gpio_pupdr = (uint32_t*)(config->GPIO_Port_Base + GPIO_PUPDR_OFFSET);
    MODIFY_REG(*gpio_pupdr,
               GPIO_PUPDR_PUPDRy_Msk(config->GPIO_Pin),
               GPIO_PUPDR_NO(config->GPIO_Pin)); /* PDF Reference (RM0368 page 159) */

    // Select the Alternate Function (AF) for the GPIO pin
    if (config->GPIO_Pin < 8) {
        uint32_t* gpio_afrl = (uint32_t*)(config->GPIO_Port_Base + GPIO_AFRL_OFFSET);
        MODIFY_REG(*gpio_afrl,
                   GPIO_AFRL_AFRLy_Msk(config->GPIO_Pin),
                   GPIO_AFRL_AFy(config->GPIO_Pin, config->GPIO_AF)); /* PDF Reference (RM0368 page 162, 149) */
    } else {
        uint32_t* gpio_afrh = (uint32_t*)(config->GPIO_Port_Base + GPIO_AFRH_OFFSET);
        MODIFY_REG(*gpio_afrh,
                   GPIO_AFRH_AFRHy_Msk(config->GPIO_Pin),
                   GPIO_AFRH_AFy(config->GPIO_Pin, config->GPIO_AF)); /* PDF Reference (RM0368 page 163, 149) */
    }

    // --- Configure Timer ---
    // Disable the counter before configuration
    uint32_t* tim_cr1 = (uint32_t*)(config->TIM_Base + TIM_CR1_OFFSET);
    CLEAR_BIT(*tim_cr1, TIM_CR1_CEN_Msk); /* PDF Reference (RM0368 page 289, 354, 398, 413) */

    // Select Upcounting mode (CMS=00, DIR=0)
    MODIFY_REG(*tim_cr1, TIM_CR1_CMS_Msk | TIM_CR1_DIR_Msk, TIM_CR1_CMS_EDGE_Aligned); /* PDF Reference (RM0368 page 288, 353) */

    // Enable Auto-Reload Preload
    SET_BIT(*tim_cr1, TIM_CR1_ARPE_Msk); /* PDF Reference (RM0368 page 288, 353, 398, 412) */

    // Set Clock Division (CKD) to 00 (tDTS = tCK_INT)
     MODIFY_REG(*tim_cr1, TIM_CR1_CKD_Msk, 0x0U << TIM_CR1_CKD_Pos); /* PDF Reference (RM0368 page 288, 353, 398, 412) */

    // Configure Capture/Compare Mode Register (CCMR)
    // Select PWM Mode 1 (OCxM=110), enable output compare preload (OCxPE)
    // Ensure channel is configured as output (CCxS=00)
    uint32_t* tim_ccmr;
    uint32_t ccmr_mask;
    uint32_t ccmr_pwm_mode;
    uint32_t ccmr_ocpe_bit;
    uint32_t ccmr_ccse_mask;
    uint32_t ccmr_ccse_output;

    if (config->TIM_Channel == 1) {
        tim_ccmr = (uint32_t*)(config->TIM_Base + TIM_CCMR1_OFFSET);
        ccmr_mask = TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE_Msk | TIM_CCMR1_CC1S_Msk; // PDF Reference (RM0368 page 300)
        ccmr_pwm_mode = TIM_CCMR1_OC1M_PWM1; // PDF Reference (RM0368 page 300)
        ccmr_ocpe_bit = TIM_CCMR1_OC1PE_Msk; // PDF Reference (RM0368 page 300)
        ccmr_ccse_mask = TIM_CCMR1_CC1S_Msk; // PDF Reference (RM0368 page 301)
        ccmr_ccse_output = TIM_CCMR1_CC1S_OUTPUT; // PDF Reference (RM0368 page 301)
    } else if (config->TIM_Channel == 2) {
        tim_ccmr = (uint32_t*)(config->TIM_Base + TIM_CCMR1_OFFSET);
        ccmr_mask = TIM_CCMR1_OC2M_Msk | TIM_CCMR1_OC2PE_Msk | TIM_CCMR1_CC2S_Msk; // PDF Reference (RM0368 page 299)
        ccmr_pwm_mode = TIM_CCMR1_OC2M_PWM1; // PDF Reference (RM0368 page 299)
        ccmr_ocpe_bit = TIM_CCMR1_OC2PE_Msk; // PDF Reference (RM0368 page 299)
        ccmr_ccse_mask = TIM_CCMR1_CC2S_Msk; // PDF Reference (RM0368 page 299)
        ccmr_ccse_output = TIM_CCMR1_CC2S_OUTPUT; // PDF Reference (RM0368 page 299)
    } else if (config->TIM_Channel == 3) {
        tim_ccmr = (uint32_t*)(config->TIM_Base + TIM_CCMR2_OFFSET); // PDF Reference (RM0368 page 302)
        ccmr_mask = TIM_CCMR2_OC3M_Msk | TIM_CCMR2_OC3PE_Msk | TIM_CCMR2_CC3S_Msk; // PDF Reference (RM0368 page 302)
        ccmr_pwm_mode = TIM_CCMR2_OC3M_PWM1; // PDF Reference (RM0368 page 302)
        ccmr_ocpe_bit = TIM_CCMR2_OC3PE_Msk; // PDF Reference (RM0368 page 302)
        ccmr_ccse_mask = TIM_CCMR2_CC3S_Msk; // PDF Reference (RM0368 page 302)
        ccmr_ccse_output = TIM_CCMR2_CC3S_OUTPUT; // PDF Reference (RM0368 page 302)
    } else if (config->TIM_Channel == 4) {
        tim_ccmr = (uint32_t*)(config->TIM_Base + TIM_CCMR2_OFFSET); // PDF Reference (RM0368 page 302)
        ccmr_mask = TIM_CCMR2_OC4M_Msk | TIM_CCMR2_OC4PE_Msk | TIM_CCMR2_CC4S_Msk; // PDF Reference (RM0368 page 302)
        ccmr_pwm_mode = TIM_CCMR2_OC4M_PWM1; // PDF Reference (RM0368 page 302)
        ccmr_ocpe_bit = TIM_CCMR2_OC4PE_Msk; // PDF Reference (RM0368 page 302)
        ccmr_ccse_mask = TIM_CCMR2_CC4S_Msk; // PDF Reference (RM0368 page 302)
        ccmr_ccse_output = TIM_CCMR2_CC4S_OUTPUT; // PDF Reference (RM0368 page 302)
    } else {
         /* Invalid channel number in map */
         return;
    }

    // Note: CCxS bits are writable only when the channel is OFF (CCxE = 0 in TIMx_CCER).
    // We ensure CCxE is off before writing CCMR by clearing the whole CCER later.
    MODIFY_REG(*tim_ccmr,
               ccmr_mask,
               ccmr_pwm_mode | ccmr_ocpe_bit | ccmr_ccse_output); /* PDF Reference (RM0368 page 299, 300, 302, 362, 363, 365, 366, 404, 405) */

    // Configure Capture/Compare Enable Register (CCER)
    // Clear all channel bits first, then configure the specific channel
    uint32_t* tim_ccer = (uint32_t*)(config->TIM_Base + TIM_CCER_OFFSET);
    CLEAR_REG(*tim_ccer); // Ensure all channels are off before configuring mode/polarity

    // Configure Output Polarity (Active High = 0) and Enable Output
    uint32_t ccer_polarity_bit;
    uint32_t ccer_enable_bit;
    uint32_t ccer_complementary_enable_bit = 0; // Not used for standard PWM
    uint32_t ccer_complementary_polarity_bit = 0; // Not used for standard PWM


    if (config->TIM_Channel == 1) {
        ccer_polarity_bit = TIM_CCER_CC1P_Msk; // PDF Reference (RM0368 page 305, 367, 407, 418)
        ccer_enable_bit = TIM_CCER_CC1E_Msk; // PDF Reference (RM0368 page 305, 367, 407, 418)
        if (config->TIM_Base == TIM1_BASE) { // TIM1 has complementary outputs
             ccer_complementary_enable_bit = TIM_CCER_CC1NE_Msk; // PDF Reference (RM0368 page 304)
             ccer_complementary_polarity_bit = TIM_CCER_CC1NP_Msk; // PDF Reference (RM0368 page 304)
        }
    } else if (config->TIM_Channel == 2) {
        ccer_polarity_bit = TIM_CCER_CC2P_Msk; // PDF Reference (RM0368 page 304, 367, 407)
        ccer_enable_bit = TIM_CCER_CC2E_Msk; // PDF Reference (RM0368 page 304, 367, 407)
         if (config->TIM_Base == TIM1_BASE) { // TIM1 has complementary outputs
             ccer_complementary_enable_bit = TIM_CCER_CC2NE_Msk; // PDF Reference (RM0368 page 304)
             ccer_complementary_polarity_bit = TIM_CCER_CC2NP_Msk; // PDF Reference (RM0368 page 304)
        }
    } else if (config->TIM_Channel == 3) { // TIM1-5 only, TIM9 has 2 channels, TIM10/11 has 1
        ccer_polarity_bit = TIM_CCER_CC3P_Msk; // PDF Reference (RM0368 page 303, 367)
        ccer_enable_bit = TIM_CCER_CC3E_Msk; // PDF Reference (RM0368 page 303, 367)
         if (config->TIM_Base == TIM1_BASE) { // TIM1 has complementary outputs
             ccer_complementary_enable_bit = TIM_CCER_CC3NE_Msk; // PDF Reference (RM0368 page 303)
             ccer_complementary_polarity_bit = TIM_CCER_CC3NP_Msk; // PDF Reference (RM0368 page 303)
        }
    } else if (config->TIM_Channel == 4) { // TIM1-5 only
        ccer_polarity_bit = TIM_CCER_CC4P_Msk; // PDF Reference (RM0368 page 303, 367)
        ccer_enable_bit = TIM_CCER_CC4E_Msk; // PDF Reference (RM0368 page 303, 367)
        // Note: TIM1 has no CC4NP bit, TIM2-5 have CC4NP but we are not using complementary here.
    } else {
         /* Invalid channel number in map */
         return;
    }

    // Set Output Polarity (0: Active High)
    CLEAR_BIT(*tim_ccer, ccer_polarity_bit); /* PDF Reference (RM0368 page 305, 367, 407, 418) */
    // Keep complementary output disabled and active low (default)
    if (config->TIM_Base == TIM1_BASE) {
        CLEAR_BIT(*tim_ccer, ccer_complementary_enable_bit);
        CLEAR_BIT(*tim_ccer, ccer_complementary_polarity_bit);
    }

    // Initial duty cycle and frequency (can be set to a default like 0 or 50%)
    // Setting ARR to max and PSC to 0 provides a base for Set_Freq.
    // A full Set_Freq call is needed to get a specific output.
    uint32_t* tim_arr = (uint32_t*)(config->TIM_Base + TIM_ARR_OFFSET); // PDF Reference (RM0368 page 307, 368, 408, 418)
    uint32_t* tim_psc = (uint32_t*)(config->TIM_Base + TIM_PSC_OFFSET); // PDF Reference (RM0368 page 307, 368, 408, 418)
    uint32_t* tim_ccr;

     if (config->TIM_Base == TIM2_BASE || config->TIM_Base == TIM5_BASE) {
        // Reserved, should not happen based on map
         return;
     } else { // 16-bit timers
        REG16(config->TIM_Base, TIM_ARR_OFFSET) = MAX_ARR_16BIT; /* PDF Reference (RM0368 page 307, 368, 408, 418) */
        REG16(config->TIM_Base, TIM_PSC_OFFSET) = 0; /* PDF Reference (RM0368 page 307, 368, 408, 418) */
        // Set initial CCR to 0 (0% duty cycle)
        if (config->TIM_Channel == 1) REG16(config->TIM_Base, TIM_CCR1_OFFSET) = 0; // PDF Reference (RM0368 page 309, 369, 409, 419)
        else if (config->TIM_Channel == 2) REG16(config->TIM_Base, TIM_CCR2_OFFSET) = 0; // PDF Reference (RM0368 page 309, 370, 409)
        else if (config->TIM_Channel == 3) REG16(config->TIM_Base, TIM_CCR3_OFFSET) = 0; // PDF Reference (RM0368 page 310, 370)
        else if (config->TIM_Channel == 4) REG16(config->TIM_Base, TIM_CCR4_OFFSET) = 0; // PDF Reference (RM0368 page 310, 371)
     }

    // For TIM1, configure BDTR (Break and Dead-Time Register)
    if (config->TIM_Base == TIM1_BASE) {
        uint32_t* tim_bdtr = (uint32_t*)(config->TIM_Base + TIM_BDTR_OFFSET);
        // Enable Main Output (MOE) - required for TIM1 outputs
        // Clear other bits like dead-time, break, lock, OSSI/OSSR for basic PWM
        // Default BDTR reset value is 0, which means MOE=0, AOE=0, BKP=0, BKE=0, OSSR=0, OSSI=0, LOCK=00, DTG=0
        // We only need to set MOE and clear everything else if needed.
        // For basic PWM, leave OSSI=0, OSSR=0 (outputs disabled when MOE=0)
        CLEAR_REG(*tim_bdtr); // Reset BDTR to default (0)
        // MOE will be set in PWM_Start
    }

    // Generate an update event to load the Prescaler and ARR registers
    // URS bit (Update Request Source) in CR1 is 0 by default, meaning UG generates update and sets UIF.
    uint32_t* tim_egr = (uint32_t*)(config->TIM_Base + TIM_EGR_OFFSET);
    SET_BIT(*tim_egr, TIM_EGR_UG_Msk); /* PDF Reference (RM0368 page 298, 361, 403, 414) */

     // Clear the update flag (UIF) after generating the update event
     // TIMx_SR register needs to be read/write cleared, except for overcapture flags which are read-only.
    uint32_t* tim_sr = (uint32_t*)(config->TIM_Base + TIM_SR_OFFSET); // PDF Reference (RM0368 page 296, 359, 402, 413)
    CLEAR_BIT(*tim_sr, 0x1FU); // Clear UIF, CC1IF, CC2IF, CC3IF, CC4IF (if they exist)

    /*
     * Counter Enable (CEN) is left disabled in Init.
     * It will be enabled in PWM_Start().
     * Channel Enable (CCxE) is left disabled in Init.
     * It will be enabled in PWM_Start().
     * Main Output Enable (MOE) for TIM1 is left disabled in Init.
     * It will be set in PWM_Start().
     */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired frequency in Hz.
 * @param duty: The desired duty cycle (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, uint32_t frequency, uint8_t duty) {
    if (TRD_Channel >= PWM_CHANNEL_COUNT || frequency == 0 || duty > 100) {
        /* Invalid channel, frequency, or duty cycle */
        return;
    }

    const PWM_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t Timer_Clock = TIMER_CLOCK_FREQ; /* Assumed PWM config - please verify */

    // Calculate total timer counts per period: Total_Counts = Timer_Clock / frequency
    uint32_t total_counts = Timer_Clock / frequency; /* PDF Reference (RM0368 page 246, 318, 379) implies relation between freq, psc, arr) */

    // Find PSC and ARR values for the desired frequency.
    // We use 16-bit timers, so ARR must be <= 65535.
    // Total_Counts = (PSC + 1) * (ARR + 1)
    uint16_t psc_val = 0;
    uint16_t arr_val = 0;

    // Check if frequency is too high (total_counts must be at least 1 for ARR=0 with PSC=0)
    if (total_counts == 0) {
        /* Frequency is too high for the timer clock */
        return;
    }

    // Find PSC and ARR. Iterate through possible (PSC + 1) values.
    // Prioritize smallest PSC for highest ARR and best resolution.
    uint32_t max_arr_plus_1 = MAX_ARR_16BIT + 1;

    for (uint32_t psc_plus_1 = 1; psc_plus_1 <= max_arr_plus_1; psc_plus_1++) {
        if ((total_counts % psc_plus_1) == 0) {
            uint32_t arr_plus_1 = total_counts / psc_plus_1;
            if (arr_plus_1 > 0 && arr_plus_1 <= max_arr_plus_1) {
                 // Found a valid PSC and ARR combination
                psc_val = psc_plus_1 - 1;
                arr_val = arr_plus_1 - 1;
                break; // Exit loop, found the smallest PSC
            }
        }
         if (psc_plus_1 == max_arr_plus_1 && arr_val == 0 && total_counts > 1) {
             // If loop finished without finding a valid pair, and total_counts > 1 (ARR=0, PSC=0 is valid for total_counts=1)
             // This frequency cannot be generated exactly with 16-bit timer using integer PSC/ARR.
             // In a real application, one might approximate, but for production-ready, we might reject.
             // Or, iterate PSC from 0 to 65535 and accept the closest match below target.
              return; /* Frequency cannot be generated exactly */ /* Assumed PWM config - cannot find exact integer PSC/ARR */
         }
    }

    // Calculate Compare value (CCR) for the desired duty cycle
    // Duty cycle = (CCR_Value + 1) / (ARR_Value + 1) * 100  (Approximate for PWM mode 1)
    // Or for PWM1 upcounting: Output High when CNT < CCR.
    // Period = ARR + 1 counts. Pulse = CCR counts. Duty = CCR / (ARR + 1) * 100.
    // CCR = (ARR + 1) * Duty / 100
    uint32_t ccr_val = (uint32_t)((arr_val + 1) * duty / 100); /* PDF Reference (RM0368 page 268, 336, 391) PWM mode logic */

    // Handle 100% duty cycle: CCR should be equal to ARR+1 to keep output high for the whole period
    if (duty == 100) {
         ccr_val = arr_val + 1; /* PDF Reference (RM0368 page 269, 337, 392) PWM mode 1 upcounting 100% duty */
    }
    // Handle 0% duty cycle: CCR should be 0 to keep output low for the whole period
    if (duty == 0) {
        ccr_val = 0; /* PDF Reference (RM0368 page 269, 337, 392) PWM mode 1 upcounting 0% duty */
    }


    // Update PSC, ARR, and CCR registers
    uint32_t* tim_arr = (uint32_t*)(config->TIM_Base + TIM_ARR_OFFSET); // PDF Reference (RM0368 page 307, 368, 408, 418)
    uint32_t* tim_psc = (uint32_t*)(config->TIM_Base + TIM_PSC_OFFSET); // PDF Reference (RM0368 page 307, 368, 408, 418)
    uint32_t* tim_ccr;

    // These timers are 16-bit, so use 16-bit access for PSC, ARR, CCR
    REG16(config->TIM_Base, TIM_PSC_OFFSET) = psc_val; /* PDF Reference (RM0368 page 307, 368, 408, 418) */
    REG16(config->TIM_Base, TIM_ARR_OFFSET) = arr_val; /* PDF Reference (RM0368 page 307, 368, 408, 418) */

    if (config->TIM_Channel == 1) {
         REG16(config->TIM_Base, TIM_CCR1_OFFSET) = (uint16_t)ccr_val; // PDF Reference (RM0368 page 309, 369, 409, 419)
    } else if (config->TIM_Channel == 2) {
         REG16(config->TIM_Base, TIM_CCR2_OFFSET) = (uint16_t)ccr_val; // PDF Reference (RM0368 page 309, 370, 409)
    } else if (config->TIM_Channel == 3) {
         REG16(config->TIM_Base, TIM_CCR3_OFFSET) = (uint16_t)ccr_val; // PDF Reference (RM0368 page 310, 370)
    } else if (config->TIM_Channel == 4) {
         REG16(config->TIM_Base, TIM_CCR4_OFFSET) = (uint16_t)ccr_val; // PDF Reference (RM0368 page 310, 371)
    }

    // Generate an update event to load the new PSC and ARR values from preload registers
    // The UG bit must be set after changing PSC and ARR (and CCR if preload is enabled)
    // Only if UDIS=0 in CR1 (checked in Init), this will update registers and can set UIF.
    // URS bit (Update Request Source) is 0 by default in CR1, so UG sets UIF.
    uint32_t* tim_egr = (uint32_t*)(config->TIM_Base + TIM_EGR_OFFSET);
    SET_BIT(*tim_egr, TIM_EGR_UG_Msk); /* PDF Reference (RM0368 page 298, 361, 403, 414) */

    // Clear the update flag (UIF) after generating the update event if it was set (URS=0)
    uint32_t* tim_sr = (uint32_t*)(config->TIM_Base + TIM_SR_OFFSET); // PDF Reference (RM0368 page 296, 359, 402, 413)
    CLEAR_BIT(*tim_sr, 0x1U); // Clear only UIF
}

/**
 * @brief Starts PWM signal generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= PWM_CHANNEL_COUNT) {
        /* Invalid channel */
        return;
    }

    const PWM_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the specific channel output
    uint32_t* tim_ccer = (uint32_t*)(config->TIM_Base + TIM_CCER_OFFSET); // PDF Reference (RM0368 page 303, 366, 407, 417)

    if (config->TIM_Channel == 1) {
         SET_BIT(*tim_ccer, TIM_CCER_CC1E_Msk); // PDF Reference (RM0368 page 305, 367, 407, 418)
    } else if (config->TIM_Channel == 2) {
         SET_BIT(*tim_ccer, TIM_CCER_CC2E_Msk); // PDF Reference (RM0368 page 304, 367, 407)
    } else if (config->TIM_Channel == 3) {
         SET_BIT(*tim_ccer, TIM_CCER_CC3E_Msk); // PDF Reference (RM0368 page 303, 367)
    } else if (config->TIM_Channel == 4) {
         SET_BIT(*tim_ccer, TIM_CCER_CC4E_Msk); // PDF Reference (RM0368 page 303, 367)
    }

    // For TIM1, enable the Main Output (MOE) in BDTR
    if (config->TIM_Base == TIM1_BASE) {
        uint32_t* tim_bdtr = (uint32_t*)(config->TIM_Base + TIM_BDTR_OFFSET); // PDF Reference (RM0368 page 310)
        SET_BIT(*tim_bdtr, TIM_BDTR_MOE_Msk); /* PDF Reference (RM0368 page 310) */
    }

    // Enable the Counter (CEN)
    uint32_t* tim_cr1 = (uint32_t*)(config->TIM_Base + TIM_CR1_OFFSET); // PDF Reference (RM0368 page 288, 353, 398, 412)
    SET_BIT(*tim_cr1, TIM_CR1_CEN_Msk); /* PDF Reference (RM0368 page 289, 354, 398, 413) */
}

/**
 * @brief Stops PWM signal output on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= PWM_CHANNEL_COUNT) {
        /* Invalid channel */
        return;
    }

    const PWM_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the specific channel output
    // Note: For TIM1, disabling MOE in BDTR affects all channels on that timer.
    // A safer approach for TIM1 might be to only disable the specific CCxE/CCxNE bits.
    uint32_t* tim_ccer = (uint32_t*)(config->TIM_Base + TIM_CCER_OFFSET); // PDF Reference (RM0368 page 303, 366, 407, 417)

     if (config->TIM_Channel == 1) {
         CLEAR_BIT(*tim_ccer, TIM_CCER_CC1E_Msk); // PDF Reference (RM0368 page 305, 367, 407, 418)
         if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC1NE_Msk); // PDF Reference (RM0368 page 304)
    } else if (config->TIM_Channel == 2) {
         CLEAR_BIT(*tim_ccer, TIM_CCER_CC2E_Msk); // PDF Reference (RM0368 page 304, 367, 407)
         if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC2NE_Msk); // PDF Reference (RM0368 page 304)
    } else if (config->TIM_Channel == 3) {
         CLEAR_BIT(*tim_ccer, TIM_CCER_CC3E_Msk); // PDF Reference (RM0368 page 303, 367)
         if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC3NE_Msk); // PDF Reference (RM0368 page 303)
    } else if (config->TIM_Channel == 4) {
         CLEAR_BIT(*tim_ccer, TIM_CCER_CC4E_Msk); // PDF Reference (RM0368 page 303, 367)
         // TIM1 does not have CC4NE
    }

    // For TIM1, check if any channel is still enabled. If not, disable MOE.
    if (config->TIM_Base == TIM1_BASE) {
        uint32_t* tim_bdtr = (uint32_t*)(config->TIM_Base + TIM_BDTR_OFFSET); // PDF Reference (RM0368 page 310)
        // Read current CCER state to see if any CCxE or CCxNE bit is set
        if ((*tim_ccer & (TIM_CCER_CC1E_Msk | TIM_CCER_CC2E_Msk | TIM_CCER_CC3E_Msk | TIM_CCER_CC4E_Msk |
                          TIM_CCER_CC1NE_Msk | TIM_CCER_CC2NE_Msk | TIM_CCER_CC3NE_Msk | TIM_CCER_CC3NP_Msk)) == 0) // Check if ANY enable bit is set
        {
            CLEAR_BIT(*tim_bdtr, TIM_BDTR_MOE_Msk); /* PDF Reference (RM0368 page 310) */
        }
    }

    // Note: The counter (CEN) is left running.
    // To stop the counter, clear the CEN bit in TIMx_CR1.
    // CLEAR_BIT(*tim_cr1, TIM_CR1_CEN_Msk); // Optional: Stop the counter for lower power
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 *        Disables clocks for all timers and GPIOs used in the pwm_channel_map.
 */
void PWM_PowerOff(void) {
    // Disable outputs and counters for all configured PWM channels
    for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        const PWM_Config_t *config = &pwm_channel_map[i];

        // Disable the specific channel output
        uint32_t* tim_ccer = (uint32_t*)(config->TIM_Base + TIM_CCER_OFFSET); // PDF Reference (RM0368 page 303, 366, 407, 417)

         if (config->TIM_Channel == 1) {
             CLEAR_BIT(*tim_ccer, TIM_CCER_CC1E_Msk); // PDF Reference (RM0368 page 305, 367, 407, 418)
             if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC1NE_Msk); // PDF Reference (RM0368 page 304)
        } else if (config->TIM_Channel == 2) {
             CLEAR_BIT(*tim_ccer, TIM_CCER_CC2E_Msk); // PDF Reference (RM0368 page 304, 367, 407)
             if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC2NE_Msk); // PDF Reference (RM0368 page 304)
        } else if (config->TIM_Channel == 3) {
             CLEAR_BIT(*tim_ccer, TIM_CCER_CC3E_Msk); // PDF Reference (RM0368 page 303, 367)
             if (config->TIM_Base == TIM1_BASE) CLEAR_BIT(*tim_ccer, TIM_CCER_CC3NE_Msk); // PDF Reference (RM0368 page 303)
        } else if (config->TIM_Channel == 4) {
             CLEAR_BIT(*tim_ccer, TIM_CCER_CC4E_Msk); // PDF Reference (RM0368 page 303, 367)
             // TIM1 does not have CC4NE
        }

        // For TIM1, disable Main Output (MOE)
        if (config->TIM_Base == TIM1_BASE) {
            uint32_t* tim_bdtr = (uint32_t*)(config->TIM_Base + TIM_BDTR_OFFSET); // PDF Reference (RM0368 page 310)
            CLEAR_BIT(*tim_bdtr, TIM_BDTR_MOE_Msk); /* PDF Reference (RM0368 page 310) */
        }

        // Disable the Counter (CEN)
        uint32_t* tim_cr1 = (uint32_t*)(config->TIM_Base + TIM_CR1_OFFSET); // PDF Reference (RM0368 page 288, 353, 398, 412)
        CLEAR_BIT(*tim_cr1, TIM_CR1_CEN_Msk); /* PDF Reference (RM0368 page 289, 354, 398, 413) */

        // Reset GPIO pin to default state (Input Floating)
        uint32_t* gpio_moder = (uint32_t*)(config->GPIO_Port_Base + GPIO_MODER_OFFSET); // PDF Reference (RM0368 page 158)
        MODIFY_REG(*gpio_moder,
                   GPIO_MODER_MODERy_Msk(config->GPIO_Pin),
                   GPIO_MODER_INPUT(config->GPIO_Pin)); /* PDF Reference (RM0368 page 158, 148) */

        // Clear Alternate Function selection
         if (config->GPIO_Pin < 8) {
            uint32_t* gpio_afrl = (uint32_t*)(config->GPIO_Port_Base + GPIO_AFRL_OFFSET); // PDF Reference (RM0368 page 162)
            MODIFY_REG(*gpio_afrl,
                       GPIO_AFRL_AFRLy_Msk(config->GPIO_Pin),
                       0);
        } else {
            uint32_t* gpio_afrh = (uint32_t*)(config->GPIO_Port_Base + GPIO_AFRH_OFFSET); // PDF Reference (RM0368 page 163)
            MODIFY_REG(*gpio_afrh,
                       GPIO_AFRH_AFRHy_Msk(config->GPIO_Pin),
                       0);
        }
         // Reset OTYPER, OSPEEDR, PUPDR bits for the pin to reset values (0)
         uint32_t* gpio_otyper = (uint32_t*)(config->GPIO_Port_Base + GPIO_OTYPER_OFFSET); // PDF Reference (RM0368 page 158)
         CLEAR_BIT(*gpio_otyper, GPIO_OTYPER_OTy_Msk(config->GPIO_Pin)); // PDF Reference (RM0368 page 158)
         uint32_t* gpio_ospeedr = (uint32_t*)(config->GPIO_Port_Base + GPIO_OSPEEDR_OFFSET); // PDF Reference (RM0368 page 159)
         MODIFY_REG(*gpio_ospeedr, GPIO_OSPEEDR_OSPEEDRy_Msk(config->GPIO_Pin), 0); // PDF Reference (RM0368 page 159)
         uint32_t* gpio_pupdr = (uint32_t*)(config->GPIO_Port_Base + GPIO_PUPDR_OFFSET); // PDF Reference (RM0368 page 159)
         MODIFY_REG(*gpio_pupdr, GPIO_PUPDR_PUPDRy_Msk(config->GPIO_Pin), 0); // PDF Reference (RM0368 page 159)
    }

    // Disable clocks for all timers used in the map
    uint32_t* rcc_apb1enr = (uint32_t*)(RCC_BASE + RCC_APB1ENR_OFFSET);
    uint32_t* rcc_apb2enr = (uint32_t*)(RCC_BASE + RCC_APB2ENR_OFFSET);
    uint32_t* rcc_ahb1enr = (uint32_t*)(RCC_BASE + RCC_AHB1ENR_OFFSET);

    uint32_t used_apb1_tim_mask = 0;
    uint32_t used_apb2_tim_mask = 0;
    uint32_t used_ahb1_gpio_mask = 0;

    for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        const PWM_Config_t *config = &pwm_channel_map[i];
        if (config->TIM_Base == TIM1_BASE || config->TIM_Base == TIM9_BASE || config->TIM_Base == TIM10_BASE || config->TIM_Base == TIM11_BASE) {
            used_apb2_tim_mask |= config->RCC_APB_ENR_Mask;
        } else { // TIM3, TIM4
            used_apb1_tim_mask |= config->RCC_APB_ENR_Mask;
        }
        used_ahb1_gpio_mask |= config->RCC_AHB1ENR_Mask;
    }

    CLEAR_BIT(*rcc_apb1enr, used_apb1_tim_mask);
    CLEAR_BIT(*rcc_apb2enr, used_apb2_tim_mask);
    CLEAR_BIT(*rcc_ahb1enr, used_ahb1_gpio_mask); /* PDF Reference (RM0368 GPIO intro) */
}