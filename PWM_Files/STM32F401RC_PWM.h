/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the PWM module on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-21
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h" /* Mandatory include for common types and definitions */
#include "STM32F401RC_GPIO.h" /* Mandatory include for GPIO definitions */

/*
 * TRD_Channel_t Enum Definition
 *
 * This enum identifies all PWM-capable channels on the STM32F401RC microcontroller,
 * excluding specific timers reserved for other system functionalities (e.g., OS ticks, delays).
 *
 * Reserved Timers (not included in this enum):
 * - TIM5 (with all its channels: CH1, CH2, CH3, CH4)
 * - TIM11 (with its channel: CH1)
 * These timers are reserved to ensure dedicated resources are available for system-level operations
 * such as RTOS timekeeping or precise delay generation, if required by the application.
 * This decision is based on the requirement to reserve at least two timers (with all their channels)
 * for non-PWM functionalities, even if they are technically PWM-capable.
 */
typedef enum TRD_Channel_t
{
    TIM1_CH1 = 0U,  /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 15 */
    TIM1_CH2,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 15 */
    TIM1_CH3,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 15 */
    TIM1_CH4,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 15 */
    TIM2_CH1,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM2_CH2,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM2_CH3,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM2_CH4,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM3_CH1,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM3_CH2,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM3_CH3,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM3_CH4,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM4_CH1,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM4_CH2,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM4_CH3,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM4_CH4,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 16 */
    TIM9_CH1,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 18 */
    TIM9_CH2,       /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 18 */
    TIM10_CH1,      /* PDF Reference: STM32F401xB/C Datasheet (DS9011), Table 10; Reference Manual (RM0368), Section 18 */
    TRD_CHANNEL_TOTAL_COUNT /* Total number of PWM channels defined in this enum. */
} TRD_Channel_t;

/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Initializes the specified PWM channel.
 * @param TRD_Channel: The PWM channel to initialize (e.g., TIMx_CHx).
 */
void PWM_Init(TRD_Channel_t TRD_Channel);

/**
 * @brief Sets the frequency and duty cycle for the specified PWM channel.
 * @param TRD_Channel: The PWM channel to configure.
 * @param frequency: The desired PWM frequency in Hz.
 * @param duty: The desired PWM duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/**
 * @brief Starts the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel);

/**
 * @brief Stops the PWM generation on the specified channel.
 * @param TRD_Channel: The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel);

/**
 * @brief De-initializes all active PWM peripherals and powers them off.
 */
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */