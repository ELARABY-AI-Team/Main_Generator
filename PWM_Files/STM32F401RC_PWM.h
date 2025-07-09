/***********************************************************************************************************************
* File Name      : STM32F401RC_PWM.h
* Description    : Header file for the PWM driver on STM32F401RC
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

/*
 * Include necessary header files.
 * STM32F401RC_MAIN.h is expected to define base types like tbyte, tlong.
 * STM32F401RC_GPIO.h is expected to define necessary GPIO configurations.
 */
#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"


/*
 * typedef enum TRD_Channel_t:
 * Defines the enumeration for all supported PWM channels on the STM32F401RC.
 * Channels are identified based on the STM32F401RC Reference Manual (RM0368).
 *
 * Note on Reserved Timers:
 * Timers TIM10 and TIM11, although capable of PWM on single channels,
 * are intentionally not included in this enum to potentially reserve
 * them for other system uses (e.g., delays, OS ticks).
 */
typedef enum TRD_Channel_t
{
    /* TIM1 channels (Advanced Control Timer) */
    TRD_TIM1_CH1, /* PDF Reference */
    TRD_TIM1_CH2, /* PDF Reference */
    TRD_TIM1_CH3, /* PDF Reference */
    TRD_TIM1_CH4, /* PDF Reference */

    /* TIM2 channels (General-Purpose Timer) */
    TRD_TIM2_CH1, /* PDF Reference */
    TRD_TIM2_CH2, /* PDF Reference */
    TRD_TIM2_CH3, /* PDF Reference */
    TRD_TIM2_CH4, /* PDF Reference */

    /* TIM3 channels (General-Purpose Timer) */
    TRD_TIM3_CH1, /* PDF Reference */
    TRD_TIM3_CH2, /* PDF Reference */
    TRD_TIM3_CH3, /* PDF Reference */
    TRD_TIM3_CH4, /* PDF Reference */

    /* TIM4 channels (General-Purpose Timer) */
    TRD_TIM4_CH1, /* PDF Reference */
    TRD_TIM4_CH2, /* PDF Reference */
    TRD_TIM4_CH3, /* PDF Reference */
    TRD_TIM4_CH4, /* PDF Reference */

    /* TIM5 channels (General-Purpose Timer) */
    TRD_TIM5_CH1, /* PDF Reference */
    TRD_TIM5_CH2, /* PDF Reference */
    TRD_TIM5_CH3, /* PDF Reference */
    TRD_TIM5_CH4, /* PDF Reference */

    /* TIM9 channels (General-Purpose Timer) */
    TRD_TIM9_CH1, /* PDF Reference */
    TRD_TIM9_CH2, /* PDF Reference */

    /* Placeholder for total count - must be the last entry */
    TRD_TOTAL_PWM_CHANNELS

} TRD_Channel_t;


/*
 * FUNCTION DECLARATIONS
 * Declarations for the PWM driver functions.
 * These functions provide the basic interface for PWM control.
 */

/***********************************************************************************************************************
* Function Name: PWM_Init
* Description  : Initializes a specific PWM channel.
* Arguments    : TRD_Channel - The PWM channel to initialize.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Init(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Set_Freq
* Description  : Sets the frequency and duty cycle for a specific PWM channel.
* Arguments    : TRD_Channel - The PWM channel to configure.
*                frequency   - The desired frequency in Hz.
*                duty        - The desired duty cycle in percentage (0-100).
* Return Value : None.
***********************************************************************************************************************/
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty);

/***********************************************************************************************************************
* Function Name: PWM_Start
* Description  : Starts the PWM generation on a specific channel.
* Arguments    : TRD_Channel - The PWM channel to start.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Start(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_Stop
* Description  : Stops the PWM generation on a specific channel.
* Arguments    : TRD_Channel - The PWM channel to stop.
* Return Value : None.
***********************************************************************************************************************/
void PWM_Stop(TRD_Channel_t TRD_Channel);

/***********************************************************************************************************************
* Function Name: PWM_PowerOff
* Description  : Powers off all active PWM peripherals and associated GPIOs.
* Arguments    : None.
* Return Value : None.
***********************************************************************************************************************/
void PWM_PowerOff(void);


#endif /* STM32F401RC_PWM_H_ */