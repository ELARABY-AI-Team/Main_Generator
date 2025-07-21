#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/* ==================== PWM CHANNEL ENUM ==================== */
typedef enum TRD_Channel_t {
    /* Timer 1 Channels */
    TIM1_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM1 has 4 channels */
    TIM1_CH2,   /* PDF Reference: STM32F401RC datasheet */
    TIM1_CH3,   /* PDF Reference: STM32F401RC datasheet */
    TIM1_CH4,   /* PDF Reference: STM32F401RC datasheet */

    /* Timer 2 Channels */
    TIM2_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM2 has 4 channels */
    TIM2_CH2,   /* PDF Reference: STM32F401RC datasheet */
    TIM2_CH3,   /* PDF Reference: STM32F401RC datasheet */
    TIM2_CH4,   /* PDF Reference: STM32F401RC datasheet */

    /* Timer 3 Channels */
    TIM3_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM3 has 4 channels */
    TIM3_CH2,   /* PDF Reference: STM32F401RC datasheet */
    TIM3_CH3,   /* PDF Reference: STM32F401RC datasheet */
    TIM3_CH4,   /* PDF Reference: STM32F401RC datasheet */

    /* Timer 4 Channels */
    TIM4_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM4 has 4 channels */
    TIM4_CH2,   /* PDF Reference: STM32F401RC datasheet */
    TIM4_CH3,   /* PDF Reference: STM32F401RC datasheet */
    TIM4_CH4,   /* PDF Reference: STM32F401RC datasheet */

    /* Timer 5 Channels */
    TIM5_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM5 has 2 channels */
    TIM5_CH2,   /* PDF Reference: STM32F401RC datasheet */

    /* Timer 8 Channels */
    TIM8_CH1,   /* PDF Reference: STM32F401RC datasheet - TIM8 has 4 channels */
    TIM8_CH2,   /* PDF Reference: STM32F401RC datasheet */
    TIM8_CH3,   /* PDF Reference: STM32F401RC datasheet */
    TIM8_CH4    /* PDF Reference: STM32F401RC datasheet */
} TRD_Channel_t;

/* Timer 6 and Timer 7 are reserved for system use and do not support PWM output */

/* ==================== FUNCTION DECLARATIONS ==================== */
void PWM_Init(TRD_Channel_t channel);
void PWM_Set_Freq(TRD_Channel_t channel, tlong frequency, tbyte duty);
void PWM_Start(TRD_Channel_t channel);
void PWM_Stop(TRD_Channel_t channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */