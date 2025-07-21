#ifndef STM32F401RC_PWM_H_
#define STM32F401RC_PWM_H_

#include "STM32F401RC_MAIN.h"
#include "STM32F401RC_GPIO.h"

/*
 * PWM Channel Definitions for STM32F401RC
 */
typedef enum TRD_Channel_t {
    TIM2_CH1,  /* PDF Reference: TIM2 CH1 - 32-bit general purpose timer */
    TIM2_CH2,  /* PDF Reference: TIM2 CH2 - 32-bit general purpose timer */
    TIM3_CH1,  /* PDF Reference: TIM3 CH1 - 32-bit general purpose timer */
    TIM3_CH2,  /* PDF Reference: TIM3 CH2 - 32-bit general purpose timer */
    TIM4_CH1,  /* PDF Reference: TIM4 CH1 - 16-bit basic timer */
    TIM4_CH2,  /* PDF Reference: TIM4 CH2 - 16-bit basic timer */
    TIM5_CH1,  /* PDF Reference: TIM5 CH1 - 16-bit basic timer */
    TIM5_CH2   /* PDF Reference: TIM5 CH2 - 16-bit basic timer */
    /* Reserved for non-PWM timing operations */
    /* TIM6: 16-bit dedicated timer for RTC (not PWM capable) */
    /* TIM7: 16-bit dedicated timer for RTC (not PWM capable) */
} TRD_Channel_t;

/*
 * PWM Function Declarations
 */
void PWM_Init(const TRD_Channel_t channel);
void PWM_Set_Freq(const TRD_Channel_t channel, const tlong frequency, const tbyte duty);
void PWM_Start(const TRD_Channel_t channel);
void PWM_Stop(const TRD_Channel_t channel);
void PWM_PowerOff(void);

#endif /* STM32F401RC_PWM_H_ */