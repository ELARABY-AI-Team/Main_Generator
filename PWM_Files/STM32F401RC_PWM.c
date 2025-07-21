#include "STM32F401RC_PWM.h"

/** @brief PWM Channel Configuration Structure */
typedef struct {
    TIM_TypeDef* TIMx;
    uint8_t       ChannelNumber;
    const char*   PortName;
    uint8_t       PinNumber;
    uint8_t       AlternateFunctionNumber;
} PWM_Channel_Config_t;

/** @brief Array mapping PWM channels to their hardware configurations */
static const PWM_Channel_Config_t pwm_channel_map[] = {
    /* Timer 1 Channels */
    {TIM1, TRD_CH1_1, "GPIOA", 8,  AF1},
    {TIM1, TRD_CH1_2, "GPIOA", 9,  AF1},
    {TIM1, TRD_CH1_3, "GPIOB", 0,  AF0}, /* Assumed PWM config */
    {TIM1, TRD_CH1_4, "GPIOB", 1,  AF0},

    /* Timer 2 Channels - Reserved for OS or delay purposes */
    /* Do not use TIM2 channels for PWM in this implementation */

    /* Timer 3 Channels */
    {TIM3, TRD_CH3_1, "GPIOA", 6,  AF2},
    {TIM3, TRD_CH3_2, "GPIOA", 7,  AF2},

    /* Timer 4 Channels */
    {TIM4, TRD_CH4_1, "GPIOB", 8,  AF2},
    {TIM4, TRD_CH4_2, "GPIOB", 9,  AF2},

    /* Timer 5 Channels - Reserved for OS or delay purposes */
    /* Do not use TIM5 channels for PWM in this implementation */

    /* Timer 8 Channels */
    {TIM8, TRD_CH8_1, "GPIOA", 0,  AF3}, /* Assumed PWM config */
    {TIM8, TRD_CH8_2, "GPIOA", 1,  AF3},

    /* Timer 9 Channels */
    {TIM9, TRD_CH9_1, "GPIOA", 4,  AF3},
    
    /* Timer 10 Channels */
    {TIM10, TRD_CH10_1, "GPIOB", 8, AF3},

    /* Timer 11 Channels */
    {TIM11, TRD_CH11_1, "GPIOA", 5, AF1}
};

/**
 * @brief Initializes the PWM hardware and configures the timer and GPIOs for the given channel.
 * 
 * @param TRD_Channel The target PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* cfg = &pwm_channel_map[TRD_Channel];
    
    /* Enable Timer clock */
    if (cfg->TIMx == TIM1 || cfg->TIMx == TIM2 || cfg->TIMx == TIM3 ||
        cfg->TIMx == TIM4) {
        RCC->APB1ENR |= (1 << 0); /* Enable Timer clock for APB1 timers */
    } else if (cfg->TIMx == TIM8 || cfg->TIMx == TIM9 ||
               cfg->TIMx == TIM10 || cfg->TIMx == TIM11) {
        RCC->APB2ENR |= (1 << 0); /* Enable Timer clock for APB2 timers */
    }

    /* Enable GPIO clock */
    if (cfg->PortName == "GPIOA") {
        RCC->IOPAEN = 0x01;
    } else if (cfg->PortName == "GPIOB") {
        RCC->IOPEEN = 0x01;
    }

    /* Configure GPIO as Alternate Function Push-Pull */
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = cfg->PinNumber + 1;     /* Add 1 since PinNumber starts at 0 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = cfg->AlternateFunctionNumber;

    if (cfg->PortName == "GPIOA") {
        GPIOA->AFR[cfg->PinNumber >> 3] |= (cfg->AlternateFunctionNumber << (4 * (cfg->PinNumber & 0x07)));
        HAL_GPIO_Init(GPIOA, &gpio_init);
    } else if (cfg->PortName == "GPIOB") {
        GPIOB->AFR[cfg->PinNumber >> 3] |= (cfg->AlternateFunctionNumber << (4 * (cfg->PinNumber & 0x07)));
        HAL_GPIO_Init(GPIOB, &gpio_init);
    }

    /* Configure Timer for PWM */
    cfg->TIMx->CCER |= (1 << (cfg->ChannelNumber + 2)); /* Enable Output */
    cfg->TIMx->CCMR1 |= (1 << 6);                     /* Select PWM Mode 1 */
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 *
 * @param TRD_Channel The target PWM channel to configure.
 * @param frequency   Desired frequency in Hz.
 * @param duty        Duty cycle as a percentage (0-100%).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    const PWM_Channel_Config_t* cfg = &pwm_channel_map[TRD_Channel];
    
    uint32_t period = SystemCoreClock / (frequency * 1);
    uint32_t prescaler = (SystemCoreClock / 1000000) / frequency;
    
    cfg->TIMx->PSC = prescaler; /* Set Prescaler */
    cfg->TIMx->ARR = period - 1; /* Set Auto-Reload Register */
    
    if (duty <= 100) {
        uint32_t duty_cycle = ((period - 1) * duty) / 100;
        cfg->TIMx->CCR[cfg->ChannelNumber] = duty_cycle;
    }
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 *
 * @param TRD_Channel The target PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* cfg = &pwm_channel_map[TRD_Channel];
    
    cfg->TIMx->CCER |= (1 << cfg->ChannelNumber); /* Enable Output */
    cfg->TIMx->CR1 |= 0x01;                      /* Start Timer */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 *
 * @param TRD_Channel The target PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    const PWM_Channel_Config_t* cfg = &pwm_channel_map[TRD_Channel];
    
    cfg->TIMx->CCER &= ~(1 << cfg->ChannelNumber); /* Disable Output */
    cfg->TIMx->CR1 &= ~0x01;                      /* Stop Timer */
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 */
void PWM_PowerOff(void) {
    /* Disable all timers */
    for (int i = 0; i < sizeof(pwm_channel_map); ++i) {
        const PWM_Channel_Config_t* cfg = &pwm_channel_map[i];
        cfg->TIMx->CR1 &= ~0x01;
        cfg->TIMx->CCER &= ~(1 << cfg->ChannelNumber);
    }

    /* Turn off all GPIO outputs */
    RCC->IOPAEN = 0;   // Disable GPIO clocks
    RCC->IOPEEN = 0;

    /* Power down peripherals */
}