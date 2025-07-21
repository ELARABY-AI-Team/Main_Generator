#include "TIM.h"

void TIM_ITRx_SetConfig(TIM_TypeDef *TIMx, uint32_t InputTriggerSource) {
    uint32_t tmpsmcr;

    /* Get the TIMx SMCR register value */
    tmpsmcr = TIMx->SMCR;
    /* Reset the TS Bits */
    tmpsmcr &= ~TIM_SMCR_TS;
    /* Set the Input Trigger source and disable external trigger (for internal triggers) */
    tmpsmcr |= InputTriggerSource | TIM_SLAVEMODE_DISABLE;
    /* Write to TIMx SMCR */
    TIMx->SMCR = tmpsmcr;
}

void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter) {
    uint32_t tmpsmcr;

    tmpsmcr = TIMx->SMCR;

    /* Reset the ETR Bits */
    tmpsmcr &= ~(TIM_SMCR_ETP | TIM_SMCR_ECE | TIM_SMCR_ETPS | TIM_SMCR_ETF);

    /* Configure ETR Prescaler, Filter and Polarity */
    tmpsmcr |= (uint32_t)(TIM_ExtTRGPrescaler |
                          ((TIM_ExtTRGPolarity == TIM_ETRPOLARITY_INVERTED) ? TIM_SMCR_ETP : 0) |
                          (ExtTRGFilter << 8));

    /* Write to TIMx SMCR */
    TIMx->SMCR = tmpsmcr;
}

void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel,
                      ControlStatus ChannelState) {
    uint32_t cc_state = ChannelState;

    switch (Channel) {
        case TIM_CHANNEL_1:
            TIMx->CCER &= ~TIM_CCER_CC1E;
            if (cc_state == ENABLE) {
                TIMx->CCER |= TIM_CCER_CC1E;
            }
            break;
        case TIM_CHANNEL_2:
            TIMx->CCER &= ~TIM_CCER_CC2E;
            if (cc_state == ENABLE) {
                TIMx->CCER |= TIM_CCER_CC2E;
            }
            break;
        case TIM_CHANNEL_3:
            TIMx->CCER &= ~TIM_CCER_CC3E;
            if (cc_state == ENABLE) {
                TIMx->CCER |= TIM_CCER_CC3E;
            }
            break;
        case TIM_CHANNEL_4:
            TIMx->CCER &= ~TIM_CCER_CC4E;
            if (cc_state == ENABLE) {
                TIMx->CCER |= TIM_CCER_CC4E;
            }
            break;
        default:
            /* Handle invalid channel */
            break;
    }
}