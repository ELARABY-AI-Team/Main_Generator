#include "stm32f4xx.h"

void WDT_Reset(void) {
    // Assume watchdog reset implementation here
}

uint8_t GPIO_Value_Get(GPIO_TypeDef* p, uint16_t pin) {
    WDT_Reset();
    return (p->IDR & (1 << pin)) ? 1 : 0;
}

void GPIO_Value_Set(GPIO_TypeDef* p, uint16_t pin, uint8_t value) {
    WDT_Reset();
    if (value) {
        p->BSRR |= (1 << pin);
    } else {
        p->BRR |= (1 << pin);
    }
}

GPIO_Direction_TypeDef GPIO_Direction_Get(GPIO_TypeDef* p, uint16_t pin) {
    WDT_Reset();
    uint32_t moder = p->MODER;
    uint8_t bits = (moder >> (pin * 2)) & 0x03;
    if (bits == 0x01) {
        return GPIO_Direction_Output;
    } else {
        return GPIO_Direction_Input;
    }
}

void GPIO_Output_Init(GPIO_TypeDef* p, uint16_t pin, uint8_t value, uint8_t pull) {
    WDT_Reset();
    // Disable any existing configuration
    uint32_t mask = (0x03 << (pin * 2));
    p->PUPDR &= ~mask;
    
    if (pull == GPIO_PuPd_NOPULL) {
        p->PUPDR |= 0x00 << (pin * 2);
    } else if (pull == GPIO_PuPd_UP) {
        p->PUPDR |= 0x01 << (pin * 2); // PUPDRy = 01 for pull-up
    } else { // GPIO_PuPd_DOWN
        p->PUPDR |= 0x02 << (pin * 2); // PUPDRy = 10 for pull-down
    }

    // Set the pin as output
    mask = (0x03 << (pin * 2));
    p->MODER &= ~mask;
    p->MODER |= 0x01 << (pin * 2);

    // Set initial value
    if (value) {
        p->BSRR |= (1 << pin);
    } else {
        p->BRR |= (1 << pin);
    }
}