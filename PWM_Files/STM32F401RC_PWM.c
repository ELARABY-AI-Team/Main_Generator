#include "stm32f4xx.h"

// Configure TIM11 for 1Hz with 50% duty cycle
void configure_TIM11(void) {
    // Enable TIM11 clock source
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; // PDF Reference

    // Prescaler and Auto-Reload configuration
    TIM11->PSC = 7999;                    // (8MHz / (8000)) = 1kHz timing clock
    TIM11->ARR = 999;                     // (1000) for 1Hz output frequency
    TIM11->CCR1 = 499;                    // 50% duty cycle

    // Configuration Mode Register 1
    TIM11->CCMR1 |= TIM_CCMR1_OC1_PE;     // PDF Reference: Output mode with preload
    TIM11->CCMR1 &= ~(TIM_CCMR1_CC1S);    // Use CC1S bits for output mode (PDF Reference)
    
    // Configuration Mode Register 2 (if needed) and Channel Enable
    TIM11->CCER |= TIM_CCER_CC1E;        // Enable output on CC1 channel

    // Generate update interrupt
    TIM11->DIER |= TIM_DIER_UIE;          // PDF Reference: Enable update interrupts
    NVIC_EnableIRQ(TIM11_IRQn);            // Enable NVIC interrupt for TIM11
    
    // Start the timer
    TIM11->CR1 |= TIM_CR1_CEN;            // Start the timer (PDF Reference)
}

// Interrupt handler for TIM11 overflow
void TIM11_IRQHandler(void) {
    if (TIM11->SR & TIM_SR_UIF) {         // Check if update interrupt occurred
        TIM11->SR &= ~TIM_SR_UIF;          // Clear the interrupt flag
        
        // Add your non-blocking application code here
        // Example: Toggle an LED or perform some action
    }
}