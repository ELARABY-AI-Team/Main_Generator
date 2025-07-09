/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : ATMEGA32 PWM Driver
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#include "ATMEGA32_PWM.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h> // Required for floating point calculations
#include <stdbool.h> // Required for boolean types

// Define the CPU frequency as a constant. This must match the actual MCU clock.
// /* Assumed F_CPU value - please verify */
#ifndef F_CPU
#define F_CPU 8000000UL // Default to 8 MHz (common ATMEGA32 speed)
#endif

// Define standard types (assuming tlong is uint32_t, tbyte is uint8_t)
/* Assumed type definitions for tlong and tbyte - please verify */
typedef uint32_t tlong;
typedef uint8_t tbyte;

/***********************************************************************************************************************
* Private Defines
***********************************************************************************************************************/

// Timer0/2 Fast PWM TOP value
#define T0_T2_PWM_TOP      255 // 0xFF /* PDF Reference - Timer/Counter0, Timer/Counter2, Fast PWM Mode 3, TOP=0xFF */
#define T0_T2_PWM_TOP_F    256.0 // TOP + 1 for calculations

// Timer1 Fast PWM TOP value (ICR1 as TOP for full 16-bit flexibility)
// WGM13=1, WGM12=1, WGM11=1, WGM10=0 -> Mode 14 (Fast PWM, TOP is ICR1)
// Requires ICR1 to be set to frequency calculation.
#define T1_PWM_MODE_WGM_BITS ((1 << WGM11) | (1 << WGM13) | (1 << WGM12)) /* PDF Reference - TCCR1A/B, WGM13:0, Mode 14 */

// Prescaler values and their corresponding CS bit settings
typedef struct {
    uint16_t prescaler_val;
    uint8_t cs_bits;
} PrescalerConfig_t;

// Timer0/1 Prescaler configurations /* PDF Reference - Timer/Counter0 and Timer/Counter1 Prescalers, Table 42 */
const PrescalerConfig_t timer0_1_prescalers[] = {
    {1,   (1 << CS00)},
    {8,   (1 << CS01)},
    {64,  (1 << CS01) | (1 << CS00)},
    {256, (1 << CS02)},
    {1024,(1 << CS02) | (1 << CS00)}
};
#define NUM_TIMER0_1_PRESCALERS (sizeof(timer0_1_prescalers) / sizeof(PrescalerConfig_t))

// Timer2 Prescaler configurations /* PDF Reference - Timer/Counter2 Prescaler, Table 54 */
const PrescalerConfig_t timer2_prescalers[] = {
    {1,   (1 << CS20)},
    {8,   (1 << CS21)},
    {32,  (1 << CS21) | (1 << CS20)},
    {64,  (1 << CS22)},
    {128, (1 << CS22) | (1 << CS20)},
    {256, (1 << CS22) | (1 << CS21)},
    {1024,(1 << CS22) | (1 << CS21) | (1 << CS20)}
};
#define NUM_TIMER2_PRESCALERS (sizeof(timer2_prescalers) / sizeof(PrescalerConfig_t))

/***********************************************************************************************************************
* Private Variables
***********************************************************************************************************************/

// Store the calculated CS bits for each channel to be used in PWM_Start
static uint8_t pwm_prescaler_cs[4] = {0};

/***********************************************************************************************************************
* Private Functions
***********************************************************************************************************************/

/**
 * @brief Calculate the best prescaler and TOP value for a target frequency.
 *
 * @param target_freq Desired frequency.
 * @param prescalers Array of available prescaler configurations.
 * @param num_prescalers Number of configurations in the array.
 * @param max_top The maximum possible TOP value for the timer.
 * @param best_prescaler_cs Output: Best CS bit configuration.
 * @param best_top_value Output: Best calculated TOP value.
 * @return true If a suitable prescaler and TOP were found.
 * @return false If no suitable configuration could be found.
 */
static bool calculate_timer_params(
    tlong target_freq,
    const PrescalerConfig_t prescalers[],
    uint8_t num_prescalers,
    tlong max_top,
    uint8_t* best_prescaler_cs,
    tlong* best_top_value)
{
    float min_freq_diff = -1.0;
    bool found = false;

    if (target_freq == 0) {
        // Frequency 0 is invalid, cannot calculate.
        return false;
    }

    for (int i = 0; i < num_prescalers; ++i) {
        uint16_t prescaler_val = prescalers[i].prescaler_val;

        // Calculate the required TOP value for this prescaler
        float required_top_float = ((float)F_CPU / (prescaler_val * target_freq)) - 1.0;
        tlong required_top = (tlong)(required_top_float + 0.5); // Round to nearest integer

        // Check if the required TOP is within the valid range
        if (required_top <= max_top) {
            // Calculate the actual frequency this configuration would produce
            float actual_freq = (float)F_CPU / (prescaler_val * (required_top + 1.0));

            // Calculate the absolute difference from the target frequency
            float freq_diff = fabsf(target_freq - actual_freq);

            // If this is the first valid configuration found, or if it's better than the previous best
            if (!found || freq_diff < min_freq_diff) {
                min_freq_diff = freq_diff;
                *best_prescaler_cs = prescalers[i].cs_bits;
                *best_top_value = required_top;
                found = true;
            }
        }
    }

    return found;
}


/***********************************************************************************************************************
* Functions ===========================================================================
***********************************************************************************************************************/

/**
 * @brief Initializes the PWM hardware for the specified channel.
 * Configures the timer and GPIOs.
 *
 * @param TRD_Channel The PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    // Disable interrupts temporarily for 16-bit register access safety (Timer1)
    uint8_t sreg = SREG;
    cli();

    switch (TRD_Channel) {
        case TRD_Channel_OC0:
            // Configure OC0 (PB3) as output
            DDRB |= (1 << PB3); /* PDF Reference - Port B, PB3 description */
            // Set Timer0 to Fast PWM mode, TOP=0xFF (WGM01=1, WGM00=1)
            TCCR0 |= (1 << WGM00) | (1 << WGM01); /* PDF Reference - TCCR0, WGM01:0, Table 38 Mode 3 */
            // Disconnect OC0 pin (COM01=0, COM00=0) - Default state, explicit clear for safety/clarity
            TCCR0 &= ~((1 << COM00) | (1 << COM01)); /* PDF Reference - TCCR0, COM01:0, Table 39 */
            // Clear Timer0 counter
            TCNT0 = 0; /* PDF Reference - TCNT0 */
            // Clear Timer0 Output Compare Register
            OCR0 = 0; /* PDF Reference - OCR0 */
            break;

        case TRD_Channel_OC1A:
        case TRD_Channel_OC1B:
            // Configure OC1A (PD5) and OC1B (PD4) as outputs
            DDRD |= (1 << PD5) | (1 << PD4); /* PDF Reference - Port D, PD5/PD4 descriptions */
            // Set Timer1 to Fast PWM mode, TOP is ICR1 (WGM13=1, WGM12=1, WGM11=1, WGM10=0)
            TCCR1A |= (1 << WGM11); /* PDF Reference - TCCR1A/B, WGM13:0, Table 47 Mode 14 */
            TCCR1B |= (1 << WGM13) | (1 << WGM12); /* PDF Reference */
            // Disconnect OC1A/OC1B pins (COM1x1=0, COM1x0=0) - Default state, explicit clear for safety/clarity
            TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1) | (1 << COM1B0) | (1 << COM1B1)); /* PDF Reference - TCCR1A, COM1x1:0, Table 44 */
            // Clear Timer1 counter (atomic write for 16-bit register)
            TCNT1 = 0; /* PDF Reference - TCNT1 */
            // Clear Timer1 Output Compare Registers and ICR1 (atomic writes for 16-bit registers)
            OCR1A = 0; /* PDF Reference - OCR1A */
            OCR1B = 0; /* PDF Reference - OCR1B */
            ICR1 = 0; /* PDF Reference - ICR1 */
            break;

        case TRD_Channel_OC2:
            // Configure OC2 (PD7) as output
            DDRD |= (1 << PD7); /* PDF Reference - Port D, PD7 description */
            // Set Timer2 to Fast PWM mode, TOP=0xFF (WGM21=1, WGM20=1)
            TCCR2 |= (1 << WGM20) | (1 << WGM21); /* PDF Reference - TCCR2, WGM21:0, Table 50 Mode 3 */
            // Disconnect OC2 pin (COM21=0, COM20=0) - Default state, explicit clear for safety/clarity
            TCCR2 &= ~((1 << COM20) | (1 << COM21)); /* PDF Reference - TCCR2, COM21:0, Table 51 */
            // Clear Timer2 counter
            TCNT2 = 0; /* PDF Reference - TCNT2 */
            // Clear Timer2 Output Compare Register
            OCR2 = 0; /* PDF Reference - OCR2 */
            // Ensure synchronous operation if needed (AS2=0) - Default state
            ASSR &= ~(1 << AS2); /* PDF Reference - ASSR, AS2 */
            // Reset prescaler if needed - optional
            SFIOR |= (1 << PSR2); /* PDF Reference - SFIOR, PSR2 */
            break;

        default:
            // Invalid channel, do nothing
            break;
    }

    // Restore interrupts
    SREG = sreg;
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * Calculates and configures the appropriate timer registers (prescaler, TOP, OCR).
 * Does not start the PWM signal.
 *
 * @param TRD_Channel The PWM channel to configure.
 * @param frequency Desired PWM frequency in Hz.
 * @param duty Desired duty cycle in percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    uint8_t sreg = SREG; // Save SREG for atomic access
    cli(); // Disable interrupts

    // Clamp duty cycle
    if (duty > 100) duty = 100;

    uint8_t best_prescaler_cs = 0;
    tlong calculated_top = 0; // Used for Timer1 (Mode 14)

    switch (TRD_Channel) {
        case TRD_Channel_OC0:
            // Timer0 Fast PWM (Mode 3), TOP = 0xFF
            // Calculate required prescaler for target frequency
            if (calculate_timer_params(frequency, timer0_1_prescalers, NUM_TIMER0_1_PRESCALERS, T0_T2_PWM_TOP, &best_prescaler_cs, &calculated_top)) {
                // Store prescaler for PWM_Start
                pwm_prescaler_cs[TRD_Channel] = best_prescaler_cs;
                // Calculate OCR0 value based on duty cycle and fixed TOP
                // For non-inverted Fast PWM: Duty = (OCR0+1)/(TOP+1)
                // OCR0 = round(Duty * (TOP+1)) - 1
                // Simplification: OCR0 = round(Duty * TOP / 100)
                OCR0 = (tbyte)((float)duty / 100.0 * T0_T2_PWM_TOP); /* PDF Reference - OCR0 */
            } else {
                 // Could not find a suitable prescaler/frequency, default to stop
                 pwm_prescaler_cs[TRD_Channel] = 0; /* PDF Reference - TCCR0, CS02:0 */
                 OCR0 = 0; /* PDF Reference - OCR0 */
            }
            break;

        case TRD_Channel_OC1A:
        case TRD_Channel_OC1B:
            // Timer1 Fast PWM (Mode 14), TOP = ICR1
            // Calculate required prescaler and TOP value for target frequency
            if (calculate_timer_params(frequency, timer0_1_prescalers, NUM_TIMER0_1_PRESCALERS, 0xFFFF, &best_prescaler_cs, &calculated_top)) {
                 // Store prescaler for PWM_Start
                 pwm_prescaler_cs[TRD_Channel] = best_prescaler_cs;

                 // Set ICR1 (atomic write for 16-bit register)
                 ICR1 = (uint16_t)calculated_top; /* PDF Reference - ICR1 */

                 // Calculate OCR1x value based on duty cycle and calculated TOP
                 // For non-inverted Fast PWM: Duty = (OCR1x+1)/(TOP+1)
                 // OCR1x = round(Duty * (TOP+1)) - 1
                 // OCR1x = round(Duty * (ICR1+1) / 100) - 1
                 uint16_t ocr_val = (uint16_t)((float)duty / 100.0 * (calculated_top + 1.0));
                 if (ocr_val > 0) ocr_val--; // Adjust for (OCR+1) in formula

                 // Set OCR1A or OCR1B (atomic write for 16-bit registers)
                 if (TRD_Channel == TRD_Channel_OC1A) {
                     OCR1A = ocr_val; /* PDF Reference - OCR1A */
                 } else {
                     OCR1B = ocr_val; /* PDF Reference - OCR1B */
                 }
            } else {
                 // Could not find a suitable prescaler/frequency, default to stop
                 pwm_prescaler_cs[TRD_Channel] = 0; /* PDF Reference - TCCR1B, CS12:0 */
                 // Set OCR1A/B and ICR1 to 0 (atomic writes for 16-bit registers)
                 ICR1 = 0; OCR1A = 0; OCR1B = 0; /* PDF Reference - ICR1, OCR1A, OCR1B */
            }
            break;

        case TRD_Channel_OC2:
             // Timer2 Fast PWM (Mode 3), TOP = 0xFF
             // Calculate required prescaler for target frequency
             if (calculate_timer_params(frequency, timer2_prescalers, NUM_TIMER2_PRESCALERS, T0_T2_PWM_TOP, &best_prescaler_cs, &calculated_top)) {
                 // Store prescaler for PWM_Start
                 pwm_prescaler_cs[TRD_Channel] = best_prescaler_cs;
                 // Calculate OCR2 value based on duty cycle and fixed TOP
                 // OCR2 = round(Duty * TOP / 100)
                 OCR2 = (tbyte)((float)duty / 100.0 * T0_T2_PWM_TOP); /* PDF Reference - OCR2 */
             } else {
                 // Could not find a suitable prescaler/frequency, default to stop
                 pwm_prescaler_cs[TRD_Channel] = 0; /* PDF Reference - TCCR2, CS22:0 */
                 OCR2 = 0; /* PDF Reference - OCR2 */
             }
            break;

        default:
            // Invalid channel, do nothing
            break;
    }

    SREG = sreg; // Restore SREG
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 *
 * @param TRD_Channel The PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    uint8_t sreg = SREG; // Save SREG for atomic access
    cli(); // Disable interrupts

    // Get stored prescaler setting. If 0, it means Set_Freq failed or hasn't been called.
    uint8_t cs_bits = pwm_prescaler_cs[TRD_Channel];

    if (cs_bits == 0) {
        // Cannot start if frequency/prescaler wasn't set successfully
        // or if it resulted in a stopped timer (CS=0).
        // Consider re-initializing the channel if this is an error case.
        // For now, just return.
        SREG = sreg; // Restore SREG
        return;
    }

    switch (TRD_Channel) {
        case TRD_Channel_OC0:
            // Set Timer0 to Fast PWM mode, TOP=0xFF (WGM01=1, WGM00=1)
            TCCR0 |= (1 << WGM00) | (1 << WGM01); /* PDF Reference - TCCR0, WGM01:0, Table 38 Mode 3 */
            // Set COM01:0 for non-inverted Fast PWM mode (Clear OC0 on compare match, set at TOP)
            TCCR0 |= (1 << COM01); /* PDF Reference - TCCR0, COM01:0, Table 40 Mode 2 */
            TCCR0 &= ~(1 << COM00); /* PDF Reference */
            // Set prescaler and start Timer0
            TCCR0 |= cs_bits; /* PDF Reference - TCCR0, CS02:0 */
            break;

        case TRD_Channel_OC1A:
            // Set Timer1 to Fast PWM mode, TOP is ICR1 (WGM13=1, WGM12=1, WGM11=1, WGM10=0)
            TCCR1A |= (1 << WGM11); /* PDF Reference - TCCR1A/B, WGM13:0, Table 47 Mode 14 */
            TCCR1B |= (1 << WGM13) | (1 << WGM12); /* PDF Reference */
             // Set COM1A1:0 for non-inverted Fast PWM mode (Clear OC1A on compare match, set at TOP)
            TCCR1A |= (1 << COM1A1); /* PDF Reference - TCCR1A, COM1A1:0, Table 45 Mode 2 */
            TCCR1A &= ~(1 << COM1A0); /* PDF Reference */
            // Set prescaler and start Timer1
            TCCR1B |= cs_bits; /* PDF Reference - TCCR1B, CS12:0 */
            break;

        case TRD_Channel_OC1B:
            // Set Timer1 to Fast PWM mode, TOP is ICR1 (WGM13=1, WGM12=1, WGM11=1, WGM10=0)
            TCCR1A |= (1 << WGM11); /* PDF Reference - TCCR1A/B, WGM13:0, Table 47 Mode 14 */
            TCCR1B |= (1 << WGM13) | (1 << WGM12); /* PDF Reference */
            // Set COM1B1:0 for non-inverted Fast PWM mode (Clear OC1B on compare match, set at TOP)
            TCCR1A |= (1 << COM1B1); /* PDF Reference - TCCR1A, COM1B1:0, Table 45 Mode 2 */
            TCCR1A &= ~(1 << COM1B0); /* PDF Reference */
            // Set prescaler and start Timer1
            TCCR1B |= cs_bits; /* PDF Reference - TCCR1B, CS12:0 */
            break;

        case TRD_Channel_OC2:
            // Set Timer2 to Fast PWM mode, TOP=0xFF (WGM21=1, WGM20=1)
            TCCR2 |= (1 << WGM20) | (1 << WGM21); /* PDF Reference - TCCR2, WGM21:0, Table 50 Mode 3 */
            // Set COM21:0 for non-inverted Fast PWM mode (Clear OC2 on compare match, set at TOP)
            TCCR2 |= (1 << COM21); /* PDF Reference - TCCR2, COM21:0, Table 52 Mode 2 */
            TCCR2 &= ~(1 << COM20); /* PDF Reference */
            // Set prescaler and start Timer2
            TCCR2 |= cs_bits; /* PDF Reference - TCCR2, CS22:0 */
             // Ensure synchronous operation if needed (AS2=0) - Default state
            ASSR &= ~(1 << AS2); /* PDF Reference - ASSR, AS2 */
            break;

        default:
            // Invalid channel, do nothing
            break;
    }

    SREG = sreg; // Restore SREG
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * Disconnects the OC pin and stops the timer.
 *
 * @param TRD_Channel The PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
     uint8_t sreg = SREG; // Save SREG for atomic access
     cli(); // Disable interrupts

    switch (TRD_Channel) {
        case TRD_Channel_OC0:
            // Stop Timer0 (clear CS bits)
            TCCR0 &= ~((1 << CS00) | (1 << CS01) | (1 << CS02)); /* PDF Reference - TCCR0, CS02:0 */
            // Disconnect OC0 pin (COM01=0, COM00=0)
            TCCR0 &= ~((1 << COM00) | (1 << COM01)); /* PDF Reference - TCCR0, COM01:0 */
            break;

        case TRD_Channel_OC1A:
            // Stop Timer1 (clear CS bits)
            TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); /* PDF Reference - TCCR1B, CS12:0 */
            // Disconnect OC1A pin (COM1A1=0, COM1A0=0)
            TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1)); /* PDF Reference - TCCR1A, COM1A1:0 */
            break;

        case TRD_Channel_OC1B:
            // Stop Timer1 (clear CS bits)
            TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); /* PDF Reference - TCCR1B, CS12:0 */
            // Disconnect OC1B pin (COM1B1=0, COM1B0=0)
            TCCR1A &= ~((1 << COM1B0) | (1 << COM1B1)); /* PDF Reference - TCCR1A, COM1B1:0 */
            break;

        case TRD_Channel_OC2:
            // Stop Timer2 (clear CS bits)
            TCCR2 &= ~((1 << CS20) | (1 << CS21) | (1 << CS22)); /* PDF Reference - TCCR2, CS22:0 */
            // Disconnect OC2 pin (COM21=0, COM20=0)
            TCCR2 &= ~((1 << COM20) | (1 << COM21)); /* PDF Reference - TCCR2, COM21:0 */
             // Ensure synchronous operation if needed (AS2=0) - Default state
            ASSR &= ~(1 << AS2); /* PDF Reference - ASSR, AS2 */
            break;

        default:
            // Invalid channel, do nothing
            break;
    }

    SREG = sreg; // Restore SREG
}

/**
 * @brief Disables all PWM peripherals and outputs to reduce power consumption.
 * Stops all timers and disconnects all OC pins.
 */
void PWM_PowerOff(void)
{
     uint8_t sreg = SREG; // Save SREG for atomic access
     cli(); // Disable interrupts

    // Stop Timer0 (clear CS bits)
    TCCR0 &= ~((1 << CS00) | (1 << CS01) | (1 << CS02)); /* PDF Reference - TCCR0, CS02:0 */
    // Disconnect OC0 pin (COM01=0, COM00=0)
    TCCR0 &= ~((1 << COM00) | (1 << COM01)); /* PDF Reference - TCCR0, COM01:0 */

    // Stop Timer1 (clear CS bits)
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); /* PDF Reference - TCCR1B, CS12:0 */
    // Disconnect OC1A/OC1B pins (COM1x1=0, COM1x0=0)
    TCCR1A &= ~((1 << COM1A0) | (1 << COM1A1) | (1 << COM1B0) | (1 << COM1B1)); /* PDF Reference - TCCR1A, COM1x1:0 */

    // Stop Timer2 (clear CS bits)
    TCCR2 &= ~((1 << CS20) | (1 << CS21) | (1 << CS22)); /* PDF Reference - TCCR2, CS22:0 */
    // Disconnect OC2 pin (COM21=0, COM20=0)
    TCCR2 &= ~((1 << COM20) | (1 << COM21)); /* PDF Reference - TCCR2, COM21:0 */
     // Ensure synchronous operation if needed (AS2=0) - Default state
    ASSR &= ~(1 << AS2); /* PDF Reference - ASSR, AS2 */

    // Note: More power can be saved by setting DDR bits back to input,
    // but this function only targets disabling the *PWM* peripherals/outputs themselves.
    // Also, disabling timers via Power Reduction Register (PRR) is not explicitly
    // mentioned in the provided ATMEGA32 PDF context.

    SREG = sreg; // Restore SREG
}