/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : PWM implementation for ATMEGA32
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-20
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include <avr/io.h>   // Required for ATMEGA32 register definitions (e.g., TCCR0, OCR0, DDRB, PORTB etc.)
#include <stddef.h>   // Required for size_t
#include <stdlib.h>   // Required for labs (long absolute value)

// Define tlong and tbyte as per the function signatures if not provided by pwm.h
#ifndef tlong
#define tlong unsigned long
#endif
#ifndef tbyte
#define tbyte unsigned char
#endif

// The TRD_Channel_t enum is assumed to be defined in pwm.h as per the problem description.
// Example of how it might be defined in pwm.h:
/*
typedef enum {
    TRD_CHANNEL_0,  // Maps to Timer0 / OC0 (PB3)
    TRD_CHANNEL_1A, // Maps to Timer1 / OC1A (PD5)
    TRD_CHANNEL_1B, // Maps to Timer1 / OC1B (PD4)
    TRD_CHANNEL_2,  // Maps to Timer2 / OC2 (PD7)
    NUM_TRD_CHANNELS // Sentinel for number of channels, must be last
} TRD_Channel_t;
*/

// Internal definitions for timer types and output compare units
typedef enum {
    TIMER_TYPE_8BIT_T0,
    TIMER_TYPE_16BIT_T1,
    TIMER_TYPE_8BIT_T2
} TimerType_t;

typedef enum {
    OC_UNIT_NONE, // Placeholder, not used for specific OC pins
    OC_UNIT_0,    // For Timer0: OCR0
    OC_UNIT_1A,   // For Timer1: OCR1A
    OC_UNIT_1B,   // For Timer1: OCR1B
    OC_UNIT_2     // For Timer2: OCR2
} OCUnit_t;

// PWM_Channel_Config_t structure definition
// Contains configuration details for a logical PWM channel on ATMEGA32
typedef struct {
    TimerType_t timer_type;    // Type of the timer (8-bit T0/T2, 16-bit T1)
    OCUnit_t oc_unit;          // Specific Output Compare unit (OC0, OC1A, OC1B, OC2)
    volatile uint8_t *ddr_reg; // Pointer to the DDR register for the associated port (e.g., &DDRB, &DDRD)
    uint8_t pin_mask;          // Bit mask for the associated GPIO pin (e.g., (1 << PB3))
} PWM_Channel_Config_t;

// pwm_channel_map array of all valid PWM-capable channels on ATMEGA32
// This array maps TRD_Channel_t enum values to specific hardware configurations.
static const PWM_Channel_Config_t pwm_channel_map[] = {
    // TRD_CHANNEL_0: Timer0, OC0 (PB3)
    [TRD_CHANNEL_0] = {
        .timer_type = TIMER_TYPE_8BIT_T0,
        .oc_unit = OC_UNIT_0,
        .ddr_reg = &DDRB,
        .pin_mask = (1 << PB3)
    },
    // TRD_CHANNEL_1A: Timer1, OC1A (PD5)
    [TRD_CHANNEL_1A] = {
        .timer_type = TIMER_TYPE_16BIT_T1,
        .oc_unit = OC_UNIT_1A,
        .ddr_reg = &DDRD,
        .pin_mask = (1 << PD5)
    },
    // TRD_CHANNEL_1B: Timer1, OC1B (PD4)
    [TRD_CHANNEL_1B] = {
        .timer_type = TIMER_TYPE_16BIT_T1,
        .oc_unit = OC_UNIT_1B,
        .ddr_reg = &DDRD,
        .pin_mask = (1 << PD4)
    },
    // TRD_CHANNEL_2: Timer2, OC2 (PD7)
    [TRD_CHANNEL_2] = {
        .timer_type = TIMER_TYPE_8BIT_T2,
        .oc_unit = OC_UNIT_2,
        .ddr_reg = &DDRD,
        .pin_mask = (1 << PD7)
    }
    // Add other channels here if they are defined in TRD_Channel_t and have PWM capabilities
};

// Static flags to track if a timer has been initialized to avoid re-configuring common timer settings
static unsigned char timer0_initialized = 0;
static unsigned char timer1_initialized = 0;
static unsigned char timer2_initialized = 0;

// Static variables to store the last calculated prescaler settings for each timer.
// These are used by PWM_Start to enable the timer with the last configured clock source.
static uint8_t timer0_prescaler_setting = 0;
static uint8_t timer1_prescaler_setting = 0;
static uint8_t timer2_prescaler_setting = 0;

// Default CPU Frequency. Can be overridden by compiler flags (-DF_CPU=...)
#ifndef F_CPU
#define F_CPU 16000000UL // Assuming a 16 MHz clock
#endif

// Prescaler values and their corresponding CS bits for Timer0 and Timer1
static const uint32_t T0_T1_PRESCALER_VALS[] = {1, 8, 64, 256, 1024};
static const uint8_t T0_T1_CS_BITS[] = {
    (1 << CS00),                        // No prescaling
    (1 << CS01),                        // clk/8
    (1 << CS01) | (1 << CS00),          // clk/64
    (1 << CS02),                        // clk/256
    (1 << CS02) | (1 << CS00)           // clk/1024
};

// Prescaler values and their corresponding CS bits for Timer2
static const uint32_t T2_PRESCALER_VALS[] = {1, 8, 32, 64, 128, 256, 1024};
static const uint8_t T2_CS_BITS[] = {
    (1 << CS20),                        // No prescaling
    (1 << CS21),                        // clk/8
    (1 << CS21) | (1 << CS20),          // clk/32
    (1 << CS22),                        // clk/64
    (1 << CS22) | (1 << CS20),          // clk/128
    (1 << CS22) | (1 << CS21),          // clk/256
    (1 << CS22) | (1 << CS21) | (1 << CS20) // clk/1024
};

/**
 * @brief Helper function to find the best prescaler settings for 8-bit timers (T0/T2).
 *        For 8-bit Fast PWM mode (WGMx0=1, WGMx1=1), TOP is fixed at 0xFF (255).
 *        This function identifies the prescaler that results in the closest frequency to the target.
 * @param target_freq The desired PWM frequency in Hz.
 * @param timer_type The type of timer (TIMER_TYPE_8BIT_T0 or TIMER_TYPE_8BIT_T2).
 * @return The clock select (CS) bits for the selected prescaler. Returns 0 if no suitable prescaler is found.
 */
static uint8_t find_8bit_prescaler(tlong target_freq, TimerType_t timer_type) {
    const uint32_t *prescaler_values;
    const uint8_t *cs_bits;
    size_t num_prescalers;

    if (timer_type == TIMER_TYPE_8BIT_T0) {
        prescaler_values = T0_T1_PRESCALER_VALS;
        cs_bits = T0_T1_CS_BITS;
        num_prescalers = sizeof(T0_T1_PRESCALER_VALS) / sizeof(T0_T1_PRESCALER_VALS[0]);
    } else { // TIMER_TYPE_8BIT_T2
        prescaler_values = T2_PRESCALER_VALS;
        cs_bits = T2_CS_BITS;
        num_prescalers = sizeof(T2_PRESCALER_VALS) / sizeof(T2_PRESCALER_VALS[0]);
    }

    uint8_t best_cs = 0; // Default to no clock source (stopped)
    tlong min_freq_diff = (tlong)-1; // Initialize with a very large number for minimum difference

    for (size_t i = 0; i < num_prescalers; i++) {
        tlong current_prescaler = prescaler_values[i];
        if (current_prescaler == 0) continue; // Should not happen with defined prescalers

        // Calculate actual frequency for fixed TOP = 255 (0xFF + 1)
        tlong actual_freq = F_CPU / (current_prescaler * 256UL);

        tlong freq_diff = labs((long)actual_freq - (long)target_freq);

        // Find the prescaler that results in the closest frequency
        if (freq_diff < min_freq_diff) {
            min_freq_diff = freq_diff;
            best_cs = cs_bits[i];
        }
    }
    return best_cs;
}

/**
 * @brief Helper function to find the best prescaler and TOP value for 16-bit Timer1.
 *        For 16-bit Fast PWM (Mode 14), TOP is set by the ICR1 register.
 * @param target_freq The desired PWM frequency in Hz.
 * @param calculated_top Pointer to store the calculated TOP value (ICR1).
 * @return The clock select (CS) bits for the selected prescaler. Returns 0 if no suitable prescaler/TOP is found.
 */
static uint8_t find_16bit_prescaler_and_top(tlong target_freq, tlong *calculated_top) {
    const uint32_t *prescaler_values = T0_T1_PRESCALER_VALS;
    const uint8_t *cs_bits = T0_T1_CS_BITS;
    size_t num_prescalers = sizeof(T0_T1_PRESCALER_VALS) / sizeof(T0_T1_PRESCALER_VALS[0]);

    uint8_t best_cs = 0; // Default to no clock source (stopped)
    tlong best_top = 0;
    tlong min_freq_diff = (tlong)-1; // Initialize with a very large number

    for (size_t i = 0; i < num_prescalers; i++) {
        tlong current_prescaler = prescaler_values[i];
        if (current_prescaler == 0) continue; // Should not happen

        tlong timer_clock = F_CPU / current_prescaler;
        tlong top_candidate = (timer_clock / target_freq); // (TOP + 1) = F_CPU / (Prescaler * target_freq)
        if (top_candidate > 0) top_candidate--; // TOP = (F_CPU / (Prescaler * target_freq)) - 1

        // Check if TOP fits within the 16-bit range and is valid (not 0, which would mean max frequency)
        if (top_candidate >= 1 && top_candidate <= 65535UL) {
            // Calculate actual frequency for this TOP candidate
            tlong actual_freq = timer_clock / (top_candidate + 1);
            tlong freq_diff = labs((long)actual_freq - (long)target_freq);

            // Find the prescaler/TOP combination that results in the closest frequency
            if (freq_diff < min_freq_diff) {
                min_freq_diff = freq_diff;
                best_top = top_candidate;
                best_cs = cs_bits[i];
            }
        }
    }
    *calculated_top = best_top;
    return best_cs;
}


/**
 * @brief Initializes a specific PWM channel.
 *        Configures the associated timer for Fast PWM mode and sets up the output pin.
 *        For timers with multiple channels (like Timer1), common timer settings
 *        are configured only once upon the first initialization of a channel on that timer.
 * @param TRD_Channel The logical PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel) {
    // Validate the input channel
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])) {
        // Invalid channel index
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Set the GPIO pin as output
    *config->ddr_reg |= config->pin_mask;

    // Configure the timer based on its type
    switch (config->timer_type) {
        case TIMER_TYPE_8BIT_T0:
            if (!timer0_initialized) {
                // Stop the timer, clear all TCCR0 bits
                TCCR0 = 0;
                // Reset counter value
                TCNT0 = 0;
                // Configure Timer0 for Fast PWM mode (Mode 3), TOP=0xFF
                TCCR0 |= (1 << WGM00) | (1 << WGM01);
                // Configure OC0 for non-inverting PWM: Clear OC0 on compare match, Set OC0 at TOP
                TCCR0 |= (1 << COM01);
                timer0_initialized = 1;
            }
            // Set initial duty cycle to 0 (off)
            OCR0 = 0;
            break;

        case TIMER_TYPE_16BIT_T1:
            if (!timer1_initialized) {
                // Stop Timer1, clear all TCCR1A and TCCR1B bits
                TCCR1A = 0;
                TCCR1B = 0;
                // Reset counter value
                TCNT1 = 0; // Accessing TCNT1 (16-bit) clears both TCNT1H and TCNT1L
                // Configure Timer1 for Fast PWM mode (Mode 14), TOP=ICR1
                TCCR1A |= (1 << WGM11);
                TCCR1B |= (1 << WGM13) | (1 << WGM12);
                // Set initial TOP value to maximum (65535) for lowest initial frequency, allowing full range
                ICR1 = 0xFFFF;
                timer1_initialized = 1;
            }
            // Configure OC1A or OC1B for non-inverting PWM
            // (Clear OC1x on compare match, Set OC1x at TOP)
            if (config->oc_unit == OC_UNIT_1A) {
                TCCR1A |= (1 << COM1A1);
                OCR1A = 0; // Set initial duty to 0
            } else if (config->oc_unit == OC_UNIT_1B) {
                TCCR1A |= (1 << COM1B1);
                OCR1B = 0; // Set initial duty to 0
            }
            break;

        case TIMER_TYPE_8BIT_T2:
            if (!timer2_initialized) {
                // Stop Timer2, clear all TCCR2 bits
                TCCR2 = 0;
                // Reset counter value
                TCNT2 = 0;
                // Configure Timer2 for Fast PWM mode (Mode 3), TOP=0xFF
                TCCR2 |= (1 << WGM20) | (1 << WGM21);
                // Configure OC2 for non-inverting PWM: Clear OC2 on compare match, Set OC2 at TOP
                TCCR2 |= (1 << COM21);
                timer2_initialized = 1;
            }
            // Set initial duty cycle to 0 (off)
            OCR2 = 0;
            break;

        default:
            // This case should ideally not be reached with valid TRD_Channel_t definitions.
            break;
    }
}

/**
 * @brief Sets the frequency and duty cycle for a specific PWM channel.
 *        This function calculates the necessary prescaler and TOP (for Timer1)
 *        values for the desired frequency, and updates the OCR value for the duty cycle.
 *        The calculated prescaler setting is stored for use by PWM_Start.
 * @param TRD_Channel The logical PWM channel to configure.
 * @param frequency The desired PWM frequency in Hz. Must be > 0.
 * @param duty The desired duty cycle as a percentage (0-100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty) {
    // Validate input parameters
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0]) || frequency == 0 || duty > 100) {
        // Invalid channel, frequency cannot be zero, duty must be 0-100.
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    tlong actual_top = 0;
    uint8_t current_prescaler_cs_bits = 0;
    tlong ocr_calculated_value = 0;

    switch (config->timer_type) {
        case TIMER_TYPE_8BIT_T0:
            current_prescaler_cs_bits = find_8bit_prescaler(frequency, TIMER_TYPE_8BIT_T0);
            actual_top = 255UL; // Fixed TOP for 8-bit Fast PWM (0xFF)
            timer0_prescaler_setting = current_prescaler_cs_bits; // Store for PWM_Start

            // Calculate OCR value
            ocr_calculated_value = (actual_top * duty) / 100UL;
            if (ocr_calculated_value > actual_top) ocr_calculated_value = actual_top; // Cap at TOP
            OCR0 = (uint8_t)ocr_calculated_value;
            break;

        case TIMER_TYPE_16BIT_T1:
            current_prescaler_cs_bits = find_16bit_prescaler_and_top(frequency, &actual_top);
            timer1_prescaler_setting = current_prescaler_cs_bits; // Store for PWM_Start
            
            // Set TOP value for Timer1 using ICR1
            ICR1 = (uint16_t)actual_top;

            // Calculate OCR value
            ocr_calculated_value = (actual_top * duty) / 100UL;
            if (ocr_calculated_value > actual_top) ocr_calculated_value = actual_top; // Cap at TOP

            if (config->oc_unit == OC_UNIT_1A) {
                OCR1A = (uint16_t)ocr_calculated_value;
            } else if (config->oc_unit == OC_UNIT_1B) {
                OCR1B = (uint16_t)ocr_calculated_value;
            }
            break;

        case TIMER_TYPE_8BIT_T2:
            current_prescaler_cs_bits = find_8bit_prescaler(frequency, TIMER_TYPE_8BIT_T2);
            actual_top = 255UL; // Fixed TOP for 8-bit Fast PWM (0xFF)
            timer2_prescaler_setting = current_prescaler_cs_bits; // Store for PWM_Start
            
            // Calculate OCR value
            ocr_calculated_value = (actual_top * duty) / 100UL;
            if (ocr_calculated_value > actual_top) ocr_calculated_value = actual_top; // Cap at TOP
            OCR2 = (uint8_t)ocr_calculated_value;
            break;

        default:
            break;
    }
}

/**
 * @brief Starts the PWM generation for a specific channel.
 *        This is achieved by applying the stored prescaler setting for the associated timer,
 *        which enables the timer clock.
 * @param TRD_Channel The logical PWM channel to start.
 */
void PWM_Start(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])) {
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Enable the timer by setting the clock select bits based on the last calculated setting.
    // Ensure WGM (Waveform Generation Mode) and COM (Compare Output Mode) bits are preserved.
    switch (config->timer_type) {
        case TIMER_TYPE_8BIT_T0:
            // Clear current clock select bits, then set with the stored prescaler setting
            TCCR0 = (TCCR0 & ~((1 << CS02) | (1 << CS01) | (1 << CS00))) | timer0_prescaler_setting;
            break;
        case TIMER_TYPE_16BIT_T1:
            // Clear current clock select bits, then set with the stored prescaler setting
            TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | timer1_prescaler_setting;
            break;
        case TIMER_TYPE_8BIT_T2:
            // Clear current clock select bits, then set with the stored prescaler setting
            TCCR2 = (TCCR2 & ~((1 << CS22) | (1 << CS21) | (1 << CS20))) | timer2_prescaler_setting;
            break;
        default:
            break;
    }
}

/**
 * @brief Stops the PWM generation for a specific channel.
 *        This is achieved by clearing the clock select bits of the associated timer,
 *        which effectively stops the timer counter.
 *        Note: If multiple channels are active on the same timer (e.g., OC1A and OC1B),
 *        stopping one channel will stop the entire timer, affecting all channels on it.
 * @param TRD_Channel The logical PWM channel to stop.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel) {
    if (TRD_Channel >= sizeof(pwm_channel_map) / sizeof(pwm_channel_map[0])) {
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];

    // Disable the timer by clearing the clock select bits
    switch (config->timer_type) {
        case TIMER_TYPE_8BIT_T0:
            TCCR0 &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
            break;
        case TIMER_TYPE_16BIT_T1:
            TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
            break;
        case TIMER_TYPE_8BIT_T2:
            TCCR2 &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
            break;
        default:
            break;
    }
}

/**
 * @brief Turns off all PWM timers (Timer0, Timer1, Timer2) and sets their associated
 *        output pins to input mode (high-impedance) for power saving.
 */
void PWM_PowerOff(void) {
    // Stop Timer0: Clear all TCCR0 bits to disable clock source and reset mode.
    TCCR0 = 0;
    timer0_initialized = 0; // Reset initialization flag

    // Stop Timer1: Clear all TCCR1A and TCCR1B bits to disable clock source and reset mode.
    TCCR1A = 0;
    TCCR1B = 0;
    timer1_initialized = 0; // Reset initialization flag

    // Stop Timer2: Clear all TCCR2 bits to disable clock source and reset mode.
    TCCR2 = 0;
    timer2_initialized = 0; // Reset initialization flag

    // Set PWM output pins to input mode (high-impedance) to save power.
    // Also, ensure the corresponding PORT bit is low to prevent pull-up before going high-Z.

    // OC0 (PB3)
    DDRB &= ~(1 << PB3);
    PORTB &= ~(1 << PB3); 

    // OC1A (PD5)
    DDRD &= ~(1 << PD5);
    PORTD &= ~(1 << PD5);

    // OC1B (PD4)
    DDRD &= ~(1 << PD4);
    PORTD &= ~(1 << PD4);

    // OC2 (PD7)
    DDRD &= ~(1 << PD7);
    PORTD &= ~(1 << PD7);
}