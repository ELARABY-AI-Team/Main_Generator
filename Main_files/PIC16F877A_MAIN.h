#ifndef PIC16F877A_MAIN_H_
#define PIC16F877A_MAIN_H_

/*
 * File: main.h
 * Description: Minimal header file for PIC16F877A embedded projects.
 * Author: Embedded C Engineer
 * Device: PIC16F877A
 * Creation Date: 2025-06-18
 * Copyright: Your Copyright Here
 */

// 2. Necessary standard and MCU-specific includes
// For Microchip PICs using XC compilers, <xc.h> is the primary include.
// It handles device-specific headers and often includes necessary standard C headers.
#include <xc.h>

// Although <xc.h> usually pulls it in, explicitly include the device header for clarity.
// Note: The exact path might vary slightly based on compiler installation, but <pic16f877a.h>
// is the standard name relative to the compiler's include paths.
#include <pic16f877a.h>

// 3. Standard C headers (as listed)
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h>
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// 4. Useful typedefs
typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;


// 5. Core macros
#define SET_BIT(reg, bit)       ((reg) |= (1U << (bit)))
#define CLR_BIT(reg, bit)       ((reg) &= ~(1U << (bit)))
#define GET_BIT(reg, bit)       (((reg) >> (bit)) & 1U)
#define TOG_BIT(reg, bit)       ((reg) ^= (1U << (bit)))

// PIC specific core macros (Interrupts, NOP, HALT)
// Global Interrupts are controlled by the GIE bit in the INTCON register.
// Peripheral Interrupts are controlled by PEIE bit in INTCON and individual bits in PIE1/PIE2.
// Using the bit-field access provided by <xc.h> is standard practice.
#define Global_Int_Enable()     (INTCONbits.GIE = 1) // Enable Global Interrupts
#define Global_Int_Disable()    (INTCONbits.GIE = 0) // Disable Global Interrupts
#define NOP()                   __builtin_nop()      // No Operation instruction (compiler builtin)
#define HALT()                  while(1)             // Infinite loop to halt execution (common embedded "halt")


// 6. SAFEGUARD MACROS

/**
 * @brief Configures all GPIO pins to a safe default state (input).
 *        Also disables internal pull-ups on PORTB.
 */
#define GPIO_SAFEGUARD_Init() do { \
    TRISA = 0xFF; /* Set all PORTA pins as inputs */ \
    TRISB = 0xFF; /* Set all PORTB pins as inputs */ \
    TRISC = 0xFF; /* Set all PORTC pins as inputs */ \
    TRISD = 0xFF; /* Set all PORTD pins as inputs */ \
    TRISE = 0xFF; /* Set all PORTE pins as inputs */ \
    /* OPTION_REGbits.RBPU controls PORTB pull-ups. RBPU = 0 enables, RBPU = 1 disables. */ \
    OPTION_REGbits.RBPU = 1; /* Disable PORTB pull-ups */ \
} while(0)

/**
 * @brief Disables unused peripherals and sets related registers to safe defaults.
 *        Disables ADC, MSSP (SPI/I2C), USART, Timers, Comparators, VREF, CCP.
 *        Configures analog pins as digital. Disables all interrupts and clears flags.
 */
#define Registers_SAFEGUARD_Init() do { \
    /* Disable Peripherals */ \
    ADCON0bits.ADON = 0;    /* Disable ADC module */ \
    SSPCONbits.SSPEN = 0;   /* Disable MSSP module (SPI/I2C) */ \
    RCSTAbits.SPEN = 0;     /* Disable USART module */ \
    T1CONbits.TMR1ON = 0;   /* Disable Timer1 module */ \
    T2CONbits.TMR2ON = 0;   /* Disable Timer2 module */ \
    /* CMCON configures Comparators. CM2:CM0 bits (CMCON<2:0>). 111 = Comparators Off. */ \
    CMCON = 0x07;           /* Disable Comparators */ \
    VREFCONbits.VREFEN = 0; /* Disable Voltage Reference module */ \
    /* CCPRxCON configures CCP/ECCP modules. CCPxM bits (CCPRxCON<3:0>). 0000 = Disabled. */ \
    CCPR1CON = 0x00;        /* Disable ECCP1 module */ \
    CCPR2CON = 0x00;        /* Disable CCP2 module */ \
    /* Timer0 module's counting is controlled by OPTION_REG bits T0CS and PSA. \
       There is no single ON/OFF bit. The default POR value of OPTION_REG (0xFF) \
       sets T0CS=1 (External clock) and PSA=1 (Prescaler assigned to WDT), \
       effectively stopping Timer0 counting from the internal clock. \
       Disabling its interrupt in INTCON is sufficient if TMR0 is unused. */ \
    \
    /* ADCON1: Configure analog pins as digital inputs if not using ADC. \
       PCFG3:PCFG0 bits (ADCON1<3:0>) configure pin functions. 1111 = AN0-AN4 digital I/O. */ \
    ADCON1 = 0x0F; /* Configure AN0-AN4 as digital I/O */ \
    \
    /* Disable Interrupts */ \
    /* INTCON contains GIE, PEIE, T0IE, INTE, RBIE. Clear all. */ \
    INTCON = 0x00; /* Disable Global, Peripheral, TMR0, INT, PORTB Change Interrupts */ \
    /* PIE1 contains TMR1IE, TMR2IE, CCPIE1, SSPIE, TXIE, RCIE, ADIE. Clear all. */ \
    PIE1 = 0x00;   /* Disable Timer1, Timer2, CCP1, MSSP, TX, RX, ADC Interrupts */ \
    /* PIE2 contains CCP2IE, PSPIE, EEIE, CMIE. Clear all. */ \
    PIE2 = 0x00;   /* Disable CCP2, PSP, EEPROM, Comparator Interrupts */ \
    /* Clear any pending interrupt flags in PIR1/PIR2 to avoid spurious interrupts later. */ \
    PIR1 = 0x00;   /* Clear Peripheral Interrupt Request Flags in PIR1 */ \
    PIR2 = 0x00;   /* Clear Peripheral Interrupt Request Flags in PIR2 */ \
} while(0)


// Add function prototypes here if needed (optional, but common in main.h)
// These functions would typically be implemented in a corresponding .c file (e.g., system.c)
// and called from main().
// e.g., void System_Init(void);    // Basic system initialization (calls safeguards, clock config, etc.)
// e.g., void Peripheral_Init(void); // Example function to initialize used peripherals


#endif // MAIN_H