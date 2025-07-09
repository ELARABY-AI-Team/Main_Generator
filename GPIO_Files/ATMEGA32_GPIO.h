/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : Header file for ATMEGA32 GPIO driver
* Author         : Technology Inovation Software Team (Generated based on requirements)
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

// Include necessary configuration file for types (tbyte)
// Assumes ATMEGA32_Config.h includes definitions from ATMEGA32_MAIN.h if required
#include "ATMEGA32_Config.h"


/* ==================== REQUIRED ENUMS ==================== */

// Purpose: Define whether the GPIO is used for general or communication-specific logic.
typedef enum
{
    normal_usage = 0u, // Standard GPIO usage
    communication_usage  // Usage related to communication peripherals (e.g., SPI, I2C, USART)
} t_usage;

// Purpose: Define the direction mode of the GPIO pin.
typedef enum
{
    output = 0u, // Pin configured as output
    input,       // Pin configured as input
    analog       // Pin configured for analog input (ADC)
} t_direction;

// Purpose: Define pull resistor settings for input pins.
typedef enum
{
    pull_up = 0u, // Internal pull-up resistor enabled
    pull_down     // Internal pull-down resistor enabled (ATMEGA32 hardware supports pull-up only)
} t_pull;

// Purpose: Define the output driving connection type.
typedef enum
{
    push_pull = 0u, // Standard output driver
    open_drain      // Open-drain output (typically simulated on ATMEGA32 by setting as input)
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

// Available ports for the ATMEGA32 microcontroller.
typedef enum
{
    port_A = 0u, // PDF Reference
    port_B,      // PDF Reference
    port_C,      // PDF Reference
    port_D       // PDF Reference
} tport;

// Available pins for ATMEGA32 ports (Pins 0 through 7).
typedef enum
{
    pin_0 = 0u, // PDF Reference
    pin_1,      // PDF Reference
    pin_2,      // PDF Reference
    pin_3,      // PDF Reference
    pin_4,      // PDF Reference
    pin_5,      // PDF Reference
    pin_6,      // PDF Reference
    pin_7       // PDF Reference
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

// Initialize a specific GPIO pin for output mode with initial value, usage, and connection type.
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

// Initialize a specific GPIO pin for input mode with usage and pull resistor configuration.
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

// Get the current direction configuration of a specific GPIO pin.
// Returns: t_direction enum value indicating output, input, or analog.
t_direction GPIO_Direction_get(tport port, tpin pin);

// Set the output value of a specific GPIO pin configured as output.
// value: 0 for low, non-zero for high.
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

// Get the input value of a specific GPIO pin configured as input.
// Returns: 0 if pin is low, 1 if pin is high.
tbyte GPIO_Value_Get(tport port, tpin pin);

// Toggle the output value of a specific GPIO pin configured as output.
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* ATMEGA32_GPIO_H_ */