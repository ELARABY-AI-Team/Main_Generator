/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : Header file for the GPIO driver on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h" /* Assumes tbyte, tword, etc. are defined here, potentially including STM32F401RC_MAIN.h */


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/* Define whether the GPIO is used for general or communication-specific logic. */
typedef enum
{
  normal_usage,       /* General purpose usage */
  communication_usage /* Usage for communication peripherals */
} t_usage;

/* Define the direction mode of the GPIO pin. */
typedef enum
{
  output, /* Output mode */
  input,  /* Input mode */
  analog  /* Analog mode */
} t_direction;

/* Define pull resistor settings for input pins. */
typedef enum
{
  pull_up,   /* Pull-up resistor enabled */
  pull_down  /* Pull-down resistor enabled */
} t_pull;

/* Define the output driving connection type. */
typedef enum
{
  push_pull,  /* Push-pull output configuration */
  open_drain  /* Open-drain output configuration */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/* Available GPIO ports on STM32F401RC (64-pin package). */
typedef enum
{
  port_A, /* PDF Reference */
  port_B, /* PDF Reference */
  port_C, /* PDF Reference */
  port_D, /* PDF Reference */
  port_H  /* PDF Reference */
} tport;

/* Available GPIO pins (0-15). */
typedef enum
{
  pin_0,  /* PDF Reference */
  pin_1,  /* PDF Reference */
  pin_2,  /* PDF Reference */
  pin_3,  /* PDF Reference */
  pin_4,  /* PDF Reference */
  pin_5,  /* PDF Reference */
  pin_6,  /* PDF Reference */
  pin_7,  /* PDF Reference */
  pin_8,  /* PDF Reference */
  pin_9,  /* PDF Reference */
  pin_10, /* PDF Reference */
  pin_11, /* PDF Reference */
  pin_12, /* PDF Reference */
  pin_13, /* PDF Reference */
  pin_14, /* PDF Reference */
  pin_15  /* PDF Reference */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/* Initializes a GPIO pin for output. */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/* Initializes a GPIO pin for input. */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/* Gets the current direction mode of a GPIO pin. */
t_direction GPIO_Direction_get(tport port, tpin pin);

/* Sets the output value of a GPIO pin configured as output. */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/* Gets the input value of a GPIO pin configured as input. */
tbyte GPIO_Value_Get(tport port, tpin pin);

/* Toggles the output value of a GPIO pin configured as output. */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */