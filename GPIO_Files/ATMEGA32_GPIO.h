/***********************************************************************************************************************
* File Name      : GPIO.h
* Description    : ATMEGA32 GPIO Driver Header File
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

#include "ATMEGA32_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/* Define enumeration for GPIO usage type */
typedef enum
{
    normal_usage,       /* Define GPIO for general purpose logic */
    communication_usage /* Define GPIO for communication-specific logic */
} t_usage;

/* Define enumeration for GPIO direction mode */
typedef enum
{
    output, /* Define GPIO as output */
    input,  /* Define GPIO as input */
    analog  /* Define GPIO as analog */
} t_direction;

/* Define enumeration for input pin pull resistor settings */
typedef enum
{
    pull_up,   /* Define input pin with internal pull-up resistor enabled */
    pull_down  /* Define input pin with internal pull-down resistor enabled */
} t_pull;

/* Define enumeration for output pin driving connection type */
typedef enum
{
    push_pull,  /* Define output pin with push-pull driving */
    open_drain  /* Define output pin with open-drain driving */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/* Define enumeration for microcontroller ports */
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D  /* PDF Reference */
} tport;

/* Define enumeration for port pins */
typedef enum
{
    pin_0, /* PDF Reference */
    pin_1, /* PDF Reference */
    pin_2, /* PDF Reference */
    pin_3, /* PDF Reference */
    pin_4, /* PDF Reference */
    pin_5, /* PDF Reference */
    pin_6, /* PDF Reference */
    pin_7  /* PDF Reference */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/* Initialize a GPIO pin as output */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/* Initialize a GPIO pin as input */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/* Get the direction mode of a GPIO pin */
t_direction GPIO_Direction_get(tport port, tpin pin);

/* Set the output value of a GPIO pin */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/* Get the input value of a GPIO pin */
tbyte GPIO_Value_Get(tport port, tpin pin);

/* Toggle the output value of a GPIO pin */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* ATMEGA32_GPIO_H_ */