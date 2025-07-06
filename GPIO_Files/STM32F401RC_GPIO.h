/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for GPIO peripheral driver on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-06
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* ==================== INCLUDES ==================== */
/* Include the configuration header file which provides common types like tbyte */
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/* Define GPIO usage type */
typedef enum
{
    normal_usage,      /* GPIO used for normal logic */
    communication_usage /* GPIO used for communication specific logic */
} t_usage;

/* Define GPIO direction mode */
typedef enum
{
    output, /* GPIO configured as output */
    input,  /* GPIO configured as input */
    analog  /* GPIO configured as analog */
} t_direction;

/* Define GPIO pull resistor settings for input pins */
typedef enum
{
    pull_up,   /* Enable internal pull-up resistor */
    pull_down  /* Enable internal pull-down resistor */
} t_pull;

/* Define GPIO output driving connection type */
typedef enum
{
    push_pull,  /* Output configured as push-pull */
    open_drain  /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/* Define available GPIO ports for STM32F401RC */
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D, /* PDF Reference */
    port_E, /* PDF Reference */
    port_H  /* PDF Reference - Used for OSC/CRS */
} tport;

/* Define available GPIO pins (0-15) */
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

/* Initialize a GPIO pin as output with specified configuration */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/* Initialize a GPIO pin as input with specified configuration */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/* Get the current direction configuration of a GPIO pin */
t_direction GPIO_Direction_get(tport port, tpin pin);

/* Set the output value of a GPIO pin */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/* Get the input value of a GPIO pin */
tbyte GPIO_Value_Get(tport port, tpin pin);

/* Toggle the output value of a GPIO pin */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */