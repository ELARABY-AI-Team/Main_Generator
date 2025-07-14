/***********************************************************************************************************************
* File Name      : let user decide
* Description    : let user decide
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-14
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_
/* Include required configuration header */
#include "STM32F401RC_Config.h"

/* Assume tbyte and tword are defined in or included by STM32F401RC_Config.h */

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */
/* Purpose: Define whether the GPIO is used for general or communication-specific logic. */
typedef enum
{
    normal_usage,      /* Normal general purpose usage */
    communication_usage/* Usage for communication protocols */
} t_usage;

/* Purpose: Define the direction mode of the GPIO pin. */
typedef enum
{
    output, /* Pin is configured as output */
    input,  /* Pin is configured as input */
    analog  /* Pin is configured as analog */
} t_direction;

/* Purpose: Define pull resistor settings for input pins. */
/* Note: No_pull state is not included as per strict requirements */
typedef enum
{
    pull_up,  /* Enable internal pull-up resistor */
    pull_down /* Enable internal pull-down resistor */
} t_pull;

/* Purpose: Define the output driving connection type. */
typedef enum
{
    push_pull,  /* Output is configured as push-pull */
    open_drain  /* Output is configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */
/* Define available GPIO ports */
typedef enum
{
    port_A, /* PDF Reference */
    port_B, /* PDF Reference */
    port_C, /* PDF Reference */
    port_D, /* PDF Reference */
    port_E, /* PDF Reference */
    port_H  /* PDF Reference */
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
/* Initialize a GPIO pin as output */
/* Parameters: port - the GPIO port, pin - the pin number */
/*             value - initial output value (0 for low, 1 for high) */
/*             usage - usage type (normal or communication) */
/*             conn - output connection type (push-pull or open-drain) */
/* Return: None */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/* Initialize a GPIO pin as input */
/* Parameters: port - the GPIO port, pin - the pin number */
/*             usage - usage type (normal or communication) */
/*             pull - pull resistor configuration (pull-up or pull-down) */
/* Return: None */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/* Get the current direction configuration of a GPIO pin */
/* Parameters: port - the GPIO port, pin - the pin number */
/* Return: The current direction of the pin (output, input, or analog) */
t_direction GPIO_Direction_get(tport port, tpin pin);

/* Set the output value of a GPIO pin */
/* Parameters: port - the GPIO port, pin - the pin number */
/*             value - the value to set (0 for low, 1 for high) */
/* Return: None */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/* Get the input value of a GPIO pin */
/* Parameters: port - the GPIO port, pin - the pin number */
/* Return: The current input value of the pin (0 for low, 1 for high) */
tbyte GPIO_Value_Get(tport port, tpin pin);

/* Toggle the output value of a GPIO pin */
/* Parameters: port - the GPIO port, pin - the pin number */
/* Return: None */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */