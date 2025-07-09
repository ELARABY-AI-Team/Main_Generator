/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for GPIO driver on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* Include required configuration file for base types (tbyte, tword). */
/* Assumes STM32F401RC_Config.h provides definitions for tbyte, tword, etc. */
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/* Define whether the GPIO is used for general or communication-specific logic. */
typedef enum
{
  normal_usage,          /* General purpose usage */
  communication_usage    /* Usage related to communication protocols (e.g., SPI, I2C) */
} t_usage;

/* Define the direction mode of the GPIO pin. */
typedef enum
{
  output,                /* Pin configured as output */
  input,                 /* Pin configured as input */
  analog                 /* Pin configured as analog */
} t_direction;

/* Define pull resistor settings for input pins. */
typedef enum
{
  pull_up,               /* Pin configured with pull-up resistor */
  pull_down              /* Pin configured with pull-down resistor */
} t_pull;

/* Define the output driving connection type. */
typedef enum
{
  push_pull,             /* Pin configured as push-pull output */
  open_drain             /* Pin configured as open-drain output */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/* Available GPIO ports for STM32F401RC microcontroller. */
typedef enum
{
  port_A, /* PDF Reference */
  port_B, /* PDF Reference */
  port_C, /* PDF Reference */
  port_D, /* PDF Reference */
  port_E, /* PDF Reference */
  port_H  /* PDF Reference */
} tport;

/* Available GPIO pins (0 to 15) for each port. */
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

/*
 * @brief Initializes a specific GPIO pin as an output.
 *
 * @param port  : The GPIO port (e.g., port_A).
 * @param pin   : The pin number within the port (e.g., pin_5).
 * @param value : The initial output value (0 for low, 1 for high).
 * @param usage : The intended usage (normal_usage or communication_usage).
 * @param conn  : The output connection type (push_pull or open_drain).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a specific GPIO pin as an input.
 *
 * @param port  : The GPIO port (e.g., port_A).
 * @param pin   : The pin number within the port (e.g., pin_5).
 * @param usage : The intended usage (normal_usage or communication_usage).
 * @param pull  : The pull resistor configuration (pull_up or pull_down).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction mode of a specific GPIO pin.
 *
 * @param port : The GPIO port (e.g., port_A).
 * @param pin  : The pin number within the port (e.g., pin_5).
 *
 * @return The direction mode of the pin (output, input, or analog).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a specific GPIO pin.
 *        This function is only applicable to pins configured as output.
 *
 * @param port  : The GPIO port (e.g., port_A).
 * @param pin   : The pin number within the port (e.g., pin_5).
 * @param value : The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a specific GPIO pin.
 *        This function is primarily applicable to pins configured as input.
 *
 * @param port : The GPIO port (e.g., port_A).
 * @param pin  : The pin number within the port (e.g., pin_5).
 *
 * @return The input value of the pin (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a specific GPIO pin.
 *        This function is only applicable to pins configured as output.
 *
 * @param port : The GPIO port (e.g., port_A).
 * @param pin  : The pin number within the port (e.g., pin_5).
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */