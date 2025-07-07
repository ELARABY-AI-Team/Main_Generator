/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for GPIO Peripheral Driver
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-07
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

/* Include required configuration and base types */
/* Assuming STM32F401RC_Config.h defines tbyte and tword */
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */

/*
 * @brief Defines the usage context of the GPIO pin.
 */
typedef enum
{
    normal_usage,           /* GPIO used for standard digital input/output */
    communication_usage     /* GPIO used for communication protocols (e.g., SPI, I2C) */
} t_usage;


/*
 * @brief Defines the direction mode of the GPIO pin.
 */
typedef enum
{
    output,                 /* Pin configured as a digital output */
    input,                  /* Pin configured as a digital input */
    analog                  /* Pin configured as analog input/output */
} t_direction;


/*
 * @brief Defines pull resistor settings for input pins.
 */
typedef enum
{
    pull_up,                /* Enable internal pull-up resistor */
    pull_down               /* Enable internal pull-down resistor */
} t_pull;


/*
 * @brief Defines the output driving connection type.
 */
typedef enum
{
    push_pull,              /* Output configured as push-pull */
    open_drain              /* Output configured as open-drain */
} t_output_conn;


/* ==================== HARDWARE DEFINITIONS ==================== */

/*
 * @brief Defines the available GPIO ports on STM32F401RC.
 */
typedef enum
{
    port_A,                 /* GPIO Port A /* PDF Reference */ */
    port_B,                 /* GPIO Port B /* PDF Reference */ */
    port_C,                 /* GPIO Port C /* PDF Reference */ */
    port_D,                 /* GPIO Port D /* PDF Reference */ */
    port_E,                 /* GPIO Port E /* PDF Reference */ */
    port_H                  /* GPIO Port H /* PDF Reference */ */
} tport;


/*
 * @brief Defines the available pins within a GPIO port (0-15).
 */
typedef enum
{
    pin_0,                  /* Pin 0 /* PDF Reference */ */
    pin_1,                  /* Pin 1 /* PDF Reference */ */
    pin_2,                  /* Pin 2 /* PDF Reference */ */
    pin_3,                  /* Pin 3 /* PDF Reference */ */
    pin_4,                  /* Pin 4 /* PDF Reference */ */
    pin_5,                  /* Pin 5 /* PDF Reference */ */
    pin_6,                  /* Pin 6 /* PDF Reference */ */
    pin_7,                  /* Pin 7 /* PDF Reference */ */
    pin_8,                  /* Pin 8 /* PDF Reference */ */
    pin_9,                  /* Pin 9 /* PDF Reference */ */
    pin_10,                 /* Pin 10 /* PDF Reference */ */
    pin_11,                 /* Pin 11 /* PDF Reference */ */
    pin_12,                 /* Pin 12 /* PDF Reference */ */
    pin_13,                 /* Pin 13 /* PDF Reference */ */
    pin_14,                 /* Pin 14 /* PDF Reference */ */
    pin_15                  /* Pin 15 /* PDF Reference */ */
} tpin;


/* ==================== FUNCTION DECLARATIONS ==================== */

/*
 * @brief Initializes a GPIO pin as an output.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 * @param[in] value: Initial output value (0 for low, 1 for high). Type tbyte.
 * @param[in] usage: Usage context (t_usage).
 * @param[in] conn: Output connection type (t_output_conn).
 */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/*
 * @brief Initializes a GPIO pin as an input.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 * @param[in] usage: Usage context (t_usage).
 * @param[in] pull: Pull resistor setting (t_pull).
 */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/*
 * @brief Gets the current direction mode of a GPIO pin.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 * @return The direction mode (t_direction).
 */
t_direction GPIO_Direction_get(tport port, tpin pin);

/*
 * @brief Sets the output value of a GPIO pin.
 *        Only applicable for pins configured as output.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 * @param[in] value: Output value to set (0 for low, 1 for high). Type tbyte.
 */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/*
 * @brief Gets the input value of a GPIO pin.
 *        Returns the output latch value for output pins.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 * @return The pin value (0 for low, 1 for high). Type tbyte.
 */
tbyte GPIO_Value_Get(tport port, tpin pin);

/*
 * @brief Toggles the output value of a GPIO pin.
 *        Only applicable for pins configured as output.
 * @param[in] port: The GPIO port (tport).
 * @param[in] pin: The pin number (tpin).
 */
void GPIO_Value_Tog(tport port, tpin pin);


#endif /* STM32F401RC_GPIO_H_ */

/* ==================== END OF FILE ==================== */