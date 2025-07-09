/***********************************************************************************************************************
* File Name      : ATMEGA32_GPIO.h
* Description    : Header file for the GPIO driver of the ATMEGA32 microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-07-09
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER-EMBEDDED SYSTEM GROUP.
***********************************************************************************************************************/

#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_

#include "ATMEGA32_MAIN.h"

/* ==================== TYPE DEFINITIONS ==================== */

/**
 * @brief Defines the possible GPIO ports available on the ATMEGA32.
 */
typedef enum
{
    GPIO_PORTA = 0,  /* PDF Reference - Port A */
    GPIO_PORTB = 1,  /* PDF Reference - Port B */
    GPIO_PORTC = 2,  /* PDF Reference - Port C */
    GPIO_PORTD = 3,  /* PDF Reference - Port D */
    GPIO_PORT_TOTAL  /* Assumed - Total number of GPIO ports */
} GPIO_PortId_t;

/**
 * @brief Defines the possible pin numbers within a GPIO port (0-7).
 */
typedef enum
{
    GPIO_PIN0 = 0,  /* PDF Reference - Pin 0 */
    GPIO_PIN1 = 1,  /* PDF Reference - Pin 1 */
    GPIO_PIN2 = 2,  /* PDF Reference - Pin 2 */
    GPIO_PIN3 = 3,  /* PDF Reference - Pin 3 */
    GPIO_PIN4 = 4,  /* PDF Reference - Pin 4 */
    GPIO_PIN5 = 5,  /* PDF Reference - Pin 5 */
    GPIO_PIN6 = 6,  /* PDF Reference - Pin 6 */
    GPIO_PIN7 = 7,  /* PDF Reference - Pin 7 */
    GPIO_PIN_TOTAL  /* Assumed - Total number of pins per port */
} GPIO_PinNum_t;

/**
 * @brief Defines the possible direction states for a GPIO pin.
 */
typedef enum
{
    GPIO_DIRECTION_INPUT = 0,   /* PDF Reference - Pin configured as input */
    GPIO_DIRECTION_OUTPUT = 1,  /* PDF Reference - Pin configured as output */
    GPIO_DIRECTION_TOTAL        /* Assumed - Total number of direction states */
} GPIO_Direction_t;

/**
 * @brief Defines the possible logic level values for a GPIO pin (High or Low).
 */
typedef enum
{
    GPIO_VALUE_LOW = 0,   /* PDF Reference - Pin is logic low (0V) */
    GPIO_VALUE_HIGH = 1,  /* PDF Reference - Pin is logic high (5V or VCC) */
    GPIO_VALUE_TOTAL      /* Assumed - Total number of value states */
} GPIO_Value_t;

/**
 * @brief Defines the possible states for the internal pull-up resistor on a GPIO pin.
 * Applicable when the pin is configured as input.
 */
typedef enum
{
    GPIO_PULLUP_DISABLED = 0,  /* PDF Reference - Internal pull-up resistor is disabled */
    GPIO_PULLUP_ENABLED = 1,   /* PDF Reference - Internal pull-up resistor is enabled */
    GPIO_PULLUP_TOTAL          /* Assumed - Total number of pull-up states */
} GPIO_PullupState_t;


/* ==================== FUNCTION DECLARATIONS ==================== */

/**
 * @brief Sets the direction (Input or Output) of a specific GPIO pin.
 * @param[in] port: The GPIO port ID (e.g., GPIO_PORTA).
 * @param[in] pin: The pin number within the port (e.g., GPIO_PIN0).
 * @param[in] direction: The desired direction (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 */
void GPIO_SetPinDirection(GPIO_PortId_t port, GPIO_PinNum_t pin, GPIO_Direction_t direction);

/**
 * @brief Sets the output value (High or Low) of a specific GPIO pin.
 * This function is typically used for pins configured as output.
 * For input pins, writing to the PORT register controls the pull-up resistor.
 * @param[in] port: The GPIO port ID (e.g., GPIO_PORTA).
 * @param[in] pin: The pin number within the port (e.g., GPIO_PIN0).
 * @param[in] value: The desired output value (GPIO_VALUE_LOW or GPIO_VALUE_HIGH).
 */
void GPIO_SetOutputPinValue(GPIO_PortId_t port, GPIO_PinNum_t pin, GPIO_Value_t value);

/**
 * @brief Toggles the output value of a specific GPIO pin.
 * This function is typically used for pins configured as output.
 * @param[in] port: The GPIO port ID (e.g., GPIO_PORTA).
 * @param[in] pin: The pin number within the port (e.g., GPIO_PIN0).
 */
void GPIO_ToggleOutputPinValue(GPIO_PortId_t port, GPIO_PinNum_t pin);

/**
 * @brief Reads the logic level value of a specific GPIO pin.
 * This function is typically used for pins configured as input.
 * @param[in] port: The GPIO port ID (e.g., GPIO_PORTA).
 * @param[in] pin: The pin number within the port (e.g., GPIO_PIN0).
 * @return GPIO_Value_t: The logic level of the pin (GPIO_VALUE_LOW or GPIO_VALUE_HIGH).
 */
GPIO_Value_t GPIO_ReadInputPinValue(GPIO_PortId_t port, GPIO_PinNum_t pin);

/**
 * @brief Sets the state of the internal pull-up resistor for a specific GPIO pin.
 * This function is applicable when the pin is configured as input.
 * @param[in] port: The GPIO port ID (e.g., GPIO_PORTA).
 * @param[in] pin: The pin number within the port (e.g., GPIO_PIN0).
 * @param[in] state: The desired pull-up state (GPIO_PULLUP_DISABLED or GPIO_PULLUP_ENABLED).
 */
void GPIO_SetPinPullupState(GPIO_PortId_t port, GPIO_PinNum_t pin, GPIO_PullupState_t state);


#endif /* ATMEGA32_GPIO_H_ */