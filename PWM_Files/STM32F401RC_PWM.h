/***********************************************************************************************************************
* File Name      : STM32F401RC_GPIO.h
* Description    : Header file for the GPIO peripheral on STM32F401RC microcontroller.
* Author         : Technology Inovation Software Team
* Tester         :
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   :
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_MAIN.h"

/*
 * The requested typedef enum TRD_Channel_t for PWM channels and the PWM function declarations
 * (PWM_Init, PWM_Set_Freq, etc.) cannot be defined or declared within this GPIO header file.
 * This is due to the strict requirement to "Use ONLY the following PDF content to define
 * enums, typedefs, Timers, or macro values". The provided PDF content is exclusively for
 * General Purpose I/Os (GPIO) and contains no information about PWM timers or channels.
 * Therefore, the PWM-specific elements are omitted here to comply with the constraints.
 */

/* ==================== GPIO PORT ENUMS ==================== */

/**
 * @brief GPIO Port Enumeration
 * Identifies the available GPIO ports.
 */
typedef enum
{
  GPIO_PORT_A,  /* PDF Reference */
  GPIO_PORT_B,  /* PDF Reference */
  GPIO_PORT_C,  /* PDF Reference */
  GPIO_PORT_D,  /* PDF Reference */
  GPIO_PORT_E,  /* PDF Reference */
  GPIO_PORT_H   /* PDF Reference */
} GPIO_Port_t;

/* ==================== GPIO PIN ENUMS ==================== */

/**
 * @brief GPIO Pin Enumeration
 * Identifies the individual pins within a GPIO port.
 */
typedef enum
{
  GPIO_PIN_0,   /* PDF Reference */
  GPIO_PIN_1,   /* PDF Reference */
  GPIO_PIN_2,   /* PDF Reference */
  GPIO_PIN_3,   /* PDF Reference */
  GPIO_PIN_4,   /* PDF Reference */
  GPIO_PIN_5,   /* PDF Reference */
  GPIO_PIN_6,   /* PDF Reference */
  GPIO_PIN_7,   /* PDF Reference */
  GPIO_PIN_8,   /* PDF Reference */
  GPIO_PIN_9,   /* PDF Reference */
  GPIO_PIN_10,  /* PDF Reference */
  GPIO_PIN_11,  /* PDF Reference */
  GPIO_PIN_12,  /* PDF Reference */
  GPIO_PIN_13,  /* PDF Reference */
  GPIO_PIN_14,  /* PDF Reference */
  GPIO_PIN_15   /* PDF Reference */
} GPIO_Pin_t;

/* ==================== GPIO MODE ENUMS ==================== */

/**
 * @brief GPIO Mode Enumeration
 * Configures the direction and function of the I/O pins.
 */
typedef enum
{
  GPIO_MODE_INPUT,        /* PDF Reference (00) */
  GPIO_MODE_OUTPUT,       /* PDF Reference (01) */
  GPIO_MODE_ALTERNATE_FN, /* PDF Reference (10) */
  GPIO_MODE_ANALOG        /* PDF Reference (11) */
} GPIO_Mode_t;

/* ==================== GPIO OUTPUT TYPE ENUMS ==================== */

/**
 * @brief GPIO Output Type Enumeration
 * Configures the output buffer type.
 */
typedef enum
{
  GPIO_OUTPUT_TYPE_PUSH_PULL,   /* PDF Reference (0) */
  GPIO_OUTPUT_TYPE_OPEN_DRAIN   /* PDF Reference (1) */
} GPIO_OutputType_t;

/* ==================== GPIO OUTPUT SPEED ENUMS ==================== */

/**
 * @brief GPIO Output Speed Enumeration
 * Configures the speed of the output buffer.
 */
typedef enum
{
  GPIO_OUTPUT_SPEED_LOW,        /* PDF Reference (00) */
  GPIO_OUTPUT_SPEED_MEDIUM,     /* PDF Reference (01) */
  GPIO_OUTPUT_SPEED_HIGH,       /* PDF Reference (10) */
  GPIO_OUTPUT_SPEED_VERY_HIGH   /* PDF Reference (11) */
} GPIO_OutputSpeed_t;

/* ==================== GPIO PULL-UP/PULL-DOWN ENUMS ==================== */

/**
 * @brief GPIO Pull-up/Pull-down Enumeration
 * Configures the internal pull-up or pull-down resistors.
 */
typedef enum
{
  GPIO_PULL_NO,     /* PDF Reference (00) */
  GPIO_PULL_UP,     /* PDF Reference (01) */
  GPIO_PULL_DOWN    /* PDF Reference (10) */
                    /* 11 is Reserved in PDF */
} GPIO_Pull_t;

/* ==================== GPIO ALTERNATE FUNCTION ENUMS ==================== */

/**
 * @brief GPIO Alternate Function Enumeration
 * Selects one of the possible peripheral alternate functions for a pin.
 */
typedef enum
{
  GPIO_AF_0,  /* PDF Reference (0000) */
  GPIO_AF_1,  /* PDF Reference (0001) */
  GPIO_AF_2,  /* PDF Reference (0010) */
  GPIO_AF_3,  /* PDF Reference (0011) */
  GPIO_AF_4,  /* PDF Reference (0100) */
  GPIO_AF_5,  /* PDF Reference (0101) */
  GPIO_AF_6,  /* PDF Reference (0110) */
  GPIO_AF_7,  /* PDF Reference (0111) */
  GPIO_AF_8,  /* PDF Reference (1000) */
  GPIO_AF_9,  /* PDF Reference (1001) */
  GPIO_AF_10, /* PDF Reference (1010) */
  GPIO_AF_11, /* PDF Reference (1011) */
  GPIO_AF_12, /* PDF Reference (1100) */
  GPIO_AF_13, /* PDF Reference (1101) */
  GPIO_AF_14, /* PDF Reference (1110) */
  GPIO_AF_15  /* PDF Reference (1111) */
} GPIO_AlternateFunction_t;


/* ==================== FUNCTION DECLARATIONS ==================== */
/*
 * Note: PWM related function declarations requested in the prompt are omitted
 * as they are not GPIO functions and cannot be derived from the provided
 * GPIO-only PDF content, adhering to strict requirement to "Use ONLY the
 * following PDF content...".
 */

#endif /* STM32F401RC_GPIO_H_ */