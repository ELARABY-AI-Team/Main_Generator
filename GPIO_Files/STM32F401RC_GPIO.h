#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_

#include "STM32F401RC_Config.h"

/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */
typedef enum {
    normal_usage,   /* General-purpose usage */
    communication_usage /* Dedicated for communication peripheral */
} t_usage;

typedef enum {
    output,         /* Pin configured as output */
    input,          /* Pin configured as input */
    analog          /* Pin configured for analog mode */
} t_direction;

typedef enum {
    pull_up,        /* Pull-up resistor enabled */
    pull_down       /* Pull-down resistor enabled */
} t_pull;

typedef enum {
    push_pull,      /* Push-pull output configuration */
    open_drain      /* Open-drain output configuration */
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */
typedef enum {
    PORT_A = 0,    /* PDF Reference: STM32F401RC datasheet */
    PORT_B = 1,    /* PDF Reference: STM32F401RC datasheet */
    PORT_C = 2,    /* PDF Reference: STM32F401RC datasheet */
    PORT_D = 3,    /* PDF Reference: STM32F401RC datasheet */
    PORT_E = 4,    /* PDF Reference: STM32F401RC datasheet */
    PORT_F = 5,    /* PDF Reference: STM32F401RC datasheet */
    PORT_G = 6,    /* PDF Reference: STM32F401RC datasheet */
    PORT_H = 7,    /* PDF Reference: STM32F401RC datasheet */
    PORT_I = 8     /* Assumed - please verify */
} tport;

typedef enum {
    PIN_0 = 0,     /* Assumed - please verify */
    PIN_1 = 1,     /* Assumed - please verify */
    PIN_2 = 2,     /* Assumed - please verify */
    PIN_3 = 3,     /* Assumed - please verify */
    PIN_4 = 4,     /* Assumed - please verify */
    PIN_5 = 5,     /* Assumed - please verify */
    PIN_6 = 6,     /* Assumed - please verify */
    PIN_7 = 7,     /* Assumed - please verify */
    PIN_8 = 8,     /* Assumed - please verify */
    PIN_9 = 9,     /* Assumed - please verify */
    PIN_10 = 10,   /* Assumed - please verify */
    PIN_11 = 11,   /* Assumed - please verify */
    PIN_12 = 12,   /* Assumed - please verify */
    PIN_13 = 13,   /* Assumed - please verify */
    PIN_14 = 14,   /* Assumed - please verify */
    PIN_15 = 15    /* Assumed - please verify */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, 
                     t_usage usage, t_output_conn conn);
/* Initialize specified pin as output with initial value */

void GPIO_Input_Init(tport port, tpin pin, t_usage usage, 
                    t_pull pull);
/* Initialize specified pin as input with pull configuration */

t_direction GPIO_Direction_get(tport port, tpin pin);
/* Retrieve current direction of specified pin */

void GPIO_Value_Set(tport port, tpin pin, tbyte value);
/* Set specified pin to high or low level */

tbyte GPIO_Value_Get(tport port, tpin pin);
/* Get current logic level of specified pin */

void GPIO_Value_Tog(tport port, tpin pin);
/* Toggle current value of specified pin */

#endif /* STM32F401RC_GPIO_H_ */