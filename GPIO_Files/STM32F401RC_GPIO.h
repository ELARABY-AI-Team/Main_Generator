#ifndef STM32F401RC_GPIO_H_
#define STM32F401RC_GPIO_H_
#include "STM32F401RC_Config.h"


/* ==================== REQUIRED ENUMS (DO NOT MODIFY) ==================== */
typedef enum {
    normal_usage,
    communication_usage
} t_usage;

typedef enum {
    output,
    input,
    analog
} t_direction;

typedef enum {
    pull_up,
    pull_down
} t_pull;

typedef enum {
    push_pull,
    open_drain
} t_output_conn;

/* ==================== HARDWARE DEFINITIONS ==================== */
typedef enum {
    /* Insert STM32F401RC ports (PORT_A, PORT_B, etc.) */
    /* Each entry MUST be annotated with either: */
    /* PDF Reference */  /* or */  /* Assumed - please verify */
    PORT_A = 0,  /* Assumed - please verify */
    PORT_B,      /* Assumed - please verify */
    PORT_C,      /* Assumed - please verify */
    PORT_D,      /* Assumed - please verify */
    PORT_H       /* Assumed - please verify */
} tport;

typedef enum {
    /* Insert STM32F401RC pins (PIN_0, PIN_1, etc.) */
    /* Each entry MUST be annotated with either: */
    /* PDF Reference */  /* or */  /* Assumed - please verify */
    PIN_0 = 0,   /* Assumed - please verify */
    PIN_1,       /* Assumed - please verify */
    PIN_2,       /* Assumed - please verify */
    PIN_3,       /* Assumed - please verify */
    PIN_4,       /* Assumed - please verify */
    PIN_5,       /* Assumed - please verify */
    PIN_6,       /* Assumed - please verify */
    PIN_7,       /* Assumed - please verify */
    PIN_8,       /* Assumed - please verify */
    PIN_9,       /* Assumed - please verify */
    PIN_10,      /* Assumed - please verify */
    PIN_11,      /* Assumed - please verify */
    PIN_12,      /* Assumed - please verify */
    PIN_13,      /* Assumed - please verify */
    PIN_14,      /* Assumed - please verify */
    PIN_15       /* Assumed - please verify */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */

/* Initialize a GPIO pin as an output */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);

/* Initialize a GPIO pin as an input */
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);

/* Get the direction of a GPIO pin */
t_direction GPIO_Direction_get(tport port, tpin pin);

/* Set the output value of a GPIO pin */
void GPIO_Value_Set(tport port, tpin pin, tbyte value);

/* Get the input value of a GPIO pin */
tbyte GPIO_Value_Get(tport port, tpin pin);

/* Toggle the output value of a GPIO pin */
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* STM32F401RC_GPIO_H_ */