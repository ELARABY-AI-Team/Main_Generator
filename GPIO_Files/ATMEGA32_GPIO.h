#ifndef ATMEGA32_GPIO_H_
#define ATMEGA32_GPIO_H_
#include "ATMEGA32_Config.h"

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
    PORT_A = 0,   /* Assumed - please verify */
    PORT_B = 1,   /* Assumed - please verify */
    PORT_C = 2,   /* Assumed - please verify */
    PORT_D = 3    /* Assumed - please verify */
} tport;

typedef enum {
    PIN_0 = 0,     /* Assumed - please verify */
    PIN_1 = 1,     /* Assumed - please verify */
    PIN_2 = 2,     /* Assumed - please verify */
    PIN_3 = 3,     /* Assumed - please verify */
    PIN_4 = 4,     /* Assumed - please verify */
    PIN_5 = 5,     /* Assumed - please verify */
    PIN_6 = 6,     /* Assumed - please verify */
    PIN_7 = 7      /* Assumed - please verify */
} tpin;

/* ==================== FUNCTION DECLARATIONS ==================== */
void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage, t_output_conn conn);
void GPIO_Input_Init(tport port, tpin pin, t_usage usage, t_pull pull);
t_direction GPIO_Direction_get(tport port, tpin pin);
void GPIO_Value_Set(tport port, tpin pin, tbyte value);
tbyte GPIO_Value_Get(tport port, tpin pin);
void GPIO_Value_Tog(tport port, tpin pin);

#endif /* ATMEGA32_GPIO_H_ */