/******************************************************************
* @Title    : ledMatrix
* @Filename : ledMatrix.h
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 29/08/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for ledMatrix component.
*******************************************************************/


#ifndef LED_MATRIX_H_
#define LED_MATRIX_H_

#include "stdint.h"
#include "led_matrix_config.h"
#include "../../core/GPIO/GPIO.h"



typedef struct
{
    gpio_port_t col_port_arr[LED_MATRIX_COL_MAX_NO];
    gpio_port_t row_port_arr[LED_MATRIX_ROW_MAX_NO];
    gpio_pin_t  col_pin_arr[LED_MATRIX_COL_MAX_NO];
    gpio_pin_t  row_pin_arr[LED_MATRIX_ROW_MAX_NO];
}led_matrix_t;

/***************************************************************/
typedef enum{
    SSDISPLAY_SYMBOL_ZERO,
    SSDISPLAY_SYMBOL_ONE,
    SSDISPLAY_SYMBOL_TWO,
    SSDISPLAY_SYMBOL_THREE,
    SSDISPLAY_SYMBOL_FOUR,
    SSDISPLAY_SYMBOL_FIVE,
    SSDISPLAY_SYMBOL_SIX,
    SSDISPLAY_SYMBOL_SEVEN,
    SSDISPLAY_SYMBOL_EIGNT,
    SSDISPLAY_SYMBOL_NINE,
    SSDISPLAY_SYMBOL_NULL,
    SSDISPLAY_SYMBOL_A,
    SSDISPLAY_SYMBOL_b,
    SSDISPLAY_SYMBOL_C,
    SSDISPLAY_SYMBOL_d,
    SSDISPLAY_SYMBOL_E,
    SSDISPLAY_SYMBOL_F,
    SSDISPLAY_SYMBOL_g,
    SSDISPLAY_SYMBOL_h,
    SSDISPLAY_SYMBOL_I,
    SSDISPLAY_SYMBOL_J,
    SSDISPLAY_SYMBOL_l,
    SSDISPLAY_SYMBOL_r,
    SSDISPLAY_SYMBOL_S,
    SSDISPLAY_SYMBOL_t,
    SSDISPLAY_SYMBOL_R,
    SSDISPLAY_SYMBOL_U,
    SSDISPLAY_SYMBOL_n,
    SSDISPLAY_SYMBOL_P,
    SSDISPLAY_SYMBOL_u,
    SSDISPLAY_SYMBOL_H,
    SSDISPLAY_SYMBOL_L,
    SSDISPLAY_SYMBOL_DOT,
    SSDISPLAY_SYMBOL_e,
    SSDISPLAY_SYMBOL_a,
    SSDISPLAY_SYMBOL_TRIPLE_DASH,
    SSDISPLAY_SYMBOL_MAX
}ssdisplay_symbol_t;

void led_matrix_init(led_matrix_t* led_matrix);

void led_matrix_set_led(led_matrix_id_t led_id , led_matrix_state_t state );

void led_matrix_set_col(uint8_t col_num,led_matrix_state_t state);

void led_matrix_update(void);

void LED_Matrix_Update(void);

void ssdisplay_set_num(ssdisplay_num_t num); // a setter function for the a certain ssdisplay value.



#endif 
