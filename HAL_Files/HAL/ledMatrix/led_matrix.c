/******************************************************************
* @Title    : ledMatrix
* @Filename : ledMatrix.c
* @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
* @Origin Date : 29/08/2023
* @Version  : 1.0.0
* @Compiler : CC-RL
* @Target   : R5F11BBC
* @brief    : This file implements device driver for ledMatrix component.
*******************************************************************/


#include "led_matrix.h"
#include "../../core/GPIO/GPIO.h"

/**************************************************************/


/*********************
 *      DEFINE
 *********************/
#if SSDISPLAY_TYPE == SSDISPLAY_COMMON_ANODE
#define SSDISPLAY_ON    LOW
#define SSDISPLAY_OFF   HIGH
#elif SSDISPLAY_TYPE == SSDISPLAY_COMMON_CATHODE
#define SSDISPLAY_ON    HIGH
#define SSDISPLAY_OFF   LOW
#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef struct {
	/*ssdisplay_symbol_t*/uint8_t symbol;
#if SSDISPLAY_DECIMALPOINT_USAGE == 1
    uint8 dp;
#endif
} ssdisplay_data_t;

/**********************
 *  STATIC VARIABLES
 **********************/
static volatile ssdisplay_data_t ssdisplay_data[NUM_OF_DIGITS];

//     --a--
//    f     b
//     --g--
//    e     c
//     --d--
//todo: add decimalpoint numbers .0 .1 ....
#if SSDISPLAY_TYPE == SSDISPLAY_COMMON_ANODE
//COMMON ANODE
static uint8 ssdisplay_symbols[] = {
		/* 0b0gfedcba  */
		~0b00111111,    //~0x3F, // 0
		~0b00000110,    //~0x06, // 1
		~0b01011011,    //~0x5B, // 2
		~0b01001111,    //~0x4F, // 3
		~0b01100110,    //~0x66, // 4
		~0b01101101,    //~0x6D, // 5
		~0b01111101,    //~0x7D, // 6
		~0b00000111,    //~0x07, // 7
		~0b01111111,    //~0x7F, // 8
		~0b01101111,    //~0x6F, // 9
		~0b00000000, 	/* NULL */
		~0b01110111,    //~0x77, // A
		~0b01111100,    //~0x7C, // b
		~0b00111001,    //~0x39, // C
		~0b01011110,    //~0x5E, // d
		~0b01111001,    //~0x79, // E
		~0b01110001,    //~0x71, // F
		~0b01101111,    //~0x6F, // g
		~0b01110100,    //~0x74, // h
		~0b00110000,    //~0x30, // I
		~0b00001110,    //~0x0E, // J
		~0b00001000,    //~0x08, // l
		~0b01010000,    //~0x50, // r
		~0b01101101,    //~0x6D, // S
		~0b01111000,    //~0x78, // t---
		~0b00110001,    //~0x31, // R---
		~0b00111110,    //~0x3E, // U
		~0b00110111,    //~0x37, // n---
		~0b01110011,    //~0x73, // P
		~0b00011100,    //~0x1C, // u
		~0b01110110,    //~0x76, // H
		~0b00111000,    //~0x38, // L
		~0b10000000,    //~0x7F, // dot
		~0b01111011,    //~0x7B, // e
		~0b01011111,    //~0x5F, // a
		~0b01001001,    //~0x49, //Triple Dash
		};
#elif SSDISPLAY_TYPE == SSDISPLAY_COMMON_CATHODE
static uint8 ssdisplay_symbols[] = {
 /* 0b0gfedcba  */
    0b00111111,    //~0x3F, // 0
    0b00000110,    //~0x06, // 1
    0b01011011,    //~0x5B, // 2
    0b01001111,    //~0x4F, // 3
    0b01100110,    //~0x66, // 4
    0b01101101,    //~0x6D, // 5
    0b01111101,    //~0x7D, // 6
    0b00000111,    //~0x07, // 7
    0b01111111,    //~0x7F, // 8
    0b01101111,    //~0x6F, // 9
    0b00000000,    /* NULL */
    0b01110111,    //~0x77, // A
    0b01111100,    //~0x7C, // b
    0b00111001,    //~0x39, // C
    0b01011110,    //~0x5E, // d
    0b01111001,    //~0x79, // E
    0b01110001,    //~0x71, // F
    0b01101111,    //~0x6F, // g
    0b01110100,    //~0x74, // h
    0b00110000,    //~0x30, // I
    0b00001110,    //~0x0E, // J
    0b00001000,    //~0x08, // l
    0b01010000,    //~0x50, // r
    0b01101101,    //~0x6D, // S
    0b01111000,    //~0x78, // t---
    0b00110001,    //~0x31, // R---
    0b00111110,    //~0x3E, // U
    0b00110111,    //~0x37, // n---
    0b01110011,    //~0x73, // P
    0b00011100,    //~0x1C, // u
    0b01110110,    //~0x76, // H
    0b00111000,    //~0x38, // L
    0b10000000,    //~0x7F, // dot
    0b01111011,    //~0x7B, // e
    0b01011111,    //~0x5F, // a
    0b01001001,    //~0x49, //Triple Dash
};
#endif

/**************************************************************/
typedef uint8_t led_matrix_row_data_t;
led_matrix_t led_matrix_container ; 
static led_matrix_row_data_t led_matrix_row_data[LED_MATRIX_COL_MAX_NO] = {0};



void led_matrix_init(led_matrix_t* led_matrix)
{
    uint8_t index = 0 ; 
    /*col handling */
    for(index = 0 ; index < LED_MATRIX_COL_MAX_NO ; index++)
    {
        led_matrix_row_data[index] = LED_MATRIX_ALL_LEDS_OFF;
        led_matrix_container.col_port_arr[index] = led_matrix->col_port_arr[index];
        led_matrix_container.col_pin_arr[index]  = led_matrix->col_pin_arr[index];
        GPIO_Pin_Output_Init(led_matrix_container.col_port_arr[index],led_matrix_container.col_pin_arr[index]);
        GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[index],led_matrix_container.col_pin_arr[index],LED_MATRIX_COL_OFF_LEVEL);
    }

    /*row handling*/
    for(index = 0 ; index < LED_MATRIX_ROW_MAX_NO; index++)
    {
        
        led_matrix_container.row_port_arr[index] = led_matrix->row_port_arr[index];
        led_matrix_container.row_pin_arr[index]  = led_matrix->row_pin_arr[index];
        GPIO_Pin_Output_Init(led_matrix_container.row_port_arr[index],led_matrix_container.row_pin_arr[index]);
        GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index],led_matrix_container.row_pin_arr[index],LED_MATRIX_ROW_OFF_LEVEL);
    }

}


void led_matrix_set_led(led_matrix_id_t led_id , led_matrix_state_t state )
{
    //get col and row 
    uint8_t row , col ; 

    row = led_id % 10 ; 
    col = led_id / 10 ; 

    //clean led row location
    led_matrix_row_data[col] &= ~(1<<row);
    //config led row value 
    led_matrix_row_data[col] |= (state << row);
}

void led_matrix_set_col(uint8_t col_num,led_matrix_state_t state)
{
    if(state == LED_MATRIX_LED_ON)
    {
         led_matrix_row_data[col_num]  = LED_MATRIX_ALL_LEDS_ON;
    }
    else
    {
        led_matrix_row_data[col_num]  = LED_MATRIX_ALL_LEDS_OFF;
    }
}

void led_matrix_update(void)
{
    static uint8_t col= 0; 
    uint8_t index = 0 ;
   
    //turn off all cols 
    for (index = 0 ; index < LED_MATRIX_COL_MAX_NO ; index++)
    {
          GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[index],led_matrix_container.col_pin_arr[index],LED_MATRIX_COL_OFF_LEVEL);
    }

    //configure rows 
    for (index = 0 ; index < LED_MATRIX_ROW_MAX_NO ; index++)
    {

        if((led_matrix_row_data[col]>>index) & 0x01)
        {
            GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index],led_matrix_container.row_pin_arr[index],LED_MATRIX_ROW_ON_LEVEL);
        }
        else
        {
            GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index],led_matrix_container.row_pin_arr[index],LED_MATRIX_ROW_OFF_LEVEL);
        }

    }

    GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[col],led_matrix_container.col_pin_arr[col],LED_MATRIX_COL_ON_LEVEL);

    col++ ; 

    if(col > LED_MATRIX_COL_MAX_NO-1)
    {
        col = 0 ;
    }

}

/***********************************************************************/
void ssdisplay_set_num(ssdisplay_num_t num) {
	uint8 digit_no = 0;
         for(digit_no =0 ; digit_no <NUM_OF_DIGITS; digit_no++ ){
		ssdisplay_data[digit_no].symbol = /*(ssdisplay_symbol_t)*/(uint8_t) (num % 10);
		num /= 10;
         }
}

void LED_Matrix_Update(void)
{

    static uint8_t col= 0;
    static uint8_t digit_index = 0 ;
    uint8_t index = 0 ;

      if(col > LED_MATRIX_COL_MAX_NO-1)
        {
            col = 0 ;
        }

    //turn off all cols
    for (index = 0 ; index < LED_MATRIX_COL_MAX_NO ; index++)
    {
          GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[index],led_matrix_container.col_pin_arr[index],LED_MATRIX_COL_OFF_LEVEL);
    }

    //configure rows

	if(col > 1)
	{
		for (index = 0 ; index < LED_MATRIX_ROW_MAX_NO ; index++)
		{

			if((led_matrix_row_data[col]>>index) & 0x01)
			{
				GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index], led_matrix_container.row_pin_arr[index],LED_MATRIX_ROW_ON_LEVEL);
			}
			else
			{
				GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index], led_matrix_container.row_pin_arr[index],LED_MATRIX_ROW_OFF_LEVEL);
			}

		}

		GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[col], led_matrix_container.col_pin_arr[col], LED_MATRIX_COL_ON_LEVEL);
	}else{
		//GPIO_Pin_Output_Init(PORT1, PIN3) ;//init A2
		GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[digit_index],
				led_matrix_container.col_pin_arr[digit_index], SSDISPLAY_OFF);

			//digit index update
			if (digit_index == (NUM_OF_DIGITS - 1)) {
				digit_index = 0;
			} else {
				digit_index++;
			}
		for (index = 0; index < 7; index++)
		{
		uint8_t data  = ((ssdisplay_symbols[ssdisplay_data[digit_index].symbol] >> index) & 0x01);
		GPIO_Set_Pin_Value(led_matrix_container.row_port_arr[index], led_matrix_container.row_pin_arr[index], data);

		}
#if SSDISPLAY_DECIMALPOINT_USAGE == 1
    GPIO_Set_Pin_Value(ssdisplay_config.segment[7].port, ssdisplay_config.segment[7].pin, ssdisplay_data[col].dp);
#endif

		GPIO_Set_Pin_Value(led_matrix_container.col_port_arr[digit_index], led_matrix_container.col_pin_arr[0], LED_MATRIX_COL_ON_LEVEL);

			}

    col++ ;
}
