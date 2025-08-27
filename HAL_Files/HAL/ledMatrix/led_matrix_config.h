/******************************************************************
 * @Title    : led Matrix
 * @Filename : led_matrix_config.h
 * @Author   : AHMED ELNIWEHY (ah-mahmoudmohamed@elarabygroup.com)
 * @Origin Date : 29/08/2023
 * @Version  : 1.0.0
 * @Compiler : CC-RL
 * @Target   : R5F11BBC
 * @brief    : This file implements device driver for ledMatrix component.
 *******************************************************************/

#ifndef LED_MATRIX_CONFIG_
#define LED_MATRIX_CONFIG_

#include "../../core/GPIO/GPIO.h"



#define LED_MATRIX_COL_MAX_NO 		4
#define LED_MATRIX_ROW_MAX_NO 		7

#define LED_MATRIX_COL_ON_LEVEL    (LOW)
#define LED_MATRIX_COL_OFF_LEVEL   (HIGH)


#define LED_MATRIX_ROW_ON_LEVEL    (LOW)
#define LED_MATRIX_ROW_OFF_LEVEL   (HIGH)

#define LED_MATRIX_ALL_LEDS_ON  0xff 
#define LED_MATRIX_ALL_LEDS_OFF 0x00

typedef enum 
{
	/*COL 0*/
	LED_MATRIX_00 = 0 ,     // col : 0 , row : 0
	LED_MATRIX_01 = 1 ,     // col : 0 , row : 1
	LED_MATRIX_02 = 2 ,     // col : 0 , row : 2
	LED_MATRIX_03 = 3 ,     // col : 0 , row : 3
	LED_MATRIX_04 = 4 ,     // col : 0 , row : 4
	LED_MATRIX_05 = 5 ,     // col : 0 , row : 5
	LED_MATRIX_06 = 6 ,     // col : 0 , row : 6


	/*COL 1*/
	LED_MATRIX_10 = 10 ,     // col : 1 , row : 0
	LED_MATRIX_11 = 11 ,     // col : 1 , row : 1
	LED_MATRIX_12 = 12 ,     // col : 1 , row : 2
	LED_MATRIX_13 = 13 ,     // col : 1 , row : 3
	LED_MATRIX_14 = 14 ,     // col : 1 , row : 4
	LED_MATRIX_15 = 15 ,     // col : 1 , row : 5
	LED_MATRIX_16 = 16 ,     // col : 1 , row : 6

	/*COL 2*/
	LED_MATRIX_20 = 20 ,     // col : 2 , row : 0
	LED_MATRIX_21 = 21 ,     // col : 2 , row : 1
	LED_MATRIX_22 = 22 ,     // col : 2 , row : 2
	LED_MATRIX_23 = 23 ,     // col : 2 , row : 3
	LED_MATRIX_24 = 24 ,     // col : 2 , row : 4
	LED_MATRIX_25 = 25 ,     // col : 2 , row : 5
	LED_MATRIX_26 = 26 ,     // col : 2 , row : 6

	/*COL 3*/
	LED_MATRIX_30 = 30 ,     // col : 3 , row : 0
	LED_MATRIX_31 = 31 ,     // col : 3 , row : 1
	LED_MATRIX_32 = 32 ,     // col : 3 , row : 2
	LED_MATRIX_33 = 33 ,     // col : 3 , row : 3
	LED_MATRIX_34 = 34 ,     // col : 3 , row : 4
	LED_MATRIX_35 = 35 ,     // col : 3 , row : 5
	LED_MATRIX_36 = 36 ,     // col : 3 , row : 6


}led_matrix_id_t; 


typedef enum 
{
	LED_MATRIX_LED_ON  = 0x01,
	LED_MATRIX_LED_OFF = 0x00,
}led_matrix_state_t ; 


/******************************************************************/

#define SSDISPLAY_COMMON_ANODE     0
#define SSDISPLAY_COMMON_CATHODE   1

/*********************
 *    USER DEFINE
 *********************/
/**
 * Defines the Seven segments IDs.
 * @note: must be in the same order of defintion at
 * ssdisplay_config instance
 */
typedef enum{
    SSDISPLAY_0 = 0,
    SSDISPLAY_1,
    SSDISPLAY_2,
    SSDISPLAY_3,
    SSDISPLAY_MAX,
} ssdisplay_t;

//define the seven segment Type
#define SSDISPLAY_TYPE       SSDISPLAY_COMMON_ANODE

#define SSDISPLAY_DECIMALPOINT_USAGE    0

#define SSDISPLAY_UPDATE_PERIOD_MILLIS   2
/******************
 *    Typedefs
*******************/
#define NUM_OF_DIGITS   SSDISPLAY_2

#if SSDISPLAY_DECIMALPOINT_USAGE == 0
#define NUM_OF_SEGMENT 7
#else
#define NUM_OF_SEGMENT 8
#endif

#if (NUMBER_OF_DIGITS > 3 && NUMBER_OF_DIGITS < 6)
typedef uint16 ssdisplay_num_t;
#elif (NUMBER_OF_DIGITS < 3)
typedef uint8 ssdisplay_num_t;
#else
#error "TOO MANY DIGITS"
#endif


#endif
