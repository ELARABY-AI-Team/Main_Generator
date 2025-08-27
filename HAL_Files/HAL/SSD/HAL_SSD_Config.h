/********************************************************************************************************************************************/
/*  Author      : Ahmed Magdy                                                                                                             */
/*  Module      : Component_SSD_Config_H                                                                                                                       */
/*  Last Update : 08 / 11 / 2023                                                                                                            */
/********************************************************************************************************************************************/
#ifndef Component_SSD_Config_H_
#define Component_SSD_Config_H_
/**Includes Components===========================================================================*/
#include "MCAL_General_Config.h"
/**PORT ==================================================================================*/
/* 
Can DEFINE 8 SSD 
Arrangegment COM0,COM1,COM2 ----,COM7
 A
B  F
C  E
 D
*/
/*COM0*/
#define COM0_PORT (Port_1)
#define COM0_PIN  (Pin_2)
/*COM1*/
#define COM1_PORT (Port_1)
#define COM1_PIN  (Pin_3)

/*A*/
#define A_PORT (Port_2)
#define A_PIN  (Pin_0)
/*B*/
#define B_PORT (Port_2)
#define B_PIN  (Pin_3)
/*C*/
#define C_PORT (Port_0)
#define C_PIN  (Pin_0)
/*D*/
#define D_PORT (Port_12)
#define D_PIN  (Pin_0)
/*E*/
#define E_PORT (Port_0)
#define E_PIN  (Pin_1)
/*F*/
#define F_PORT (Port_2)
#define F_PIN  (Pin_2)
/*G*/
#define G_PORT (Port_2)
#define G_PIN  (Pin_1)
/**CONFIG ==================================================================================*/
#define SSD_MAX (2)
#define SSD_FLASH_TIME (100) /*Best Practice */
/*Seven segment Common anode or common cathode */
#define ssd_on  (0)
#define ssd_off (1)                  	                 
/**END ==================================================================================*/
#endif