/********************************************************************************************************************************************/
/*  Author      : Sherif Elgiar                                                                                                             */
/*  Module      : Component_SSD_H                                                                                                                       */
/*  Last Update : 01 / 04 / 2023                                                                                                            */
/********************************************************************************************************************************************/
#ifndef Component_SSD_H_
#define Component_SSD_H_
/**Includes =====================================================================================*/
#include "Component_SSD_Config.h"
/**Definitions ==================================================================================*/

/**Selections ===================================================================================*/
/*Digit Limits
0,1,2,3,4,5,6,7,8,9,
'A','b','C','d','E','F','G','h','i','J','L','N','n','o','P','r','t','U','u'
*/                    	                  
/**Prototypes ===================================================================================*/
/* @brief :*/
void SSD_Init();
void SSD_Update(void/*5Msec*/);
void SSD_8Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5,tbyte d6,tbyte d7);
void SSD_7Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5,tbyte d6);
void SSD_6Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4,tbyte d5);
void SSD_5Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3,tbyte d4);
void SSD_4Digits_Set(tbyte d0,tbyte d1,tbyte d2,tbyte d3);
void SSD_3Digits_Set(tbyte d0,tbyte d1,tbyte d2);
void SSD_2Digits_Set(tbyte d0,tbyte d1);
void SSD_1Digits_Set(tbyte d0);
void SSD_Flash_Set();
void SSD_Flash_Reset();
/**END ==================================================================================*/
#endif