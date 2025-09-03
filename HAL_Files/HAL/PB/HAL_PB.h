/********************************************************************************************************************************************/
/*  Author      : Sherif Elgiar                                                                                                             */
/*  Module      : Component_PB_H                                                                                                                       */
/*  Last Update : 01 / 01 / 2023                                                                                                            */
/********************************************************************************************************************************************/
#ifndef Component_PB_H_
#define Component_PB_H_
/**Includes ==================================================================================*/
#include "MCAL_General_Config.h"
#include "Component_PB_Config.h"
/**Definitions ===============================================================================*/
#define short_press (4)
#define short_release (10)
/**Selections ================================================================================*/
 typedef enum {
 	          falling_edge=0 ,
			  rising_edge=1
              }tedge;
 typedef struct
               {
               	tport   port;
               	tbyte   pin;
               	tedge   edge;
               	tbyte   press_i;
               	tbyte   unpress_i;
               	tbyte   flag;
               }tpb;      
/**Prototypes ================================================================================*/
/* @brief: PB_Init(PB1,PB1_PORT,PB1_PIN,rising_edge);} */
void PB_Init(tbyte name,tport port,tbyte pin,tedge edge);
/* @brief:OS_TT_Update(5);} */
void PB_Update(void);
/* @brief: if(PB_Get(PB1)){TOG_BIT(_pd,4);} */
tbyte PB_Get(tbyte name);
/**END =======================================================================================*/
#endif