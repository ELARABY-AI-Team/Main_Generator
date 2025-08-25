
/********************************************************************************************************************************************/
/*  Author      : Sherif Elgiar                                                                                                             */
/*  Module      : Component_PB_C                                                                                                                      */
/*  Last Update : 01 / 01 / 2023                                                                                                            */
/********************************************************************************************************************************************/
/**Includes ==================================================================================*/
#include "Component_PB.h"
/**Static Variables ==================================================================================*/
static tpb PB[PB_MAX];
static tbyte pb_n;
/**Functions ==================================================================================*/
/* @brief  :*/
void PB_Init(tbyte name,tport port,tbyte pin,tedge edge)
 {
 	WDT_Reset();
 	PB[name].port=port;
 	PB[name].pin=pin;
 	PB[name].edge=edge;
 	PB[name].press_i=0;
 	PB[name].unpress_i=0;
 	PB[name].flag=0;
	GPIO_Input_Init(PB[name].port,PB[name].pin,normal_usage);	
 }
/* @brief  :*/
void PB_Update()
 {
 	WDT_Reset();
 for(pb_n=0;pb_n<PB_MAX;pb_n++)
 	{
 	if(PB[pb_n].flag==0)
 	  {
 	   if(GPIO_Value_Get(PB[pb_n].port,PB[pb_n].pin)==PB[pb_n].edge){PB[pb_n].press_i++;}
 	   if(PB[pb_n].press_i==short_press){PB[pb_n].flag=1;PB[pb_n].unpress_i=0;}
 	  }
 	   if(GPIO_Value_Get(PB[pb_n].port,PB[pb_n].pin)!=PB[pb_n].edge){PB[pb_n].unpress_i++;}
 	   if(PB[pb_n].unpress_i==short_release){PB[pb_n].flag=0;PB[pb_n].press_i=0;}
 	}
 
 }
 /* @brief  :*/
tbyte PB_Get(tbyte name)
 {
 	WDT_Reset();
 	if(PB[name].flag==1){PB[name].flag=2;return (1);}
 	else{return (0);}
 }
/**END ==================================================================================*/
