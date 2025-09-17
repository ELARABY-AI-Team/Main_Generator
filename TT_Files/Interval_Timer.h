/******************************************************************
* @Title    : 12 - BIT INTERVAL TIMER
* @Filename : INTERVAL_TIMER.h
* @Author   : Ahmed Magdy
* @Date	    : 11/10/2023
* @Version  : 1.0.0                                                                           
* @Compiler : CC-RL                                                                           
* @Target   : R5F11BBC
* @brief    : This file contains the prototypes of the INTERVAL TIMER Module
*******************************************************************/

#ifndef _INTERVAL_TIMER_H
#define _INTERVAL_TIMER_H

/**********************************
 *           INCLUDES             *
 **********************************/
#include "MCAL_R5F11BBC_Config.h"

/**********************************
 *           DATA TYPES           *
 **********************************/
typedef enum
{
    SUBSYSTEM_EXTERNAL_CLOCK = 0U,
    LOW_SPEED_INTERNAL_CLOCK
} t_clk_source;

typedef enum
{  Tick_0_133_ms = 0U ,
   Tick_0_333_ms,
   Tick_0_400_ms,
   Tick_0_533_ms,
   Tick_0_600_ms,
   Tick_0_733_ms,
   Tick_0_800_ms,
   Tick_1_ms
	}t_Tick_Time ;

/**********************************
 *     FUNCTION PROTOTYPES        *
 **********************************/
void Interval_Timer_Init(t_clk_source CLOCK_FEED_TO_12BIT_TIMER, t_Tick_Time Tick_Time_ms);
void Interval_Timer_Enable(void);
void Interval_Timer_Disable(void);

void Interval_Timer_ISR_Callback (void (*ptr)(void));

#endif
