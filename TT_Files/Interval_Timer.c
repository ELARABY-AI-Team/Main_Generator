/******************************************************************
 * @Title    : 12 - BIT INTERVAL TIMER
 * @Filename : INTERVAL_TIMER.c
 * @Author   : Ahmed Magdy
 * @Date     : 11/10/2023
 * @Version  : 1.0.0
 * Tester    :Ahmed Yasser
 * @Compiler : CC-RL
 * @Target   : R5F11BBC
 * @brief    : This file contains the prototypes of the INTERVAL TIMER Module
 *******************************************************************/
/**********************************
 *           INCLUDES             *
 **********************************/
#include "iodefine.h"
#include "Interval_Timer.h"

/**********************************
 *     PRAGMA DIRECTIVE           *
 **********************************/
#pragma interrupt INTERVAL_TIMER_ISR(vect=INTIT)

/**********************************
 *     GLOBAL VARIABLES           *
 **********************************/
static tlong counter_value = 0;
static tlong frequency_value = 0;
static void (*Interval_Timer_CallBack)(void);

/**********************************
 *     GLOBAL FUNCTIONS           *
 **********************************/




void Interval_Timer_Init(t_clk_source CLOCK_FEED_TO_12BIT_TIMER, t_Tick_Time Tick_Time_ms)
{
	ITMK = 1U;      /* disable INTIT interrupt */
	ITIF = 0U;      /* clear INTIT interrupt flag */

	RTCEN = 1 ; 				/* Enable Clock to the interval timer */

	switch(CLOCK_FEED_TO_12BIT_TIMER)
	{
	case SUBSYSTEM_EXTERNAL_CLOCK :
		OSMC &= ~(1 << 4);
		frequency_value = 32768 ;
		break;
	case LOW_SPEED_INTERNAL_CLOCK :
		OSMC |= (1 << 4);
		frequency_value = 15000 ;
		break;
	default :
		OSMC |= (1 << 4);
		frequency_value = 15000 ;
		break;
	}

	switch(Tick_Time_ms){
	case Tick_0_133_ms  :
		counter_value =1;break ;
	case Tick_0_333_ms  :
		counter_value = 4;break ;
	case Tick_0_400_ms  :
		counter_value = 5;break ;
	case Tick_0_533_ms  :
		counter_value = 7;break ;
	case Tick_0_600_ms  :
	    counter_value = 8;break ;
	case Tick_0_733_ms  :
			counter_value = 10;break ;
	case Tick_0_800_ms  :
			counter_value = 11;break ;
	case Tick_1_ms      :
		counter_value = 14;break ;
	default : frequency_value= frequency_value/1000 ;
		      counter_value = ( frequency_value * Tick_Time_ms )-1;
		      break ;
	}

	/* Set INTIT high priority */
	ITPR1 = 0U;
	ITPR0 = 0U;

	ITIF = 0U;      /* clear INTIT interrupt flag */
	ITMK = 0U;      /* enable INTIT interrupt */

	ITMC = (tword) counter_value;/*PUT Counter value on register*/
}

void Interval_Timer_Enable(void)
{
	/*Timer Start*/
	ITMC |= 0x8000U;
}

void Interval_Timer_Disable(void)
{
	ITMC &= (tword)~(0x8000);  /* Stop IT */
}

void Interval_Timer_ISR_Callback (void (*ptr)(void))
{
	Interval_Timer_CallBack = ptr;
}

static void __near INTERVAL_TIMER_ISR(void)
{
	Interval_Timer_CallBack();
}
