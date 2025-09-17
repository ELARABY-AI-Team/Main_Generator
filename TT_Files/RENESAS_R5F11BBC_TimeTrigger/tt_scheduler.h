/******************************************************************
* @Title    : TIME TRIGGERED Scheduler (TTSCH)                        
* @Filename : schedule.h                              
* @Author   : Ahmed Magdy                     
* @Origin Date : 12/10/2023                                                                    
* @Version  : 1.0.0                                                                           
* @Compiler : GCC                                                                           
* @Target   : R5F11BBC                                                                     
* @brief    : This file contains the prototypes of the SCH Module                         
* @Notes    : You have to configure here the number of tasks                                        
*******************************************************************/
					         
					         
#ifndef TT_SCH_H		         
#define TT_SCH_H		       
                                     

#include "MCAL_General_Config.h"
#include "Interval_Timer.h"
//#define ERROR_HANDLE
//#define SCH_REPORT_ERRORS
                                        
// ------ Public data type declarations ----------------------------
#define  TT_NUMBER_TASKS        10
// Store in DATA area, if possible, for rapid access  
// Total memory per task is 7 bytes
typedef struct 
   {
   // Pointer to the task (must be a 'void (void)' function)
   void (*pTask)(void);  

   // delay (ticks) until the function will (next) be run
   // - see sch_add_task() for further details
   volatile tword delay;

   // Interval (ticks) between subsequent runs.
   // - see sch_add_task() for further details
   tword period;

   // Incremented (by scheduler) when task is due to execute
   tbyte run_me;
   } tcb_t; 

// ------ Public function prototypes -------------------------------

void tt_init(t_Tick_Time Tick_Time_ms  );//delet clock source
void tt_start(void);
void tt_dispatch_task(void);
void TT_ISR(void) ;
tbyte tt_add_task(void (* Task) (void), const tword, const tword);
void tt_delete_task(const tbyte TASK_INDEX);


// ------ Public constants -----------------------------------------
                                
// The maximum number of tasks required at any one time
// during the execution  of the program
//                                       
// MUST BE ADJUSTED FOR EACH NEW PROJECT

/**
* Defines number of tasks used in the system.
*/
#if defined(CONFIGURATION_FILE)
#define SCH_NUMBER_TASKS   (SCH_NUM_TASKS)
#else
#define SCH_NUMBER_TASKS (4)
#endif

#endif
                              
/*------------------------------------------------------------------*-
  ---- END OF FILE -------------------------------------------------
-*------------------------------------------------------------------*/
