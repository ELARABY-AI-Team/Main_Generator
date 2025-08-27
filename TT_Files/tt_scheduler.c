/******************************************************************
* @Title : Scheduler (SCH)
* @Author : Ahmed Magdy
* @Origin Date : 12/10/2023
* @brief  : This file contains the implementation of the SCH Module
*******************************************************************/
#include "tt_scheduler.h"
#include "Interval_Timer.h"
// ------ Public variable definitions ------------------------------

// The array of tasks
tcb_t tasks[TT_NUMBER_TASKS];

// Used to display the error code
// See Main.H for details of error codes
// See Port.H for details of the error port


// ------ Private function prototypes ------------------------------

static void tt_sleep(void);

// ------ Private variables ----------------------------------------



/*------------------------------------------------------------------*-

  TT_ISR()

  This is the scheduler ISR.  It is called at a rate 
  determined by the timer settings in the 'init' function.
-*------------------------------------------------------------------*/

void TT_ISR(void)
{
   tbyte i;

   // NOTE: calculations are in *TICKS* (not milliseconds)
   for (i = 0; i < TT_NUMBER_TASKS; i++)
   {
      // Check if there is a task at this location
      if ( tasks[i].pTask)
      {

        

         if (tasks[i].delay == 0)
         {
            // The task is due to run
            tasks[i].run_me += 1; // Inc. the 'run_me' flag

            if (tasks[i].period)
            {
               // Schedule regular tasks to run again
               tasks[i].delay = tasks[i].period;
            }
         }
         else
         {
            tasks[i].delay -= 1;
         }
      }
   }
}
/*------------------------------------------------------------------*-

  tt_dispatch_task()

  This is the 'dispatcher' function.  When a task (function)
  is due to run, tt_dispatch_task() will run it.
  This function must be called (repeatedly) from the main loop.

-*------------------------------------------------------------------*/
void tt_dispatch_task(void)
{
   tbyte i;

   // Dispatches (runs) the next task (if one is ready)
   for (i = 0; i < TT_NUMBER_TASKS; i++)
   {
      if (tasks[i].run_me > 0)
      {
         (*tasks[i].pTask)(); // Run the task

         tasks[i].run_me -= 1; // Reset / reduce run_me flag

         // Periodic tasks will automatically run again
         // - if this is a 'one shot' task, remove it from the array
         if (tasks[i].period == 0)
         {
            tt_delete_task(i);
         }
      }
   }

   // Report system status


   // The scheduler enters idle mode at this point
   tt_sleep();
}

/*------------------------------------------------------------------*-
  tt_add_task()

  Causes a task (function) to be executed at regular intervals 
  or after a user-defined delay

  Fn_P   - The name of the function which is to be scheduled.
           NOTE: All scheduled functions must be 'void, void' -
           that is, they must take no parameters, and have 
           a void return type. 
                   
  DELAY  - The interval (TICKS) before the task is first executed

  PERIOD - If 'PERIOD' is 0, the function is only called once,
           at the time determined by 'DELAY'.  If PERIOD is non-zero,
           then the function is called repeatedly at an interval
           determined by the value of PERIOD (see below for examples
           which should help clarify this).


  RETURN VALUE:  

  Returns the position in the task array at which the task has been 
  added.  If the return value is TT_NUMBER_TASKS then the task could 
  not be added to the array (there was insufficient space).  If the
  return value is < TT_NUMBER_TASKS, then the task was added 
  successfully.  

  Note: this return value may be required, if a task is
  to be subsequently deleted - see schduler_delete_task().

  EXAMPLES:

  Task_ID = tt_add_task(Do_X,1000,0);
  Causes the function Do_X() to be executed once after 1000 sch ticks.            

  Task_ID = tt_add_task(Do_X,0,1000);
  Causes the function Do_X() to be executed regularly, every 1000 sch ticks.            

  Task_ID = tt_add_task(Do_X,300,1000);
  Causes the function Do_X() to be executed regularly, every 1000 ticks.
  Task will be first executed at T = 300 ticks, then 1300, 2300, etc.            
 
-*------------------------------------------------------------------*/
tbyte tt_add_task(void (*pFunction)(),
                     const tword DELAY,
                     const tword PERIOD)
{
   tbyte i = 0;

   // First find a gap in the array (if there is one)
   while ((tasks[i].pTask != 0) && (i < TT_NUMBER_TASKS))
   {
      i++;
   }

   // Have we reached the end of the list?
   if (i == TT_NUMBER_TASKS)
   {
      // Task list is full
      // Also return an error code
      return TT_NUMBER_TASKS;
   }

   // If we're here, there is a space in the task array
   tasks[i].pTask = pFunction;

   tasks[i].delay = DELAY;
   tasks[i].period = PERIOD;

   tasks[i].run_me = 0;

   return i; // return position of task (to allow later deletion)
}

/*------------------------------------------------------------------*-
  tt_delete_task()

  Removes a task from the scheduler.  Note that this does
  *not* delete the associated function from memory: 
  it simply means that it is no longer called by the scheduler. 
 
  TASK_INDEX - The task index.  Provided by tt_add_task(). 

  RETURN VALUE:  RETURN_ERROR or RETURN_NORMAL

-*------------------------------------------------------------------*/
void tt_delete_task(const tbyte TASK_INDEX)
{

   tasks[TASK_INDEX].pTask = 0x0000;
   tasks[TASK_INDEX].delay = 0;
   tasks[TASK_INDEX].period = 0;
   tasks[TASK_INDEX].run_me = 0;
}

// ------ Public variable declarations -----------------------------

// The array of tasks (see Sch51.C)
extern tcb_t tasks[TT_NUMBER_TASKS];

// Used to display the error code
// See Main.H for details of error codes
// See Port.H for details of the error port
/*------------------------------------------------------------------*-

  tt_init()

  Scheduler initialization function.  Prepares scheduler
  data structures and sets up timer interrupts at required rate.

  You must call this function before using the scheduler.  

-*------------------------------------------------------------------*/
void tt_init(t_Tick_Time Tick_Time_ms )
{
   tbyte i;

   for (i = 0; i < TT_NUMBER_TASKS; i++)
   {
      tt_delete_task(i);
   }

   // Reset the global error variable
   // - schduler_delete_task() will generate an error code,
   //   (because the task array is empty)

   Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK ,  Tick_Time_ms);
   Interval_Timer_ISR_Callback(TT_ISR) ;
}

/*------------------------------------------------------------------*-

  tt_start()

  Starts the scheduler, by enabling interrupts.

  NOTE: Usually called after all regular tasks are added,
  to keep the tasks synchronized.

  NOTE: ONLY THE SCHEDULER INTERRUPT SHOULD BE ENABLED!!! 

-*------------------------------------------------------------------*/
void tt_start(void)
{
   Interval_Timer_Enable();
}

/*------------------------------------------------------------------*-
  ---- END OF FILE -------------------------------------------------
-*------------------------------------------------------------------*/

/*------------------------------------------------------------------*-

  tt_report_status()

  Simple function to display error codes.

  This version displays code on a port with attached LEDs:
  adapt, if required, to report errors over serial link, etc.

  Errors are only displayed for a limited period 
  (60000 ticks = 1 minute at 1ms tick interval).
  After this the the error code is reset to 0. 

  This code may be easily adapted to display the last
  error 'for ever': this may be appropriate in your
  application.

  See Chapter 10 for further information.

-*------------------------------------------------------------------*/

/*------------------------------------------------------------------*-

  tt_sleep()

  This scheduler enters 'idle mode' between clock ticks
  to save power.  The next clock tick will return the processor
  to the normal operating state.

  Note: a slight performance improvement is possible if this
  function is implemented as a macro, or if the code here is simply 
  pasted into the 'dispatch' function.  

  However, by making this a function call, it becomes easier 
  - during development - to assess the performance of the 
  scheduler, using the 'performance analyser' in the Keil 
  hardware simulator. See Chapter 14 for examples for this. 

  *** May wish to disable this if using a watchdog ***

  *** ADAPT AS REQUIRED FOR YOUR HARDWARE ***

-*------------------------------------------------------------------*/
void tt_sleep()
{
   //TODO:
}

/*------------------------------------------------------------------*-
  ---- END OF FILE -------------------------------------------------
-*------------------------------------------------------------------*/
