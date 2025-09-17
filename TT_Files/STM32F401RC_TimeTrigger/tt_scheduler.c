#include "tt_scheduler.h"

static tcb_t tasks[TT_NUMBER_TASKS];

static void tt_sleep(void)
{
    __WFI(); // Optional: Enter low-power mode
}

void TT_ISR(void)
{
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        if (tasks[i].pTask)
        {
            if (tasks[i].delay == 0)
            {
                tasks[i].run_me++;
                if (tasks[i].period)
                    tasks[i].delay = tasks[i].period-1;
            }
            else
            {
                tasks[i].delay--;
            }
        }
    }
}

void tt_dispatch_task(void)
{
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        if (tasks[i].run_me > 0)
        {
            (*tasks[i].pTask)();
            tasks[i].run_me--;

            if (tasks[i].period == 0)
            {
                tt_delete_task(i);
            }
        }
    }

    tt_sleep();
}

tbyte tt_add_task(void (*pFunction)(void), const tword delay, const tword period)
{
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        if (tasks[i].pTask == 0)
        {
            tasks[i].pTask = pFunction;
            tasks[i].delay = delay;
            tasks[i].period = period;
            tasks[i].run_me = 0;
            return i;
        }
    }

    return TT_NUMBER_TASKS; // No space
}

void tt_delete_task(const tbyte task_index)
{
    tasks[task_index].pTask = 0;
    tasks[task_index].delay = 0;
    tasks[task_index].period = 0;
    tasks[task_index].run_me = 0;
}

void tt_init(uint16_t Tick_Time_ms)
{
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        tt_delete_task(i);
    }

    // Setup SysTick for tick time in ms
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000) * Tick_Time_ms;
    SysTick_Config(ticks);
}

void tt_start(void)
{
    // Nothing needed — SysTick already enabled
}
