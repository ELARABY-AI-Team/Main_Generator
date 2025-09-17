#ifndef TT_SCH_H
#define TT_SCH_H

#include "main.h"

// Config: number of allowed tasks
#define TT_NUMBER_TASKS 7

typedef uint8_t tbyte;
typedef uint16_t tword;

// Task Control Block
typedef struct {
    void (*pTask)(void);  // Task function
    volatile tword delay; // Ticks until run
    tword period;         // Ticks between runs
    tbyte run_me;         // Incremented when task is due
} tcb_t;

// Public API
void tt_init(uint16_t Tick_Time_ms);
void tt_start(void);
void tt_dispatch_task(void);
void TT_ISR(void);
tbyte tt_add_task(void (*Task)(void), const tword delay, const tword period);
void tt_delete_task(const tbyte task_index);

#endif
