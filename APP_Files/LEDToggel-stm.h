#ifndef LEDTOGGEL_STM_H
#define LEDTOGGEL_STM_H

#include "MCAL.h" // For MCAL types like t_port, t_pin, tbyte etc.

/**
 * @brief Initializes the LED Toggle application middleware.
 *        This includes MCU configuration, GPIO setup for the LED,
 *        and setting the initial LED state.
 */
void App_Init(void);

/**
 * @brief Time-triggered task to toggle the LED state.
 *        This function reads the current LED state and switches it.
 *        It should be added to the time-triggered scheduler.
 */
void App_LED_Toggle_Task(void);

#endif // LEDTOGGEL_STM_H
