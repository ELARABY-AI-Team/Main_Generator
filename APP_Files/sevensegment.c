/**
 * @file sevensegment.c
 * @brief Main source file for the sevensegment middleware application.
 *
 * This file contains the initialization, middleware logic, and application
 * entry points for controlling a seven-segment display based on touch input,
 * using the provided MCAL, HAL (SSD, Touch_Pad), and time-triggered scheduler.
 */

//=============================================================================
// Includes
//=============================================================================
#include "sevensegment.h"
#include "sevensegment_user.h"
#include "HAL_SSD.h"
#include "Touch_Pad.h"
#include "Interval_Timer.h" // For Interval Timer specific functions
#include "tt_scheduler.h"   // For TT_ISR and tt_delete_task

//=============================================================================
// Private Variables
//=============================================================================

/**
 * @brief Current value to be displayed on the seven-segment display.
 * This value is updated by user input.
 */
static tbyte SSD_Value;

/**
 * @brief Temporary value used for calculating new display values
 * before updating SSD_Value.
 */
static tbyte TempValue;

//=============================================================================
// Global/Public Functions
//=============================================================================

/**
 * @brief Initializes the sevensegment application middleware.
 *
 * This function performs all necessary hardware and software initializations
 * for the application, including MCAL, HAL components, and the time-triggered
 * scheduler's timer.
 */
void App_Init(void)
{
    // Ensure Watchdog Timer is reset periodically during long initialization
    WDT_Reset();

    // 1. Enable Global Interrupt (as per flowchart)
    // This is enabled early, but will be re-enabled in main() for consistency
    // with typical system startup sequences.
    Global_interrupt_Enable();

    // 2. Initialize Clock to 32MHz and other core MCU configurations.
    // MCU_Config_Init will handle system clock setup, Low Voltage Reset (LVR),
    // and disable unused peripherals as per MCAL requirements.
    MCU_Config_Init(SYS_VOLTAGE);

    // 3. Initialize Seven-Segment Display (SSD) HAL.
    SSD_Init();
    // Initialize the application's internal SSD value and update the display.
    SSD_Value = INITIAL_SSD_DISPLAY_VALUE;
    // SSD_MAX is configured to 2 in HAL_SSD_Config.h, so use SSD_2Digits_Set.
    SSD_2Digits_Set(SSD_Value / 10, SSD_Value % 10);

    // 4. Initialize Touch Pad HAL.
    TOUCH_PAD_Init();

    // 5. Initialize the Time Trigger system (Interval Timer for scheduler).
    // The tt_scheduler's internal task array must be cleared before adding new tasks.
    // Note: We bypass tt_scheduler.c's tt_init() because it uses SysTick,
    // and we are required to use Interval_Timer.
    for (tbyte i = 0; i < TT_NUMBER_TASKS; i++)
    {
        tt_delete_task(i); // Clear all existing scheduler tasks.
    }

    // Configure and initialize the Interval Timer as the scheduler's tick source.
    Interval_Timer_Init(SCHEDULER_CLOCK_SOURCE, SCHEDULER_TICK_TIME_ENUM);
    // Register the scheduler's ISR (TT_ISR) to be called by the Interval Timer's interrupt.
    Interval_Timer_ISR_Callback(TT_ISR);

    // Final Watchdog Timer reset before leaving initialization.
    WDT_Reset();
}

/**
 * @brief The main application logic task for sevensegment display control.
 *
 * This task checks for touch pad inputs (PLUS/MINUS) to adjust the displayed
 * value on the seven-segment display and updates it if there is a change.
 * This function runs periodically via the time-triggered scheduler.
 */
void App_MainTask(void)
{
    // Always reset WDT at the start of a task.
    WDT_Reset();

    // Initialize TempValue with current SSD_Value for comparison.
    TempValue = SSD_Value;

    // Check Touch Pads for input.
    // If PLUS_TOUCH is pressed, increment TempValue.
    if (TOUCH_PAD_get_state(PLUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        // Ensure value does not exceed maximum.
        if (TempValue < MAX_SSD_DISPLAY_VALUE)
        {
            TempValue++;
        }
        else
        {
            TempValue = MIN_SSD_DISPLAY_VALUE; // Wrap around or clamp
        }
    }
    // If MINUS_TOUCH is pressed, decrement TempValue.
    else if (TOUCH_PAD_get_state(MINUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        // Ensure value does not go below minimum.
        if (TempValue > MIN_SSD_DISPLAY_VALUE)
        {
            TempValue--;
        }
        else
        {
            TempValue = MAX_SSD_DISPLAY_VALUE; // Wrap around or clamp
        }
    }
    // If neither PLUS nor MINUS is pressed, TempValue remains current SSD_Value.

    // Compare TempValue with SSD_Value.
    if (TempValue != SSD_Value)
    {
        // If different, update SSD_Value and refresh the display.
        SSD_Value = TempValue;
        // Update the SSD_HAL with the new two-digit value.
        SSD_2Digits_Set(SSD_Value / 10, SSD_Value % 10);
        // The actual display refresh will be handled by the periodic SSD_Update task.
    }
    // Else (if TempValue == SSD_Value), skip update.
}

//=============================================================================
// End of File
//=============================================================================