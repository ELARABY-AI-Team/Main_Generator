#include "7segment.h"
#include "7segment_user.h"
#include "Interval_Timer.h" // For Interval_Timer_Init and ISR callback

/**
 * @file 7segment.c
 * @brief Middleware logic for the 7segment application.
 *
 * This file implements the application's behavior, integrating HAL drivers
 * and the time-triggered scheduler.
 */

//==================================================================================================
// Global Variables
//==================================================================================================

tword SSD_Value = APP_INITIAL_SSD_VALUE;
tword TempValue = APP_INITIAL_SSD_VALUE;

//==================================================================================================
// Static Helper Functions
//==================================================================================================

/**
 * @brief Updates the 7-segment display with a given 16-bit value.
 *
 * This function takes an integer value, breaks it into individual digits,
 * and calls the appropriate SSD HAL function to display it on the
 * 7-segment display. It handles multi-digit displays based on SSD_MAX.
 *
 * @param value The 16-bit integer value to display.
 */
static void SSD_Display_Value(tword value)
{
    // Ensure value stays within displayable range
    if (value > APP_SSD_MAX_DISPLAY_VALUE)
    {
        value = APP_SSD_MAX_DISPLAY_VALUE;
    }
    else if (value < APP_SSD_MIN_DISPLAY_VALUE)
    {
        value = APP_SSD_MIN_DISPLAY_VALUE;
    }

    // This implementation assumes SSD_MAX is 2 for a 2-digit display.
    // For more digits, extend with SSD_3Digits_Set, SSD_4Digits_Set etc.
    #if SSD_MAX == 2
    tbyte d0 = (tbyte)(value % 10);
    tbyte d1 = (tbyte)(value / 10);
    SSD_2Digits_Set(d0, d1); // Note: SSD_2Digits_Set takes d0 as the rightmost digit, d1 as the leftmost.
                             // Reversing the order to match typical display (tens, ones).
    #elif SSD_MAX == 1
    tbyte d0 = (tbyte)(value % 10);
    SSD_1Digits_Set(d0);
    #else
    // Default to a 2-digit display if SSD_MAX is not explicitly defined or is > 2.
    // In a real system, you would need to dynamically determine the number of digits.
    tbyte d0 = (tbyte)(value % 10);
    tbyte d1 = (tbyte)(value / 10);
    SSD_2Digits_Set(d0, d1);
    #endif
}


//==================================================================================================
// Application Functions
//==================================================================================================

/**
 * @brief Initializes the 7segment application middleware.
 *
 * This function performs all necessary initializations for the 7segment
 * application, including hardware abstraction layers (HAL) and the
 * time-triggered scheduler.
 */
void App_Init(void)
{
    // WDT_Reset is called implicitly by MCAL APIs at the start of each.
    // MCU_Config_Init implicitly enables global interrupts if not explicitly handled here.
    MCU_Config_Init(sys_volt_5v); // Initialize MCU (e.g., clock 32MHz, LVR, WDT).
                                  // Assuming 5V system voltage and that this function sets the clock as per requirements.

    // Initialize SSD HAL
    SSD_Init();

    // Initialize Touch Pad HAL
    TOUCH_PAD_Init();

    // Initialize the Interval Timer for scheduler ticks (1 ms)
    Interval_Timer_Init(LOW_SPEED_INTERNAL_CLOCK, Tick_1_ms);
    // Set the scheduler's ISR as the callback for the Interval Timer
    Interval_Timer_ISR_Callback(TT_ISR);

    // Initial display value
    SSD_Display_Value(APP_INITIAL_SSD_VALUE);
    SSD_Value = APP_INITIAL_SSD_VALUE;
    TempValue = APP_INITIAL_SSD_VALUE;
}

/**
 * @brief Main task logic for the 7segment application.
 *
 * This function implements the core behavior of the 7segment application,
 * checking touch pad inputs, updating the display value, and refreshing
 * the 7-segment display. This task is scheduled by the time-triggered scheduler.
 */
void App_MainTask(void)
{
    // WDT_Reset is called implicitly by HAL APIs.
    tword current_temp_value = SSD_Value; // Start with current displayed value

    // Check PLUS touch pad
    if (TOUCH_PAD_get_state(APP_TOUCH_PLUS_ID) == TOUCH_PRESSED)
    {
        // Increment value, handle rollover at max
        if (current_temp_value < APP_SSD_MAX_DISPLAY_VALUE)
        {
            current_temp_value++;
        }
        else
        {
            current_temp_value = APP_SSD_MIN_DISPLAY_VALUE; // Roll over to min value
        }
    }
    // Check MINUS touch pad
    else if (TOUCH_PAD_get_state(APP_TOUCH_MINUS_ID) == TOUCH_PRESSED)
    {
        // Decrement value, handle rollover at min
        if (current_temp_value > APP_SSD_MIN_DISPLAY_VALUE)
        {
            current_temp_value--;
        }
        else
        {
            current_temp_value = APP_SSD_MAX_DISPLAY_VALUE; // Roll over to max value
        }
    }
    // If neither PLUS nor MINUS is pressed, TempValue remains the current SSD_Value.

    // Compare TempValue with SSD_Value
    if (current_temp_value != SSD_Value)
    {
        // Update SSD_Value and refresh display
        SSD_Value = current_temp_value;
        SSD_Display_Value(SSD_Value);
    }
    else
    {
        // Skip update (SSD_Value remains unchanged, display continues to show it)
    }
}
