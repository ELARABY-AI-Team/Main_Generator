/**
 * @file sevensegment.c
 * @brief Main source file for the Seven Segment Display middleware application.
 *
 * This file contains the initialization, middleware logic, and application
 * entry points for the sevensegment display control.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "sevensegment.h"
#include "sevensegment_user.h"

//==================================================================================================
// Global Variables
//==================================================================================================
/**
 * @brief Stores the current value to be displayed on the seven segment.
 *        Initialized to the minimum displayable value.
 */
static tbyte SSD_Current_Value = SEVENSEGMENT_MIN_VALUE;

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the sevensegment application middleware.
 *
 * This function performs all necessary initializations for the sevensegment
 * application, including MCU configuration, peripheral initializations (SSD, Touch Pad),
 * and any other required startup routines.
 */
void App_Init(void)
{
    // 1. MCU Configuration as per APP_Init Process flowchart.
    // Initialize MCU with a default system voltage (e.g., 5V).
    MCU_Config_Init(sys_volt_5v);

    // Placeholder for "Initialize Clock 32MHz" from the flowchart.
    // The provided MCAL.h does not explicitly expose an API for setting the main
    // clock frequency to 32MHz. This typically involves configuring specific
    // RCC (Reset and Clock Control) registers (e.g., HOCODIV, OSCCTL, CKC)
    // which would be handled by dedicated MCAL_CLOCK APIs if they were available.
    // For this implementation, we assume the MCU operates at a sufficient
    // default speed or that a higher-level initialization handles this aspect.
    // Example if an API existed: MCAL_CLOCK_SetMainFrequency(32000000UL);

    // 2. Initialize Seven Segment Display (SSD).
    // The flowchart mentions "SSD_Init(25)". However, HAL_SSD.c's SSD_Init takes no arguments.
    // We proceed by calling SSD_Init() without arguments. The '25' might have been a
    // placeholder for a parameter or a reference to a different component.
    SSD_Init();
    // Initialize the display with the starting value
    SSD_2Digits_Set(SSD_Current_Value / 10, SSD_Current_Value % 10);

    // 3. Initialize Touch Pads.
    TOUCH_PAD_Init();
}

/**
 * @brief Task wrapper for updating the Seven Segment Display.
 *
 * This function calls the underlying HAL_SSD_Update to refresh the display.
 * It is intended to be scheduled by the time-triggered scheduler with a 5ms period.
 */
void SSD_Update_Task(void)
{
    // WDT_Reset() is handled by the main dispatch loop.
    SSD_Update();
}

/**
 * @brief Task wrapper for updating the Touch Pad states.
 *
 * This function calls the underlying HAL_TOUCH_PAD_update to poll touch pad inputs.
 * It is intended to be scheduled by the time-triggered scheduler with an 18ms period.
 */
void TOUCH_PAD_Update_Task(void)
{
    // WDT_Reset() is handled by the main dispatch loop.
    TOUCH_PAD_update();
}

/**
 * @brief Main application logic task for the sevensegment display.
 *
 * This task implements the logic described in the "APP_MainTask (Logic)" flowchart.
 * It checks for touch pad inputs (PLUS/MINUS), calculates a temporary value,
 * and if this temporary value differs from the currently displayed value,
 * it updates the seven segment display accordingly.
 */
void App_MainTask(void)
{
    // WDT_Reset() is handled by the main dispatch loop.

    tbyte TempValue = SSD_Current_Value; // Start with the current displayed value

    // Check for PLUS button press
    if (TOUCH_PAD_get_state(SEVENSEGMENT_PLUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        TempValue++;
        if (TempValue > SEVENSEGMENT_MAX_VALUE)
        {
            TempValue = SEVENSEGMENT_MIN_VALUE; // Wrap around to the minimum value
        }
    }
    // Check for MINUS button press
    else if (TOUCH_PAD_get_state(SEVENSEGMENT_MINUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        if (TempValue == SEVENSEGMENT_MIN_VALUE)
        {
            TempValue = SEVENSEGMENT_MAX_VALUE; // Wrap around to the maximum value
        }
        else
        {
            TempValue--;
        }
    }
    // If neither PLUS nor MINUS is pressed, TempValue remains the same as SSD_Current_Value
    // (unless multiple buttons are pressed, which this simple logic doesn't fully handle beyond the first 'if' match).

    // Compare TempValue with SSD_Current_Value
    if (TempValue != SSD_Current_Value)
    {
        // If different: Update SSD_Current_Value and refresh the display.
        SSD_Current_Value = TempValue;

        // Split the new value into tens and units digits for a 2-digit display.
        tbyte tens_digit = SSD_Current_Value / 10;
        tbyte units_digit = SSD_Current_Value % 10;
        
        // Update the SSD with the new digits.
        // Assuming d0 corresponds to the tens digit (most significant) and d1 to the units digit (least significant)
        // for typical 2-digit display arrangements in SSD_2Digits_Set.
        SSD_2Digits_Set(tens_digit, units_digit);
    }
    // Else: Skip Update (do nothing, SSD_Current_Value remains unchanged and display continues to show it).
}
