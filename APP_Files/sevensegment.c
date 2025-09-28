#include "sevensegment.h"
#include "sevensegment_user.h"
#include <stdio.h> // For snprintf, if needed for string manipulation for SSD

//==============================================================================
// Global Variables for Application State
//==============================================================================
/**
 * @brief Stores the current value displayed on the sevensegment.
 */
static tbyte SSD_CurrentValue = SEVENSEGMENT_DEFAULT_VALUE;

/**
 * @brief Temporary variable to hold new sevensegment value before committing.
 */
static tbyte SSD_TempValue = SEVENSEGMENT_DEFAULT_VALUE;

//==============================================================================
// Private Function Prototypes
//==============================================================================
static void SevenSegment_UpdateDisplay(tbyte value);

//==============================================================================
// Function Implementations
//==============================================================================

/**
 * @brief Initializes the sevensegment application.
 *
 * This function performs all necessary initializations for the sevensegment
 * application, including MCAL, HAL components, and setting up initial display values.
 *
 * Corresponds to the "APP_Init Process" flowchart.
 */
void App_Init(void)
{
    // Initialize MCU configuration (clock, LVD, WDT, etc.)
    // Assuming this implicitly sets up the clock as required (e.g., 32MHz).
    // The choice between sys_volt_3v and sys_volt_5v is arbitrary for this example.
    MCU_Config_Init(sys_volt_5v);

    // Initialize Touch Pads HAL
    TOUCH_PAD_Init();

    // Initialize Seven Segment Display HAL
    SSD_Init();
    
    // Set initial display value
    SSD_CurrentValue = SEVENSEGMENT_DEFAULT_VALUE;
    SSD_TempValue = SEVENSEGMENT_DEFAULT_VALUE; // Initialize temp value as well
    SevenSegment_UpdateDisplay(SSD_CurrentValue);
}

/**
 * @brief Updates the sevensegment display with a given 2-digit value.
 *
 * This function takes an 8-bit value, splits it into two digits (tens and units),
 * and sends them to the SSD HAL for display.
 * Assumes SSD_MAX is 2, supporting 0-99.
 *
 * @param value The 8-bit number to display (0-99).
 */
static void SevenSegment_UpdateDisplay(tbyte value)
{
    tbyte units = value % 10;
    tbyte tens = value / 10;

    // Use SSD_2Digits_Set if SSD_MAX is 2, otherwise adjust.
    // Based on HAL_SSD_Config.h, SSD_MAX is 2 by default.
    SSD_2Digits_Set(units, tens); // Assuming d0 is rightmost (units), d1 is leftmost (tens)
}


/**
 * @brief Main application logic task for the sevensegment display.
 *
 * This task handles reading touch pad inputs (PLUS/MINUS), updating the
 * internal sevensegment value, and refreshing the display if the value changes.
 * This function is designed to be called periodically by the time-triggered scheduler.
 *
 * Corresponds to the "APP_MainTask (Logic)" flowchart.
 */
void APP_MainTask(void)
{
    // Keep WDT happy during task execution
    WDT_Reset();

    // Store current value to compare against potential changes
    SSD_TempValue = SSD_CurrentValue;

    // Check Touch Pads
    if (TOUCH_PAD_get_state(PLUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        if (SSD_TempValue < SEVENSEGMENT_MAX_VALUE)
        {
            SSD_TempValue++;
        }
        else
        {
            SSD_TempValue = SEVENSEGMENT_MIN_VALUE; // Wrap around to min
        }
    }
    else if (TOUCH_PAD_get_state(MINUS_TOUCH_ID) == TOUCH_PRESSED)
    {
        if (SSD_TempValue > SEVENSEGMENT_MIN_VALUE)
        {
            SSD_TempValue--;
        }
        else
        {
            SSD_TempValue = SEVENSEGMENT_MAX_VALUE; // Wrap around to max
        }
    }

    // Compare TempValue with SSD_CurrentValue
    if (SSD_TempValue != SSD_CurrentValue)
    {
        // If Different: Update SSD_CurrentValue + Refresh Display
        SSD_CurrentValue = SSD_TempValue;
        SevenSegment_UpdateDisplay(SSD_CurrentValue);
    }
    // Else (Same): Skip Update (do nothing, display already shows correct value)
}
