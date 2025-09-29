/**
 * @file sevensegment.c
 * @brief Main source file for the Seven Segment middleware application.
 *
 * This file contains the initialization routines, middleware logic, and
 * application entry points for controlling a seven segment display
 * via touch pad inputs.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "sevensegment.h"
#include "sevensegment_user.h" // User-defined configurations

//==================================================================================================
// Private Global Variables
//==================================================================================================

/**
 * @brief Current value displayed on the Seven Segment Display.
 */
static int8_t SSD_Value = SEVENSEGMENT_INITIAL_VALUE;

/**
 * @brief Temporary value used for calculating new display values from user input.
 */
static int8_t TempValue = SEVENSEGMENT_INITIAL_VALUE;

//==================================================================================================
// Function Implementations
//==================================================================================================

/**
 * @brief Initializes the Seven Segment application middleware.
 *
 * This function performs all necessary initializations for the application,
 * including MCU clock, SSD, and touch pads.
 */
void App_Init(void)
{
    // 1. Initialize Clock and MCU configuration
    // The flowchart specifies 32MHz, but MCAL_Config_Init doesn't have a clock frequency parameter.
    // It configures system voltage, WDT, LVR. We assume the base clock (e.g., HOCO) is configured
    // elsewhere or defaults to 32MHz in the Renesas startup code.
    MCU_Config_Init(SEVENSEGMENT_SYSTEM_VOLTAGE);

    // 2. Initialize Seven Segment Display
    // The flowchart says SSD_Init(25), but HAL_SSD.c SSD_Init takes no arguments.
    // We will call SSD_Init() and then use SSD_1Digits_Set to set the initial value.
    SSD_Init();
    // Initialize with the configured initial value
    SSD_1Digits_Set(SSD_Value);

    // 3. Initialize Touch Pads
    TOUCH_PAD_Init();

    // 4. Enable Global Interrupts (as per flowchart "APP_Init Process")
    Global_interrupt_Enable();
}

/**
 * @brief Main logic task for the Seven Segment application.
 *
 * This task continuously checks for touchpad inputs to increment or decrement
 * the displayed value on the seven segment display. It updates the display
 * only if the value has changed.
 */
void APP_MainTask(void)
{
    // Reset watchdog timer to prevent timeout
    WDT_Reset();

    // Read current touchpad states
    bool plus_pressed = (TOUCH_PAD_get_state(SEVENSEGMENT_PLUS_TOUCH_PAD_ID) == TOUCH_PRESSED);
    bool minus_pressed = (TOUCH_PAD_get_state(SEVENSEGMENT_MINUS_TOUCH_PAD_ID) == TOUCH_PRESSED);

    // Start with current displayed value for calculations
    TempValue = SSD_Value;

    // Check for PLUS button press
    if (plus_pressed)
    {
        TempValue++;
    }
    // Check for MINUS button press
    else if (minus_pressed)
    {
        TempValue--;
    }

    // Clamp TempValue within the allowed range
    if (TempValue < SEVENSEGMENT_MIN_VALUE)
    {
        TempValue = SEVENSEGMENT_MIN_VALUE;
    }
    else if (TempValue > SEVENSEGMENT_MAX_VALUE)
    {
        TempValue = SEVENSEGMENT_MAX_VALUE;
    }

    // Compare TempValue with SSD_Value
    if (TempValue != SSD_Value)
    {
        // If different, update SSD_Value and refresh display
        SSD_Value = TempValue;
        // Assuming a 2-digit display based on SSD_MAX (2) in HAL_SSD_Config.h
        // We will display two digits.
        tbyte d0 = SSD_Value % 10;
        tbyte d1 = SSD_Value / 10;
        SSD_2Digits_Set(d0, d1); // Display the new value
    }
    // Else, skip update (no change needed)
}
