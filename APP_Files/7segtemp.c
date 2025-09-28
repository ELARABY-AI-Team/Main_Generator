#include "7segtemp.h"
#include "7segtemp_user.h"

// HAL includes
#include "HAL_SSD.h"
#include "Touch_Pad.h"

/**
 * @brief Static variable to store the current temperature value displayed on the SSD.
 */
static tbyte SSD_Value = INITIAL_TEMP_VALUE;

/**
 * @brief Local helper function to update the 7-segment display.
 *
 * This function takes the current `SSD_Value` and updates the 7-segment display
 * using the `HAL_SSD` driver. Assumes a 2-digit display for values 0-99.
 */
static void SevenSeg_Display_Update(void);

/**
 * @brief Static helper function to update the 7-segment display.
 *
 * This function converts the global `SSD_Value` into individual digits
 * and sends them to the `HAL_SSD` driver. It assumes a 2-digit display.
 */
static void SevenSeg_Display_Update(void)
{
    tbyte units_digit = SSD_Value % 10;
    tbyte tens_digit = SSD_Value / 10;

#if (DISPLAY_NUM_DIGITS == 2)
    SSD_2Digits_Set(units_digit, tens_digit);
#elif (DISPLAY_NUM_DIGITS == 1)
    SSD_1Digits_Set(units_digit); // Only displays units digit if 1-digit
#else
    // Handle other digit counts or errors as needed
#warning "Unsupported DISPLAY_NUM_DIGITS in 7segtemp.c"
#endif
}

/**
 * @brief Initializes the 7segtemp application middleware.
 *
 * This function performs all necessary initializations for the 7segtemp
 * application as described in the `APP_Init Process` flowchart:
 * - Initializes MCU configuration (e.g., clock, LVR, WDT).
 * - Initializes the 7-segment display (SSD) driver.
 * - Initializes the Touch Pad driver.
 */
void App_Init(void)
{
    // 1. Initialize Clock 32MHz (Assumed by MCU_Config_Init for system voltage)
    // The specific clock frequency setting is abstracted by MCU_Config_Init.
    // Assuming sys_volt_5v results in a 32MHz clock as per common Renesas practice.
    MCU_Config_Init(sys_volt_5v);

    // 2. Initialize SSD (SSD_Init(25) from flowchart, but HAL_SSD.c SSD_Init takes no param)
    SSD_Init();
    SevenSeg_Display_Update(); // Initial display refresh

    // 3. Initialize Touch Pads
    TOUCH_PAD_Init();
}

/**
 * @brief Main application logic task for 7segtemp.
 *
 * This task implements the `APP_MainTask (Logic)` flowchart:
 * - Checks for presses on "PLUS" and "MINUS" touch pads.
 * - Updates a temporary temperature value.
 * - Compares the temporary value with the current display value.
 * - If different, updates the display value and refreshes the 7-segment display.
 * - Ensures temperature value stays within `MIN_TEMP_VALUE` and `MAX_TEMP_VALUE`.
 */
void APP_MainTask(void)
{
    tbyte TempValue = SSD_Value; // Start with current displayed value

    // 1. Check Touch Pads (states are updated by TOUCH_PAD_update scheduled task)
    // 2. PLUS Pressed? TempValue = SSD_Value + 1
    if (TOUCH_PAD_get_state(TEMP_PLUS_BUTTON_ID) == TOUCH_PRESSED)
    {
        if (TempValue < MAX_TEMP_VALUE)
        {
            TempValue++;
        }
    }
    // 3. MINUS Pressed? TempValue = SSD_Value - 1
    else if (TOUCH_PAD_get_state(TEMP_MINUS_BUTTON_ID) == TOUCH_PRESSED)
    {
        if (TempValue > MIN_TEMP_VALUE)
        {
            TempValue--;
        }
    }
    // else: No button pressed, TempValue remains SSD_Value.

    // 4. Compare TempValue with SSD_Value
    if (TempValue != SSD_Value)
    {
        // 5. If Different: Update SSD_Value + Refresh Display
        SSD_Value = TempValue;
        SevenSeg_Display_Update();
    }
    // 6. Else: Skip Update (handled by the if condition)
}
