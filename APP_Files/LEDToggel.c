/**
 * @file LEDToggel.c
 * @brief Main source file for the LEDToggel middleware application.
 *
 * This file contains the initialization logic and the core middleware task for
 * toggling an LED, integrated with the time-triggered scheduler.
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h"       /* For MCAL layer functions like GPIO, MCU_Config */
#include "tt_scheduler.h" /* For adding tasks to the scheduler */

/*==================================================================================================
*                                      LOCAL DEFINES
==================================================================================================*/

/**
 * @brief Defines the physical GPIO port for the LED.
 *        Refer to the Flowchart and MCAL.h for available ports.
 *        Flowchart specifies Port 7.
 */
#define LEDTOGGLE_PHYSICAL_PORT  port_7

/**
 * @brief Defines the physical GPIO pin for the LED.
 *        Refer to the Flowchart and MCAL.h for available pins.
 *        Flowchart specifies Pin 4.
 */
#define LEDTOGGLE_PHYSICAL_PIN   pin_4

/**
 * @brief Determines the logic level for the LED to be considered ON.
 *        Based on LEDTOGGLE_ACTIVE_LOW user configuration.
 */
#if LEDTOGGLE_ACTIVE_LOW
    #define LED_ACTIVE_VALUE_ON  ((tbyte)0)
    #define LED_ACTIVE_VALUE_OFF ((tbyte)1)
#else
    #define LED_ACTIVE_VALUE_ON  ((tbyte)1)
    #define LED_ACTIVE_VALUE_OFF ((tbyte)0)
#endif

/**
 * @brief Determines the initial pin value based on user configuration.
 *        Used during GPIO initialization.
 */
#if LEDTOGGLE_INITIAL_STATE_ON
    #define LED_INITIAL_PIN_VALUE LED_ACTIVE_VALUE_ON
#else
    #define LED_INITIAL_PIN_VALUE LED_ACTIVE_VALUE_OFF
#endif

/*==================================================================================================
*                                      LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/**
 * @brief Toggles the state of the LED.
 *        This function is executed by the time-triggered scheduler.
 */
static void LED_Toggle_Task(void);

/*==================================================================================================
*                                      GLOBAL FUNCTIONS
==================================================================================================*/

/**
 * @brief Initializes the LEDToggel application middleware.
 *
 * This function performs the necessary MCU configurations, initializes the LED
 * GPIO pin, sets its initial state, and adds the LED toggling task to the
 * time-triggered scheduler.
 *
 * Refer to the "APP_Init Process" flowchart for the sequence.
 */
void App_Init(void)
{
    // 1. MCU_Config
    // Initialize MCU with 5V system voltage as a common default.
    // This will also enable WDT and configure LVR.
    MCU_Config_Init(sys_volt_5v);

    // 2. GPIO_Init (Port 7, Pin 4, Active Low0)
    // Configure the LED pin as an output with its initial state (OFF).
    // The initial state (ON/OFF) and active logic (LOW/HIGH) are configured in LEDToggel_user.h
    // and translated by local defines.
    GPIO_Output_Init(LEDTOGGLE_PHYSICAL_PORT, LEDTOGGLE_PHYSICAL_PIN, LED_INITIAL_PIN_VALUE);

    // 3. LED_SetOn (0=ON, 1=OFF)
    // The flowchart indicates setting LED to ON (value 0) after GPIO_Init.
    // However, the `GPIO_Output_Init` above already sets the initial state based on `LEDTOGGLE_INITIAL_STATE_ON`.
    // If the requirement is strictly to set ON *after* init regardless of `LEDTOGGLE_INITIAL_STATE_ON`,
    // uncomment the line below. Otherwise, the initial state is already handled.
    // GPIO_Value_Set(LEDTOGGLE_PHYSICAL_PORT, LEDTOGGLE_PHYSICAL_PIN, LED_ACTIVE_VALUE_ON);

    // 4. Add Task: LED_Toggle (delay=0, period=500ms)
    tt_add_task(&LED_Toggle_Task, 0, LEDTOGGLE_PERIOD_MS);
}

/*==================================================================================================
*                                      LOCAL FUNCTIONS
==================================================================================================*/

/**
 * @brief Toggles the state of the LED.
 *
 * This function is designed to be called periodically by the time-triggered
 * scheduler. It reads the current state of the LED pin and switches it
 * between ON and OFF.
 *
 * Refer to the "LED Toggle Task" flowchart for the logic.
 */
static void LED_Toggle_Task(void)
{
    // Read the current physical pin value of the LED.
    tbyte current_pin_value = GPIO_Value_Get(LEDTOGGLE_PHYSICAL_PORT, LEDTOGGLE_PHYSICAL_PIN);

    // Check if the LED is currently ON based on its active state logic.
    if (current_pin_value == LED_ACTIVE_VALUE_ON)
    {
        // LED is currently ON, so set it to OFF.
        GPIO_Value_Set(LEDTOGGLE_PHYSICAL_PORT, LEDTOGGLE_PHYSICAL_PIN, LED_ACTIVE_VALUE_OFF);
    }
    else
    {
        // LED is currently OFF, so set it to ON.
        GPIO_Value_Set(LEDTOGGLE_PHYSICAL_PORT, LEDTOGGLE_PHYSICAL_PIN, LED_ACTIVE_VALUE_ON);
    }
}

/*==================================================================================================
*                                        END OF FILE
==================================================================================================*/
