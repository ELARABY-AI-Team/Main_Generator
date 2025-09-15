#include "LEDToggel.h"
#include "LEDToggel_user.h"
#include "MCAL.h" /* For GPIO and MCU_Config functions */

/*==================================================================================================
*                                       LOCAL CONSTANTS
==================================================================================================*/

/* Define initial LED state based on common anode/cathode. Assuming active high for 'on' */
#if (LEDTOGGEL_LED_ACTIVE_HIGH == 1)
#define LEDTOGGEL_STATE_ON  (1)
#define LEDTOGGEL_STATE_OFF (0)
#else
#define LEDTOGGEL_STATE_ON  (0)
#define LEDTOGGEL_STATE_OFF (1)
#endif

/*==================================================================================================
*                                       LOCAL VARIABLES
==================================================================================================*/

/**
 * @brief Stores the current state of the LED.
 *        Initialized to the configured default state.
 */
static tbyte current_led_state = LEDTOGGEL_INITIAL_STATE_DEFAULT;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/

/**
 * @brief Gets the actual millisecond value for a given scheduler tick time enum.
 * @param tick_enum The t_tick_time enum value.
 * @return The corresponding millisecond value.
 */
static tword get_scheduler_tick_ms(t_tick_time tick_enum);

/*==================================================================================================
*                                     FUNCTION DEFINITIONS
==================================================================================================*/

/**
 * @brief Gets the actual millisecond value for a given scheduler tick time enum.
 * @param tick_enum The t_tick_time enum value.
 * @return The corresponding millisecond value.
 */
static tword get_scheduler_tick_ms(t_tick_time tick_enum)
{
    tword tick_ms;
    switch (tick_enum)
    {
        case tick_time_1ms:   tick_ms = 1;    break;
        case tick_time_10ms:  tick_ms = 10;   break;
        case tick_time_100ms: tick_ms = 100;  break;
        case tick_time_1s:    tick_ms = 1000; break;
        default:              tick_ms = 1;    break; /* Default to 1ms if an invalid enum is provided */
    }
    return tick_ms;
}

/**
 * @brief Initializes the LEDToggel application.
 *
 * This function performs the necessary MCU configurations,
 * initializes the GPIO for the LED, and sets the initial LED state.
 * It follows the APP_Init Process flowchart.
 */
void App_Init(void)
{
    /* 1. MCU_Config (from flowchart) */
    /* Initialize MCU with specified system voltage (e.g., 5V) */
    MCU_Config_Init(LEDTOGGEL_SYS_VOLTAGE);

    /* 2. GPIO_Init (from flowchart) */
    /* Initialize the LED pin as an output with the default initial state */
    GPIO_Output_Init(LEDTOGGEL_STATUS_PORT, LEDTOGGEL_STATUS_PIN, LEDTOGGEL_INITIAL_STATE_DEFAULT);
    
    /* 3. LED_Seton (from flowchart) - Initial state is LEDTOGGEL_INITIAL_STATE_DEFAULT (e.g., OFF) */
    /* Update internal state variable */
    current_led_state = LEDTOGGEL_INITIAL_STATE_DEFAULT;
    
    /* Set the LED to the initial state (e.g., turn it OFF) */
    GPIO_Value_Set(LEDTOGGEL_STATUS_PORT, LEDTOGGEL_STATUS_PIN, current_led_state);
}

/**
 * @brief Toggles the state of the LED.
 *
 * This function reads the current state of the LED and
 * switches it to the opposite state (ON to OFF, or OFF to ON).
 * It follows the Leg Toggle Process flowchart.
 */
void App_ToggleLed(void)
{
    /* The MCAL function GPIO_Value_Tog provides a direct way to toggle the pin */
    GPIO_Value_Tog(LEDTOGGEL_STATUS_PORT, LEDTOGGEL_STATUS_PIN);
    
    /* Update internal state variable after toggling */
    current_led_state = GPIO_Value_Get(LEDTOGGEL_STATUS_PORT, LEDTOGGEL_STATUS_PIN);
}

/* Helper function for scheduler if needed for period calculation */
tword App_GetSchedulerTickMs(t_tick_time tick_enum)
{
    return get_scheduler_tick_ms(tick_enum);
}
