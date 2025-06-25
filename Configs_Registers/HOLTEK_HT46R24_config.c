/**
 * @file config.c
 * @brief Microcontroller configuration functions for HOLTEK HT46R24.
 *
 * Implements initialization routines for essential peripherals like
 * GPIO, System Registers, Low Voltage Reset (LVR), Watchdog Timer (WDT),
 * and Clock (CLC).
 */

#include "config.h"
#include "HOLTEK_HT46R24_MAIN.h" // Provides typedefs, macros, register definitions

// --- Local Defines ---
// Define timeout for clock stabilization if not provided by main.h
// Adjust this value based on expected clock startup time and MCU frequency.
#ifndef CLOCK_STABILIZATION_TIMEOUT
#define CLOCK_STABILIZATION_TIMEOUT 10000 // Arbitrary large number of loops
#endif

// --- Static Function Prototypes ---
static void safe_guards(void);
static void LVR_init(void);
static void LVR_Enable(void);
static void WDT_INIT(void);

// --- Function Implementations ---

/**
 * @brief Performs initial safeguard configurations.
 *
 * Calls critical initialization functions for GPIO and Registers
 * to ensure a safe default state before main configuration.
 */
static void safe_guards(void)
{
    // Initialize GPIO pins to a safe state (e.g., input with pull-up disabled)
    GPIO_SAFEGUARD_Init();
    // Initialize critical registers to known, safe default values
    Registers_SAFEGUARD_Init();
}

/**
 * @brief Initializes the Low Voltage Reset (LVR) threshold.
 *
 * Configures the LVR voltage detection level. Assumes a macro
 * like _LVR_VOLTAGE_3V3 exists in HOLTEK_HT46R24_MAIN.h.
 */
static void LVR_init(void)
{
    // Access LVR control register (LVRC?) and set the desired threshold bits
    // This is a placeholder; register and bit names should come from main.h
    // Example assuming a register LVRC and bits for voltage selection:
    // LVRC = (LVRC & ~_LVR_VOLTAGE_MASK) | _LVR_VOLTAGE_3V3;
    // Since no specific register name is given, using a generic placeholder.
    // Assumes _LVR_REG controls the threshold.
    // Example: _LVR_REG = _LVR_VOLTAGE_3V3; // Direct assignment if simple register
    // Example: _LVR_REG |= _LVR_VOLTAGE_3V3; // Setting bits
    // Placeholder: Assuming _LVR_VOLTAGE_BITS sets the threshold in an LVR control register.
    // Need to clear existing voltage bits first if necessary.
    // Using a generic register name placeholder. Let's assume a register named LVRC
    // and a macro _LVR_VOLTAGE_3V3 and maybe a mask _LVR_VOLTAGE_MASK
#ifdef _LVR_REG
    // Example: _LVR_REG = (_LVR_REG & ~_LVR_VOLTAGE_MASK) | _LVR_VOLTAGE_3V3;
    // Simple example assuming a direct assignment macro exists:
    _LVR_REG = _LVR_VOLTAGE_3V3; // Set LVR threshold to 3.3V
#else
    // Placeholder if specific LVR macros/registers are unknown
    // You would replace this with actual register writes based on datasheet/main.h
    // Example: LVRC = (LVRC & 0xF0) | 0x03; // Assuming 0x03 is the code for 3.3V
#warning "LVR_init placeholder used. Replace with actual LVR register configuration from main.h."
#endif
}

/**
 * @brief Enables the Low Voltage Reset (LVR) function.
 *
 * Activates the LVR circuit. Assumes a bit macro like _LVR_EN
 * exists in HOLTEK_HT46R24_MAIN.h.
 */
static void LVR_Enable(void)
{
    // Enable LVR function using SET_BIT macro
    // Assumes a register _LVR_REG and an enable bit _LVR_EN
#ifdef _LVR_REG
    SET_BIT(_LVR_REG, _LVR_EN); // Enable LVR
#else
#warning "LVR_Enable placeholder used. Replace with actual LVR enable bit setting from main.h."
    // Example: SET_BIT(LVRC, 7); // Assuming bit 7 enables LVR
#endif
}

/**
 * @brief Initializes the Watchdog Timer (WDT) period.
 *
 * Configures the WDT timeout period to be >= 8ms. Assumes a macro
 * like _WDT_PERIOD_8MS exists in HOLTEK_HT46R24_MAIN.h. Does not enable the WDT.
 */
static void WDT_INIT(void)
{
    // Configure WDT period using the appropriate register(s) and macros
    // This is a placeholder; register and bit names should come from main.h
    // Example assuming a register WDTC and bits for period selection:
    // WDTC = (WDTC & ~_WDT_PERIOD_MASK) | _WDT_PERIOD_8MS;
    // Assumes _WDT_REG controls the period.
#ifdef _WDT_REG
    // Example: _WDT_REG = (_WDT_REG & ~_WDT_PERIOD_MASK) | _WDT_PERIOD_8MS;
    // Simple example assuming a direct assignment macro exists:
    _WDT_REG = _WDT_PERIOD_8MS; // Set WDT period to >= 8ms
#else
    // Placeholder if specific WDT macros/registers are unknown
    // Example: WDTC = (WDTC & 0xF8) | 0x01; // Assuming 0x01 is the code for >= 8ms
#warning "WDT_INIT placeholder used. Replace with actual WDT period configuration from main.h."
#endif
}

/**
 * @brief Enables the Watchdog Timer (WDT) and configures window.
 *
 * Activates the WDT and sets its window mode if applicable.
 * Assumes macros like _WDT_ENABLE and _WDT_WINDOW_XXX exist in main.h.
 */
void WDT_Enable(void)
{
    // Enable WDT and set window config.
    // Assumes _WDT_REG controls enable and window.
    // Need to combine enable bit and window bits.
#ifdef _WDT_REG
    // Example: _WDT_REG = (_WDT_REG & ~_WDT_WINDOW_MASK) | _WDT_WINDOW_DEFAULT | _WDT_ENABLE;
    // Simple example combining period (set in WDT_INIT), window, and enable
    // This might overwrite period set in WDT_INIT depending on register structure.
    // A more robust approach might involve multiple writes or reading the current state.
    // Assuming _WDT_REG allows setting enable and window bits together, possibly
    // after the period is set in WDT_INIT.
    // Let's assume a macro combines enable and window setting.
    _WDT_REG = _WDT_ENABLE_AND_WINDOW_CONFIG; // Placeholder macro combining enable and window
#else
#warning "WDT_Enable placeholder used. Replace with actual WDT enable/window setting from main.h."
    // Example: WDTC |= (1 << _WDT_EN_BIT) | (_WDT_WINDOW_CODE << _WDT_WINDOW_SHIFT);
#endif
}

/**
 * @brief Configures the microcontroller's clock source.
 *
 * Sets the clock source to the Internal oscillator. Includes a timeout
 * check for stabilization. Assumes macros like _CLK_REG, _CLK_SRC_INTERNAL,
 * and _CLK_BUSY exist in HOLTEK_HT46R24_MAIN.h.
 */
void CLC_Init(void)
{
    volatile tword timeout_counter = CLOCK_STABILIZATION_TIMEOUT;

    // Select Internal clock source
    // Assumes a register _CLK_REG controlling clock source selection
    // Need to clear existing source bits first if necessary.
#ifdef _CLK_REG
    // Example: _CLK_REG = (_CLK_REG & ~_CLK_SRC_MASK) | _CLK_SRC_INTERNAL;
    // Simple example assuming a direct assignment macro exists:
    _CLK_REG = _CLK_SRC_INTERNAL; // Select internal clock
#else
#warning "CLC_Init clock source selection placeholder used. Replace with actual CLK register configuration from main.h."
    // Example: CLKC = (CLKC & 0xF0) | 0x00; // Assuming 0x00 selects internal RC
#endif

    // Wait for clock to stabilize if there's a busy/ready flag
    // Assumes a status bit _CLK_BUSY in a register _CLK_STATUS_REG
#if defined(_CLK_STATUS_REG) && defined(_CLK_BUSY)
    while (GET_BIT(_CLK_STATUS_REG, _CLK_BUSY) && timeout_counter > 0)
    {
        timeout_counter--;
    }

    // Handle timeout if clock didn't stabilize
    if (timeout_counter == 0)
    {
        // Clock did not stabilize within the timeout period.
        // This indicates a critical failure.
        // Possible actions:
        // 1. Enter an infinite loop (common in simple embedded systems)
        // 2. Trigger a reset (if supported)
        // 3. Flash an error code on LEDs (if GPIO configured)
        // For this example, entering an infinite loop.
        while (1) { /* Clock initialization failed */ }
    }
#else
#warning "CLC_Init clock stabilization check disabled. Define _CLK_STATUS_REG and _CLK_BUSY or remove check if not needed/possible."
#endif
}

/**
 * @brief Resets or refreshes the Watchdog Timer (WDT).
 *
 * Prevents the WDT from timing out and resetting the microcontroller.
 * This function should be called periodically by the main application loop.
 * Assumes a macro or sequence to reset WDT exists, e.g., _WDT_RESET.
 */
void WDI_Reset(void)
{
    // Refresh the WDT. This typically involves writing a specific value
    // or sequence to a register, or calling a specific macro.
    // Assumes a macro _WDT_RESET exists in main.h that performs the reset.
#ifdef _WDT_RESET
    _WDT_RESET(); // Perform WDT reset sequence/write
#else
#warning "WDI_Reset placeholder used. Replace with actual WDT reset macro or sequence from main.h."
    // Example: WDTR = 0x5A; WDTR = 0xA5; // Common two-write sequence
    // Example: _WDT_REG = _WDT_RESET_VALUE; // Single write
#endif
}

/**
 * @brief Main microcontroller configuration function.
 *
 * Initializes all necessary system components in the correct order
 * for production readiness.
 */
void mcu_config_Init(void)
{
    // 1. Perform initial safeguard configurations
    safe_guards();

    // 2. Initialize Low Voltage Reset (LVR) threshold (3.3V)
    LVR_init();

    // 3. Enable LVR
    LVR_Enable();

    // 4. Setup Watchdog Timer (>= 8ms period)
    WDT_INIT();

    // 5. Enable WDT with default window config
    WDT_Enable(); // This function is explicitly *not* static

    // 6. Configure clock source to Internal
    CLC_Init(); // This function is explicitly *not* static
}