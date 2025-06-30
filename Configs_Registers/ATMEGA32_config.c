#include "ATMEGA32_config.h"
#include "ATMEGA32_MAIN.h"

/**
 * @brief Safe guard function to initialize basic protections for GPIO and registers
 */
static int safe_guards(void) {
    if (GPIO_SAFEGUARD_Init() != 0) {
        return -1;  // Failed to initialize GPIO safeguards
    }
    
    if (Registers_SAFEGUARD_Init() != 0) {
        return -2;  // Failed to initialize register safeguards
    }
    
    return 0;
}

/**
 * @brief Initialize Low Voltage Reset (LVR) configuration
 */
static int LVR_init(void) {
    // Setting LVR threshold to 3.3V level
    // Assuming bit values based on datasheet, please verify and update if necessary
    SET_BIT(MCUCR, MCUCR_LVRT_0);
    SET_BIT(MCUCR, MCUCR_LVRT_1);
    
    return 0;
}

/**
 * @brief Enable Low Voltage Reset (LVR) with configured threshold
 */
static int LVR_Enable(void) {
    // Enable LVR using the selected threshold level
    SET_BIT(MCUCR, MCUCR_LVCE);
    
    return 0;
}

/**
 * @brief Initialize Watchdog Timer (WDT) configuration
 */
static int WDT_INIT(void) {
    uint8_t timeout = WD_TOUT_8MS;  // Selecting minimum 8ms timeout
    
    // Setting WDT prescaler to achieve desired timeout
    // Assuming TWSRE0 bit value, please verify and update if necessary
    SET_BIT(WDTCSR, TWSRE0);
    
    return 0;
}

/**
 * @brief Enable Watchdog Timer (WDT) with correct window configuration
 */
int WDT_Enable(void) {
    uint8_t wdt_status = WD_STATUS_NORMAL;
    
    // Enabling WDT in interrupt mode
    SET_BIT(WDTCSR, WDCE);
    SET_BIT(WDTCSR, WDE);
    
    if ((GET_BIT(WDTCSR, WDIE)) != 1) {
        return -3;  // Failed to enable WDT interrupt
    }
    
    return 0;
}

/**
 * @brief Initialize Clock Logic Circuit (CLC)
 */
int CLC_Init(void) {
    // Setting internal clock source configuration
    // Assuming CKSEL setting for external crystal, please verify and update if necessary
    SET_BIT(CPUCTL, CPUCTL_CKSEL_0);
    CLEAR_BIT(CPUCTL, CPUCTL_CKSEL_1);
    
    return 0;
}

/**
 * @brief Main MCU initialization function
 */
void mcu_config_Init(void) {
    // Execute initialization sequence
    if (safe_guards() != 0) {
        // Handle error: Failed to initialize safeguards
    }
    
    LVR_init();
    LVR_Enable();
    
    WDT_INIT();
    WDT_Enable();
    
    CLC_Init();
}

/**
 * @brief Reset/Refresh Watchdog Timer
 */
void WDI_Reset(void) {
    // Writing magic value to reset watchdog timer
    WDTCR = WDTCR | (1 << WDCE);
    WDTCR = WDTCR | (1 << WDE);
}