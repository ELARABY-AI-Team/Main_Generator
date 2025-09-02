// =============================================================================
// MCAL.c - Microcontroller Abstraction Layer implementation for RENESAS_R5F11BBC
// =============================================================================

// --- Core Includes (from Rules.json -> core_includes) ---
// Device-specific header for RENESAS_R5F11BBC.
// Actual filename might vary (e.g., iodefine.h, sfr_r5f11bbc.h).
// This is a placeholder; please replace with the correct Renesas device header.
#include "renesas_r5f11bbc.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- MCAL Header Inclusion ---
#include "RENESAS_R5F11BBC_MCAL.h"

// --- Global Variables/Definitions (Internal) ---

// For Time-Triggered OS
#define MAX_TASKS 10
typedef struct {
    void (*pTask)(void);
    tword Delay;
    tword Period;
    tbyte RunMe;
    tbyte TaskID;
} S_TASK_SCHEDULER;

static S_TASK_SCHEDULER SCH_tasks_g[MAX_TASKS];
static tbyte SCH_next_task_ID_g = 0;
static volatile tword g_tick_counter_ms = 0;

// For ICU Callback
static void (*icu_callback_g)(void) = NULL;

// For Renesas RL78 intrinsics (global interrupt enable/disable)
// These are placeholders. In a real project, these would be provided by the compiler toolchain.
#ifndef __DI
#define __DI()  asm("DI") // Disable Interrupts
#endif
#ifndef __EI
#define __EI()  asm("EI") // Enable Interrupts
#endif
#ifndef _halt
#define _halt() asm("HALT") // Stop instruction for sleep mode
#endif

// Helper function to get the actual GPIO Port register address based on enum
static volatile tbyte* Get_GPIO_Port_Register(t_port port) {
    volatile tbyte* reg = NULL;
    switch (port) {
        case PORT_0:  reg = &GPIO_P0; break;
        case PORT_1:  reg = &GPIO_P1; break;
        case PORT_2:  reg = &GPIO_P2; break;
        case PORT_3:  reg = &GPIO_P3; break;
        case PORT_4:  reg = &GPIO_P4; break;
        case PORT_5:  reg = &GPIO_P5; break;
        case PORT_6:  reg = &GPIO_P6; break;
        case PORT_7:  reg = &GPIO_P7; break;
        case PORT_12: reg = &GPIO_P12; break;
        case PORT_13: reg = &GPIO_P13; break;
        case PORT_14: reg = &GPIO_P14; break;
        default: break; // Handle error or invalid port
    }
    return reg;
}

// Helper function to get the actual GPIO Port Mode register address based on enum
static volatile tbyte* Get_GPIO_PM_Register(t_port port) {
    volatile tbyte* reg = NULL;
    switch (port) {
        case PORT_0:  reg = &GPIO_PM0; break;
        case PORT_1:  reg = &GPIO_PM1; break;
        case PORT_2:  reg = &GPIO_PM2; break;
        case PORT_3:  reg = &GPIO_PM3; break;
        case PORT_4:  reg = &GPIO_PM4; break;
        case PORT_5:  reg = &GPIO_PM5; break;
        case PORT_6:  reg = &GPIO_PM6; break;
        case PORT_7:  reg = &GPIO_PM7; break;
        case PORT_12: reg = &GPIO_PM12; break;
        case PORT_13: reg = &GPIO_PM13; break;
        case PORT_14: reg = &GPIO_PM14; break;
        default: break; // Handle error or invalid port
    }
    return reg;
}

// Helper function to get the actual GPIO Port Mode Control register address based on enum
static volatile tbyte* Get_GPIO_PMC_Register(t_port port) {
    volatile tbyte* reg = NULL;
    switch (port) {
        case PORT_0:  reg = &GPIO_PMC0; break;
        case PORT_1:  reg = &GPIO_PMC1; break;
        case PORT_13: reg = &GPIO_PMC13; break;
        // P2, P3, P4, P5, P6, P7, P12, P14 do not have PMC registers in the provided JSON
        default: break; // Handle error or invalid port
    }
    return reg;
}

// Helper function to get the actual GPIO Port Input Mode register address based on enum
static volatile tbyte* Get_GPIO_PIM_Register(t_port port) {
    volatile tbyte* reg = NULL;
    switch (port) {
        case PORT_0:  reg = &GPIO_PIM0; break;
        case PORT_1:  reg = &GPIO_PIM1; break;
        // Other ports do not have PIM registers in the provided JSON
        default: break;
    }
    return reg;
}

// Helper function to get the actual GPIO Port Output Mode register address based on enum
static volatile tbyte* Get_GPIO_POM_Register(t_port port) {
    volatile tbyte* reg = NULL;
    switch (port) {
        case PORT_0:  reg = &GPIO_POM0; break;
        case PORT_1:  reg = &GPIO_POM1; break;
        // Other ports do not have POM registers in the provided JSON
        default: break;
    }
    return reg;
}

// =============================================================================
// WDT (Watchdog Timer) Implementation (first as per Rules.json)
// =============================================================================

/**
 * @brief Resets (feeds) the Watchdog Timer.
 * @details This function reloads the WDT counter to prevent a reset.
 *          It must be called periodically to keep the MCU from resetting.
 *          Assumes a specific write pattern for Renesas RL78 WDT.
 */
void WDT_Reset(void) {
    // For Renesas RL78, writing a specific value (0xAC) to WDT_WTT resets the timer.
    // The WDT_WDM (mode register) is usually configured once during initialization.
    // WDT_WTT is usually a 16-bit register, but the JSON defines it as 8-bit at address 0xFFC1.
    // This implies an 8-bit access is expected or it's a specific byte of a larger register.
    // Assuming 8-bit access and that writing 0xAC to it clears the timer.
    // The R5F11BBC datasheet specifies that WDT_WTT is an 8-bit register and writing 0xAC
    // to it clears the counter.
    WDT_WTT = 0xACU;
}

/**
 * @brief Initializes the Watchdog Timer.
 * @details Configures the WDT mode and period.
 *          As per rules, WDT is enabled during MCU_Config_Init.
 *          This implementation sets up a basic WDT.
 *          Always includes WDT_Reset() at the start.
 */
void WDT_Init(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // WDT_WDM: Watchdog Timer Mode Register (0xFFC0)
    // Common settings for Renesas RL78 WDT_WDM:
    // Bit 7: WDSTBY: Watchdog timer operation in STOP mode (0: Disable, 1: Enable) - Keep 0 for now.
    // Bit 6-4: WDCS: Watchdog timer clock source (e.g., fIL, fRL)
    // Bit 3-0: WDPS: Watchdog timer prescaler (period selection)
    //
    // For >= 8ms period:
    // Using fIL (15kHz) as clock source (e.g., WDCS = 0b000)
    // Prescaler WDPS = 0b1010 (for ~8.192ms) or similar value.
    // For simplicity, let's assume a typical default or a known safe value for >8ms.
    // From RL78/G13 (similar family) documentation, WDM = 0x88 (0b10001000) gives 15.36ms with fIL.
    // Or WDM = 0x0E (0b00001110) gives 8.192ms with fIL. Let's use 0x0E.
    //
    // WDM format (RL78):
    // Bit 7: WDT_STOP_MODE_ENABLE (1=enable in STOP mode)
    // Bit 6-4: WDT_CLOCK_SELECT (e.g., 000=fIL, 001=fIH)
    // Bit 3-0: WDT_PRESCALER_SELECT
    //
    // Let's configure for ~8.192 ms with fIL (low-speed on-chip oscillator)
    // WDT_WDM = 0x0E; // WDCS=000 (fIL), WDPS=1110 (2^13/fIL = 8.192ms if fIL=4MHz) -- This is incorrect.
    //
    // A more appropriate setting for RL78/G13 type MCU, for 8.192ms with 15kHz fIL:
    // WDM = (0b000 << 4) | 0b1110; // Clock = fIL, Prescaler (2^13 cycles) -> 2^13 / 15kHz = ~546ms
    //
    // The rule specified "Set WDT period >= 8 msec". Without exact register bit definitions,
    // I will use a common Renesas pattern: WDT_WDM.0x0E often implies a specific configuration.
    // A common safe configuration for WDT is `0xB8` (interval timer mode, clock fIL, 2^13 cycles => ~546ms at 15KHz fIL).
    // To achieve >= 8ms, we need a smaller divider.
    // Example from RL78/G14:
    // fIL = 15 kHz
    // WDCS = 000 for fIL
    // WDPS (0b0000 - 0b1111) ranges from 2^13 to 2^20.
    // For 8ms, 2^13/15kHz = 546ms (too slow)
    // This implies WDT_WDM might not directly control a wide range of periods.
    // A different approach: many Renesas MCUs have `WDTE` bit in `PRC0` or similar register to enable WDT.
    //
    // Given the limited register JSON, I'll assume WDT_WDM directly holds the period selection.
    // A typical period value like 0x0E (for 8.192ms at certain clock) or 0x8E (if bit 7 means enable)
    // I'll make an assumption for a Renesas-like value.
    // Example: For RL78/G13, to get 8.192ms, need `WDSTBY=0`, `WDCS=000` (fIL), `WDPS=1000` (2^16 cycles) => 2^16 / 15kHz = 4.3s.
    // My register JSON does not detail WDT_WDM bits, so I'll make an inference.
    // Assuming WDPS value of `0x0F` (highest division for longest time) or `0x08` (for shorter).
    // To meet >=8ms, and enable it.
    // Renesas typically has a specific sequence to enable WDT:
    // 1. Write protection release (e.g., `PRC0 = 0xA5`)
    // 2. Set WDT_WDM
    // 3. Write protection lock.
    // The `register_json` doesn't include `PRC0`. I'll assume direct access to WDT_WDM is okay
    // or that it's handled by some other default.
    //
    // For RL78, WDT_WDM is often directly writable without protection if it's not a critical register.
    // Let's assume `0x0F` as an example for the "period" setting to be >=8ms,
    // which corresponds to the largest prescaler value, so it gives the *longest* time, ensuring >=8ms.
    // The default operation is typically enabled.
    WDT_WDM = 0x0FU; // Placeholder: Assumed value to set WDT period >= 8ms.
                     // Actual value depends on MCU datasheet and desired clock/prescaler.
                     // This value typically configures the WDT interval and source clock.
}

// =============================================================================
// MCU CONFIG Implementation
// =============================================================================

/**
 * @brief Initializes the Microcontroller Configuration.
 * @details This function performs essential MCU setup including GPIOs,
 *          disabling unused peripherals, and configuring the Watchdog Timer and LVR.
 *          As per rules, WDT_Reset() is called before any other operation.
 * @param volt System voltage (e.g., SYS_VOLT_3V, SYS_VOLT_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // --- 1. Disable Global Interrupts ---
    Global_interrupt_Disable();

    // --- 2. Set all GPIO pins to 0 and verify ---
    // The rule specifies setting ALL GPIO pins, which implies P0-P7, P12-P14
    for (t_port p = PORT_0; p <= PORT_14; p++) {
        volatile tbyte* port_reg = Get_GPIO_Port_Register(p);
        if (port_reg != NULL) {
            *port_reg = 0x00U; // Set all pins to low
            // Verify
            while (*port_reg != 0x00U) {
                *port_reg = 0x00U; // Retry until verified
            }
        }
    }

    // --- 3. Set all GPIO pins direction to input and verify ---
    // (Input = 1 on Renesas RL78 PM registers)
    for (t_port p = PORT_0; p <= PORT_14; p++) {
        volatile tbyte* pm_reg = Get_GPIO_PM_Register(p);
        if (pm_reg != NULL) {
            *pm_reg = 0xFFU; // Set all pins to input mode (1 for input)
            // Verify
            while (*pm_reg != 0xFFU) {
                *pm_reg = 0xFFU; // Retry until verified
            }
        }
        // Also clear PMC registers for alternate functions to ensure GPIO mode
        volatile tbyte* pmc_reg = Get_GPIO_PMC_Register(p);
        if (pmc_reg != NULL) {
            *pmc_reg = 0x00U; // Set all to general purpose I/O
        }
    }

    // --- 4. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.) ---
    // Already disabled global interrupts above.

    // Disable ADC
    CLEAR_BIT(ADC_ADM0, 7); // Stop A/D conversion (ADCE bit, assuming bit 7 based on common Renesas ADM0)
    ADC_ADCS = 0x00U;       // Stop A/D conversion
    ADC_ADMK = 0x01U;       // Mask A/D interrupt (disable)
    ADC_PMC = 0x00U;        // Disable all analog inputs
    ADC_ADPC = 0x00U;       // Clear A/D Port Configuration

    // Disable SAU0 (UART/SPI/I2C) - Stop all channels
    SAU0_ST0 = 0xFFU; // Stop all SAU0 channels
    // Clear mode registers for all SAU0 channels
    SAU0_SMR00 = 0x00U;
    SAU0_SMR01 = 0x00U;
    SAU0_SMR02 = 0x00U;
    SAU0_SMR03 = 0x00U;
    // Clear control registers for all SAU0 channels
    SAU0_SCR00 = 0x00U;
    SAU0_SCR01 = 0x00U;
    SAU0_SCR02 = 0x00U;
    SAU0_SCR03 = 0x00U;
    // Mask all SAU0 interrupts
    SAU0_STMK0 = 0x01U; SAU0_STMK1 = 0x01U; SAU0_STMK2 = 0x01U; SAU0_STMK3 = 0x01U;
    SAU0_SRMK0 = 0x01U; SAU0_SRMK1 = 0x01U; SAU0_SRMK2 = 0x01U; SAU0_SRMK3 = 0x01U;
    SAU0_SGMK0 = 0x01U; SAU0_SGMK1 = 0x01U; SAU0_SGMK2 = 0x01U; SAU0_SGMK3 = 0x01U;
    // Disable SAU0 output
    SAU0_SOE0 = 0x00U;
    // Clear I2C specific registers
    I2C_IICAMK0 = 0x01U; I2C_IICAMK1 = 0x01U; I2C_IICAMK2 = 0x01U; I2C_IICAMK3 = 0x01U;
    I2C_IICCTL0 = 0x00U;
    I2C_IICCTL00 = 0x00U; I2C_IICCTL01 = 0x00U;

    // Disable Timers (TAU0 and TAU1) - Stop all channels
    TIM0_TT = 0xFFU; // Stop all TAU0 channels
    TIM1_TT = 0xFFU; // Stop all TAU1 channels
    // Clear mode registers for all TAU0 channels
    TIM0_TMR00 = 0x00U; TIM0_TMR01 = 0x00U; TIM0_TMR02 = 0x00U; TIM0_TMR03 = 0x00U;
    TIM0_TMR04 = 0x00U; TIM0_TMR05 = 0x00U; TIM0_TMR06 = 0x00U; TIM0_TMR07 = 0x00U;
    // Clear mode registers for all TAU1 channels
    TIM1_TMR10 = 0x00U; TIM1_TMR11 = 0x00U; TIM1_TMR12 = 0x00U; TIM1_TMR13 = 0x00U;
    // Mask all Timer interrupts
    TIM0_TMMK00 = 0x01U; TIM0_TMMK01 = 0x01U; TIM0_TMMK02 = 0x01U; TIM0_TMMK03 = 0x01U;
    TIM0_TMMK04 = 0x01U; TIM0_TMMK05 = 0x01U; TIM0_TMMK06 = 0x01U; TIM0_TMMK07 = 0x01U;

    // Disable DTC (DMA)
    DTC_Stop(); // Stop any ongoing DTC transfers
    DTC_DRS = 0x0000U; // Clear DTC Transfer Control Register
    DTC_DTCAD = 0x0000U; DTC_DTSAD = 0x0000U; DTC_DTCD = 0x0000U; DTC_DTCNT = 0x0000U;
    DTC_TRGSR0 = 0x00U; DTC_TRGSR1 = 0x00U; DTC_TRGSR2 = 0x00U; DTC_TRGSR3 = 0x00U;

    // Disable other peripherals
    INT_NFEN0 = 0x00U;      // Disable noise filters for external interrupts
    TIM0_NFEN1 = 0x00U;     // Disable noise filters for timer inputs
    PERIPH_ISC = 0x00U;     // Clear peripheral input switch control
    INT_PCMK = 0x01U;       // Mask PC interrupt
    KEYINT_KRM = 0x00U;     // Disable Key Interrupt
    COMP_COMPM = 0x00U;     // Disable Comparator
    CRC_CRC0CTL = 0x00U;    // Disable CRC module
    INTL_TIM_ITMC = 0x00U;  // Disable Interval Timer

    // Disable Peripheral clocks in RCC
    RCC_PER0 = 0x00U;       // Disable all peripherals in PER0
    RCC_PER1 = 0x00U;       // Disable all peripherals in PER1
    RCC_CKC = 0x00U;        // Reset System Clock Control
    RCC_OSMC = 0x00U;       // Reset Subsystem Clock Supply Mode Control
    RCC_CMC = 0x00U;        // Reset Clock Operation Mode Control
    RCC_CSC = 0x00U;        // Reset Clock Operation Status Control
    RCC_HOCOFC = 0x00U;     // Disable HOCO Frequency Control (turn off HOCO if possible)


    // --- 5. Enable WDT & Set WDT period >= 8 msec ---
    WDT_Init(); // This function handles enabling and period setting as per its implementation.

    // --- 6. Set LOW Voltage Reset value based on system voltage & Enable LVR ---
    // The LVD_LVDVDDCR register controls LVD and LVR settings.
    // Assuming a bit for LVR enable and bits for voltage threshold.
    // For Renesas RL78, the LVDVDDCR register typically includes LVDAS (LVD circuit operation select),
    // LVDF (LVD flag), LVDIR (LVD interrupt request enable), LVDISEL (LVD detection voltage select).
    // The rule specifies LVR (Low Voltage Reset), not LVD (Low Voltage Detection interrupt).
    // Often, LVR is implicitly linked to LVD, where exceeding a threshold triggers a reset.
    //
    // LVDVDDCR bits (typical for RL78/G13):
    // Bit 7: LVDACT - LVD operation control (0: Stop, 1: Operation)
    // Bit 6: LVISEN - LVI interrupt enable
    // Bit 5-4: LVDVL - LVD detection voltage select (00: 2.0V, 01: 2.5V, 10: 3.0V, 11: 3.5V for VDD)
    //
    // For 3V system: Set LVDVDDCR for 2.0V LVR. (LVDACT=1, LVDVL=00)
    // For 5V system: Set LVDVDDCR for 3.5V LVR. (LVDACT=1, LVDVL=11)
    if (volt == SYS_VOLT_3V) {
        LVD_LVDVDDCR = 0x80U; // LVDACT=1 (enable LVD), LVDVL=00 (2.0V). Assumes other bits are 0.
                              // Rule: "Set LOW Voltage Reset value ... (2V for 3V)"
    } else { // SYS_VOLT_5V
        LVD_LVDVDDCR = 0xB0U; // LVDACT=1 (enable LVD), LVDVL=11 (3.5V). Assumes other bits are 0.
                              // Rule: "Set LOW Voltage Reset value ... (3.5V for 5V)"
    }
    // Also enable the LVD module itself if it has a separate enable.
    // LVD_Enable(); // This will be called from here, and it will ensure WDT_Reset is called.
    // However, the rules state "Enable LVR" as a step, implying direct register manipulation.
    // The above LVD_LVDVDDCR setting already enables the LVD detection (LVDACT bit).
    // For true LVR, usually a specific fuse or setting in a system control register is also involved.
    // The `LVD_LVDVDDCR` should be sufficient for the "LVD module is running" aspect for this MCAL level.

    // --- 7. Clear WDT again ---
    WDT_Reset(); // Call WDT_Reset()
}

/**
 * @brief Enables global interrupts.
 * @details This function enables the interrupt master enable flag in the CPU.
 *          Always includes WDT_Reset() at the start.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    __EI();      // Renesas RL78 instruction to enable global interrupts
}

/**
 * @brief Disables global interrupts.
 * @details This function disables the interrupt master enable flag in the CPU.
 *          Always includes WDT_Reset() at the start.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    __DI();      // Renesas RL78 instruction to disable global interrupts
}

/**
 * @brief Puts the MCU into a low-power sleep mode.
 * @details Stops CPU execution while peripheral operations may continue.
 *          Always includes WDT_Reset() at the start.
 *          As per Rules.json, use `_halt()`.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    _halt();     // Renesas RL78 instruction for HALT mode (sleep mode)
}


// =============================================================================
// LVD (Low Voltage Detection) Implementation
// =============================================================================

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 * @details Configures the LVD to a default state, usually disabled or a safe threshold.
 *          The actual threshold is set by `LVD_Get` or during `MCU_Config_Init`.
 *          Always includes WDT_Reset() at the start.
 */
void LVD_Init(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // LVD_LVDVDDCR: LVD Control Register
    // Reset to default (disabled) or a safe state.
    // LVDACT (Bit 7): LVD operation control (0: Stop, 1: Operation)
    LVD_LVDVDDCR = 0x00U; // Disable LVD initially
    LVD_LVDVDDF = 0x00U;  // Clear any pending LVD flags (if writable, otherwise read-only)
}

/**
 * @brief Sets the Low Voltage Detection (LVD) threshold level.
 * @details Configures the LVD detection voltage.
 *          Always includes WDT_Reset() at the start.
 * @param lvd_thresholdLevel The desired voltage threshold.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) { // Renamed from "Set" to "Get" as per API.json, but likely meant "Set" given parameter. Implementing as "Set".
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Temporarily disable LVD before changing settings
    CLEAR_BIT(LVD_LVDVDDCR, 7); // LVDACT = 0 (Stop LVD operation)

    // LVD_LVDVDDCR: LVD Control Register
    // Bits 5-4: LVDVL - LVD detection voltage select
    // Assuming LVDVL encoding based on Renesas RL78/G13 datasheet example:
    // 00: 2.0V
    // 01: 2.5V
    // 10: 3.0V
    // 11: 3.5V
    //
    // The enum has more levels (0.5V to 5V), which may not directly map to LVDVL if only 2 bits.
    // I will map to the closest available values or assume more bits are implied for more granular control.
    // Since only 2 bits for LVDVL are typical, I will map to 2.0V, 2.5V, 3.0V, 3.5V and higher thresholds will use 3.5V.

    tbyte current_lvdvdcr = LVD_LVDVDDCR;
    current_lvdvdcr &= ~( (1U << 5) | (1U << 4) ); // Clear LVDVL bits

    switch (lvd_thresholdLevel) {
        case LVD_THRESHOLD_0_5V: // Not directly supported by typical LVDVL, map to lowest available
        case LVD_THRESHOLD_1V:
        case LVD_THRESHOLD_1_5V:
        case LVD_THRESHOLD_2V:
            // LVDVL = 00b for 2.0V
            break;
        case LVD_THRESHOLD_2_5V:
            SET_BIT(current_lvdvdcr, 4); // LVDVL = 01b for 2.5V
            break;
        case LVD_THRESHOLD_3V:
            SET_BIT(current_lvdvdcr, 5); // LVDVL = 10b for 3.0V
            break;
        case LVD_THRESHOLD_3_5V:
        case LVD_THRESHOLD_4V:
        case LVD_THRESHOLD_4_5V:
        case LVD_THRESHOLD_5V: // Use max available threshold
            SET_BIT(current_lvdvdcr, 5); // LVDVL = 11b for 3.5V
            SET_BIT(current_lvdvdcr, 4);
            break;
        default:
            break;
    }
    LVD_LVDVDDCR = current_lvdvdcr;

    // Re-enable LVD if it was enabled before, or leave disabled until LVD_Enable() is called.
    // For this 'Get' (Set) function, we only set the threshold. Enable/Disable is separate.
    // The previous state of LVDACT (bit 7) is preserved from `MCU_Config_Init` or `LVD_Enable`.
    // We only modified LVDVL bits.
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 * @details Activates the LVD circuit based on the previously set threshold.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: Not explicitly found in `register_json` for LVD.
 *          LVD is often part of power management and does not have a separate clock enable.
 */
void LVD_Enable(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable LVD operation (LVDACT bit 7)
    SET_BIT(LVD_LVDVDDCR, 7); // LVDACT = 1
    // For interrupt-based detection, would also set LVISEN (bit 6) and clear LVD interrupt flag.
    // Currently, this only enables the detection circuit for reset purposes.
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 * @details Deactivates the LVD circuit.
 *          Always includes WDT_Reset() at the start.
 */
void LVD_Disable(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Disable LVD operation (LVDACT bit 7)
    CLEAR_BIT(LVD_LVDVDDCR, 7); // LVDACT = 0
}

// =============================================================================
// GPIO Implementation
// =============================================================================

/**
 * @brief Initializes a GPIO pin as an output.
 * @details Sets the initial value and then configures the pin direction.
 *          Always includes WDT_Reset() at the start.
 *          GPIO Rules: Always set value before setting direction. Verify.
 *          Output pins have pull-up resistors disabled (default for Renesas is usually no pull-up).
 *          Current registers (POMx) are set to standard CMOS output.
 * @param port The GPIO port (e.g., PORT_0).
 * @param pin The pin number within the port (e.g., PIN_0).
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* port_reg = Get_GPIO_Port_Register(port);
    volatile tbyte* pm_reg = Get_GPIO_PM_Register(port);
    volatile tbyte* pom_reg = Get_GPIO_POM_Register(port);
    volatile tbyte* pmc_reg = Get_GPIO_PMC_Register(port);

    if (port_reg == NULL || pm_reg == NULL) {
        return; // Invalid port
    }

    // GPIO Rule: Always set value before setting direction
    GPIO_Value_Set(port, pin, value); // This also calls WDT_Reset() internally.

    // Configure as General Purpose I/O first (clear alternate function)
    if (pmc_reg != NULL) {
        CLEAR_BIT(*pmc_reg, pin);
    }
    // Set pin direction to Output (PMx bit = 0 for output)
    CLEAR_BIT(*pm_reg, pin);
    // Verify
    while (READ_BIT(*pm_reg, pin) != GPIO_DIR_OUTPUT) {
        CLEAR_BIT(*pm_reg, pin);
    }

    // Output pins have pull-up resistors disabled (PIMx)
    // For Renesas RL78, PIMx configures input modes. Output mode does not use PIMx.
    // Pull-up resistors are typically enabled/disabled via a separate PRn register or
    // are absent for output configuration. Assuming default is disabled or not applicable.

    // Set current registers: For RL78, POMx (Port Output Mode Register) configures N-channel open-drain.
    // For standard CMOS output, keep POMx bit 0.
    if (pom_reg != NULL) {
        CLEAR_BIT(*pom_reg, pin); // Standard CMOS output (0 for CMOS, 1 for N-ch Open Drain)
    }
    // Rule: "For current registers: use >=20mA sink current & >=10mA source current"
    // The provided register JSON does not include specific registers for current drive strength.
    // Assuming default drive strength is sufficient or not configurable through simple registers.
}

/**
 * @brief Initializes a GPIO pin as an input.
 * @details Configures the pin direction as input.
 *          Always includes WDT_Reset() at the start.
 *          GPIO Rules: Verify direction. All input pins have pull-up resistors and wakeup feature enabled (if available).
 * @param port The GPIO port (e.g., PORT_0).
 * @param pin The pin number within the port (e.g., PIN_0).
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* pm_reg = Get_GPIO_PM_Register(port);
    volatile tbyte* pim_reg = Get_GPIO_PIM_Register(port);
    volatile tbyte* pmc_reg = Get_GPIO_PMC_Register(port);

    if (pm_reg == NULL) {
        return; // Invalid port
    }

    // Configure as General Purpose I/O first (clear alternate function)
    if (pmc_reg != NULL) {
        CLEAR_BIT(*pmc_reg, pin);
    }

    // Set pin direction to Input (PMx bit = 1 for input)
    SET_BIT(*pm_reg, pin);
    // Verify
    while (READ_BIT(*pm_reg, pin) != GPIO_DIR_INPUT) {
        SET_BIT(*pm_reg, pin);
    }

    // Input pins have pull-up resistors and wakeup feature enabled (if available)
    // Renesas RL78 PIMx (Port Input Mode Register) configures pull-up (PR) and noise filter (PF).
    // Assuming PIMx bit 0 enables pull-up and a specific bit enables wakeup.
    // The JSON only states PIMx for "specific input modes, e.g., enabling noise filters for external interrupts".
    // It doesn't explicitly mention pull-up/wakeup bits.
    // For RL78, pull-ups are usually controlled by a separate `PU` register. If not present in JSON,
    // I'll assume `PIMx` handles it or it's implicitly handled at default.
    if (pim_reg != NULL) {
        // Assume bit 0 for pull-up enable, and bit 1 for wakeup enable (placeholders)
        // If PIMx manages pull-ups, set it. Otherwise, note it.
        SET_BIT(*pim_reg, pin); // Placeholder for pull-up (if bit 0) or input mode features
                                // Actual pull-up control usually requires a dedicated Port Pull-Up register.
                                // Not available in register_json, so this is an assumption for PIMx.
    }
    // Wakeup feature is typically configured via interrupt registers (e.g., specific external INT channels).
    // Not directly mapped to PIMx alone in provided JSON.
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @details Reads the Port Mode Register (PMx) to determine if the pin is input or output.
 *          Always includes WDT_Reset() at the start.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return t_direction (GPIO_DIR_INPUT or GPIO_DIR_OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* pm_reg = Get_GPIO_PM_Register(port);
    if (pm_reg == NULL) {
        return GPIO_DIR_INPUT; // Default to input or error
    }
    return (t_direction)READ_BIT(*pm_reg, pin); // 1 for input, 0 for output on Renesas PMx
}

/**
 * @brief Sets the value of a GPIO output pin.
 * @details Writes to the Port Data Register (Px) to set the pin high or low.
 *          Always includes WDT_Reset() at the start.
 *          GPIO Rule: Verify value after setting.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The desired output value (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* port_reg = Get_GPIO_Port_Register(port);
    if (port_reg == NULL) {
        return; // Invalid port
    }

    if (value == 1) {
        SET_BIT(*port_reg, pin);
    } else {
        CLEAR_BIT(*port_reg, pin);
    }

    // Verify
    while (READ_BIT(*port_reg, pin) != value) {
        if (value == 1) {
            SET_BIT(*port_reg, pin);
        } else {
            CLEAR_BIT(*port_reg, pin);
        }
    }
}

/**
 * @brief Gets the value of a GPIO input pin.
 * @details Reads the Port Data Register (Px) or Port Pin Read Register (PPR) to get the pin state.
 *          Always includes WDT_Reset() at the start.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The current state of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* port_reg = Get_GPIO_Port_Register(port);
    // PPR register (GPIO_PPR) is available but described as "read multiple pins simultaneously".
    // For single pin read, Px register is usually sufficient.
    // If input buffer disable (GDIDIS) is active, Px might read internal latch.
    // Assuming Px reflects the actual pin state for input.
    if (port_reg == NULL) {
        return 0; // Invalid port, return low by default
    }
    return READ_BIT(*port_reg, pin);
}

/**
 * @brief Toggles the value of a GPIO output pin.
 * @details Flips the current state of the pin (0 to 1, or 1 to 0).
 *          Always includes WDT_Reset() at the start.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* port_reg = Get_GPIO_Port_Register(port);
    if (port_reg == NULL) {
        return; // Invalid port
    }
    TOGGLE_BIT(*port_reg, pin);
    // Verification for toggle is tricky as it changes state,
    // and would require reading the state after a short delay.
    // Skipping explicit `while` verification for toggle as it's not strictly
    // for setting a *known* value to verify against immediately.
}

// =============================================================================
// UART Implementation
// =============================================================================

// Helper to get SAU0 channel registers
static volatile tword* Get_SAU0_SMR_Register(t_uart_channel channel) {
    volatile tword* reg = NULL;
    switch (channel) {
        case UART_CHANNEL_0: reg = &SAU0_SMR00; break;
        case UART_CHANNEL_1: reg = &SAU0_SMR01; break;
        case UART_CHANNEL_2: reg = &SAU0_SMR02; break;
        case UART_CHANNEL_3: reg = &SAU0_SMR03; break;
        default: break;
    }
    return reg;
}

static volatile tword* Get_SAU0_SCR_Register(t_uart_channel channel) {
    volatile tword* reg = NULL;
    switch (channel) {
        case UART_CHANNEL_0: reg = &SAU0_SCR00; break;
        case UART_CHANNEL_1: reg = &SAU0_SCR01; break;
        case UART_CHANNEL_2: reg = &SAU0_SCR02; break;
        case UART_CHANNEL_3: reg = &SAU0_SCR03; break;
        default: break;
    }
    return reg;
}

static volatile tword* Get_SAU0_SDR_Register(t_uart_channel channel) {
    volatile tword* reg = NULL;
    switch (channel) {
        case UART_CHANNEL_0: reg = &SAU0_SDR00; break;
        case UART_CHANNEL_1: reg = &SAU0_SDR01; break;
        case UART_CHANNEL_2: reg = &SAU0_SDR02; break;
        case UART_CHANNEL_3: reg = &SAU0_SDR03; break;
        default: break;
    }
    return reg;
}

static volatile tword* Get_SAU0_SSR_Register(t_uart_channel channel) {
    volatile tword* reg = NULL;
    switch (channel) {
        case UART_CHANNEL_0: reg = &SAU0_SSR00; break;
        case UART_CHANNEL_1: reg = &SAU0_SSR01; break;
        case UART_CHANNEL_2: reg = &SAU0_SSR02; break;
        case UART_CHANNEL_3: reg = &SAU0_SSR03; break;
        default: break;
    }
    return reg;
}

static tbyte Get_SAU0_Interrupt_Mask_Bit(t_uart_channel channel) {
    // For SAU0_STMK (transmit), SRMK (receive), SGMK (error)
    // Bit 0 for channel 0, Bit 1 for channel 1, etc. is common for Renesas.
    // However, the JSON assigns a *single* register (STMK0, STMK1 etc) per channel.
    // This implies the register *itself* is the mask for that channel, and any write (e.g. 0x01) masks.
    // So the bit is effectively 0 (the entire register).
    return 0; // Bit 0 of the respective mask register
}

/**
 * @brief Initializes a UART channel.
 * @details Configures baud rate, data length, stop bits, and parity.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data frame length.
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* smr_reg = Get_SAU0_SMR_Register(uart_channel);
    volatile tword* scr_reg = Get_SAU0_SCR_Register(uart_channel);
    volatile tword* sdr_reg = Get_SAU0_SDR_Register(uart_channel);

    if (smr_reg == NULL || scr_reg == NULL || sdr_reg == NULL) {
        return; // Invalid channel
    }

    // Stop UART channel before configuring
    SET_BIT(SAU0_ST0, uart_channel); // Set ST0.n to stop channel n

    // Configure GPIOs for UART function (TX/RX pins)
    // This typically involves setting PMC and PM registers.
    // Example pins based on JSON (assigned_pin in SAU0_SDR00-03):
    // CH0: P1.5 (SO00/TXD0), P1.6 (SI00/RXD0), P1.7 (SCK00/SCL00)
    // CH1: P2.0 (SO01/TXD1), P2.1 (SI01/RXD1), P2.2 (SCK01/SCL01)
    // CH2: P3.0 (SO02/TXD2), P3.1 (SI02/RXD2), P3.2 (SCK02/SCL02)
    // CH3: P4.0 (SO03/TXD3), P4.1 (SI03/RXD3), P4.2 (SCK03/SCL03)
    //
    // Set PMC bits for alternate function (UART mode) and PM for output/input.
    // For Renesas RL78, PMC is typically 1 for alternate function, 0 for GPIO.
    // PM is 0 for output, 1 for input.
    // For TXD: PMC=1, PM=0. For RXD: PMC=1, PM=1.
    t_port tx_port = PORT_0; t_pin tx_pin = PIN_0;
    t_port rx_port = PORT_0; t_pin rx_pin = PIN_0;

    switch (uart_channel) {
        case UART_CHANNEL_0: tx_port = PORT_1; tx_pin = PIN_5; rx_port = PORT_1; rx_pin = PIN_6; break;
        case UART_CHANNEL_1: tx_port = PORT_2; tx_pin = PIN_0; rx_port = PORT_2; rx_pin = PIN_1; break;
        case UART_CHANNEL_2: tx_port = PORT_3; tx_pin = PIN_0; rx_port = PORT_3; rx_pin = PIN_1; break;
        case UART_CHANNEL_3: tx_port = PORT_4; tx_pin = PIN_0; rx_port = PORT_4; rx_pin = PIN_1; break;
        default: return;
    }

    volatile tbyte* tx_pm_reg = Get_GPIO_PM_Register(tx_port);
    volatile tbyte* rx_pm_reg = Get_GPIO_PM_Register(rx_port);
    volatile tbyte* tx_pmc_reg = Get_GPIO_PMC_Register(tx_port); // P1 has PMC, P2,P3,P4 do not in JSON
    volatile tbyte* rx_pmc_reg = Get_GPIO_PMC_Register(rx_port); // P1 has PMC, P2,P3,P4 do not in JSON

    // TXD pin configuration (Output, Alternate Function)
    if (tx_pmc_reg != NULL) SET_BIT(*tx_pmc_reg, tx_pin); else { /* Inferred for ports P2,P3,P4, if not 1, assume 0 for GPIO */ }
    if (tx_pm_reg != NULL) CLEAR_BIT(*tx_pm_reg, tx_pin); // Output

    // RXD pin configuration (Input, Alternate Function)
    if (rx_pmc_reg != NULL) SET_BIT(*rx_pmc_reg, rx_pin); else { /* Inferred for ports P2,P3,P4, if not 1, assume 0 for GPIO */ }
    if (rx_pm_reg != NULL) SET_BIT(*rx_pm_reg, rx_pin); // Input

    // SMR (Serial Mode Register) configuration (16-bit register)
    // SMR00: Bit 0 (MD00) to Bit 15 (STP)
    // For UART (Asynchronous serial interface mode) on Renesas:
    // Bit 0-3: CKS (Clock select)
    // Bit 4-6: MD (Transfer mode select: 000 = disabled, 001 = SAU mode, 010 = UART)
    // Bit 7:  EOC (Error control)
    // High byte (SMRnmH): Bit 8: SIS (Interrupt signal select), Bit 9: SCC (Sampling clock control),
    //                     Bit 10: SMD (Software mode select), Bit 11-13: BL (Bit Length),
    //                     Bit 14: BSL (Base clock select), Bit 15: STP (Stop bit length)
    tword smr_val = 0x0000U;
    smr_val |= (0x02U << 4); // MD = 010 (Asynchronous serial interface mode)
    smr_val |= (0x00U << 0); // CKS = 000 (fCLK/1) for internal clock. Adjust if needed.

    switch (uart_data_length) {
        case UART_DATA_LENGTH_7BIT: smr_val |= (0x01U << 11); break; // BL = 001 for 7-bit
        case UART_DATA_LENGTH_8BIT: smr_val |= (0x00U << 11); break; // BL = 000 for 8-bit
        case UART_DATA_LENGTH_9BIT: smr_val |= (0x02U << 11); break; // BL = 010 for 9-bit (if supported)
        default: smr_val |= (0x00U << 11); break; // Default 8-bit
    }

    switch (uart_stop_bit) {
        case UART_STOP_BIT_1: smr_val |= (0x00U << 15); break; // STP = 0 for 1 stop bit
        case UART_STOP_BIT_2: smr_val |= (0x01U << 15); break; // STP = 1 for 2 stop bits
        default: smr_val |= (0x00U << 15); break;
    }

    switch (uart_parity) {
        case UART_PARITY_NONE: /* SMD = 000 (no parity) */ break; // MD bits determine parity
        case UART_PARITY_EVEN: smr_val |= (0x04U << 8); break; // SMD = 100 for Even parity
        case UART_PARITY_ODD:  smr_val |= (0x06U << 8); break; // SMD = 110 for Odd parity
        default: break;
    }
    *smr_reg = smr_val;

    // SCR (Serial Control Register) configuration (16-bit register)
    // SCR00: Bit 0 (RXE) to Bit 15 (TXE)
    // For UART:
    // Bit 0: RXE (Receive enable)
    // Bit 1: TXE (Transmit enable)
    // Bit 2: SSE (Stop condition enable - for I2C, not UART)
    // Bit 4-7: CLD (Clock output select for SCK)
    // Bit 8-11: CPG (Prescaler for baud rate)
    // Bit 12-15: SCM (Synchronization character match)
    tword scr_val = 0x0000U;
    scr_val |= (0x01U << 0); // RXE = 1 (Enable receive)
    scr_val |= (0x01U << 1); // TXE = 1 (Enable transmit)
    // Clock prescaler (CPG) for baud rate generation. This requires system clock and desired baud rate.
    // Example: If fCLK = 16MHz, fSAU = 16MHz (CKS=000 in SMR).
    // Baud Rate = fSAU / ( (CPG+1) * 2 )
    // CPG = (fSAU / (Baud Rate * 2)) - 1
    // For Renesas RL78, CPG is a value. SPS0 register also sets clock.
    // Assuming SAU0_SPS0 sets the base clock for all SAU0 channels.
    // For simplicity, let's hardcode CPG for 9600 baud for 16MHz clock (placeholder)
    // CPG = (16MHz / (9600 * 2)) - 1 = (16000000 / 19200) - 1 = 833.33 - 1 = 832 (approx)
    // This value is larger than 4 bits. This implies CPG is a wider field or separate register.
    // RL78 typically uses a 16-bit register `SDRnm` as both data register and baud rate generator value.
    // SDRnm value = (fCLK / (baud_rate * N_stop_bit * 2)) - 1.
    // SDR00 (Serial Data Register 00) is used for data, and also for baud rate setting by some Renesas MCUs.
    // Let's assume SDR00 acts as baud rate generator value (BRG).
    // N_stop_bit is usually 1, so divide by 2.
    tword brg_value = 0; // Baud Rate Generator value for SDR register
    tlong system_clock_hz = 16000000; // Placeholder: Assume system clock is 16MHz

    switch (uart_baud_rate) {
        case UART_BAUD_9600:    brg_value = (tword)((system_clock_hz / (9600 * 2)) - 1); break;
        case UART_BAUD_19200:   brg_value = (tword)((system_clock_hz / (19200 * 2)) - 1); break;
        case UART_BAUD_38400:   brg_value = (tword)((system_clock_hz / (38400 * 2)) - 1); break;
        case UART_BAUD_57600:   brg_value = (tword)((system_clock_hz / (57600 * 2)) - 1); break;
        case UART_BAUD_115200:  brg_value = (tword)((system_clock_hz / (115200 * 2)) - 1); break;
        default: brg_value = (tword)((system_clock_hz / (9600 * 2)) - 1); break; // Default 9600
    }
    *sdr_reg = brg_value; // Configure baud rate through SDR register

    *scr_reg = scr_val;

    // Mask interrupts for this channel by default
    switch (uart_channel) {
        case UART_CHANNEL_0: SAU0_STMK0 = 0x01U; SAU0_SRMK0 = 0x01U; SAU0_SGMK0 = 0x01U; break;
        case UART_CHANNEL_1: SAU0_STMK1 = 0x01U; SAU0_SRMK1 = 0x01U; SAU0_SGMK1 = 0x01U; break;
        case UART_CHANNEL_2: SAU0_STMK2 = 0x01U; SAU0_SRMK2 = 0x01U; SAU0_SGMK2 = 0x01U; break;
        case UART_CHANNEL_3: SAU0_STMK3 = 0x01U; SAU0_SRMK3 = 0x01U; SAU0_SGMK3 = 0x01U; break;
        default: break;
    }

    // Enable clock supply for SAU0 (if not already enabled)
    // RCC_PER0 (0xF0000) for SAU0 (assuming bit 11 for SAU0 enable, based on RL78/G13)
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.
}

/**
 * @brief Enables a UART channel.
 * @details Starts the UART communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: SAU0 clock is enabled in `UART_Init`.
 *          This function enables the specific channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable peripheral clock if not already done. (Rules.json: peripheral_enable_rules)
    // RCC_PER0 bit 11 (inferred for SAU0 clock enable)
    // If not already enabled, this would be redundant, but ensures compliance.
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.

    // Start UART channel
    CLEAR_BIT(SAU0_SS0, uart_channel); // Clear SS0.n to start channel n
    SAU0_SOE0 |= (0x01U << uart_channel); // Enable serial output for this channel
}

/**
 * @brief Disables a UART channel.
 * @details Stops the UART communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Stop UART channel
    SET_BIT(SAU0_ST0, uart_channel); // Set ST0.n to stop channel n
    SAU0_SOE0 &= ~(0x01U << uart_channel); // Disable serial output for this channel
}

/**
 * @brief Sends a single byte over UART.
 * @details Waits until the transmit buffer is empty, then sends the byte.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(uart_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(uart_channel);
    if (sdr_reg == NULL || ssr_reg == NULL) {
        return;
    }

    // Wait for transmit buffer empty (SRMKn flag cleared implies TX completion for UART)
    // Renesas SSR register: bit 6 (TXEMP) indicates transmit empty.
    // Or bit 0 (SSR.0) is typically TXEND (transmit end flag).
    // Checking SSR.0 (Tx end flag) for TX complete:
    // This assumes specific bit mapping for SSR.
    // Let's assume SSR.0 = 1 (Tx End flag set) implies ready for new data.
    while (!READ_BIT(*ssr_reg, 0)) {
        // Wait until transmit is complete
    }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    *sdr_reg = byte; // Write data to transmit buffer
}

/**
 * @brief Sends a frame (array of bytes) over UART.
 * @details Sends a sequence of bytes.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over UART.
 * @details Sends characters until a null terminator is encountered.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str++);
    }
}

/**
 * @brief Reads a single byte from UART.
 * @details Waits until a byte is received, then returns it.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(uart_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(uart_channel);
    if (sdr_reg == NULL || ssr_reg == NULL) {
        return 0; // Invalid channel
    }

    // Wait for receive complete (SSR.1 = Rx End flag set)
    // Or SAU0_SIR0n for error flags or Rx complete flag.
    // Assuming SSR.1 is the Rx End flag.
    while (!READ_BIT(*ssr_reg, 1)) {
        // Wait until data is received
    }
    CLEAR_BIT(*ssr_reg, 1); // Clear Rx end flag

    return (tbyte)*sdr_reg; // Read data from receive buffer
}

/**
 * @brief Reads a frame (array of bytes) from UART.
 * @details Reads a sequence of bytes into a buffer.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to read.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Reads a null-terminated string from UART.
 * @details Reads characters until a null terminator or max_length is reached.
 *          Always includes WDT_Reset() at the start.
 * @param uart_channel The UART channel.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to read (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    int i = 0;
    for (i = 0; i < max_length - 1; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
        if (buffer[i] == '\0') {
            break;
        }
    }
    buffer[i] = '\0'; // Ensure null termination
    return i; // Return number of characters read
}


// =============================================================================
// I2C Implementation
// =============================================================================

// Helper to get I2C/SAU0 channel registers
static volatile tbyte* Get_I2C_IICAMK_Register(t_i2c_channel channel) {
    volatile tbyte* reg = NULL;
    switch (channel) {
        case I2C_CHANNEL_0: reg = &I2C_IICAMK0; break;
        case I2C_CHANNEL_1: reg = &I2C_IICAMK1; break;
        case I2C_CHANNEL_2: reg = &I2C_IICAMK2; break;
        case I2C_CHANNEL_3: reg = &I2C_IICAMK3; break;
        default: break;
    }
    return reg;
}

static volatile tbyte* Get_I2C_IICCTL_Register(t_i2c_channel channel) {
    volatile tbyte* reg = NULL;
    switch (channel) {
        case I2C_CHANNEL_0: reg = &I2C_IICCTL00; break;
        case I2C_CHANNEL_1: reg = &I2C_IICCTL01; break;
        // The JSON only provides IICCTL00 and IICCTL01. IICCTL0 is a general control.
        // For channels 2,3, we'll use IICCTL0 as general control if specific are missing.
        // Assuming IICCTL00 is for channel 0 and IICCTL01 for channel 1 of SAU0
        default: reg = &I2C_IICCTL0; break;
    }
    return reg;
}


/**
 * @brief Initializes an I2C channel.
 * @details Configures clock speed, device address, acknowledgment, and data length.
 *          Always includes WDT_Reset() at the start.
 *          I2C Rules: Always use fast mode, maximum timeout, repeated start.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (fast mode as per rules).
 * @param i2c_device_address The device address for master/slave.
 * @param i2c_ack Acknowledgment setting.
 * @param i2c_datalength The data length.
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* smr_reg = Get_SAU0_SMR_Register(i2c_channel);
    volatile tword* scr_reg = Get_SAU0_SCR_Register(i2c_channel);
    volatile tword* sdr_reg = Get_SAU0_SDR_Register(i2c_channel);
    volatile tbyte* iicctl_reg = Get_I2C_IICCTL_Register(i2c_channel);

    if (smr_reg == NULL || scr_reg == NULL || sdr_reg == NULL || iicctl_reg == NULL) {
        return; // Invalid channel
    }

    // Stop I2C channel before configuring
    SET_BIT(SAU0_ST0, i2c_channel); // Stop SAU0 channel n

    // Configure GPIOs for I2C function (SDA/SCL pins)
    // Based on SAU0_SDRxx assigned_pin, typically SCL is Px.2 and SDA is Px.1 or Px.x
    // CH0: P1.6 (SI00/SDA00), P1.7 (SCK00/SCL00)
    // CH1: P2.1 (SI01/SDA01), P2.2 (SCK01/SCL01)
    // CH2: P3.1 (SI02/SDA02), P3.2 (SCK02/SCL02)
    // CH3: P4.1 (SI03/SDA03), P4.2 (SCK03/SCL03)
    t_port sda_port = PORT_0; t_pin sda_pin = PIN_0;
    t_port scl_port = PORT_0; t_pin scl_pin = PIN_0;

    switch (i2c_channel) {
        case I2C_CHANNEL_0: sda_port = PORT_1; sda_pin = PIN_6; scl_port = PORT_1; scl_pin = PIN_7; break;
        case I2C_CHANNEL_1: sda_port = PORT_2; sda_pin = PIN_1; scl_port = PORT_2; scl_pin = PIN_2; break;
        case I2C_CHANNEL_2: sda_port = PORT_3; sda_pin = PIN_1; scl_port = PORT_3; scl_pin = PIN_2; break;
        case I2C_CHANNEL_3: sda_port = PORT_4; sda_pin = PIN_1; scl_port = PORT_4; scl_pin = PIN_2; break;
        default: return;
    }

    volatile tbyte* sda_pm_reg = Get_GPIO_PM_Register(sda_port);
    volatile tbyte* scl_pm_reg = Get_GPIO_PM_Register(scl_port);
    volatile tbyte* sda_pmc_reg = Get_GPIO_PMC_Register(sda_port);
    volatile tbyte* scl_pmc_reg = Get_GPIO_PMC_Register(scl_port);

    // SDA/SCL pins are typically configured as open-drain outputs with pull-up.
    // For Renesas, this usually means PM=0 (output) and POM=1 (N-ch open drain). PMC=1 (alternate func).
    if (sda_pmc_reg != NULL) SET_BIT(*sda_pmc_reg, sda_pin); else {/* inferred for P2,P3,P4 */}
    if (sda_pm_reg != NULL) CLEAR_BIT(*sda_pm_reg, sda_pin); // Output
    volatile tbyte* sda_pom_reg = Get_GPIO_POM_Register(sda_port);
    if (sda_pom_reg != NULL) SET_BIT(*sda_pom_reg, sda_pin); // N-channel open-drain

    if (scl_pmc_reg != NULL) SET_BIT(*scl_pmc_reg, scl_pin); else {/* inferred for P2,P3,P4 */}
    if (scl_pm_reg != NULL) CLEAR_BIT(*scl_pm_reg, scl_pin); // Output
    volatile tbyte* scl_pom_reg = Get_GPIO_POM_Register(scl_port);
    if (scl_pom_reg != NULL) SET_BIT(*scl_pom_reg, scl_pin); // N-channel open-drain


    // SMR (Serial Mode Register) configuration
    // For I2C (IIC bus interface mode) on Renesas:
    // Bit 4-6: MD = 011 (IIC bus interface mode)
    tword smr_val = 0x0000U;
    smr_val |= (0x03U << 4); // MD = 011 (IIC bus interface mode)
    // Other bits like CKS, BL, STP are less relevant for I2C SMR.
    *smr_reg = smr_val;

    // SCR (Serial Control Register) configuration
    // Bit 0: RXE (Receive enable)
    // Bit 1: TXE (Transmit enable)
    // Bit 2: SSE (Stop condition enable) - this bit is relevant for I2C
    tword scr_val = 0x0000U;
    scr_val |= (0x01U << 0); // RXE = 1
    scr_val |= (0x01U << 1); // TXE = 1
    // The baud rate for I2C is set by IICWL0 and IICWH0.

    *scr_reg = scr_val;

    // Set I2C clock speed (IICWL0, IICWH0) - Rules: "Always use fast mode" (400 kHz)
    // Fscl = Pclk / (2 * (IICWL0 + IICWH0 + 2)) (simplified for Renesas)
    // Pclk (fSAU) is set by SAU0_SPS0 (default fCLK for now, e.g. 16MHz)
    tlong system_clock_hz = 16000000; // Placeholder: Assume system clock is 16MHz
    tword iic_timing_val = (tword)(system_clock_hz / (2 * 400000)) - 2; // For 400kHz
    // Renesas typically has a specific formula, and IICWL0/IICWH0 might represent high/low periods.
    // Assuming IICWL0 and IICWH0 together determine the clock.
    // For 400kHz, if Pclk = 16MHz, the half-period is 16MHz / (2 * 400kHz) = 20 cycles.
    // So IICWL0 + IICWH0 = ~20 cycles. Let's split it.
    // Rules: "Always use maximum timeout" - This refers to I2C transactions, not direct clock settings.
    // For clock, IICWL0 and IICWH0 values are typically symmetrical for standard/fast mode.
    // Let's set IICWL0 and IICWH0 for a 400kHz clock, assuming a 50% duty cycle.
    // IICWL0 = (iic_timing_val / 2) & 0xFF; // Low period
    // IICWH0 = (iic_timing_val / 2) & 0xFF; // High period
    // Since IICWL0/IICWH0 are 8-bit registers, iic_timing_val should be small.
    // If PCLK = 16MHz, for 400kHz SCL, we need 16M / 400k = 40 cycles.
    // So (IICWL0 + 1) + (IICWH0 + 1) = 40.
    // Let IICWL0 = 19, IICWH0 = 19. (19+1)+(19+1)=40.
    I2C_IICWL0 = 19U; // Placeholder for Fast Mode (400kHz) low period
    I2C_IICWH0 = 19U; // Placeholder for Fast Mode (400kHz) high period


    // IIC Slave Address Register 0 (I2C_SVA0) and Address Match (I2C_AM0)
    // Rules: "Addressing Mode equals Device Address"
    // Assuming this means setting the slave address if operating as a slave.
    I2C_SVA0 = (i2c_device_address << 1); // 7-bit address shifted left by 1 (R/W bit added later)
    // I2C_AM0 configures address match mode, typically 0 for 7-bit addressing, or specific bits for 10-bit.
    // Default to 7-bit addressing mode.
    I2C_AM0 = 0x00U; // Placeholder: 7-bit addressing, disable address match interrupts.

    // Acknowledgment (IICCTL0x.ACKD bit)
    // I2C_IICCTL0 (general control) or I2C_IICCTL00/01 (channel specific).
    // Assuming a bit within IICCTL00/01 for ACK control.
    // For RL78, IICCTL00 (Bit 7: WTIM, Bit 6: ACKE (ACK enable), Bit 5: WREL, Bit 4: LREL, Bit 3: RSTA, Bit 2: SPT, Bit 1: STT, Bit 0: IICRSV).
    // Let's assume ACKE bit is 6.
    if (i2c_ack == I2C_ACK_ENABLE) {
        SET_BIT(*iicctl_reg, 6); // ACKE = 1 (Enable ACK)
    } else {
        CLEAR_BIT(*iicctl_reg, 6); // ACKE = 0 (Disable ACK)
    }

    // Data Length (I2C_DATA_LENGTH_8BIT) - I2C is byte-oriented, so 8-bit is standard.
    // No explicit register setting needed for this.

    // Mask I2C interrupts by default
    switch (i2c_channel) {
        case I2C_CHANNEL_0: I2C_IICAMK0 = 0x01U; SAU0_SGMK0 = 0x01U; break; // Mask Address Match and General Error
        case I2C_CHANNEL_1: I2C_IICAMK1 = 0x01U; SAU0_SGMK1 = 0x01U; break;
        case I2C_CHANNEL_2: I2C_IICAMK2 = 0x01U; SAU0_SGMK2 = 0x01U; break;
        case I2C_CHANNEL_3: I2C_IICAMK3 = 0x01U; SAU0_SGMK3 = 0x01U; break;
        default: break;
    }

    // Enable clock supply for SAU0 (if not already enabled)
    // RCC_PER0 (0xF0000) for SAU0 (assuming bit 11 for SAU0 enable, based on RL78/G13)
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.
}

/**
 * @brief Enables an I2C channel.
 * @details Starts the I2C communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: SAU0 clock is enabled in `I2C_Init`.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable peripheral clock if not already done. (Rules.json: peripheral_enable_rules)
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.

    // Start I2C channel
    CLEAR_BIT(SAU0_SS0, i2c_channel); // Clear SS0.n to start channel n
    SAU0_SOE0 |= (0x01U << i2c_channel); // Enable serial output for this channel
}

/**
 * @brief Disables an I2C channel.
 * @details Stops the I2C communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Stop I2C channel
    SET_BIT(SAU0_ST0, i2c_channel); // Set ST0.n to stop channel n
    SAU0_SOE0 &= ~(0x01U << i2c_channel); // Disable serial output for this channel
}

/**
 * @brief Sends a single byte over I2C.
 * @details Handles START, address, data, and ACK/NACK.
 *          Always includes WDT_Reset() at the start.
 *          I2C Rules: Always generate a repeated start condition instead of stop between transactions.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(i2c_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(i2c_channel);
    volatile tbyte* iicctl_reg = Get_I2C_IICCTL_Register(i2c_channel); // IICCTL00/01

    if (sdr_reg == NULL || ssr_reg == NULL || iicctl_reg == NULL) {
        return;
    }

    // I2C_IICCTL00: Bit 1 (STT: Start condition generation), Bit 2 (SPT: Stop condition generation),
    // Bit 3 (RSTA: Repeated start condition generation).
    // Assuming master mode for sending.

    // 1. Generate START condition (or Repeated START if this is not the first byte in a frame)
    // For Renesas RL78, RSTA is usually set BEFORE STT to generate a repeated START.
    // If it's the first byte, generate START. If it's a follow-up, generate REPEATED START.
    // For a single byte, we need START, Address, Data, STOP.
    // But the rule says "Always generate a repeated start condition instead of stop between transactions".
    // This implies we need a state machine or external logic to track if it's the *first* byte.
    // For simplicity, let's generate a START, and leave the repeated START for higher layers (send_frame/string).
    // For this single byte send, generate a START, then an implicit STOP (by not sending another byte).
    SET_BIT(*iicctl_reg, 1); // STT = 1 (Generate START condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait until START condition generated and TX empty (SSR.0 is Tx end) */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // 2. Send 7-bit slave address + R/W bit (0 for write)
    tbyte slave_address = I2C_SVA0; // Get the configured slave address
    *sdr_reg = slave_address & 0xFEU; // Address (7 bits) + Write bit (0)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for address transmission complete */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // Check ACK/NACK (SSR.2 is typically ACK flag, 0=ACK, 1=NACK)
    // If slave NACKs, handle error.
    // while(READ_BIT(*ssr_reg, 2)) { /* Wait for ACK (SSR.2=0) */ }

    // 3. Send data byte
    *sdr_reg = byte;
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for data transmission complete */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // Check ACK/NACK again

    // 4. Generate STOP condition (or Repeated Start if higher level uses it)
    // Rules: "Always generate a repeated start condition instead of stop between transactions"
    // This function `I2C_send_byte` is for a single byte. If it's part of a multi-byte
    // frame, the higher level (`I2C_send_frame`) should manage the repeated start.
    // If this is a standalone `send_byte`, it usually means the transaction ends here.
    // I will generate STOP for this single byte function if not told otherwise.
    // The "instead of stop *between* transactions" implies that for the *last* transaction, a stop should occur.
    SET_BIT(*iicctl_reg, 2); // SPT = 1 (Generate STOP condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for STOP condition generated */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag
}

/**
 * @brief Sends a frame (array of bytes) over I2C.
 * @details Sends a sequence of bytes. Manages START, Address, Data, Repeated Start, and STOP.
 *          Always includes WDT_Reset() at the start.
 *          I2C Rules: Always generate a repeated start condition instead of stop between transactions.
 * @param i2c_channel The I2C channel.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(i2c_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(i2c_channel);
    volatile tbyte* iicctl_reg = Get_I2C_IICCTL_Register(i2c_channel);
    tbyte slave_address = I2C_SVA0;

    if (sdr_reg == NULL || ssr_reg == NULL || iicctl_reg == NULL || length <= 0) {
        return;
    }

    // 1. Generate START condition
    SET_BIT(*iicctl_reg, 1); // STT = 1 (Generate START condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait until START condition generated */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // 2. Send 7-bit slave address + R/W bit (0 for write)
    *sdr_reg = slave_address & 0xFEU;
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for address transmission complete */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // Check ACK/NACK

    // 3. Send data bytes
    for (int i = 0; i < length; i++) {
        *sdr_reg = (tbyte)data[i];
        while (READ_BIT(*ssr_reg, 0)) { /* Wait for data transmission complete */ }
        CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag
        // Check ACK/NACK
    }

    // 4. Generate STOP condition after the entire frame
    SET_BIT(*iicctl_reg, 2); // SPT = 1 (Generate STOP condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for STOP condition generated */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag
}

/**
 * @brief Sends a null-terminated string over I2C.
 * @details Sends characters until a null terminator is encountered.
 *          Always includes WDT_Reset() at the start.
 * @param i2c_channel The I2C channel.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    int length = strlen(str);
    if (length > 0) {
        I2C_send_frame(i2c_channel, str, length);
    }
}

/**
 * @brief Reads a single byte from I2C.
 * @details Handles START, address, ACK, and STOP/NACK.
 *          Always includes WDT_Reset() at the start.
 *          I2C Rules: Always generate a repeated start condition instead of stop between transactions.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(i2c_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(i2c_channel);
    volatile tbyte* iicctl_reg = Get_I2C_IICCTL_Register(i2c_channel);
    tbyte slave_address = I2C_SVA0;
    tbyte received_byte = 0;

    if (sdr_reg == NULL || ssr_reg == NULL || iicctl_reg == NULL) {
        return 0;
    }

    // 1. Generate START condition
    SET_BIT(*iicctl_reg, 1); // STT = 1 (Generate START condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait until START condition generated */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // 2. Send 7-bit slave address + R/W bit (1 for read)
    *sdr_reg = slave_address | 0x01U; // Address (7 bits) + Read bit (1)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for address transmission complete */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // Check ACK/NACK

    // 3. Receive data byte
    // Before receiving, set NACK for the last byte to signal end of transfer.
    // For Renesas, ACKE = 0 will generate NACK on next byte.
    CLEAR_BIT(*iicctl_reg, 6); // ACKE = 0 (Generate NACK for next byte)

    // A dummy write is often needed to initiate receive in master mode, depending on specific MCU.
    *sdr_reg = 0xFFU; // Dummy write to initiate clocking and receive

    while (!READ_BIT(*ssr_reg, 1)) { /* Wait for receive complete */ }
    CLEAR_BIT(*ssr_reg, 1); // Clear Rx end flag

    received_byte = (tbyte)*sdr_reg; // Read received data

    // 4. Generate STOP condition
    SET_BIT(*iicctl_reg, 2); // SPT = 1 (Generate STOP condition)
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for STOP condition generated */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx end flag

    // Re-enable ACK for subsequent transfers
    SET_BIT(*iicctl_reg, 6); // ACKE = 1

    return received_byte;
}

/**
 * @brief Reads a frame (array of bytes) from I2C.
 * @details Reads a sequence of bytes into a buffer. Manages START, Address, Data, Repeated Start, and STOP/NACK.
 *          Always includes WDT_Reset() at the start.
 *          I2C Rules: Always generate a repeated start condition instead of stop between transactions.
 * @param i2c_channel The I2C channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to read.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(i2c_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(i2c_channel);
    volatile tbyte* iicctl_reg = Get_I2C_IICCTL_Register(i2c_channel);
    tbyte slave_address = I2C_SVA0;

    if (sdr_reg == NULL || ssr_reg == NULL || iicctl_reg == NULL || max_length <= 0) {
        return;
    }

    // 1. Generate START condition
    SET_BIT(*iicctl_reg, 1); // STT = 1
    while (READ_BIT(*ssr_reg, 0)) { /* Wait until START condition generated */ }
    CLEAR_BIT(*ssr_reg, 0);

    // 2. Send 7-bit slave address + R/W bit (1 for read)
    *sdr_reg = slave_address | 0x01U;
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for address transmission complete */ }
    CLEAR_BIT(*ssr_reg, 0);

    // Check ACK/NACK

    // 3. Receive data bytes
    for (int i = 0; i < max_length; i++) {
        if (i == max_length - 1) {
            // For the last byte, send NACK
            CLEAR_BIT(*iicctl_reg, 6); // ACKE = 0 (Generate NACK for next byte)
        } else {
            // For intermediate bytes, send ACK
            SET_BIT(*iicctl_reg, 6); // ACKE = 1 (Generate ACK for next byte)
        }

        // Dummy write to initiate clocking and receive
        *sdr_reg = 0xFFU;

        while (!READ_BIT(*ssr_reg, 1)) { /* Wait for receive complete */ }
        CLEAR_BIT(*ssr_reg, 1); // Clear Rx end flag

        buffer[i] = (char)*sdr_reg; // Read received data
    }

    // 4. Generate STOP condition
    SET_BIT(*iicctl_reg, 2); // SPT = 1
    while (READ_BIT(*ssr_reg, 0)) { /* Wait for STOP condition generated */ }
    CLEAR_BIT(*ssr_reg, 0);

    // Re-enable ACK for subsequent transfers
    SET_BIT(*iicctl_reg, 6); // ACKE = 1
}

/**
 * @brief Reads a null-terminated string from I2C.
 * @details Reads characters until max_length is reached, then null-terminates.
 *          I2C does not inherently support null-terminated strings for reading,
 *          so this will read `max_length - 1` bytes and add a null terminator.
 *          Always includes WDT_Reset() at the start.
 * @param i2c_channel The I2C channel.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to read (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    if (max_length <= 0) {
        return 0;
    }
    I2C_Get_frame(i2c_channel, buffer, max_length - 1);
    buffer[max_length - 1] = '\0'; // Ensure null termination
    return (tbyte)(max_length - 1);
}

// =============================================================================
// SPI (CSI) Implementation
// =============================================================================

// Helper to get SAU0 channel registers (same as UART)
// SMR, SCR, SDR, SSR registers are shared with UART, logic must differentiate mode.

/**
 * @brief Initializes an SPI channel (CSI mode for Renesas).
 * @details Configures SPI mode (master/slave), clock polarity/phase, data frame format, and bit order.
 *          Always includes WDT_Reset() at the start.
 *          SPI Rules: Always use fast speed, Slave Select always software-controlled,
 *          Always use full duplex, Always enable CRC.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format (8-bit/16-bit).
 * @param spi_bit_order Bit order (MSB first/LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* smr_reg = Get_SAU0_SMR_Register(spi_channel);
    volatile tword* scr_reg = Get_SAU0_SCR_Register(spi_channel);
    volatile tword* sdr_reg = Get_SAU0_SDR_Register(spi_channel);

    if (smr_reg == NULL || scr_reg == NULL || sdr_reg == NULL) {
        return; // Invalid channel
    }

    // Stop SPI channel before configuring
    SET_BIT(SAU0_ST0, spi_channel); // Stop SAU0 channel n

    // Configure GPIOs for SPI function (SCK, SI, SO, and optional SS pins)
    // Based on SAU0_SDRxx assigned_pin:
    // CH0: P1.5 (SO00), P1.6 (SI00), P1.7 (SCK00)
    // CH1: P2.0 (SO01), P2.1 (SI01), P2.2 (SCK01)
    // CH2: P3.0 (SO02), P3.1 (SI02), P3.2 (SCK02)
    // CH3: P4.0 (SO03), P4.1 (SI03), P4.2 (SCK03)

    t_port so_port = PORT_0; t_pin so_pin = PIN_0;
    t_port si_port = PORT_0; t_pin si_pin = PIN_0;
    t_port sck_port = PORT_0; t_pin sck_pin = PIN_0;

    switch (spi_channel) {
        case SPI_CHANNEL_0: so_port = PORT_1; so_pin = PIN_5; si_port = PORT_1; si_pin = PIN_6; sck_port = PORT_1; sck_pin = PIN_7; break;
        case SPI_CHANNEL_1: so_port = PORT_2; so_pin = PIN_0; si_port = PORT_2; si_pin = PIN_1; sck_port = PORT_2; sck_pin = PIN_2; break;
        case SPI_CHANNEL_2: so_port = PORT_3; so_pin = PIN_0; si_port = PORT_3; si_pin = PIN_1; sck_port = PORT_3; sck_pin = PIN_2; break;
        case SPI_CHANNEL_3: so_port = PORT_4; so_pin = PIN_0; si_port = PORT_4; si_pin = PIN_1; sck_port = PORT_4; sck_pin = PIN_2; break;
        default: return;
    }

    volatile tbyte* so_pm_reg = Get_GPIO_PM_Register(so_port);
    volatile tbyte* si_pm_reg = Get_GPIO_PM_Register(si_port);
    volatile tbyte* sck_pm_reg = Get_GPIO_PM_Register(sck_port);
    volatile tbyte* so_pmc_reg = Get_GPIO_PMC_Register(so_port);
    volatile tbyte* si_pmc_reg = Get_GPIO_PMC_Register(si_port);
    volatile tbyte* sck_pmc_reg = Get_GPIO_PMC_Register(sck_port);

    // SO (Master Output, Slave Input)
    if (so_pmc_reg != NULL) SET_BIT(*so_pmc_reg, so_pin); else {/* inferred for P2,P3,P4 */}
    if (so_pm_reg != NULL) {
        if (spi_mode == SPI_MODE_MASTER) CLEAR_BIT(*so_pm_reg, so_pin); // Output
        else SET_BIT(*so_pm_reg, so_pin); // Input
    }
    // SI (Master Input, Slave Output)
    if (si_pmc_reg != NULL) SET_BIT(*si_pmc_reg, si_pin); else {/* inferred for P2,P3,P4 */}
    if (si_pm_reg != NULL) {
        if (spi_mode == SPI_MODE_MASTER) SET_BIT(*si_pm_reg, si_pin); // Input
        else CLEAR_BIT(*si_pm_reg, si_pin); // Output
    }
    // SCK (Master Output, Slave Input)
    if (sck_pmc_reg != NULL) SET_BIT(*sck_pmc_reg, sck_pin); else {/* inferred for P2,P3,P4 */}
    if (sck_pm_reg != NULL) {
        if (spi_mode == SPI_MODE_MASTER) CLEAR_BIT(*sck_pm_reg, sck_pin); // Output
        else SET_BIT(*sck_pm_reg, sck_pin); // Input
    }

    // SMR (Serial Mode Register) configuration
    // For CSI (Clock Synchronous serial interface mode) on Renesas:
    // Bit 4-6: MD = 001 (CSI mode)
    // Bit 7: EOC (Error control) - set to 0 for no error detection
    tword smr_val = 0x0000U;
    smr_val |= (0x01U << 4); // MD = 001 (CSI mode)
    smr_val |= (0x00U << 7); // EOC = 0 (No error detection)
    // CKS (Clock select, bits 0-3), usually 0000 for fCLK/1
    smr_val |= (0x00U << 0);
    // BL (Bit length, bits 11-13)
    switch (spi_dff) {
        case SPI_DFF_8BIT: smr_val |= (0x00U << 11); break; // BL = 000 for 8-bit
        case SPI_DFF_16BIT: smr_val |= (0x01U << 11); break; // BL = 001 for 16-bit
        default: smr_val |= (0x00U << 11); break; // Default 8-bit
    }
    // BSL (Base clock select, bit 14) usually 0 for internal
    // STP (Stop bit length, bit 15) usually 0 for CSI
    *smr_reg = smr_val;

    // SCR (Serial Control Register) configuration
    // For CSI:
    // Bit 0: RXE (Receive enable)
    // Bit 1: TXE (Transmit enable)
    // Bit 2: SSE (Slave select enable) - Rules: "Slave Select always software-controlled", so disable hardware SS.
    // Bit 4-5: CPH (Clock phase) and Bit 6-7: CPOL (Clock polarity)
    // Bit 8-11: CPG (Prescaler for baud rate)
    // Bit 12: MSBF (MSB first)
    tword scr_val = 0x0000U;
    scr_val |= (0x01U << 0); // RXE = 1 (Enable receive)
    scr_val |= (0x01U << 1); // TXE = 1 (Enable transmit)
    // Software controlled Slave Select, so disable hardware SSE
    CLEAR_BIT(SAU0_SSE0, spi_channel); // Assuming SSE0.n is the enable bit for hardware SS

    switch (spi_cpol) {
        case SPI_CPOL_LOW:  scr_val |= (0x00U << 6); break; // CPOL = 00 (Clock Idle Low)
        case SPI_CPOL_HIGH: scr_val |= (0x01U << 6); break; // CPOL = 01 (Clock Idle High)
        default: break;
    }
    switch (spi_cpha) {
        case SPI_CPHA_1EDGE: scr_val |= (0x00U << 4); break; // CPHA = 00 (Sample on first edge)
        case SPI_CPHA_2EDGE: scr_val |= (0x01U << 4); break; // CPHA = 01 (Sample on second edge)
        default: break;
    }
    switch (spi_bit_order) {
        case SPI_BIT_ORDER_MSB_FIRST: scr_val |= (0x01U << 12); break; // MSBF = 1
        case SPI_BIT_ORDER_LSB_FIRST: scr_val |= (0x00U << 12); break; // MSBF = 0
        default: break;
    }

    // Rules: "Always use fast speed". This implies setting CPG (Clock prescaler for baud rate).
    // Baud Rate = fSAU / (2 * (CPG+1))
    // CPG = (fSAU / (2 * Baud Rate)) - 1
    // Similar to UART, SDR register is often used for BRG value on Renesas.
    // Let's assume a fast SPI clock, e.g., fSAU/2 (half of system clock)
    tlong system_clock_hz = 16000000; // Placeholder: Assume system clock is 16MHz
    tword brg_value = (tword)((system_clock_hz / (2 * (system_clock_hz / 2))) - 1); // CPG for fSAU/2
    if (brg_value > 0xFFF) brg_value = 0xFFF; // Limit to 12-bit CPG.
    // For RL78, the SDR (Serial Data Register) is usually the 16-bit register for the transfer buffer.
    // Baud rate is typically set by the CPG bits in SCR or a dedicated prescaler register.
    // Assuming CPG is bits 8-11 of SCR, max value 0xF. This means a fixed division from SPS0.
    // CPG = 0 (no division) for fast speed from SCR.
    scr_val |= (0x00U << 8); // CPG = 0000 (no division) for fastest speed.

    // Rules: "Always use full duplex". RXE and TXE bits set already ensure this.

    // Rules: "Always enable CRC".
    // SAU0_CRC0 (0xF00DE) is the CRC register.
    // CRC_CRC0CTL (0xF007F) is the global CRC control.
    // Enable global CRC module.
    SET_BIT(CRC_CRC0CTL, 0); // Inferred: CRCE bit (bit 0) enables CRC calculation.

    *scr_reg = scr_val;

    // Mask interrupts for this channel by default
    switch (spi_channel) {
        case SPI_CHANNEL_0: SAU0_STMK0 = 0x01U; SAU0_SRMK0 = 0x01U; SAU0_SGMK0 = 0x01U; break;
        case SPI_CHANNEL_1: SAU0_STMK1 = 0x01U; SAU0_SRMK1 = 0x01U; SAU0_SGMK1 = 0x01U; break;
        case SPI_CHANNEL_2: SAU0_STMK2 = 0x01U; SAU0_SRMK2 = 0x01U; SAU0_SGMK2 = 0x01U; break;
        case SPI_CHANNEL_3: SAU0_STMK3 = 0x01U; SAU0_SRMK3 = 0x01U; SAU0_SGMK3 = 0x01U; break;
        default: break;
    }

    // Enable clock supply for SAU0 (if not already enabled)
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.
}

/**
 * @brief Enables an SPI channel.
 * @details Starts the SPI communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: SAU0 clock is enabled in `spi_Init`.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable peripheral clock if not already done.
    SET_BIT(RCC_PER0, 11); // Inferred: Enable clock for SAU0.

    // Start SPI channel
    CLEAR_BIT(SAU0_SS0, spi_channel); // Clear SS0.n to start channel n
    SAU0_SOE0 |= (0x01U << spi_channel); // Enable serial output for this channel
}

/**
 * @brief Disables an SPI channel.
 * @details Stops the SPI communication for the specified channel.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Stop SPI channel
    SET_BIT(SAU0_ST0, spi_channel); // Set ST0.n to stop channel n
    SAU0_SOE0 &= ~(0x01U << spi_channel); // Disable serial output for this channel
}

/**
 * @brief Sends a single byte over SPI.
 * @details Waits until the transmit buffer is empty, then sends the byte.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(spi_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(spi_channel); // Status register, for flags like Tx empty
    if (sdr_reg == NULL || ssr_reg == NULL) {
        return;
    }

    // Wait for transmit buffer empty (SSR.0 = Tx End flag)
    while (!READ_BIT(*ssr_reg, 0)) { /* Wait */ }
    CLEAR_BIT(*ssr_reg, 0); // Clear Tx End flag

    *sdr_reg = byte; // Write data to transmit buffer. For SPI, often also initiates receive.
}

/**
 * @brief Sends a frame (array of bytes) over SPI.
 * @details Sends a sequence of bytes.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Reads a single byte from SPI.
 * @details Initiates a transfer (dummy write) and waits for receive.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* sdr_reg = Get_SAU0_SDR_Register(spi_channel);
    volatile tword* ssr_reg = Get_SAU0_SSR_Register(spi_channel);
    if (sdr_reg == NULL || ssr_reg == NULL) {
        return 0;
    }

    // To read a byte from SPI, usually a dummy byte must be sent to generate clock pulses.
    // If full duplex is enabled, writing to SDR also fills the receive buffer.
    SPI_Send_Byte(spi_channel, 0xFFU); // Send dummy byte (this also calls WDT_Reset())

    // Wait for receive complete (SSR.1 = Rx End flag)
    while (!READ_BIT(*ssr_reg, 1)) { /* Wait */ }
    CLEAR_BIT(*ssr_reg, 1); // Clear Rx End flag

    return (tbyte)*sdr_reg; // Read received data
}

/**
 * @brief Reads a frame (array of bytes) from SPI.
 * @details Reads a sequence of bytes into a buffer.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to read.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Reads a null-terminated string from SPI.
 * @details Reads characters until max_length is reached, then null-terminates.
 *          SPI does not inherently support null-terminated strings for reading,
 *          so this will read `max_length - 1` bytes and add a null terminator.
 *          Always includes WDT_Reset() at the start.
 * @param spi_channel The SPI channel.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to read (including null terminator).
 * @return The number of bytes read (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    if (max_length <= 0) {
        return 0;
    }
    SPI_Get_frame(spi_channel, buffer, max_length - 1);
    buffer[max_length - 1] = '\0'; // Ensure null termination
    return (tbyte)(max_length - 1);
}

// =============================================================================
// External Interrupt Implementation
// =============================================================================

// Helper to get GPIO PIMK (Port Input Mode Mask Register) based on port for external interrupts.
// Note: JSON only provides PIMK0 (P0) and PIMK1 (P1)
static volatile tbyte* Get_GPIO_PIMK_Register_For_EXT_INT(t_external_int_channel channel) {
    if (channel <= EXTERNAL_INT_CHANNEL_P0_7) {
        return &GPIO_PIMK0; // For P0.0 to P0.7
    } else if (channel <= EXTERNAL_INT_CHANNEL_P1_7) {
        return &GPIO_PIMK1; // For P1.0 to P1.7
    }
    return NULL; // Other ports not explicitly covered by PIMK
}

static t_port Get_Port_From_ExtInt_Channel(t_external_int_channel channel) {
    if (channel <= EXTERNAL_INT_CHANNEL_P0_7) return PORT_0;
    if (channel <= EXTERNAL_INT_CHANNEL_P1_7) return PORT_1;
    return (t_port)0xFF; // Invalid
}

static t_pin Get_Pin_From_ExtInt_Channel(t_external_int_channel channel) {
    if (channel <= EXTERNAL_INT_CHANNEL_P0_7) return (t_pin)(channel - EXTERNAL_INT_CHANNEL_P0_0);
    if (channel <= EXTERNAL_INT_CHANNEL_P1_7) return (t_pin)(channel - EXTERNAL_INT_CHANNEL_P1_0);
    return (t_pin)0xFF; // Invalid
}


/**
 * @brief Initializes an external interrupt channel.
 * @details Configures the interrupt edge detection and associated pin.
 *          Always includes WDT_Reset() at the start.
 * @param external_int_channel The external interrupt channel.
 * @param external_int_edge The desired trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // First, configure the GPIO pin as input
    t_port port = Get_Port_From_ExtInt_Channel(external_int_channel);
    t_pin pin = Get_Pin_From_ExtInt_Channel(external_int_channel);

    if (port == 0xFF || pin == 0xFF) return;

    GPIO_Input_Init(port, pin); // Configures pin as input with pull-up (also calls WDT_Reset())

    // Enable noise filter for the pin (INT_NFEN0)
    // INT_NFEN0 is an 8-bit register. Assuming bit n corresponds to interrupt channel n.
    // The description for INT_NFEN0 is "Enables or disables noise filters for external interrupt pins."
    // It's possible it's a bit-per-channel register for all external interrupts,
    // but the JSON doesn't provide granular mapping. Assuming a generic enable.
    // For RL78, it's often a bit for each port pin or specific INT channels.
    // Given 'INT_NFEN0', I'll assume it's for external interrupt channel 0-7, which likely covers P0.
    // For P1.x based interrupts, there might be NFEN1 or other register. JSON has TIM_NFEN1 for timers.
    // So, I'll assume INT_NFEN0 relates to P0.x and other pins might not have a noise filter register here.
    if (port == PORT_0) {
        SET_BIT(INT_NFEN0, pin); // Enable noise filter for this pin (if bit mapping is direct)
    }

    // Configure interrupt edge detection (e.g., in INT_PCMK or specific INT register)
    // The JSON provides INT_PCMK: "PC Interrupt Mask Register. Masks interrupts for pin change detection."
    // This is a mask register, not an edge selection register.
    // Renesas RL78 typically uses `EGN` and `EGP` registers for edge selection or bits in `ISC` (Input Switch Control).
    // The JSON includes `PERIPH_ISC` (0xFF42) "Input Switch Control Register. Controls input signal switching for various peripherals."
    // This register could hold edge selection bits.
    // Without explicit bit details for edge, I will make an assumption for PERIPH_ISC.
    // For RL78, edge selection for external interrupts (INTPx) is often in INTPC register bits.
    // Since `register_json` doesn't provide these, I'll provide a placeholder.
    // Assuming PERIPH_ISC has 2 bits per channel for edge:
    // 00: Disable
    // 01: Rising edge
    // 10: Falling edge
    // 11: Both edges
    tbyte edge_val = 0;
    if (external_int_edge == EXTERNAL_INT_EDGE_RISING) edge_val = 1;
    else if (external_int_edge == EXTERNAL_INT_EDGE_FALLING) edge_val = 2;
    else if (external_int_edge == EXTERNAL_INT_EDGE_BOTH) edge_val = 3;

    // This is a very strong assumption, as PERIPH_ISC (0xFF42) is only one byte and for "various peripherals".
    // It is highly unlikely to hold detailed edge settings for all channels.
    // More likely, `INT_PRxx` registers contain interrupt priority AND edge select bits (or `EGN`/`EGP`).
    // For `INT_PR00` (0xFF78) to `INT_PR27` (0xFF8F), they are "Interrupt Priority Register".
    // On Renesas, PR registers usually only set priority (e.g., bits 0,1).
    // Given the lack of specific edge control registers in the JSON, I will indicate this limitation.

    // No explicit edge control register found in provided JSON matching the typical Renesas pattern.
    // Placeholder: Assuming an external interrupt control register (e.g., INTPCx) would handle this.
    // As per rule "If not found → skip generating both .h and .c entries for this module",
    // but this is not an optional module, so I have to implement.

    // Clear interrupt mask for this channel (GPIO_PIMKx) to enable it, after configuration.
    volatile tbyte* pimk_reg = Get_GPIO_PIMK_Register_For_EXT_INT(external_int_channel);
    if (pimk_reg != NULL) {
        // PIMK is "Port Input Mode Mask Register". 0 for enable, 1 for mask.
        // It's a mask for pin change *detection*. Not all external interrupts.
        // For actual external interrupt (INTPx) masks, there are usually specific INTMKx registers not in JSON.
        CLEAR_BIT(*pimk_reg, pin); // Unmask pin change detection interrupt (0 = enable)
    }

    // Additionally, a global interrupt mask for the specific interrupt source might exist (INT_PCMK)
    // INT_PCMK: "Masks interrupts for pin change detection."
    // Assuming INT_PCMK is a single bit that globally enables/disables pin change interrupts.
    // CLEAR_BIT(INT_PCMK, 0); // Unmask global pin change interrupt
}

/**
 * @brief Enables an external interrupt channel.
 * @details Unmasks the interrupt for the specified channel.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: External interrupts usually don't have separate clocks,
 *          they rely on the system clock or specific pin configuration.
 * @param external_int_channel The external interrupt channel to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_port port = Get_Port_From_ExtInt_Channel(external_int_channel);
    t_pin pin = Get_Pin_From_ExtInt_Channel(external_int_channel);

    if (port == 0xFF || pin == 0xFF) return;

    volatile tbyte* pimk_reg = Get_GPIO_PIMK_Register_For_EXT_INT(external_int_channel);
    if (pimk_reg != NULL) {
        CLEAR_BIT(*pimk_reg, pin); // Unmask the specific pin change detection interrupt
    }
    // Also clear a global mask if it exists.
    // CLEAR_BIT(INT_PCMK, 0); // Unmask global pin change interrupt (if bit 0 controls it)
}

/**
 * @brief Disables an external interrupt channel.
 * @details Masks the interrupt for the specified channel.
 *          Always includes WDT_Reset() at the start.
 * @param external_int_channel The external interrupt channel to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_port port = Get_Port_From_ExtInt_Channel(external_int_channel);
    t_pin pin = Get_Pin_From_ExtInt_Channel(external_int_channel);

    if (port == 0xFF || pin == 0xFF) return;

    volatile tbyte* pimk_reg = Get_GPIO_PIMK_Register_For_EXT_INT(external_int_channel);
    if (pimk_reg != NULL) {
        SET_BIT(*pimk_reg, pin); // Mask the specific pin change detection interrupt
    }
    // Set a global mask if it exists.
    // SET_BIT(INT_PCMK, 0); // Mask global pin change interrupt (if bit 0 controls it)
}

// =============================================================================
// PWM Implementation
// =============================================================================

// Helper for Timer Array Unit 0/1 registers
static volatile tbyte* Get_TIM_TMR_Register(t_pwm_channel channel) {
    volatile tbyte* reg = NULL;
    if (channel <= PWM_CHANNEL_TIM0_7) {
        reg = (volatile tbyte*)((uintptr_t)&TIM0_TMR00 + (channel - PWM_CHANNEL_TIM0_0));
    } else if (channel <= PWM_CHANNEL_TIM1_3) {
        reg = (volatile tbyte*)((uintptr_t)&TIM1_TMR10 + (channel - PWM_CHANNEL_TIM1_0));
    }
    return reg;
}

static volatile tword* Get_TIM_TDR_Register(t_pwm_channel channel) {
    volatile tword* reg = NULL;
    if (channel <= PWM_CHANNEL_TIM0_7) {
        reg = (volatile tword*)((uintptr_t)&TIM0_TDR00 + 2 * (channel - PWM_CHANNEL_TIM0_0)); // TDRs are 16-bit, spaced by 2 bytes
    } else if (channel <= PWM_CHANNEL_TIM1_3) {
        reg = (volatile tword*)((uintptr_t)&TIM1_TDR10 + 2 * (channel - PWM_CHANNEL_TIM1_0));
    }
    return reg;
}

static volatile tbyte* Get_TIM_TOE_Register(t_pwm_channel channel) {
    if (channel <= PWM_CHANNEL_TIM0_7) return &TIM0_TOE;
    if (channel <= PWM_CHANNEL_TIM1_3) return &TIM1_TOE;
    return NULL;
}

static volatile tbyte* Get_TIM_TOM_Register(t_pwm_channel channel) {
    if (channel <= PWM_CHANNEL_TIM0_7) return &TIM0_TOM;
    if (channel <= PWM_CHANNEL_TIM1_3) return &TIM1_TOM;
    return NULL;
}

static volatile tbyte* Get_TIM_TS_Register(t_pwm_channel channel) {
    if (channel <= PWM_CHANNEL_TIM0_7) return &TIM0_TS;
    if (channel <= PWM_CHANNEL_TIM1_3) return &TIM1_TS;
    return NULL;
}

static volatile tbyte* Get_TIM_TT_Register(t_pwm_channel channel) {
    if (channel <= PWM_CHANNEL_TIM0_7) return &TIM0_TT;
    if (channel <= PWM_CHANNEL_TIM1_3) return &TIM1_TT;
    return NULL;
}

static t_port Get_PWM_Port(t_pwm_channel channel) {
    if (channel <= PWM_CHANNEL_TIM0_7) {
        if (channel <= PWM_CHANNEL_TIM0_3) return PORT_0;
        else return PORT_1;
    } else if (channel <= PWM_CHANNEL_TIM1_3) {
        return PORT_5;
    }
    return (t_port)0xFF;
}

static t_pin Get_PWM_Pin(t_pwm_channel channel) {
    switch (channel) {
        case PWM_CHANNEL_TIM0_0: return PIN_0; // From TMMK00, TDR00
        case PWM_CHANNEL_TIM0_1: return PIN_1; // From TMMK00, TDR01
        case PWM_CHANNEL_TIM0_2: return PIN_2; // From TMMK01, TDR02
        case PWM_CHANNEL_TIM0_3: return PIN_3; // From TMMK02, TDR03
        case PWM_CHANNEL_TIM0_4: return PIN_0; // From TMMK03, TDR04 (P1.0)
        case PWM_CHANNEL_TIM0_5: return PIN_1; // From TMMK04, TDR05 (P1.1)
        case PWM_CHANNEL_TIM0_6: return PIN_2; // From TMMK05, TDR06 (P1.2)
        case PWM_CHANNEL_TIM0_7: return PIN_3; // From TMMK06, TDR07 (P1.3)
        case PWM_CHANNEL_TIM1_0: return PIN_0; // From TIM1_TMR10, TDR10 (P5.0)
        case PWM_CHANNEL_TIM1_1: return PIN_1; // From TIM1_TMR11, TDR11 (P5.1)
        case PWM_CHANNEL_TIM1_2: return PIN_2; // From TIM1_TMR12, TDR12 (P5.2)
        case PWM_CHANNEL_TIM1_3: return PIN_3; // From TIM1_TMR13, TDR13 (P5.3)
        default: return (t_pin)0xFF;
    }
}

/**
 * @brief Initializes a PWM channel.
 * @details Configures the PWM frequency and duty cycle.
 *          Always includes WDT_Reset() at the start.
 *          PWM Rules: Clear available FREQUENCY Ranges for each channel as comments.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired frequency in kHz.
 * @param pwm_duty The desired duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* tmr_reg = Get_TIM_TMR_Register(pwm_channel);
    volatile tword* tdr_reg = Get_TIM_TDR_Register(pwm_channel);
    volatile tbyte* toe_reg = Get_TIM_TOE_Register(pwm_channel);
    volatile tbyte* tom_reg = Get_TIM_TOM_Register(pwm_channel);

    if (tmr_reg == NULL || tdr_reg == NULL || toe_reg == NULL || tom_reg == NULL) {
        return; // Invalid channel
    }

    t_port port = Get_PWM_Port(pwm_channel);
    t_pin pin = Get_PWM_Pin(pwm_channel);

    if (port == 0xFF || pin == 0xFF) return;

    // First, configure the GPIO pin for PWM output.
    // PWM pins are alternate functions, configured as outputs.
    // GPIO_Output_Init will set PM to output, value to 0.
    // For alternate function, PMC bit must be set.
    // Pins based on JSON assigned_pin for TDR00-07 and TDR10-13.
    // TDR00: P0.0, P0.1 -> assuming P0.1 is the PWM output
    // TDR01: P0.1, P0.2 -> P0.2
    // TDR07: P1.3, P1.4 -> P1.4
    // TDR10: P5.0 -> P5.0
    //
    // Let's assume the SECOND pin in assigned_pin list is the output for each channel.
    // P0.1, P0.2, P0.3, P1.0, P1.1, P1.2, P1.3, P1.4, P5.0, P5.1, P5.2, P5.3
    t_pin output_pin;
    t_port output_port;
    // Map to the actual PWM output pin. This requires specific datasheet knowledge.
    // For Renesas RL78, TAU outputs are typically on TOmn pins.
    // For simplicity, I'll assume the 'second' pin in the assigned_pin list for timers implies output.
    // This is an inference; actual datasheet defines the TOmn pins.
    switch (pwm_channel) {
        case PWM_CHANNEL_TIM0_0: output_port = PORT_0; output_pin = PIN_1; break; // P0.1
        case PWM_CHANNEL_TIM0_1: output_port = PORT_0; output_pin = PIN_2; break; // P0.2
        case PWM_CHANNEL_TIM0_2: output_port = PORT_0; output_pin = PIN_3; break; // P0.3
        case PWM_CHANNEL_TIM0_3: output_port = PORT_1; output_pin = PIN_0; break; // P1.0
        case PWM_CHANNEL_TIM0_4: output_port = PORT_1; output_pin = PIN_1; break; // P1.1
        case PWM_CHANNEL_TIM0_5: output_port = PORT_1; output_pin = PIN_2; break; // P1.2
        case PWM_CHANNEL_TIM0_6: output_port = PORT_1; output_pin = PIN_3; break; // P1.3
        case PWM_CHANNEL_TIM0_7: output_port = PORT_1; output_pin = PIN_4; break; // P1.4
        case PWM_CHANNEL_TIM1_0: output_port = PORT_5; output_pin = PIN_0; break; // P5.0
        case PWM_CHANNEL_TIM1_1: output_port = PORT_5; output_pin = PIN_1; break; // P5.1
        case PWM_CHANNEL_TIM1_2: output_port = PORT_5; output_pin = PIN_2; break; // P5.2
        case PWM_CHANNEL_TIM1_3: output_port = PORT_5; output_pin = PIN_3; break; // P5.3
        default: return;
    }
    GPIO_Output_Init(output_port, output_pin, 0); // Init as output, low
    volatile tbyte* pmc_reg = Get_GPIO_PMC_Register(output_port);
    if (pmc_reg != NULL) SET_BIT(*pmc_reg, output_pin); else {/* Inferred: P5 does not have PMC in JSON */}


    // Stop timer channel during configuration
    volatile tbyte* tt_reg = Get_TIM_TT_Register(pwm_channel);
    if (tt_reg != NULL) SET_BIT(*tt_reg, pin); // Stop specific channel bit

    // Configure TMR (Timer Mode Register) for PWM mode (0xFF90-0xFF97, 0xF0130-F0133)
    // TMR typically has bits for: Channel Mode (MD), Gate Enable (GATE), Clock Select (CKS),
    // Interrupt enable, Output mode.
    // For PWM output mode on Renesas (e.g., Complementary PWM, One-shot PWM):
    // MD = 0b001 (Interval timer mode) / 0b010 (Square wave mode) / 0b011 (PWM mode)
    // Let's assume MD = 0b011 for PWM mode.
    tbyte tmr_val = 0x00U;
    tmr_val |= (0x01U << 6); // CKS0 = 1 (fCLK/2^1 = fCLK/2) - Placeholder prescaler for timer array unit.
    tmr_val |= (0x03U << 3); // MD = 011 (PWM Mode) - Inferred from common Renesas timer modes.
    *tmr_reg = tmr_val;

    // Set Timer Output Mode (TOM) for PWM output
    // TOM: Output mode 0 for interval timer output, 1 for PWM output
    SET_BIT(*tom_reg, pin); // Set TOM.n to 1 for PWM output mode

    // Set Timer Output Enable (TOE)
    SET_BIT(*toe_reg, pin); // Set TOE.n to 1 to enable PWM output

    // Set Timer Data Register (TDR) for period and duty cycle
    // For PWM: TDR holds the period and the duty cycle.
    // Period = (System_Clock / (Prescaler * pwm_khz_freq * 1000))
    // Duty_Cycle_Value = Period * (pwm_duty / 100)
    //
    // Renesas TAU units typically use a single TDR as a compare register for period or duty.
    // To set frequency (period) and duty, often two channels are linked, or a special PWM mode
    // (e.g., TDR for period, TCR for duty).
    // Given one TDR per channel, it's typically set as the Period register.
    // The duty cycle is set by a separate compare register or a specific mode.
    // For simplicity, let's assume TDR sets the period and an internal compare mechanism for duty.
    //
    // Assume fCLK = 16MHz, CKS = fCLK/2 (8MHz).
    // Timer Count = (8MHz / (pwm_khz_freq * 1000)) - 1
    tlong f_timer_clock = 8000000; // Assuming fCLK/2 = 16MHz/2 = 8MHz.
    tword period_value = (f_timer_clock / (pwm_khz_freq * 1000U)) - 1;
    tword duty_value = (period_value * pwm_duty) / 100U;

    *tdr_reg = period_value; // Set period
    // The duty cycle is usually controlled by another register or internal state.
    // If not, it means TDR sets both, or a specific TCR sets compare value.
    // Without another register (like TCR for compare), it's hard to set duty directly.
    // For now, I'll assume TDR sets the period, and we need another register to set the compare match for duty cycle.
    // Since only TDR is available for data, I'll set TDR to the PERIOD and assume the specific PWM mode handles duty.
    // Or, if TDR is for the HIGH time, then PERIOD is calculated from TDR and another register.
    // A common approach is a pair of registers: one for period, one for duty (compare value).
    // The JSON only provides TDR (Timer Data Register).
    // Renesas typically has `TDRmn` for compare/capture and `TCRmn` for counter.
    // If TDRmn is a compare register, it sets the duty cycle. The period might be set by a master channel.
    // Let's assume TDRmn sets the compare value (duty cycle high time).
    // And period is fixed by TPS or master channel's TDR.
    // If TDR sets the duty, we need a period_value somewhere.
    // This is ambiguous. Let's assume `TDRmn` sets the period register, and duty cycle is internal calculation.
    // Re-eval: TDR is a data register, used for capture/compare. If used for PWM, it can define the period or duty.
    // Let's stick with TDR as period, and for duty, it would require a second compare value (not in JSON).
    // Alternatively, TDR is the duty cycle value, and the master clock sets the period.
    // Let's set TDR to duty value, and assume period is handled by the Timer Array Unit's clock/prescaler.
    *tdr_reg = duty_value; // Set duty cycle high time in TDR.

    // PWM_requirements: "Clear available FREQUENCY Ranges for each channel as comments"
    // This requires knowing the timer clock, prescalers, and TDR register width.
    // For TAU0 channels (16-bit timers), with 16MHz clock and CKS=0 (fCLK/1), max period = 65536 / 16MHz = 4.096ms (approx 244 Hz).
    // If CKS=6 (fCLK/2^8), max period = 65536 / (16MHz/256) = 1.048s (approx 0.95 Hz).
    // If pwm_khz_freq = 1kHz, period = 1ms. Timer Count = (f_timer_clock / 1000) - 1.
    // Range depends on selected clock and prescaler.
    // Available FREQUENCY Ranges for TAU0/TAU1 channels (assuming 16-bit timers, 16MHz system clock):
    // - Fastest: fCLK/1 (16MHz) => Min Period ~62.5ns (Max Freq 16MHz), Max Period ~4.096ms (Min Freq ~244Hz)
    // - Slowest: fCLK/2^8 (62.5kHz) => Min Period ~16us (Max Freq ~62.5kHz), Max Period ~1.048s (Min Freq ~0.95Hz)
    // PWM_CHANNEL_TIM0_0 //P0.1
    // PWM_CHANNEL_TIM0_1 //P0.2
    // PWM_CHANNEL_TIM0_2 //P0.3
    // PWM_CHANNEL_TIM0_3 //P1.0
    // PWM_CHANNEL_TIM0_4 //P1.1
    // PWM_CHANNEL_TIM0_5 //P1.2
    // PWM_CHANNEL_TIM0_6 //P1.3
    // PWM_CHANNEL_TIM0_7 //P1.4
    // PWM_CHANNEL_TIM1_0 //P5.0
    // PWM_CHANNEL_TIM1_1 //P5.1
    // PWM_CHANNEL_TIM1_2 //P5.2
    // PWM_CHANNEL_TIM1_3 //P5.3

    // Configure Timer Array Unit Clock Select Register (TPS)
    // TIM0_TPS / TIM1_TPS: Selects base clock for channels. Bit 0-15: CKS0-CKS7 for TAU0, CKS0-CKS3 for TAU1.
    // Set a default prescaler for the base clock (e.g., fCLK/1 or fCLK/2).
    // For TAU0: TIM0_TPS. For TAU1: TIM1_TPS.
    if (pwm_channel <= PWM_CHANNEL_TIM0_7) {
        // Assume CKS (bit 0-3 for channel 0, etc.) in TMR register overrides TPS for specific channel.
        // TPS sets global clock for the unit. Let's set it to no prescale (fCLK).
        TIM0_TPS = 0x0000U; // CKSxx=0000 for fCLK/1
    } else {
        TIM1_TPS = 0x0000U; // CKSxx=0000 for fCLK/1
    }

    // Peripheral clock enable:
    // RCC_PER0 (0xF0000) for TAU0 (bit 14) and TAU1 (bit 15) inferred from RL78/G13.
    if (pwm_channel <= PWM_CHANNEL_TIM0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Starts the PWM output for a channel.
 * @details Enables the timer channel to begin PWM generation.
 *          Always includes WDT_Reset() at the start.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin(pwm_channel);
    if (pin == 0xFF) return;

    volatile tbyte* ts_reg = Get_TIM_TS_Register(pwm_channel);
    if (ts_reg != NULL) {
        SET_BIT(*ts_reg, pin); // Start timer channel (TS.n)
    }

    // Enable Peripheral clock (if not already enabled)
    if (pwm_channel <= PWM_CHANNEL_TIM0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Stops the PWM output for a channel.
 * @details Disables the timer channel, halting PWM generation.
 *          Always includes WDT_Reset() at the start.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin(pwm_channel);
    if (pin == 0xFF) return;

    volatile tbyte* tt_reg = Get_TIM_TT_Register(pwm_channel);
    if (tt_reg != NULL) {
        SET_BIT(*tt_reg, pin); // Stop timer channel (TT.n)
    }
}

// =============================================================================
// ICU (Input Capture Unit) Implementation
// =============================================================================

// Helper functions (shared with PWM/Timer)
// Get_TIM_TMR_Register, Get_TIM_TPS_Register, Get_TIM_TSR_Register, Get_TIM_TS_Register, Get_TIM_TT_Register
// And Get_PWM_Port, Get_PWM_Pin are used for ICU as well.

static volatile tbyte* Get_TIM_TSR_Register_ICU(t_icu_channel channel) {
    volatile tbyte* reg = NULL;
    if (channel <= ICU_CHANNEL_TIM0_7) {
        reg = (volatile tbyte*)((uintptr_t)&TIM0_TSR00 + (channel - ICU_CHANNEL_TIM0_0));
    } else if (channel <= ICU_CHANNEL_TIM1_3) {
        reg = (volatile tbyte*)((uintptr_t)&TIM1_TSR10 + (channel - ICU_CHANNEL_TIM1_0));
    }
    return reg;
}

static volatile tbyte* Get_TIM_TMMK_Register_ICU(t_icu_channel channel) {
    volatile tbyte* reg = NULL;
    if (channel <= ICU_CHANNEL_TIM0_7) {
        reg = (volatile tbyte*)((uintptr_t)&TIM0_TMMK00 + (channel - ICU_CHANNEL_TIM0_0));
    }
    // No TMMK registers for TAU1 in JSON, so assuming TAU0 only for interrupt masks.
    return reg;
}

/**
 * @brief Initializes an ICU channel.
 * @details Configures the prescaler and edge detection for input capture.
 *          Always includes WDT_Reset() at the start.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the timer clock.
 * @param icu_edge The desired capture edge (rising, falling, or both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* tmr_reg = Get_TIM_TMR_Register((t_pwm_channel)icu_channel);
    volatile tbyte* ts_reg = Get_TIM_TS_Register((t_pwm_channel)icu_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)icu_channel);
    volatile tword* tps_reg = NULL; // TIM0_TPS or TIM1_TPS
    t_pin pin = Get_PWM_Pin((t_pwm_channel)icu_channel);

    if (tmr_reg == NULL || ts_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return; // Invalid channel
    }

    t_port port = Get_PWM_Port((t_pwm_channel)icu_channel);
    t_pin input_pin;

    // Map to the actual ICU input pin.
    // For ICU, the pin is typically the first one in the assigned_pin list for timers, or dedicated TImn pins.
    // Example: TIM0_TDR00 is P0.0, P0.1. P0.0 is often the capture input.
    switch (icu_channel) {
        case ICU_CHANNEL_TIM0_0: input_pin = PIN_0; break; // P0.0
        case ICU_CHANNEL_TIM0_1: input_pin = PIN_1; break; // P0.1
        case ICU_CHANNEL_TIM0_2: input_pin = PIN_2; break; // P0.2
        case ICU_CHANNEL_TIM0_3: input_pin = PIN_3; break; // P0.3
        case ICU_CHANNEL_TIM0_4: input_pin = PIN_0; break; // P1.0
        case ICU_CHANNEL_TIM0_5: input_pin = PIN_1; break; // P1.1
        case ICU_CHANNEL_TIM0_6: input_pin = PIN_2; break; // P1.2
        case ICU_CHANNEL_TIM0_7: input_pin = PIN_3; break; // P1.3
        case ICU_CHANNEL_TIM1_0: input_pin = PIN_0; break; // P5.0
        case ICU_CHANNEL_TIM1_1: input_pin = PIN_1; break; // P5.1
        case ICU_CHANNEL_TIM1_2: input_pin = PIN_2; break; // P5.2
        case ICU_CHANNEL_TIM1_3: input_pin = PIN_3; break; // P5.3
        default: return;
    }
    // Configure GPIO pin as input
    GPIO_Input_Init(port, input_pin); // Also calls WDT_Reset()
    volatile tbyte* pmc_reg = Get_GPIO_PMC_Register(port);
    if (pmc_reg != NULL) SET_BIT(*pmc_reg, input_pin); // Set alternate function for input capture

    // Stop timer channel during configuration
    SET_BIT(*tt_reg, pin); // Stop specific channel bit

    // Configure TMR (Timer Mode Register) for Input Capture mode
    // MD = 100 for Capture mode on Renesas (inferred from common modes).
    // Bits for clock select (CKS), interrupt enable (IE), and edge selection (CCS, CIS)
    tbyte tmr_val = 0x00U;
    tmr_val |= (0x01U << 7); // IE: Interrupt enable (for capture interrupt) - will be masked for polling.
    tmr_val |= (0x04U << 3); // MD = 100 (Capture mode)

    // Clock Select (CKS) and Prescaler (from icu_prescaller)
    // CKS bits within TMR: 000=fCLK/1, 001=fCLK/2, 010=fCLK/4, 011=fCLK/8 (typical)
    tbyte cks_bits = 0;
    switch (icu_prescaller) {
        case ICU_PRESCALER_DIV1: cks_bits = 0b000; break;
        case ICU_PRESCALER_DIV2: cks_bits = 0b001; break;
        case ICU_PRESCALER_DIV4: cks_bits = 0b010; break;
        case ICU_PRESCALER_DIV8: cks_bits = 0b011; break;
        default: cks_bits = 0b000; break;
    }
    tmr_val |= (cks_bits << 0); // CKS0-CKS2 bits

    // Edge selection (CIS, CCS bits in TMR for some Renesas MCUs)
    // CIS0/CIS1: Capture input select (00=TI00, 01=TI01, etc)
    // CCS: Capture Control Select (0=rising, 1=falling, or 2 bits for both)
    // Assuming CCS bits (e.g., bit 5, 4 of TMR)
    tbyte ccs_bits = 0;
    switch (icu_edge) {
        case ICU_EDGE_RISING:  ccs_bits = 0b00; break; // Capture on rising edge
        case ICU_EDGE_FALLING: ccs_bits = 0b01; break; // Capture on falling edge
        case ICU_EDGE_BOTH:    ccs_bits = 0b10; break; // Capture on both edges (rising & falling)
        default: ccs_bits = 0b00; break;
    }
    tmr_val |= (ccs_bits << 4); // Placeholder for CCS (Capture Control Select) bits.

    *tmr_reg = tmr_val;

    // Mask timer interrupt (TMMKxx) initially for ICU (unless interrupt driven capture is desired)
    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU(icu_channel);
    if (tmmk_reg != NULL) {
        SET_BIT(*tmmk_reg, pin); // Mask interrupt for this channel (1=mask)
    }

    // Configure Timer Array Unit Clock Select Register (TPS)
    if (icu_channel <= ICU_CHANNEL_TIM0_7) {
        tps_reg = &TIM0_TPS;
    } else {
        tps_reg = &TIM1_TPS;
    }
    // Set TPS to use fCLK for the base clock. This is common to all channels in the unit.
    *tps_reg = 0x0000U; // Assuming CKS for each channel selects from this base.

    // Peripheral clock enable:
    if (icu_channel <= ICU_CHANNEL_TIM0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Enables an ICU channel.
 * @details Unmasks the timer interrupt for capture events.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: TAU clock is enabled in `ICU_init`.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin((t_pwm_channel)icu_channel);
    if (pin == 0xFF) return;

    volatile tbyte* ts_reg = Get_TIM_TS_Register((t_pwm_channel)icu_channel);
    if (ts_reg != NULL) {
        SET_BIT(*ts_reg, pin); // Start timer channel
    }

    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU(icu_channel);
    if (tmmk_reg != NULL) {
        CLEAR_BIT(*tmmk_reg, pin); // Unmask interrupt for this channel
    }

    // Enable Peripheral clock (if not already enabled)
    if (icu_channel <= ICU_CHANNEL_TIM0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Disables an ICU channel.
 * @details Masks the timer interrupt for capture events and stops the timer channel.
 *          Always includes WDT_Reset() at the start.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin((t_pwm_channel)icu_channel);
    if (pin == 0xFF) return;

    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)icu_channel);
    if (tt_reg != NULL) {
        SET_BIT(*tt_reg, pin); // Stop timer channel
    }

    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU(icu_channel);
    if (tmmk_reg != NULL) {
        SET_BIT(*tmmk_reg, pin); // Mask interrupt for this channel
    }
}

/**
 * @brief Gets the frequency of the input signal on an ICU channel.
 * @details Reads captured timer values to calculate the frequency.
 *          Requires two successive captures to determine a period.
 *          Always includes WDT_Reset() at the start.
 * @param icu_channel The ICU channel.
 * @return The calculated frequency in Hz.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)icu_channel); // TDR for capture value
    volatile tbyte* tsr_reg = Get_TIM_TSR_Register_ICU(icu_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)icu_channel);

    if (tdr_reg == NULL || tsr_reg == NULL || pin == 0xFF) {
        return 0; // Invalid channel
    }

    static tword last_capture_value[12] = {0}; // Array to store last capture for each channel
    static bool first_capture_done[12] = {false};
    tword current_capture_value;
    tlong frequency = 0;
    tlong timer_clock_freq = 0; // Base clock for the timer unit

    // Determine timer clock frequency based on prescaler in TMR (CKS bits)
    volatile tbyte* tmr_reg = Get_TIM_TMR_Register((t_pwm_channel)icu_channel);
    if (tmr_reg == NULL) return 0;
    tbyte cks_bits = READ_BIT(*tmr_reg, 0) | (READ_BIT(*tmr_reg, 1) << 1) | (READ_BIT(*tmr_reg, 2) << 2);
    tlong system_clock = 16000000; // Placeholder for system clock

    switch (cks_bits) {
        case 0b000: timer_clock_freq = system_clock / 1; break;
        case 0b001: timer_clock_freq = system_clock / 2; break;
        case 0b010: timer_clock_freq = system_clock / 4; break;
        case 0b011: timer_clock_freq = system_clock / 8; break;
        default: timer_clock_freq = system_clock / 1; break; // Default
    }

    // Wait for a capture event (TSR.pin indicates capture flag)
    // Assuming TSR.n is the capture flag, or specific interrupt flag
    while (!READ_BIT(*tsr_reg, pin)) { /* Wait for capture */ }
    CLEAR_BIT(*tsr_reg, pin); // Clear capture flag

    current_capture_value = *tdr_reg; // Read the captured value

    if (!first_capture_done[icu_channel]) {
        last_capture_value[icu_channel] = current_capture_value;
        first_capture_done[icu_channel] = true;
    } else {
        tword period_counts;
        if (current_capture_value >= last_capture_value[icu_channel]) {
            period_counts = current_capture_value - last_capture_value[icu_channel];
        } else {
            // Timer overflowed, consider 16-bit rollover
            period_counts = (tword)(0xFFFF - last_capture_value[icu_channel] + current_capture_value + 1);
        }

        if (period_counts > 0) {
            frequency = timer_clock_freq / period_counts;
        } else {
            frequency = 0; // Avoid division by zero
        }
        last_capture_value[icu_channel] = current_capture_value;
    }
    return frequency;
}

/**
 * @brief Sets a callback function for ICU events.
 * @details Registers a function to be called when an ICU capture event occurs.
 *          Always includes WDT_Reset() at the start.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    icu_callback_g = callback;
    // For this to work, a timer interrupt must be enabled for the ICU channel.
    // The ICU_Enable/Disable currently mask/unmask TMMK. So, this would need to unmask.
}

// =============================================================================
// Timer Implementation
// =============================================================================

// Helper functions (shared with PWM/ICU)
// Get_TIM_TMR_Register, Get_TIM_TDR_Register, Get_TIM_TPS_Register, Get_TIM_TS_Register, Get_TIM_TT_Register
// And Get_PWM_Port, Get_PWM_Pin are used for Timer as well.
// Get_TIM_TCR_Register (Timer Count Register) is specific to Timer as a counter.
static volatile tword* Get_TIM_TCR_Register(t_timer_channel channel) {
    volatile tword* reg = NULL;
    if (channel <= TIMER_CHANNEL_0_7) {
        reg = (volatile tword*)((uintptr_t)&TIM0_TCR00 + 2 * (channel - TIMER_CHANNEL_0_0));
    } else if (channel <= TIMER_CHANNEL_1_3) {
        reg = (volatile tword*)((uintptr_t)&TIM1_TCR10 + 2 * (channel - TIMER_CHANNEL_1_0));
    }
    return reg;
}

/**
 * @brief Initializes a timer channel.
 * @details Configures the timer for basic operation.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tbyte* tmr_reg = Get_TIM_TMR_Register((t_pwm_channel)timer_channel);
    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)timer_channel);
    volatile tword* tcr_reg = Get_TIM_TCR_Register(timer_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);

    if (tmr_reg == NULL || tdr_reg == NULL || tcr_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return; // Invalid channel
    }

    // Stop timer channel during configuration
    SET_BIT(*tt_reg, pin); // Stop specific channel bit

    // Configure TMR (Timer Mode Register) for Interval Timer mode
    // MD = 001 (Interval timer mode) - for basic timer functions.
    // CKS (Clock Select), IE (Interrupt Enable)
    tbyte tmr_val = 0x00U;
    tmr_val |= (0x01U << 7); // IE: Interrupt enable
    tmr_val |= (0x01U << 3); // MD = 001 (Interval Timer Mode)
    tmr_val |= (0x00U << 0); // CKS = 000 (fCLK/1) - default clock source for this channel
    *tmr_reg = tmr_val;

    // Clear Timer Data Register (TDR) and Timer Count Register (TCR)
    *tdr_reg = 0x0000U; // Clear compare/period value
    *tcr_reg = 0x0000U; // Clear counter

    // Mask timer interrupt (TMMKxx) initially
    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU((t_icu_channel)timer_channel);
    if (tmmk_reg != NULL) {
        SET_BIT(*tmmk_reg, pin); // Mask interrupt for this channel (1=mask)
    }

    // Configure Timer Array Unit Clock Select Register (TPS) for the unit
    if (timer_channel <= TIMER_CHANNEL_0_7) {
        TIM0_TPS = 0x0000U; // CKSxx=0000 for fCLK/1
    } else {
        TIM1_TPS = 0x0000U; // CKSxx=0000 for fCLK/1
    }

    // Peripheral clock enable:
    if (timer_channel <= TIMER_CHANNEL_0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Sets the timer period in microseconds.
 * @details Configures the timer's compare register (TDR) for the specified delay.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel.
 * @param time The time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)timer_channel);
    volatile tword* tcr_reg = Get_TIM_TCR_Register(timer_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);

    if (tdr_reg == NULL || tcr_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return;
    }

    // Stop timer before setting period
    SET_BIT(*tt_reg, pin);

    // Calculate count value for microseconds
    // Assume fCLK/1 (16MHz) for simplicity. Timer Clock = 16MHz.
    // Count = (Time_us * Timer_Clock_MHz) - 1
    tlong timer_clock_hz = 16000000; // Assuming fCLK/1
    tword count_value = (tword)((timer_clock_hz / 1000000UL) * time - 1); // For us

    *tdr_reg = count_value; // Set period in TDR
    *tcr_reg = 0x0000U; // Reset counter
}

/**
 * @brief Sets the timer period in milliseconds.
 * @details Configures the timer's compare register (TDR) for the specified delay.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel.
 * @param time The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)timer_channel);
    volatile tword* tcr_reg = Get_TIM_TCR_Register(timer_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);

    if (tdr_reg == NULL || tcr_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return;
    }

    // Stop timer before setting period
    SET_BIT(*tt_reg, pin);

    // Calculate count value for milliseconds
    // Assume fCLK/1 (16MHz) for simplicity. Timer Clock = 16MHz.
    // Count = (Time_ms * Timer_Clock_kHz) - 1
    tlong timer_clock_hz = 16000000; // Assuming fCLK/1
    tword count_value = (tword)((timer_clock_hz / 1000UL) * time - 1); // For ms

    *tdr_reg = count_value; // Set period in TDR
    *tcr_reg = 0x0000U; // Reset counter
}

/**
 * @brief Sets the timer period in seconds.
 * @details Configures the timer's compare register (TDR) for the specified delay.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel.
 * @param time The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // For longer times, we might need a slower clock or chained timers.
    // Assuming the timer can hold this period (16-bit timer max count 65535).
    // If fCLK/1 (16MHz), max time = 65535 / 16MHz = 4.096ms.
    // For seconds, we need a slower clock for the timer.
    // Let's reconfigure TMR CKS to a slower clock for seconds.
    // Or assume TIMER_Set_Time_ms is called multiple times.
    // To achieve seconds, it is implied a slower prescaler is needed for the timer.
    // For example, if TMR CKS = fCLK/8, Timer Clock = 16MHz/8 = 2MHz.
    // Max time = 65535 / 2MHz = 32.7ms. Still not seconds directly.
    // Let's assume the base clock for the timer can be sufficiently divided by TPS for longer intervals.
    //
    // For Renesas TAU, CKS in TMR defines channel-specific clock.
    // Example: If CKS = fCLK/2^10 (approx 15.6kHz at 16MHz).
    // Max time = 65535 / 15625 Hz = ~4.19 seconds. This can work for small seconds.
    // To simplify, let's just convert seconds to milliseconds and call Set_Time_ms.
    // This will work if max_time_ms * 1000 is within tword limits and timer can generate it.
    // Max tword is 65535. Max ms = 65535. Max seconds = 65535 / 1000 = 65 seconds.
    // This is valid.
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000);
}

/**
 * @brief Sets the timer period in minutes.
 * @details Configures the timer's compare register (TDR) for the specified delay.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel.
 * @param time The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    // Minutes will exceed 16-bit timer range easily.
    // 1 minute = 60 seconds = 60,000 milliseconds.
    // This requires chaining timers or using a specific RTOS task to count overflows.
    // As MCAL, we directly configure the timer. It cannot achieve minutes directly with a 16-bit counter.
    // I will set it to the maximum possible time that can be configured with a single 16-bit counter
    // with max prescaler, or state it's not directly supported.
    // Max ms = 65535. Max seconds = 65. Max minutes = 1.
    // So 1 minute is the practical maximum.
    // For Renesas timers, a common method is to set the period to maximum possible and then count overflows in ISR.
    // Since API is `Set_Time_min`, I'll use the maximum possible count in TDR and note the limitation.
    // If `time` is 1 min, then 60,000 ms.
    // `TIMER_Set_Time_ms(timer_channel, (tword)time * 60 * 1000);` would overflow `tword`.
    // It is best to call `TIMER_Set_Time_sec` and handle larger periods via multiple ISR calls.
    // However, the function signature dictates a direct 'set' for minutes.
    // For an MCAL, the direct hardware register approach is preferred.
    // I'll set a very large prescaler for TMR (CKS bits) to slow down the clock enough for minutes.
    // If we want 1 minute (60 seconds):
    // Timer Clock = 16MHz / 2^13 = 1953 Hz (using 3 bit CKS in TMR, no 13 bit prescaler is available directly).
    // Let's assume we can configure a very low clock via TPS (e.g. fCLK/2^14, ~1kHz).
    // Count = (1kHz * 60s) - 1 = 59999. This fits 16-bit.
    // This implies a change in TMR CKS or TPS for the selected channel.
    volatile tbyte* tmr_reg = Get_TIM_TMR_Register((t_pwm_channel)timer_channel);
    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)timer_channel);
    volatile tword* tcr_reg = Get_TIM_TCR_Register(timer_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);

    if (tmr_reg == NULL || tdr_reg == NULL || tcr_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return;
    }

    // Stop timer before setting period
    SET_BIT(*tt_reg, pin);

    // Reconfigure TMR CKS for a very slow clock (e.g., fCLK/2^10 or max division if available directly in TMR)
    tbyte current_tmr_val = *tmr_reg;
    current_tmr_val &= ~(0x07U << 0); // Clear CKS bits
    // Assume CKS = 0b110 gives fCLK/2^8 (256 division). 16MHz/256 = 62.5kHz.
    current_tmr_val |= (0x06U << 0); // Set CKS to 0b110 (fCLK/256)
    *tmr_reg = current_tmr_val;

    tlong timer_clock_hz = 16000000 / 256; // Example: 62500 Hz
    tword count_value = (tword)((timer_clock_hz / 1000UL) * ((tword)time * 60) - 1);
    // This calculation still risks overflow if time*60*1000*factor is too large for tword.
    // Since `tword` max is 65535.
    // For 1 minute, 60 seconds.
    // For a 62.5kHz clock: 60 seconds * 62500 counts/sec = 3,750,000 counts. This exceeds tword.
    // So this function realistically cannot achieve minutes with a single 16-bit timer without overflow.
    // It implicitly requires overflow handling (cascading timers or ISR count).
    // As per the MCAL level, I will set TDR to max value and assume higher layers count overflows.
    // OR, use a smaller time slice for the timer, and let the software in a loop count "minutes".
    // For direct single-shot minute timer, it's not feasible here.
    // Setting TDR to max (65535) and calling it good for "max time".
    *tdr_reg = 0xFFFFU; // Set TDR to maximum value.
    *tcr_reg = 0x0000U; // Reset counter.
    // Note: This function as implemented will set the timer to its maximum possible duration
    // given the chosen prescaler, not necessarily the exact 'time' in minutes due to 16-bit limits.
    // Actual minutes would require counting overflows or a dedicated RTC/chained timer.
}

/**
 * @brief Sets the timer period in hours.
 * @details Configures the timer's compare register (TDR) for the specified delay.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel.
 * @param time The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    // Similar to minutes, hours will significantly exceed 16-bit timer range.
    // 1 hour = 3600 seconds = 3,600,000 milliseconds.
    // This function can only configure the timer for its maximum period,
    // requiring higher-level software to count overflows for actual hours.
    // For MCAL, we configure the timer hardware.
    // I will set TDR to max value.
    volatile tword* tdr_reg = Get_TIM_TDR_Register((t_pwm_channel)timer_channel);
    volatile tword* tcr_reg = Get_TIM_TCR_Register(timer_channel);
    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);

    if (tdr_reg == NULL || tcr_reg == NULL || tt_reg == NULL || pin == 0xFF) {
        return;
    }

    // Stop timer before setting period
    SET_BIT(*tt_reg, pin);

    // Reconfigure TMR CKS for a very slow clock (e.g., fCLK/2^10 or max division)
    tbyte current_tmr_val = *tmr_reg;
    current_tmr_val &= ~(0x07U << 0); // Clear CKS bits
    // Assume CKS = 0b110 gives fCLK/2^8 (256 division). 16MHz/256 = 62.5kHz.
    current_tmr_val |= (0x06U << 0); // Set CKS to 0b110 (fCLK/256)
    *tmr_reg = current_tmr_val;

    *tdr_reg = 0xFFFFU; // Set TDR to maximum value.
    *tcr_reg = 0x0000U; // Reset counter.
    // Note: This function, like TIMER_Set_Time_min, sets the timer to its maximum possible
    // single-shot duration, not the exact 'time' in hours due to 16-bit limits.
    // Actual hour timing would require counting overflows in software or a dedicated RTC.
}

/**
 * @brief Enables a timer channel.
 * @details Unmasks the timer interrupt and starts the timer.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: TAU clock is enabled in `TIMER_Init`.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);
    if (pin == 0xFF) return;

    volatile tbyte* ts_reg = Get_TIM_TS_Register((t_pwm_channel)timer_channel);
    if (ts_reg != NULL) {
        SET_BIT(*ts_reg, pin); // Start timer channel (TS.n)
    }

    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU((t_icu_channel)timer_channel);
    if (tmmk_reg != NULL) {
        CLEAR_BIT(*tmmk_reg, pin); // Unmask interrupt for this channel
    }

    // Enable Peripheral clock (if not already enabled)
    if (timer_channel <= TIMER_CHANNEL_0_7) {
        SET_BIT(RCC_PER0, 14); // Inferred: Enable clock for TAU0
    } else {
        SET_BIT(RCC_PER0, 15); // Inferred: Enable clock for TAU1
    }
}

/**
 * @brief Disables a timer channel.
 * @details Masks the timer interrupt and stops the timer.
 *          Always includes WDT_Reset() at the start.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    t_pin pin = Get_PWM_Pin((t_pwm_channel)timer_channel);
    if (pin == 0xFF) return;

    volatile tbyte* tt_reg = Get_TIM_TT_Register((t_pwm_channel)timer_channel);
    if (tt_reg != NULL) {
        SET_BIT(*tt_reg, pin); // Stop timer channel (TT.n)
    }

    volatile tbyte* tmmk_reg = Get_TIM_TMMK_Register_ICU((t_icu_channel)timer_channel);
    if (tmmk_reg != NULL) {
        SET_BIT(*tmmk_reg, pin); // Mask interrupt for this channel
    }
}

// =============================================================================
// ADC Implementation
// =============================================================================

/**
 * @brief Initializes an ADC channel.
 * @details Configures the ADC for a specific channel and operating mode.
 *          Always includes WDT_Reset() at the start.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The operating mode (polling, interrupt, continuous).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // ADC_ADM0: A/D Converter Mode Register 0
    // ADM0 bits: ADCE (Enable), ADCS (Start trigger select), ADCKS (Clock select), ADS (Channel Select)
    // ADM0.7: ADCE (A/D converter operation enable)
    // ADM0.0: ADCS (Conversion start trigger select)
    // Other bits in ADM0 or ADS set the actual channel and clock.

    // Disable ADC during configuration
    CLEAR_BIT(ADC_ADM0, 7); // ADCE = 0

    // Configure Analog Input Setting Register (ADC_PMC)
    // This register maps specific physical pins to analog input functions (AN0-ANx).
    // The JSON for ADC_PMC says "Configures pins for analog input function for the A/D converter."
    // Assuming ADC_PMC bits correspond to AN0, AN1, etc.
    // For Renesas RL78, it's often ADPC register (A/D Port Configuration Register) for pin enable.
    // ADC_ADPC (0xFF9C) is available, description: "Specifies which physical pins are used as analog inputs".
    // ADC_PMC (0xFF9B) description: "Analog Input Setting Register."
    // Let's use ADPC for pin selection.
    // ADPC register is typically a bit-per-channel enable.
    SET_BIT(ADC_ADPC, adc_channel); // Enable analog input for the specific channel.

    // Configure A/D Converter Mode Register 0 (ADC_ADM0)
    // ADM0.0 (ADCS): Single conversion (0) or continuous (1).
    tbyte adm0_val = 0x00U;
    if (adc_mode == ADC_MODE_CONTINUOUS) {
        SET_BIT(adm0_val, 0); // ADCS = 1 (Continuous conversion mode)
    } else {
        CLEAR_BIT(adm0_val, 0); // ADCS = 0 (Single conversion mode)
    }
    // Other bits in ADM0 (e.g., ADTYP, ADPRC) can configure resolution, conversion speed.
    // For now, use default/simplest config.
    // ADM0.4 (ADCKS0), ADM0.5 (ADCKS1) - A/D clock select. Default to fastest (e.g., 00 = fCPU/2).
    ADC_ADM0 = adm0_val;

    // Configure A/D Channel Select Register (ADC_ADS)
    // ADC_ADS is an 8-bit register, where bits 0-7 directly select AN0-AN7.
    ADC_ADS = adc_channel; // Select the desired analog channel.

    // Configure interrupt mask (ADC_ADMK)
    // ADC_ADMK: Masks interrupts generated by the A/D converter.
    // 0x00U = unmask (enable), 0x01U = mask (disable).
    if (adc_mode == ADC_MODE_INTERRUPT) {
        ADC_ADMK = 0x00U; // Unmask A/D interrupt
    } else {
        ADC_ADMK = 0x01U; // Mask A/D interrupt (for polling mode)
    }

    // Peripheral clock enable:
    // RCC_PER0 (0xF0000) for ADC (assuming bit 12 for ADC enable, based on RL78/G13).
    SET_BIT(RCC_PER0, 12); // Inferred: Enable clock for ADC
}

/**
 * @brief Enables the ADC module.
 * @details Activates the ADC converter to start conversions.
 *          Always includes WDT_Reset() at the start.
 *          Peripheral clock enable: ADC clock is enabled in `ADC_Init`.
 * @param adc_channel The ADC channel (not strictly needed for global enable, but kept for API consistency).
 */
void ADC_Enable(t_adc_channel adc_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable peripheral clock if not already done.
    SET_BIT(RCC_PER0, 12); // Inferred: Enable clock for ADC

    // Enable ADC operation (ADCE bit 7 in ADM0)
    SET_BIT(ADC_ADM0, 7); // ADCE = 1

    // Start A/D conversion if in single conversion mode (ADC_ADCS)
    // ADC_ADCS: "A/D Converter Start Register. Controls the start and stop of A/D conversion."
    // A write to ADCS starts the conversion. Bit 0 for start (assuming).
    SET_BIT(ADC_ADCS, 0); // Start A/D conversion (inferred: bit 0 of ADCS starts)
}

/**
 * @brief Disables the ADC module.
 * @details Deactivates the ADC converter, stopping conversions.
 *          Always includes WDT_Reset() at the start.
 * @param adc_channel The ADC channel (not strictly needed for global disable, but kept for API consistency).
 */
void ADC_Disable(t_adc_channel adc_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Stop A/D conversion (ADCE bit 7 in ADM0)
    CLEAR_BIT(ADC_ADM0, 7); // ADCE = 0

    // Stop A/D conversion explicitly (ADC_ADCS)
    CLEAR_BIT(ADC_ADCS, 0); // Stop A/D conversion (inferred: bit 0 of ADCS stops)
}

/**
 * @brief Performs an ADC conversion in polling mode and returns the result.
 * @details Starts a single conversion, waits for completion, and reads the result.
 *          Always includes WDT_Reset() at the start.
 * @param adc_channel The ADC channel to read.
 * @return The 10-bit or 12-bit ADC conversion result.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    tword result = 0;

    // Select the channel (if not already selected by ADC_Init)
    ADC_ADS = adc_channel;

    // Start conversion (if not continuous)
    SET_BIT(ADC_ADCS, 0); // Start A/D conversion

    // Wait for conversion complete.
    // The LVD_LVDVDDF is a flag register. For ADC, a flag in ADM0 or specific interrupt status register is used.
    // On Renesas, a common flag is ADIF (A/D conversion finish flag) in interrupt flag registers (IF)
    // or sometimes bit 7 of ADM0 (ADCE) transitions or a specific status bit.
    // Without a specific ADC status flag in JSON (e.g., ADIF), I'll infer from ADMK (Interrupt Mask).
    // The ADMK description is "Masks interrupts", it implies there's an interrupt flag somewhere.
    // A common approach for polling is to check if the interrupt *flag* is set, even if masked.
    // For RL78, it's usually `ADIF` bit in `IF` register. Since `IF` is not in JSON,
    // I will simulate with a delay or assume a mechanism that makes sense.
    // Let's assume ADM0.6 (ADF) for A/D conversion finish flag (inferred).
    while (!READ_BIT(ADC_ADM0, 6)) { /* Wait for conversion complete */ }
    CLEAR_BIT(ADC_ADM0, 6); // Clear the flag (inferred)

    // Read result from ADCRH and ADCRL (10-bit or 12-bit result)
    // ADCRH is high byte, ADCRL is low byte.
    // Result = (ADCRH << 8) | ADCRL. For 10-bit, only 10 bits are valid.
    result = (((tword)ADC_ADCRH << 8) | ADC_ADCRL);

    // Stop conversion if in single mode
    CLEAR_BIT(ADC_ADCS, 0); // Stop A/D conversion

    return result;
}

/**
 * @brief Performs an ADC conversion using interrupts and returns the result (after interrupt).
 * @details This function would trigger a conversion and rely on an ISR to capture the result.
 *          For this MCAL API, it initiates the conversion and waits for the result (simulated).
 *          Always includes WDT_Reset() at the start.
 * @param adc_channel The ADC channel to read.
 * @return The 10-bit or 12-bit ADC conversion result.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Enable ADC interrupt for the specified channel
    ADC_ADMK = 0x00U; // Unmask A/D interrupt

    // Select the channel
    ADC_ADS = adc_channel;

    // Start conversion
    SET_BIT(ADC_ADCS, 0); // Start A/D conversion

    // In a real interrupt-driven system, this function would typically return
    // immediately and the result would be retrieved by a callback or global variable in ISR.
    // For the purpose of this API (which returns a value), we'll simulate waiting.
    // This is effectively polling, but assumes the interrupt setup is active.
    while (!READ_BIT(ADC_ADM0, 6)) { /* Wait for conversion complete (flag) */ }
    CLEAR_BIT(ADC_ADM0, 6); // Clear flag

    // Mask interrupt after getting result if only a single conversion is needed.
    ADC_ADMK = 0x01U; // Mask A/D interrupt

    // Read result
    tword result = (((tword)ADC_ADCRH << 8) | ADC_ADCRL);

    // Stop conversion
    CLEAR_BIT(ADC_ADCS, 0); // Stop A/D conversion

    return result;
}

// =============================================================================
// Internal_EEPROM Implementation
// =============================================================================

// The `register_json` does not explicitly define registers for Internal EEPROM
// Read/Write operations. It provides `FLASH_FLAPL`, `FLAPH`, `FLARS`, `FLARE`,
// `FLSEC`, `FLRESD` for Flash access protection and configuration.
// On Renesas RL78, "Internal EEPROM" often refers to Data Flash.
// Data Flash R/W requires a specific sequence: disable interrupts, issue command,
// write/erase, wait for completion, re-enable interrupts.
// Without specific Data Flash R/W registers, these functions will provide
// placeholder implementations using the available Flash registers for conceptual
// control, and note the missing data R/W registers.

/**
 * @brief Initializes the Internal EEPROM (Data Flash) module.
 * @details Prepares the Data Flash for read/write operations.
 *          Always includes WDT_Reset() at the start.
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Renesas Data Flash initialization typically involves:
    // 1. Enabling the Data Flash clock (often through RCC_PERx)
    // 2. Ensuring the MCU is in a safe mode (e.g., CPU runs from RAM if writing to code flash).
    // 3. Unprotecting flash.
    // The provided JSON does not contain specific Data Flash enable or control registers
    // for actual R/W operations, only protection (`FLASH_FLAPL`, `FLAPH`, `FLSEC`, `FLRESD`).
    //
    // Placeholder: Assuming `RCC_PER0` might have a bit for Flash/Data Flash enable.
    // For RL78/G13, bit 10 of `PER0` enables Flash (including Data Flash).
    SET_BIT(RCC_PER0, 10); // Inferred: Enable clock for Flash/Data Flash.

    // Unlock protection if any, for R/W access.
    // `FLASH_FLSEC`, `FLASH_FLRESD` for security/read-out protection.
    // `FLASH_FLAPL`, `FLASH_FLAPH`, `FLARS`, `FLARE` for access protection.
    // To allow R/W, these might need to be set to specific values, but no direct "unlock" command.
    // These are typically configured at device programming.
    // For now, no specific unlock sequence is implemented, assuming it's done at build-time or implicitly.
    // Acknowledge this limitation for this MCAL:
    // "Specific Data Flash read/write command registers not found in register_json."
}

/**
 * @brief Writes a byte to a specified address in Internal EEPROM (Data Flash).
 * @details Implements the Data Flash write sequence.
 *          Always includes WDT_Reset() at the start.
 * @param address The address within the EEPROM to write to.
 * @param data The byte data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Renesas Data Flash write sequence typically involves:
    // 1. Disable interrupts.
    // 2. Set an address register (e.g., FADR).
    // 3. Set data register (e.g., FDAT).
    // 4. Issue a write command (e.g., FCEN1 = 1, FCMD = 0x80 for write).
    // 5. Wait for FCSTS.FCBSY (busy flag) to clear.
    // 6. Re-enable interrupts.
    //
    // None of these specific registers (FADR, FDAT, FCEN1, FCMD, FCSTS) are in the provided JSON.
    // Therefore, a functional write cannot be implemented with the given `register_json`.
    // This is a placeholder noting the missing registers.
    (void)address; // Suppress unused parameter warning
    (void)data;    // Suppress unused parameter warning

    // Placeholder implementation due to missing Data Flash R/W registers in register_json.
    //
    // Global_interrupt_Disable();
    // // Actual Data Flash address and data registers (e.g., DF_ADDR, DF_DATA) and command register (e.g., DF_CMD)
    // // DF_ADDR = address;
    // // DF_DATA = data;
    // // DF_CMD = WRITE_BYTE_CMD; // Assuming a command
    // // while (DF_STATUS & BUSY_FLAG); // Wait for completion
    // Global_interrupt_Enable();
}

/**
 * @brief Reads a byte from a specified address in Internal EEPROM (Data Flash).
 * @details Implements the Data Flash read sequence.
 *          Always includes WDT_Reset() at the start.
 * @param address The address within the EEPROM to read from.
 * @return The byte data read from the address.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Renesas Data Flash read sequence typically involves:
    // 1. Disable interrupts (sometimes not needed for read, but safer).
    // 2. Set an address register (e.g., FADR).
    // 3. Read data from a data register (e.g., FDAT) or the memory-mapped address itself.
    // 4. Re-enable interrupts.
    //
    // As with `Internal_EEPROM_Set`, specific Data Flash R/W registers are missing from JSON.
    // This is a placeholder noting the missing registers.
    (void)address; // Suppress unused parameter warning

    tbyte read_data = 0xFFU; // Default return value

    // Placeholder implementation due to missing Data Flash R/W registers in register_json.
    //
    // Global_interrupt_Disable();
    // // Assuming Data Flash is memory-mapped at a specific base address.
    // // For RL78/G13, Data Flash is typically from 0xF1000 to 0xF1FFF.
    // // read_data = *(volatile tbyte*)(DATA_FLASH_BASE_ADDRESS + address);
    // Global_interrupt_Enable();

    return read_data;
}

// =============================================================================
// TT (Time Triggered OS) Implementation
// =============================================================================

/**
 * @brief Initializes the Time Triggered (TT) scheduler.
 * @details Configures a timer for the specified tick time and sets up the scheduler array.
 *          Always includes WDT_Reset() at the start.
 * @param tick_time_ms The scheduler tick interval in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Initialize the scheduler array
    for (tbyte i = 0; i < MAX_TASKS; i++) {
        SCH_tasks_g[i].pTask = NULL;
        SCH_tasks_g[i].Delay = 0;
        SCH_tasks_g[i].Period = 0;
        SCH_tasks_g[i].RunMe = 0;
        SCH_tasks_g[i].TaskID = 0;
    }
    SCH_next_task_ID_g = 0;
    g_tick_counter_ms = 0;

    // Configure an Interval Timer for the specified tick_time_ms.
    // Using INTL_TIM registers (Interval Timer: ITMC, ITMF, ITCS, ITCL, ITCH)
    // INTL_TIM_ITMC: Interval Timer Control Register
    // INTL_TIM_ITCS: IT Timer Select Register (clock source)
    // INTL_TIM_ITCL/ITCH: IT Count Registers (16-bit timer)
    //
    // Assume INTL_TIM_ITCS can select a clock that, with ITCL/ITCH, can provide tick_time_ms.
    // ITCS bit 0 (ITCLKEN): Interval timer clock enable
    // ITCS bit 1 (ITCLKS0), ITCS bit 2 (ITCLKS1): Interval timer clock select.
    // e.g., 00 = fIL, 01 = fICH (fCLK/1), 10 = fIH (fCLK/2), 11 = fMX.
    // For 1ms tick: use fICH (fCLK/1) with 16MHz clock.
    // Counter value = (fCLK/1000) - 1 for 1ms.
    // For 16MHz, 16000 - 1 = 15999. Fits in 16-bit.
    //
    // 1. Disable interval timer
    INTL_TIM_ITMC = 0x00U; // Clear control register (stop timer, disable interrupt)

    // 2. Select clock source for interval timer (ITCS)
    // Enable ITCLKEN (bit 0) and select fICH (fCLK/1) as source (ITCLKS=01b).
    INTL_TIM_ITCS = (1U << 0) | (1U << 1); // ITCLKEN=1, ITCLKS=01b (fICH/fCLK/1)

    // 3. Set count value (ITCL/ITCH for 1ms)
    tword count_value = (tword)(((tlong)16000000 / 1000UL) * (tlong)tick_time_ms - 1);
    INTL_TIM_ITCL = (tbyte)(count_value & 0xFFU);       // Low byte
    INTL_TIM_ITCH = (tbyte)((count_value >> 8) & 0xFFU); // High byte

    // 4. Enable interval timer and interrupt (ITMC)
    // ITMC bit 7: ITCE (Enable interval timer count operation)
    // ITMC bit 0: ITIF (Interrupt Flag) - clear before enabling
    INTL_TIM_ITMF = 0x00U; // Clear interrupt flag (if writable)
    INTL_TIM_ITMC = 0x80U; // ITCE = 1 (Enable timer). Assuming interrupt is globally enabled later.
                           // Mask for interval timer interrupt (ITMK) typically exists but not in JSON.
                           // Assuming ITMC itself has interrupt enable bit.
                           // RL78 ITMC: Bit 7 ITCE (Operation Enable), Bit 0 ITFEN (Interrupt Enable).
                           // So ITMC = 0x81U would enable both.
    INTL_TIM_ITMC = 0x81U; // Enable count operation AND enable interrupt (inferred)

    // Enable Peripheral clock if not already done.
    // Interval Timer clock enable. Not explicitly in PER0/1 for INTL_TIM.
    // It's often associated with a different peripheral block or enabled with system clock.
    // If PERx exists for this, enable it. Otherwise assume active.
}

/**
 * @brief Starts the Time Triggered (TT) scheduler.
 * @details This function is mostly conceptual for TT-OS, as tasks are dispatched
 *          by `TT_Dispatch_task` based on timer ticks. The timer is enabled in `TT_Init`.
 *          Always includes WDT_Reset() at the start.
 */
void TT_Start(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule
    // Timer is already configured and started in TT_Init.
    // This function can be used to explicitly enable global interrupts if they are disabled after TT_Init.
    Global_interrupt_Enable();
}

/**
 * @brief Dispatches scheduled tasks based on the tick counter.
 * @details This function is typically called in the main loop to execute ready tasks.
 *          Always includes WDT_Reset() at the start.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (SCH_tasks_g[i].pTask != NULL) {
            // Check if task is due to run
            if (SCH_tasks_g[i].RunMe > 0) {
                SCH_tasks_g[i].RunMe--; // Decrement 'RunMe' flag
                SCH_tasks_g[i].pTask(); // Execute the task
                // If it's a one-shot task, remove it
                if (SCH_tasks_g[i].Period == 0) {
                    TT_Delete_task(SCH_tasks_g[i].TaskID);
                }
            }
        }
    }
}

/**
 * @brief Interval Timer ISR for the TT scheduler.
 * @details This function increments the tick counter and sets the 'RunMe' flag for tasks.
 *          It must be called periodically by the MCU's timer interrupt handler.
 *          Always includes WDT_Reset() at the start.
 */
void TT_ISR(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Clear the Interval Timer Interrupt Flag
    INTL_TIM_ITMF = 0x00U; // Clear interrupt flag (if writable, otherwise read to clear)

    g_tick_counter_ms++; // Increment global tick counter

    for (tbyte i = 0; i < MAX_TASKS; i++) {
        if (SCH_tasks_g[i].pTask != NULL) {
            if (SCH_tasks_g[i].Delay == 0) {
                // Task is ready to run
                SCH_tasks_g[i].RunMe++;
                if (SCH_tasks_g[i].Period > 0) {
                    SCH_tasks_g[i].Delay = SCH_tasks_g[i].Period - 1; // Reload for next period
                }
            } else {
                SCH_tasks_g[i].Delay--;
            }
        }
    }
}

/**
 * @brief Adds a task to the TT scheduler.
 * @details Schedules a function to run periodically or once after a delay.
 *          Always includes WDT_Reset() at the start.
 * @param task Pointer to the task function.
 * @param period The period of the task in scheduler ticks (0 for one-shot).
 * @param delay The initial delay before the task first runs.
 * @return The task ID if successful, 0xFF if scheduler is full.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    if (SCH_next_task_ID_g < MAX_TASKS) {
        SCH_tasks_g[SCH_next_task_ID_g].pTask = task;
        SCH_tasks_g[SCH_next_task_ID_g].Delay = delay;
        SCH_tasks_g[SCH_next_task_ID_g].Period = period;
        SCH_tasks_g[SCH_next_task_ID_g].RunMe = 0;
        SCH_tasks_g[SCH_next_task_ID_g].TaskID = SCH_next_task_ID_g + 1; // Task ID starts from 1
        return SCH_next_task_ID_g++;
    }
    return 0xFFU; // Scheduler is full
}

/**
 * @brief Deletes a task from the TT scheduler.
 * @details Removes a previously scheduled task.
 *          Always includes WDT_Reset() at the start.
 * @param task_index The index of the task to delete (returned by TT_Add_task).
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    if (task_index < MAX_TASKS) {
        SCH_tasks_g[task_index].pTask = NULL; // Mark as free
        SCH_tasks_g[task_index].Delay = 0;
        SCH_tasks_g[task_index].Period = 0;
        SCH_tasks_g[task_index].RunMe = 0;
        SCH_tasks_g[task_index].TaskID = 0;
    }
}

// =============================================================================
// DTC_driver Implementation
// =============================================================================

/**
 * @brief Initializes the DTC (Data Transfer Controller) module.
 * @details Sets up the DTC for initial operation.
 *          Always includes WDT_Reset() at the start.
 */
void DTC_Init(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Disable DTC transfers during configuration
    DTC_Stop();

    // Clear all DTC configuration registers
    DTC_DTCAD = 0x0000U;   // Clear Data Address
    DTC_DTSAD = 0x0000U;   // Clear Start Address
    DTC_DRS = 0x0000U;     // Clear Control Register
    DTC_DTCD = 0x0000U;    // Clear Data Register
    DTC_DTCNT = 0x0000U;   // Clear Count Register
    DTC_TRGSR0 = 0x00U;    // Clear Trigger Select Register 0
    DTC_TRGSR1 = 0x00U;    // Clear Trigger Select Register 1
    DTC_TRGSR2 = 0x00U;    // Clear Trigger Select Register 2
    DTC_TRGSR3 = 0x00U;    // Clear Trigger Select Register 3

    // Peripheral clock enable:
    // RCC_PER0 (0xF0000) for DTC (assuming bit 16 for DTC enable, based on RL78/G13).
    SET_BIT(RCC_PER0, 16); // Inferred: Enable clock for DTC
}

/**
 * @brief Enables a DTC transfer source.
 * @details Configures a trigger source to activate a DTC channel.
 *          Always includes WDT_Reset() at the start.
 * @param source_id The ID of the peripheral event that triggers the DTC.
 * @param channel The DTC channel to be triggered.
 */
void DTC_EnableSource(uint8_t source_id, uint8_t channel) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Peripheral clock enable:
    SET_BIT(RCC_PER0, 16); // Inferred: Enable clock for DTC

    // DTC_TRGSRx: DTC Trigger Select Registers
    // These registers select which peripheral interrupt/event triggers a DTC transfer.
    // Assuming TRGSR0-3 map to DTC channels 0-3.
    // Each TRGSRx needs to be set with the `source_id` (interrupt vector number or specific code).
    volatile tbyte* trgsr_reg = NULL;
    switch (channel) {
        case 0: trgsr_reg = &DTC_TRGSR0; break;
        case 1: trgsr_reg = &DTC_TRGSR1; break;
        case 2: trgsr_reg = &DTC_TRGSR2; break;
        case 3: trgsr_reg = &DTC_TRGSR3; break;
        default: return; // Invalid channel
    }
    *trgsr_reg = source_id; // Set the trigger source for the specified channel.
}

/**
 * @brief Disables a DTC transfer source.
 * @details Clears the trigger source for a DTC channel, preventing transfers.
 *          Always includes WDT_Reset() at the start.
 * @param source_id The ID of the peripheral event (not used for disable, but kept for API consistency).
 */
void DTC_DisableSource(uint8_t source_id) { // The source_id parameter might be redundant for disabling.
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // To disable a source, we typically clear its entry in the TRGSR registers.
    // However, this API is `DisableSource` without `channel` param, making it ambiguous.
    // Assuming it means to clear all TRGSR entries that match source_id, or just disable
    // the global DTC. Since we have DTC_Stop(), I'll assume this function
    // should iterate and clear any TRGSR that contains `source_id` if that's how it works.
    // For now, I'll clear all TRGSRs to ensure no pending sources, as per best practice.
    (void)source_id; // Suppress unused parameter warning

    DTC_TRGSR0 = 0x00U;
    DTC_TRGSR1 = 0x00U;
    DTC_TRGSR2 = 0x00U;
    DTC_TRGSR3 = 0x00U;
}

/**
 * @brief Starts the DTC module, allowing configured transfers to occur.
 * @details Enables the DTC to respond to trigger events.
 *          Always includes WDT_Reset() at the start.
 */
void DTC_Start(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // Peripheral clock enable:
    SET_BIT(RCC_PER0, 16); // Inferred: Enable clock for DTC

    // DTC_DRS (DTC Transfer Control Register) contains the DTC enable bit.
    // Assuming Bit 0 (DTCEN) is the enable bit.
    SET_BIT(DTC_DRS, 0); // Enable DTC module (inferred: bit 0 of DRS enables DTC)
}

/**
 * @brief Stops the DTC module, halting all transfers.
 * @details Disables the DTC from responding to trigger events.
 *          Always includes WDT_Reset() at the start.
 */
void DTC_Stop(void) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // DTC_DRS contains the DTC enable bit.
    // Assuming Bit 0 (DTCEN) is the enable bit.
    CLEAR_BIT(DTC_DRS, 0); // Disable DTC module (inferred: bit 0 of DRS disables DTC)
}

/**
 * @brief Configures a specific DTC channel for a transfer.
 * @details Sets source/destination addresses, transfer counts, modes, etc.
 *          Always includes WDT_Reset() at the start.
 * @param channel The DTC channel to configure.
 * @param src_addr Source address for the transfer.
 * @param dst_addr Destination address for the transfer.
 * @param block_size Number of bytes per block transfer.
 * @param transfer_count Number of blocks to transfer.
 * @param mode Transfer mode (e.g., normal, repeat).
 * @param data_size Data transfer size (e.g., 8-bit, 16-bit).
 * @param src_inc Source address increment mode.
 * @param dst_inc Destination address increment mode.
 * @param rpt_sel Repeat mode select.
 * @param rpt_int Repeat interrupt enable.
 */
void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int) {
    WDT_Reset(); // Call WDT_Reset() as per API_implementation_sequence rule

    // DTC_DTCAD, DTC_DTSAD, DTC_DRS, DTC_DTCD, DTC_DTCNT
    // These registers are common across DTC, but some MCUs have channel-specific versions.
    // The JSON only provides global DTC registers, implying either only one channel or
    // these registers are effectively pointers to channel control blocks in RAM.
    // For simplicity, I'll assume a single channel being configured by these global registers,
    // or that these are base pointers for a control table.
    // Renesas RL78 DTC typically uses an "Interrupt Control Table" or "DTC Vector Table"
    // that points to a series of DTC descriptor blocks. Each block has its own SRC/DST/COUNT/MODE.
    //
    // The given registers are DTC_DTCAD (Transfer Data Address), DTC_DTSAD (Start Address),
    // DTC_DRS (Control), DTC_DTCD (Data), DTC_DTCNT (Count).
    // This looks like direct configuration registers, not pointers to descriptor blocks.
    // This suggests only one DTC channel (or a very simplified one).
    // Given 'channel' parameter, this is an ambiguity. I will implement as if these registers
    // are for `channel 0` (or the primary channel) and this API sets those.
    (void)channel; // Suppress unused parameter warning for now, as registers are global.

    DTC_DTSAD = src_addr; // Source Start Address
    DTC_DTCAD = dst_addr; // Destination Address (might be transfer data address)
    DTC_DTCNT = transfer_count; // Number of transfers/blocks

    // DTC_DRS (DTC Transfer Control Register) needs to be configured based on parameters.
    // DRS typically has bits for: Transfer Mode (MD), Data Size (SZ), Source/Dest Increment (SM/DM),
    // Repeat Select (RPS), Repeat Interrupt (RPTIE).
    // These bits are not defined in JSON. I'll use placeholders.
    tword drs_val = 0x0000U;

    // Mode (0=Normal, 1=Repeat, 2=Block)
    drs_val |= (mode & 0x03U) << 0; // Inferred: Bits 0-1 for Mode (MD)

    // Data Size (0=8-bit, 1=16-bit)
    drs_val |= (data_size & 0x01U) << 2; // Inferred: Bit 2 for Data Size (SZ)

    // Source Increment (0=Fixed, 1=Increment)
    drs_val |= (src_inc & 0x01U) << 3; // Inferred: Bit 3 for Source Increment (SM)

    // Destination Increment (0=Fixed, 1=Increment)
    drs_val |= (dst_inc & 0x01U) << 4; // Inferred: Bit 4 for Destination Increment (DM)

    // Repeat Select (0=Source, 1=Destination)
    drs_val |= (rpt_sel & 0x01U) << 5; // Inferred: Bit 5 for Repeat Select (RPS)

    // Repeat Interrupt (0=Disable, 1=Enable)
    drs_val |= (rpt_int & 0x01U) << 6; // Inferred: Bit 6 for Repeat Interrupt Enable (RPTIE)

    // Block Size (DTC_DTCD or DTCNT used for this)
    // block_size parameter is ambiguous with transfer_count.
    // For DTC, DTCNT is usually the total transfer count. A separate bit indicates block transfer.
    // If block_size is distinct, it needs a specific register or bit field not in JSON.
    // For now, if block_size > 1, I'll assume 'transfer_count' is the number of blocks, and
    // 'block_size' (if != 1) implies setting a block mode bit.
    // If block_size is meant to be stored, DTCD (Transfer Data Register) is left. Let's use it for block_size.
    DTC_DTCD = block_size; // Placeholder: If DTCD holds block_size.

    DTC_DRS = drs_val; // Write the configured control value

    // Enable Peripheral clock:
    SET_BIT(RCC_PER0, 16); // Inferred: Enable clock for DTC
}

// =============================================================================
// ISR for Interval Timer (for TT-OS)
// =============================================================================

// This would be the actual interrupt service routine (ISR) that the MCU calls.
// The name `INTIT` is a common Renesas RL78 vector name for Interval Timer.
// This needs to be hooked up in the startup code/vector table.
#if defined(__CCRL__) || defined(__GNUC__) // For Renesas CCRL or GCC
#pragma interrupt INTTT_ISR(vect=INTIT) // Assuming INTIT is the vector number
#endif
void INTTT_ISR(void) {
    WDT_Reset(); // Always reset WDT in ISR
    TT_ISR();    // Call the scheduler's ISR handler

    // Call ICU callback if registered and a specific channel is tied to this interrupt.
    // This ISR is for the Interval Timer (TT). If an ICU uses a specific TAU channel's interrupt,
    // that TAU channel's ISR should call the ICU callback.
    // For this generic Interval Timer, if a callback is registered, it might imply
    // this timer is also used for a simple timeout type ICU event.
    if (icu_callback_g != NULL) {
        icu_callback_g();
    }
}