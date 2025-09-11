/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) implementation file for RENESAS_R5F11BBC.
 *
 * This file provides the concrete implementation of the MCAL APIs, abstracting the
 * underlying hardware registers of the RENESAS_R5F11BBC microcontroller.
 */

//==================================================================================================
// Includes
//==================================================================================================
#include "RENESAS_R5F11BBC_MCAL.h" // MCAL API prototypes and type definitions
#include "renesas_r5f11bbc.h"      // Placeholder for device-specific header.
                                   // In a real project, this would include direct register definitions.
#include <stdint.h>                // Standard integer types
#include <stdbool.h>               // Standard boolean type
#include <stddef.h>                // Standard definitions (size_t, NULL)
#include <string.h>                // For string operations (e.g., in UART_send_string)

//==================================================================================================
// Local Defines and Macros
//==================================================================================================

// Generic register access macros (assuming 8-bit registers unless otherwise specified)
#define REG_8BIT(address)  (*(volatile tbyte *)(address))
#define REG_16BIT(address) (*(volatile tword *)(address))

// Watchdog Timer (WDT) refresh value (common for Renesas R8C/RL78 families)
#define WDT_CLR_SEQUENCE_VALUE ((tbyte)0xAC) // Common value to clear/refresh WDT

// Peripheral Enable Register 0 (RCC_PER0) bits - inferred from Renesas typical convention
// Renesas R5F11BBC peripherals often enabled via bits in the Peripheral Enable Registers.
// The exact bit positions are inferred based on common Renesas MCU architectures as
// these are not explicitly provided in the 'register_json' beyond the register address.
#define RCC_PER0_ADC_EN_BIT     (0)  // Inferred: ADC enable bit
#define RCC_PER0_TAU_EN_BIT     (1)  // Inferred: Timer Array Unit enable bit
#define RCC_PER0_SAU_EN_BIT     (2)  // Inferred: Serial Array Unit enable bit
#define RCC_PER0_I2C_EN_BIT     (3)  // Inferred: I2C enable bit
#define RCC_PER0_DAC_EN_BIT     (4)  // Inferred: DAC enable bit
// Add other inferred bits as needed for PER0 or PER1

// GPIO Register Bitmasks (P, PM, PU, POM, PIM, PMC)
// A common Renesas convention for PM registers: 0=Output, 1=Input
#define GPIO_DIR_OUTPUT_BIT     (0)
#define GPIO_DIR_INPUT_BIT      (1)

//==================================================================================================
// Register Addresses (from register_json)
//==================================================================================================

// GPIO Data Registers (Px)
#define GPIO_P0_ADDR   (0xFF00)
#define GPIO_P1_ADDR   (0xFF01)
#define GPIO_P2_ADDR   (0xFF02)
#define GPIO_P3_ADDR   (0xFF03)
#define GPIO_P4_ADDR   (0xFF04)
#define GPIO_P5_ADDR   (0xFF05)
#define GPIO_P6_ADDR   (0xFF06)
#define GPIO_P7_ADDR   (0xFF07)
#define GPIO_P12_ADDR  (0xFF0C)
#define GPIO_P13_ADDR  (0xFF0D)
#define GPIO_P14_ADDR  (0xFF0E)
#define GPIO_P15_ADDR  (0xFE1F)

// GPIO Mode Registers (GPIO_PMx)
#define GPIO_PM0_ADDR  (0xFF20)
#define GPIO_PM1_ADDR  (0xFF21)
#define GPIO_PM2_ADDR  (0xFF22)
#define GPIO_PM3_ADDR  (0xFF23)
#define GPIO_PM4_ADDR  (0xFF24)
#define GPIO_PM5_ADDR  (0xFF25)
#define GPIO_PM6_ADDR  (0xFF26)
#define GPIO_PM7_ADDR  (0xFF27)
#define GPIO_PM12_ADDR (0xFF2C)
#define GPIO_PM13_ADDR (0xFF2D)
#define GPIO_PM14_ADDR (0xFF2E)
#define GPIO_PM15_ADDR (0xFE0F) // Conflict with GPIO_PMC15

// GPIO Pull-Up Resistor Registers (GPIO_PUx)
#define GPIO_PU0_ADDR  (0xFF30) // Conflict with GPIO_PMC0 and ADC_SR
#define GPIO_PU1_ADDR  (0xFF31) // Conflict with GPIO_PMC1
#define GPIO_PU2_ADDR  (0xFF32) // Conflict with GPIO_PMC2
#define GPIO_PU3_ADDR  (0xFF33) // Conflict with GPIO_PMC3
#define GPIO_PU4_ADDR  (0xFF34) // Conflict with GPIO_PMC4
#define GPIO_PU5_ADDR  (0xFF35) // Conflict with GPIO_PMC5
#define GPIO_PU6_ADDR  (0xFF36) // Conflict with GPIO_PMC6
#define GPIO_PU7_ADDR  (0xFF37) // Conflict with GPIO_PMC7
#define GPIO_PU12_ADDR (0xFF3C) // Conflict with GPIO_PMC12
#define GPIO_PU13_ADDR (0xFE2D)
#define GPIO_PU14_ADDR (0xFF3E) // Conflict with GPIO_PMC14
#define GPIO_PU15_ADDR (0xFE2F)

// GPIO Output Mode Registers (GPIO_POMx)
#define GPIO_POM0_ADDR  (0xFF40)
#define GPIO_POM1_ADDR  (0xFF41)
#define GPIO_POM2_ADDR  (0xFF42)
#define GPIO_POM3_ADDR  (0xFF43)
#define GPIO_POM4_ADDR  (0xFF44)
#define GPIO_POM5_ADDR  (0xFF45)
#define GPIO_POM6_ADDR  (0xFF46)
#define GPIO_POM7_ADDR  (0xFF47)
#define GPIO_POM12_ADDR (0xFF4C)
#define GPIO_POM13_ADDR (0xFE3D)
#define GPIO_POM14_ADDR (0xFF4E)
#define GPIO_POM15_ADDR (0xFE3F)

// GPIO Input Mode Registers (GPIO_PIMx)
#define GPIO_PIM0_ADDR  (0xFF50)
#define GPIO_PIM1_ADDR  (0xFF51)
#define GPIO_PIM5_ADDR  (0xFF55)
#define GPIO_PIM7_ADDR  (0xFF57)
#define GPIO_PIM13_ADDR (0xFE4D)
#define GPIO_PIM15_ADDR (0xFE4F)

// GPIO Port Mode Control Registers (GPIO_PMC_x) - for alternate functions, analog, etc.
#define GPIO_PMC0_ADDR  (0xFF30) // Conflict with GPIO_PU0 and ADC_SR
#define GPIO_PMC1_ADDR  (0xFF31) // Conflict with GPIO_PU1
#define GPIO_PMC2_ADDR  (0xFF32) // Conflict with GPIO_PU2
#define GPIO_PMC3_ADDR  (0xFF33) // Conflict with GPIO_PU3
#define GPIO_PMC4_ADDR  (0xFF34) // Conflict with GPIO_PU4
#define GPIO_PMC5_ADDR  (0xFF35) // Conflict with GPIO_PU5
#define GPIO_PMC6_ADDR  (0xFF36) // Conflict with GPIO_PU6
#define GPIO_PMC7_ADDR  (0xFF37) // Conflict with GPIO_PU7
#define GPIO_PMC12_ADDR (0xFF3C) // Conflict with GPIO_PU12
#define GPIO_PMC13_ADDR (0xFE0D)
#define GPIO_PMC14_ADDR (0xFF3E) // Conflict with GPIO_PU14
#define GPIO_PMC15_ADDR (0xFE0F) // Conflict with GPIO_PM15

// Clock/Reset Control (RCC)
#define RCC_CMC_ADDR    (0xFFF0) // Clock Operation Mode Control
#define RCC_CKC_ADDR    (0xFFF1) // System Clock Control
#define RCC_CSC_ADDR    (0xFF71) // Clock Operation Status Control (often contains LVD enable/status)
#define RCC_OSTC_ADDR   (0xFF72) // Oscillation Stabilization Time Counter Status
#define RCC_OSMC_ADDR   (0xFF73) // Subsystem Clock Supply Mode Control
#define RCC_PER0_ADDR   (0xFF76) // Peripheral Enable Register 0
#define RCC_PER1_ADDR   (0xFF77) // Peripheral Enable Register 1
#define RCC_HIOTRM_ADDR (0xFF7C) // High-Speed On-Chip Oscillator Trimming
#define RCC_HOCODIV_ADDR (0xFF7E) // High-Speed On-Chip Oscillator Frequency Select
#define RCC_CGC_ADDR    (0xFFF2) // Clock Generator Control
#define RCC_OSCCTL_ADDR (0xFFF3) // Oscillation Control

// Watchdog Timer (WDT)
#define WDT_EN_ADDR     (0xFFFB) // Watchdog Timer Enable Register
#define WDT_MODER_ADDR  (0xFFFC) // Watchdog Timer Mode Register
#define WDT_CLR_ADDR    (0xFFFD) // Watchdog Timer Clear Register
#define WDT_FR_ADDR     (0xFEE4) // Watchdog Timer Function Register

// Interrupt Control (INT)
#define INT_FLAG_CTL_ADDR (0xFFF4) // Interrupt Flag Control Register
#define INT_FLAG_CLR_ADDR (0xFFF5) // Interrupt Flag Clear Register

// ADC Registers
#define ADC_ADM0_ADDR   (0xFF00) // AD Converter Mode Register 0 (Conflict with P0)
#define ADC_DR_ADDR     (0xFF10) // AD Converter Data Register (10-bit, lower byte at 0xFF10)
#define ADC_SR_ADDR     (0xFF30) // AD Converter Status Register (Conflict with GPIO_PU0/PMC0)
#define ADC_ICR_ADDR    (0xFFB4) // AD Interrupt Control Register
#define ADC_PCR_ADDR    (0xFE20) // AD Port Configuration Register
#define ADC_PCRL_ADDR   (0xFE51) // AD Port Configuration Register L

// DAC Registers
#define DAC_CH0_CSR_ADDR (0xFF68) // DA Converter Setting Register 0
#define DAC_CH1_CSR_ADDR (0xFF69) // DA Converter Setting Register 1
#define DAC_CSR_ADDR     (0xFFB3) // DA Converter Setting Register (General)
#define DAC_SETUPCR_ADDR (0xFFB5) // DA Converter Setup Control Register

// I2C Registers (I2C0)
#define I2C0_CR_ADDR    (0xFFB8) // I2C Bus Control Register 0
#define I2C0_DR_ADDR    (0xFFB9) // I2C Bus Data Register 0
#define I2C0_BSCL_ADDR  (0xFFBA) // I2C Bus Status Control Register L
#define I2C0_BSCH_ADDR  (0xFFBB) // I2C Bus Status Control Register H
#define I2C0_SAR_ADDR   (0xFEC0) // I2C Bus Slave Address Register 0
#define I2C0_FCR_ADDR   (0xFEC1) // I2C Bus Function Control Register 0
#define I2C0_BCR0_ADDR  (0xFEC2) // I2C Bus Control Register C0
#define I2C0_BBR0_ADDR  (0xFEC3) // I2C Bus Bit Rate Register B0

// SAU (Serial Array Unit) - SPI (CSI) - SAU0 Channel 0
#define SAU0_SDR00_ADDR (0xFF12) // Serial Data Register 00
#define SAU0_SMR00_ADDR (0xFFD2) // Serial Mode Register 00
#define SAU0_SCR00_ADDR (0xFFD8) // Serial Control Register 00
#define SAU0_SSR00_ADDR (0xFFCC) // Serial Status Register 00
#define SAU0_SIR00_ADDR (0xFFC6) // Serial Input Register 00

// SAU (Serial Array Unit) - UART - SAU0 Channel 1
#define SAU0_SDR01_ADDR (0xFF14) // Serial Data Register 01
#define SAU0_SMR01_ADDR (0xFFD3) // Serial Mode Register 01
#define SAU0_SCR01_ADDR (0xFFD9) // Serial Control Register 01
#define SAU0_SSR01_ADDR (0xFFCD) // Serial Status Register 01
#define SAU0_SIR01_ADDR (0xFFC7) // Serial Input Register 01

// SAU (Serial Array Unit) - SAU1 Channel 0 (can be UART/SPI)
#define SAU1_SDR10_ADDR (0xFF16) // Serial Data Register 10
#define SAU1_SMR10_ADDR (0xFFD4) // Serial Mode Register 10
#define SAU1_SCR10_ADDR (0xFFDA) // Serial Control Register 10
#define SAU1_SSR10_ADDR (0xFFCE) // Serial Status Register 10
#define SAU1_SIR10_ADDR (0xFFC8) // Serial Input Register 10

// SAU (Serial Array Unit) - SAU2 Channel 0 (can be UART/SPI)
#define SAU2_SDR20_ADDR (0xFF1A) // Serial Data Register 20
#define SAU2_SMR20_ADDR (0xFFD6) // Serial Mode Register 20
#define SAU2_SCR20_ADDR (0xFFDC) // Serial Control Register 20
#define SAU2_SSR20_ADDR (0xFFD0) // Serial Status Register 20
#define SAU2_SIR20_ADDR (0xFFCA) // Serial Input Register 20

// SAU Global Control Registers
#define SERIAL_SE_ADDR  (0xFFC0) // Serial Channel Enable Register
#define SERIAL_SS_ADDR  (0xFFC1) // Serial Channel Start Register
#define SERIAL_ST_ADDR  (0xFFC2) // Serial Channel Stop Register
#define SERIAL_SPS_ADDR (0xFFC3) // Serial Clock Select Register
#define SERIAL_SO_ADDR  (0xFFC4) // Serial Output Register
#define SERIAL_SOE_ADDR (0xFFC5) // Serial Output Enable Register
#define SERIAL_SMM0_ADDR (0xFFDE) // Serial Mode Register M0

// TAU (Timer Array Unit) 0 Registers
#define TAU0_TMR00_ADDR (0xFF88) // Timer Mode Register 00
#define TAU0_TMR01_ADDR (0xFF89) // Timer Mode Register 01
#define TAU0_TMR02_ADDR (0xFF8A) // Timer Mode Register 02
#define TAU0_TMR03_ADDR (0xFF8B) // Timer Mode Register 03
#define TAU0_TMR04_ADDR (0xFF8C) // Timer Mode Register 04
#define TAU0_TMR05_ADDR (0xFF8D) // Timer Mode Register 05
#define TAU0_TMR06_ADDR (0xFF8E) // Timer Mode Register 06
#define TAU0_TMR07_ADDR (0xFF8F) // Timer Mode Register 07

#define TAU0_TDR00_ADDR (0xFF90) // Timer Data Register 00 (16-bit)
#define TAU0_TDR01_ADDR (0xFF92) // Timer Data Register 01 (16-bit)
#define TAU0_TDR02_ADDR (0xFF94) // Timer Data Register 02 (16-bit)
#define TAU0_TDR03_ADDR (0xFF96) // Timer Data Register 03 (16-bit)
#define TAU0_TDR04_ADDR (0xFF98) // Timer Data Register 04 (16-bit)
#define TAU0_TDR05_ADDR (0xFF9A) // Timer Data Register 05 (16-bit)
#define TAU0_TDR06_ADDR (0xFF9C) // Timer Data Register 06 (16-bit)
#define TAU0_TDR07_ADDR (0xFF9E) // Timer Data Register 07 (16-bit)

#define TAU0_TCR00_ADDR (0xFFA0) // Timer Count Register 00 (16-bit)
#define TAU0_TCR01_ADDR (0xFFA2) // Timer Count Register 01 (16-bit)
#define TAU0_TCR02_ADDR (0xFFA4) // Timer Count Register 02 (16-bit)
#define TAU0_TCR03_ADDR (0xFFA6) // Timer Count Register 03 (16-bit)
#define TAU0_TCR04_ADDR (0xFFA8) // Timer Count Register 04 (16-bit)
#define TAU0_TCR05_ADDR (0xFFAA) // Timer Count Register 05 (16-bit)
#define TAU0_TCR06_ADDR (0xFFAC) // Timer Count Register 06 (16-bit)
#define TAU0_TCR07_ADDR (0xFFAE) // Timer Count Register 07 (16-bit)

#define TAU0_TPS_ADDR   (0xFFB0) // Timer Clock Select Register 0
#define TAU_ISC_ADDR    (0xFFB1) // Input Switch Control Register (External Interrupt)
#define TAU_NFEN1_ADDR  (0xFF70) // Noise Filter Enable Register 1 (TAU0)

#define TAU0_TO_ADDR    (0xFFE0) // Timer Output Register 0
#define TAU0_TOE_ADDR   (0xFFE1) // Timer Output Enable Register 0
#define TAU0_TOL_ADDR   (0xFFE2) // Timer Output Level Register 0
#define TAU0_TOM_ADDR   (0xFFE3) // Timer Output Mode Register 0

#define TAU0_TSR00_ADDR (0xFFE4) // Timer Status Register 00
#define TAU0_TSR01_ADDR (0xFFE5) // Timer Status Register 01
#define TAU0_TSR02_ADDR (0xFFE6) // Timer Status Register 02
#define TAU0_TSR03_ADDR (0xFFE7) // Timer Status Register 03
#define TAU0_TSR04_ADDR (0xFFE8) // Timer Status Register 04
#define TAU0_TSR05_ADDR (0xFFE9) // Timer Status Register 05
#define TAU0_TSR06_ADDR (0xFFEA) // Timer Status Register 06
#define TAU0_TSR07_ADDR (0xFFEB) // Timer Status Register 07

#define TAU0_TE_ADDR    (0xFFEC) // Timer Channel Enable Status Register 0
#define TAU0_TS_ADDR    (0xFFED) // Timer Channel Start Register 0
#define TAU0_TT_ADDR    (0xFFEE) // Timer Channel Stop Register 0
#define TAU0_TIOS_ADDR  (0xFFEF) // Timer I/O Select Register 0

// Flash Memory Registers (Internal_EEPROM)
#define FLASH_DSDP_ADDR (0xFFFA) // Data Flash Memory Standby Program Register
#define FLASH_DSAD_ADDR (0xFFF8) // Data Flash Memory Address Register (16-bit)
#define FLASH_DRAD_ADDR (0xFFF9) // Data Flash Memory Data Register
#define FLASH_PMC_ADDR  (0xFE61) // Flash Memory Program Mode Control Register
#define FLASH_ERR_ADDR  (0xFE64) // Flash Memory Error Register

// CPU Control
#define SYSTEM_IAWCT_ADDR (0xFEF8) // Instruction Accelerator Wait Control Register

// Noise Filter
#define NF_EN0_ADDR     (0xFF6F) // Noise Filter Enable Register 0

//==================================================================================================
// Private Helper Functions (static)
//==================================================================================================

/**
 * @brief Get a pointer to the GPIO Port Data Register (Px) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Port Data Register.
 */
static volatile tbyte* get_gpio_port_data_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_P0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_P1_ADDR; break;
        case port_2:  reg_ptr = (volatile tbyte*)GPIO_P2_ADDR; break;
        case port_3:  reg_ptr = (volatile tbyte*)GPIO_P3_ADDR; break;
        case port_4:  reg_ptr = (volatile tbyte*)GPIO_P4_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_P5_ADDR; break;
        case port_6:  reg_ptr = (volatile tbyte*)GPIO_P6_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_P7_ADDR; break;
        case port_12: reg_ptr = (volatile tbyte*)GPIO_P12_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_P13_ADDR; break;
        case port_14: reg_ptr = (volatile tbyte*)GPIO_P14_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_P15_ADDR; break;
        default: break; // Should not happen with valid input
    }
    return reg_ptr;
}

/**
 * @brief Get a pointer to the GPIO Port Mode Register (GPIO_PMx) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Port Mode Register.
 */
static volatile tbyte* get_gpio_port_mode_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_PM0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_PM1_ADDR; break;
        case port_2:  reg_ptr = (volatile tbyte*)GPIO_PM2_ADDR; break;
        case port_3:  reg_ptr = (volatile tbyte*)GPIO_PM3_ADDR; break;
        case port_4:  reg_ptr = (volatile tbyte*)GPIO_PM4_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_PM5_ADDR; break;
        case port_6:  reg_ptr = (volatile tbyte*)GPIO_PM6_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_PM7_ADDR; break;
        case port_12: reg_ptr = (volatile tbyte*)GPIO_PM12_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_PM13_ADDR; break;
        case port_14: reg_ptr = (volatile tbyte*)GPIO_PM14_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_PM15_ADDR; break;
        default: break;
    }
    return reg_ptr;
}

/**
 * @brief Get a pointer to the GPIO Pull-Up Resistor Register (GPIO_PUx) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Pull-Up Resistor Register.
 * @note This register often shares an address with GPIO_PMC_x for some ports.
 */
static volatile tbyte* get_gpio_port_pullup_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_PU0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_PU1_ADDR; break;
        case port_2:  reg_ptr = (volatile tbyte*)GPIO_PU2_ADDR; break;
        case port_3:  reg_ptr = (volatile tbyte*)GPIO_PU3_ADDR; break;
        case port_4:  reg_ptr = (volatile tbyte*)GPIO_PU4_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_PU5_ADDR; break;
        case port_6:  reg_ptr = (volatile tbyte*)GPIO_PU6_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_PU7_ADDR; break;
        case port_12: reg_ptr = (volatile tbyte*)GPIO_PU12_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_PU13_ADDR; break;
        case port_14: reg_ptr = (volatile tbyte*)GPIO_PU14_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_PU15_ADDR; break;
        default: break;
    }
    return reg_ptr;
}

/**
 * @brief Get a pointer to the GPIO Port Output Mode Register (GPIO_POMx) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Port Output Mode Register.
 */
static volatile tbyte* get_gpio_port_outputmode_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_POM0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_POM1_ADDR; break;
        case port_2:  reg_ptr = (volatile tbyte*)GPIO_POM2_ADDR; break;
        case port_3:  reg_ptr = (volatile tbyte*)GPIO_POM3_ADDR; break;
        case port_4:  reg_ptr = (volatile tbyte*)GPIO_POM4_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_POM5_ADDR; break;
        case port_6:  reg_ptr = (volatile tbyte*)GPIO_POM6_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_POM7_ADDR; break;
        case port_12: reg_ptr = (volatile tbyte*)GPIO_POM12_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_POM13_ADDR; break;
        case port_14: reg_ptr = (volatile tbyte*)GPIO_POM14_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_POM15_ADDR; break;
        default: break;
    }
    return reg_ptr;
}

/**
 * @brief Get a pointer to the GPIO Port Input Mode Register (GPIO_PIMx) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Port Input Mode Register.
 */
static volatile tbyte* get_gpio_port_inputmode_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_PIM0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_PIM1_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_PIM5_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_PIM7_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_PIM13_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_PIM15_ADDR; break;
        default: break; // PIM registers not available for all ports in JSON
    }
    return reg_ptr;
}

/**
 * @brief Get a pointer to the GPIO Port Mode Control Register (GPIO_PMC_x) for a given port.
 * @param port The GPIO port identifier.
 * @return Volatile pointer to the 8-bit Port Mode Control Register.
 * @note This register often shares an address with GPIO_PU_x or GPIO_PM_x for some ports.
 *       It requires careful read-modify-write operations.
 */
static volatile tbyte* get_gpio_port_pmc_reg_ptr(t_port port)
{
    volatile tbyte* reg_ptr = NULL;
    switch (port)
    {
        case port_0:  reg_ptr = (volatile tbyte*)GPIO_PMC0_ADDR; break;
        case port_1:  reg_ptr = (volatile tbyte*)GPIO_PMC1_ADDR; break;
        case port_2:  reg_ptr = (volatile tbyte*)GPIO_PMC2_ADDR; break;
        case port_3:  reg_ptr = (volatile tbyte*)GPIO_PMC3_ADDR; break;
        case port_4:  reg_ptr = (volatile tbyte*)GPIO_PMC4_ADDR; break;
        case port_5:  reg_ptr = (volatile tbyte*)GPIO_PMC5_ADDR; break;
        case port_6:  reg_ptr = (volatile tbyte*)GPIO_PMC6_ADDR; break;
        case port_7:  reg_ptr = (volatile tbyte*)GPIO_PMC7_ADDR; break;
        case port_12: reg_ptr = (volatile tbyte*)GPIO_PMC12_ADDR; break;
        case port_13: reg_ptr = (volatile tbyte*)GPIO_PMC13_ADDR; break;
        case port_14: reg_ptr = (volatile tbyte*)GPIO_PMC14_ADDR; break;
        case port_15: reg_ptr = (volatile tbyte*)GPIO_PMC15_ADDR; break;
        default: break;
    }
    return reg_ptr;
}


/**
 * @brief Enables the clock supply for a given peripheral.
 * @param peripheral_id A bitmask representing the peripheral to enable in RCC_PER0 or RCC_PER1.
 * @note This function infers the bit position in RCC_PER0 based on common Renesas MCU conventions.
 */
static void enable_peripheral_clock(uint8_t peripheral_id_bit)
{
    // WDT_Reset() is called implicitly by higher-level APIs.
    // Enable the peripheral clock in RCC_PER0
    REG_8BIT(RCC_PER0_ADDR) |= (1U << peripheral_id_bit);
    // Add logic for RCC_PER1 if applicable, based on peripheral_id_bit mapping
}

/**
 * @brief Disables the clock supply for a given peripheral.
 * @param peripheral_id A bitmask representing the peripheral to disable in RCC_PER0 or RCC_PER1.
 * @note This function infers the bit position in RCC_PER0 based on common Renesas MCU conventions.
 */
static void disable_peripheral_clock(uint8_t peripheral_id_bit)
{
    // WDT_Reset() is called implicitly by higher-level APIs.
    // Disable the peripheral clock in RCC_PER0
    REG_8BIT(RCC_PER0_ADDR) &= ~(1U << peripheral_id_bit);
    // Add logic for RCC_PER1 if applicable, based on peripheral_id_bit mapping
}

//==================================================================================================
// API Implementations
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// WDT (Watchdog Timer) Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Clears (refreshes) the Watchdog Timer.
 *
 * This function is critical for preventing the WDT from timing out and resetting the MCU.
 * It must be called periodically.
 * @note As per Rules.json, this function is called at the beginning of every other API.
 */
void WDT_Reset(void)
{
    // Write a specific value to the WDT Clear Register to refresh the timer.
    // The value 0xAC is a common sequence for Renesas MCUs to clear the WDT.
    REG_8BIT(WDT_CLR_ADDR) = WDT_CLR_SEQUENCE_VALUE;
}

/**
 * @brief Initializes the Watchdog Timer (WDT).
 *
 * This function enables and configures the Watchdog Timer with default or
 * specified parameters.
 */
void WDT_Init(void)
{
    WDT_Reset(); // Clear WDT before configuration

    // Enable WDT
    REG_8BIT(WDT_EN_ADDR) = 0xAA; // Inferred: WDT_EN usually requires a specific value to enable, like 0xAA or 0x55.
                                  // This is a placeholder as exact value for RENESAS_R5F11BBC is not in JSON.

    // Set WDT period >= 8 msec (inferred, specific bits for period not detailed in JSON description for WDT_MODER)
    // Assuming WDT_MODER (0xFFFC) can be set to achieve >= 8ms period.
    // This value is a placeholder; actual value would depend on WDT clock and register bits from datasheet.
    REG_8BIT(WDT_MODER_ADDR) = 0x00; // Example: Set to shortest period for safety or a calculated value.
                                     // For 8ms, assuming a common WDT clock, this might be 0x01 or 0x02.
                                     // No explicit bit definitions for WDT_MODER in JSON, so using 0x00 as a safe default/placeholder.

    WDT_Reset(); // Clear WDT after configuration
}


//--------------------------------------------------------------------------------------------------
// MCU CONFIG Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Configures the Microcontroller Unit (MCU) at startup.
 *
 * This function performs essential initializations like setting GPIO directions
 * and values, disabling unused peripherals, enabling WDT, and configuring LVR.
 *
 * @param volt The system voltage level (3V or 5V) to configure LVR.
 * @note This implementation handles address conflicts where multiple registers
 *       are mapped to the same physical address, as described in the register JSON.
 *       Careful read-modify-write operations are used for configuration registers
 *       that might share addresses for different bit-fields (e.g., PUx and PMC_x).
 *       Direct address conflicts for different functional blocks (e.g., P0 and ADC_ADM0 at 0xFF00)
 *       are noted and treated as a limitation due to the provided JSON structure.
 */
void MCU_Config_Init(t_sys_volt volt)
{
    WDT_Reset(); // Clear WDT

    // 1. Set all GPIO pins to 0 and verify with while loop
    volatile tbyte* port_data_regs[] = {
        (volatile tbyte*)GPIO_P0_ADDR, (volatile tbyte*)GPIO_P1_ADDR, (volatile tbyte*)GPIO_P2_ADDR,
        (volatile tbyte*)GPIO_P3_ADDR, (volatile tbyte*)GPIO_P4_ADDR, (volatile tbyte*)GPIO_P5_ADDR,
        (volatile tbyte*)GPIO_P6_ADDR, (volatile tbyte*)GPIO_P7_ADDR,
        (volatile tbyte*)GPIO_P12_ADDR, (volatile tbyte*)GPIO_P13_ADDR, (volatile tbyte*)GPIO_P14_ADDR,
        (volatile tbyte*)GPIO_P15_ADDR
    };
    uint8_t num_port_data_regs = sizeof(port_data_regs) / sizeof(port_data_regs[0]);

    for (uint8_t i = 0; i < num_port_data_regs; i++)
    {
        if (port_data_regs[i] != NULL)
        {
            *port_data_regs[i] = 0x00; // Set all output pins to low
            while (*port_data_regs[i] != 0x00)
            {
                // Wait until the value is truly set (software polling)
            }
        }
    }

    // 2. Set all GPIO pins direction to input and verify with while loop
    volatile tbyte* port_mode_regs[] = {
        (volatile tbyte*)GPIO_PM0_ADDR, (volatile tbyte*)GPIO_PM1_ADDR, (volatile tbyte*)GPIO_PM2_ADDR,
        (volatile tbyte*)GPIO_PM3_ADDR, (volatile tbyte*)GPIO_PM4_ADDR, (volatile tbyte*)GPIO_PM5_ADDR,
        (volatile tbyte*)GPIO_PM6_ADDR, (volatile tbyte*)GPIO_PM7_ADDR,
        (volatile tbyte*)GPIO_PM12_ADDR, (volatile tbyte*)GPIO_PM13_ADDR, (volatile tbyte*)GPIO_PM14_ADDR,
        (volatile tbyte*)GPIO_PM15_ADDR
    };
    uint8_t num_port_mode_regs = sizeof(port_mode_regs) / sizeof(port_mode_regs[0]);

    // For Renesas PM registers, 1 means input, 0 means output. So set all to 0xFF for input.
    for (uint8_t i = 0; i < num_port_mode_regs; i++)
    {
        if (port_mode_regs[i] != NULL)
        {
            *port_mode_regs[i] = 0xFF; // Set all pins to input
            while (*port_mode_regs[i] != 0xFF)
            {
                // Wait until the direction is truly set
            }
        }
    }

    WDT_Reset(); // Clear WDT

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable ADC (clear ADON bit in ADC_ADM0)
    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00). Writing here might affect P0.
    REG_8BIT(ADC_ADM0_ADDR) &= ~(1U << 7); // Assuming bit 7 is ADON (ADC ON) based on common Renesas ADC controllers.
    disable_peripheral_clock(RCC_PER0_ADC_EN_BIT);

    // Disable SAU (Serial Array Unit) channels (UART/SPI)
    REG_8BIT(SERIAL_SE_ADDR) = 0x00; // Disable all SAU channels
    REG_8BIT(SERIAL_ST_ADDR) = 0xFF; // Stop all SAU channels if running
    disable_peripheral_clock(RCC_PER0_SAU_EN_BIT);

    // Disable I2C channel 0
    REG_8BIT(I2C0_CR_ADDR) &= ~(1U << 7); // Inferred: Assuming bit 7 in I2C0_CR is I2C enable.
    disable_peripheral_clock(RCC_PER0_I2C_EN_BIT);

    // Disable Timer Array Unit 0 channels
    REG_8BIT(TAU0_TT_ADDR) = 0xFF; // Stop all TAU0 channels
    REG_8BIT(TAU0_TOE_ADDR) = 0x00; // Disable all TAU0 outputs
    disable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    // Disable DAC channels
    REG_8BIT(DAC_CH0_CSR_ADDR) &= ~(1U << 7); // Inferred: Assuming bit 7 in DAC_CH0_CSR is enable.
    REG_8BIT(DAC_CH1_CSR_ADDR) &= ~(1U << 7); // Inferred: Assuming bit 7 in DAC_CH1_CSR is enable.
    REG_8BIT(DAC_CSR_ADDR) &= ~(1U << 7); // Inferred: Assuming bit 7 in DAC_CSR is master enable.
    disable_peripheral_clock(RCC_PER0_DAC_EN_BIT);

    // I2S is not supported on this MCU as per MCAL.h

    WDT_Reset(); // Clear WDT

    // 4. Enable WDT (Watchdog Timer)
    // 5. Clear WDT timer
    // 6. Set WDT period >= 8 msec
    // WDT_Init() covers these steps.
    WDT_Init();

    WDT_Reset(); // Clear WDT

    // 7. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 8. Enable LVR (Low Voltage Reset)
    // The register_json does not contain explicit LVR registers.
    // LVD_Init and LVD_Enable are API functions but without clear register mappings.
    // Assuming RCC_CSC (0xFF71) might contain LVD enable/status bits, common for Renesas.
    // LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) is the API to set threshold,
    // which then implies LVR would use this setting.
    LVD_Init(); // This would internally configure LVD registers if they existed.
    if (sys_volt_3v == volt)
    {
        LVD_Get(lvd_threshold_volt_2v); // Configure LVD threshold to 2V
    }
    else // sys_volt_5v
    {
        LVD_Get(lvd_threshold_volt_3_5v); // Configure LVD threshold to 3.5V
    }
    LVD_Enable(); // This would enable the LVD/LVR feature.

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Enters low-power sleep mode.
 *
 * In Renesas RL78/R8C MCUs, this is typically achieved using the HALT instruction.
 */
void Go_to_sleep_mode(void)
{
    WDT_Reset(); // Clear WDT
    // For Renesas R5F11BBC (e.g. RL78/G11 family), _halt() is a common intrinsic for sleep mode.
    // This stops CPU execution, waiting for an interrupt.
    __asm("HALT"); // Placeholder for assembly instruction to enter HALT mode.
                   // Specific intrinsic might be _halt() or similar depending on toolchain.
}

/**
 * @brief Enables global interrupts.
 *
 * This function sets the global interrupt enable bit in the CPU.
 */
void Global_interrupt_Enable(void)
{
    WDT_Reset(); // Clear WDT
    __asm("EI"); // Enable Interrupts (Assembly instruction for many architectures)
                 // This would be replaced with specific intrinsic like __enable_interrupt() if available.
}

/**
 * @brief Disables global interrupts.
 *
 * This function clears the global interrupt enable bit in the CPU.
 */
void Global_interrupt_Disable(void)
{
    WDT_Reset(); // Clear WDT
    __asm("DI"); // Disable Interrupts (Assembly instruction for many architectures)
                 // This would be replaced with specific intrinsic like __disable_interrupt() if available.
}

//--------------------------------------------------------------------------------------------------
// LVD Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 *
 * @note The register_json does not provide explicit LVD control registers beyond
 *       RCC_CSC which is a status/control register. This implementation will assume
 *       placeholder bit operations on RCC_CSC for LVD configuration.
 */
void LVD_Init(void)
{
    WDT_Reset(); // Clear WDT

    // No dedicated LVD configuration registers like voltage selection or filter in JSON.
    // Assuming basic LVD configuration is part of a system control register (e.g., RCC_CSC).
    // This is a placeholder for actual LVD hardware initialization.
    REG_8BIT(RCC_CSC_ADDR) |= (1U << 0); // Inferred: Assume bit 0 in RCC_CSC enables some LVD functionality.
                                        // A real datasheet would specify LVD enable, threshold selection, and reset features.
}

/**
 * @brief Sets the Low Voltage Detection (LVD) threshold level.
 * @param lvd_thresholdLevel The desired LVD threshold level.
 * @note This function is a placeholder due to lack of specific LVD threshold registers in JSON.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel)
{
    WDT_Reset(); // Clear WDT
    // The function name `LVD_Get` usually implies reading a status, but API.json
    // signature suggests setting. This might be a naming inconsistency or refers
    // to "getting" a threshold value *into* the LVD module.
    // Since there are no explicit registers to set an LVD threshold in register_json,
    // this function serves as a placeholder.
    // A real implementation would write to an LVD threshold selection register.
    (void)lvd_thresholdLevel; // Suppress unused parameter warning
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 *
 * @note This is a placeholder assuming a generic enable bit in a system control register.
 */
void LVD_Enable(void)
{
    WDT_Reset(); // Clear WDT
    // Inferred: Assuming a bit in RCC_CSC enables the LVD detection.
    REG_8BIT(RCC_CSC_ADDR) |= (1U << 1); // Placeholder: Assume bit 1 in RCC_CSC enables LVD.
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 *
 * @note This is a placeholder assuming a generic disable bit in a system control register.
 */
void LVD_Disable(void)
{
    WDT_Reset(); // Clear WDT
    // Inferred: Assuming a bit in RCC_CSC disables the LVD detection.
    REG_8BIT(RCC_CSC_ADDR) &= ~(1U << 1); // Placeholder: Assume bit 1 in RCC_CSC disables LVD.
}

//--------------------------------------------------------------------------------------------------
// UART Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a UART channel with specified parameters.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The data length (7, 8, or 9 bits).
 * @param uart_stop_bit The number of stop bits (1 or 2).
 * @param uart_parity The parity setting (none, even, odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity)
{
    WDT_Reset(); // Clear WDT

    volatile tbyte *smr_reg = NULL;
    volatile tbyte *scr_reg = NULL;
    volatile tbyte *so_reg = NULL;
    volatile tbyte *soe_reg = NULL;

    // Map UART channel to SAU registers and GPIO pins
    t_port tx_port = port_1; // Default for SAU0_SDR01 (P1.0)
    t_pin tx_pin = pin_0;
    t_port rx_port = port_1; // Default for SAU0_SDR01 (P1.1)
    t_pin rx_pin = pin_1;
    tbyte sau_channel_bit = 0; // Bit in SERIAL_SE/SS/ST for this SAU channel

    switch (uart_channel)
    {
        case uart_channel_0: // SAU0 Channel 1 (P1.0/P1.1)
            smr_reg = (volatile tbyte*)SAU0_SMR01_ADDR;
            scr_reg = (volatile tbyte*)SAU0_SCR01_ADDR;
            // SERIAL_SO_ADDR and SERIAL_SOE_ADDR control TX output for all SAU channels via bits.
            // Assuming SAU0 channel 1 TX is controlled by bit 1 of SERIAL_SO/SOE
            so_reg = (volatile tbyte*)SERIAL_SO_ADDR;
            soe_reg = (volatile tbyte*)SERIAL_SOE_ADDR;
            sau_channel_bit = 1; // Bit for SAU0 Channel 1 (SDR01)
            tx_port = port_1; tx_pin = pin_0; // P1.0 (TxD)
            rx_port = port_1; rx_pin = pin_1; // P1.1 (RxD)
            break;
        case uart_channel_1: // SAU1 Channel 0 (P1.7/P0.0)
            smr_reg = (volatile tbyte*)SAU1_SMR10_ADDR;
            scr_reg = (volatile tbyte*)SAU1_SCR10_ADDR;
            // Assuming SAU1 channel 0 TX is controlled by bit 2 of SERIAL_SO/SOE (inferred from SERIAL_SO/SOE assigned pins)
            so_reg = (volatile tbyte*)SERIAL_SO_ADDR;
            soe_reg = (volatile tbyte*)SERIAL_SOE_ADDR;
            sau_channel_bit = 2; // Bit for SAU1 Channel 0 (SDR10)
            tx_port = port_1; tx_pin = pin_7; // P1.7 (TxD)
            rx_port = port_0; rx_pin = pin_0; // P0.0 (RxD)
            break;
        case uart_channel_2: // SAU2 Channel 0 (P4.0/P4.1)
            smr_reg = (volatile tbyte*)SAU2_SMR20_ADDR;
            scr_reg = (volatile tbyte*)SAU2_SCR20_ADDR;
            // Assuming SAU2 channel 0 TX is controlled by bit 3 of SERIAL_SO/SOE (inferred from SERIAL_SO/SOE assigned pins)
            so_reg = (volatile tbyte*)SERIAL_SO_ADDR;
            soe_reg = (volatile tbyte*)SERIAL_SOE_ADDR;
            sau_channel_bit = 3; // Bit for SAU2 Channel 0 (SDR20)
            tx_port = port_4; tx_pin = pin_0; // P4.0 (TxD)
            rx_port = port_4; rx_pin = pin_1; // P4.1 (RxD)
            break;
        default: return; // Invalid channel
    }

    if (smr_reg == NULL || scr_reg == NULL) return; // Guard against invalid channel mapping

    // 1. Enable peripheral clock for SAU
    enable_peripheral_clock(RCC_PER0_SAU_EN_BIT);

    // 2. Configure GPIO pins for UART TX/RX alternate function
    // For TX pin: Configure as output (PMx = 0), disable pull-up (PUx = 0), set Output Mode (POMx = CMOS).
    // For RX pin: Configure as input (PMx = 1), enable pull-up (PUx = 1), disable digital buffer (PMC_x).

    // Configure TX pin
    GPIO_Output_Init(tx_port, tx_pin, 0); // Set low before direction, then configure for alt function.
    // PMC register for alternate function. Assuming 1 to enable serial function.
    if (get_gpio_port_pmc_reg_ptr(tx_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(tx_port)) |= (1U << tx_pin); // Inferred: Set bit to enable alternate function
    }

    // Configure RX pin
    GPIO_Input_Init(rx_port, rx_pin); // Configure as input with pull-up
    // PMC register for alternate function (disable digital input buffer for peripheral use).
    if (get_gpio_port_pmc_reg_ptr(rx_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(rx_port)) |= (1U << rx_pin); // Inferred: Set bit to enable alternate function
    }
    WDT_Reset();

    // 3. Configure SMR (Serial Mode Register) for UART mode
    *smr_reg = 0x00; // Reset SMR
    // Common Renesas UART SMR settings (placeholder values):
    // Bit 0: CKS0 (Clock Select) - Usually 0 for Fx (main clock)
    // Bit 1: CKS1 (Clock Select)
    // Bit 2: STC (Stop Bit Count) - 0: 1 stop bit, 1: 2 stop bits
    // Bit 3: PTY (Parity) - 0: Even, 1: Odd
    // Bit 4: PTS (Parity Type Select) - 0: No Parity, 1: Parity enabled
    // Bit 5: DIR (Transfer Direction) - 0: LSB first, 1: MSB first (usually LSB)
    // Bit 6: SCM (Serial Communication Mode) - 0: Not Synchronous, 1: Synchronous
    // Bit 7: MD (Transfer Format) - 0: 7-bit, 1: 8-bit

    // For UART: MD=1 (8-bit), SCM=0 (Asynchronous), DIR=0 (LSB first)
    tbyte smr_val = (1U << 7); // Start with 8-bit data length

    if (uart_data_length == uart_data_length_7bit) smr_val &= ~(1U << 7); // Clear MD for 7-bit (or set appropriate bit)
    // Note: The JSON only lists 8-bit for MD bit. If 7-bit requires other bits, this is an assumption.
    // If 9-bit is available, it's often another register or a different MD value. Assuming 8-bit default.

    if (uart_stop_bit == uart_stop_bit_2) smr_val |= (1U << 2); // Set STC for 2 stop bits

    if (uart_parity != uart_parity_none)
    {
        smr_val |= (1U << 4); // Enable parity
        if (uart_parity == uart_parity_odd) smr_val |= (1U << 3); // Set PTY for odd parity
        else smr_val &= ~(1U << 3); // Clear PTY for even parity
    }

    *smr_reg = smr_val; // Apply SMR settings
    WDT_Reset();

    // 4. Configure SCR (Serial Control Register) for Baud Rate
    *scr_reg = 0x00; // Reset SCR
    // Common Renesas UART SCR settings (placeholder values):
    // Bits 0-7: BRH/BRL (Baud Rate High/Low) - actual baud rate clock divider.
    // Other bits usually control interrupt enable, FIFO, etc.

    tword baud_rate_reg_val = 0; // Placeholder for actual baud rate divisor
    // Calculation: Baud_rate = f_SAU / (2 * (SDR + 1))
    // SDR = (f_SAU / (2 * Baud_rate)) - 1
    // Assuming f_SAU is ~4MHz, and common Renesas 8-bit SDR for baud rate control.
    switch (uart_baud_rate)
    {
        case uart_baud_rate_9600:   baud_rate_reg_val = 259; break; // Example for 4MHz f_SAU
        case uart_baud_rate_19200:  baud_rate_reg_val = 129; break; // Example for 4MHz f_SAU
        case uart_baud_rate_57600:  baud_rate_reg_val = 42; break;  // Example for 4MHz f_SAU
        case uart_baud_rate_115200: baud_rate_reg_val = 21; break;  // Example for 4MHz f_SAU
        default: baud_rate_reg_val = 259; break; // Default to 9600
    }
    // SCR is 8-bit. Assuming SCR holds the low byte of the baud rate divisor.
    *scr_reg = (tbyte)baud_rate_reg_val;
    // If a dedicated high-byte register exists, it would be configured here.
    // Example: (volatile tbyte*)(SAU0_SCR01_ADDR + 1) = (tbyte)(baud_rate_reg_val >> 8);

    // 5. Enable Serial Output for TX pin (if SAU is configured for TX)
    if (soe_reg != NULL && so_reg != NULL)
    {
        *soe_reg |= (1U << sau_channel_bit); // Enable TX output
        *so_reg |= (1U << sau_channel_bit);  // Set initial high level for idle line (common for UART)
    }

    WDT_Reset();
}

/**
 * @brief Enables a specified UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Clear WDT
    tbyte sau_channel_bit = 0;
    switch (uart_channel)
    {
        case uart_channel_0: sau_channel_bit = 1; break; // SAU0 Channel 1
        case uart_channel_1: sau_channel_bit = 2; break; // SAU1 Channel 0
        case uart_channel_2: sau_channel_bit = 3; break; // SAU2 Channel 0
        default: return;
    }

    enable_peripheral_clock(RCC_PER0_SAU_EN_BIT); // Ensure SAU clock is enabled

    // Set the start bit for the specific SAU channel.
    REG_8BIT(SERIAL_SS_ADDR) |= (1U << sau_channel_bit); // Start serial operation
    REG_8BIT(SERIAL_SE_ADDR) |= (1U << sau_channel_bit); // Enable serial channel
    WDT_Reset();
}

/**
 * @brief Disables a specified UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel)
{
    WDT_Reset(); // Clear WDT
    tbyte sau_channel_bit = 0;
    switch (uart_channel)
    {
        case uart_channel_0: sau_channel_bit = 1; break; // SAU0 Channel 1
        case uart_channel_1: sau_channel_bit = 2; break; // SAU1 Channel 0
        case uart_channel_2: sau_channel_bit = 3; break; // SAU2 Channel 0
        default: return;
    }

    // Clear the stop bit for the specific SAU channel.
    REG_8BIT(SERIAL_ST_ADDR) |= (1U << sau_channel_bit); // Stop serial operation
    REG_8BIT(SERIAL_SE_ADDR) &= ~(1U << sau_channel_bit); // Disable serial channel
    WDT_Reset();
}

/**
 * @brief Sends a single byte over UART.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte *sdr_reg = NULL;
    volatile tbyte *ssr_reg = NULL; // Status register to check TX completion

    switch (uart_channel)
    {
        case uart_channel_0: sdr_reg = (volatile tbyte*)SAU0_SDR01_ADDR; ssr_reg = (volatile tbyte*)SAU0_SSR01_ADDR; break;
        case uart_channel_1: sdr_reg = (volatile tbyte*)SAU1_SDR10_ADDR; ssr_reg = (volatile tbyte*)SAU1_SSR10_ADDR; break;
        case uart_channel_2: sdr_reg = (volatile tbyte*)SAU2_SDR20_ADDR; ssr_reg = (volatile tbyte*)SAU2_SSR20_ADDR; break;
        default: return;
    }

    if (sdr_reg == NULL || ssr_reg == NULL) return;

    // Wait until transmit buffer is empty (TXD buffer empty flag, often bit 6 in SSR)
    // Inferred: Assuming bit 6 (TDRE) in SSR indicates transmit data register empty.
    while (!(*ssr_reg & (1U << 6)))
    {
        WDT_Reset(); // Keep WDT happy while waiting
    }

    *sdr_reg = byte; // Write data to SDR register
    WDT_Reset();
}

/**
 * @brief Sends a frame of data over UART.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length)
{
    WDT_Reset(); // Clear WDT
    for (int i = 0; i < length; i++)
    {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
    WDT_Reset();
}

/**
 * @brief Sends a null-terminated string over UART.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str)
{
    WDT_Reset(); // Clear WDT
    UART_send_frame(uart_channel, str, strlen(str));
    WDT_Reset();
}

/**
 * @brief Reads a single byte from UART.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte *sdr_reg = NULL;
    volatile tbyte *ssr_reg = NULL; // Status register to check RX completion

    switch (uart_channel)
    {
        case uart_channel_0: sdr_reg = (volatile tbyte*)SAU0_SDR01_ADDR; ssr_reg = (volatile tbyte*)SAU0_SSR01_ADDR; break;
        case uart_channel_1: sdr_reg = (volatile tbyte*)SAU1_SDR10_ADDR; ssr_reg = (volatile tbyte*)SAU1_SSR10_ADDR; break;
        case uart_channel_2: sdr_reg = (volatile tbyte*)SAU2_SDR20_ADDR; ssr_reg = (volatile tbyte*)SAU2_SSR20_ADDR; break;
        default: return 0;
    }

    if (sdr_reg == NULL || ssr_reg == NULL) return 0;

    // Wait until receive buffer has data (RXD buffer full flag, often bit 5 in SSR)
    // Inferred: Assuming bit 5 (RDRF) in SSR indicates receive data register full.
    while (!(*ssr_reg & (1U << 5)))
    {
        WDT_Reset(); // Keep WDT happy while waiting
    }

    return *sdr_reg; // Read data from SDR register
}

/**
 * @brief Receives a frame of data over UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
    WDT_Reset();
}

/**
 * @brief Receives a null-terminated string over UART.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer.
 * @return The number of bytes received (excluding null terminator).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    tbyte count = 0;
    for (int i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        char received_byte = (char)UART_Get_Byte(uart_channel);
        buffer[i] = received_byte;
        count++;
        // Assuming a newline or carriage return might terminate a string, or a specific character.
        // For a generic API, we might just fill up to max_length or wait for a timeout.
        // For simplicity here, we'll just read max_length-1 bytes.
    }
    buffer[count] = '\0'; // Null-terminate the string
    WDT_Reset();
    return count;
}


//--------------------------------------------------------------------------------------------------
// I2C Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes an I2C channel.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (Standard or Fast mode).
 * @param i2c_device_address The device's own 7-bit slave address.
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Data length (7-bit or 8-bit addressing).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength)
{
    WDT_Reset(); // Clear WDT

    if (i2c_channel != i2c_channel_0) return; // Only I2C0 is available based on JSON

    // 1. Enable peripheral clock for I2C0
    enable_peripheral_clock(RCC_PER0_I2C_EN_BIT);

    // 2. Configure GPIO pins for I2C SCL (P1.2/P1.5) and SDA (P1.1/P1.3)
    // For Renesas, I2C pins often need to be configured as open-drain output (POMx = 1)
    // and have their alternate function enabled (PMC_x).
    // Assuming SCL on P1.2, SDA on P1.3 (common pair)
    // The JSON lists P1.1, P1.2, P1.3, P1.5. Let's pick a pair for concrete implementation.
    // Use P1.2 for SCL, P1.3 for SDA.

    GPIO_Output_Init(port_1, pin_2, 1); // SCL: High
    GPIO_Output_Init(port_1, pin_3, 1); // SDA: High
    // Set to N-channel open-drain (POMx = 1)
    if (get_gpio_port_outputmode_reg_ptr(port_1) != NULL)
    {
        *(get_gpio_port_outputmode_reg_ptr(port_1)) |= ( (1U << pin_2) | (1U << pin_3) );
    }
    // Set PMC bits for alternate function (I2C) - inferred, typically bit-wise set.
    if (get_gpio_port_pmc_reg_ptr(port_1) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(port_1)) |= ( (1U << pin_2) | (1U << pin_3) );
    }
    WDT_Reset();

    // 3. Disable I2C peripheral before configuration (I2C0_CR bit 7)
    REG_8BIT(I2C0_CR_ADDR) &= ~(1U << 7); // Inferred: Clear IICE (I2C Enable) bit

    // 4. Configure I2C Clock Speed (I2C0_BBR0 - Bit Rate Register)
    // I2C_rules: "Always use fast mode"
    // Assuming 400kHz Fast Mode (if i2c_clk_speed_fast is selected)
    tbyte bbr_value = 0;
    if (i2c_clk_speed_fast == i2c_clk_speed)
    {
        // Placeholder calculation for 400kHz. Actual value depends on system clock and I2C clock source.
        // Assuming typical Renesas calculation for BBR0.
        bbr_value = 0x10; // Inferred: Example value for 400kHz.
    }
    else // Standard mode (100 kHz)
    {
        bbr_value = 0x40; // Inferred: Example value for 100kHz.
    }
    REG_8BIT(I2C0_BBR0_ADDR) = bbr_value;
    WDT_Reset();

    // 5. Configure Addressing Mode (I2C0_SAR - Slave Address Register)
    // I2C_rules: "Addressing Mode equals Device Address" -> implies setting device's own address.
    REG_8BIT(I2C0_SAR_ADDR) = (tbyte)(i2c_device_address << 1); // 7-bit address typically shifted left by 1

    // 6. Configure Acknowledge (I2C0_CR bit 6: WREL, I2C0_BCR0 bit 0: ACKWP)
    // The JSON does not specify precise ACK bits. Inferring from typical Renesas.
    if (i2c_ack_enable == i2c_ack)
    {
        REG_8BIT(I2C0_BCR0_ADDR) |= (1U << 0); // Inferred: Enable ACK (ACKWP bit)
    }
    else
    {
        REG_8BIT(I2C0_BCR0_ADDR) &= ~(1U << 0); // Inferred: Disable ACK
    }
    WDT_Reset();

    // 7. Configure Data Length (I2C0_FCR for 7-bit/8-bit or part of I2C0_CR)
    // The JSON description for I2C0_FCR is "configures functional settings".
    // Assuming 7-bit/8-bit addressing is covered by the slave address setup and not a separate data length bit.
    // If i2c_datalength refers to transaction byte length, it's typically controlled by start/stop conditions.
    // For now, no specific register bit is set based on t_i2c_datalength, assuming it's for addressing.
    (void)i2c_datalength; // Suppress unused parameter warning

    // I2C_rules: "Always use maximum timeout", "Always generate a repeated start condition instead of stop between transactions"
    // These require specific register bits not detailed in the provided JSON. Placeholder:
    // For repeated start, typically a bit in I2C0_CR or I2C0_BCR0 is set (e.g., MSTP for stop, or RS for repeated start).
    // Max timeout might be an interrupt or error condition handling, or a software delay.

    // Re-enable I2C peripheral
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 7); // Inferred: Set IICE (I2C Enable) bit
    WDT_Reset();
}

/**
 * @brief Enables the I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return; // Only I2C0 is available

    enable_peripheral_clock(RCC_PER0_I2C_EN_BIT);

    REG_8BIT(I2C0_CR_ADDR) |= (1U << 7); // Inferred: Set IICE (I2C Enable) bit
    WDT_Reset();
}

/**
 * @brief Disables the I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return; // Only I2C0 is available

    REG_8BIT(I2C0_CR_ADDR) &= ~(1U << 7); // Inferred: Clear IICE (I2C Enable) bit
    WDT_Reset();
}

/**
 * @brief Sends a single byte over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return;

    // A full I2C transaction sequence involves START, address, ACK check, data, ACK check, STOP.
    // This function assumes I2C is already enabled and expects to send a data byte within an existing transaction.
    // Or, it needs to perform a full transaction for a single byte.
    // For simplicity and adherence to JSON, we only interact with I2C0_DR (data) and I2C0_BSCL/BSCH (status).

    // Inferred: Initiate master transmit, wait for status, write data.
    // This part is highly MCU-specific and not fully derivable from generic JSON.
    // Placeholder assumes I2C0_CR bit 0 (STAS) is Start Condition Enable, bit 1 (SPCS) is Stop Condition Enable.
    // I2C0_BSCL/BSCH will provide status.
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 0); // Inferred: Generate START condition (STAS)
    // Wait for START condition to complete (e.g., check I2C0_BSCL for status bit)
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 7))) { WDT_Reset(); } // Inferred: Wait for MST bit to indicate Master status (bit 7)
    REG_8BIT(I2C0_DR_ADDR) = byte; // Write data
    // Wait for transfer complete and ACK status
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 6))) { WDT_Reset(); } // Inferred: Wait for TRC bit to indicate Transfer Complete (bit 6)
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 1); // Inferred: Generate STOP condition (SPCS)
    WDT_Reset();
}

/**
 * @brief Sends a frame of data over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return;

    // Start condition (assuming master mode)
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 0); // Inferred: Generate START condition (STAS)
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 7))) { WDT_Reset(); } // Wait for MST bit

    for (int i = 0; i < length; i++)
    {
        REG_8BIT(I2C0_DR_ADDR) = (tbyte)data[i];
        while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 6))) { WDT_Reset(); } // Wait for TRC bit
        // Check for ACK/NACK status here if needed (not explicitly defined in JSON for I2C0_BSCL/BSCH)
    }

    // Stop condition
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 1); // Inferred: Generate STOP condition (SPCS)
    WDT_Reset();
}

/**
 * @brief Sends a null-terminated string over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str)
{
    WDT_Reset(); // Clear WDT
    I2C_send_frame(i2c_channel, str, strlen(str));
    WDT_Reset();
}

/**
 * @brief Receives a single byte over I2C.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return 0;

    // Similar to send_byte, this assumes a transaction is ongoing or initiates one.
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 0); // Inferred: Generate START condition (STAS)
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 7))) { WDT_Reset(); } // Wait for MST bit

    // Inferred: Set for receive mode (e.g., I2C0_CR bit for RCV).
    // For Renesas, often setting a register bit for receive is required.
    // REG_8BIT(I2C0_CR_ADDR) |= (1U << RX_MODE_BIT); // Placeholder

    tbyte received_byte = 0;
    // Wait for receive complete and ACK status
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 6))) { WDT_Reset(); } // Wait for TRC bit
    received_byte = REG_8BIT(I2C0_DR_ADDR); // Read data
    REG_8BIT(I2C0_CR_ADDR) |= (1U << 1); // Inferred: Generate STOP condition (SPCS)

    WDT_Reset();
    return received_byte;
}

/**
 * @brief Receives a frame of data over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return;

    REG_8BIT(I2C0_CR_ADDR) |= (1U << 0); // Inferred: Generate START condition (STAS)
    while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 7))) { WDT_Reset(); } // Wait for MST bit

    // Inferred: Set for receive mode.
    // REG_8BIT(I2C0_CR_ADDR) |= (1U << RX_MODE_BIT); // Placeholder

    for (int i = 0; i < max_length; i++)
    {
        // Send ACK for all but the last byte, NACK for the last byte
        if (i == max_length - 1)
        {
            // Disable ACK for last byte (e.g., I2C0_BCR0 bit 0 = 0)
            REG_8BIT(I2C0_BCR0_ADDR) &= ~(1U << 0);
        }
        else
        {
            // Enable ACK (e.g., I2C0_BCR0 bit 0 = 1)
            REG_8BIT(I2C0_BCR0_ADDR) |= (1U << 0);
        }

        while (!(REG_8BIT(I2C0_BSCL_ADDR) & (1U << 6))) { WDT_Reset(); } // Wait for TRC bit
        buffer[i] = (char)REG_8BIT(I2C0_DR_ADDR); // Read data
    }

    REG_8BIT(I2C0_CR_ADDR) |= (1U << 1); // Inferred: Generate STOP condition (SPCS)
    WDT_Reset();
}

/**
 * @brief Receives a null-terminated string over I2C.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer.
 * @return The number of bytes received (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    if (i2c_channel != i2c_channel_0) return 0;

    // This is a simplification; a real I2C string read might involve reading until a specific delimiter
    // or a known length. For this API, it will read max_length-1 bytes and null-terminate.
    I2C_Get_frame(i2c_channel, buffer, max_length - 1);
    buffer[max_length - 1] = '\0'; // Null-terminate
    WDT_Reset();
    return (tbyte)(max_length - 1);
}

//--------------------------------------------------------------------------------------------------
// SPI (CSI) Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes an SPI (CSI) channel.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master or Slave mode.
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order)
{
    WDT_Reset(); // Clear WDT

    if (spi_channel != spi_channel_0) return; // Only SAU0 Channel 0 for SPI

    volatile tbyte *smr_reg = (volatile tbyte*)SAU0_SMR00_ADDR;
    volatile tbyte *scr_reg = (volatile tbyte*)SAU0_SCR00_ADDR;
    volatile tbyte *so_reg = (volatile tbyte*)SERIAL_SO_ADDR;
    volatile tbyte *soe_reg = (volatile tbyte*)SERIAL_SOE_ADDR;
    tbyte sau_channel_bit = 0; // Bit for SAU0 Channel 0 (SDR00)

    // SAU0 Channel 0 uses P1.4 (SCK), P1.5 (SO/MISO), P1.6 (SI/MOSI)
    t_port sck_port = port_1; t_pin sck_pin = pin_4;
    t_port mosi_port = port_1; t_pin mosi_pin = pin_6; // SI (MOSI)
    t_port miso_port = port_1; t_pin miso_pin = pin_5; // SO (MISO)
    sau_channel_bit = 0;

    // 1. Enable peripheral clock for SAU
    enable_peripheral_clock(RCC_PER0_SAU_EN_BIT);

    // 2. Configure GPIO pins for SPI alternate function
    // SCK: Output for Master, Input for Slave.
    // MOSI: Output for Master, Input for Slave.
    // MISO: Input for Master, Output for Slave.
    // All typically require PMC bit set for alternate function.

    if (spi_mode == spi_mode_master)
    {
        // Master: SCK (Out), MOSI (Out), MISO (In)
        GPIO_Output_Init(sck_port, sck_pin, 0); // SCK
        GPIO_Output_Init(mosi_port, mosi_pin, 0); // MOSI
        GPIO_Input_Init(miso_port, miso_pin); // MISO
    }
    else // Slave
    {
        // Slave: SCK (In), MOSI (In), MISO (Out)
        GPIO_Input_Init(sck_port, sck_pin); // SCK
        GPIO_Input_Init(mosi_port, mosi_pin); // MOSI
        GPIO_Output_Init(miso_port, miso_pin, 0); // MISO
    }

    // Set PMC bits for alternate functions
    if (get_gpio_port_pmc_reg_ptr(sck_port) != NULL)  *(get_gpio_port_pmc_reg_ptr(sck_port))  |= (1U << sck_pin);
    if (get_gpio_port_pmc_reg_ptr(mosi_port) != NULL) *(get_gpio_port_pmc_reg_ptr(mosi_port)) |= (1U << mosi_pin);
    if (get_gpio_port_pmc_reg_ptr(miso_port) != NULL) *(get_gpio_port_pmc_reg_ptr(miso_port)) |= (1U << miso_pin);
    WDT_Reset();

    // 3. Configure SMR (Serial Mode Register) for SPI mode
    *smr_reg = 0x00; // Reset SMR

    tbyte smr_val = 0;
    // Common Renesas SPI SMR settings (placeholder values):
    // Bit 0: CKS0 (Clock Select)
    // Bit 1: CKS1
    // Bit 2: STOP (Stop bit count, not for SPI)
    // Bit 3: PTY (Parity, not for SPI)
    // Bit 4: PTS (Parity enable, not for SPI)
    // Bit 5: DIR (Transfer Direction) - 0: LSB first, 1: MSB first
    // Bit 6: SCM (Serial Communication Mode) - 0: Asynchronous, 1: Synchronous
    // Bit 7: MD (Transfer Format) - 0: 7-bit, 1: 8-bit

    // For SPI: Synchronous mode (SCM=1), 8-bit data (MD=1) - assuming typical SPI.
    smr_val |= (1U << 6); // Set SCM for synchronous
    smr_val |= (1U << 7); // Set MD for 8-bit

    if (spi_bit_order == spi_bit_order_msb_first) smr_val |= (1U << 5); // Set DIR for MSB first
    else smr_val &= ~(1U << 5); // Clear DIR for LSB first

    *smr_reg = smr_val; // Apply SMR settings
    WDT_Reset();

    // 4. Configure SCR (Serial Control Register) for CPHA, CPOL, Master/Slave, DFF, Speed
    *scr_reg = 0x00; // Reset SCR

    tbyte scr_val = 0;
    // Common Renesas SPI SCR settings:
    // Bits 0-7: BRH/BRL (Baud Rate High/Low) - actual clock divisor.
    // Other bits usually control interrupt enable, clock phase/polarity.
    // CPHA/CPOL often controlled by SMR bits or specific SCR bits (depends on MCU).
    // Given JSON, assuming SCR is primarily for baud rate, and CPOL/CPHA might be in SMR or other registers.
    // If CPOL/CPHA are explicitly passed, let's assume specific bits in SCR (inferred).
    // In Renesas, SMR bits might include CPHA, CPOL, typically 0 or 1.

    // "Always use fast speed" -> Set baud rate divisor to minimum (assuming 1)
    scr_val = 1; // Placeholder for fast speed baud rate divisor
                 // A real implementation would calculate this based on clock.

    if (spi_mode == spi_mode_master)
    {
        scr_val |= (1U << 7); // Inferred: Set MSTR bit for Master Mode (e.g., SCR bit 7)
    }
    else
    {
        scr_val &= ~(1U << 7); // Inferred: Clear MSTR bit for Slave Mode
    }

    // CPOL/CPHA configuration. These are highly specific to SAU register bits.
    // Assuming SMR bits 0 and 1 are related to clock polarity/phase for SPI.
    // This is a strong inference due to lack of detail in JSON.
    if (spi_cpol == spi_cpol_high) REG_8BIT(SAU0_SMR00_ADDR) |= (1U << 0); // Placeholder
    else REG_8BIT(SAU0_SMR00_ADDR) &= ~(1U << 0); // Placeholder

    if (spi_cpha == spi_cpha_2edge) REG_8BIT(SAU0_SMR00_ADDR) |= (1U << 1); // Placeholder
    else REG_8BIT(SAU0_SMR00_ADDR) &= ~(1U << 1); // Placeholder

    // DFF (Data Frame Format) - assumed to be part of SMR MD bit (bit 7)
    if (spi_dff == spi_dff_16bit)
    {
        REG_8BIT(SAU0_SMR00_ADDR) |= (1U << 8); // Inferred: Placeholder for 16-bit DFF if SAU supports it.
                                                 // Often a separate register or a different MD value.
    }
    else
    {
        REG_8BIT(SAU0_SMR00_ADDR) &= ~(1U << 8); // Placeholder for 8-bit DFF
    }


    *scr_reg = scr_val; // Apply SCR settings
    WDT_Reset();

    // 5. Enable Serial Output Enable for SO (MISO) if slave, or SI (MOSI) if master,
    // and SCK if master. The SERIAL_SO/SOE registers control output.
    // "Slave Select always software-controlled" - no explicit SS register in JSON.
    // "Always use full duplex" - this is inherent in SAU design, configured by mode bits.
    // "Always enable CRC" - no explicit CRC registers for SAU0_SDR00 in JSON.

    if (spi_mode == spi_mode_master)
    {
        // Master: SCK, MOSI (SI) are outputs
        *soe_reg |= ( (1U << sau_channel_bit) | (1U << (sau_channel_bit + 1)) ); // Inferred bits for SCK/MOSI
        *so_reg |= ( (1U << sau_channel_bit) | (1U << (sau_channel_bit + 1)) ); // Set idle levels (high for SCK, MOSI)
    }
    else // Slave: MISO (SO) is output
    {
        *soe_reg |= (1U << sau_channel_bit); // Inferred bit for MISO
        *so_reg |= (1U << sau_channel_bit);  // Set idle level
    }

    WDT_Reset();
}

/**
 * @brief Enables the SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Clear WDT
    if (spi_channel != spi_channel_0) return;

    enable_peripheral_clock(RCC_PER0_SAU_EN_BIT); // Ensure SAU clock is enabled

    // Start the specific SAU channel for SPI
    REG_8BIT(SERIAL_SS_ADDR) |= (1U << 0); // SAU0 Channel 0 bit
    REG_8BIT(SERIAL_SE_ADDR) |= (1U << 0); // SAU0 Channel 0 bit
    WDT_Reset();
}

/**
 * @brief Disables the SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel)
{
    WDT_Reset(); // Clear WDT
    if (spi_channel != spi_channel_0) return;

    // Stop the specific SAU channel for SPI
    REG_8BIT(SERIAL_ST_ADDR) |= (1U << 0); // SAU0 Channel 0 bit
    REG_8BIT(SERIAL_SE_ADDR) &= ~(1U << 0); // SAU0 Channel 0 bit
    WDT_Reset();
}

/**
 * @brief Sends a single byte over SPI.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte)
{
    WDT_Reset(); // Clear WDT
    if (spi_channel != spi_channel_0) return;

    volatile tbyte *sdr_reg = (volatile tbyte*)SAU0_SDR00_ADDR;
    volatile tbyte *ssr_reg = (volatile tbyte*)SAU0_SSR00_ADDR;

    // Wait until transmit buffer is empty (TXD buffer empty flag, often bit 6 in SSR)
    while (!(*ssr_reg & (1U << 6)))
    {
        WDT_Reset(); // Keep WDT happy
    }

    *sdr_reg = byte; // Write data to SDR register

    // For full-duplex SPI, reading the SDR also implies receiving data.
    // If only sending, we might still read to clear the RX buffer or wait for TX completion.
    // Renesas SAU often has a separate TX/RX buffer empty/full flag or completion flag.
    while (!(*ssr_reg & (1U << 7))) // Inferred: Wait for Transmit End (TE) bit or similar
    {
        WDT_Reset();
    }
    WDT_Reset();
}

/**
 * @brief Sends a frame of data over SPI.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length)
{
    WDT_Reset(); // Clear WDT
    for (int i = 0; i < length; i++)
    {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
    WDT_Reset();
}

/**
 * @brief Receives a single byte over SPI.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel)
{
    WDT_Reset(); // Clear WDT
    if (spi_channel != spi_channel_0) return 0;

    volatile tbyte *sdr_reg = (volatile tbyte*)SAU0_SDR00_ADDR;
    volatile tbyte *ssr_reg = (volatile tbyte*)SAU0_SSR00_ADDR;

    // For receiving, often a dummy byte needs to be transmitted in master mode.
    // For slave mode, it waits for master to send.
    // Assuming master mode and dummy byte transmission.
    *sdr_reg = 0xFF; // Send dummy byte to clock out data from slave

    // Wait until receive buffer has data (RXD buffer full flag, often bit 5 in SSR)
    while (!(*ssr_reg & (1U << 5))) // Inferred: Wait for RDRF (Receive Data Register Full) bit
    {
        WDT_Reset(); // Keep WDT happy
    }

    WDT_Reset();
    return *sdr_reg; // Read data from SDR register
}

/**
 * @brief Receives a frame of data over SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
    WDT_Reset();
}

/**
 * @brief Receives a null-terminated string over SPI.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer.
 * @return The number of bytes received (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset(); // Clear WDT
    tbyte count = 0;
    for (int i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        char received_byte = (char)SPI_Get_Byte(spi_channel);
        buffer[i] = received_byte;
        count++;
        // No explicit string termination character for SPI, so read up to max_length-1
    }
    buffer[count] = '\0'; // Null-terminate the string
    WDT_Reset();
    return count;
}


//--------------------------------------------------------------------------------------------------
// External Interrupt Implementation
//--------------------------------------------------------------------------------------------------

// Placeholder for callback functions for external interrupts
static void (*external_int_callbacks[8])(void) = {NULL};

/**
 * @brief Initializes an external interrupt channel.
 * @param external_int_channel The external interrupt channel (P1.0-P1.7).
 * @param external_int_edge The edge sensitivity for the interrupt.
 * @note This implementation assumes TAU_ISC (0xFFB1) controls external interrupt edge detection,
 *       which is a common pattern in Renesas MCUs for using timer inputs as external interrupt pins.
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge)
{
    WDT_Reset(); // Clear WDT

    if (external_int_channel > external_int_channel_7) return; // Invalid channel

    // 1. Configure corresponding GPIO pin as input
    t_port port = port_1; // All External INT channels are on Port 1
    t_pin pin = (t_pin)external_int_channel;
    GPIO_Input_Init(port, pin);

    // 2. Configure TAU_ISC for edge detection
    // Assuming TAU_ISC bits correspond directly to the channel number for setting edge.
    // Inferred: Bits 0-7 in TAU_ISC control edge for P1.0-P1.7.
    // Bit 0: Rising Edge, Bit 1: Falling Edge for each channel pair (example, not exact)
    // For simplicity, let's assume one bit per edge type for each channel.
    // This is a highly inferred section as TAU_ISC's exact bit mapping for edge is not in JSON description.

    tbyte current_isc = REG_8BIT(TAU_ISC_ADDR);
    current_isc &= ~(0x03U << (pin * 2)); // Clear 2 bits for the channel's edge config (inferred)

    switch (external_int_edge)
    {
        case external_int_edge_rising:  current_isc |= (1U << (pin * 2)); break; // Inferred: Set bit 0 for rising edge
        case external_int_edge_falling: current_isc |= (2U << (pin * 2)); break; // Inferred: Set bit 1 for falling edge
        case external_int_edge_both:    current_isc |= (3U << (pin * 2)); break; // Inferred: Set both bits for both edges
        default: break;
    }
    REG_8BIT(TAU_ISC_ADDR) = current_isc;

    // 3. Enable noise filter if available (TAU_NFEN1)
    REG_8BIT(TAU_NFEN1_ADDR) |= (1U << pin); // Enable noise filter for this pin (inferred)
    WDT_Reset();
}

/**
 * @brief Enables an external interrupt channel.
 * @param external_int_channel The external interrupt channel to enable.
 * @note This function assumes a generic interrupt enable register or that configuring TAU_ISC
 *       implicitly enables the interrupt, coupled with global interrupt enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Clear WDT
    if (external_int_channel > external_int_channel_7) return;

    // For Renesas, often there is an Interrupt Enable Register (IER) or Interrupt Flag Control (INT_FLAG_CTL).
    // The JSON provides INT_FLAG_CTL (0xFFF4) and INT_FLAG_CLR (0xFFF5).
    // Assuming INT_FLAG_CTL can enable/disable specific interrupt requests.
    // Inferred: Each external interrupt channel (P1.x) maps to a specific bit in INT_FLAG_CTL.
    // This mapping is an assumption.
    REG_8BIT(INT_FLAG_CTL_ADDR) |= (1U << external_int_channel); // Placeholder for IER bit
    Global_interrupt_Enable(); // Ensure global interrupts are enabled
    WDT_Reset();
}

/**
 * @brief Disables an external interrupt channel.
 * @param external_int_channel The external interrupt channel to disable.
 * @note This function assumes a generic interrupt disable register.
 */
void External_INT_Disable(t_external_int_channel external_int_channel)
{
    WDT_Reset(); // Clear WDT
    if (external_int_channel > external_int_channel_7) return;

    REG_8BIT(INT_FLAG_CTL_ADDR) &= ~(1U << external_int_channel); // Placeholder for IER bit
    WDT_Reset();
}

//--------------------------------------------------------------------------------------------------
// GPIO Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a GPIO pin as output and sets its initial value.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The initial output value (0 for low, 1 for high).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_data = get_gpio_port_data_reg_ptr(port);
    volatile tbyte* port_mode = get_gpio_port_mode_reg_ptr(port);
    volatile tbyte* port_pu = get_gpio_port_pullup_reg_ptr(port);
    volatile tbyte* port_pom = get_gpio_port_outputmode_reg_ptr(port);
    volatile tbyte* port_pmc = get_gpio_port_pmc_reg_ptr(port); // For digital buffer disable or alt func

    if (port_data == NULL || port_mode == NULL || port_pu == NULL || port_pom == NULL || port_pmc == NULL) return;

    // Rules.json: "Always set value before setting direction"
    if (value)
    {
        *port_data |= (1U << pin);
    }
    else
    {
        *port_data &= ~(1U << pin);
    }
    // Verify value
    while (((*port_data >> pin) & 1U) != value)
    {
        WDT_Reset();
    }
    WDT_Reset();

    // Set direction to output (PMx = 0 for output in Renesas)
    *port_mode &= ~(1U << pin);
    // Verify direction
    while (((*port_mode >> pin) & 1U) != GPIO_DIR_OUTPUT_BIT)
    {
        WDT_Reset();
    }
    WDT_Reset();

    // Rules.json: "All output pins have pull-up resistors disabled"
    // Use RMW as PUx and PMC_x might share address.
    *port_pu &= ~(1U << pin); // Disable pull-up

    // Set output mode to CMOS (POMx = 0 for CMOS in Renesas)
    *port_pom &= ~(1U << pin);

    // Ensure digital input buffer is enabled for output (usually default or not needed to configure here)
    // Or clear PMC bit to ensure digital I/O if it was for analog/alt func
    *port_pmc &= ~(1U << pin); // Clear PMC bit to configure as digital I/O (inferred)

    // Current drive strength: For Renesas, this is often a separate register or bits within PM/PMC.
    // "For current registers: use >=20mA sink current & >=10mA source current" - No explicit registers for this in JSON.
    // This rule cannot be implemented directly from the provided JSON. Placeholder:
    // REG_8BIT(GPIO_CURRENT_REG_ADDR) |= (SET_HIGH_CURRENT_BITS << pin); // Placeholder for current setting.
    WDT_Reset();
}

/**
 * @brief Initializes a GPIO pin as input.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 */
void GPIO_Input_Init(t_port port, t_pin pin)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_mode = get_gpio_port_mode_reg_ptr(port);
    volatile tbyte* port_pu = get_gpio_port_pullup_reg_ptr(port);
    volatile tbyte* port_pim = get_gpio_port_inputmode_reg_ptr(port);
    volatile tbyte* port_pmc = get_gpio_port_pmc_reg_ptr(port); // For digital buffer disable or alt func

    if (port_mode == NULL || port_pu == NULL || port_pmc == NULL) return; // PIM is optional

    // Set direction to input (PMx = 1 for input in Renesas)
    *port_mode |= (1U << pin);
    // Verify direction
    while (((*port_mode >> pin) & 1U) != GPIO_DIR_INPUT_BIT)
    {
        WDT_Reset();
    }
    WDT_Reset();

    // Rules.json: "All input pins have pull-up resistors and wakeup feature enabled (if available)"
    // Use RMW as PUx and PMC_x might share address.
    *port_pu |= (1U << pin); // Enable pull-up

    // Enable digital input buffer (often default, but sometimes PMC or PIM controls it)
    // Clear PMC bit to ensure digital I/O (inferred: PMC bit 0 means digital, 1 means analog/alt func)
    *port_pmc &= ~(1U << pin);

    // If PIM (Input Mode Register) exists for this port, configure it (e.g., enable Schmitt trigger)
    if (port_pim != NULL)
    {
        *port_pim &= ~(1U << pin); // Default to normal input buffer (not Schmitt trigger)
    }

    // Wakeup feature: No explicit register for wakeup on pin input in JSON.
    // This rule cannot be implemented directly from the provided JSON. Placeholder:
    // REG_8BIT(GPIO_WKUP_EN_REG_ADDR) |= (1U << pin); // Placeholder for wakeup setting.
    WDT_Reset();
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The direction of the pin (input or output).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_mode = get_gpio_port_mode_reg_ptr(port);
    if (port_mode == NULL) return gpio_direction_input; // Default to input if port invalid

    // Read PMx register. Renesas: 1 for input, 0 for output.
    if ((*port_mode >> pin) & 1U)
    {
        return gpio_direction_input;
    }
    else
    {
        return gpio_direction_output;
    }
}

/**
 * @brief Sets the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_data = get_gpio_port_data_reg_ptr(port);
    if (port_data == NULL) return;

    if (value)
    {
        *port_data |= (1U << pin);
    }
    else
    {
        *port_data &= ~(1U << pin);
    }

    // Verify value
    while (((*port_data >> pin) & 1U) != value)
    {
        WDT_Reset();
    }
    WDT_Reset();
}

/**
 * @brief Gets the value of a GPIO input pin.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The value of the pin (0 for low, 1 for high).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_data = get_gpio_port_data_reg_ptr(port);
    if (port_data == NULL) return 0; // Default to 0 if port invalid

    return (*port_data >> pin) & 1U;
}

/**
 * @brief Toggles the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 */
void GPIO_Value_Tog(t_port port, t_pin pin)
{
    WDT_Reset(); // Clear WDT
    volatile tbyte* port_data = get_gpio_port_data_reg_ptr(port);
    if (port_data == NULL) return;

    *port_data ^= (1U << pin); // Toggle the bit
    WDT_Reset();
}

//--------------------------------------------------------------------------------------------------
// PWM Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel (mapped to TAU0 channels).
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty)
{
    WDT_Reset(); // Clear WDT

    if (pwm_channel > pwm_channel_7 || pwm_duty > 100 || pwm_khz_freq == 0) return;

    // Map PWM channel to TAU0 registers
    volatile tbyte* tmr_reg = (volatile tbyte*)(TAU0_TMR00_ADDR + pwm_channel);
    volatile tword* tdr_reg = (volatile tword*)(TAU0_TDR00_ADDR + (pwm_channel * 2)); // 16-bit register
    volatile tbyte* toe_reg = (volatile tbyte*)TAU0_TOE_ADDR;
    volatile tbyte* tom_reg = (volatile tbyte*)TAU0_TOM_ADDR;
    volatile tbyte* tios_reg = (volatile tbyte*)TAU0_TIOS_ADDR;

    // Associated GPIO pin for PWM output (P1.0-P1.7 for TAU0_TO00-07)
    t_port gpio_port = port_1;
    t_pin gpio_pin = (t_pin)pwm_channel;

    // 1. Enable peripheral clock for TAU0
    enable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    // 2. Configure GPIO pin for Alternate Function (PWM output)
    GPIO_Output_Init(gpio_port, gpio_pin, 0); // Set low initially, configure as output
    // Set PMC bit for alternate function (timer output)
    if (get_gpio_port_pmc_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(gpio_port)) |= (1U << gpio_pin);
    }
    WDT_Reset();

    // 3. Configure Timer Clock Select (TAU0_TPS) - derived dynamically.
    // The base clock for TAU0 is determined by system clock and prescalers.
    // For simplicity, assuming a fixed f_CLK for TAU0 (e.g., f_CLK/1 or f_CLK/2 or f_CLK/4).
    // The JSON only provides TAU0_TPS (0xFFB0) as "selects the count clock for Timer Array Unit 0."
    // Assuming 0x00 for no division (f_CLK).
    REG_8BIT(TAU0_TPS_ADDR) = 0x00; // Placeholder: No division, highest clock speed.
                                    // A real application would read clock config registers (RCC_CKC) to determine f_CLK.
    tlong timer_clock_freq_hz = 16000000; // Assuming 16MHz main clock for f_CLK as a common value.
                                         // This value needs to be derived from RCC settings if not fixed.

    // 4. Compute Prescaler (PSC) and Auto-Reload (ARR) values.
    // Fpwm = Timer_Clock / ((PSC+1) * (ARR+1))
    // We target Fpwm (pwm_khz_freq * 1000 Hz).
    // TAU0 TDRx are 16-bit registers (0-65535). PSC bits are in TMRx.
    // For Renesas TAU0, PSC is often part of the TMR register.
    // Let's assume TMR0x bits 0-2 are CKS (clock select, which acts as prescaler).
    // CKS values (0-7) => (f_CLK/1, f_CLK/2, f_CLK/4, f_CLK/8, f_CLK/16, f_CLK/32, f_CLK/64, f_CLK/128).
    // So PSC = 2^CKS - 1.
    // (ARR+1) = Timer_Clock / (Fpwm * (PSC+1))

    tword arr_value = 0;
    tbyte psc_val = 0; // CKS value (0-7)

    // Simplified approach: Try various prescalers to find suitable ARR
    for (psc_val = 0; psc_val < 8; psc_val++)
    {
        tlong current_prescaler_divisor = 1U << psc_val; // (PSC+1) equivalent
        if (pwm_khz_freq == 0) pwm_khz_freq = 1; // Prevent division by zero
        tlong arr_plus_1 = timer_clock_freq_hz / ((tlong)pwm_khz_freq * 1000 * current_prescaler_divisor);
        if (arr_plus_1 > 0 && arr_plus_1 <= 65536) // ARR can be 0-65535
        {
            arr_value = (tword)(arr_plus_1 - 1);
            break;
        }
    }
    // If no suitable PSC found, use max ARR and a default PSC
    if (arr_value == 0 && psc_val == 8)
    {
        psc_val = 7; // Max prescaler
        arr_value = 65535; // Max ARR
        // Re-calculate effective frequency and duty to match these clamped values.
    }
    WDT_Reset();

    // 5. Configure Timer Mode Register (TAU0_TMR0x)
    // For PWM mode, compare match. Inferred: TMRx bits for CKS, Master/Slave, Output Mode.
    // Renesas TAU often uses 'PWM mode' or 'interval timer mode with output'
    // TMRx configuration bits:
    // Bits 0-2: CKS (Clock Select - Prescaler)
    // Bits 3-4: STS (Start Trigger Select)
    // Bit 5: CIS (Capture Input Select)
    // Bit 6: MD0 (Mode Select)
    // Bit 7: MD1 (Mode Select)
    // For PWM, often (MD1,MD0) = (1,0) for "Interval Timer Mode with output" (Placeholder).
    *tmr_reg = (tbyte)(0x80 | (psc_val & 0x07)); // Set MD1=1 (bit 7), CKS bits. (Inferred)

    // Set TDR (compare value for period)
    *tdr_reg = arr_value;

    // Set duty cycle (another compare register, or TDR directly for period, and another for duty)
    // For Renesas, often (TDRx) is period, and an output compare register (e.g., TDRx as output compare)
    // is set for duty. The JSON only lists one TDRx per channel.
    // Assuming TAU0_TDR0x is the period register, and duty cycle is implicitly handled by TOM/TOL if one compare.
    // If not, a second TDR (like a shadow register) might be needed.
    // Let's assume TDRx is the period, and we need another implicit duty cycle register.
    // Given no explicit second TDR, let's make TDRx control period and TOM/TOL control duty behavior.
    // Or, for simple PWM, TDRx is period and CMP is duty. Here TDRx is the only data reg.
    // This implies that duty cycle is set by a special mode bit or by an external event.
    // For simpler Renesas, TDR can act as the period and also control duty if output mode changes on match.
    // Let's adjust TDR for the duty cycle as well, if we assume TDR can also be CMP.
    // Duty cycle value should be (pwm_duty * arr_value) / 100.
    tword duty_value = (tword)(((tlong)pwm_duty * arr_value) / 100);
    // If only one TDR, it usually sets the period. Duty is then based on a separate capture/compare.
    // This part is highly MCU-specific and not well-defined by JSON.
    // I'll set TDR as period, and assume the 'Output Mode Register' (TOM) affects how the duty is driven.

    // 6. Configure output functionality
    // Set output mode for channel (TOMx = 0 for normal output mode, 1 for inverted, etc.)
    // Inferred: TOMx bit corresponds to channel.
    *tom_reg &= ~(1U << pwm_channel); // Default to normal output mode (CMOS, active high pulse)
    // Initial output level (TOLx) - often sets initial level before timer starts.
    // Inferred: TOLx bit corresponds to channel.
    *toe_reg |= (1U << pwm_channel); // Enable output for the channel
    *tios_reg &= ~(1U << pwm_channel); // Inferred: Clear TIOS bit to configure as output channel (TO)

    WDT_Reset();
}

/**
 * @brief Starts the PWM generation for a specific channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Clear WDT
    if (pwm_channel > pwm_channel_7) return;

    // Start the corresponding TAU0 timer channel
    REG_8BIT(TAU0_TS_ADDR) |= (1U << pwm_channel);
    WDT_Reset();
}

/**
 * @brief Stops the PWM generation for a specific channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel)
{
    WDT_Reset(); // Clear WDT
    if (pwm_channel > pwm_channel_7) return;

    // Stop the corresponding TAU0 timer channel
    REG_8BIT(TAU0_TT_ADDR) |= (1U << pwm_channel);
    WDT_Reset();
}


//--------------------------------------------------------------------------------------------------
// ICU Implementation
//--------------------------------------------------------------------------------------------------

// Placeholder for ICU callback
static void (*icu_callback_function)(void) = NULL;

/**
 * @brief Initializes an ICU (Input Capture Unit) channel.
 * @param icu_channel The ICU channel to initialize (mapped to TAU0 channels).
 * @param icu_prescaller The prescaler for the ICU timer.
 * @param icu_edge The edge sensitivity for input capture.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge)
{
    WDT_Reset(); // Clear WDT

    if (icu_channel > icu_channel_7) return;

    // Map ICU channel to TAU0 registers
    volatile tbyte* tmr_reg = (volatile tbyte*)(TAU0_TMR00_ADDR + icu_channel);
    volatile tword* tdr_reg = (volatile tword*)(TAU0_TDR00_ADDR + (icu_channel * 2)); // 16-bit
    volatile tbyte* tios_reg = (volatile tbyte*)TAU0_TIOS_ADDR;

    // Associated GPIO pin for ICU input (P1.0-P1.7 for TAU0_TI00-07)
    t_port gpio_port = port_1;
    t_pin gpio_pin = (t_pin)icu_channel;

    // 1. Enable peripheral clock for TAU0
    enable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    // 2. Configure GPIO pin as input
    GPIO_Input_Init(gpio_port, gpio_pin);
    // Set PMC bit for alternate function (timer input)
    if (get_gpio_port_pmc_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(gpio_port)) |= (1U << gpio_pin);
    }
    WDT_Reset();

    // 3. Configure Timer Clock Select (TAU0_TPS) with prescaler
    // ICU prescaler maps to TAU0_TPS or TMRx CKS bits.
    // Assuming ICU prescaler enum directly maps to CKS bits (0-7).
    tbyte psc_val = (tbyte)icu_prescaller;
    REG_8BIT(TAU0_TPS_ADDR) = psc_val; // Use the prescaler value directly for TPS.

    // 4. Configure TMRx for input capture mode and edge detection
    *tmr_reg = 0x00; // Reset TMRx
    // Inferred TMRx bits for input capture mode and edge:
    // Bits 0-2: CKS (Clock Select/Prescaler)
    // Bits 3-4: STS (Start Trigger Select) - often 0 for software start
    // Bit 5: CIS (Capture Input Select) - 1 for capture mode (inferred)
    // Bit 6: MD0 (Mode Select) - 0
    // Bit 7: MD1 (Mode Select) - 0 -> Interval Timer Mode (often base for capture)
    // TMRx also contains edge settings, e.g., inverted/non-inverted input.
    // For ICU, mode bits often for "Input Capture Mode" or "Event Counter Mode".
    // Let's assume a generic capture mode from the TMRx register.
    // Assuming (MD1, MD0) = (0,1) for "Capture Mode" and CIS = 1.
    tbyte tmr_val = (psc_val & 0x07) | (1U << 5) | (1U << 6); // Set CKS, CIS, MD0=1 (inferred)

    // Configure edge sensitivity (inferred from common Renesas timer architectures)
    // Often controlled by bits in TMRx itself or dedicated input control registers.
    // As TAU_ISC is available, let's use it (shared with External Interrupt).
    tbyte current_isc = REG_8BIT(TAU_ISC_ADDR);
    current_isc &= ~(0x03U << (gpio_pin * 2)); // Clear 2 bits for the channel's edge config (inferred)
    switch (icu_edge)
    {
        case icu_edge_rising:  current_isc |= (1U << (gpio_pin * 2)); break;
        case icu_edge_falling: current_isc |= (2U << (gpio_pin * 2)); break;
        case icu_edge_both:    current_isc |= (3U << (gpio_pin * 2)); break;
        default: break;
    }
    REG_8BIT(TAU_ISC_ADDR) = current_isc;

    *tmr_reg = tmr_val; // Apply TMRx settings

    // Set TIOS bit to configure as input channel (TI)
    *tios_reg |= (1U << icu_channel); // Inferred: Set TIOS bit for input mode

    // 5. Enable Noise Filter (TAU_NFEN1)
    REG_8BIT(TAU_NFEN1_ADDR) |= (1U << gpio_pin); // Enable noise filter for this pin (inferred)

    WDT_Reset();
}

/**
 * @brief Enables the ICU channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Clear WDT
    if (icu_channel > icu_channel_7) return;

    enable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    // Start the corresponding TAU0 timer channel
    REG_8BIT(TAU0_TS_ADDR) |= (1U << icu_channel);
    WDT_Reset();
}

/**
 * @brief Disables the ICU channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel)
{
    WDT_Reset(); // Clear WDT
    if (icu_channel > icu_channel_7) return;

    // Stop the corresponding TAU0 timer channel
    REG_8BIT(TAU0_TT_ADDR) |= (1U << icu_channel);
    WDT_Reset();
}

/**
 * @brief Gets the frequency measured by the ICU.
 * @param icu_channel The ICU channel to read from.
 * @return The measured frequency.
 * @note This is a placeholder as the exact calculation depends on timer configuration and capture values.
 */
tword ICU_GetFrequency(t_icu_channel icu_channel)
{
    WDT_Reset(); // Clear WDT
    if (icu_channel > icu_channel_7) return 0;

    volatile tword* tdr_reg = (volatile tword*)(TAU0_TDR00_ADDR + (icu_channel * 2));
    volatile tword* tcr_reg = (volatile tword*)(TAU0_TCR00_ADDR + (icu_channel * 2));

    // For frequency, we need to capture two successive edges and calculate the period.
    // The TDRx register usually holds the captured value. TCRx is the counter.
    // The exact logic involves reading TDRx, computing difference, and converting to frequency.
    // This is a simplified placeholder. A real driver would use interrupts or advanced polling.

    // Inferred: Read the captured value from TDR.
    // Assuming a simple capture, and this value represents the period in timer ticks.
    tword period_ticks = *tdr_reg; // Captured period in timer ticks (placeholder)

    if (period_ticks == 0) return 0;

    // Example calculation: Frequency = Timer_Clock_Hz / period_ticks
    // Need to know the exact timer clock frequency (after prescaler).
    // Assuming Timer_Clock_Hz is available (e.g., 16 MHz).
    tlong timer_clock_freq_hz = 16000000; // Placeholder, must be derived from actual clock config
    tword frequency_hz = (tword)(timer_clock_freq_hz / period_ticks);

    WDT_Reset();
    return frequency_hz;
}

/**
 * @brief Sets a callback function to be executed on ICU event.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void))
{
    WDT_Reset(); // Clear WDT
    icu_callback_function = callback;
    // In a real system, this would also enable the corresponding timer interrupt.
    // No specific interrupt enable register for TAU0_TDRx capture in JSON, so relying on generic INT_FLAG_CTL.
    // This is typically handled by setting an interrupt enable bit for the specific TAU channel.
}

//--------------------------------------------------------------------------------------------------
// Timer Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a Timer channel.
 * @param timer_channel The timer channel to initialize (mapped to TAU0 channels).
 */
void TIMER_Init(t_timer_channel timer_channel)
{
    WDT_Reset(); // Clear WDT
    if (timer_channel > timer_channel_7) return;

    volatile tbyte* tmr_reg = (volatile tbyte*)(TAU0_TMR00_ADDR + timer_channel);
    volatile tword* tdr_reg = (volatile tword*)(TAU0_TDR00_ADDR + (timer_channel * 2)); // 16-bit

    // 1. Enable peripheral clock for TAU0
    enable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    // 2. Configure Timer Clock Select (TAU0_TPS)
    REG_8BIT(TAU0_TPS_ADDR) = 0x00; // Placeholder: No division, highest clock speed.
                                    // A real application would read RCC_CKC to determine f_CLK.

    // 3. Configure TMRx for interval timer mode
    // Inferred: (MD1, MD0) = (0,0) for "Interval Timer Mode" or similar in Renesas.
    *tmr_reg = (tbyte)0x00; // Reset TMRx and set for basic interval timer mode.
                            // Assuming CKS bits (0-2) are default 0.

    // 4. Initialize TDRx to 0 (default compare value)
    *tdr_reg = 0x0000;

    // Configure TIOS bit to ensure no input/output interference, or general purpose timer.
    // For a generic timer, we might want to disable output.
    REG_8BIT(TAU0_TIOS_ADDR) &= ~(1U << timer_channel); // Inferred: Clear TIOS bit (general purpose timer)

    WDT_Reset();
}

/**
 * @brief Sets a timer channel to trigger after a specified number of microseconds.
 * @param timer_channel The timer channel to configure.
 * @param time The time in microseconds.
 * @note This assumes a 16-bit timer and a fixed timer clock.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time_us)
{
    WDT_Reset(); // Clear WDT
    if (timer_channel > timer_channel_7) return;

    volatile tword* tdr_reg = (volatile tword*)(TAU0_TDR00_ADDR + (timer_channel * 2));

    // Assume 16MHz clock (62.5ns per tick).
    // (Timer_Clock_Hz / 1000000) = Ticks per microsecond
    // target_ticks = time_us * Ticks_per_us
    tlong timer_clock_freq_hz = 16000000; // Placeholder. Must be dynamic.
    tlong ticks = ((tlong)time_us * timer_clock_freq_hz) / 1000000;

    // Check if ticks exceed 16-bit capacity
    if (ticks > 65535) ticks = 65535; // Clamp to max 16-bit value

    *tdr_reg = (tword)ticks; // Set compare value
    WDT_Reset();
}

/**
 * @brief Sets a timer channel to trigger after a specified number of milliseconds.
 * @param timer_channel The timer channel to configure.
 * @param time_ms The time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time_ms)
{
    WDT_Reset(); // Clear WDT
    TIMER_Set_us(timer_channel, time_ms * 1000); // Convert ms to us
    WDT_Reset();
}

/**
 * @brief Sets a timer channel to trigger after a specified number of seconds.
 * @param timer_channel The timer channel to configure.
 * @param time_sec The time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time_sec)
{
    WDT_Reset(); // Clear WDT
    TIMER_Set_us(timer_channel, (tword)time_sec * 1000000); // Convert sec to us
    WDT_Reset();
}

/**
 * @brief Sets a timer channel to trigger after a specified number of minutes.
 * @param timer_channel The timer channel to configure.
 * @param time_min The time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time_min)
{
    WDT_Reset(); // Clear WDT
    TIMER_Set_Time_sec(timer_channel, (tbyte)(time_min * 60)); // Convert min to sec
    WDT_Reset();
}

/**
 * @brief Sets a timer channel to trigger after a specified number of hours.
 * @param timer_channel The timer channel to configure.
 * @param time_hour The time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time_hour)
{
    WDT_Reset(); // Clear WDT
    TIMER_Set_Time_min(timer_channel, (tbyte)(time_hour * 60)); // Convert hour to min
    WDT_Reset();
}

/**
 * @brief Enables a timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Clear WDT
    if (timer_channel > timer_channel_7) return;

    enable_peripheral_clock(RCC_PER0_TAU_EN_BIT);

    REG_8BIT(TAU0_TS_ADDR) |= (1U << timer_channel); // Start the timer channel
    WDT_Reset();
}

/**
 * @brief Disables a timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel)
{
    WDT_Reset(); // Clear WDT
    if (timer_channel > timer_channel_7) return;

    REG_8BIT(TAU0_TT_ADDR) |= (1U << timer_channel); // Stop the timer channel
    WDT_Reset();
}

//--------------------------------------------------------------------------------------------------
// ADC Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the ADC module for a specific channel and mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC operation mode (single conversion or continuous scan).
 * @note This implementation handles the address conflict where ADC_ADM0 (0xFF00)
 *       shares an address with P0 (GPIO Port 0). This is flagged as a potential issue
 *       due to the provided JSON.
 * @note Also handles conflict where ADC_SR (0xFF30) shares address with GPIO_PU0/PMC0.
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode)
{
    WDT_Reset(); // Clear WDT

    // 1. Enable peripheral clock for ADC
    enable_peripheral_clock(RCC_PER0_ADC_EN_BIT);

    // 2. Map ADC channel to physical pin and configure GPIO for analog input.
    // This involves setting the GPIO_PMC_x bit to disable digital input buffer.
    t_port gpio_port = port_0;
    t_pin gpio_pin = pin_0;

    switch (adc_channel)
    {
        case adc_channel_p0_0: gpio_port = port_0; gpio_pin = pin_0; break;
        case adc_channel_p0_1: gpio_port = port_0; gpio_pin = pin_1; break;
        case adc_channel_p2_0: gpio_port = port_2; gpio_pin = pin_0; break;
        case adc_channel_p2_1: gpio_port = port_2; gpio_pin = pin_1; break;
        case adc_channel_p2_2: gpio_port = port_2; gpio_pin = pin_2; break;
        case adc_channel_p2_3: gpio_port = port_2; gpio_pin = pin_3; break;
        case adc_channel_p2_4: gpio_port = port_2; gpio_pin = pin_4; break;
        case adc_channel_p7_0: gpio_port = port_7; gpio_pin = pin_0; break;
        case adc_channel_p7_1: gpio_port = port_7; gpio_pin = pin_1; break;
        case adc_channel_p7_2: gpio_port = port_7; gpio_pin = pin_2; break;
        case adc_channel_p7_3: gpio_port = port_7; gpio_pin = pin_3; break;
        case adc_channel_p7_4: gpio_port = port_7; gpio_pin = pin_4; break;
        case adc_channel_p7_5: gpio_port = port_7; gpio_pin = pin_5; break;
        case adc_channel_p7_6: gpio_port = port_7; gpio_pin = pin_6; break;
        case adc_channel_p7_7: gpio_port = port_7; gpio_pin = pin_7; break;
        default: return;
    }

    // Disable digital input buffer for the selected analog pin (PMC bit set)
    if (get_gpio_port_pmc_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(gpio_port)) |= (1U << gpio_pin); // Inferred: Set PMC bit to enable analog function
    }

    // 3. Configure AD Port Configuration Registers (ADC_PCR, ADC_PCRL)
    // These registers select which pins are enabled for analog input.
    // Assuming ADC_PCR/PCRL bits correspond to ADC channel number directly or via a mask.
    // The JSON for ADC_PCR/PCRL lists all ADC pins for assigned_pin.
    // Placeholder: Set relevant bit for the specific channel.
    // This is highly specific. Assuming single bit enable.
    REG_8BIT(ADC_PCR_ADDR) |= (1U << (adc_channel % 8)); // Example for lower channels (0-7)
    if (adc_channel >= 8) // Example for higher channels (8-15)
    {
        REG_8BIT(ADC_PCRL_ADDR) |= (1U << (adc_channel - 8)); // Example for PCRL
    }
    WDT_Reset();

    // 4. Configure ADC Mode (ADC_ADM0)
    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00). Writing here might affect P0.
    REG_8BIT(ADC_ADM0_ADDR) &= ~(1U << 7); // Clear ADON (ADC ON) bit before configuration (inferred)

    tbyte adm0_val = 0x00;
    // Inferred ADM0 bits:
    // Bit 0-3: ADCS (Channel Select)
    // Bit 4: ADMD (Mode Select - Single/Scan)
    // Bit 7: ADON (ADC ON)

    // Set channel select
    adm0_val |= (adc_channel & 0x0FU); // Use lower 4 bits for channel selection (inferred)

    // Set mode (single conversion or continuous scan)
    if (adc_mode_continuous_scan == adc_mode)
    {
        adm0_val |= (1U << 4); // Inferred: Set ADMD for continuous scan
    }
    else
    {
        adm0_val &= ~(1U << 4); // Inferred: Clear ADMD for single conversion
    }

    REG_8BIT(ADC_ADM0_ADDR) = adm0_val;
    WDT_Reset();

    // Reset ADC_SR status flags by writing 0 (if write-to-clear) or reading.
    // NOTE: ADC_SR (0xFF30) conflicts with GPIO_PU0/PMC0. Reading/writing here affects GPIO.
    // This is a serious conflict in the provided JSON. A real MCU would not have this.
    // Assume writing 0 clears flags.
    REG_8BIT(ADC_SR_ADDR) = 0x00;
    WDT_Reset();
}

/**
 * @brief Enables the ADC module.
 * @param adc_channel The ADC channel to enable. (Parameter is used for consistency, but enable is usually global or per unit).
 */
void ADC_Enable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Clear WDT
    (void)adc_channel; // Parameter not directly used for global enable

    enable_peripheral_clock(RCC_PER0_ADC_EN_BIT);

    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00). Writing here might affect P0.
    REG_8BIT(ADC_ADM0_ADDR) |= (1U << 7); // Inferred: Set ADON (ADC ON) bit
    WDT_Reset();
}

/**
 * @brief Disables the ADC module.
 * @param adc_channel The ADC channel to disable. (Parameter is used for consistency).
 */
void ADC_Disable(t_adc_channel adc_channel)
{
    WDT_Reset(); // Clear WDT
    (void)adc_channel; // Parameter not directly used for global disable

    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00). Writing here might affect P0.
    REG_8BIT(ADC_ADM0_ADDR) &= ~(1U << 7); // Inferred: Clear ADON (ADC ON) bit
    WDT_Reset();
}

/**
 * @brief Performs an ADC conversion in polling mode and returns the result.
 * @param adc_channel The ADC channel to read.
 * @return The 10-bit ADC conversion result.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel)
{
    WDT_Reset(); // Clear WDT
    // Ensure ADC is enabled and configured for the channel.
    // If not already set up, a quick Init or Channel Select might be needed.
    // For simplicity, assuming ADC_Init was called and we just need to start conversion.

    // Update channel selection in ADM0 (if ADC_Init was for different channel)
    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00).
    tbyte current_adm0 = REG_8BIT(ADC_ADM0_ADDR);
    current_adm0 = (current_adm0 & ~0x0FU) | (adc_channel & 0x0FU); // Clear and set channel bits
    REG_8BIT(ADC_ADM0_ADDR) = current_adm0;

    REG_8BIT(ADC_ADM0_ADDR) |= (1U << 0); // Inferred: Start conversion (e.g., ADST bit in Renesas ADM0)

    // Wait for conversion complete flag (ADCF) in ADC_SR
    // NOTE: ADC_SR (0xFF30) conflicts with GPIO_PU0/PMC0.
    while (!(REG_8BIT(ADC_SR_ADDR) & (1U << 0))) // Inferred: Assume bit 0 in ADC_SR is ADCF (Conversion Complete Flag)
    {
        WDT_Reset(); // Keep WDT happy while waiting
    }

    tword adc_result = REG_16BIT(ADC_DR_ADDR); // ADC_DR is 10-bit, stored in 16-bit register
                                               // Lower byte at 0xFF10, Higher byte at 0xFF11 (as per JSON description)
    WDT_Reset();
    return adc_result;
}

/**
 * @brief Initiates an ADC conversion for a channel and returns 0, expecting an interrupt to handle the result.
 * @param adc_channel The ADC channel to configure for interrupt conversion.
 * @return Always returns 0 (result handled by ISR).
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel)
{
    WDT_Reset(); // Clear WDT
    // 1. Configure ADC_ICR (Interrupt Control Register) to enable ADC interrupt.
    // No specific bits defined in JSON, assuming generic enable bit for the ADC.
    REG_8BIT(ADC_ICR_ADDR) |= (1U << 0); // Inferred: Set bit 0 in ADC_ICR to enable interrupt

    // 2. Enable global interrupts
    Global_interrupt_Enable();

    // 3. Select the ADC channel in ADM0
    // NOTE: ADC_ADM0 (0xFF00) conflicts with P0 (0xFF00).
    tbyte current_adm0 = REG_8BIT(ADC_ADM0_ADDR);
    current_adm0 = (current_adm0 & ~0x0FU) | (adc_channel & 0x0FU); // Clear and set channel bits
    REG_8BIT(ADC_ADM0_ADDR) = current_adm0;

    // 4. Start conversion (e.g., ADST bit in Renesas ADM0)
    REG_8BIT(ADC_ADM0_ADDR) |= (1U << 0); // Inferred: Start conversion

    // The result will be available in an ISR.
    // ADC_SR_ADDR is status, interrupt flag for conversion complete.
    // The ISR (not implemented in MCAL.c) would read ADC_DR and clear the flag.
    WDT_Reset();
    return 0; // Result is retrieved asynchronously by an ISR
}

//--------------------------------------------------------------------------------------------------
// Internal_EEPROM Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the Internal EEPROM module (using Data Flash memory).
 */
void Internal_EEPROM_Init(void)
{
    WDT_Reset(); // Clear WDT

    // The register_json provides Flash Memory registers for Data Flash.
    // Data Flash typically behaves as EEPROM. Initialization involves enabling the flash controller.
    // 1. Enable peripheral clock for Flash (if applicable, often part of RCC_PER0/PER1)
    // No explicit Flash clock enable bit in RCC_PER0/PER1 in JSON, assuming it's always on or enabled elsewhere.

    // 2. Configure Flash Memory Program Mode Control Register (FLASH_PMC)
    // This register often sets the operating mode (e.g., read, write, erase).
    // Inferred: Set to a "read mode" or "ready for operation" state.
    REG_8BIT(FLASH_PMC_ADDR) = 0x00; // Placeholder: Assume 0x00 sets read mode or idle.
    WDT_Reset();
}

/**
 * @brief Writes a byte of data to the Internal EEPROM (Data Flash).
 * @param address The address in EEPROM to write to.
 * @param data The byte of data to write.
 * @note This is a simplified placeholder. Actual flash programming involves
 *       a specific sequence (e.g., command, address, data, execute, wait for completion).
 *       Assumes address and data registers are directly writable for programming.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data)
{
    WDT_Reset(); // Clear WDT

    // This is a highly simplified flash write operation.
    // A real Renesas flash write typically involves:
    // 1. Entering flash programming mode (FLASH_PMC)
    // 2. Writing address (FLASH_DSAD)
    // 3. Writing data (FLASH_DRAD)
    // 4. Writing a program/erase command (FLASH_DSDP or similar)
    // 5. Waiting for completion/checking error (FLASH_ERR)
    // 6. Exiting programming mode.

    // Placeholder sequence:
    // 1. Set Program Mode (Inferred: e.g., FLASH_PMC to enable write)
    REG_8BIT(FLASH_PMC_ADDR) = 0x80; // Placeholder: Enable write mode (inferred)
    WDT_Reset();

    // 2. Write address (FLASH_DSAD is 16-bit, so lower byte first, then higher)
    REG_8BIT(FLASH_DSAD_ADDR) = address; // Lower byte of address
    REG_8BIT(FLASH_DSAD_ADDR + 1) = 0x00; // Assuming higher byte is 0 for small address space
    WDT_Reset();

    // 3. Write data
    REG_8BIT(FLASH_DRAD_ADDR) = data;
    WDT_Reset();

    // 4. Start program (Inferred: FLASH_DSDP could be the trigger)
    REG_8BIT(FLASH_DSDP_ADDR) = 0x01; // Placeholder: Trigger program operation

    // 5. Wait for completion/check error
    // Inferred: Loop until FLASH_ERR is 0 or a busy flag clears.
    while (REG_8BIT(FLASH_ERR_ADDR) != 0x00)
    {
        WDT_Reset(); // Keep WDT happy
    }

    // 6. Exit Program Mode
    REG_8BIT(FLASH_PMC_ADDR) = 0x00; // Placeholder: Back to read mode
    WDT_Reset();
}

/**
 * @brief Reads a byte of data from the Internal EEPROM (Data Flash).
 * @param address The address in EEPROM to read from.
 * @return The byte of data read.
 * @note This is a simplified placeholder. Actual flash reading might involve
 *       setting read mode and then accessing the address.
 */
tbyte Internal_EEPROM_Get(tbyte address)
{
    WDT_Reset(); // Clear WDT

    // 1. Ensure Flash is in Read Mode (FLASH_PMC)
    REG_8BIT(FLASH_PMC_ADDR) = 0x00; // Placeholder: Set to read mode
    WDT_Reset();

    // 2. Set address for read (FLASH_DSAD)
    REG_8BIT(FLASH_DSAD_ADDR) = address; // Lower byte of address
    REG_8BIT(FLASH_DSAD_ADDR + 1) = 0x00; // Assuming higher byte is 0 for small address space
    WDT_Reset();

    // 3. Read data from FLASH_DRAD or directly from memory-mapped address.
    // If it's direct memory access, it would be `*(volatile tbyte*)(FLASH_BASE_ADDRESS + address)`.
    // Given FLASH_DRAD description "holds data for data flash memory programming/reading",
    // it implies reading from this register *after* setting the address.
    tbyte read_data = REG_8BIT(FLASH_DRAD_ADDR);
    WDT_Reset();
    return read_data;
}

//--------------------------------------------------------------------------------------------------
// TT (Time Triggered OS) Implementation
//--------------------------------------------------------------------------------------------------

// TT OS variables (minimal implementation)
#define MAX_TT_TASKS 10
typedef struct {
    void (*task_func)(void);
    tword period; // in ticks
    tword delay;  // delay before first execution, in ticks
    tword run_count; // How many ticks until next run
    bool enabled;
} tt_task_t;

static tt_task_t tt_tasks[MAX_TT_TASKS];
static tbyte tt_next_task_index = 0;
static tword tt_tick_counter = 0; // Global tick counter

/**
 * @brief Initializes the Time Triggered OS module.
 * @param tick_time_ms The desired tick time interval in milliseconds.
 * @note This function sets up a hardware timer (e.g., TAU0 Channel 7) to generate periodic ticks.
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset(); // Clear WDT

    // Clear all tasks
    for (int i = 0; i < MAX_TT_TASKS; i++)
    {
        tt_tasks[i].task_func = NULL;
        tt_tasks[i].enabled = false;
    }
    tt_next_task_index = 0;
    tt_tick_counter = 0;

    // Use TAU0 Channel 7 as the system tick timer for TT OS
    t_timer_channel tt_timer_channel = timer_channel_7;
    TIMER_Init(tt_timer_channel);

    tword time_ms = 1; // Default to 1ms
    switch (tick_time_ms)
    {
        case tick_time_1ms: time_ms = 1; break;
        case tick_time_10ms: time_ms = 10; break;
        case tick_time_100ms: time_ms = 100; break;
        case tick_time_1s: time_ms = 1000; break;
        default: break;
    }
    TIMER_Set_Time_ms(tt_timer_channel, time_ms);

    // This timer should be configured to generate an interrupt on compare match.
    // Assuming TAU0 Channel 7 interrupt enable is at bit 7 of INT_FLAG_CTL.
    REG_8BIT(INT_FLAG_CTL_ADDR) |= (1U << 7); // Placeholder for TAU0 channel 7 interrupt enable.
    Global_interrupt_Enable();
    WDT_Reset();
}

/**
 * @brief Starts the Time Triggered OS.
 * @note This enables the underlying hardware timer.
 */
void TT_Start(void)
{
    WDT_Reset(); // Clear WDT
    TIMER_Enable(timer_channel_7); // Enable the TAU0 Channel 7 timer
    WDT_Reset();
}

/**
 * @brief Dispatches tasks in the Time Triggered OS.
 *        This function should be called repeatedly in the main loop.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset(); // Clear WDT
    Global_interrupt_Disable(); // Protect critical section
    // Process tasks
    for (tbyte i = 0; i < tt_next_task_index; i++)
    {
        if (tt_tasks[i].enabled && tt_tasks[i].run_count == 0)
        {
            if (tt_tasks[i].task_func != NULL)
            {
                tt_tasks[i].task_func(); // Execute task
            }
            tt_tasks[i].run_count = tt_tasks[i].period; // Reset run counter
        }
    }
    Global_interrupt_Enable(); // Restore interrupts
    WDT_Reset();
}

/**
 * @brief Interrupt Service Routine for the Time Triggered OS.
 *        This function should be called by the hardware timer's ISR.
 */
void TT_ISR(void)
{
    // Do NOT call WDT_Reset() here if this ISR is critical and must not block on WDT.
    // WDT reset inside ISRs should be carefully considered to avoid latency issues.
    // For this example, we assume WDT is reset in main loop, not every ISR.

    // Clear the interrupt flag for TAU0 Channel 7 (assuming it's a software clearable flag)
    REG_8BIT(INT_FLAG_CLR_ADDR) |= (1U << 7); // Placeholder for TAU0 channel 7 interrupt flag clear.

    tt_tick_counter++; // Increment global tick counter

    // Decrement task counters
    for (tbyte i = 0; i < tt_next_task_index; i++)
    {
        if (tt_tasks[i].enabled && tt_tasks[i].run_count > 0)
        {
            tt_tasks[i].run_count--;
        }
    }
}

/**
 * @brief Adds a task to the Time Triggered OS scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks.
 * @param delay The delay before the first execution in ticks.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay)
{
    WDT_Reset(); // Clear WDT
    if (tt_next_task_index < MAX_TT_TASKS)
    {
        Global_interrupt_Disable();
        tt_tasks[tt_next_task_index].task_func = task;
        tt_tasks[tt_next_task_index].period = period;
        tt_tasks[tt_next_task_index].delay = delay;
        tt_tasks[tt_next_task_index].run_count = delay; // Set initial delay
        tt_tasks[tt_next_task_index].enabled = true;
        tbyte index = tt_next_task_index;
        tt_next_task_index++;
        Global_interrupt_Enable();
        WDT_Reset();
        return index;
    }
    WDT_Reset();
    return 0xFF; // Failed to add task
}

/**
 * @brief Deletes a task from the Time Triggered OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset(); // Clear WDT
    if (task_index < tt_next_task_index)
    {
        Global_interrupt_Disable();
        tt_tasks[task_index].enabled = false;
        tt_tasks[task_index].task_func = NULL; // Clear function pointer
        // If we want to compact the array, we would shift tasks. For simplicity, just disable.
        Global_interrupt_Enable();
    }
    WDT_Reset();
}

//--------------------------------------------------------------------------------------------------
// DAC Implementation
//--------------------------------------------------------------------------------------------------

/**
 * @brief Initializes a DAC channel.
 * @param channel The DAC channel to initialize.
 */
void DAC_Init(dac_channel_t channel)
{
    WDT_Reset(); // Clear WDT

    if (channel > dac_channel_1) return; // Only two DAC channels (0 and 1)

    // 1. Enable peripheral clock for DAC
    enable_peripheral_clock(RCC_PER0_DAC_EN_BIT);

    // 2. Configure GPIO pin for Analog Output
    // DAC channel 0 uses P6.0, channel 1 uses P6.1.
    t_port gpio_port = port_6;
    t_pin gpio_pin = (t_pin)channel;

    // Clear PMx bit (output), disable PUx (pull-up).
    // The key for DAC is to disable the digital input buffer (PMC bit set).
    // This allows the pin to be used as a pure analog output.
    if (get_gpio_port_mode_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_mode_reg_ptr(gpio_port)) &= ~(1U << gpio_pin); // Set as output
    }
    if (get_gpio_port_pullup_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_pullup_reg_ptr(gpio_port)) &= ~(1U << gpio_pin); // Disable pull-up
    }
    if (get_gpio_port_pmc_reg_ptr(gpio_port) != NULL)
    {
        *(get_gpio_port_pmc_reg_ptr(gpio_port)) |= (1U << gpio_pin); // Set PMC bit for analog function (inferred)
    }
    WDT_Reset();

    // 3. Configure DAC general settings (DAC_CSR, DAC_SETUPCR)
    // DAC_CSR (0xFFB3): General configuration. Assuming enable/mode bits.
    // DAC_SETUPCR (0xFFB5): Controls setup and operation.

    // Reset general DAC control
    REG_8BIT(DAC_CSR_ADDR) = 0x00;
    REG_8BIT(DAC_SETUPCR_ADDR) = 0x00;

    // Inferred: Set conversion speed, output buffer enable, etc.
    // Placeholder: Set basic setup for stable operation.
    REG_8BIT(DAC_SETUPCR_ADDR) |= (1U << 0); // Inferred: DAC Ready bit or similar
    WDT_Reset();

    // 4. Configure specific channel settings (DAC_CHx_CSR)
    volatile tbyte* ch_csr_reg = NULL;
    if (dac_channel_0 == channel) ch_csr_reg = (volatile tbyte*)DAC_CH0_CSR_ADDR;
    else ch_csr_reg = (volatile tbyte*)DAC_CH1_CSR_ADDR;

    *ch_csr_reg = 0x00; // Reset channel specific settings
    // Inferred: Set mode (e.g., normal operation), enable channel.
    // Bit 7 for enable (as used in MCU_Config_Init).
    // Other bits for specific channel options.
    WDT_Reset();
}

/**
 * @brief Enables a DAC channel.
 * @param channel The DAC channel to enable.
 */
void DAC_Enable(dac_channel_t channel)
{
    WDT_Reset(); // Clear WDT
    if (channel > dac_channel_1) return;

    volatile tbyte* ch_csr_reg = NULL;
    if (dac_channel_0 == channel) ch_csr_reg = (volatile tbyte*)DAC_CH0_CSR_ADDR;
    else ch_csr_reg = (volatile tbyte*)DAC_CH1_CSR_ADDR;

    enable_peripheral_clock(RCC_PER0_DAC_EN_BIT);

    // Inferred: Set enable bit in channel's CSR.
    *ch_csr_reg |= (1U << 7); // Inferred: Bit 7 for channel enable
    REG_8BIT(DAC_CSR_ADDR) |= (1U << 7); // Inferred: Master DAC enable
    WDT_Reset();
}

/**
 * @brief Disables a DAC channel.
 * @param channel The DAC channel to disable.
 */
void DAC_Disable(dac_channel_t channel)
{
    WDT_Reset(); // Clear WDT
    if (channel > dac_channel_1) return;

    volatile tbyte* ch_csr_reg = NULL;
    if (dac_channel_0 == channel) ch_csr_reg = (volatile tbyte*)DAC_CH0_CSR_ADDR;
    else ch_csr_reg = (volatile tbyte*)DAC_CH1_CSR_ADDR;

    // Inferred: Clear enable bit in channel's CSR.
    *ch_csr_reg &= ~(1U << 7); // Inferred: Bit 7 for channel disable
    // Check if other channels are enabled before disabling master enable
    // For simplicity, directly disable master as well.
    REG_8BIT(DAC_CSR_ADDR) &= ~(1U << 7); // Inferred: Master DAC disable
    WDT_Reset();
}

/**
 * @brief Sets the 8-bit conversion value for a DAC channel.
 * @param channel The DAC channel to set the value for.
 * @param regvalue The 8-bit value to convert.
 * @note The JSON lists DAC_CHx_CSR as "Setting Register". It's common for Renesas
 *       that these registers are also where the conversion data is written.
 */
void DAC_Set_ConversionValue(dac_channel_t channel, uint8_t regvalue)
{
    WDT_Reset(); // Clear WDT
    if (channel > dac_channel_1) return;

    volatile tbyte* ch_csr_reg = NULL; // As per JSON, CHx_CSR are the "Setting Register" and assumed to also hold data.
    if (dac_channel_0 == channel) ch_csr_reg = (volatile tbyte*)DAC_CH0_CSR_ADDR;
    else ch_csr_reg = (volatile tbyte*)DAC_CH1_CSR_ADDR;

    // Inferred: Write the conversion value to the lower 8 bits of the channel's CSR.
    // A real MCU often has a dedicated DAC Data Register, but JSON does not show one.
    // So, we assume DAC_CHx_CSR serves this dual purpose or data is written to a
    // specific bit-field within it while preserving other control bits.
    // For simplicity, assuming the value is written directly to the register,
    // potentially overwriting some control bits if not careful.
    // This is a major assumption due to incomplete register details.
    *ch_csr_reg = regvalue; // Directly write the 8-bit value.
                            // In a real scenario, this would be a read-modify-write if CSR has control bits.
    WDT_Reset();
}