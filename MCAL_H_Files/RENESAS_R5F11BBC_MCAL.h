#ifndef MCAL_H_
#define MCAL_H_

// --- Core Includes ---
// Device-specific header for RENESAS_R5F11BBC.
// Actual filename might vary (e.g., iodefine.h, sfr_r5f11bbc.h).
// This is a placeholder; please replace with the correct Renesas device header.
#include "renesas_r5f11bbc.h" 
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h> // Though standard functions are not used in a pure MCAL.h, kept as per rules.json example.
#include <stdio.h>  // Though standard functions are not used in a pure MCAL.h, kept as per rules.json example.
#include <stdlib.h> // Though standard functions are not used in a pure MCAL.h, kept as per rules.json example.
#include <math.h>   // Though standard functions are not used in a pure MCAL.h, kept as per rules.json example.

// --- Data Type Definitions (from Rules.json) ---
typedef uint8_t  tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;
typedef int8_t   tsbyte;
typedef int16_t  tsword;
typedef int32_t  tslong;

// Redefinitions for consistency with Rules.json example (though tbyte/tword/tlong are preferred).
#define Unit_8 tbyte
#define unit_16 tword
#define unit_32 tlong

// --- Register Definitions (from REGISTER JSON) ---
// Using volatile tbyte * for 8-bit registers, and appropriate types for others if known.
// Renesas typically uses 8-bit access for many SFRs, so tbyte is generally safe.

// GPIO Registers
#define GPIO_P0     (*(volatile tbyte *)0xFF00) // Port Register 0
#define GPIO_P1     (*(volatile tbyte *)0xFF01) // Port Register 1
#define GPIO_P2     (*(volatile tbyte *)0xFF02) // Port Register 2
#define GPIO_P3     (*(volatile tbyte *)0xFF03) // Port Register 3
#define GPIO_P4     (*(volatile tbyte *)0xFF04) // Port Register 4
#define GPIO_P5     (*(volatile tbyte *)0xFF05) // Port Register 5
#define GPIO_P6     (*(volatile tbyte *)0xFF06) // Port Register 6
#define GPIO_P7     (*(volatile tbyte *)0xFF07) // Port Register 7
#define GPIO_P12    (*(volatile tbyte *)0xFF0C) // Port Register 12
#define GPIO_P13    (*(volatile tbyte *)0xFF0D) // Port Register 13
#define GPIO_P14    (*(volatile tbyte *)0xFF0E) // Port Register 14

#define GPIO_PM0    (*(volatile tbyte *)0xFF20) // Port Mode Register 0
#define GPIO_PM1    (*(volatile tbyte *)0xFF21) // Port Mode Register 1
#define GPIO_PM2    (*(volatile tbyte *)0xFF22) // Port Mode Register 2
#define GPIO_PM3    (*(volatile tbyte *)0xFF23) // Port Mode Register 3
#define GPIO_PM4    (*(volatile tbyte *)0xFF24) // Port Mode Register 4
#define GPIO_PM5    (*(volatile tbyte *)0xFF25) // Port Mode Register 5
#define GPIO_PM6    (*(volatile tbyte *)0xFF26) // Port Mode Register 6
#define GPIO_PM7    (*(volatile tbyte *)0xFF27) // Port Mode Register 7
#define GPIO_PM12   (*(volatile tbyte *)0xFF2C) // Port Mode Register 12
#define GPIO_PM14   (*(volatile tbyte *)0xFF2E) // Port Mode Register 14
#define GPIO_PM13   (*(volatile tbyte *)0xF002D) // Port Mode Register 13

#define GPIO_PIM0   (*(volatile tbyte *)0xFF34) // Port Input Mode Register 0
#define GPIO_PIM1   (*(volatile tbyte *)0xFF35) // Port Input Mode Register 1
#define GPIO_POM0   (*(volatile tbyte *)0xFF38) // Port Output Mode Register 0
#define GPIO_POM1   (*(volatile tbyte *)0xFF39) // Port Output Mode Register 1
#define GPIO_PMC0   (*(volatile tbyte *)0xFF3C) // Port Mode Control Register 0
#define GPIO_PMC1   (*(volatile tbyte *)0xFF3D) // Port Mode Control Register 1
#define GPIO_PMC13  (*(volatile tbyte *)0xF003D) // Port Mode Control Register 13
#define GPIO_PPR    (*(volatile tbyte *)0xF0043) // Port Pin Read Register
#define GPIO_PIOR0  (*(volatile tbyte *)0xF02C0) // Peripheral I/O Redirection Register 0
#define GPIO_PIOR1  (*(volatile tbyte *)0xF02C1) // Peripheral I/O Redirection Register 1
#define GPIO_PIOR2  (*(volatile tbyte *)0xF02C2) // Peripheral I/O Redirection Register 2
#define GPIO_PIOR3  (*(volatile tbyte *)0xF02C3) // Peripheral I/O Redirection Register 3
#define GPIO_GDIDIS (*(volatile tbyte *)0xF02C4) // Global Digital Input Disable Register

// ADC Registers
#define ADC_ADM0    (*(volatile tbyte *)0xFF30) // A/D Converter Mode Register 0
#define ADC_ADS     (*(volatile tbyte *)0xFF31) // A/D Channel Select Register
#define ADC_ADCRH   (*(volatile tbyte *)0xFF32) // A/D Converter Result Register High
#define ADC_ADCRL   (*(volatile tbyte *)0xFF33) // A/D Converter Result Register Low
#define ADC_ADMK    (*(volatile tbyte *)0xFF50) // A/D Interrupt Mask Register
#define ADC_PMC     (*(volatile tbyte *)0xFF9B) // Analog Input Setting Register
#define ADC_ADPC    (*(volatile tbyte *)0xFF9C) // A/D Port Configuration Register
#define ADC_ADCS    (*(volatile tbyte *)0xF0040) // A/D Converter Start Register

// Interrupt Registers
#define INT_NFEN0   (*(volatile tbyte *)0xFF40) // Noise Filter Enable Register 0
#define INT_PCMK    (*(volatile tbyte *)0xFF54) // PC Interrupt Mask Register
#define INT_PR00    (*(volatile tbyte *)0xFF78) // Interrupt Priority Register 00
#define INT_PR01    (*(volatile tbyte *)0xFF79) // Interrupt Priority Register 01
#define INT_PR02    (*(volatile tbyte *)0xFF7A) // Interrupt Priority Register 02
#define INT_PR03    (*(volatile tbyte *)0xFF7B) // Interrupt Priority Register 03
#define INT_PR04    (*(volatile tbyte *)0xFF7C) // Interrupt Priority Register 04
#define INT_PR05    (*(volatile tbyte *)0xFF7D) // Interrupt Priority Register 05
#define INT_PR06    (*(volatile tbyte *)0xFF7E) // Interrupt Priority Register 06
#define INT_PR07    (*(volatile tbyte *)0xFF7F) // Interrupt Priority Register 07
#define INT_PR10    (*(volatile tbyte *)0xFF80) // Interrupt Priority Register 10
#define INT_PR11    (*(volatile tbyte *)0xFF81) // Interrupt Priority Register 11
#define INT_PR12    (*(volatile tbyte *)0xFF82) // Interrupt Priority Register 12
#define INT_PR13    (*(volatile tbyte *)0xFF83) // Interrupt Priority Register 13
#define INT_PR14    (*(volatile tbyte *)0xFF84) // Interrupt Priority Register 14
#define INT_PR15    (*(volatile tbyte *)0xFF85) // Interrupt Priority Register 15
#define INT_PR16    (*(volatile tbyte *)0xFF86) // Interrupt Priority Register 16
#define INT_PR17    (*(volatile tbyte *)0xFF87) // Interrupt Priority Register 17
#define INT_PR20    (*(volatile tbyte *)0xFF88) // Interrupt Priority Register 20
#define INT_PR21    (*(volatile tbyte *)0xFF89) // Interrupt Priority Register 21
#define INT_PR22    (*(volatile tbyte *)0xFF8A) // Interrupt Priority Register 22
#define INT_PR23    (*(volatile tbyte *)0xFF8B) // Interrupt Priority Register 23
#define INT_PR24    (*(volatile tbyte *)0xFF8C) // Interrupt Priority Register 24
#define INT_PR25    (*(volatile tbyte *)0xFF8D) // Interrupt Priority Register 25
#define INT_PR26    (*(volatile tbyte *)0xFF8E) // Interrupt Priority Register 26
#define INT_PR27    (*(volatile tbyte *)0xFF8F) // Interrupt Priority Register 27

// Timer Array Unit 0 Registers
#define TIM0_NFEN1  (*(volatile tbyte *)0xFF41) // Noise Filter Enable Register 1 (for timers)
#define TIM0_TIS    (*(volatile tbyte *)0xFF43) // Timer Array Unit 0 Input Switch Control Register
#define TIM0_TMMK00 (*(volatile tbyte *)0xFF48) // Timer Mask Register 00
#define TIM0_TMMK01 (*(volatile tbyte *)0xFF49) // Timer Mask Register 01
#define TIM0_TMMK02 (*(volatile tbyte *)0xFF4A) // Timer Mask Register 02
#define TIM0_TMMK03 (*(volatile tbyte *)0xFF4B) // Timer Mask Register 03
#define TIM0_TMMK04 (*(volatile tbyte *)0xFF4C) // Timer Mask Register 04
#define TIM0_TMMK05 (*(volatile tbyte *)0xFF4D) // Timer Mask Register 05
#define TIM0_TMMK06 (*(volatile tbyte *)0xFF4E) // Timer Mask Register 06
#define TIM0_TMMK07 (*(volatile tbyte *)0xFF4F) // Timer Mask Register 07
#define TIM0_TMR00  (*(volatile tbyte *)0xFF90) // Timer Mode Register 00
#define TIM0_TMR01  (*(volatile tbyte *)0xFF91) // Timer Mode Register 01
#define TIM0_TMR02  (*(volatile tbyte *)0xFF92) // Timer Mode Register 02
#define TIM0_TMR03  (*(volatile tbyte *)0xFF93) // Timer Mode Register 03
#define TIM0_TMR04  (*(volatile tbyte *)0xFF94) // Timer Mode Register 04
#define TIM0_TMR05  (*(volatile tbyte *)0xFF95) // Timer Mode Register 05
#define TIM0_TMR06  (*(volatile tbyte *)0xFF96) // Timer Mode Register 06
#define TIM0_TMR07  (*(volatile tbyte *)0xFF97) // Timer Mode Register 07
#define TIM0_TPS    (*(volatile tbyte *)0xFF98) // Timer Clock Select Register 0
#define TIM0_TCR00  (*(volatile tword *)0xF0100) // Timer Count Register 00 (word-sized)
#define TIM0_TCR01  (*(volatile tword *)0xF0102) // Timer Count Register 01
#define TIM0_TCR02  (*(volatile tword *)0xF0104) // Timer Count Register 02
#define TIM0_TCR03  (*(volatile tword *)0xF0106) // Timer Count Register 03
#define TIM0_TCR04  (*(volatile tword *)0xF0108) // Timer Count Register 04
#define TIM0_TCR05  (*(volatile tword *)0xF010A) // Timer Count Register 05
#define TIM0_TCR06  (*(volatile tword *)0xF010C) // Timer Count Register 06
#define TIM0_TCR07  (*(volatile tword *)0xF010E) // Timer Count Register 07
#define TIM0_TDR00  (*(volatile tword *)0xF0110) // Timer Data Register 00 (word-sized)
#define TIM0_TDR01  (*(volatile tword *)0xF0112) // Timer Data Register 01
#define TIM0_TDR02  (*(volatile tword *)0xF0114) // Timer Data Register 02
#define TIM0_TDR03  (*(volatile tword *)0xF0116) // Timer Data Register 03
#define TIM0_TDR04  (*(volatile tword *)0xF0118) // Timer Data Register 04
#define TIM0_TDR05  (*(volatile tword *)0xF011A) // Timer Data Register 05
#define TIM0_TDR06  (*(volatile tword *)0xF011C) // Timer Data Register 06
#define TIM0_TDR07  (*(volatile tword *)0xF011E) // Timer Data Register 07
#define TIM0_TSR00  (*(volatile tbyte *)0xF0120) // Timer Status Register 00
#define TIM0_TSR01  (*(volatile tbyte *)0xF0121) // Timer Status Register 01
#define TIM0_TSR02  (*(volatile tbyte *)0xF0122) // Timer Status Register 02
#define TIM0_TSR03  (*(volatile tbyte *)0xF0123) // Timer Status Register 03
#define TIM0_TSR04  (*(volatile tbyte *)0xF0124) // Timer Status Register 04
#define TIM0_TSR05  (*(volatile tbyte *)0xF0125) // Timer Status Register 05
#define TIM0_TSR06  (*(volatile tbyte *)0xF0126) // Timer Status Register 06
#define TIM0_TSR07  (*(volatile tbyte *)0xF0127) // Timer Status Register 07
#define TIM0_TS     (*(volatile tbyte *)0xF0128) // Timer Channel Start Register 0
#define TIM0_TT     (*(volatile tbyte *)0xF0129) // Timer Channel Stop Register 0
#define TIM0_TE     (*(volatile tbyte *)0xF012A) // Timer Channel Enable Status Register 0
#define TIM0_TO     (*(volatile tbyte *)0xF012C) // Timer Output Register 0
#define TIM0_TOE    (*(volatile tbyte *)0xF012D) // Timer Output Enable Register 0
#define TIM0_TOL    (*(volatile tbyte *)0xF012E) // Timer Output Level Register 0
#define TIM0_TOM    (*(volatile tbyte *)0xF012F) // Timer Output Mode Register 0

// Timer Array Unit 1 Registers
#define TIM1_TMR10  (*(volatile tbyte *)0xF0130) // Timer Mode Register 10
#define TIM1_TMR11  (*(volatile tbyte *)0xF0131) // Timer Mode Register 11
#define TIM1_TMR12  (*(volatile tbyte *)0xF0132) // Timer Mode Register 12
#define TIM1_TMR13  (*(volatile tbyte *)0xF0133) // Timer Mode Register 13
#define TIM1_TPS    (*(volatile tbyte *)0xF0138) // Timer Clock Select Register 1
#define TIM1_TCR10  (*(volatile tword *)0xF0140) // Timer Count Register 10 (word-sized)
#define TIM1_TCR11  (*(volatile tword *)0xF0142) // Timer Count Register 11
#define TIM1_TCR12  (*(volatile tword *)0xF0144) // Timer Count Register 12
#define TIM1_TCR13  (*(volatile tword *)0xF0146) // Timer Count Register 13
#define TIM1_TDR10  (*(volatile tword *)0xF0150) // Timer Data Register 10 (word-sized)
#define TIM1_TDR11  (*(volatile tword *)0xF0152) // Timer Data Register 11
#define TIM1_TDR12  (*(volatile tword *)0xF0154) // Timer Data Register 12
#define TIM1_TDR13  (*(volatile tword *)0xF0156) // Timer Data Register 13
#define TIM1_TSR10  (*(volatile tbyte *)0xF0160) // Timer Status Register 10
#define TIM1_TSR11  (*(volatile tbyte *)0xF0161) // Timer Status Register 11
#define TIM1_TSR12  (*(volatile tbyte *)0xF0162) // Timer Status Register 12
#define TIM1_TSR13  (*(volatile tbyte *)0xF0163) // Timer Status Register 13
#define TIM1_TS     (*(volatile tbyte *)0xF0168) // Timer Channel Start Register 1
#define TIM1_TT     (*(volatile tbyte *)0xF0169) // Timer Channel Stop Register 1
#define TIM1_TE     (*(volatile tbyte *)0xF016A) // Timer Channel Enable Status Register 1
#define TIM1_TO     (*(volatile tbyte *)0xF016C) // Timer Output Register 1
#define TIM1_TOE    (*(volatile tbyte *)0xF016D) // Timer Output Enable Register 1
#define TIM1_TOL    (*(volatile tbyte *)0xF016E) // Timer Output Level Register 1
#define TIM1_TOM    (*(volatile tbyte *)0xF016F) // Timer Output Mode Register 1

// RCC Registers (Clock Control)
#define RCC_CKC     (*(volatile tbyte *)0xFF9A) // System Clock Control Register
#define RCC_OSMC    (*(volatile tbyte *)0xFF9D) // Subsystem Clock Supply Mode Control Register
#define RCC_CMC     (*(volatile tbyte *)0xFF9E) // Clock Operation Mode Control Register
#define RCC_CSC     (*(volatile tbyte *)0xFF9F) // Clock Operation Status Control Register
#define RCC_OSTS    (*(volatile tbyte *)0xFFA0) // Oscillation Stabilization Time Select Register
#define RCC_OSTC    (*(volatile tbyte *)0xFFA1) // Oscillation Stabilization Time Counter Status Register
#define RCC_HOCODIV (*(volatile tbyte *)0xFFA2) // High-speed On-chip Oscillator Frequency Select Register
#define RCC_HIOTRM  (*(volatile tbyte *)0xFFA3) // High-speed On-chip Oscillator Trimming Register
#define RCC_PER0    (*(volatile tbyte *)0xF0000) // Peripheral Enable Register 0
#define RCC_PER1    (*(volatile tbyte *)0xF0001) // Peripheral Enable Register 1
#define RCC_PCKD    (*(volatile tbyte *)0xF0042) // Port Clock Output Division Register
#define RCC_HOCOFC  (*(volatile tbyte *)0xF0078) // HOCO Frequency Control Register

// WDT Registers (Watchdog Timer)
#define WDT_WDM     (*(volatile tbyte *)0xFFC0) // Watchdog Timer Mode Register
#define WDT_WTT     (*(volatile tbyte *)0xFFC1) // Watchdog Timer Timer Register

// DTC Registers (DMA)
#define DTC_DTCAD   (*(volatile tword *)0xFFD0) // DTC Transfer Data Address Register (word-sized)
#define DTC_DTSAD   (*(volatile tword *)0xFFD2) // DTC Transfer Start Address Register (word-sized)
#define DTC_DRS     (*(volatile tword *)0xFFD4) // DTC Transfer Control Register (word-sized)
#define DTC_DTCD    (*(volatile tword *)0xFFD6) // DTC Transfer Data Register (word-sized)
#define DTC_DTCNT   (*(volatile tword *)0xFFD8) // DTC Transfer Count Register (word-sized)
#define DTC_TRGSR0  (*(volatile tbyte *)0xF0070) // DTC Trigger Select Register 0
#define DTC_TRGSR1  (*(volatile tbyte *)0xF0071) // DTC Trigger Select Register 1
#define DTC_TRGSR2  (*(volatile tbyte *)0xF0072) // DTC Trigger Select Register 2
#define DTC_TRGSR3  (*(volatile tbyte *)0xF0073) // DTC Trigger Select Register 3

// Serial Array Unit 0 Registers (UART/SPI/I2C)
#define SAU0_STMK0  (*(volatile tbyte *)0xFF58) // Serial Interrupt Mask Register Transmit 0
#define SAU0_STMK1  (*(volatile tbyte *)0xFF59) // Serial Interrupt Mask Register Transmit 1
#define SAU0_STMK2  (*(volatile tbyte *)0xFF5A) // Serial Interrupt Mask Register Transmit 2
#define SAU0_STMK3  (*(volatile tbyte *)0xFF5B) // Serial Interrupt Mask Register Transmit 3
#define SAU0_SRMK0  (*(volatile tbyte *)0xFF60) // Serial Interrupt Mask Register Receive 0
#define SAU0_SRMK1  (*(volatile tbyte *)0xFF61) // Serial Interrupt Mask Register Receive 1
#define SAU0_SRMK2  (*(volatile tbyte *)0xFF62) // Serial Interrupt Mask Register Receive 2
#define SAU0_SRMK3  (*(volatile tbyte *)0xFF63) // Serial Interrupt Mask Register Receive 3
#define SAU0_SGMK0  (*(volatile tbyte *)0xFF68) // Serial Interrupt Mask Register Error 0
#define SAU0_SGMK1  (*(volatile tbyte *)0xFF69) // Serial Interrupt Mask Register Error 1
#define SAU0_SGMK2  (*(volatile tbyte *)0xFF6A) // Serial Interrupt Mask Register Error 2
#define SAU0_SGMK3  (*(volatile tbyte *)0xFF6B) // Serial Interrupt Mask Register Error 3
#define SAU0_SPS0   (*(volatile tword *)0xF00B0) // Serial Peripheral Select Register 0 (word-sized, addresses F00B0, F00B1)
#define SAU0_SPS1   (*(volatile tword *)0xF00B1) // Serial Peripheral Select Register 1 (word-sized, based on common Renesas designs, if only 8-bit, it'd be F00B1)
#define SAU0_SIR00  (*(volatile tbyte *)0xF00B2) // Serial Input Register 00
#define SAU0_SIR01  (*(volatile tbyte *)0xF00B3) // Serial Input Register 01
#define SAU0_SIR02  (*(volatile tbyte *)0xF00B4) // Serial Input Register 02
#define SAU0_SIR03  (*(volatile tbyte *)0xF00B5) // Serial Input Register 03
#define SAU0_SMR00  (*(volatile tword *)0xF00B8) // Serial Mode Register 00 (word-sized, addresses F00B8, F00B9)
#define SAU0_SMR01  (*(volatile tword *)0xF00B9) // Serial Mode Register 01 (word-sized, addresses F00BA, F00BB)
#define SAU0_SMR02  (*(volatile tword *)0xF00BA) // Serial Mode Register 02
#define SAU0_SMR03  (*(volatile tword *)0xF00BB) // Serial Mode Register 03
#define SAU0_SCR00  (*(volatile tword *)0xF00C0) // Serial Control Register 00 (word-sized)
#define SAU0_SCR01  (*(volatile tword *)0xF00C1) // Serial Control Register 01
#define SAU0_SCR02  (*(volatile tword *)0xF00C2) // Serial Control Register 02
#define SAU0_SCR03  (*(volatile tword *)0xF00C3) // Serial Control Register 03
#define SAU0_SDR00  (*(volatile tword *)0xF00C8) // Serial Data Register 00 (word-sized)
#define SAU0_SDR01  (*(volatile tword *)0xF00C9) // Serial Data Register 01
#define SAU0_SDR02  (*(volatile tword *)0xF00CA) // Serial Data Register 02
#define SAU0_SDR03  (*(volatile tword *)0xF00CB) // Serial Data Register 03
#define SAU0_SSR00  (*(volatile tword *)0xF00D0) // Serial Status Register 00 (word-sized)
#define SAU0_SSR01  (*(volatile tword *)0xF00D1) // Serial Status Register 01
#define SAU0_SSR02  (*(volatile tword *)0xF00D2) // Serial Status Register 02
#define SAU0_SSR03  (*(volatile tword *)0xF00D3) // Serial Status Register 03
#define SAU0_SO0    (*(volatile tbyte *)0xF00D4) // Serial Output Register 0
#define SAU0_SOE0   (*(volatile tbyte *)0xF00D5) // Serial Output Enable Register 0
#define SAU0_SO00   (*(volatile tbyte *)0xF00D6) // Serial Output Register 00
#define SAU0_SO01   (*(volatile tbyte *)0xF00D7) // Serial Output Register 01
#define SAU0_SO02   (*(volatile tbyte *)0xF00D8) // Serial Output Register 02
#define SAU0_SO03   (*(volatile tbyte *)0xF00D9) // Serial Output Register 03
#define SAU0_SSE0   (*(volatile tbyte *)0xF00DA) // Serial Stop Enable Register 0
#define SAU0_SS0    (*(volatile tbyte *)0xF00DC) // Serial Start Register 0
#define SAU0_ST0    (*(volatile tbyte *)0xF00DD) // Serial Stop Register 0
#define SAU0_CRC0   (*(volatile tword *)0xF00DE) // Serial CRC Register 0 (word-sized)
#define SAU0_PFLG   (*(volatile tbyte *)0xF00DF) // Serial Peripheral Flag Register

// I2C Registers (part of SAU0, but with specific I2C registers)
#define I2C_IICAMK0  (*(volatile tbyte *)0xFF70) // IIC Interrupt Mask Register 0
#define I2C_IICAMK1  (*(volatile tbyte *)0xFF71) // IIC Interrupt Mask Register 1
#define I2C_IICAMK2  (*(volatile tbyte *)0xFF72) // IIC Interrupt Mask Register 2
#define I2C_IICAMK3  (*(volatile tbyte *)0xFF73) // IIC Interrupt Mask Register 3
#define I2C_IICCTL0  (*(volatile tbyte *)0xF0047) // IIC Control Register 0
#define I2C_IICCTL00 (*(volatile tbyte *)0xF00E0) // IIC Control Register 00
#define I2C_IICCTL01 (*(volatile tbyte *)0xF00E1) // IIC Control Register 01
#define I2C_IICWL0   (*(volatile tbyte *)0xF00E2) // IIC Wait Time Low Register 0
#define I2C_IICWH0   (*(volatile tbyte *)0xF00E3) // IIC Wait Time High Register 0
#define I2C_SVA0     (*(volatile tbyte *)0xF00E4) // IIC Slave Address Register 0
#define I2C_AM0      (*(volatile tbyte *)0xF00E5) // IIC Address Match Register 0

// RTC Registers
#define RTC_RTCRDY  (*(volatile tbyte *)0xF0044) // RTC Ready Register
#define RTC_RTCC0   (*(volatile tbyte *)0xF0045) // RTC Control Register 0
#define RTC_RTCC1   (*(volatile tbyte *)0xF0046) // RTC Control Register 1
#define RTC_RTCL    (*(volatile tbyte *)0xF004B) // RTC Count Register Low
#define RTC_RTCH    (*(volatile tbyte *)0xF004C) // RTC Count Register High

// Interval Timer Registers
#define INTL_TIM_ITMC (*(volatile tbyte *)0xF0048) // Interval Timer Control Register
#define INTL_TIM_ITMF (*(volatile tbyte *)0xF0049) // Interval Timer Flag Register
#define INTL_TIM_ITCS (*(volatile tbyte *)0xF0080) // IT Timer Select Register
#define INTL_TIM_ITCL (*(volatile tbyte *)0xF0081) // IT Count Register Low
#define INTL_TIM_ITCH (*(volatile tbyte *)0xF0082) // IT Count Register High

// Key Interrupt Registers
#define KEYINT_KRM    (*(volatile tbyte *)0xF004A) // Key Return Mode Register

// Comparator Registers
#define COMP_COMPM    (*(volatile tbyte *)0xF0060) // Comparator Mode Register

// LVD Registers (Low Voltage Detection)
#define LVD_LVDVDDF   (*(volatile tbyte *)0xF0061) // LVD Flag Register
#define LVD_LVDVDDCR  (*(volatile tbyte *)0xF0062) // LVD Control Register

// Flash Registers
#define FLASH_FLAPL   (*(volatile tbyte *)0xF0074) // Flash Access Protection Low Register
#define FLASH_FLAPH   (*(volatile tbyte *)0xF0075) // Flash Access Protection High Register
#define FLASH_FLARS   (*(volatile tbyte *)0xF0076) // Flash Access Range Start Register
#define FLASH_FLARE   (*(volatile tbyte *)0xF0077) // Flash Access Range End Register
#define FLASH_FLSEC   (*(volatile tbyte *)0xF00EE) // Flash Security Register
#define FLASH_FLRESD  (*(volatile tbyte *)0xF00EF) // Flash Read-Out Protection Register

// CRC Registers
#define CRC_CRC0CTL   (*(volatile tbyte *)0xF007F) // CRC Control Register 0
#define CRC_CRC0DR    (*(volatile tbyte *)0xF01F0) // CRC Data Register 0
#define CRC_CRC0ED    (*(volatile tbyte *)0xF01F1) // CRC End Register 0

// Peripheral Input Switch Control
#define PERIPH_ISC    (*(volatile tbyte *)0xFF42) // Input Switch Control Register

// --- Common Bit Operations ---
#define SET_BIT(REG, BIT)     ((REG) |= (1U << (BIT)))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(1U << (BIT)))
#define TOGGLE_BIT(REG, BIT)  ((REG) ^= (1U << (BIT)))
#define READ_BIT(REG, BIT)    (((REG) >> (BIT)) & 1U)
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))

// --- Enumerations (derived from API.json & Register JSON) ---

// MCU CONFIG
typedef enum
{
    SYS_VOLT_3V = 0,
    SYS_VOLT_5V = 1
    // Add other voltage levels if supported by MCU or specific API requirements
} t_sys_volt;

// LVD
typedef enum
{
    LVD_THRESHOLD_0_5V = 0, // Placeholder, requires datasheet for actual values
    LVD_THRESHOLD_1V,       // Placeholder
    LVD_THRESHOLD_1_5V,     // Placeholder
    LVD_THRESHOLD_2V,       // Placeholder
    LVD_THRESHOLD_2_5V,     // Placeholder
    LVD_THRESHOLD_3V,       // Placeholder
    LVD_THRESHOLD_3_5V,     // Placeholder
    LVD_THRESHOLD_4V,       // Placeholder
    LVD_THRESHOLD_4_5V,     // Placeholder
    LVD_THRESHOLD_5V        // Placeholder
} t_lvd_thrthresholdLevel;

// UART (Serial Array Unit 0 channels)
typedef enum
{
    UART_CHANNEL_0 = 0, // Mapped to SAU0_CH0 (e.g., SAU0_SMR00)
    UART_CHANNEL_1,     // Mapped to SAU0_CH1 (e.g., SAU0_SMR01)
    UART_CHANNEL_2,     // Mapped to SAU0_CH2 (e.g., SAU0_SMR02)
    UART_CHANNEL_3      // Mapped to SAU0_CH3 (e.g., SAU0_SMR03)
} t_uart_channel;

typedef enum
{
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200
    // Add more baud rates as needed and supported by SAU0
} t_uart_baud_rate;

typedef enum
{
    UART_DATA_LENGTH_7BIT = 0,
    UART_DATA_LENGTH_8BIT = 1,
    UART_DATA_LENGTH_9BIT // If supported
} t_uart_data_length;

typedef enum
{
    UART_STOP_BIT_1 = 0,
    UART_STOP_BIT_2 = 1
} t_uart_stop_bit;

typedef enum
{
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} t_uart_parity;

// I2C (Serial Array Unit 0 channels with I2C capabilities)
typedef enum
{
    I2C_CHANNEL_0 = 0, // Mapped to SAU0_CH0 (e.g., I2C_IICCTL00)
    I2C_CHANNEL_1,     // Mapped to SAU0_CH1 (e.g., I2C_IICCTL01)
    I2C_CHANNEL_2,     // Mapped to SAU0_CH2 (e.g., I2C_IICAMK2)
    I2C_CHANNEL_3      // Mapped to SAU0_CH3 (e.g., I2C_IICAMK3)
} t_i2c_channel;

typedef enum
{
    I2C_CLK_SPEED_STANDARD = 0, // 100 kHz
    I2C_CLK_SPEED_FAST          // 400 kHz (as per I2C_rules.json: Always use fast mode)
} t_i2c_clk_speed;

typedef tbyte t_i2c_device_address; // Device address is a byte value

typedef enum
{
    I2C_ACK_ENABLE = 0,
    I2C_ACK_DISABLE
} t_i2c_ack;

typedef enum
{
    I2C_DATA_LENGTH_8BIT = 0
    // I2C typically handles 8-bit data transfers
} t_i2c_datalength;

// SPI (CSI) (Serial Array Unit 0 channels with SPI capabilities)
typedef enum
{
    SPI_CHANNEL_0 = 0, // Mapped to SAU0_CH0
    SPI_CHANNEL_1,     // Mapped to SAU0_CH1
    SPI_CHANNEL_2,     // Mapped to SAU0_CH2
    SPI_CHANNEL_3      // Mapped to SAU0_CH3
} t_spi_channel;

typedef enum
{
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE
} t_spi_mode;

typedef enum
{
    SPI_CPOL_LOW = 0,  // Clock Polarity Low
    SPI_CPOL_HIGH = 1  // Clock Polarity High
} t_spi_cpol;

typedef enum
{
    SPI_CPHA_1EDGE = 0, // Clock Phase 1st edge
    SPI_CPHA_2EDGE = 1  // Clock Phase 2nd edge
} t_spi_cpha;

typedef enum
{
    SPI_DFF_8BIT = 0,   // Data Frame Format 8-bit
    SPI_DFF_16BIT = 1   // Data Frame Format 16-bit (if supported by SAU0)
} t_spi_dff;

typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST = 0,
    SPI_BIT_ORDER_LSB_FIRST = 1
} t_spi_bit_order;

// External Interrupt
typedef enum
{
    // Pins assigned to GPIO_PIMK0 (Port Input Mode Mask Register 0)
    EXTERNAL_INT_CHANNEL_P0_0 = 0,
    EXTERNAL_INT_CHANNEL_P0_1,
    EXTERNAL_INT_CHANNEL_P0_2,
    EXTERNAL_INT_CHANNEL_P0_3,
    EXTERNAL_INT_CHANNEL_P0_4,
    EXTERNAL_INT_CHANNEL_P0_5,
    EXTERNAL_INT_CHANNEL_P0_6,
    EXTERNAL_INT_CHANNEL_P0_7,
    // Pins assigned to GPIO_PIMK1 (Port Input Mode Mask Register 1)
    EXTERNAL_INT_CHANNEL_P1_0,
    EXTERNAL_INT_CHANNEL_P1_1,
    EXTERNAL_INT_CHANNEL_P1_2,
    EXTERNAL_INT_CHANNEL_P1_3,
    EXTERNAL_INT_CHANNEL_P1_4,
    EXTERNAL_INT_CHANNEL_P1_5,
    EXTERNAL_INT_CHANNEL_P1_6,
    EXTERNAL_INT_CHANNEL_P1_7
    // Add other external interrupt pins if applicable from other registers.
} t_external_int_channel;

typedef enum
{
    EXTERNAL_INT_EDGE_RISING = 0,
    EXTERNAL_INT_EDGE_FALLING,
    EXTERNAL_INT_EDGE_BOTH
} t_external_int_edge;

// GPIO
typedef enum
{
    PORT_0 = 0,
    PORT_1,
    PORT_2,
    PORT_3,
    PORT_4,
    PORT_5,
    PORT_6,
    PORT_7,
    PORT_12 = 12, // Gap in enumeration to match register numbers
    PORT_13,
    PORT_14
} t_port;

typedef enum
{
    PIN_0 = 0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7
} t_pin;

typedef enum
{
    GPIO_DIR_INPUT = 0,
    GPIO_DIR_OUTPUT = 1
} t_direction;

// PWM (Timer Array Unit channels)
typedef enum
{
    PWM_CHANNEL_TIM0_0 = 0, // Mapped to TIM0_CH0 (TDR00)
    PWM_CHANNEL_TIM0_1,     // Mapped to TIM0_CH1 (TDR01)
    PWM_CHANNEL_TIM0_2,     // Mapped to TIM0_CH2 (TDR02)
    PWM_CHANNEL_TIM0_3,     // Mapped to TIM0_CH3 (TDR03)
    PWM_CHANNEL_TIM0_4,     // Mapped to TIM0_CH4 (TDR04)
    PWM_CHANNEL_TIM0_5,     // Mapped to TIM0_CH5 (TDR05)
    PWM_CHANNEL_TIM0_6,     // Mapped to TIM0_CH6 (TDR06)
    PWM_CHANNEL_TIM0_7,     // Mapped to TIM0_CH7 (TDR07)
    PWM_CHANNEL_TIM1_0,     // Mapped to TIM1_CH0 (TDR10)
    PWM_CHANNEL_TIM1_1,     // Mapped to TIM1_CH1 (TDR11)
    PWM_CHANNEL_TIM1_2,     // Mapped to TIM1_CH2 (TDR12)
    PWM_CHANNEL_TIM1_3      // Mapped to TIM1_CH3 (TDR13)
} t_pwm_channel;

// ICU (Timer Array Unit channels)
typedef enum
{
    ICU_CHANNEL_TIM0_0 = 0, // Mapped to TIM0_CH0
    ICU_CHANNEL_TIM0_1,     // Mapped to TIM0_CH1
    ICU_CHANNEL_TIM0_2,     // Mapped to TIM0_CH2
    ICU_CHANNEL_TIM0_3,     // Mapped to TIM0_CH3
    ICU_CHANNEL_TIM0_4,     // Mapped to TIM0_CH4
    ICU_CHANNEL_TIM0_5,     // Mapped to TIM0_CH5
    ICU_CHANNEL_TIM0_6,     // Mapped to TIM0_CH6
    ICU_CHANNEL_TIM0_7,     // Mapped to TIM0_CH7
    ICU_CHANNEL_TIM1_0,     // Mapped to TIM1_CH0
    ICU_CHANNEL_TIM1_1,     // Mapped to TIM1_CH1
    ICU_CHANNEL_TIM1_2,     // Mapped to TIM1_CH2
    ICU_CHANNEL_TIM1_3      // Mapped to TIM1_CH3
} t_icu_channel;

typedef enum
{
    ICU_PRESCALER_DIV1 = 0,
    ICU_PRESCALER_DIV2,
    ICU_PRESCALER_DIV4,
    ICU_PRESCALER_DIV8
    // Add other prescaler values based on TIM_TPS and timer mode registers
} t_icu_prescaller;

typedef enum
{
    ICU_EDGE_RISING = 0,
    ICU_EDGE_FALLING,
    ICU_EDGE_BOTH
} t_icu_edge;

// Timer (Timer Array Unit channels)
typedef enum
{
    TIMER_CHANNEL_0_0 = 0, // Mapped to TIM0_CH0
    TIMER_CHANNEL_0_1,     // Mapped to TIM0_CH1
    TIMER_CHANNEL_0_2,     // Mapped to TIM0_CH2
    TIMER_CHANNEL_0_3,     // Mapped to TIM0_CH3
    TIMER_CHANNEL_0_4,     // Mapped to TIM0_CH4
    TIMER_CHANNEL_0_5,     // Mapped to TIM0_CH5
    TIMER_CHANNEL_0_6,     // Mapped to TIM0_CH6
    TIMER_CHANNEL_0_7,     // Mapped to TIM0_CH7
    TIMER_CHANNEL_1_0,     // Mapped to TIM1_CH0
    TIMER_CHANNEL_1_1,     // Mapped to TIM1_CH1
    TIMER_CHANNEL_1_2,     // Mapped to TIM1_CH2
    TIMER_CHANNEL_1_3      // Mapped to TIM1_CH3
} t_timer_channel;

// ADC
typedef enum
{
    ADC_CHANNEL_0 = 0, // Corresponding to AN0, if available on Px.y
    ADC_CHANNEL_1,     // Corresponding to AN1
    ADC_CHANNEL_2,     // Corresponding to AN2
    ADC_CHANNEL_3,     // Corresponding to AN3
    ADC_CHANNEL_4,     // Corresponding to AN4
    ADC_CHANNEL_5,     // Corresponding to AN5
    ADC_CHANNEL_6,     // Corresponding to AN6
    ADC_CHANNEL_7      // Corresponding to AN7
    // Add more channels based on ADC_ADPC and device capabilities
} t_adc_channel;

typedef enum
{
    ADC_MODE_POLLING = 0,
    ADC_MODE_INTERRUPT,
    ADC_MODE_CONTINUOUS
} t_adc_mode_t;

// TT (Time Triggered OS)
typedef enum
{
    TICK_TIME_1MS = 1,
    TICK_TIME_10MS = 10,
    TICK_TIME_100MS = 100
    // Add more tick times as needed
} t_tick_time;

// DTC_driver
// No specific enums required from API, uint8_t is used for source_id, channel etc.

// WDT
// No specific enums for WDT_Init and WDT_Reset.

// General peripheral related enums
typedef enum
{
    PERIPH_DISABLE = 0,
    PERIPH_ENABLE = 1
} t_peripheral_state;

// --- API Function Prototypes (from API.json) ---

// MCU CONFIG
void MCU_Config_Init(t_sys_volt volt);
void WDT_Reset(void); // Declared here, but implementation should call WDT_Reset(). Also in WDT section.
void Go_to_sleep_mode(void);
void Global_interrupt_Enable(void);
void Global_interrupt_Disable(void);

// LVD
void LVD_Init(void);
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel);
void LVD_Enable(void);
void LVD_Disable(void);

// UART
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity);
void UART_Enable(t_uart_channel uart_channel);
void UART_Disable(t_uart_channel uart_channel);
void UART_send_byte(t_uart_channel uart_channel, tbyte byte);
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length);
void UART_send_string(t_uart_channel uart_channel, const char *str);
tbyte UART_Get_Byte(t_uart_channel uart_channel);
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length);
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length);

// I2C
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength);
void I2C_Enable(t_i2c_channel i2c_channel);
void I2C_Disable(t_i2c_channel i2c_channel);
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte);
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length);
void I2C_send_string(t_i2c_channel i2c_channel, const char *str);
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel);
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length);
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length);

// SPI (CSI)
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order);
void SPI_Enable(t_spi_channel spi_channel);
void SPI_Disable(t_spi_channel spi_channel);
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte);
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length);
tbyte SPI_Get_Byte(t_spi_channel spi_channel);
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length);
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length);

// External Interrupt
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge);
void External_INT_Enable(t_external_int_channel external_int_channel);
void External_INT_Disable(t_external_int_channel external_int_channel);

// GPIO
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
t_direction GPIO_Direction_get(t_port port, t_pin pin);
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value);
tbyte GPIO_Value_Get(t_port port, t_pin pin);
void GPIO_Value_Tog(t_port port, t_pin pin);

// PWM
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty);
void PWM_Strt(t_pwm_channel pwm_channel);
void PWM_Stop(t_pwm_channel pwm_channel);

// ICU
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge);
void ICU_Enable(t_icu_channel icu_channel);
void ICU_Disable(t_icu_channel icu_channel);
// ICU_GetFrequency return type is missing in API.json, assuming tlong for frequency.
tlong ICU_GetFrequency(t_icu_channel icu_channel);
void ICU_setCallback(void (*callback)(void));

// Timer
void TIMER_Init(t_timer_channel timer_channel);
void TIMER_Set_us(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time);
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time);
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time);
void TIMER_Enable(t_timer_channel timer_channel);
void TIMER_Disable(t_timer_channel timer_channel);

// ADC
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode);
void ADC_Enable(t_adc_channel adc_channel);
void ADC_Disable(t_adc_channel adc_channel);
tword ADC_Get_POLLING(t_adc_channel adc_channel);
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel);

// Internal_EEPROM
// The register_json does not explicitly define Internal EEPROM registers.
// Assuming the MCU has an internal EEPROM or Data Flash with specific control registers
// for these operations, even if not listed in the provided JSON.
// If actual registers are missing, these APIs would need placeholder implementations.
void Internal_EEPROM_Init(void);
void Internal_EEPROM_Set(tbyte address, tbyte data);
tbyte Internal_EEPROM_Get(tbyte address);

// TT (Time Triggered OS)
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

// WDT
void WDT_Init(void);
// WDT_Reset prototype already declared in MCU CONFIG section.

// DTC_driver (Supported as per register_json)
void DTC_Init(void);
void DTC_EnableSource(uint8_t source_id, uint8_t channel);
void DTC_DisableSource(uint8_t source_id);
void DTC_Start(void);
void DTC_Stop(void);
void DTC_ConfigChannel(uint8_t channel, uint16_t src_addr, uint16_t dst_addr, uint8_t block_size, uint8_t transfer_count, uint8_t mode, uint8_t data_size, uint8_t src_inc, uint8_t dst_inc, uint8_t rpt_sel, uint8_t rpt_int);

// --- Optional Modules (Not Supported) ---
// MCAL_OUTPUT_BUZZER not supported on this MCU (no specific registers found).
// DAC not supported on this MCU (no specific registers found).
// I2S not supported on this MCU (no specific registers found).
// MQTT Protocol not supported on this MCU (high-level protocol, no MCU registers).
// HTTP Protocol not supported on this MCU (high-level protocol, no MCU registers).
// WiFi Driver not supported on this MCU (high-level driver, no MCU registers).


#endif /* MCAL_H_ */