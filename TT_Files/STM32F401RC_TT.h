#ifndef TT_H
#define TT_H

#include <stdint.h> // global_includes as per Rules.json

// --- Global Type Definitions (from Rules.json -> TT_general_rules -> data_types) ---
typedef uint32_t t_tick_time; // Unsigned integer representing milliseconds for initial tick setup
typedef uint8_t  tbyte;       // Unsigned 8-bit integer for task indexes
typedef uint16_t tword;       // Unsigned 16-bit integer for time values (periods, delays in ticks)

// --- Module Configuration ---
#define TT_NUMBER_TASKS      10U // Maximum number of tasks the scheduler can manage

// --- Peripheral Register Definitions (based on REGISTER_JSON and STM32F401RC specific knowledge) ---

// RCC Registers (Clock Control)
#define RCC_BASE            0x40023800UL
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED0[4];      // Gap from 0x1C to 0x30
  volatile uint32_t AHB1ENR;   // 0x40023830
  volatile uint32_t AHB2ENR;   // 0x40023834
  volatile uint32_t APB1ENR;   // 0x40023838 (Relevant for TIM2)
  volatile uint32_t APB2ENR;   // 0x4002383C
  uint32_t RESERVED1[4];      // Gap from 0x3C to 0x50
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t RESERVED2[2];      // Gap from 0x5C to 0x70
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t RESERVED3[2];      // Gap from 0x74 to 0x80
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  uint32_t RESERVED4[1];      // Gap from 0x84 to 0x8C
  volatile uint32_t DCKCFGR;
} RCC_TypeDef;
#define RCC                 ((RCC_TypeDef *) RCC_BASE)

// TIM2 Registers (General Purpose Timer for scheduling)
#define TIM2_BASE           0x40000000UL
typedef struct {
  volatile uint32_t CR1;    // 0x40000000 (Control register 1)
  volatile uint32_t CR2;    // 0x40000004 (Control register 2)
  volatile uint32_t SMCR;   // 0x40000008 (Slave mode control register)
  volatile uint32_t DIER;   // 0x4000000C (DMA/Interrupt enable register)
  volatile uint32_t SR;     // 0x40000010 (Status register)
  volatile uint32_t EGR;    // 0x40000014 (Event generation register)
  volatile uint32_t CCMR1;  // 0x40000018 (Capture/compare mode register 1)
  volatile uint32_t CCMR2;  // 0x4000001C (Capture/compare mode register 2)
  volatile uint32_t CCER;   // 0x40000020 (Capture/compare enable register)
  volatile uint32_t CNT;    // 0x40000024 (Counter)
  volatile uint32_t PSC;    // 0x40000028 (Prescaler)
  volatile uint32_t ARR;    // 0x4000002C (Auto-reload register)
  volatile uint32_t RESERVED0; // 0x40000030, placeholder to match addresses
  volatile uint32_t CCR1;   // 0x40000034 (Capture/compare register 1)
  volatile uint32_t CCR2;   // 0x40000038 (Capture/compare register 2)
  volatile uint32_t CCR3;   // 0x4000003C (Capture/compare register 3)
  volatile uint32_t CCR4;   // 0x40000040 (Capture/compare register 4)
  volatile uint32_t RESERVED1; // 0x40000044, placeholder to match addresses
  volatile uint32_t DCR;    // 0x40000048 (DMA control register)
  volatile uint32_t DMAR;   // 0x4000004C (DMA address for full transfer)
  volatile uint32_t OR;     // 0x40000050 (Option register)
} TIM2_TypeDef;
#define TIM2                 ((TIM2_TypeDef *) TIM2_BASE)

// IWDG Registers (Independent Watchdog - not explicitly in REGISTER_JSON, but required by rules for STM32F401RC)
#define IWDG_BASE            (0x40003000UL)
typedef struct {
  volatile uint32_t KR;   // Key register
  volatile uint32_t PR;   // Prescaler register
  volatile uint32_t RLR;  // Reload register
  volatile uint32_t SR;   // Status register
} IWDG_TypeDef;
#define IWDG                 ((IWDG_TypeDef *) IWDG_BASE)

// NVIC Registers (Nested Vectored Interrupt Controller - not explicitly in REGISTER_JSON, but required by rules)
// Simplified structure for relevant ISER/ICPR registers, based on ARM Cortex-M architecture.
#define NVIC_BASE            (0xE000E100UL) // ISER[0] address
typedef struct {
  volatile uint32_t ISER[8]; // Interrupt Set-Enable Registers (ISER0-ISER7)
  uint32_t          RESERVED0[24]; // Gap to ICER registers
  volatile uint32_t ICER[8]; // Interrupt Clear-Enable Registers (ICER0-ICER7)
  uint32_t          RESERVED1[24]; // Gap to ISPR registers
  volatile uint32_t ISPR[8]; // Interrupt Set-Pending Registers (ISPR0-ISPR7)
  uint32_t          RESERVED2[24]; // Gap to ICPR registers
  volatile uint32_t ICPR[8]; // Interrupt Clear-Pending Registers (ICPR0-ICPR7)
  uint32_t          RESERVED3[24]; // Gap to IABR registers
  volatile uint32_t IABR[8]; // Interrupt Active Bit Registers (IABR0-IABR7)
  uint32_t          RESERVED4[56]; // Gap to IP registers
  volatile uint8_t  IP[240]; // Interrupt Priority Registers (IPR0-IPR239, 4 per word)
  uint32_t          RESERVED5[644]; // Gap to STIR register
  volatile uint32_t STIR; // Software Trigger Interrupt Register
}  NVIC_TypeDef;
#define NVIC                 ((NVIC_TypeDef *) NVIC_BASE)

// STM32F401RC specific IRQ number for TIM2 (from reference manual)
#define TIM2_IRQn            (28) 

// --- API Function Prototypes (from TT_API.json) ---
void TT_Init(t_tick_time tick_time_ms);
void TT_Start(void);
void TT_Dispatch_task(void);
void TT_ISR(void);
tbyte TT_Add_task(void (* task) (void), const tword period, const tword delay);
void TT_Delete_task(const tbyte task_index);

#endif // TT_H