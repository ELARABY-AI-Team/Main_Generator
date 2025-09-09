// core_includes: Required include statements as per Rules.json
#include "STM32F401RC_MCAL.h" // Include the generated header file

// Standard C library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h> // For snprintf, if used for debugging/string conversion
#include <stdlib.h> // For general utilities
#include <math.h>   // For mathematical operations, e.g., in PWM/Timer calculations

// ARM Cortex-M specific intrinsics for interrupt control and low power modes
#if defined(__ARMCC_VERSION) || defined(__ICCARM__) || defined(__GNUC__)
#include <cmsis_compiler.h>
#else
// Fallback for other compilers if needed
#define __enable_irq() __asm("CPSIE I")
#define __disable_irq() __asm("CPSID I")
#define __WFI() __asm("WFI")
#define __WFE() __asm("WFE")
#endif

// =============================================================================
// Private Defines and Macros
// =============================================================================

// Define peripheral base addresses for convenience in internal helpers
#define GPIOA_BASE              0x40020000UL
#define GPIOB_BASE              0x40020400UL
#define GPIOC_BASE              0x40020800UL
#define GPIOD_BASE              0x40020C00UL
#define GPIOE_BASE              0x40021000UL
#define GPIOH_BASE              0x40021C00UL

#define USART1_BASE             0x40011000UL
#define USART2_BASE             0x40004400UL
#define USART6_BASE             0x40011400UL

#define I2C1_BASE               0x40005400UL
#define I2C2_BASE               0x40005800UL
#define I2C3_BASE               0x40005C00UL

#define SPI1_BASE               0x40013000UL
#define SPI2_BASE               0x40003800UL
#define SPI3_BASE               0x40003C00UL

#define TIM1_BASE               0x40010000UL
#define TIM2_BASE               0x40000000UL
#define TIM3_BASE               0x40000400UL
#define TIM4_BASE               0x40000800UL
#define TIM5_BASE               0x40000C00UL
#define TIM9_BASE               0x40014000UL
#define TIM10_BASE              0x40014400UL
#define TIM11_BASE              0x40014800UL

#define ADC1_BASE               0x40012000UL

#define SYSCFG_BASE             0x40013800UL
#define EXTI_BASE               0x40013C00UL

// IWDG Registers (inferred, not in provided register_json)
#define IWDG_KR                 (*((volatile uint32_t *)0x40003000UL)) // Key register
#define IWDG_PR                 (*((volatile uint32_t *)0x40003004UL)) // Prescaler register
#define IWDG_RLR                (*((volatile uint32_t *)0x40003008UL)) // Reload register
#define IWDG_SR                 (*((volatile uint32_t *)0x4000300CUL)) // Status register

// RCC Clock Enable Bit Positions (inferred for STM32F401RC)
#define RCC_AHB1ENR_GPIOAEN     (1U << 0)
#define RCC_AHB1ENR_GPIOBEN     (1U << 1)
#define RCC_AHB1ENR_GPIOCEN     (1U << 2)
#define RCC_AHB1ENR_GPIODEN     (1U << 3)
#define RCC_AHB1ENR_GPIOEEN     (1U << 4)
#define RCC_AHB1ENR_GPIOHEN     (1U << 7)

#define RCC_APB1ENR_TIM2EN      (1U << 0)
#define RCC_APB1ENR_TIM3EN      (1U << 1)
#define RCC_APB1ENR_TIM4EN      (1U << 2)
#define RCC_APB1ENR_TIM5EN      (1U << 3)
#define RCC_APB1ENR_USART2EN    (1U << 17)
#define RCC_APB1ENR_I2C1EN      (1U << 21)
#define RCC_APB1ENR_I2C2EN      (1U << 22)
#define RCC_APB1ENR_I2C3EN      (1U << 23)
#define RCC_APB1ENR_SPI2EN      (1U << 14)
#define RCC_APB1ENR_SPI3EN      (1U << 15)

#define RCC_APB2ENR_TIM1EN      (1U << 0)
#define RCC_APB2ENR_USART1EN    (1U << 4)
#define RCC_APB2ENR_USART6EN    (1U << 5)
#define RCC_APB2ENR_ADC1EN      (1U << 8)
#define RCC_APB2ENR_SPI1EN      (1U << 12)
#define RCC_APB2ENR_SYSCFGEN    (1U << 14)
#define RCC_APB2ENR_TIM9EN      (1U << 16)
#define RCC_APB2ENR_TIM10EN     (1U << 17)
#define RCC_APB2ENR_TIM11EN     (1U << 18)

// GPIO Mode Register (MODER) bit values
#define GPIO_MODER_INPUT        (0U) // 00: Input mode
#define GPIO_MODER_OUTPUT       (1U) // 01: General purpose output mode
#define GPIO_MODER_AF           (2U) // 10: Alternate function mode
#define GPIO_MODER_ANALOG       (3U) // 11: Analog mode

// GPIO Output Type Register (OTYPER) bit values
#define GPIO_OTYPER_PUSH_PULL   (0U) // 0: Output push-pull
#define GPIO_OTYPER_OPEN_DRAIN  (1U) // 1: Output open-drain

// GPIO Output Speed Register (OSPEEDR) bit values
#define GPIO_OSPEEDR_LOW        (0U) // 00: Low speed
#define GPIO_OSPEEDR_MEDIUM     (1U) // 01: Medium speed
#define GPIO_OSPEEDR_HIGH       (2U) // 10: High speed
#define GPIO_OSPEEDR_VERY_HIGH  (3U) // 11: Very high speed (>=20mA sink current & >=10mA source current)

// GPIO Pull-up/Pull-down Register (PUPDR) bit values
#define GPIO_PUPDR_NOPULL       (0U) // 00: No pull-up, pull-down
#define GPIO_PUPDR_PULLUP       (1U) // 01: Pull-up
#define GPIO_PUPDR_PULLDOWN     (2U) // 10: Pull-down

// USART Control Register 1 (CR1) bits
#define USART_CR1_UE            (1U << 13) // USART Enable
#define USART_CR1_M             (1U << 12) // Word length
#define USART_CR1_PCE           (1U << 10) // Parity control enable
#define USART_CR1_PS            (1U << 9)  // Parity selection (0: Even, 1: Odd)
#define USART_CR1_TE            (1U << 3)  // Transmitter enable
#define USART_CR1_RE            (1U << 2)  // Receiver enable

// USART Control Register 2 (CR2) bits
#define USART_CR2_STOP_0_5      (1U << 12) // 0.5 Stop bit
#define USART_CR2_STOP_1        (0U << 12) // 1 Stop bit
#define USART_CR2_STOP_1_5      (3U << 12) // 1.5 Stop bits
#define USART_CR2_STOP_2        (2U << 12) // 2 Stop bits

// USART Status Register (SR) bits
#define USART_SR_TXE            (1U << 7) // Transmit data register empty
#define USART_SR_RXNE           (1U << 5) // Read data register not empty

// I2C Control Register 1 (CR1) bits
#define I2C_CR1_PE              (1U << 0)  // Peripheral enable
#define I2C_CR1_SMBUS           (1U << 1)  // SMBus mode
#define I2C_CR1_SMBTYPE         (1U << 3)  // SMBus type
#define I2C_CR1_ENARP           (1U << 4)  // ARP enable
#define I2C_CR1_ENPEC           (1U << 5)  // PEC enable
#define I2C_CR1_ENGC            (1U << 6)  // General call enable
#define I2C_CR1_NOSTRETCH       (1U << 7)  // Clock stretching disable (Slave mode)
#define I2C_CR1_START           (1U << 8)  // Start generation
#define I2C_CR1_STOP            (1U << 9)  // Stop generation
#define I2C_CR1_ACK             (1U << 10) // Acknowledge enable
#define I2C_CR1_POS             (1U << 11) // Acknowledge position (for next byte in receive mode)
#define I2C_CR1_PEC             (1U << 12) // Packet error checking
#define I2C_CR1_ALERT           (1U << 13) // SMBus alert
#define I2C_CR1_SWRST           (1U << 15) // Software reset

// I2C Control Register 2 (CR2) bits
#define I2C_CR2_FREQ_Pos        (0U) // Peripheral clock frequency bits (0-5)
#define I2C_CR2_ITERREN         (1U << 8) // Error interrupt enable
#define I2C_CR2_ITEVTEN         (1U << 9) // Event interrupt enable
#define I2C_CR2_ITBUFEN         (1U << 10) // Buffer interrupt enable
#define I2C_CR2_DMAEN           (1U << 11) // DMA requests enable
#define I2C_CR2_LAST            (1U << 12) // DMA last transfer

// I2C Status Register 1 (SR1) bits
#define I2C_SR1_SB              (1U << 0)  // Start bit (Master mode)
#define I2C_SR1_ADDR            (1U << 1)  // Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF             (1U << 2)  // Byte transfer finished
#define I2C_SR1_ADD10           (1U << 3)  // 10-bit header sent (Master mode)
#define I2C_SR1_STOPF           (1U << 4)  // Stop detected (Slave mode)
#define I2C_SR1_RXNE            (1U << 6)  // Data register not empty
#define I2C_SR1_TXE             (1U << 7)  // Data register empty
#define I2C_SR1_BERR            (1U << 8)  // Bus error
#define I2C_SR1_ARLO            (1U << 9)  // Arbitration lost
#define I2C_SR1_AF              (1U << 10) // Acknowledge failure
#define I2C_SR1_OVR             (1U << 11) // Overrun/Underrun
#define I2C_SR1_PECERR          (1U << 12) // PEC Error in reception
#define I2C_SR1_TIMEOUT         (1U << 14) // Timeout or Tlow error
#define I2C_SR1_SMBALERT        (1U << 15) // SMBus alert

// I2C Status Register 2 (SR2) bits
#define I2C_SR2_MSL             (1U << 0)  // Master/Slave mode
#define I2C_SR2_BUSY            (1U << 1)  // Bus busy
#define I2C_SR2_TRA             (1U << 2)  // Transmitter/Receiver
#define I2C_SR2_GENCALL         (1U << 4)  // General call address (Slave mode)
#define I2C_SR2_DUALF           (1U << 7)  // Dual flag (Slave mode)

// SPI Control Register 1 (CR1) bits
#define SPI_CR1_CPHA            (1U << 0)  // Clock phase
#define SPI_CR1_CPOL            (1U << 1)  // Clock polarity
#define SPI_CR1_MSTR            (1U << 2)  // Master selection
#define SPI_CR1_BR_Pos          (3U)       // Baud rate control (3 bits)
#define SPI_CR1_SPE             (1U << 6)  // SPI enable
#define SPI_CR1_LSBFIRST        (1U << 7)  // Frame format (LSB first)
#define SPI_CR1_SSI             (1U << 8)  // Internal slave select
#define SPI_CR1_SSM             (1U << 9)  // Software slave management
#define SPI_CR1_RXONLY          (1U << 10) // Receive only mode enable
#define SPI_CR1_DFF             (1U << 11) // Data frame format (0: 8-bit, 1: 16-bit)
#define SPI_CR1_CRCEN           (1U << 13) // CRC calculation enable
#define SPI_CR1_BIDIOE          (1U << 14) // Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE        (1U << 15) // Bidirectional data mode enable

// SPI Control Register 2 (CR2) bits
#define SPI_CR2_RXDMAEN         (1U << 0)  // Rx buffer DMA enable
#define SPI_CR2_TXDMAEN         (1U << 1)  // Tx buffer DMA enable
#define SPI_CR2_SSOE            (1U << 2)  // SS output enable
#define SPI_CR2_FRF             (1U << 4)  // Frame format
#define SPI_CR2_ERRIE           (1U << 5)  // Error interrupt enable
#define SPI_CR2_RXNEIE          (1U << 6)  // RXNE interrupt enable
#define SPI_CR2_TXEIE           (1U << 7)  // TXE interrupt enable

// SPI Status Register (SR) bits
#define SPI_SR_RXNE             (1U << 0)  // Receive buffer not empty
#define SPI_SR_TXE              (1U << 1)  // Transmit buffer empty
#define SPI_SR_CRCERR           (1U << 4)  // CRC error flag
#define SPI_SR_MODF             (1U << 5)  // Mode fault
#define SPI_SR_OVR              (1U << 6)  // Overrun flag
#define SPI_SR_BSY              (1U << 7)  // Busy flag
#define SPI_SR_FRE              (1U << 8)  // Frame format error

// I2S Configuration Register (I2SCFGR) bits (mapped to SPIx_I2SCFGR)
#define SPI_I2SCFGR_I2SMOD      (1U << 11) // I2S mode enable
#define SPI_I2SCFGR_I2SE        (1U << 10) // I2S Enable
#define SPI_I2SCFGR_I2SCFG_Pos  (8U)       // I2S configuration (2 bits)
#define SPI_I2SCFGR_I2SCFG_SLAVE_TX (0U << SPI_I2SCFGR_I2SCFG_Pos)
#define SPI_I2SCFGR_I2SCFG_SLAVE_RX (1U << SPI_I2SCFGR_I2SCFG_Pos)
#define SPI_I2SCFGR_I2SCFG_MASTER_TX (2U << SPI_I2SCFGR_I2SCFG_Pos)
#define SPI_I2SCFGR_I2SCFG_MASTER_RX (3U << SPI_I2SCFGR_I2SCFG_Pos)
#define SPI_I2SCFGR_PCMSYNC     (1U << 7)  // PCM frame synchronization
#define SPI_I2SCFGR_VGM         (1U << 6)  // Voice activity detection
#define SPI_I2SCFGR_DATLEN_Pos  (4U)       // Data length to be transferred (2 bits)
#define SPI_I2SCFGR_DATLEN_16B  (0U << SPI_I2SCFGR_DATLEN_Pos)
#define SPI_I2SCFGR_DATLEN_24B  (1U << SPI_I2SCFGR_DATLEN_Pos)
#define SPI_I2SCFGR_DATLEN_32B  (2U << SPI_I2SCFGR_DATLEN_Pos)
#define SPI_I2SCFGR_CHLEN       (1U << 3)  // Channel length
#define SPI_I2SCFGR_CKPOL       (1U << 2)  // Steady state clock polarity
#define SPI_I2SCFGR_I2SSTD_Pos  (0U)       // I2S standard selection (2 bits)
#define SPI_I2SCFGR_I2SSTD_PHILIPS (0U << SPI_I2SCFGR_I2SSTD_Pos)
#define SPI_I2SCFGR_I2SSTD_MSB  (1U << SPI_I2SCFGR_I2SSTD_Pos)
#define SPI_I2SCFGR_I2SSTD_LSB  (2U << SPI_I2SCFGR_I2SSTD_Pos)
#define SPI_I2SCFGR_I2SSTD_PCM  (3U << SPI_I2SCFGR_I2SSTD_Pos)

// I2S Prescaler Register (I2SPR) bits
#define SPI_I2SPR_I2SDIV_Pos    (0U) // I2S Linear prescaler (8 bits)
#define SPI_I2SPR_ODD           (1U << 8)  // Odd factor for the prescaler
#define SPI_I2SPR_MCKOE         (1U << 9)  // Master clock output enable

// TIMx_CR1 bits
#define TIM_CR1_CEN             (1U << 0) // Counter enable
#define TIM_CR1_UDIS            (1U << 1) // Update disable
#define TIM_CR1_URS             (1U << 2) // Update request source
#define TIM_CR1_OPM             (1U << 3) // One-pulse mode
#define TIM_CR1_DIR             (1U << 4) // Direction
#define TIM_CR1_CMS_Pos         (5U)      // Center-aligned mode selection
#define TIM_CR1_ARPE            (1U << 7) // Auto-reload preload enable
#define TIM_CR1_CKD_Pos         (8U)      // Clock division

// TIMx_CCMRx bits (Output Compare mode)
#define TIM_CCMR_OCM_PWM1       (6U << 4) // PWM mode 1
#define TIM_CCMR_OCM_PWM2       (7U << 4) // PWM mode 2
#define TIM_CCMR_OCPE           (1U << 3) // Output compare preload enable (CCxPE)

// TIMx_CCMRx bits (Input Capture mode)
#define TIM_CCMR_CCxS_INPUT_TI1 (1U << 0) // CCx channel is configured as input, ICx is mapped on TIx
#define TIM_CCMR_ICxF_Pos       (4U)      // Input capture filter
#define TIM_CCMR_ICxPSC_Pos     (2U)      // Input capture prescaler

// TIMx_CCER bits
#define TIM_CCER_CCxE           (1U << 0) // Capture/Compare output enable
#define TIM_CCER_CCxP           (1U << 1) // Capture/Compare output Polarity
#define TIM_CCER_CCxNE          (1U << 2) // Capture/Compare complementary output enable
#define TIM_CCER_CCxNP          (1U << 3) // Capture/Compare complementary output Polarity

// TIMx_BDTR bits (Break and Dead-time Register for Advanced Control Timers like TIM1)
#define TIM_BDTR_MOE            (1U << 15) // Main Output Enable

// TIMx_DIER bits
#define TIM_DIER_UIE            (1U << 0) // Update interrupt enable
#define TIM_DIER_CC1IE          (1U << 1) // Capture/Compare 1 interrupt enable
#define TIM_DIER_CC2IE          (1U << 2) // Capture/Compare 2 interrupt enable
#define TIM_DIER_CC3IE          (1U << 3) // Capture/Compare 3 interrupt enable
#define TIM_DIER_CC4IE          (1U << 4) // Capture/Compare 4 interrupt enable

// TIMx_SR bits
#define TIM_SR_UIF              (1U << 0) // Update interrupt flag
#define TIM_SR_CC1IF            (1U << 1) // Capture/Compare 1 interrupt flag
#define TIM_SR_CC2IF            (1U << 2) // Capture/Compare 2 interrupt flag
#define TIM_SR_CC3IF            (1U << 3) // Capture/Compare 3 interrupt flag
#define TIM_SR_CC4IF            (1U << 4) // Capture/Compare 4 interrupt flag

// EXTI Register bits
#define EXTI_RTSR_BIT(LINE) (1U << (LINE))
#define EXTI_FTSR_BIT(LINE) (1U << (LINE))
#define EXTI_IMR_BIT(LINE)  (1U << (LINE))
#define EXTI_PR_BIT(LINE)   (1U << (LINE))

// ADC Control Register 2 (CR2) bits
#define ADC_CR2_ADON            (1U << 0) // A/D Converter ON
#define ADC_CR2_CONT            (1U << 1) // Continuous conversion
#define ADC_CR2_SWSTART         (1U << 30) // Start regular conversion

// ADC Status Register (SR) bits
#define ADC_SR_EOC              (1U << 1) // End of conversion

// ADC Common Control Register (CCR) bits (STM32F4xx has this)
#define ADC_CCR_ADCPRE_Pos      (16U) // ADC prescaler (2 bits)
#define ADC_CCR_ADCPRE_DIV2     (0U << ADC_CCR_ADCPRE_Pos)
#define ADC_CCR_ADCPRE_DIV4     (1U << ADC_CCR_ADCPRE_Pos)
#define ADC_CCR_ADCPRE_DIV6     (2U << ADC_CCR_ADCPRE_Pos)
#define ADC_CCR_ADCPRE_DIV8     (3U << ADC_CCR_ADCPRE_Pos)

// MCU clock speeds (inferred for STM32F401RC, assuming max 84MHz APB2, 42MHz APB1)
#define F_CPU                   84000000UL // Max CPU frequency for STM32F401RC
#define APB1_TIMER_CLK_FREQ     84000000UL // APB1 timer clock can be x2 of APB1 peripheral clock if APB1 prescaler > 1
#define APB2_TIMER_CLK_FREQ     84000000UL // APB2 timer clock can be x2 of APB2 peripheral clock if APB2 prescaler > 1
#define APB1_PERIPH_CLK_FREQ    42000000UL // Max APB1 peripheral clock
#define APB2_PERIPH_CLK_FREQ    84000000UL // Max APB2 peripheral clock

// =============================================================================
// Private Data Types
// =============================================================================

// Structure to hold GPIO register pointers for internal use
typedef struct
{
  volatile uint32_t *MODER;
  volatile uint32_t *OTYPER;
  volatile uint32_t *OSPEEDR;
  volatile uint32_t *PUPDR;
  volatile uint32_t *IDR;
  volatile uint32_t *ODR;
  volatile uint32_t *BSRR;
  volatile uint32_t *LCKR;
  volatile uint32_t *AFRL;
  volatile uint32_t *AFRH;
} GPIO_Regs_t;

// Structure to hold USART register pointers for internal use
typedef struct
{
  volatile uint32_t *SR;
  volatile uint32_t *DR;
  volatile uint32_t *BRR;
  volatile uint32_t *CR1;
  volatile uint32_t *CR2;
  volatile uint32_t *CR3;
  volatile uint32_t *GTPR;
} USART_Regs_t;

// Structure to hold I2C register pointers for internal use
typedef struct
{
  volatile uint32_t *CR1;
  volatile uint32_t *CR2;
  volatile uint32_t *OAR1;
  volatile uint32_t *OAR2;
  volatile uint32_t *DR;
  volatile uint32_t *SR1;
  volatile uint32_t *SR2;
  volatile uint32_t *CCR;
  volatile uint32_t *TRISE;
  volatile uint32_t *FLTR;
} I2C_Regs_t;

// Structure to hold SPI register pointers for internal use (also used for I2S)
typedef struct
{
  volatile uint32_t *CR1;
  volatile uint32_t *CR2;
  volatile uint32_t *SR;
  volatile uint32_t *DR;
  volatile uint32_t *CRCPR;
  volatile uint32_t *RXCRCR;
  volatile uint32_t *TXCRCR;
  volatile uint32_t *I2SCFGR;
  volatile uint32_t *I2SPR;
} SPI_Regs_t;

// Structure to hold TIM register pointers for internal use
typedef struct
{
  volatile uint32_t *CR1;
  volatile uint32_t *CR2;
  volatile uint32_t *SMCR;
  volatile uint32_t *DIER;
  volatile uint32_t *SR;
  volatile uint32_t *EGR;
  volatile uint32_t *CCMR1;
  volatile uint32_t *CCMR2;
  volatile uint32_t *CCER;
  volatile uint32_t *CNT;
  volatile uint32_t *PSC;
  volatile uint32_t *ARR;
  volatile uint32_t *RCR;  // Only for advanced timers like TIM1
  volatile uint32_t *CCR1;
  volatile uint32_t *CCR2;
  volatile uint32_t *CCR3;
  volatile uint32_t *CCR4;
  volatile uint32_t *BDTR; // Only for advanced timers like TIM1
  volatile uint32_t *DCR;
  volatile uint32_t *DMAR;
  volatile uint32_t *OR;   // Only for TIM2, TIM5
} TIM_Regs_t;

// Structure to hold ADC register pointers for internal use
typedef struct
{
  volatile uint32_t *SR;
  volatile uint32_t *CR1;
  volatile uint32_t *CR2;
  volatile uint32_t *SMPR1;
  volatile uint32_t *SMPR2;
  volatile uint32_t *JOFR1;
  volatile uint32_t *JOFR2;
  volatile uint32_t *JOFR3;
  volatile uint32_t *JOFR4;
  volatile uint32_t *HTR;
  volatile uint32_t *LTR;
  volatile uint32_t *SQR1;
  volatile uint32_t *SQR2;
  volatile uint32_t *SQR3;
  volatile uint32_t *JSQR;
  volatile uint32_t *JDR1;
  volatile uint32_t *JDR2;
  volatile uint32_t *JDR3;
  volatile uint32_t *JDR4;
  volatile uint32_t *DR;
} ADC_Regs_t;


// TT Task structure
typedef struct {
    void (*pTask)(void);
    tword delay;
    tword period;
    bool run_me;
} TT_Task_t;

// TT config
#define TT_MAX_TASKS 10
static TT_Task_t TT_Tasks[TT_MAX_TASKS];
static tword TT_Tick_ms = 0; // The actual tick time in ms set by TT_Init

// =============================================================================
// Private Function Prototypes (Helper functions for internal use)
// =============================================================================

static GPIO_Regs_t* get_gpio_regs(t_port port);
static USART_Regs_t* get_usart_regs(t_uart_channel channel);
static I2C_Regs_t* get_i2c_regs(t_i2c_channel channel);
static SPI_Regs_t* get_spi_regs(t_spi_channel channel);
static TIM_Regs_t* get_timer_regs(t_timer_channel channel);
static ADC_Regs_t* get_adc_regs(void); // Only one ADC peripheral, ADC1

static void enable_gpio_clock(t_port port);
static void enable_usart_clock(t_uart_channel channel);
static void enable_i2c_clock(t_i2c_channel channel);
static void enable_spi_clock(t_spi_channel channel);
static void enable_timer_clock(t_timer_channel channel);
static void enable_adc_clock(void);
static void enable_syscfg_clock(void);
static void set_gpio_alternate_function(t_port port, t_pin pin, tbyte af_value);

// =============================================================================
// Private Global Variables
// =============================================================================
static void (*icu_callback_ptr)(void) = NULL;

// =============================================================================
// Private Helper Functions Implementation
// =============================================================================

/**
 * @brief Get GPIO register structure for a given port.
 * @param port The GPIO port (A, B, C, D, E, H).
 * @return Pointer to the GPIO_Regs_t structure, or NULL if invalid port.
 */
static GPIO_Regs_t* get_gpio_regs(t_port port)
{
  GPIO_Regs_t* gpio = (GPIO_Regs_t*)NULL;
  uint32_t base_addr = 0;

  switch (port)
  {
    case PORT_A: base_addr = GPIOA_BASE; break;
    case PORT_B: base_addr = GPIOB_BASE; break;
    case PORT_C: base_addr = GPIOC_BASE; break;
    case PORT_D: base_addr = GPIOD_BASE; break;
    case PORT_E: base_addr = GPIOE_BASE; break;
    case PORT_H: base_addr = GPIOH_BASE; break;
    default: return (GPIO_Regs_t*)NULL;
  }

  // Cast base address to GPIO_Regs_t pointer (assuming registers are contiguous from base)
  // Note: This is a simplified representation. Actual STM32 peripherals are structs,
  // but for generic register access, this pointer arithmetic works with the provided #defines.
  // For proper structure, one would define `typedef struct { volatile uint32_t MODER; ... } GPIO_TypeDef;`
  // and then `(GPIO_TypeDef *)base_addr;`
  // Given the current register #defines, we'll map them individually for clarity.
  // This helper will return a 'fictional' structure constructed on the fly.
  // Since we cannot return a dynamically constructed struct, we will use individual register defines
  // directly in the public API functions. The rule "Do not expose pointer types in public API signatures"
  // implies that the *conversion* from enum to pointer happens *inside* the API.
  // However, the "pointer_variable_consistency" also mentions `static void Peripheral_SetDirection(void *peripheral_ptr, ...)`
  // which implies passing a base address. I will revise to return base address as void* or uint32_t.

  // Re-evaluating `pointer_variable_consistency`: "Static helper functions (...) may use pointers internally to access hardware registers directly."
  // "Public API functions (...) must use variables/enums (...) not pointers."
  // "All conversions from variables/enums to pointers must happen inside the public API functions using a generic mapping function (e.g., `get_peripheral_ptr(peripheral_enum)`), not in the helper functions."
  // So, this `get_gpio_regs` should return the base address.
  return (GPIO_Regs_t*)base_addr;
}

/**
 * @brief Get USART register structure for a given channel.
 * @param channel The UART channel (1, 2, 6).
 * @return Pointer to the USART_Regs_t structure, or NULL if invalid channel.
 */
static USART_Regs_t* get_usart_regs(t_uart_channel channel)
{
    uint32_t base_addr = 0;
    switch (channel)
    {
        case UART_CHANNEL_1: base_addr = USART1_BASE; break;
        case UART_CHANNEL_2: base_addr = USART2_BASE; break;
        case UART_CHANNEL_6: base_addr = USART6_BASE; break;
        default: return (USART_Regs_t*)NULL;
    }
    return (USART_Regs_t*)base_addr;
}

/**
 * @brief Get I2C register structure for a given channel.
 * @param channel The I2C channel (1, 2, 3).
 * @return Pointer to the I2C_Regs_t structure, or NULL if invalid channel.
 */
static I2C_Regs_t* get_i2c_regs(t_i2c_channel channel)
{
    uint32_t base_addr = 0;
    switch (channel)
    {
        case I2C_CHANNEL_1: base_addr = I2C1_BASE; break;
        case I2C_CHANNEL_2: base_addr = I2C2_BASE; break;
        case I2C_CHANNEL_3: base_addr = I2C3_BASE; break;
        default: return (I2C_Regs_t*)NULL;
    }
    return (I2C_Regs_t*)base_addr;
}

/**
 * @brief Get SPI register structure for a given channel.
 * @param channel The SPI channel (1, 2, 3).
 * @return Pointer to the SPI_Regs_t structure, or NULL if invalid channel.
 */
static SPI_Regs_t* get_spi_regs(t_spi_channel channel)
{
    uint32_t base_addr = 0;
    switch (channel)
    {
        case SPI_CHANNEL_1: base_addr = SPI1_BASE; break;
        case SPI_CHANNEL_2: base_addr = SPI2_BASE; break;
        case SPI_CHANNEL_3: base_addr = SPI3_BASE; break;
        default: return (SPI_Regs_t*)NULL;
    }
    return (SPI_Regs_t*)base_addr;
}

/**
 * @brief Get TIM register structure for a given channel.
 * @param channel The Timer channel (1, 2, 3, 4, 5, 9, 10, 11).
 * @return Pointer to the TIM_Regs_t structure, or NULL if invalid channel.
 */
static TIM_Regs_t* get_timer_regs(t_timer_channel channel)
{
    uint32_t base_addr = 0;
    switch (channel)
    {
        case TIMER_CHANNEL_1:  base_addr = TIM1_BASE; break;
        case TIMER_CHANNEL_2:  base_addr = TIM2_BASE; break;
        case TIMER_CHANNEL_3:  base_addr = TIM3_BASE; break;
        case TIMER_CHANNEL_4:  base_addr = TIM4_BASE; break;
        case TIMER_CHANNEL_5:  base_addr = TIM5_BASE; break;
        case TIMER_CHANNEL_9:  base_addr = TIM9_BASE; break;
        case TIMER_CHANNEL_10: base_addr = TIM10_BASE; break;
        case TIMER_CHANNEL_11: base_addr = TIM11_BASE; break;
        default: return (TIM_Regs_t*)NULL;
    }
    return (TIM_Regs_t*)base_addr;
}

/**
 * @brief Get ADC register structure (for ADC1).
 * @return Pointer to the ADC_Regs_t structure.
 */
static ADC_Regs_t* get_adc_regs(void)
{
    return (ADC_Regs_t*)ADC1_BASE; // Only ADC1 in STM32F401RC
}

/**
 * @brief Enables the clock for a given GPIO port.
 * @param port The GPIO port to enable clock for.
 */
static void enable_gpio_clock(t_port port)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    uint32_t enr_bit = 0;
    switch (port)
    {
        case PORT_A: enr_bit = RCC_AHB1ENR_GPIOAEN; break;
        case PORT_B: enr_bit = RCC_AHB1ENR_GPIOBEN; break;
        case PORT_C: enr_bit = RCC_AHB1ENR_GPIOCEN; break;
        case PORT_D: enr_bit = RCC_AHB1ENR_GPIODEN; break;
        case PORT_E: enr_bit = RCC_AHB1ENR_GPIOEEN; break;
        case PORT_H: enr_bit = RCC_AHB1ENR_GPIOHEN; break;
        default: return; // Invalid port
    }
    RCC_AHB1ENR |= enr_bit;
    // Dummy read to ensure clock is ready
    (void)RCC_AHB1ENR;
}

/**
 * @brief Enables the clock for a given USART peripheral.
 * @param channel The USART channel to enable clock for.
 */
static void enable_usart_clock(t_uart_channel channel)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    switch (channel)
    {
        case UART_CHANNEL_1:
            RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
            (void)RCC_APB2ENR;
            break;
        case UART_CHANNEL_2:
            RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
            (void)RCC_APB1ENR;
            break;
        case UART_CHANNEL_6:
            RCC_APB2ENR |= RCC_APB2ENR_USART6EN;
            (void)RCC_APB2ENR;
            break;
        default: break; // Invalid channel
    }
}

/**
 * @brief Enables the clock for a given I2C peripheral.
 * @param channel The I2C channel to enable clock for.
 */
static void enable_i2c_clock(t_i2c_channel channel)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    switch (channel)
    {
        case I2C_CHANNEL_1:
            RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;
            (void)RCC_APB1ENR;
            break;
        case I2C_CHANNEL_2:
            RCC_APB1ENR |= RCC_APB1ENR_I2C2EN;
            (void)RCC_APB1ENR;
            break;
        case I2C_CHANNEL_3:
            RCC_APB1ENR |= RCC_APB1ENR_I2C3EN;
            (void)RCC_APB1ENR;
            break;
        default: break; // Invalid channel
    }
}

/**
 * @brief Enables the clock for a given SPI peripheral.
 * @param channel The SPI channel to enable clock for.
 */
static void enable_spi_clock(t_spi_channel channel)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    switch (channel)
    {
        case SPI_CHANNEL_1:
            RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
            (void)RCC_APB2ENR;
            break;
        case SPI_CHANNEL_2:
            RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
            (void)RCC_APB1ENR;
            break;
        case SPI_CHANNEL_3:
            RCC_APB1ENR |= RCC_APB1ENR_SPI3EN;
            (void)RCC_APB1ENR;
            break;
        default: break; // Invalid channel
    }
}

/**
 * @brief Enables the clock for a given Timer peripheral.
 * @param channel The Timer channel to enable clock for.
 */
static void enable_timer_clock(t_timer_channel channel)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    switch (channel)
    {
        case TIMER_CHANNEL_1:  RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;  (void)RCC_APB2ENR; break;
        case TIMER_CHANNEL_2:  RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;  (void)RCC_APB1ENR; break;
        case TIMER_CHANNEL_3:  RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;  (void)RCC_APB1ENR; break;
        case TIMER_CHANNEL_4:  RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;  (void)RCC_APB1ENR; break;
        case TIMER_CHANNEL_5:  RCC_APB1ENR |= RCC_APB1ENR_TIM5EN;  (void)RCC_APB1ENR; break;
        case TIMER_CHANNEL_9:  RCC_APB2ENR |= RCC_APB2ENR_TIM9EN;  (void)RCC_APB2ENR; break;
        case TIMER_CHANNEL_10: RCC_APB2ENR |= RCC_APB2ENR_TIM10EN; (void)RCC_APB2ENR; break;
        case TIMER_CHANNEL_11: RCC_APB2ENR |= RCC_APB2ENR_TIM11EN; (void)RCC_APB2ENR; break;
        default: break; // Invalid channel
    }
}

/**
 * @brief Enables the clock for ADC1 peripheral.
 */
static void enable_adc_clock(void)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR; // Dummy read to ensure clock is ready
}

/**
 * @brief Enables the clock for SYSCFG peripheral.
 */
static void enable_syscfg_clock(void)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC_APB2ENR; // Dummy read to ensure clock is ready
}

/**
 * @brief Sets a GPIO pin to a specific Alternate Function mode.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param af_value The alternate function value (0-15).
 */
static void set_gpio_alternate_function(t_port port, t_pin pin, tbyte af_value)
{
    // WDT_Reset() needs to be called by public APIs before calling this helper.
    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return; // Invalid port

    volatile uint33_t *AFR_reg;
    tbyte af_offset = 0;

    if (pin < PIN_8) { // AFR[L]
        AFR_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x20UL); // GPIOx_AFRL
        af_offset = pin * 4;
    } else { // AFR[H]
        AFR_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x24UL); // GPIOx_AFRH
        af_offset = (pin - PIN_8) * 4;
    }

    *AFR_reg &= ~(0xFUL << af_offset);       // Clear current AF setting
    *AFR_reg |= ((uint32_t)af_value << af_offset); // Set new AF value
}

// =============================================================================
// MCAL API Function Implementations
// =============================================================================

/**
 * @brief Initializes the MCU configuration, including GPIOs, WDT, and LVR settings.
 *        As per Rules.json:
 *        - Set all GPIO pins to 0 and verify.
 *        - Set all GPIO pins direction to input and verify.
 *        - Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.).
 *        - Enable WDT, clear WDT timer, set WDT period >= 8 msec.
 *        - Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V).
 *        - Enable LVR (Low Voltage Reset).
 *        - Clear WDT again.
 * @param volt The system voltage (3V or 5V) to configure LVR.
 */
void MCU_Config_Init(t_sys_volt volt)
{
    // Ensure WDT is reset before any critical configuration
    WDT_Reset();

    // 1. Set all GPIO pins to 0 and verify with while loop
    //    and set all GPIO pins direction to input and verify with while loop
    //    All input pins have pull-up resistors and wakeup feature enabled (if available)
    //    All output pins have pull-up resistors disabled (already default for input)

    // Enable all GPIO clocks first to configure them
    RCC_AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN);
    (void)RCC_AHB1ENR; // Dummy read

    t_port ports[] = {PORT_A, PORT_B, PORT_C, PORT_D, PORT_E, PORT_H};
    const uint32_t GPIO_MAX_PINS = 16;
    for (size_t p = 0; p < sizeof(ports) / sizeof(ports[0]); p++)
    {
        GPIO_Regs_t* gpio = get_gpio_regs(ports[p]);
        if (gpio == (GPIO_Regs_t*)NULL) continue;

        // Reset all registers to default state (0 or input mode)
        // MODER: 00 (Input mode) for all pins
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) ) = 0x00000000UL; // GPIOx_MODER
        // OTYPER: 0 (Push-pull) for all pins (default)
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x04UL) ) = 0x00000000UL; // GPIOx_OTYPER
        // OSPEEDR: 00 (Low speed) for all pins (default)
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x08UL) ) = 0x00000000UL; // GPIOx_OSPEEDR
        // PUPDR: 00 (No pull-up/pull-down) for all pins, will set pull-up for inputs
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL) ) = 0x00000000UL; // GPIOx_PUPDR
        // ODR: 0 (Low) for all output pins
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) ) = 0x00000000UL; // GPIOx_ODR
        // AFRL/AFRH: 0 (AF0) for all pins
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x20UL) ) = 0x00000000UL; // GPIOx_AFRL
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x24UL) ) = 0x00000000UL; // GPIOx_AFRH

        for (t_pin pin = PIN_0; pin < GPIO_MAX_PINS; pin++)
        {
            // Set initial output value to 0 for all pins (before setting direction)
            // GPIO_Value_Set(ports[p], pin, 0); // Not using this directly as we're initializing all
            // Writing to ODR directly for mass configuration
            *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) ) &= ~(1UL << pin);

            // Set direction to input and configure pull-up
            *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) ) &= ~(GPIO_MODER_ANALOG << (pin * 2)); // Set to 00: Input mode
            *( (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL) ) &= ~(GPIO_PUPDR_PULLDOWN << (pin * 2)); // Clear bits
            *( (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL) ) |= (GPIO_PUPDR_PULLUP << (pin * 2)); // Set to 01: Pull-up

            // Verification (simplified: check if MODER is input and PUPDR is pull-up)
            volatile uint32_t moder_val = *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) );
            volatile uint32_t pupdr_val = *( (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL) );
            while (((moder_val >> (pin * 2)) & GPIO_MODER_ANALOG) != GPIO_MODER_INPUT) { /* Wait */ }
            while (((pupdr_val >> (pin * 2)) & GPIO_PUPDR_PULLDOWN) != GPIO_PUPDR_PULLUP) { /* Wait */ }
        }
    }

    // 2. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable ADC1
    ADC1_CR2 &= ~ADC_CR2_ADON;

    // Disable all USARTs
    USART1_CR1 &= ~USART_CR1_UE;
    USART2_CR1 &= ~USART_CR1_UE;
    USART6_CR1 &= ~USART_CR1_UE;

    // Disable all I2Cs
    I2C1_CR1 &= ~I2C_CR1_PE;
    I2C2_CR1 &= ~I2C_CR1_PE;
    I2C3_CR1 &= ~I2C_CR1_PE;

    // Disable all SPIs and I2S
    SPI1_CR1 &= ~SPI_CR1_SPE;
    SPI1_I2SCFGR &= ~SPI_I2SCFGR_I2SE; // Disable I2S mode if active
    SPI2_CR1 &= ~SPI_CR1_SPE;
    SPI2_I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    SPI3_CR1 &= ~SPI_CR1_SPE;
    SPI3_I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // Disable all Timers
    TIM1_CR1 &= ~TIM_CR1_CEN;
    TIM2_CR1 &= ~TIM_CR1_CEN;
    TIM3_CR1 &= ~TIM_CR1_CEN;
    TIM4_CR1 &= ~TIM_CR1_CEN;
    TIM5_CR1 &= ~TIM_CR1_CEN;
    TIM9_CR1 &= ~TIM_CR1_CEN;
    TIM10_CR1 &= ~TIM_CR1_CEN;
    TIM11_CR1 &= ~TIM_CR1_CEN;

    // 3. Enable WDT, clear WDT timer, set WDT period >= 8 msec.
    // For STM32, this refers to the Independent Watchdog (IWDG).
    // IWDG is clocked by LSI (internal low-speed oscillator) typically around 32kHz.
    // IWDG period = (RLR + 1) * 2^PR / LSI_FREQ.
    // LSI_FREQ is around 32kHz. To get >= 8ms:
    // With PR=0 (prescaler /4), RLR_max = 0xFFF.
    // Max period = (4095 + 1) * 4 / 32000Hz = 0.512 seconds.
    // To get >= 8ms, we can use PR=0, RLR = (8ms * 32000 / 4) - 1 = 64 - 1 = 63.
    // This value is safe and well within limits.

    IWDG_KR = 0x5555; // Enable write access to PR and RLR
    IWDG_PR = 0x0;    // Set prescaler to /4 (PR=0, so 2^0 * 4 = 4) (In STM32, PR register values map differently: 0->div4, 1->div8, etc.)
                      // For STM32F4, IWDG_PR: 0b000 -> div4, 0b001 -> div8, ... 0b110 -> div256. Use 0 for div4.
    IWDG_RLR = 63;    // Reload value for ~8ms (LSI_CLK / (prescaler * RLR_val))
                      // Period = (RLR + 1) * Prescaler_Div / LSI_Freq.
                      // 8ms = (RLR + 1) * 4 / 32000 => RLR + 1 = 8 * 32 / 4 = 64 => RLR = 63.
    IWDG_KR = 0xAAAA; // Reload the watchdog counter
    IWDG_KR = 0xCCCC; // Start watchdog (enable access for IWDG_CR and enable the peripheral)

    // 4. Set LOW Voltage Reset value based on system voltage and Enable LVR.
    // STM32F401RC uses the Power Control (PWR) peripheral for PVD (Programmable Voltage Detector).
    // The provided register_json does not include PWR registers (e.g., PWR_CR, PWR_CSR).
    // Placeholder comment as per rules:
    // PVD/LVR configuration would typically involve PWR_CR register (PVDEN, PLS bits).
    // For example, for 3V system, threshold might be 2V, for 5V system, threshold might be 3.5V.
    // These specific register manipulations are outside the scope of provided JSON.
    (void)volt; // Suppress unused parameter warning
    // For STM32F401RC, PVD is configured via PWR->CR.
    // Example: enable_pwr_clock(); PWR->CR |= (PWR_CR_PVDE | (threshold_level << PWR_CR_PLS_Pos));
    // As PWR registers are not in JSON, this part cannot be fully implemented.
    // RCC_APB1ENR |= RCC_APB1ENR_PWREN; // Inferring PWR clock enable
    // PWR_CR |= (PWR_CR_PVDE | (PVD_THRESHOLD_2V << PWR_CR_PLS_Pos)); // Placeholder for 2V threshold.
    // LVD configuration not fully implementable without specific PWR registers in register_json.

    // 5. Clear WDT again.
    WDT_Reset();
}

/**
 * @brief Resets the Watchdog Timer (WDT).
 *        As per Rules.json: All API bodies must include WDT_Reset().
 */
void WDT_Reset(void)
{
    // For STM32 IWDG, writing 0xAAAA to IWDG_KR reloads the counter.
    IWDG_KR = 0xAAAA;
}

/**
 * @brief Puts the MCU into a low-power sleep mode.
 *        As per Rules.json: Sleep mode stops executing code and peripherals (except OS timer).
 */
void Go_to_sleep_mode(void)
{
    WDT_Reset();
    // For ARM Cortex-M, WFI (Wait For Interrupt) is a common instruction for sleep mode.
    // It halts the CPU until an interrupt occurs.
    __WFI();
}

/**
 * @brief Enables global interrupts.
 *        As per Rules.json: All API bodies must include WDT_Reset().
 */
void Global_interrupt_Enable(void)
{
    WDT_Reset();
    __enable_irq(); // CMSIS intrinsic for enabling global interrupts
}

/**
 * @brief Disables global interrupts.
 *        As per Rules.json: All API bodies must include WDT_Reset().
 */
void Global_interrupt_Disable(void)
{
    WDT_Reset();
    __disable_irq(); // CMSIS intrinsic for disabling global interrupts
}

// LVD (Low Voltage Detection)
// As LVD registers (PVD related in STM32) are not provided in register_json,
// and are usually part of the Power Control (PWR) peripheral, these functions
// cannot be fully implemented. Only WDT_Reset will be present.

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 */
void LVD_Init(void)
{
    WDT_Reset();
    // LVD initialization for STM32F401RC would involve enabling the PWR clock
    // and configuring the PVD in PWR_CR. Registers are not in register_json.
    // RCC_APB1ENR |= RCC_APB1ENR_PWREN; // Inferring PWR clock enable
    // PWR_CR &= ~PWR_CR_PVDE; // Disable PVD first
    // No further implementation possible with provided register_json.
}

/**
 * @brief Sets the LVD threshold level.
 * @param lvd_thresholdLevel The desired threshold level.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel)
{
    WDT_Reset();
    // LVD threshold setting for STM32F401RC would involve PWR_CR.PLS bits.
    // Example: PWR_CR = (PWR_CR & ~PWR_CR_PLS_Msk) | (lvd_thresholdLevel << PWR_CR_PLS_Pos);
    (void)lvd_thresholdLevel; // Suppress unused parameter warning
    // No further implementation possible with provided register_json.
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 */
void LVD_Enable(void)
{
    WDT_Reset();
    // LVD enable for STM32F401RC would involve setting PWR_CR.PVDE bit.
    // Example: PWR_CR |= PWR_CR_PVDE;
    // No further implementation possible with provided register_json.
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 */
void LVD_Disable(void)
{
    WDT_Reset();
    // LVD disable for STM32F401RC would involve clearing PWR_CR.PVDE bit.
    // Example: PWR_CR &= ~PWR_CR_PVDE;
    // No further implementation possible with provided register_json.
}

// UART
/**
 * @brief Initializes a specific UART channel with the given parameters.
 * @param uart_channel The UART channel to initialize.
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The number of data bits (8 or 9).
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting (None, Even, Odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity)
{
    WDT_Reset();

    USART_Regs_t* usart = get_usart_regs(uart_channel);
    if (usart == (USART_Regs_t*)NULL) return;

    // Enable USART clock and associated GPIO clock
    enable_usart_clock(uart_channel);

    t_port tx_port = PORT_A; t_pin tx_pin = PIN_9; tbyte tx_af = 7; // USART1_TX: PA9 (AF7)
    t_port rx_port = PORT_A; t_pin rx_pin = PIN_10; tbyte rx_af = 7; // USART1_RX: PA10 (AF7)

    // Determine GPIO pins and AF based on channel
    switch (uart_channel)
    {
        case UART_CHANNEL_1:
            tx_port = PORT_A; tx_pin = PIN_9; rx_port = PORT_A; rx_pin = PIN_10; tx_af = 7; rx_af = 7;
            // Also possible: PB6(TX), PB7(RX) AF7
            enable_gpio_clock(PORT_A);
            break;
        case UART_CHANNEL_2:
            tx_port = PORT_A; tx_pin = PIN_2; rx_port = PORT_A; rx_pin = PIN_3; tx_af = 7; rx_af = 7;
            // Also possible: PD5(TX), PD6(RX) AF7
            enable_gpio_clock(PORT_A);
            break;
        case UART_CHANNEL_6:
            tx_port = PORT_C; tx_pin = PIN_6; rx_port = PORT_C; rx_pin = PIN_7; tx_af = 8; rx_af = 8;
            // Also possible: PA11(TX), PA12(RX) AF8
            enable_gpio_clock(PORT_C);
            break;
        default: return; // Should not happen due to previous check
    }

    // Configure GPIO pins for Alternate Function (TX and RX)
    // Always set value before setting direction (not applicable for AF directly)
    // Output Type: Push-pull (default), Speed: Very High, Pull-up for RX
    GPIO_Output_Init(tx_port, tx_pin, 0); // Init as output first, then set AF
    GPIO_Input_Init(rx_port, rx_pin);     // Init as input first, then set AF

    // Set AF mode for TX pin
    volatile uint32_t* tx_moder_reg = (volatile uint32_t*)( (uint32_t)get_gpio_regs(tx_port) + 0x00UL);
    *tx_moder_reg &= ~(GPIO_MODER_ANALOG << (tx_pin * 2)); // Clear mode bits
    *tx_moder_reg |= (GPIO_MODER_AF << (tx_pin * 2));     // Set AF mode
    set_gpio_alternate_function(tx_port, tx_pin, tx_af);

    // Set AF mode for RX pin
    volatile uint32_t* rx_moder_reg = (volatile uint32_t*)( (uint32_t)get_gpio_regs(rx_port) + 0x00UL);
    *rx_moder_reg &= ~(GPIO_MODER_ANALOG << (rx_pin * 2)); // Clear mode bits
    *rx_moder_reg |= (GPIO_MODER_AF << (rx_pin * 2));     // Set AF mode
    set_gpio_alternate_function(rx_port, rx_pin, rx_af);

    // Disable USART before configuration
    *usart->CR1 &= ~USART_CR1_UE;

    // Configure Baud Rate
    uint32_t pclk_freq = 0;
    if (uart_channel == UART_CHANNEL_1 || uart_channel == UART_CHANNEL_6) {
        pclk_freq = APB2_PERIPH_CLK_FREQ; // APB2 clock for USART1/6
    } else { // UART_CHANNEL_2
        pclk_freq = APB1_PERIPH_CLK_FREQ; // APB1 clock for USART2
    }

    uint32_t baud_val = 0;
    switch (uart_baud_rate) {
        case UART_BAUD_RATE_2400:   baud_val = 2400;   break;
        case UART_BAUD_RATE_4800:   baud_val = 4800;   break;
        case UART_BAUD_RATE_9600:   baud_val = 9600;   break;
        case UART_BAUD_RATE_19200:  baud_val = 19200;  break;
        case UART_BAUD_RATE_38400:  baud_val = 38400;  break;
        case UART_BAUD_RATE_57600:  baud_val = 57600;  break;
        case UART_BAUD_RATE_115200: baud_val = 115200; break;
        case UART_BAUD_RATE_230400: baud_val = 230400; break;
        case UART_BAUD_RATE_460800: baud_val = 460800; break;
        case UART_BAUD_RATE_921600: baud_val = 921600; break;
        default: baud_val = 9600; break; // Default to 9600
    }
    // Baud rate calculation: PCLK / (8 * (2 - OVER8) * Baudrate)
    // Assuming OVER8 = 0 (oversampling by 16)
    uint32_t usartdiv = (uint32_t)((pclk_freq * 25) / (baud_val * 4));
    uint32_t mantissa = usartdiv / 100;
    uint32_t fraction = (usartdiv - (mantissa * 100)) * 16 / 100;
    *usart->BRR = (mantissa << 4) | (fraction & 0xF);

    // Configure Data Length (M bit in CR1)
    *usart->CR1 &= ~USART_CR1_M; // Clear M bit (8-bit word length)
    if (uart_data_length == UART_DATA_LENGTH_9_BITS) {
        *usart->CR1 |= USART_CR1_M; // Set M bit (9-bit word length)
    }

    // Configure Stop Bits (STOP bits in CR2)
    *usart->CR2 &= ~(USART_CR2_STOP_0_5 | USART_CR2_STOP_1_5 | USART_CR2_STOP_2); // Clear stop bits
    switch (uart_stop_bit) {
        case UART_STOP_BIT_0_5: *usart->CR2 |= USART_CR2_STOP_0_5; break;
        case UART_STOP_BIT_1:   /* Default, no bits set */         break;
        case UART_STOP_BIT_1_5: *usart->CR2 |= USART_CR2_STOP_1_5; break;
        case UART_STOP_BIT_2:   *usart->CR2 |= USART_CR2_STOP_2;   break;
    }

    // Configure Parity (PCE, PS bits in CR1)
    *usart->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS); // Clear parity bits
    switch (uart_parity) {
        case UART_PARITY_NONE: /* Default, no bits set */         break;
        case UART_PARITY_EVEN: *usart->CR1 |= USART_CR1_PCE;      break; // PCE=1, PS=0
        case UART_PARITY_ODD:  *usart->CR1 |= (USART_CR1_PCE | USART_CR1_PS); break; // PCE=1, PS=1
    }

    // Enable Transmitter and Receiver
    *usart->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART peripheral (UE bit in CR1)
    *usart->CR1 |= USART_CR1_UE;
}

/**
 * @brief Enables a specific UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel)
{
    WDT_Reset();
    USART_Regs_t* usart = get_usart_regs(uart_channel);
    if (usart == (USART_Regs_t*)NULL) return;

    // Peripheral clock must be enabled before enabling the peripheral itself
    enable_usart_clock(uart_channel); // RCC clock enable (inferred)
    *usart->CR1 |= USART_CR1_UE; // Enable USART peripheral
}

/**
 * @brief Disables a specific UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel)
{
    WDT_Reset();
    USART_Regs_t* usart = get_usart_regs(uart_channel);
    if (usart == (USART_Regs_t*)NULL) return;
    *usart->CR1 &= ~USART_CR1_UE; // Disable USART peripheral
}

/**
 * @brief Sends a single byte over a specific UART channel.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte)
{
    WDT_Reset();
    USART_Regs_t* usart = get_usart_regs(uart_channel);
    if (usart == (USART_Regs_t*)NULL) return;

    // Wait until the transmit data register is empty
    while (!(*usart->SR & USART_SR_TXE)) { /* Wait */ }
    *usart->DR = byte; // Write data to the data register
}

/**
 * @brief Sends a frame (array of bytes) over a specific UART channel.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length)
{
    WDT_Reset();
    for (int i = 0; i < length; i++)
    {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over a specific UART channel.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str)
{
    WDT_Reset();
    while (*str != '\0')
    {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from a specific UART channel.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel)
{
    WDT_Reset();
    USART_Regs_t* usart = get_usart_regs(uart_channel);
    if (usart == (USART_Regs_t*)NULL) return 0; // Or error code

    // Wait until data is received
    while (!(*usart->SR & USART_SR_RXNE)) { /* Wait */ }
    return (tbyte)(*usart->DR & 0xFF); // Read data from the data register
}

/**
 * @brief Receives a frame (array of bytes) from a specific UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset();
    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a string from a specific UART channel, ending on a newline or max_length.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive.
 * @return The received byte (last char, or 0 if timeout/error, or newline)
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length)
{
    WDT_Reset();
    tbyte received_byte = 0;
    for (int i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        received_byte = UART_Get_Byte(uart_channel);
        buffer[i] = (char)received_byte;
        if (received_byte == '\n' || received_byte == '\r') { // Assuming newline terminates string
            buffer[i+1] = '\0';
            return received_byte;
        }
    }
    buffer[max_length-1] = '\0'; // Ensure null termination
    return received_byte;
}

// I2C
/**
 * @brief Initializes a specific I2C channel with the given parameters.
 *        As per Rules.json: Always use fast mode, maximum timeout, and repeated start.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed The desired clock speed (must be fast mode).
 * @param i2c_device_address The device addressing mode (7-bit or 10-bit).
 * @param i2c_ack Acknowledge enable/disable.
 * @param i2c_datalength Not used for typical I2C byte transfers (always 8-bit).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength)
{
    WDT_Reset();

    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    // Enable I2C clock and associated GPIO clock
    enable_i2c_clock(i2c_channel);

    t_port scl_port, sda_port;
    t_pin scl_pin, sda_pin;
    tbyte af_val = 4; // I2C1, I2C2, I2C3 all use AF4

    switch (i2c_channel) {
        case I2C_CHANNEL_1:
            scl_port = PORT_B; scl_pin = PIN_6; sda_port = PORT_B; sda_pin = PIN_7; // PB6(SCL), PB7(SDA) AF4
            enable_gpio_clock(PORT_B);
            break;
        case I2C_CHANNEL_2:
            scl_port = PORT_B; scl_pin = PIN_10; sda_port = PORT_B; sda_pin = PIN_11; // PB10(SCL), PB11(SDA) AF4
            enable_gpio_clock(PORT_B);
            break;
        case I2C_CHANNEL_3:
            scl_port = PORT_A; scl_pin = PIN_8; sda_port = PORT_C; sda_pin = PIN_9; // PA8(SCL), PC9(SDA) AF4
            enable_gpio_clock(PORT_A); enable_gpio_clock(PORT_C);
            break;
        default: return;
    }

    // Configure GPIO pins for I2C (Alternate Function Open-Drain with Pull-up)
    // SCL
    volatile uint32_t* scl_moder_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(scl_port) + 0x00UL);
    volatile uint32_t* scl_otyper_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(scl_port) + 0x04UL);
    volatile uint32_t* scl_pupdr_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(scl_port) + 0x0CUL);

    *scl_moder_reg &= ~(GPIO_MODER_ANALOG << (scl_pin * 2));
    *scl_moder_reg |= (GPIO_MODER_AF << (scl_pin * 2));
    *scl_otyper_reg |= (GPIO_OTYPER_OPEN_DRAIN << scl_pin);
    *scl_pupdr_reg &= ~(GPIO_PUPDR_PULLDOWN << (scl_pin * 2));
    *scl_pupdr_reg |= (GPIO_PUPDR_PULLUP << (scl_pin * 2)); // Pull-up for SCL
    set_gpio_alternate_function(scl_port, scl_pin, af_val);

    // SDA
    volatile uint32_t* sda_moder_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(sda_port) + 0x00UL);
    volatile uint32_t* sda_otyper_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(sda_port) + 0x04UL);
    volatile uint32_t* sda_pupdr_reg = (volatile uint32_t*)((uint32_t)get_gpio_regs(sda_port) + 0x0CUL);

    *sda_moder_reg &= ~(GPIO_MODER_ANALOG << (sda_pin * 2));
    *sda_moder_reg |= (GPIO_MODER_AF << (sda_pin * 2));
    *sda_otyper_reg |= (GPIO_OTYPER_OPEN_DRAIN << sda_pin);
    *sda_pupdr_reg &= ~(GPIO_PUPDR_PULLDOWN << (sda_pin * 2));
    *sda_pupdr_reg |= (GPIO_PUPDR_PULLUP << (sda_pin * 2)); // Pull-up for SDA
    set_gpio_alternate_function(sda_port, sda_pin, af_val);

    // Disable I2C peripheral during configuration
    *i2c->CR1 &= ~I2C_CR1_PE;

    // Reset I2C peripheral (software reset)
    *i2c->CR1 |= I2C_CR1_SWRST;
    __NOP(); // Short delay
    *i2c->CR1 &= ~I2C_CR1_SWRST;

    // Configure CR2: Peripheral clock frequency
    uint32_t pclk1_freq_mhz = APB1_PERIPH_CLK_FREQ / 1000000UL;
    *i2c->CR2 = pclk1_freq_mhz & 0x3F; // Set FREQ bits

    // Configure Clock Control Register (CCR) for Fast Mode (required by rules.json)
    // T_high + T_low = PCLK1_FREQ / I2C_SPEED
    // Fast mode: duty cycle 2 (T_high/T_low = 2) or duty cycle 16/9 (T_high/T_low = 16/9)
    // For 400kHz, PCLK1=42MHz:
    // f_pclk1 / (2 * f_i2c) = 42MHz / (2 * 400kHz) = 42000 / 800 = 52.5.
    // Use fast mode, duty cycle 2: CCR = (f_pclk1 / (3 * f_i2c))
    // CCR = 42000000 / (3 * 400000) = 35. This is 0x23.
    // TRISE calculation for Fast Mode: (f_pclk1_mhz * 300ns) + 1 = (42 * 0.3) + 1 = 12.6 + 1 = 13.6 -> 14.

    uint32_t ccr_val = 0;
    uint32_t trise_val = 0;

    // Fast Mode (400kHz)
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST)
    {
        ccr_val = APB1_PERIPH_CLK_FREQ / (3 * 400000UL); // CCR = PCLK1 / (3 * I2C_SPEED) for fast mode (T_high/T_low = 2)
        ccr_val |= (1U << 15); // Set F/S bit for Fast Mode
        ccr_val |= (1U << 14); // Set DUTY bit for T_low/T_high = 2 (0b10 in bit 14:15 means duty cycle 2)
        trise_val = (pclk1_freq_mhz * 300UL / 1000U) + 1U; // Trise = (PCLK1_MHz * 300ns) + 1 (max 300ns for fast mode)
        if (trise_val < 2) trise_val = 2; // Minimum value
        if (trise_val > 0x3F) trise_val = 0x3F; // Max value
    }
    else // Standard Mode (100kHz) - fallback if rule wasn't strictly for all modes
    {
        ccr_val = APB1_PERIPH_CLK_FREQ / (2 * 100000UL); // CCR = PCLK1 / (2 * I2C_SPEED) for standard mode
        trise_val = (pclk1_freq_mhz * 1000UL / 1000U) + 1U; // Trise = (PCLK1_MHz * 1000ns) + 1 (max 1000ns for standard mode)
        if (trise_val < 2) trise_val = 2; // Minimum value
        if (trise_val > 0x3F) trise_val = 0x3F; // Max value
    }

    *i2c->CCR = ccr_val;
    *i2c->TRISE = trise_val;

    // Set Own Address 1 (Addressing Mode equals Device Address rule applies)
    // The API uses t_i2c_device_address, but OAR1 itself sets the 7/10 bit mode.
    // For simplicity, we'll assume a fixed 7-bit address for now. The actual address would be passed during connection.
    *i2c->OAR1 &= ~((1U << 15) | (1U << 14)); // Clear ADDMODE and R/W bit
    if (i2c_device_address == I2C_DEVICE_ADDRESS_10BIT)
    {
        *i2c->OAR1 |= (1U << 15); // Set ADDMODE bit for 10-bit address
    }
    *i2c->OAR1 |= (0x00U << 1); // Default own address to 0x00 for now. (Should be a configurable param).
                                 // Addresing Mode equals Device Address: The rule implies that OAR1 should reflect this.
                                 // OAR1 contains the LSB for 7-bit addressing, or 10-bit address.
                                 // The API doesn't provide the actual address, so this is a basic setup.
                                 // For a master, this isn't strictly needed unless it also acts as a slave.

    // Configure ACK (ACK bit in CR1)
    if (i2c_ack == I2C_ACK_ENABLE)
    {
        *i2c->CR1 |= I2C_CR1_ACK;
    }
    else
    {
        *i2c->CR1 &= ~I2C_CR1_ACK;
    }

    // Configure data length (Not explicitly available in CR1 for I2C, always byte-oriented)
    // I2C_datalength parameter is ignored.

    // Enable I2C peripheral
    *i2c->CR1 |= I2C_CR1_PE;

    // Set maximum timeout (No direct register for "max timeout" on STM32 I2C, usually SW controlled)
    // The I2C_SR1_TIMEOUT flag exists, but it's a status flag. Manual software timeouts are common.
}

/**
 * @brief Enables a specific I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    enable_i2c_clock(i2c_channel); // RCC clock enable (inferred)
    *i2c->CR1 |= I2C_CR1_PE; // Enable I2C peripheral
}

/**
 * @brief Disables a specific I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;
    *i2c->CR1 &= ~I2C_CR1_PE; // Disable I2C peripheral
}

/**
 * @brief Sends a single byte over a specific I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send. (Assumes master transmit, address already sent)
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    // Wait for TXE (Transmit data register empty)
    while (!(*i2c->SR1 & I2C_SR1_TXE)) { /* Wait */ }
    *i2c->DR = byte; // Write data to data register
    while (!(*i2c->SR1 & I2C_SR1_BTF)) { /* Wait for Byte Transfer Finished */ }
}

/**
 * @brief Sends a frame (array of bytes) over a specific I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send. (Assumes master transmit, address already sent)
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    // The API implies sending a frame, but the sequence (start, address) is missing.
    // Assuming this function is called after START and ADDRESS has been sent.
    // To comply with "Always generate a repeated start condition instead of stop between transactions",
    // the calling application would need to handle START/STOP/REPEATED START.
    // Here, we just send bytes.

    for (int i = 0; i < length; i++)
    {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over a specific I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string. (Assumes master transmit, address already sent)
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    while (*str != '\0')
    {
        I2C_send_byte(i2c_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from a specific I2C channel.
 * @param i2c_channel The I2C channel to use. (Assumes master receive, address already sent)
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return 0;

    // Wait for RXNE (Receive data register not empty)
    while (!(*i2c->SR1 & I2C_SR1_RXNE)) { /* Wait */ }
    return (tbyte)(*i2c->DR & 0xFF); // Read data from data register
}

/**
 * @brief Receives a frame (array of bytes) from a specific I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive. (Assumes master receive, address already sent)
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset();
    I2C_Regs_t* i2c = get_i2c_regs(i2c_channel);
    if (i2c == (I2C_Regs_t*)NULL) return;

    for (int i = 0; i < max_length; i++)
    {
        if (i == (max_length - 1)) {
            *i2c->CR1 &= ~I2C_CR1_ACK; // Disable ACK for the last byte
        }
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
    }
    // After receiving the last byte, a STOP condition is typically generated by the master,
    // or a repeated start. This would be handled by the higher-level caller.
}

/**
 * @brief Receives a string from a specific I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive.
 * @return The last received byte.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length)
{
    WDT_Reset();
    tbyte received_byte = 0;
    for (int i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        // For I2C, a string would need a termination character defined by protocol,
        // or a fixed length. Here we assume fixed length up to max_length.
        received_byte = I2C_Get_Byte(i2c_channel);
        buffer[i] = (char)received_byte;
        if (received_byte == '\0' || received_byte == '\n' || received_byte == '\r') {
             buffer[i] = '\0'; // Null-terminate if a known terminator is received
             return received_byte;
        }
    }
    buffer[max_length-1] = '\0'; // Ensure null termination
    return received_byte;
}

// SPI (CSI)
/**
 * @brief Initializes a specific SPI channel with the given parameters.
 *        As per Rules.json: Always use fast speed, slave select software-controlled, full duplex, and enable CRC.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master or Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order)
{
    WDT_Reset();

    SPI_Regs_t* spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return;

    // Enable SPI clock and associated GPIO clock
    enable_spi_clock(spi_channel);

    t_port sck_port, miso_port, mosi_port, nss_port;
    t_pin sck_pin, miso_pin, mosi_pin, nss_pin;
    tbyte af_val = 5; // Default AF for SPI1, 2, 3 (check datasheet for specifics)

    // Assign GPIOs and AF for SPI
    switch (spi_channel) {
        case SPI_CHANNEL_1: // PA5(SCK), PA6(MISO), PA7(MOSI), PA4(NSS) - AF5
            sck_port = PORT_A; sck_pin = PIN_5;
            miso_port = PORT_A; miso_pin = PIN_6;
            mosi_port = PORT_A; mosi_pin = PIN_7;
            nss_port = PORT_A; nss_pin = PIN_4;
            af_val = 5;
            enable_gpio_clock(PORT_A);
            break;
        case SPI_CHANNEL_2: // PB10(SCK), PB14(MISO), PB15(MOSI), PB9(NSS) - AF5
            sck_port = PORT_B; sck_pin = PIN_10;
            miso_port = PORT_B; miso_pin = PIN_14;
            mosi_port = PORT_B; mosi_pin = PIN_15;
            nss_port = PORT_B; nss_pin = PIN_9;
            af_val = 5;
            enable_gpio_clock(PORT_B);
            break;
        case SPI_CHANNEL_3: // PB3(SCK), PB4(MISO), PB5(MOSI), PA15(NSS) - AF6
            sck_port = PORT_B; sck_pin = PIN_3;
            miso_port = PORT_B; miso_pin = PIN_4;
            mosi_port = PORT_B; mosi_pin = PIN_5;
            nss_port = PORT_A; nss_pin = PIN_15;
            af_val = 6; // SPI3 uses AF6
            enable_gpio_clock(PORT_A); enable_gpio_clock(PORT_B);
            break;
        default: return;
    }

    // Configure GPIO pins for Alternate Function
    // SCK, MISO, MOSI, NSS (all AF with Push-pull, Very High Speed)
    GPIO_Output_Init(sck_port, sck_pin, 0); // Init as output before AF
    GPIO_Input_Init(miso_port, miso_pin);   // Init as input before AF
    GPIO_Output_Init(mosi_port, mosi_pin, 0); // Init as output before AF
    GPIO_Output_Init(nss_port, nss_pin, 0); // Init as output for software NSS

    volatile uint33_t* sck_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(sck_port) + 0x00UL);
    volatile uint33_t* miso_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(miso_port) + 0x00UL);
    volatile uint33_t* mosi_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(mosi_port) + 0x00UL);
    volatile uint33_t* nss_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(nss_port) + 0x00UL);

    *sck_moder &= ~(GPIO_MODER_ANALOG << (sck_pin * 2)); *sck_moder |= (GPIO_MODER_AF << (sck_pin * 2));
    *miso_moder &= ~(GPIO_MODER_ANALOG << (miso_pin * 2)); *miso_moder |= (GPIO_MODER_AF << (miso_pin * 2));
    *mosi_moder &= ~(GPIO_MODER_ANALOG << (mosi_pin * 2)); *mosi_moder |= (GPIO_MODER_AF << (mosi_pin * 2));
    *nss_moder &= ~(GPIO_MODER_ANALOG << (nss_pin * 2)); *nss_moder |= (GPIO_MODER_AF << (nss_pin * 2));

    set_gpio_alternate_function(sck_port, sck_pin, af_val);
    set_gpio_alternate_function(miso_port, miso_pin, af_val);
    set_gpio_alternate_function(mosi_port, mosi_pin, af_val);
    set_gpio_alternate_function(nss_port, nss_pin, af_val);

    // Disable SPI peripheral during configuration
    *spi->CR1 &= ~SPI_CR1_SPE;

    // Clear CR1 and CR2 for fresh configuration
    *spi->CR1 = 0;
    *spi->CR2 = 0;

    // Master/Slave Mode (MSTR bit in CR1)
    if (spi_mode == SPI_MODE_MASTER) {
        *spi->CR1 |= SPI_CR1_MSTR;
    }

    // Clock Polarity (CPOL bit in CR1)
    if (spi_cpol == SPI_CPOL_HIGH) {
        *spi->CR1 |= SPI_CR1_CPOL;
    }

    // Clock Phase (CPHA bit in CR1)
    if (spi_cpha == SPI_CPHA_2ND_EDGE) {
        *spi->CR1 |= SPI_CR1_CPHA;
    }

    // Data Frame Format (DFF bit in CR1)
    if (spi_dff == SPI_DFF_16_BIT) {
        *spi->CR1 |= SPI_CR1_DFF;
    }

    // Bit Order (LSBFIRST bit in CR1)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        *spi->CR1 |= SPI_CR1_LSBFIRST;
    }

    // Fast Speed (BR bits in CR1) - Set to /2 prescaler for fastest possible
    // PCLK2 for SPI1 (84MHz), PCLK1 for SPI2/3 (42MHz).
    // Min Prescaler is /2.
    // Baud rate control: BR[2:0] = 0b000 -> /2, 0b001 -> /4, ... 0b111 -> /256
    *spi->CR1 &= ~(0x7U << SPI_CR1_BR_Pos); // Clear BR bits
    *spi->CR1 |= (0U << SPI_CR1_BR_Pos);   // Set BR to 0b000 for /2 prescaler

    // Slave Select always software-controlled (SSM and SSI bits in CR1)
    *spi->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI); // Enable software slave management and set internal NSS high

    // Always use full duplex (BIDIMODE = 0, default, BIDIOE = 0)
    // Implicitly full duplex with default settings (no BIDIMODE bit set)

    // Always enable CRC (CRCEN bit in CR1)
    *spi->CR1 |= SPI_CR1_CRCEN;

    // Enable SPI peripheral
    *spi->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Enables a specific SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel)
{
    WDT_Reset();
    SPI_Regs_t* spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return;

    enable_spi_clock(spi_channel); // RCC clock enable (inferred)
    *spi->CR1 |= SPI_CR1_SPE; // Enable SPI peripheral
}

/**
 * @brief Disables a specific SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel)
{
    WDT_Reset();
    SPI_Regs_t* spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return;
    *spi->CR1 &= ~SPI_CR1_SPE; // Disable SPI peripheral
}

/**
 * @brief Sends a single byte over a specific SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte)
{
    WDT_Reset();
    SPI_Regs_t* spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return;

    // Wait until transmit buffer is empty
    while (!(*spi->SR & SPI_SR_TXE)) { /* Wait */ }
    *spi->DR = byte; // Write data to data register
    // For blocking transmit, wait until SPI is not busy
    while (*spi->SR & SPI_SR_BSY) { /* Wait */ }
}

/**
 * @brief Sends a frame (array of bytes) over a specific SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length The number of bytes to send.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length)
{
    WDT_Reset();
    for (int i = 0; i < length; i++)
    {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Receives a single byte from a specific SPI channel.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel)
{
    WDT_Reset();
    SPI_Regs_t* spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return 0;

    // To receive, we must transmit something (e.g., dummy byte) in full-duplex mode
    SPI_Send_Byte(spi_channel, 0x00); // Send dummy byte
    // Wait until receive buffer is not empty
    while (!(*spi->SR & SPI_SR_RXNE)) { /* Wait */ }
    return (tbyte)(*spi->DR & 0xFF); // Read data from data register
}

/**
 * @brief Receives a frame (array of bytes) from a specific SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum number of bytes to receive.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset();
    for (int i = 0; i < max_length; i++)
    {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Receives a string from a specific SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the string to receive.
 * @return The last received byte.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length)
{
    WDT_Reset();
    tbyte received_byte = 0;
    for (int i = 0; i < max_length - 1; i++) // Leave space for null terminator
    {
        received_byte = SPI_Get_Byte(spi_channel);
        buffer[i] = (char)received_byte;
        if (received_byte == '\0' || received_byte == '\n' || received_byte == '\r') {
             buffer[i] = '\0'; // Null-terminate if a known terminator is received
             return received_byte;
        }
    }
    buffer[max_length-1] = '\0'; // Ensure null termination
    return received_byte;
}

// External Interrupt
/**
 * @brief Initializes an external interrupt channel with a specified edge trigger.
 * @param external_int_channel The EXTI line (0-15).
 * @param external_int_edge The trigger edge (Rising, Falling, or Both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge)
{
    WDT_Reset();

    enable_syscfg_clock(); // Enable SYSCFG clock
    t_port gpio_port;
    t_pin gpio_pin = (t_pin)external_int_channel; // EXTI line corresponds to pin number

    // Determine the source GPIO port for the EXTI line.
    // SYSCFG_EXTICRx registers map EXTI lines 0-15 to GPIO ports A-H.
    // EXTIx line is mapped to GPIOx port by writing 0 to 3 to SYSCFG_EXTICRx (x = 0-3 for PA, PB, PC, PD, PE, PH)
    // Example: SYSCFG_EXTICR1 selects sources for EXTI0, EXTI1, EXTI2, EXTI3
    // Assuming generic GPIOA as source if not explicitly mapped for a given channel.
    // For this generic initialization, we will just ensure SYSCFG is clocked and then configure EXTI regs.
    // The actual GPIO port associated with the EXTI channel needs to be externally known or passed.
    // The 'assigned_pin' in register_json for SYSCFG_EXTICRx entries shows which pins can be routed.
    // Let's assume the user will configure the GPIO pin itself for input, and this function configures EXTI.

    // Clear previous EXTI configuration (4 bits per EXTI line in SYSCFG_EXTICRx)
    uint32_t exti_reg_idx = external_int_channel / 4;
    uint32_t exti_bit_pos = (external_int_channel % 4) * 4;
    volatile uint32_t *exti_config_reg = NULL; // SYSCFG_EXTICR1-4
    switch (exti_reg_idx) {
        case 0: exti_config_reg = &SYSCFG_EXTICR1; break;
        case 1: exti_config_reg = &SYSCFG_EXTICR2; break;
        case 2: exti_config_reg = &SYSCFG_EXTICR3; break;
        case 3: exti_config_reg = &SYSCFG_EXTICR4; break;
        default: return; // Invalid EXTI channel
    }
    *exti_config_reg &= ~(0xFUL << exti_bit_pos);
    // For now, setting to GPIOA (0b0000). The specific port would need to be a parameter.
    // Example: *exti_config_reg |= (GPIO_PORT_CODE << exti_bit_pos);

    // Clear pending bit (writing 1 clears the bit)
    EXTI_PR = EXTI_PR_BIT(external_int_channel);

    // Configure trigger edge
    EXTI_RTSR &= ~EXTI_RTSR_BIT(external_int_channel); // Clear rising trigger
    EXTI_FTSR &= ~EXTI_FTSR_BIT(external_int_channel); // Clear falling trigger

    switch (external_int_edge) {
        case EXTERNAL_INT_EDGE_RISING:
            EXTI_RTSR |= EXTI_RTSR_BIT(external_int_channel);
            break;
        case EXTERNAL_INT_EDGE_FALLING:
            EXTI_FTSR |= EXTI_FTSR_BIT(external_int_channel);
            break;
        case EXTERNAL_INT_EDGE_RISING_FALLING:
            EXTI_RTSR |= EXTI_RTSR_BIT(external_int_channel);
            EXTI_FTSR |= EXTI_FTSR_BIT(external_int_channel);
            break;
        default: break;
    }
}

/**
 * @brief Enables a specific external interrupt channel.
 * @param external_int_channel The EXTI line to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel)
{
    WDT_Reset();
    // Enable interrupt mask for the EXTI line
    EXTI_IMR |= EXTI_IMR_BIT(external_int_channel);
    // Note: NVIC interrupt enable would also be required (e.g., NVIC_EnableIRQ(EXTI_x_IRQn)).
    // This is typically handled outside MCAL if generic or for specific EXTI lines.
}

/**
 * @brief Disables a specific external interrupt channel.
 * @param external_int_channel The EXTI line to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel)
{
    WDT_Reset();
    // Disable interrupt mask for the EXTI line
    EXTI_IMR &= ~EXTI_IMR_BIT(external_int_channel);
    // Note: NVIC interrupt disable would also be required (e.g., NVIC_DisableIRQ(EXTI_x_IRQn)).
}

// GPIO
/**
 * @brief Initializes a specific GPIO pin as an output.
 *        As per Rules.json: All output pins have pull-up resistors disabled, set value before direction.
 *        For current registers: use >=20mA sink current & >=10mA source current (Very High Speed).
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The initial output value (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset();

    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return;

    enable_gpio_clock(port);

    // 1. Set initial value before setting direction
    if (value == 0) {
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x18UL) ) = (1UL << (pin + 16)); // BSRR - reset bit
    } else {
        *( (volatile uint32_t*)( (uint32_t)gpio + 0x18UL) ) = (1UL << pin);      // BSRR - set bit
    }
    // Verify initial value
    volatile uint32_t current_odr = *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) );
    while (((current_odr >> pin) & 1U) != value) {
        current_odr = *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) );
    }

    // 2. Configure output type: Push-pull
    volatile uint32_t* otyper_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x04UL);
    *otyper_reg &= ~(GPIO_OTYPER_OPEN_DRAIN << pin); // Set to Push-pull (0)

    // 3. Configure output speed: Very High Speed (>=20mA sink / >=10mA source)
    volatile uint32_t* ospeedr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x08UL);
    *ospeedr_reg &= ~(GPIO_OSPEEDR_VERY_HIGH << (pin * 2)); // Clear bits
    *ospeedr_reg |= (GPIO_OSPEEDR_VERY_HIGH << (pin * 2)); // Set to Very High Speed (3)

    // 4. Configure pull-up/pull-down: No pull-up/pull-down for output pins
    volatile uint32_t* pupdr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL);
    *pupdr_reg &= ~(GPIO_PUPDR_PULLDOWN << (pin * 2)); // Set to No pull-up/pull-down (0)

    // 5. Configure mode: General purpose output mode
    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x00UL);
    *moder_reg &= ~(GPIO_MODER_ANALOG << (pin * 2)); // Clear mode bits
    *moder_reg |= (GPIO_MODER_OUTPUT << (pin * 2)); // Set to Output mode (1)

    // Verify direction
    volatile uint32_t current_moder = *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) );
    while (((current_moder >> (pin * 2)) & GPIO_MODER_ANALOG) != GPIO_MODER_OUTPUT) {
        current_moder = *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) );
    }
}

/**
 * @brief Initializes a specific GPIO pin as an input.
 *        As per Rules.json: All input pins have pull-up resistors and wakeup feature enabled (if available).
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin)
{
    WDT_Reset();

    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return;

    enable_gpio_clock(port);

    // Configure mode: Input mode
    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x00UL);
    *moder_reg &= ~(GPIO_MODER_ANALOG << (pin * 2)); // Set to Input mode (0)

    // Configure pull-up/pull-down: Pull-up resistors
    volatile uint32_t* pupdr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x0CUL);
    *pupdr_reg &= ~(GPIO_PUPDR_PULLDOWN << (pin * 2)); // Clear bits
    *pupdr_reg |= (GPIO_PUPDR_PULLUP << (pin * 2));    // Set to Pull-up (1)

    // Other configurations (OTYPER, OSPEEDR) are irrelevant for input, set to default 0s

    // Verify direction
    volatile uint32_t current_moder = *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) );
    while (((current_moder >> (pin * 2)) & GPIO_MODER_ANALOG) != GPIO_MODER_INPUT) {
        current_moder = *( (volatile uint32_t*)( (uint32_t)gpio + 0x00UL) );
    }

    // Wakeup feature enabled (if available) - this typically refers to EXTI configuration.
    // This function only configures the GPIO. EXTI should be configured separately if wakeup is needed.
}

/**
 * @brief Gets the current direction of a specific GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction (input or output).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin)
{
    WDT_Reset();
    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return GPIO_DIRECTION_INPUT; // Default to input on error

    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x00UL);
    uint32_t mode = (*moder_reg >> (pin * 2)) & GPIO_MODER_ANALOG;

    if (mode == GPIO_MODER_OUTPUT) {
        return GPIO_DIRECTION_OUTPUT;
    } else {
        return GPIO_DIRECTION_INPUT; // Includes AF and Analog modes as non-output
    }
}

/**
 * @brief Sets the output value of a specific GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value)
{
    WDT_Reset();
    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return;

    // Use BSRR for atomic set/reset
    volatile uint32_t* bsrr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x18UL);
    if (value == 0) {
        *bsrr_reg = (1UL << (pin + 16)); // Write to BRx bit (reset bit)
    } else {
        *bsrr_reg = (1UL << pin);      // Write to BSx bit (set bit)
    }

    // Verify value
    volatile uint32_t current_odr = *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) );
    while (((current_odr >> pin) & 1U) != value) {
        current_odr = *( (volatile uint32_t*)( (uint32_t)gpio + 0x14UL) );
    }
}

/**
 * @brief Gets the input value of a specific GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The input value (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin)
{
    WDT_Reset();
    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return 0; // Default to 0 on error

    volatile uint32_t* idr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x10UL);
    return (tbyte)((*idr_reg >> pin) & 1U);
}

/**
 * @brief Toggles the output value of a specific GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin)
{
    WDT_Reset();
    GPIO_Regs_t* gpio = get_gpio_regs(port);
    if (gpio == (GPIO_Regs_t*)NULL) return;

    volatile uint32_t* odr_reg = (volatile uint32_t*)( (uint32_t)gpio + 0x14UL);
    *odr_reg ^= (1UL << pin); // Toggle the bit

    // No explicit verification loop for toggle, as it's typically done by reading
    // back the ODR. A short delay might be needed for external components.
}

// PWM
/**
 * @brief Initializes a specific PWM channel.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The initial PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;
    uint32_t channel_idx; // 0 for CH1, 1 for CH2, etc.
    uint32_t timer_clock_freq;
    t_port gpio_port;
    t_pin gpio_pin;
    tbyte gpio_af;

    // Determine timer, channel index, GPIO, and AF based on pwm_channel enum
    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:  timer_channel = TIMER_CHANNEL_1; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_8; gpio_af = 1; break; // PA8 AF1
        case PWM_CHANNEL_TIM1_CH2:  timer_channel = TIMER_CHANNEL_1; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_9; gpio_af = 1; break; // PA9 AF1
        case PWM_CHANNEL_TIM1_CH3:  timer_channel = TIMER_CHANNEL_1; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_10; gpio_af = 1; break; // PA10 AF1
        case PWM_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_11; gpio_af = 1; break; // PA11 AF1

        case PWM_CHANNEL_TIM2_CH1:  timer_channel = TIMER_CHANNEL_2; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_0; gpio_af = 1; break; // PA0 AF1
        case PWM_CHANNEL_TIM2_CH2:  timer_channel = TIMER_CHANNEL_2; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_1; gpio_af = 1; break; // PA1 AF1
        case PWM_CHANNEL_TIM2_CH3:  timer_channel = TIMER_CHANNEL_2; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 1; break; // PA2 AF1
        case PWM_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 1; break; // PA3 AF1

        case PWM_CHANNEL_TIM3_CH1:  timer_channel = TIMER_CHANNEL_3; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_6; gpio_af = 2; break; // PA6 AF2
        case PWM_CHANNEL_TIM3_CH2:  timer_channel = TIMER_CHANNEL_3; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_7; gpio_af = 2; break; // PA7 AF2
        case PWM_CHANNEL_TIM3_CH3:  timer_channel = TIMER_CHANNEL_3; channel_idx = 2; gpio_port = PORT_B; gpio_pin = PIN_0; gpio_af = 2; break; // PB0 AF2
        case PWM_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; channel_idx = 3; gpio_port = PORT_B; gpio_pin = PIN_1; gpio_af = 2; break; // PB1 AF2

        case PWM_CHANNEL_TIM4_CH1:  timer_channel = TIMER_CHANNEL_4; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_6; gpio_af = 2; break; // PB6 AF2
        case PWM_CHANNEL_TIM4_CH2:  timer_channel = TIMER_CHANNEL_4; channel_idx = 1; gpio_port = PORT_B; gpio_pin = PIN_7; gpio_af = 2; break; // PB7 AF2
        case PWM_CHANNEL_TIM4_CH3:  timer_channel = TIMER_CHANNEL_4; channel_idx = 2; gpio_port = PORT_B; gpio_pin = PIN_8; gpio_af = 2; break; // PB8 AF2
        case PWM_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; channel_idx = 3; gpio_port = PORT_B; gpio_pin = PIN_9; gpio_af = 2; break; // PB9 AF2

        case PWM_CHANNEL_TIM5_CH1:  timer_channel = TIMER_CHANNEL_5; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_0; gpio_af = 2; break; // PA0 AF2
        case PWM_CHANNEL_TIM5_CH2:  timer_channel = TIMER_CHANNEL_5; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_1; gpio_af = 2; break; // PA1 AF2
        case PWM_CHANNEL_TIM5_CH3:  timer_channel = TIMER_CHANNEL_5; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 2; break; // PA2 AF2
        case PWM_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 2; break; // PA3 AF2

        case PWM_CHANNEL_TIM9_CH1:  timer_channel = TIMER_CHANNEL_9; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 3; break; // PA2 AF3
        case PWM_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 3; break; // PA3 AF3

        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_8; gpio_af = 3; break; // PB8 AF3

        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_9; gpio_af = 3; break; // PB9 AF3

        default: return; // Invalid PWM channel
    }

    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    // 1. Enable timer clock and associated GPIO clock
    enable_timer_clock(timer_channel);
    enable_gpio_clock(gpio_port);

    // 2. Configure GPIO pin to Alternate Function mode
    GPIO_Output_Init(gpio_port, gpio_pin, 0); // Init as output first
    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)get_gpio_regs(gpio_port) + 0x00UL);
    *moder_reg &= ~(GPIO_MODER_ANALOG << (gpio_pin * 2)); // Clear mode bits
    *moder_reg |= (GPIO_MODER_AF << (gpio_pin * 2));     // Set AF mode
    set_gpio_alternate_function(gpio_port, gpio_pin, gpio_af);

    // Determine timer clock frequency (APB1/APB2 timer clocks can be x2 of peripheral clock)
    if (timer_channel == TIMER_CHANNEL_1 || timer_channel == TIMER_CHANNEL_9 ||
        timer_channel == TIMER_CHANNEL_10 || timer_channel == TIMER_CHANNEL_11) {
        timer_clock_freq = APB2_TIMER_CLK_FREQ; // APB2 timers
    } else {
        timer_clock_freq = APB1_TIMER_CLK_FREQ; // APB1 timers
    }

    // Disable timer before configuration
    *tim->CR1 &= ~TIM_CR1_CEN;

    // 3. Compute prescaler (PSC) and auto-reload (ARR) values
    // Fpwm = Timer_Clock / ((PSC+1) * (ARR+1))
    // To minimize error and achieve desired frequency, choose ARR first (e.g., max 0xFFFF for 16-bit timers)
    // or a fixed value for resolution (e.g., 999 for 1000 steps).
    // Let's target ARR = 999 for 1000 steps resolution (0-999)

    uint32_t prescaler = 0;
    uint32_t arr_value = 999; // 1000 steps for duty cycle

    // Ensure pwm_khz_freq is not zero to prevent division by zero
    if (pwm_khz_freq == 0) pwm_khz_freq = 1; // Default to 1 kHz if 0 is given

    uint32_t target_freq_hz = (uint32_t)pwm_khz_freq * 1000;
    if (target_freq_hz == 0) target_freq_hz = 1; // Prevent division by zero

    prescaler = (timer_clock_freq / (target_freq_hz * (arr_value + 1))) - 1;

    // Clamp prescaler to max 16-bit value (0xFFFF)
    if (prescaler > 0xFFFF) {
        prescaler = 0xFFFF;
        // Recalculate ARR if prescaler maxed out
        arr_value = (timer_clock_freq / (target_freq_hz * (prescaler + 1))) - 1;
        if (arr_value > 0xFFFF) arr_value = 0xFFFF; // ARR also max 16-bit
    } else if (prescaler < 0) { // Should not happen with unsigned but for safety
        prescaler = 0;
    }
    if (arr_value < 0) arr_value = 0; // Should not happen

    // Check if the calculated frequency is achievable within typical timer limits
    // Max PWM Freq = Timer_Clock / (1 * 1) = Timer_Clock (prescaler 0, ARR 0)
    // Min PWM Freq = Timer_Clock / (65536 * 65536) (prescaler max, ARR max)
    // Max Freq = 84MHz. Min Freq = 84MHz / (65536*65536) ~ 0.019 Hz.
    // The given pwm_khz_freq should be achievable within this range.

    *tim->PSC = (uint32_t)prescaler;
    *tim->ARR = (uint32_t)arr_value;

    // 4. Configure TIMx_CR1 (Up-counting, ARPE enabled)
    *tim->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS_Pos); // Clear direction and center-aligned mode bits
    *tim->CR1 |= TIM_CR1_ARPE; // Enable Auto-reload preload

    // 5. Configure the timer channel in PWM Mode (OCxM bits), enable preload (OCxPE)
    volatile uint32_t* ccmr_reg;
    uint32_t ccmr_offset;
    if (channel_idx < 2) { // CCMR1 for CH1/CH2
        ccmr_reg = (volatile uint32_t*)((uint32_t)tim + 0x18UL); // TIMx_CCMR1
        ccmr_offset = channel_idx * 8; // Each channel takes 8 bits in CCMR1
    } else { // CCMR2 for CH3/CH4
        ccmr_reg = (volatile uint32_t*)((uint32_t)tim + 0x1CUL); // TIMx_CCMR2
        ccmr_offset = (channel_idx - 2) * 8;
    }
    *ccmr_reg &= ~(0x7FU << ccmr_offset); // Clear CCxS and OCxM bits
    *ccmr_reg |= (TIM_CCMR_OCM_PWM1 << ccmr_offset); // Set PWM Mode 1
    *ccmr_reg |= (TIM_CCMR_OCPE << (ccmr_offset + 3)); // Enable output compare preload (OCxPE bit)

    // 6. Set Capture/Compare Register (CCR) for initial duty cycle
    uint32_t compare_value = (arr_value + 1) * pwm_duty / 100;
    switch (channel_idx) {
        case 0: *tim->CCR1 = compare_value; break;
        case 1: *tim->CCR2 = compare_value; break;
        case 2: *tim->CCR3 = compare_value; break;
        case 3: *tim->CCR4 = compare_value; break;
        default: break;
    }

    // 7. Enable the channel output (CCxE in TIMx_CCER)
    volatile uint32_t* ccer_reg = (volatile uint32_t*)((uint32_t)tim + 0x20UL); // TIMx_CCER
    *ccer_reg |= (TIM_CCER_CCxE << (channel_idx * 4)); // Enable CCx output

    // 8. For advanced timers (TIM1), enable Main Output Enable (MOE) in BDTR
    if (timer_channel == TIMER_CHANNEL_1) {
        volatile uint32_t* bdtr_reg = (volatile uint32_t*)((uint32_t)tim + 0x44UL); // TIM1_BDTR
        *bdtr_reg |= TIM_BDTR_MOE;
    }

    // 9. Generate Update Event to load PSC and ARR values
    volatile uint32_t* egr_reg = (volatile uint32_t*)((uint32_t)tim + 0x14UL); // TIMx_EGR
    *egr_reg |= (1U << 0); // UG bit (Update Generation)
}

/**
 * @brief Starts the PWM output for a specific channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:  case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:  case PWM_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM2_CH1:  case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:  case PWM_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM3_CH1:  case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:  case PWM_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM4_CH1:  case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:  case PWM_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM5_CH1:  case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:  case PWM_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM9_CH1:  case PWM_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; break;
        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; break;
        default: return;
    }
    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    *tim->CR1 |= TIM_CR1_CEN; // Enable counter
}

/**
 * @brief Stops the PWM output for a specific channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;

    switch (pwm_channel) {
        case PWM_CHANNEL_TIM1_CH1:  case PWM_CHANNEL_TIM1_CH2:
        case PWM_CHANNEL_TIM1_CH3:  case PWM_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; break;
        case PWM_CHANNEL_TIM2_CH1:  case PWM_CHANNEL_TIM2_CH2:
        case PWM_CHANNEL_TIM2_CH3:  case PWM_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; break;
        case PWM_CHANNEL_TIM3_CH1:  case PWM_CHANNEL_TIM3_CH2:
        case PWM_CHANNEL_TIM3_CH3:  case PWM_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; break;
        case PWM_CHANNEL_TIM4_CH1:  case PWM_CHANNEL_TIM4_CH2:
        case PWM_CHANNEL_TIM4_CH3:  case PWM_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; break;
        case PWM_CHANNEL_TIM5_CH1:  case PWM_CHANNEL_TIM5_CH2:
        case PWM_CHANNEL_TIM5_CH3:  case PWM_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; break;
        case PWM_CHANNEL_TIM9_CH1:  case PWM_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; break;
        case PWM_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; break;
        case PWM_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; break;
        default: return;
    }
    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    *tim->CR1 &= ~TIM_CR1_CEN; // Disable counter
}

// ICU
/**
 * @brief Initializes a specific Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller The prescaler for the input capture.
 * @param icu_edge The edge trigger for capture (Rising, Falling, or Both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;
    uint32_t channel_idx; // 0 for CH1, 1 for CH2, etc.
    t_port gpio_port;
    t_pin gpio_pin;
    tbyte gpio_af;

    // Determine timer, channel index, GPIO, and AF based on icu_channel enum
    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:  timer_channel = TIMER_CHANNEL_1; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_8; gpio_af = 1; break;
        case ICU_CHANNEL_TIM1_CH2:  timer_channel = TIMER_CHANNEL_1; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_9; gpio_af = 1; break;
        case ICU_CHANNEL_TIM1_CH3:  timer_channel = TIMER_CHANNEL_1; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_10; gpio_af = 1; break;
        case ICU_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_11; gpio_af = 1; break;

        case ICU_CHANNEL_TIM2_CH1:  timer_channel = TIMER_CHANNEL_2; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_0; gpio_af = 1; break;
        case ICU_CHANNEL_TIM2_CH2:  timer_channel = TIMER_CHANNEL_2; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_1; gpio_af = 1; break;
        case ICU_CHANNEL_TIM2_CH3:  timer_channel = TIMER_CHANNEL_2; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 1; break;
        case ICU_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 1; break;

        case ICU_CHANNEL_TIM3_CH1:  timer_channel = TIMER_CHANNEL_3; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_6; gpio_af = 2; break;
        case ICU_CHANNEL_TIM3_CH2:  timer_channel = TIMER_CHANNEL_3; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_7; gpio_af = 2; break;
        case ICU_CHANNEL_TIM3_CH3:  timer_channel = TIMER_CHANNEL_3; channel_idx = 2; gpio_port = PORT_B; gpio_pin = PIN_0; gpio_af = 2; break;
        case ICU_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; channel_idx = 3; gpio_port = PORT_B; gpio_pin = PIN_1; gpio_af = 2; break;

        case ICU_CHANNEL_TIM4_CH1:  timer_channel = TIMER_CHANNEL_4; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_6; gpio_af = 2; break;
        case ICU_CHANNEL_TIM4_CH2:  timer_channel = TIMER_CHANNEL_4; channel_idx = 1; gpio_port = PORT_B; gpio_pin = PIN_7; gpio_af = 2; break;
        case ICU_CHANNEL_TIM4_CH3:  timer_channel = TIMER_CHANNEL_4; channel_idx = 2; gpio_port = PORT_B; gpio_pin = PIN_8; gpio_af = 2; break;
        case ICU_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; channel_idx = 3; gpio_port = PORT_B; gpio_pin = PIN_9; gpio_af = 2; break;

        case ICU_CHANNEL_TIM5_CH1:  timer_channel = TIMER_CHANNEL_5; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_0; gpio_af = 2; break;
        case ICU_CHANNEL_TIM5_CH2:  timer_channel = TIMER_CHANNEL_5; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_1; gpio_af = 2; break;
        case ICU_CHANNEL_TIM5_CH3:  timer_channel = TIMER_CHANNEL_5; channel_idx = 2; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 2; break;
        case ICU_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; channel_idx = 3; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 2; break;

        case ICU_CHANNEL_TIM9_CH1:  timer_channel = TIMER_CHANNEL_9; channel_idx = 0; gpio_port = PORT_A; gpio_pin = PIN_2; gpio_af = 3; break;
        case ICU_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; channel_idx = 1; gpio_port = PORT_A; gpio_pin = PIN_3; gpio_af = 3; break;

        case ICU_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_8; gpio_af = 3; break;

        case ICU_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; channel_idx = 0; gpio_port = PORT_B; gpio_pin = PIN_9; gpio_af = 3; break;

        default: return; // Invalid ICU channel
    }

    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    // 1. Enable timer clock and associated GPIO clock
    enable_timer_clock(timer_channel);
    enable_gpio_clock(gpio_port);

    // 2. Configure GPIO pin to Alternate Function (input) mode
    GPIO_Input_Init(gpio_port, gpio_pin); // Init as input first
    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)get_gpio_regs(gpio_port) + 0x00UL);
    *moder_reg &= ~(GPIO_MODER_ANALOG << (gpio_pin * 2)); // Clear mode bits
    *moder_reg |= (GPIO_MODER_AF << (gpio_pin * 2));     // Set AF mode
    set_gpio_alternate_function(gpio_port, gpio_pin, gpio_af);

    // Disable timer before configuration
    *tim->CR1 &= ~TIM_CR1_CEN;

    // 3. Configure TIMx_PSC for prescaler (input capture prescaler)
    // The ICU prescaler (ICxPSC in CCMRx) is different from the main timer prescaler.
    // The input capture prescaler samples the input at the given prescaler value.
    // The API's `icu_prescaller` enum seems to refer to the input capture prescaler.
    // Example: ICxPSC 00=no prescaler, 01=capture every 2 events, 10=every 4 events, 11=every 8 events.
    // The enum values are Div1, Div2, Div4, Div8, ... Div1024 which suggests a general timer prescaler.
    // Assuming this refers to the main timer prescaler.
    *tim->PSC = (uint32_t)icu_prescaller; // Assign directly if enum maps to specific prescaler values

    // Set auto-reload register to max for maximum period detection
    *tim->ARR = 0xFFFF; // Max value for 16-bit timers, or 0xFFFFFFFF for 32-bit (TIM2, TIM5)
                        // For STM32F401, TIM2/5 are 32-bit. Others are 16-bit.
    if (timer_channel == TIMER_CHANNEL_2 || timer_channel == TIMER_CHANNEL_5) {
        *tim->ARR = 0xFFFFFFFFUL;
    }

    // 4. Configure TIMx_CCMRx for Input Capture mode (ICxS bits)
    volatile uint32_t* ccmr_reg;
    uint32_t ccmr_offset;
    if (channel_idx < 2) { // CCMR1 for CH1/CH2
        ccmr_reg = (volatile uint32_t*)((uint32_t)tim + 0x18UL); // TIMx_CCMR1
        ccmr_offset = channel_idx * 8; // Each channel takes 8 bits in CCMR1
    } else { // CCMR2 for CH3/CH4
        ccmr_reg = (volatile uint32_t*)((uint32_t)tim + 0x1CUL); // TIMx_CCMR2
        ccmr_offset = (channel_idx - 2) * 8;
    }

    // Clear CCxS, ICxPSC, ICxF
    *ccmr_reg &= ~(0xFFFFUL << ccmr_offset); // Clear relevant bits (16 bits per channel in CCMRx)
    *ccmr_reg |= (TIM_CCMR_CCxS_INPUT_TI1 << ccmr_offset); // Set channel as input mapped on TIx

    // 5. Configure TIMx_CCER for input polarity (rising/falling/both)
    volatile uint32_t* ccer_reg = (volatile uint32_t*)((uint32_t)tim + 0x20UL); // TIMx_CCER
    *ccer_reg &= ~( (TIM_CCER_CCxE | TIM_CCER_CCxP) << (channel_idx * 4) ); // Clear enable and polarity bits
    // Note: STM32 uses CCxP and CCxNP for rising/falling.
    // CCxP=0, CCxNP=0: Rising edge
    // CCxP=1, CCxNP=0: Falling edge
    // CCxP=1, CCxNP=1: Both edges (for advanced timers, some others might use different method)

    switch (icu_edge) {
        case ICU_EDGE_RISING:
            // No bits set for CCxP/CCxNP for rising edge in CCER
            break;
        case ICU_EDGE_FALLING:
            *ccer_reg |= (TIM_CCER_CCxP << (channel_idx * 4)); // Set CCxP for falling edge
            break;
        case ICU_EDGE_BOTH:
            *ccer_reg |= ((TIM_CCER_CCxP | TIM_CCER_CCxNE) << (channel_idx * 4)); // Set CCxP and CCxNE for both edges
            break;
        default: break;
    }

    // 6. Enable Capture/Compare interrupt for the selected channel
    volatile uint32_t* dier_reg = (volatile uint32_t*)((uint32_t)tim + 0x0CUL); // TIMx_DIER
    *dier_reg |= (TIM_DIER_CC1IE << channel_idx); // Enable CCxIE interrupt

    // 7. Enable counter (TIMx_CR1.CEN will be set by ICU_Enable)
}

/**
 * @brief Enables a specific Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;
    uint32_t channel_idx;

    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:  timer_channel = TIMER_CHANNEL_1; channel_idx = 0; break;
        case ICU_CHANNEL_TIM1_CH2:  timer_channel = TIMER_CHANNEL_1; channel_idx = 1; break;
        case ICU_CHANNEL_TIM1_CH3:  timer_channel = TIMER_CHANNEL_1; channel_idx = 2; break;
        case ICU_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; channel_idx = 3; break;

        case ICU_CHANNEL_TIM2_CH1:  timer_channel = TIMER_CHANNEL_2; channel_idx = 0; break;
        case ICU_CHANNEL_TIM2_CH2:  timer_channel = TIMER_CHANNEL_2; channel_idx = 1; break;
        case ICU_CHANNEL_TIM2_CH3:  timer_channel = TIMER_CHANNEL_2; channel_idx = 2; break;
        case ICU_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; channel_idx = 3; break;

        case ICU_CHANNEL_TIM3_CH1:  timer_channel = TIMER_CHANNEL_3; channel_idx = 0; break;
        case ICU_CHANNEL_TIM3_CH2:  timer_channel = TIMER_CHANNEL_3; channel_idx = 1; break;
        case ICU_CHANNEL_TIM3_CH3:  timer_channel = TIMER_CHANNEL_3; channel_idx = 2; break;
        case ICU_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; channel_idx = 3; break;

        case ICU_CHANNEL_TIM4_CH1:  timer_channel = TIMER_CHANNEL_4; channel_idx = 0; break;
        case ICU_CHANNEL_TIM4_CH2:  timer_channel = TIMER_CHANNEL_4; channel_idx = 1; break;
        case ICU_CHANNEL_TIM4_CH3:  timer_channel = TIMER_CHANNEL_4; channel_idx = 2; break;
        case ICU_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; channel_idx = 3; break;

        case ICU_CHANNEL_TIM5_CH1:  timer_channel = TIMER_CHANNEL_5; channel_idx = 0; break;
        case ICU_CHANNEL_TIM5_CH2:  timer_channel = TIMER_CHANNEL_5; channel_idx = 1; break;
        case ICU_CHANNEL_TIM5_CH3:  timer_channel = TIMER_CHANNEL_5; channel_idx = 2; break;
        case ICU_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; channel_idx = 3; break;

        case ICU_CHANNEL_TIM9_CH1:  timer_channel = TIMER_CHANNEL_9; channel_idx = 0; break;
        case ICU_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; channel_idx = 1; break;

        case ICU_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; channel_idx = 0; break;

        case ICU_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; channel_idx = 0; break;

        default: return;
    }
    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    enable_timer_clock(timer_channel); // RCC clock enable (inferred)

    // Enable the Capture/Compare channel
    volatile uint32_t* ccer_reg = (volatile uint32_t*)((uint32_t)tim + 0x20UL); // TIMx_CCER
    *ccer_reg |= (TIM_CCER_CCxE << (channel_idx * 4)); // Enable CCx input

    // Enable the timer counter
    *tim->CR1 |= TIM_CR1_CEN;
    // Note: NVIC interrupt enable for the corresponding TIM_IRQn would also be necessary.
}

/**
 * @brief Disables a specific Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel)
{
    WDT_Reset();

    TIM_Regs_t* tim = (TIM_Regs_t*)NULL;
    t_timer_channel timer_channel;
    uint32_t channel_idx;

    switch (icu_channel) {
        case ICU_CHANNEL_TIM1_CH1:  timer_channel = TIMER_CHANNEL_1; channel_idx = 0; break;
        case ICU_CHANNEL_TIM1_CH2:  timer_channel = TIMER_CHANNEL_1; channel_idx = 1; break;
        case ICU_CHANNEL_TIM1_CH3:  timer_channel = TIMER_CHANNEL_1; channel_idx = 2; break;
        case ICU_CHANNEL_TIM1_CH4:  timer_channel = TIMER_CHANNEL_1; channel_idx = 3; break;

        case ICU_CHANNEL_TIM2_CH1:  timer_channel = TIMER_CHANNEL_2; channel_idx = 0; break;
        case ICU_CHANNEL_TIM2_CH2:  timer_channel = TIMER_CHANNEL_2; channel_idx = 1; break;
        case ICU_CHANNEL_TIM2_CH3:  timer_channel = TIMER_CHANNEL_2; channel_idx = 2; break;
        case ICU_CHANNEL_TIM2_CH4:  timer_channel = TIMER_CHANNEL_2; channel_idx = 3; break;

        case ICU_CHANNEL_TIM3_CH1:  timer_channel = TIMER_CHANNEL_3; channel_idx = 0; break;
        case ICU_CHANNEL_TIM3_CH2:  timer_channel = TIMER_CHANNEL_3; channel_idx = 1; break;
        case ICU_CHANNEL_TIM3_CH3:  timer_channel = TIMER_CHANNEL_3; channel_idx = 2; break;
        case ICU_CHANNEL_TIM3_CH4:  timer_channel = TIMER_CHANNEL_3; channel_idx = 3; break;

        case ICU_CHANNEL_TIM4_CH1:  timer_channel = TIMER_CHANNEL_4; channel_idx = 0; break;
        case ICU_CHANNEL_TIM4_CH2:  timer_channel = TIMER_CHANNEL_4; channel_idx = 1; break;
        case ICU_CHANNEL_TIM4_CH3:  timer_channel = TIMER_CHANNEL_4; channel_idx = 2; break;
        case ICU_CHANNEL_TIM4_CH4:  timer_channel = TIMER_CHANNEL_4; channel_idx = 3; break;

        case ICU_CHANNEL_TIM5_CH1:  timer_channel = TIMER_CHANNEL_5; channel_idx = 0; break;
        case ICU_CHANNEL_TIM5_CH2:  timer_channel = TIMER_CHANNEL_5; channel_idx = 1; break;
        case ICU_CHANNEL_TIM5_CH3:  timer_channel = TIMER_CHANNEL_5; channel_idx = 2; break;
        case ICU_CHANNEL_TIM5_CH4:  timer_channel = TIMER_CHANNEL_5; channel_idx = 3; break;

        case ICU_CHANNEL_TIM9_CH1:  timer_channel = TIMER_CHANNEL_9; channel_idx = 0; break;
        case ICU_CHANNEL_TIM9_CH2:  timer_channel = TIMER_CHANNEL_9; channel_idx = 1; break;

        case ICU_CHANNEL_TIM10_CH1: timer_channel = TIMER_CHANNEL_10; channel_idx = 0; break;

        case ICU_CHANNEL_TIM11_CH1: timer_channel = TIMER_CHANNEL_11; channel_idx = 0; break;

        default: return;
    }
    tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    // Disable the Capture/Compare channel
    volatile uint32_t* ccer_reg = (volatile uint32_t*)((uint32_t)tim + 0x20UL); // TIMx_CCER
    *ccer_reg &= ~(TIM_CCER_CCxE << (channel_idx * 4));

    // Disable Capture/Compare interrupt for the selected channel
    volatile uint32_t* dier_reg = (volatile uint32_t*)((uint32_t)tim + 0x0CUL); // TIMx_DIER
    *dier_reg &= ~(TIM_DIER_CC1IE << channel_idx);

    // If no other channels are active, disable the timer counter.
    // This is a simplification; in a real scenario, you'd check other DIER bits.
    // *tim->CR1 &= ~TIM_CR1_CEN;
    // Note: NVIC interrupt disable would also be required for the TIM_IRQn.
}

/**
 * @brief Gets the frequency measured by an ICU channel.
 *        As per Rules.json: "Get frequency when edge happens". This implies an ISR would store the value.
 *        Since ISRs are not generated here, this function will return a placeholder value.
 * @param icu_channel The ICU channel to read from.
 * @return The measured frequency in Hz.
 */
float ICU_GetFrequency(t_icu_channel icu_channel)
{
    WDT_Reset();
    // This function requires an underlying interrupt service routine (ISR)
    // to capture timer values on edges and calculate frequency.
    // Since ISRs are not generated, this will be a placeholder.
    // In a real application, the ISR would update a global variable that this function reads.
    (void)icu_channel; // Suppress unused parameter warning
    return 0.0f; // Placeholder
}

/**
 * @brief Sets a callback function to be executed on an ICU event.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void))
{
    WDT_Reset();
    icu_callback_ptr = callback;
    // Note: The actual timer ISR would need to call this callback.
}

// Timer
/**
 * @brief Initializes a specific timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel)
{
    WDT_Reset();

    TIM_Regs_t* tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    enable_timer_clock(timer_channel); // Enable timer clock

    // Disable timer before configuration
    *tim->CR1 &= ~TIM_CR1_CEN;

    // Configure CR1: Up-counting mode, Auto-reload preload enable
    *tim->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS_Pos); // Up-counting
    *tim->CR1 |= TIM_CR1_ARPE;

    // Set prescaler and auto-reload to default/max values for initial state.
    // Max timer clock for APB1 timers is 84MHz, for APB2 timers is 84MHz.
    // For 1ms tick: PSC = 84000-1, ARR = 1-1 (for 1 tick) or ARR = some_value-1 for a counter.
    // Let's set a default prescaler and ARR for a general purpose counter, e.g., 1ms tick rate.
    uint32_t timer_clock_freq;
    if (timer_channel == TIMER_CHANNEL_1 || timer_channel == TIMER_CHANNEL_9 ||
        timer_channel == TIMER_CHANNEL_10 || timer_channel == TIMER_CHANNEL_11) {
        timer_clock_freq = APB2_TIMER_CLK_FREQ; // APB2 timers
    } else {
        timer_clock_freq = APB1_TIMER_CLK_FREQ; // APB1 timers
    }

    // Default to 1ms period (prescaler for 1MHz, ARR for 1000 counts)
    uint32_t prescaler = (timer_clock_freq / 1000000UL) - 1; // Prescaler for 1MHz counter clock
    uint32_t arr_value = 999; // 1000 counts for 1ms period

    *tim->PSC = prescaler;
    *tim->ARR = arr_value;

    // Clear counter
    *tim->CNT = 0;

    // Generate Update Event to load PSC and ARR values
    volatile uint32_t* egr_reg = (volatile uint32_t*)((uint32_t)tim + 0x14UL); // TIMx_EGR
    *egr_reg |= (1U << 0); // UG bit (Update Generation)
}

/**
 * @brief Sets a specific timer channel to generate an interrupt after a specified number of microseconds.
 * @param timer_channel The timer channel to configure.
 * @param time The time in microseconds (us).
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time)
{
    WDT_Reset();

    TIM_Regs_t* tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;
    if (time == 0) time = 1; // Prevent division by zero

    uint32_t timer_clock_freq;
    if (timer_channel == TIMER_CHANNEL_1 || timer_channel == TIMER_CHANNEL_9 ||
        timer_channel == TIMER_CHANNEL_10 || timer_channel == TIMER_CHANNEL_11) {
        timer_clock_freq = APB2_TIMER_CLK_FREQ; // APB2 timers
    } else {
        timer_clock_freq = APB1_TIMER_CLK_FREQ; // APB1 timers
    }

    uint32_t prescaler = 0;
    uint32_t arr_value = 0;

    // Calculate prescaler for 1 us tick (1 MHz counter frequency)
    prescaler = (timer_clock_freq / 1000000UL) - 1;
    arr_value = time - 1; // ARR counts up to 'time'

    // Clamp prescaler to 16-bit
    if (prescaler > 0xFFFF) prescaler = 0xFFFF;
    if (prescaler < 0) prescaler = 0;

    // Clamp ARR to max for 16-bit or 32-bit timers
    if (timer_channel == TIMER_CHANNEL_2 || timer_channel == TIMER_CHANNEL_5) {
        if (arr_value > 0xFFFFFFFFUL) arr_value = 0xFFFFFFFFUL;
    } else {
        if (arr_value > 0xFFFF) arr_value = 0xFFFF;
    }
    if (arr_value < 0) arr_value = 0;


    *tim->PSC = prescaler;
    *tim->ARR = arr_value;
    *tim->CNT = 0; // Reset counter
    volatile uint32_t* egr_reg = (volatile uint32_t*)((uint32_t)tim + 0x14UL);
    *egr_reg |= (1U << 0); // Generate Update Event
}

/**
 * @brief Sets a specific timer channel to generate an interrupt after a specified number of milliseconds.
 * @param timer_channel The timer channel to configure.
 * @param time The time in milliseconds (ms).
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time)
{
    WDT_Reset();
    TIMER_Set_us(timer_channel, (tlong)time * 1000);
}

/**
 * @brief Sets a specific timer channel to generate an interrupt after a specified number of seconds.
 * @param timer_channel The timer channel to configure.
 * @param time The time in seconds (sec).
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset();
    TIMER_Set_us(timer_channel, (tlong)time * 1000 * 1000);
}

/**
 * @brief Sets a specific timer channel to generate an interrupt after a specified number of minutes.
 * @param timer_channel The timer channel to configure.
 * @param time The time in minutes (min).
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset();
    TIMER_Set_Time_sec(timer_channel, (tbyte)((tlong)time * 60));
}

/**
 * @brief Sets a specific timer channel to generate an interrupt after a specified number of hours.
 * @param timer_channel The timer channel to configure.
 * @param time The time in hours (hour).
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time)
{
    WDT_Reset();
    TIMER_Set_Time_min(timer_channel, (tbyte)((tlong)time * 60));
}

/**
 * @brief Enables a specific timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel)
{
    WDT_Reset();
    TIM_Regs_t* tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;

    enable_timer_clock(timer_channel); // RCC clock enable (inferred)
    *tim->CR1 |= TIM_CR1_CEN; // Enable counter
}

/**
 * @brief Disables a specific timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel)
{
    WDT_Reset();
    TIM_Regs_t* tim = get_timer_regs(timer_channel);
    if (tim == (TIM_Regs_t*)NULL) return;
    *tim->CR1 &= ~TIM_CR1_CEN; // Disable counter
}

// ADC
/**
 * @brief Initializes the ADC peripheral for a specific channel and mode.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode The ADC mode (polling or interrupt).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode)
{
    WDT_Reset();

    ADC_Regs_t* adc = get_adc_regs(); // Only ADC1
    if (adc == (ADC_Regs_t*)NULL) return;

    // 1. Enable ADC clock and associated GPIO clock
    enable_adc_clock();
    t_port gpio_port;
    t_pin gpio_pin;

    // Determine GPIO port and pin for the ADC channel (only analog mode)
    // Assigned pins from register_json "ADC1_SMPR2" for channels 0-9 and "ADC1_SMPR1" for channels 10-18
    switch (adc_channel) {
        case ADC_CHANNEL_0: gpio_port = PORT_A; gpio_pin = PIN_0; break;
        case ADC_CHANNEL_1: gpio_port = PORT_A; gpio_pin = PIN_1; break;
        case ADC_CHANNEL_2: gpio_port = PORT_A; gpio_pin = PIN_2; break;
        case ADC_CHANNEL_3: gpio_port = PORT_A; gpio_pin = PIN_3; break;
        case ADC_CHANNEL_4: gpio_port = PORT_A; gpio_pin = PIN_4; break;
        case ADC_CHANNEL_5: gpio_port = PORT_A; gpio_pin = PIN_5; break;
        case ADC_CHANNEL_6: gpio_port = PORT_A; gpio_pin = PIN_6; break;
        case ADC_CHANNEL_7: gpio_port = PORT_A; gpio_pin = PIN_7; break;
        case ADC_CHANNEL_8: gpio_port = PORT_B; gpio_pin = PIN_0; break;
        case ADC_CHANNEL_9: gpio_port = PORT_B; gpio_pin = PIN_1; break;
        case ADC_CHANNEL_10: gpio_port = PORT_C; gpio_pin = PIN_0; break;
        case ADC_CHANNEL_11: gpio_port = PORT_C; gpio_pin = PIN_1; break;
        case ADC_CHANNEL_12: gpio_port = PORT_C; gpio_pin = PIN_2; break;
        case ADC_CHANNEL_13: gpio_port = PORT_C; gpio_pin = PIN_3; break;
        case ADC_CHANNEL_14: gpio_port = PORT_C; gpio_pin = PIN_4; break;
        case ADC_CHANNEL_15: gpio_port = PORT_C; gpio_pin = PIN_5; break;
        case ADC_CHANNEL_16: // Internal Temp Sensor
        case ADC_CHANNEL_17: // Internal VrefInt
        case ADC_CHANNEL_18: // Internal Vbat
            // No GPIO configuration needed for internal channels, but enable_gpio_clock() might still be called
            // if other GPIOs are used for ADC (e.g., if this function supports multiple channels)
            return; // Skip GPIO for internal channels for this simplified impl
        default: return;
    }
    enable_gpio_clock(gpio_port);

    // Configure GPIO pin to Analog mode
    volatile uint32_t* moder_reg = (volatile uint32_t*)( (uint32_t)get_gpio_regs(gpio_port) + 0x00UL);
    *moder_reg &= ~(GPIO_MODER_ANALOG << (gpio_pin * 2)); // Clear mode bits
    *moder_reg |= (GPIO_MODER_ANALOG << (gpio_pin * 2));     // Set Analog mode (3)
    // No pull-up/pull-down needed for analog input

    // Disable ADC during configuration
    *adc->CR2 &= ~ADC_CR2_ADON;

    // 2. Configure ADC_CCR (Common Control Register) - prescaler for ADCCLK
    // ADCCLK = PCLK2 / prescaler. PCLK2 = 84MHz. Max ADCCLK = 36MHz.
    // 84MHz / 4 = 21MHz (safe). So, prescaler /4.
    ADC_CCR &= ~(ADC_CCR_ADCPRE_DIV8); // Clear bits
    ADC_CCR |= ADC_CCR_ADCPRE_DIV4;    // Set ADC prescaler to /4

    // 3. Configure ADC1_CR1 (resolution, scan mode, watchdog)
    // For simplicity, using default 12-bit resolution. Disable scan, watchdog.
    *adc->CR1 = 0; // Reset CR1
    if (adc_mode == ADC_MODE_INTERRUPT) {
        *adc->CR1 |= (1U << 5); // EOCIE: End of Conversion Interrupt Enable
    }

    // 4. Configure ADC1_CR2 (data alignment, continuous mode, DMA, external trigger)
    *adc->CR2 = 0; // Reset CR2
    // Right alignment (default), disable DMA (for polling/interrupt), software trigger.
    if (adc_mode == ADC_MODE_POLLING) {
        *adc->CR2 &= ~ADC_CR2_CONT; // Single conversion mode
    } else { // Interrupt mode
        *adc->CR2 |= ADC_CR2_CONT; // Continuous conversion mode for repeated interrupts
    }

    // 5. Configure ADC1_SMPR1/ADC1_SMPR2 (sample time)
    // Recommended sample time based on ADC clock for full accuracy.
    // Let's use 480 cycles (max for internal channels) or 3 cycles for external.
    uint32_t smpr_val = 0; // Use 3 cycles (0b000) for fastest conversion.
    uint32_t smp_shift;

    if (adc_channel < ADC_CHANNEL_10) { // Channels 0-9 in SMPR2
        smp_shift = (adc_channel % 10) * 3;
        *adc->SMPR2 &= ~(0x7UL << smp_shift);
        *adc->SMPR2 |= (smpr_val << smp_shift);
    } else { // Channels 10-18 in SMPR1
        smp_shift = (adc_channel - ADC_CHANNEL_10) * 3;
        *adc->SMPR1 &= ~(0x7UL << smp_shift);
        *adc->SMPR1 |= (smpr_val << smp_shift);
    }

    // 6. Configure ADC1_SQRx for regular sequence
    // For single channel conversion, L=0 (1 conversion), SQ1 = adc_channel.
    *adc->SQR1 &= ~(0xFUL << 20); // Clear L bits (sequence length, L=0 means 1 conversion)
    *adc->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    *adc->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to the current channel

    // 7. Enable ADC
    *adc->CR2 |= ADC_CR2_ADON;
}

/**
 * @brief Enables the ADC peripheral.
 * @param adc_channel The ADC channel (ignored, enables ADC1).
 */
void ADC_Enable(t_adc_channel adc_channel)
{
    WDT_Reset();
    ADC_Regs_t* adc = get_adc_regs();
    if (adc == (ADC_Regs_t*)NULL) return;

    enable_adc_clock(); // RCC clock enable (inferred)
    (void)adc_channel; // Suppress unused parameter warning
    *adc->CR2 |= ADC_CR2_ADON; // Enable ADC
}

/**
 * @brief Disables the ADC peripheral.
 * @param adc_channel The ADC channel (ignored, disables ADC1).
 */
void ADC_Disable(t_adc_channel adc_channel)
{
    WDT_Reset();
    ADC_Regs_t* adc = get_adc_regs();
    if (adc == (ADC_Regs_t*)NULL) return;
    (void)adc_channel; // Suppress unused parameter warning
    *adc->CR2 &= ~ADC_CR2_ADON; // Disable ADC
}

/**
 * @brief Gets an ADC conversion result using polling.
 * @param adc_channel The ADC channel to read.
 * @return The converted 12-bit digital value.
 */
tword ADC_Get_POLLING(t_adc_channel adc_channel)
{
    WDT_Reset();
    ADC_Regs_t* adc = get_adc_regs();
    if (adc == (ADC_Regs_t*)NULL) return 0;

    // For single conversion, reconfigure SQR3 each time if channel changes
    *adc->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    *adc->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to the current channel

    *adc->CR2 |= ADC_CR2_SWSTART; // Start regular conversion
    while (!(*adc->SR & ADC_SR_EOC)) { /* Wait for End of Conversion */ }
    tword result = (tword)(*adc->DR & 0xFFF); // Read the 12-bit result
    *adc->SR &= ~ADC_SR_EOC; // Clear EOC flag
    return result;
}

/**
 * @brief Gets an ADC conversion result using interrupts.
 *        This function initiates a conversion and expects an ISR to handle the result.
 *        Since ISRs are not generated here, this function will primarily set up for interrupt
 *        and potentially return a placeholder, or the last known value from an ISR.
 * @param adc_channel The ADC channel to read.
 * @return The converted 12-bit digital value.
 */
tword ADC_Get_INTERRUPT(t_adc_channel adc_channel)
{
    WDT_Reset();
    ADC_Regs_t* adc = get_adc_regs();
    if (adc == (ADC_Regs_t*)NULL) return 0;

    // For single conversion, reconfigure SQR3 each time if channel changes
    *adc->SQR3 &= ~(0x1FUL << 0); // Clear SQ1 bits
    *adc->SQR3 |= ((uint32_t)adc_channel << 0); // Set SQ1 to the current channel

    // This function assumes that ADC_Init has configured EOCIE and NVIC for ADC interrupts.
    // Start conversion
    *adc->CR2 |= ADC_CR2_SWSTART;
    // In an interrupt-driven system, the result would be read in the ADC ISR.
    // For this API, we can either return a dummy value or a globally stored last conversion.
    // Let's return 0 as a placeholder, as no ISR logic is here to update a global var.
    return 0; // Placeholder; actual value comes from ISR.
}

// Internal_EEPROM not supported on STM32F401RC (as noted in MCAL.h)

// TT (Time Triggered OS)
// For TT, we need a timer for the tick. TIM2/TIM5 are 32-bit, others 16-bit.
// Let's use TIM2 as the system tick timer.
// Assuming F_CPU = 84MHz.
// For a 1ms tick:
// Prescaler for 1MHz = (84MHz / 1MHz) - 1 = 83.
// ARR for 1ms = (1MHz * 1ms) - 1 = 999.
static tbyte current_task_count = 0;

/**
 * @brief Initializes the Time Triggered (TT) OS with a specified tick time.
 *        Uses TIM2 for the system tick.
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms)
{
    WDT_Reset();

    TIM_Regs_t* tim2 = get_timer_regs(TIMER_CHANNEL_2);
    if (tim2 == (TIM_Regs_t*)NULL) return;

    // Store the actual tick time
    switch (tick_time_ms) {
        case TICK_TIME_1MS:   TT_Tick_ms = 1;     break;
        case TICK_TIME_5MS:   TT_Tick_ms = 5;     break;
        case TICK_TIME_10MS:  TT_Tick_ms = 10;    break;
        case TICK_TIME_100MS: TT_Tick_ms = 100;   break;
        case TICK_TIME_1S:    TT_Tick_ms = 1000;  break;
        default: TT_Tick_ms = 1; break; // Default to 1ms
    }

    enable_timer_clock(TIMER_CHANNEL_2); // Enable TIM2 clock

    // Disable TIM2 during configuration
    *tim2->CR1 &= ~TIM_CR1_CEN;

    uint32_t timer_clock_freq = APB1_TIMER_CLK_FREQ; // TIM2 uses APB1 clock

    // Calculate PSC and ARR for the desired tick_time_ms
    uint32_t desired_freq_hz = 1000U / TT_Tick_ms; // e.g., for 1ms tick, desired_freq = 1000Hz
    uint32_t prescaler = (timer_clock_freq / (desired_freq_hz * 1000U)) - 1; // Prescaler for 1kHz counter
    uint32_t arr_value = 999; // ARR for 1kHz, 1000 counts

    // If prescaler becomes too large, adjust ARR.
    // Given the constraints and desired tick_time_ms, a 1kHz counter is reasonable.
    // e.g., 84MHz / (83+1) = 1MHz. Then ARR = (1MHz * TT_Tick_ms/1000) - 1.
    prescaler = (timer_clock_freq / 1000U) - 1; // Prescaler for 1kHz counter clock
    arr_value = (1000U * TT_Tick_ms / 1000U) - 1; // ARR value for the specified millisecond tick

    // Clamp values if necessary (TIM2 is 32-bit, so large values are okay)
    if (prescaler > 0xFFFF) prescaler = 0xFFFF; // Prescaler is 16-bit
    if (arr_value > 0xFFFFFFFFUL) arr_value = 0xFFFFFFFFUL;

    *tim2->PSC = prescaler;
    *tim2->ARR = arr_value;
    *tim2->CNT = 0; // Clear counter

    // Configure TIM2_CR1: Up-counting mode, Auto-reload preload enable
    *tim2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS_Pos);
    *tim2->CR1 |= TIM_CR1_ARPE;

    // Enable Update Interrupt
    volatile uint32_t* dier_reg = (volatile uint32_t*)((uint32_t)tim2 + 0x0CUL);
    *dier_reg |= TIM_DIER_UIE;

    // Generate Update Event to load PSC and ARR
    volatile uint32_t* egr_reg = (volatile uint32_t*)((uint32_t)tim2 + 0x14UL);
    *egr_reg |= (1U << 0); // UG bit

    // Initialize task list
    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        TT_Tasks[i].pTask = NULL;
        TT_Tasks[i].delay = 0;
        TT_Tasks[i].period = 0;
        TT_Tasks[i].run_me = false;
    }
    current_task_count = 0;

    // Note: NVIC_EnableIRQ(TIM2_IRQn) must be called (likely in an OS-specific config)
}

/**
 * @brief Starts the Time Triggered (TT) OS scheduler.
 */
void TT_Start(void)
{
    WDT_Reset();

    TIM_Regs_t* tim2 = get_timer_regs(TIMER_CHANNEL_2);
    if (tim2 == (TIM_Regs_t*)NULL) return;

    *tim2->CR1 |= TIM_CR1_CEN; // Enable TIM2 counter
}

/**
 * @brief Dispatches tasks based on their scheduled delays and periods.
 */
void TT_Dispatch_task(void)
{
    WDT_Reset();
    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        if (TT_Tasks[i].pTask != NULL && TT_Tasks[i].run_me == true) {
            TT_Tasks[i].pTask(); // Execute the task
            TT_Tasks[i].run_me = false; // Reset 'run_me' flag
            // Reschedule if periodic
            if (TT_Tasks[i].period > 0) {
                TT_Tasks[i].delay = TT_Tasks[i].period;
            } else { // Delete if one-shot
                TT_Delete_task(i);
            }
        }
    }
}

/**
 * @brief The Interrupt Service Routine (ISR) for the TT OS tick.
 *        This function should be called by the timer ISR (e.g., TIM2_IRQHandler).
 */
void TT_ISR(void)
{
    // WDT_Reset() is typically not called from within a critical ISR section to avoid delays.
    // It's usually handled by a main loop or a less frequent task.

    // Clear the timer update interrupt flag
    TIM_Regs_t* tim2 = get_timer_regs(TIMER_CHANNEL_2);
    if (tim2 != (TIM_Regs_t*)NULL) {
        if (*tim2->SR & TIM_SR_UIF) {
            *tim2->SR &= ~TIM_SR_UIF;

            // Increment tick counter and update task delays
            for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
                if (TT_Tasks[i].pTask != NULL) {
                    if (TT_Tasks[i].delay == 0) {
                        TT_Tasks[i].run_me = true;
                    } else {
                        TT_Tasks[i].delay--;
                    }
                }
            }
        }
    }
}

/**
 * @brief Adds a new task to the TT OS scheduler.
 * @param task Pointer to the task function.
 * @param period The period of the task in ticks (0 for one-shot).
 * @param delay The initial delay before the first execution in ticks.
 * @return The index of the added task, or 0xFF if the task list is full.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay)
{
    WDT_Reset();
    if (current_task_count < TT_MAX_TASKS) {
        tbyte i = 0;
        for (i = 0; i < TT_MAX_TASKS; i++) {
            if (TT_Tasks[i].pTask == NULL) { // Find an empty slot
                TT_Tasks[i].pTask = task;
                TT_Tasks[i].delay = delay / TT_Tick_ms; // Convert to actual ticks
                TT_Tasks[i].period = period / TT_Tick_ms; // Convert to actual ticks
                TT_Tasks[i].run_me = false;
                current_task_count++;
                return i;
            }
        }
    }
    return 0xFF; // Task list full
}

/**
 * @brief Deletes a task from the TT OS scheduler.
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index)
{
    WDT_Reset();
    if (task_index < TT_MAX_TASKS && TT_Tasks[task_index].pTask != NULL) {
        TT_Tasks[task_index].pTask = NULL;
        TT_Tasks[task_index].delay = 0;
        TT_Tasks[task_index].period = 0;
        TT_Tasks[task_index].run_me = false;
        current_task_count--;
    }
}

// MCAL_OUTPUT_BUZZER not supported on STM32F401RC (as noted in MCAL.h)

// WDT
// WDT_Init and WDT_Reset are already implemented under MCU CONFIG and are globally available.
// WDT_Reset is also a required part of all API bodies.

/**
 * @brief Initializes the Watchdog Timer (WDT).
 *        This function is primarily handled by MCU_Config_Init.
 */
void WDT_Init(void)
{
    WDT_Reset();
    // IWDG_KR = 0x5555; // Enable write access to PR and RLR
    // IWDG_PR = 0x0;    // Set prescaler to /4
    // IWDG_RLR = 63;    // Reload value for ~8ms
    // IWDG_KR = 0xAAAA; // Reload the watchdog counter
    // IWDG_KR = 0xCCCC; // Start watchdog
    // This functionality is already covered in MCU_Config_Init.
}

// DAC not supported on STM32F401RC (as noted in MCAL.h)

// I2S
/**
 * @brief Initializes a specific I2S channel.
 * @param channel The I2S channel to initialize.
 * @param mode The I2S mode (master/slave, transmit/receive).
 * @param standard The I2S standard (Philips, MSB, LSB, PCM).
 * @param data_format The data length (16-bit, 24-bit, 32-bit).
 * @param channel_mode The channel mode (stereo, mono).
 * @param sample_rate The audio sample rate (e.g., 48kHz).
 * @param mclk_freq The master clock frequency (if MCLK output is enabled).
 * @param dma_buffer_size The size of the DMA buffer (if DMA is used).
 */
void I2S_Init(t_i2s_channel channel, I2S_Mode_t mode, I2S_Standard_t standard, I2S_DataFormat_t data_format, I2S_ChannelMode_t channel_mode, uint32_t sample_rate, uint32_t mclk_freq, uint32_t dma_buffer_size)
{
    WDT_Reset();

    SPI_Regs_t* spi = (SPI_Regs_t*)NULL;
    t_spi_channel spi_channel;
    t_port ws_port, ck_port, sd_port, mck_port;
    t_pin ws_pin, ck_pin, sd_pin, mck_pin;
    tbyte af_val = 5; // Default for SPI1/2, AF6 for SPI3

    // Determine associated SPI peripheral and GPIOs
    switch (channel) {
        case I2S_CHANNEL_1: // SPI1: PA4 (WS), PA5 (CK), PA7 (SD), PB0 (MCK) - AF5 (WS, CK, SD), AF5 (MCK)
            spi_channel = SPI_CHANNEL_1;
            ws_port = PORT_A; ws_pin = PIN_4;
            ck_port = PORT_A; ck_pin = PIN_5;
            sd_port = PORT_A; sd_pin = PIN_7;
            mck_port = PORT_B; mck_pin = PIN_0; // Not in register_json, infer common AF for I2S MCK
            af_val = 5;
            enable_gpio_clock(PORT_A); enable_gpio_clock(PORT_B);
            break;
        case I2S_CHANNEL_2: // SPI2: PB12 (WS), PB13 (CK), PB15 (SD), PC6 (MCK) - AF5
            spi_channel = SPI_CHANNEL_2;
            ws_port = PORT_B; ws_pin = PIN_12;
            ck_port = PORT_B; ck_pin = PIN_13;
            sd_port = PORT_B; sd_pin = PIN_15;
            mck_port = PORT_C; mck_pin = PIN_6; // Not in register_json, infer common AF for I2S MCK
            af_val = 5;
            enable_gpio_clock(PORT_B); enable_gpio_clock(PORT_C);
            break;
        case I2S_CHANNEL_3: // SPI3: PA4 (WS), PB3 (CK), PB5 (SD), PC6 (MCK) - AF6
            spi_channel = SPI_CHANNEL_3;
            ws_port = PORT_A; ws_pin = PIN_4; // or PA15 for WS
            ck_port = PORT_B; ck_pin = PIN_3;
            sd_port = PORT_B; sd_pin = PIN_5;
            mck_port = PORT_C; mck_pin = PIN_6; // Not in register_json, infer common AF for I2S MCK
            af_val = 6;
            enable_gpio_clock(PORT_A); enable_gpio_clock(PORT_B); enable_gpio_clock(PORT_C);
            break;
        default: return;
    }
    spi = get_spi_regs(spi_channel);
    if (spi == (SPI_Regs_t*)NULL) return;

    // Enable SPI peripheral clock
    enable_spi_clock(spi_channel);

    // Configure GPIOs for Alternate Function
    GPIO_Output_Init(ws_port, ws_pin, 0);
    GPIO_Output_Init(ck_port, ck_pin, 0);
    GPIO_Output_Init(sd_port, sd_pin, 0);
    GPIO_Output_Init(mck_port, mck_pin, 0);

    volatile uint32_t* ws_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(ws_port) + 0x00UL);
    volatile uint32_t* ck_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(ck_port) + 0x00UL);
    volatile uint32_t* sd_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(sd_port) + 0x00UL);
    volatile uint32_t* mck_moder = (volatile uint32_t*)((uint32_t)get_gpio_regs(mck_port) + 0x00UL);

    *ws_moder &= ~(GPIO_MODER_ANALOG << (ws_pin * 2)); *ws_moder |= (GPIO_MODER_AF << (ws_pin * 2));
    *ck_moder &= ~(GPIO_MODER_ANALOG << (ck_pin * 2)); *ck_moder |= (GPIO_MODER_AF << (ck_pin * 2));
    *sd_moder &= ~(GPIO_MODER_ANALOG << (sd_pin * 2)); *sd_moder |= (GPIO_MODER_AF << (sd_pin * 2));
    *mck_moder &= ~(GPIO_MODER_ANALOG << (mck_pin * 2)); *mck_moder |= (GPIO_MODER_AF << (mck_pin * 2));

    set_gpio_alternate_function(ws_port, ws_pin, af_val);
    set_gpio_alternate_function(ck_port, ck_pin, af_val);
    set_gpio_alternate_function(sd_port, sd_pin, af_val);
    set_gpio_alternate_function(mck_port, mck_pin, af_val);

    // Disable I2S before configuration
    *spi->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // Configure SPIx_I2SCFGR (I2S mode enable, standard, data format, channel mode)
    *spi->I2SCFGR &= ~(SPI_I2SCFGR_I2SMOD | (0x3U << SPI_I2SCFGR_I2SCFG_Pos) | (0x3U << SPI_I2SCFGR_DATLEN_Pos) | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_CKPOL | (0x3U << SPI_I2SCFGR_I2SSTD_Pos));
    *spi->I2SCFGR |= SPI_I2SCFGR_I2SMOD; // Set I2SMOD bit to enable I2S mode

    // I2S Configuration (Master/Slave, Tx/Rx)
    uint32_t i2scfg_bits = 0;
    switch (mode) {
        case I2S_MODE_SLAVE_TX:  i2scfg_bits = SPI_I2SCFGR_I2SCFG_SLAVE_TX;  break;
        case I2S_MODE_SLAVE_RX:  i2scfg_bits = SPI_I2SCFGR_I2SCFG_SLAVE_RX;  break;
        case I2S_MODE_MASTER_TX: i2scfg_bits = SPI_I2SCFGR_I2SCFG_MASTER_TX; break;
        case I2S_MODE_MASTER_RX: i2scfg_bits = SPI_I2SCFGR_I2SCFG_MASTER_RX; break;
        default: /* default to master tx */ i2scfg_bits = SPI_I2SCFGR_I2SCFG_MASTER_TX; break;
    }
    *spi->I2SCFGR |= i2scfg_bits;

    // I2S Standard
    uint32_t i2sstd_bits = 0;
    switch (standard) {
        case I2S_STANDARD_PHILIPS: i2sstd_bits = SPI_I2SCFGR_I2SSTD_PHILIPS; break;
        case I2S_STANDARD_MSB:     i2sstd_bits = SPI_I2SCFGR_I2SSTD_MSB;     break;
        case I2S_STANDARD_LSB:     i2sstd_bits = SPI_I2SCFGR_I2SSTD_LSB;     break;
        case I2S_STANDARD_PCM_SHORT: // PCM can be short or long
        case I2S_STANDARD_PCM_LONG:  i2sstd_bits = SPI_I2SCFGR_I2SSTD_PCM;   break;
        default: /* default to Philips */ i2sstd_bits = SPI_I2SCFGR_I2SSTD_PHILIPS; break;
    }
    *spi->I2SCFGR |= i2sstd_bits;
    if (standard == I2S_STANDARD_PCM_LONG) { // Assuming PCMSYNC for long frame
        *spi->I2SCFGR |= SPI_I2SCFGR_PCMSYNC;
    }

    // Data Format
    uint32_t datlen_bits = 0;
    switch (data_format) {
        case I2S_DATAFORMAT_16B:           datlen_bits = SPI_I2SCFGR_DATLEN_16B;     break;
        case I2S_DATAFORMAT_16B_EXTENDED:  datlen_bits = (SPI_I2SCFGR_DATLEN_16B | SPI_I2SCFGR_CHLEN); break; // 16-bit data, 32-bit channel frame (extended)
        case I2S_DATAFORMAT_24B:           datlen_bits = SPI_I2SCFGR_DATLEN_24B;     break;
        case I2S_DATAFORMAT_32B:           datlen_bits = SPI_I2SCFGR_DATLEN_32B;     break;
        default: /* default to 16B */      datlen_bits = SPI_I2SCFGR_DATLEN_16B;     break;
    }
    *spi->I2SCFGR |= datlen_bits;

    // Channel Mode (Mono/Stereo - CHLEN bit typically defines 16-bit or 32-bit channel length.
    // 0: 16-bit/channel, 1: 32-bit/channel. Affects stereo data.
    if (channel_mode == I2S_CHANNELMODE_MONO) {
        // Mono mode implies 16-bit data on one channel, so CHLEN is 0 (16-bit)
        *spi->I2SCFGR &= ~SPI_I2SCFGR_CHLEN;
    } else { // Stereo
        // Stereo usually implies 32-bit channel frame for 16-bit data or 64-bit for 32-bit data etc.
        // If data_format is 16B, then CHLEN=1 (32bit frame) is standard for stereo.
        if (data_format == I2S_DATAFORMAT_16B) {
            *spi->I2SCFGR |= SPI_I2SCFGR_CHLEN; // 32-bit channel frame
        }
    }


    // Configure SPIx_I2SPR (prescaler for sample rate, MCLK enable)
    // The I2S clock (I2Sx_CK) is derived from PLLI2S_R (PLLI2SCFGR.PLLI2SR) or PLL_R.
    // I2SPR.I2SDIV and I2SPR.ODD determine the final clock frequency.
    // fs = I2S_Clock / ( (16*2*((I2SDIV*2)+ODD)) for 16-bit data, etc.)
    // For specific sample rates, precise PLLI2S/PLL configuration is usually needed.
    // Given no direct control over PLLI2S from register_json, we approximate.
    uint32_t i2s_clock_freq = F_CPU; // Assuming system clock for simplicity (PLLI2SR not in JSON)
    uint32_t i2sdiv = 2; // Default smallest div
    uint32_t odd = 0;

    // A full calculation for sample rate involves the PLLI2S_R (RCC_PLLI2SCFGR).
    // Given only high-level registers, a precise match to `sample_rate` is complex.
    // We will set a default prescaler that gives a common audio frequency.
    // For 48kHz, with 256*fs (master clock output), and typical PLLI2S_R.
    // For example, PLLI2S_VCO = (HSE_VALUE / PLLM) * PLLI2SN (PLLI2S = 256 for STM32F4)
    // PLLI2S_R = PLLI2S_VCO / PLLI2SR
    // For now, setting a simple prescaler, acknowledging limitations.
    (void)sample_rate; // Suppress unused parameter warning

    *spi->I2SPR = 0; // Clear prescaler
    *spi->I2SPR |= ((i2sdiv & 0xFF) << SPI_I2SPR_I2SDIV_Pos); // Set I2SDIV
    if (odd) *spi->I2SPR |= SPI_I2SPR_ODD; // Set ODD bit

    // MCLK output enable (if mclk_freq is non-zero and for master mode)
    if (mclk_freq > 0 && (mode == I2S_MODE_MASTER_TX || mode == I2S_MODE_MASTER_RX)) {
        *spi->I2SPR |= SPI_I2SPR_MCKOE;
    }

    // DMA buffer size (if DMA is intended, configure CR2.RXDMAEN/TXDMAEN)
    (void)dma_buffer_size; // Suppress unused parameter
    // If DMA were used, enable specific DMA channels and link to SPIx_DR for transfers.

    // Enable I2S peripheral
    *spi->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

/**
 * @brief Enables a specific I2S channel.
 * @param channel The I2S channel to enable.
 */
void I2S_Enable(t_i2s_channel channel)
{
    WDT_Reset();
    SPI_Regs_t* spi = (SPI_Regs_t*)NULL;
    switch (channel) {
        case I2S_CHANNEL_1: spi = get_spi_regs(SPI_CHANNEL_1); break;
        case I2S_CHANNEL_2: spi = get_spi_regs(SPI_CHANNEL_2); break;
        case I2S_CHANNEL_3: spi = get_spi_regs(SPI_CHANNEL_3); break;
        default: return;
    }
    if (spi == (SPI_Regs_t*)NULL) return;

    enable_spi_clock(channel); // RCC clock enable (inferred)
    *spi->I2SCFGR |= SPI_I2SCFGR_I2SE; // Enable I2S
}

/**
 * @brief Transmits data over a specific I2S channel.
 * @param channel The I2S channel to use.
 * @param data Pointer to the data to transmit.
 * @param length The number of data units (e.g., samples) to transmit.
 */
void I2S_Transmit(t_i2s_channel channel, const void *data, size_t length)
{
    WDT_Reset();
    SPI_Regs_t* spi = (SPI_Regs_t*)NULL;
    switch (channel) {
        case I2S_CHANNEL_1: spi = get_spi_regs(SPI_CHANNEL_1); break;
        case I2S_CHANNEL_2: spi = get_spi_regs(SPI_CHANNEL_2); break;
        case I2S_CHANNEL_3: spi = get_spi_regs(SPI_CHANNEL_3); break;
        default: return;
    }
    if (spi == (SPI_Regs_t*)NULL) return;

    const uint16_t* tx_data = (const uint16_t*)data; // Assuming 16-bit data for simplicity
    for (size_t i = 0; i < length; i++) {
        // Wait until transmit buffer is empty
        while (!(*spi->SR & SPI_SR_TXE)) { /* Wait */ }
        *spi->DR = tx_data[i]; // Write data
    }
    // For blocking transmit, wait until SPI is not busy
    while (*spi->SR & SPI_SR_BSY) { /* Wait */ }
}

/**
 * @brief Receives data from a specific I2S channel.
 * @param channel The I2S channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param length The number of data units (e.g., samples) to receive.
 */
void I2S_Receive(t_i2s_channel channel, void *buffer, size_t length)
{
    WDT_Reset();
    SPI_Regs_t* spi = (SPI_Regs_t*)NULL;
    switch (channel) {
        case I2S_CHANNEL_1: spi = get_spi_regs(SPI_CHANNEL_1); break;
        case I2S_CHANNEL_2: spi = get_spi_regs(SPI_CHANNEL_2); break;
        case I2S_CHANNEL_3: spi = get_spi_regs(SPI_CHANNEL_3); break;
        default: return;
    }
    if (spi == (SPI_Regs_t*)NULL) return;

    uint16_t* rx_buffer = (uint16_t*)buffer; // Assuming 16-bit data for simplicity
    for (size_t i = 0; i < length; i++) {
        // Wait until receive buffer is not empty
        while (!(*spi->SR & SPI_SR_RXNE)) { /* Wait */ }
        rx_buffer[i] = (uint16_t)*spi->DR; // Read data
    }
}

// MQTT Protocol not supported on this MCU (as noted in MCAL.h)
// HTTP Protocol not supported on this MCU (as noted in MCAL.h)
// WiFi Driver not supported on this MCU (as noted in MCAL.h)
// DTC_driver not supported on this MCU (as noted in MCAL.h)