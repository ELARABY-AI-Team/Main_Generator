#include "MCAL.h"

// =============================================================================
// Register Definitions (Normalized by base address)
// =============================================================================

// FLASH Registers
#define FLASH_ACR_REG       ((volatile tlong*)0x40023C00UL)
#define FLASH_KEYR_REG      ((volatile tlong*)0x40023C04UL)
#define FLASH_OPTKEYR_REG   ((volatile tlong*)0x40023C08UL)
#define FLASH_SR_REG        ((volatile tlong*)0x40023C0CUL)
#define FLASH_CR_REG        ((volatile tlong*)0x40023C10UL)
#define FLASH_OPTCR_REG     ((volatile tlong*)0x40023C14UL)

// CRC Registers
#define CRC_DR_REG          ((volatile tlong*)0x40023000UL)
#define CRC_IDR_REG         ((volatile tlong*)0x40023004UL)
#define CRC_CR_REG          ((volatile tlong*)0x40023008UL)

// PWR Registers
#define PWR_CR_REG          ((volatile tlong*)0x40007000UL)
#define PWR_CSR_REG         ((volatile tlong*)0x40007004UL)

// RCC Registers
#define RCC_CR_REG          ((volatile tlong*)0x40023800UL)
#define RCC_PLLCFGR_REG     ((volatile tlong*)0x40023804UL)
#define RCC_CFGR_REG        ((volatile tlong*)0x40023808UL)
#define RCC_CIR_REG         ((volatile tlong*)0x4002380CUL)
#define RCC_AHB1RSTR_REG    ((volatile tlong*)0x40023810UL)
#define RCC_AHB2RSTR_REG    ((volatile tlong*)0x40023814UL)
#define RCC_APB1RSTR_REG    ((volatile tlong*)0x40023818UL)
#define RCC_APB2RSTR_REG    ((volatile tlong*)0x4002381CUL)
#define RCC_AHB1ENR_REG     ((volatile tlong*)0x40023830UL)
#define RCC_AHB2ENR_REG     ((volatile tlong*)0x40023834UL)
#define RCC_APB1ENR_REG     ((volatile tlong*)0x40023838UL)
#define RCC_APB2ENR_REG     ((volatile tlong*)0x4002383CUL)
#define RCC_AHB1LPENR_REG   ((volatile tlong*)0x40023850UL)
#define RCC_AHB2LPENR_REG   ((volatile tlong*)0x40023854UL)
#define RCC_APB1LPENR_REG   ((volatile tlong*)0x40023858UL)
#define RCC_APB2LPENR_REG   ((volatile tlong*)0x4002385CUL)
#define RCC_BDCR_REG        ((volatile tlong*)0x40023870UL)
#define RCC_CSR_REG         ((volatile tlong*)0x40023874UL)
#define RCC_SSCGR_REG       ((volatile tlong*)0x40023880UL)
#define RCC_PLLI2SCFGR_REG  ((volatile tlong*)0x40023884UL)
#define RCC_DCKCFGR_REG     ((volatile tlong*)0x4002388CUL)

// SYSCFG Registers
#define SYSCFG_MEMRMP_REG   ((volatile tlong*)0x40013800UL)
#define SYSCFG_PMC_REG      ((volatile tlong*)0x40013804UL)
#define SYSCFG_EXTICR1_REG  ((volatile tlong*)0x40013808UL)
#define SYSCFG_EXTICR2_REG  ((volatile tlong*)0x4001380CUL)
#define SYSCFG_EXTICR3_REG  ((volatile tlong*)0x40013810UL)
#define SYSCFG_EXTICR4_REG  ((volatile tlong*)0x40013814UL)
#define SYSCFG_CMPCR_REG    ((volatile tlong*)0x40013820UL)

// EXTI Registers
#define EXTI_IMR_REG        ((volatile tlong*)0x40013C00UL)
#define EXTI_EMR_REG        ((volatile tlong*)0x40013C04UL)
#define EXTI_RTSR_REG       ((volatile tlong*)0x40013C08UL)
#define EXTI_FTSR_REG       ((volatile tlong*)0x40013C0CUL)
#define EXTI_SWIER_REG      ((volatile tlong*)0x40013C10UL)
#define EXTI_PR_REG         ((volatile tlong*)0x40013C14UL)

// ADC Registers
#define ADC_SR_REG          ((volatile tlong*)0x40012000UL)
#define ADC_CR1_REG         ((volatile tlong*)0x40012004UL)
#define ADC_CR2_REG         ((volatile tlong*)0x40012008UL)
#define ADC_SMPR1_REG       ((volatile tlong*)0x4001200CUL)
#define ADC_SMPR2_REG       ((volatile tlong*)0x40012010UL)
#define ADC_JOFR1_REG       ((volatile tlong*)0x40012014UL)
#define ADC_JOFR2_REG       ((volatile tlong*)0x40012018UL)
#define ADC_JOFR3_REG       ((volatile tlong*)0x4001201CUL)
#define ADC_JOFR4_REG       ((volatile tlong*)0x40012020UL)
#define ADC_HTR_REG         ((volatile tlong*)0x40012024UL)
#define ADC_LTR_REG         ((volatile tlong*)0x40012028UL)
#define ADC_SQR1_REG        ((volatile tlong*)0x4001202CUL)
#define ADC_SQR2_REG        ((volatile tlong*)0x40012030UL)
#define ADC_SQR3_REG        ((volatile tlong*)0x40012034UL)
#define ADC_JSQR_REG        ((volatile tlong*)0x40012038UL)
#define ADC_JDR1_REG        ((volatile tlong*)0x4001203CUL)
#define ADC_JDR2_REG        ((volatile tlong*)0x40012040UL)
#define ADC_JDR3_REG        ((volatile tlong*)0x40012044UL)
#define ADC_JDR4_REG        ((volatile tlong*)0x40012048UL)
#define ADC_DR_REG          ((volatile tlong*)0x4001204CUL)
#define ADC_CCR_REG         ((volatile tlong*)0x40012300UL)

// Base addresses for GPIO ports
#define GPIOA_BASE_ADDR     0x40020000UL
#define GPIOB_BASE_ADDR     0x40020400UL
#define GPIOC_BASE_ADDR     0x40020800UL
#define GPIOD_BASE_ADDR     0x40020C00UL
#define GPIOE_BASE_ADDR     0x40021000UL
#define GPIOH_BASE_ADDR     0x40021C00UL

// Helper macro for GPIO register access
#define GPIO_REG(PORT_BASE, OFFSET) ((volatile tlong*)((PORT_BASE) + (OFFSET)))

// GPIO Registers (Offsets)
#define GPIO_MODER_OFFSET   0x00UL
#define GPIO_OTYPER_OFFSET  0x04UL
#define GPIO_OSPEEDR_OFFSET 0x08UL
#define GPIO_PUPDR_OFFSET   0x0CUL
#define GPIO_IDR_OFFSET     0x10UL
#define GPIO_ODR_OFFSET     0x14UL
#define GPIO_BSRR_OFFSET    0x18UL
#define GPIO_LCKR_OFFSET    0x1CUL
#define GPIO_AFRL_OFFSET    0x20UL
#define GPIO_AFRH_OFFSET    0x24UL

// Helper macro to get GPIO port base address
static volatile tlong* _get_gpio_base(t_port port) {
    switch(port) {
        case PORT_A: return (volatile tlong*)GPIOA_BASE_ADDR;
        case PORT_B: return (volatile tlong*)GPIOB_BASE_ADDR;
        case PORT_C: return (volatile tlong*)GPIOC_BASE_ADDR;
        case PORT_D: return (volatile tlong*)GPIOD_BASE_ADDR;
        case PORT_E: return (volatile tlong*)GPIOE_BASE_ADDR;
        case PORT_H: return (volatile tlong*)GPIOH_BASE_ADDR;
        default: return (volatile tlong*)NULL; // Should not happen
    }
}

// Base addresses for USARTs
#define USART1_BASE_ADDR    0x40011000UL
#define USART2_BASE_ADDR    0x40004400UL
#define USART6_BASE_ADDR    0x40011400UL

// Helper macro for USART register access
#define USART_REG(USART_BASE, OFFSET) ((volatile tlong*)((USART_BASE) + (OFFSET)))

// USART Registers (Offsets)
#define USART_SR_OFFSET     0x00UL
#define USART_DR_OFFSET     0x04UL
#define USART_BRR_OFFSET    0x08UL
#define USART_CR1_OFFSET    0x0CUL
#define USART_CR2_OFFSET    0x10UL
#define USART_CR3_OFFSET    0x14UL
#define USART_GTPR_OFFSET   0x18UL

// Helper macro to get USART port base address
static volatile tlong* _get_usart_base(t_uart_channel channel) {
    switch(channel) {
        case UART_CH1: return (volatile tlong*)USART1_BASE_ADDR;
        case UART_CH2: return (volatile tlong*)USART2_BASE_ADDR;
        case UART_CH6: return (volatile tlong*)USART6_BASE_ADDR;
        default: return (volatile tlong*)NULL; // Should not happen
    }
}

// Base addresses for I2Cs
#define I2C1_BASE_ADDR      0x40005400UL
#define I2C2_BASE_ADDR      0x40005800UL
#define I2C3_BASE_ADDR      0x40005C00UL

// Helper macro for I2C register access
#define I2C_REG(I2C_BASE, OFFSET) ((volatile tlong*)((I2C_BASE) + (OFFSET)))

// I2C Registers (Offsets)
#define I2C_CR1_OFFSET      0x00UL
#define I2C_CR2_OFFSET      0x04UL
#define I2C_OAR1_OFFSET     0x08UL
#define I2C_OAR2_OFFSET     0x0CUL
#define I2C_DR_OFFSET       0x10UL
#define I2C_SR1_OFFSET      0x14UL
#define I2C_SR2_OFFSET      0x18UL
#define I2C_CCR_OFFSET      0x1CUL
#define I2C_TRISE_OFFSET    0x20UL
#define I2C_FLTR_OFFSET     0x24UL

// Helper macro to get I2C port base address
static volatile tlong* _get_i2c_base(t_i2c_channel channel) {
    switch(channel) {
        case I2C_CH1: return (volatile tlong*)I2C1_BASE_ADDR;
        case I2C_CH2: return (volatile tlong*)I2C2_BASE_ADDR;
        case I2C_CH3: return (volatile tlong*)I2C3_BASE_ADDR;
        default: return (volatile tlong*)NULL; // Should not happen
    }
}

// Base addresses for SPIs
#define SPI1_BASE_ADDR      0x40013000UL
#define SPI2_BASE_ADDR      0x40003800UL
#define SPI3_BASE_ADDR      0x40003C00UL

// Helper macro for SPI register access
#define SPI_REG(SPI_BASE, OFFSET) ((volatile tlong*)((SPI_BASE) + (OFFSET)))

// SPI Registers (Offsets)
#define SPI_CR1_OFFSET      0x00UL
#define SPI_CR2_OFFSET      0x04UL
#define SPI_SR_OFFSET       0x08UL
#define SPI_DR_OFFSET       0x0CUL
#define SPI_CRCPR_OFFSET    0x10UL
#define SPI_RXCRCR_OFFSET   0x14UL
#define SPI_TXCRCR_OFFSET   0x18UL
#define SPI_I2SCFGR_OFFSET  0x1CUL
#define SPI_I2SPR_OFFSET    0x20UL

// Helper macro to get SPI port base address
static volatile tlong* _get_spi_base(t_spi_channel channel) {
    switch(channel) {
        case SPI_CH1: return (volatile tlong*)SPI1_BASE_ADDR;
        case SPI_CH2: return (volatile tlong*)SPI2_BASE_ADDR;
        case SPI_CH3: return (volatile tlong*)SPI3_BASE_ADDR;
        default: return (volatile tlong*)NULL; // Should not happen
    }
}

// Base addresses for TIMERS
#define TIM1_BASE_ADDR      0x40010000UL
#define TIM2_BASE_ADDR      0x40000000UL
#define TIM3_BASE_ADDR      0x40000400UL
#define TIM4_BASE_ADDR      0x40000800UL
#define TIM5_BASE_ADDR      0x40000C00UL
#define TIM9_BASE_ADDR      0x40014000UL
#define TIM10_BASE_ADDR     0x40014400UL
#define TIM11_BASE_ADDR     0x40014800UL

// Helper macro for TIM register access
#define TIM_REG(TIM_BASE, OFFSET) ((volatile tlong*)((TIM_BASE) + (OFFSET)))

// TIM Registers (Offsets)
#define TIM_CR1_OFFSET      0x00UL
#define TIM_CR2_OFFSET      0x04UL
#define TIM_SMCR_OFFSET     0x08UL
#define TIM_DIER_OFFSET     0x0CUL
#define TIM_SR_OFFSET       0x10UL
#define TIM_EGR_OFFSET      0x14UL
#define TIM_CCMR1_OFFSET    0x18UL
#define TIM_CCMR2_OFFSET    0x1CUL // Only for general purpose timers with 4 channels
#define TIM_CCER_OFFSET     0x20UL
#define TIM_CNT_OFFSET      0x24UL
#define TIM_PSC_OFFSET      0x28UL
#define TIM_ARR_OFFSET      0x2CUL
#define TIM_RCR_OFFSET      0x30UL // Only for advanced timers (TIM1)
#define TIM_CCR1_OFFSET     0x34UL
#define TIM_CCR2_OFFSET     0x38UL
#define TIM_CCR3_OFFSET     0x3CUL
#define TIM_CCR4_OFFSET     0x40UL
#define TIM_BDTR_OFFSET     0x44UL // Only for advanced timers (TIM1)
#define TIM_DCR_OFFSET      0x48UL
#define TIM_DMAR_OFFSET     0x4CUL
#define TIM_OR_OFFSET       0x50UL // Only for general purpose timers (TIM2, TIM5)

// Helper macro to get TIM base address
static volatile tlong* _get_timer_base(t_timer_channel channel) {
    switch(channel) {
        case TIMER_TIM1:  return (volatile tlong*)TIM1_BASE_ADDR;
        case TIMER_TIM2:  return (volatile tlong*)TIM2_BASE_ADDR;
        case TIMER_TIM3:  return (volatile tlong*)TIM3_BASE_ADDR;
        case TIMER_TIM4:  return (volatile tlong*)TIM4_BASE_ADDR;
        case TIMER_TIM5:  return (volatile tlong*)TIM5_BASE_ADDR;
        case TIMER_TIM9:  return (volatile tlong*)TIM9_BASE_ADDR;
        case TIMER_TIM10: return (volatile tlong*)TIM10_BASE_ADDR;
        case TIMER_TIM11: return (volatile tlong*)TIM11_BASE_ADDR;
        default: return (volatile tlong*)NULL; // Should not happen
    }
}

// =============================================================================
// Private Helper Functions (Internal to MCAL)
// =============================================================================

// Helper for GPIO operations
static volatile tlong* _get_gpio_reg_ptr(t_port port, tlong offset) {
    volatile tlong* base_addr = _get_gpio_base(port);
    if (base_addr == NULL) return NULL;
    return (volatile tlong*)((tlong)base_addr + offset);
}

// Helper to calculate USART DIV factor based on baud rate
static tword _usart_get_div_factor(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate) {
    // This is a simplified calculation, actual calculation depends on PCLK speed and oversampling.
    // For STM32F401RC: PCLK1 for USART2 (max 42MHz), PCLK2 for USART1, USART6 (max 84MHz)
    // Assuming 84MHz PCLK for USART1/6, 42MHz PCLK for USART2, Oversampling by 16
    tlong pclk_freq = 0;
    if (uart_channel == UART_CH2) {
        pclk_freq = 42000000UL; // APB1 bus clock
    } else {
        pclk_freq = 84000000UL; // APB2 bus clock
    }

    tlong baud_rate_val = 0;
    switch (uart_baud_rate) {
        case UART_BAUD_9600:    baud_rate_val = 9600; break;
        case UART_BAUD_19200:   baud_rate_val = 19200; break;
        case UART_BAUD_38400:   baud_rate_val = 38400; break;
        case UART_BAUD_57600:   baud_rate_val = 57600; break;
        case UART_BAUD_115200:  baud_rate_val = 115200; break;
        default: return 0; // Invalid baud rate
    }

    // Formula: USARTDIV = PCLKx / (8 * (2 - OVER8) * Baud_rate)
    // For OVER8 = 0 (Oversampling by 16, default): USARTDIV = PCLKx / (16 * Baud_rate)
    tlong usartdiv = (pclk_freq * 100) / (16 * baud_rate_val); // Multiply by 100 for precision

    // BRR register format: 11-4 = DIV_M, 3-0 = DIV_F
    // DIV_M = integer part of USARTDIV, DIV_F = fractional part * 16
    tword div_mantissa = usartdiv / 100;
    tword div_fraction = ((usartdiv % 100) * 16 + 50) / 100; // Rounding

    return (div_mantissa << 4) | (div_fraction & 0xF);
}

// =============================================================================
// MCU CONFIG APIs
// =============================================================================

/**
 * @brief Initializes MCU configuration including GPIOs, disabling peripherals,
 *        and setting up Watchdog Timer (WDT) and Low Voltage Reset (LVR).
 * @param volt System voltage source (Vsource_3V or Vsource_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Reset WDT first as per rule

    // 1. Set all GPIO pins to 0 and verify with while loop
    // 2. Set all GPIO pins direction to input and verify with while loop
    // STM32 GPIO reset values are typically input floating.
    // Setting MODER to 00 (input) and PUPDR to 00 (no pull-up/down) resets them.
    // Clock for GPIOs must be enabled before touching their registers.
    volatile tlong* gpio_ports_base[] = {
        (volatile tlong*)GPIOA_BASE_ADDR, (volatile tlong*)GPIOB_BASE_ADDR,
        (volatile tlong*)GPIOC_BASE_ADDR, (volatile tlong*)GPIOD_BASE_ADDR,
        (volatile tlong*)GPIOE_BASE_ADDR, (volatile tlong*)GPIOH_BASE_ADDR
    };
    tbyte i;
    for (i = 0; i < sizeof(gpio_ports_base)/sizeof(gpio_ports_base[0]); i++) {
        WDT_Reset(); // Keep WDT happy during loop
        volatile tlong* rcc_ahb1enr_val = RCC_AHB1ENR_REG; // For GPIO clocks
        if (gpio_ports_base[i] == (volatile tlong*)GPIOA_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 0); } // GPIOAEN - Inferred
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOB_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 1); } // GPIOBEN - Inferred
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOC_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 2); } // GPIOCEN - Inferred
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOD_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 3); } // GPIODEN - Inferred
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOE_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 4); } // GPIOEEN - Inferred
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOH_BASE_ADDR) { *rcc_ahb1enr_val |= (1UL << 7); } // GPIOHEN - Inferred

        // Set all GPIO pins to 0 (ODR) and direction to input (MODER=00, PUPDR=00)
        volatile tlong* moder = (volatile tlong*)((tlong)gpio_ports_base[i] + GPIO_MODER_OFFSET);
        volatile tlong* pupdr = (volatile tlong*)((tlong)gpio_ports_base[i] + GPIO_PUPDR_OFFSET);
        volatile tlong* odr   = (volatile tlong*)((tlong)gpio_ports_base[i] + GPIO_ODR_OFFSET);

        *odr = 0x00000000UL; // Set all output data to 0
        *moder = 0x00000000UL; // Set all pins to input mode (00)
        *pupdr = 0x00000000UL; // Set all pins to no pull-up/pull-down (00)

        // Verify with while loop
        while (*odr != 0x00000000UL) { WDT_Reset(); /* Stall until verified */ }
        while (*moder != 0x00000000UL) { WDT_Reset(); /* Stall until verified */ }
        while (*pupdr != 0x00000000UL) { WDT_Reset(); /* Stall until verified */ }

        // Disable GPIO clock to save power
        if (gpio_ports_base[i] == (volatile tlong*)GPIOA_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 0); }
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOB_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 1); }
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOC_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 2); }
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOD_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 3); }
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOE_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 4); }
        else if (gpio_ports_base[i] == (volatile tlong*)GPIOH_BASE_ADDR) { *rcc_ahb1enr_val &= ~(1UL << 7); }
    }


    WDT_Reset();

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable();

    // Disable ADC (CR2.ADON=0)
    if (ADC_CR2_REG != NULL) {
        *ADC_CR2_REG &= ~(1UL << 0); // Clear ADON bit (ADC enable)
        *RCC_APB2ENR_REG &= ~(1UL << 8); // ADC1EN - Inferred
    }

    // Disable all USARTs (CR1.UE=0) and their clocks
    if (_get_usart_base(UART_CH1) != NULL) {
        *(USART_REG(_get_usart_base(UART_CH1), USART_CR1_OFFSET)) &= ~(1UL << 13); // UE (USART Enable)
        *RCC_APB2ENR_REG &= ~(1UL << 4); // USART1EN - Inferred
    }
    if (_get_usart_base(UART_CH2) != NULL) {
        *(USART_REG(_get_usart_base(UART_CH2), USART_CR1_OFFSET)) &= ~(1UL << 13); // UE
        *RCC_APB1ENR_REG &= ~(1UL << 17); // USART2EN - Inferred
    }
    if (_get_usart_base(UART_CH6) != NULL) {
        *(USART_REG(_get_usart_base(UART_CH6), USART_CR1_OFFSET)) &= ~(1UL << 13); // UE
        *RCC_APB2ENR_REG &= ~(1UL << 5); // USART6EN - Inferred
    }

    // Disable all I2Cs (CR1.PE=0) and their clocks
    if (_get_i2c_base(I2C_CH1) != NULL) {
        *(I2C_REG(_get_i2c_base(I2C_CH1), I2C_CR1_OFFSET)) &= ~(1UL << 0); // PE (Peripheral Enable)
        *RCC_APB1ENR_REG &= ~(1UL << 21); // I2C1EN - Inferred
    }
    if (_get_i2c_base(I2C_CH2) != NULL) {
        *(I2C_REG(_get_i2c_base(I2C_CH2), I2C_CR1_OFFSET)) &= ~(1UL << 0); // PE
        *RCC_APB1ENR_REG &= ~(1UL << 22); // I2C2EN - Inferred
    }
    if (_get_i2c_base(I2C_CH3) != NULL) {
        *(I2C_REG(_get_i2c_base(I2C_CH3), I2C_CR1_OFFSET)) &= ~(1UL << 0); // PE
        *RCC_APB1ENR_REG &= ~(1UL << 23); // I2C3EN - Inferred
    }

    // Disable all SPIs (CR1.SPE=0) and their clocks
    if (_get_spi_base(SPI_CH1) != NULL) {
        *(SPI_REG(_get_spi_base(SPI_CH1), SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPE (SPI Enable)
        *RCC_APB2ENR_REG &= ~(1UL << 0); // SPI1EN - Inferred
    }
    if (_get_spi_base(SPI_CH2) != NULL) {
        *(SPI_REG(_get_spi_base(SPI_CH2), SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPE
        *RCC_APB1ENR_REG &= ~(1UL << 14); // SPI2EN - Inferred
    }
    if (_get_spi_base(SPI_CH3) != NULL) {
        *(SPI_REG(_get_spi_base(SPI_CH3), SPI_CR1_OFFSET)) &= ~(1UL << 6); // SPE
        *RCC_APB1ENR_REG &= ~(1UL << 15); // SPI3EN - Inferred
    }

    // Disable all TIMERS (CR1.CEN=0) and their clocks
    for (i = TIMER_TIM1; i < TIMER_MAX; i++) {
        WDT_Reset(); // Keep WDT happy during loop
        volatile tlong* tim_base = _get_timer_base((t_timer_channel)i);
        if (tim_base != NULL) {
            *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0); // CEN (Counter Enable)

            // Disable TIM clocks - Inferred
            if (i == TIMER_TIM1 || i == TIMER_TIM9 || i == TIMER_TIM10 || i == TIMER_TIM11) {
                // APB2 Timers
                *RCC_APB2ENR_REG &= ~(1UL << (i == TIMER_TIM1 ? 0 : (i == TIMER_TIM9 ? 16 : (i == TIMER_TIM10 ? 17 : 18))));
            } else {
                // APB1 Timers
                *RCC_APB1ENR_REG &= ~(1UL << (i == TIMER_TIM2 ? 0 : (i == TIMER_TIM3 ? 1 : (i == TIMER_TIM4 ? 2 : 3))));
            }
        }
    }
    
    // Disable SYSCFG clock (needed for EXTI external interrupt config)
    *RCC_APB2ENR_REG &= ~(1UL << 14); // SYSCFGEN - Inferred

    WDT_Reset();

    // 4. Enable WDT (Watchdog Timer)
    // 5. Clear WDT timer
    // 6. Set WDT period >= 8 msec
    // 7. Clear WDT again
    // STM32 IWDG implementation (registers are inferred as they are not in the JSON)
    // IWDG_KR: Key Register (Write Access to IWDG_PR and IWDG_RLR)
    // IWDG_PR: Prescaler Register (0: div4, 1: div8, ..., 6: div256)
    // IWDG_RLR: Reload Register (0x000 to 0xFFF)
    // LSI (Low Speed Internal RC oscillator) typically ~32kHz for IWDG clock source

    // Enable write access to IWDG_PR and IWDG_RLR
    *((volatile tword*)0x40003000UL) = 0x5555; // IWDG_KR - Inferred: enable register access
    // Set prescaler to maximum (div256) to allow larger reload value for 8ms+
    *((volatile tword*)0x40003004UL) = 0x00000006; // IWDG_PR (Prescaler bits) -> div256 - Inferred
    // Calculate RLR for >= 8ms. LSI freq ~32kHz. Period = RLR * Prescaler / LSI_Freq.
    // 8ms = RLR * 256 / 32000 => RLR = 8 * 32000 / 256 = 1000.
    // Set RLR to 1000. Maximum RLR is 0xFFF (4095).
    *((volatile tword*)0x40003008UL) = 1000 - 1; // IWDG_RLR (Reload value) - Inferred (value - 1)

    // Start IWDG (enable watchdog)
    *((volatile tword*)0x40003000UL) = 0xCCCC; // IWDG_KR - Inferred: start watchdog

    // Clear WDT again (reload watchdog counter)
    WDT_Reset();

    // 8. Set LOW Voltage Reset value based on system voltage (2V for 3V, 3.5V for 5V)
    // 9. Enable LVR (Low Voltage Reset)
    // STM32 PVD (Programmable Voltage Detector)
    // PWR_CR register: PVDE (bit 4), PLS[2:0] (bits 7:5)
    tbyte pls_value = 0; // Default to 0
    if (volt == Vsource_3V) {
        // For 3V system, set LVR to 2V. PLS[2:0] = 000 (2.2V) or 001 (2.3V). Choosing 2.2V as close to 2V.
        pls_value = 0b000; // PVD level 0 (2.2V) - Inferred based on STM32 PVD levels
    } else if (volt == Vsource_5V) {
        // For 5V system, set LVR to 3.5V. PLS[2:0] = 100 (3.2V) or 101 (3.3V) or 110 (3.4V) or 111 (3.5V). Choosing 3.5V.
        pls_value = 0b111; // PVD level 7 (3.5V) - Inferred based on STM32 PVD levels
    }
    *PWR_CR_REG &= ~((0b111UL) << 5); // Clear PLS bits
    *PWR_CR_REG |= ((tlong)pls_value << 5); // Set PLS bits
    *PWR_CR_REG |= (1UL << 4); // Set PVDE bit (PVD enable)

    WDT_Reset(); // Clear WDT again
}

/**
 * @brief Resets the Watchdog Timer (WDT) to prevent a system reset.
 *        This function must be called periodically to keep the WDT happy.
 */
void WDT_Reset(void) {
    // STM32 IWDG reload key
    *((volatile tword*)0x40003000UL) = 0xAAAA; // IWDG_KR - Inferred: reload watchdog counter
}

/**
 * @brief Puts the microcontroller into sleep mode (low power mode).
 *        In this mode, the CPU stops executing code, and peripherals
 *        continue to operate.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset();
    // For ARM Cortex-M microcontrollers, __WFI() (Wait For Interrupt) is commonly used.
    __WFI(); // Enter Wait For Interrupt mode
}

/**
 * @brief Enables global interrupts.
 *        Allows the CPU to respond to interrupt requests.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset();
    __enable_irq(); // Enable global interrupts
}

/**
 * @brief Disables global interrupts.
 *        Prevents the CPU from responding to interrupt requests.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset();
    __disable_irq(); // Disable global interrupts
}

// =============================================================================
// LVD APIs
// =============================================================================

/**
 * @brief Initializes the Low Voltage Detection (LVD) module.
 * @param lvd_thresholdLevel The desired voltage threshold level.
 */
void LVD_Init(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset();
    // Enable PWR clock first for PVD configuration
    *RCC_APB1ENR_REG |= (1UL << 28); // PWREN - Inferred

    // Map t_lvd_thrthresholdLevel to PLS bits in PWR_CR
    tbyte pls_val = 0;
    switch (lvd_thresholdLevel) {
        // Note: STM32 PVD levels are fixed. Mapping closest values.
        // Consult STM32F401RC reference manual for exact PVD levels.
        case Volt_0_5V: // Not directly supported by STM32 PVD
        case Volt_1V:   // Not directly supported by STM32 PVD
        case Volt_1_5V: // Not directly supported by STM32 PVD
        case Volt_2V:   pls_val = 0b000; break; // PVD_LEVEL_0: 2.2V
        case Volt_2_5V:
        case Volt_2_6V: pls_val = 0b001; break; // PVD_LEVEL_1: 2.3V
        case Volt_2_7V:
        case Volt_2_8V: pls_val = 0b010; break; // PVD_LEVEL_2: 2.4V
        case Volt_2_9V:
        case Volt_3V:   pls_val = 0b011; break; // PVD_LEVEL_3: 2.5V
        case Volt_3_1V:
        case Volt_3_2V: pls_val = 0b100; break; // PVD_LEVEL_4: 2.6V
        case Volt_3_3V:
        case Volt_3_4V: pls_val = 0b101; break; // PVD_LEVEL_5: 2.7V
        case Volt_3_5V:
        case Volt_3_6V: pls_val = 0b110; break; // PVD_LEVEL_6: 2.8V
        case Volt_3_7V:
        case Volt_3_8V:
        case Volt_3_9V:
        case Volt_4V:
        case Volt_4_1V:
        case Volt_4_2V:
        case Volt_4_3V:
        case Volt_4_4V:
        case Volt_4_5V:
        case Volt_4_6V:
        case Volt_4_7V:
        case Volt_4_8V:
        case Volt_4_9V:
        case Volt_5V:   pls_val = 0b111; break; // PVD_LEVEL_7: 2.9V (Maximum PVD for F401 is 2.9V)
        default: pls_val = 0b000; break; // Default to lowest if invalid
    }

    *PWR_CR_REG &= ~((0b111UL) << 5); // Clear PLS bits (bits 7:5)
    *PWR_CR_REG |= ((tlong)pls_val << 5); // Set PLS bits
}

/**
 * @brief Retrieves the current LVD status (whether voltage is below threshold).
 * @param lvd_thresholdLevel The threshold level to check against.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset();
    // This function checks the PVD output status, PVDO bit in PWR_CSR.
    // The input `lvd_thresholdLevel` is redundant if PVD is already configured.
    // It should ideally return the status as a boolean or enum, not void.
    // For now, it will just read the status.
    tbyte pvd_status = (*PWR_CSR_REG >> 2) & 0x01; // PVDO bit 2
    (void)pvd_status; // Suppress unused warning. User would check this in higher layer.
}

/**
 * @brief Enables the Low Voltage Detection (LVD) module.
 */
void LVD_Enable(void) {
    WDT_Reset();
    // Enable PWR clock first for PVD configuration
    *RCC_APB1ENR_REG |= (1UL << 28); // PWREN - Inferred
    // Set PVDE bit (PVD enable)
    *PWR_CR_REG |= (1UL << 4);
}

/**
 * @brief Disables the Low Voltage Detection (LVD) module.
 */
void LVD_Disable(void) {
    WDT_Reset();
    // Clear PVDE bit (PVD enable)
    *PWR_CR_REG &= ~(1UL << 4);
}

/**
 * @brief Clears the LVD flag.
 * @param lvd_channel Placeholder for LVD channel.
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel) {
    WDT_Reset();
    // STM32 PVD status is generally read from PWR_CSR.PVDO.
    // For PVD interrupt, the flag is typically in EXTI_PR register (EXTI line 16 for PVD output).
    // To clear EXTI line 16 pending bit: write 1 to EXTI_PR bit 16.
    if (EXTI_PR_REG != NULL) {
        *EXTI_PR_REG = (1UL << 16); // Clear PVD EXTI pending flag - Inferred
    }
    (void)lvd_channel; // Suppress unused warning for placeholder parameter.
}

// =============================================================================
// UART APIs
// =============================================================================

/**
 * @brief Initializes a UART channel.
 * @param uart_channel The UART channel to initialize (UART_CH1, UART_CH2, UART_CH6).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length Data frame length (8-bit or 9-bit).
 * @param uart_stop_bit Number of stop bits.
 * @param uart_parity Parity configuration (None, Even, Odd).
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;

    // Enable clock for the specific USART peripheral (done in UART_Enable)
    // Enable GPIO clock and configure pins (TX/RX alternate function)
    t_port tx_port, rx_port;
    t_pin tx_pin, rx_pin;
    tbyte af_val;

    switch (uart_channel) {
        case UART_CH1: // USART1 (TX: PA9/PB6, RX: PA10/PB7)
            tx_port = PORT_A; tx_pin = PIN9; rx_port = PORT_A; rx_pin = PIN10; af_val = 7; // AF7 for USART1
            *RCC_AHB1ENR_REG |= (1UL << 0); // GPIOAEN - Inferred
            break;
        case UART_CH2: // USART2 (TX: PA2/PD5, RX: PA3/PD6)
            tx_port = PORT_A; tx_pin = PIN2; rx_port = PORT_A; rx_pin = PIN3; af_val = 7; // AF7 for USART2
            *RCC_AHB1ENR_REG |= (1UL << 0); // GPIOAEN - Inferred
            break;
        case UART_CH6: // USART6 (TX: PA11/PC6, RX: PA12/PC7)
            tx_port = PORT_C; tx_pin = PIN6; rx_port = PORT_C; rx_pin = PIN7; af_val = 8; // AF8 for USART6
            *RCC_AHB1ENR_REG |= (1UL << 2); // GPIOCEN - Inferred
            break;
        default: return;
    }

    // Configure TX pin as Alternate Function Push-Pull
    volatile tlong* moder_tx = _get_gpio_reg_ptr(tx_port, GPIO_MODER_OFFSET);
    volatile tlong* otyper_tx = _get_gpio_reg_ptr(tx_port, GPIO_OTYPER_OFFSET);
    volatile tlong* pupdr_tx = _get_gpio_reg_ptr(tx_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_tx = (tx_pin < 8) ? _get_gpio_reg_ptr(tx_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(tx_port, GPIO_AFRH_OFFSET);

    *moder_tx |= (0b10UL << (tx_pin * 2)); // Alternate function mode
    *otyper_tx &= ~(1UL << tx_pin); // Push-pull
    *pupdr_tx &= ~(0b11UL << (tx_pin * 2)); // No pull-up/pull-down (Floating)

    if (tx_pin < 8) {
        *afr_tx &= ~((0b1111UL) << (tx_pin * 4));
        *afr_tx |= ((tlong)af_val << (tx_pin * 4));
    } else {
        *afr_tx &= ~((0b1111UL) << ((tx_pin - 8) * 4));
        *afr_tx |= ((tlong)af_val << ((tx_pin - 8) * 4));
    }

    // Configure RX pin as Alternate Function Input (Floating)
    volatile tlong* moder_rx = _get_gpio_reg_ptr(rx_port, GPIO_MODER_OFFSET);
    volatile tlong* pupdr_rx = _get_gpio_reg_ptr(rx_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_rx = (rx_pin < 8) ? _get_gpio_reg_ptr(rx_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(rx_port, GPIO_AFRH_OFFSET);

    *moder_rx |= (0b10UL << (rx_pin * 2)); // Alternate function mode
    *pupdr_rx &= ~(0b11UL << (rx_pin * 2)); // No pull-up/pull-down (Floating)

    if (rx_pin < 8) {
        *afr_rx &= ~((0b1111UL) << (rx_pin * 4));
        *afr_rx |= ((tlong)af_val << (rx_pin * 4));
    } else {
        *afr_rx &= ~((0b1111UL) << ((rx_pin - 8) * 4));
        *afr_rx |= ((tlong)af_val << ((rx_pin - 8) * 4));
    }

    // Disable USART before configuration
    *(USART_REG(usart_base, USART_CR1_OFFSET)) &= ~(1UL << 13); // Clear UE bit

    // Set Baud Rate (BRR)
    tword brr_val = _usart_get_div_factor(uart_channel, uart_baud_rate);
    *(USART_REG(usart_base, USART_BRR_OFFSET)) = brr_val;

    // Configure Data Length (CR1.M)
    if (uart_data_length == UART_DATA_9BIT) {
        *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 12); // M bit for 9-bit data
    } else {
        *(USART_REG(usart_base, USART_CR1_OFFSET)) &= ~(1UL << 12); // M bit for 8-bit data
    }

    // Configure Stop Bits (CR2.STOP)
    *(USART_REG(usart_base, USART_CR2_OFFSET)) &= ~(0b11UL << 12); // Clear STOP bits
    switch (uart_stop_bit) {
        case UART_STOP_0_5BIT: *(USART_REG(usart_base, USART_CR2_OFFSET)) |= (0b01UL << 12); break;
        case UART_STOP_2BIT:   *(USART_REG(usart_base, USART_CR2_OFFSET)) |= (0b10UL << 12); break;
        case UART_STOP_1_5BIT: *(USART_REG(usart_base, USART_CR2_OFFSET)) |= (0b11UL << 12); break;
        case UART_STOP_1BIT:
        default: break; // 00 for 1 stop bit
    }

    // Configure Parity (CR1.PCE, PS)
    *(USART_REG(usart_base, USART_CR1_OFFSET)) &= ~(0b11UL << 9); // Clear PCE and PS bits
    if (uart_parity != UART_PARITY_NONE) {
        *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 10); // PCE (Parity Control Enable)
        if (uart_parity == UART_PARITY_ODD) {
            *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 9); // PS (Parity Selection: 1 for Odd)
        }
    }

    // Enable Transmitter and Receiver (TE, RE bits in CR1)
    *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 3); // TE (Transmitter Enable)
    *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 2); // RE (Receiver Enable)
}

/**
 * @brief Enables a UART channel.
 * @param uart_channel The UART channel to enable.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;

    // Enable peripheral clock first (RCC)
    WDT_Reset();
    if (uart_channel == UART_CH1) {
        *RCC_APB2ENR_REG |= (1UL << 4); // USART1EN - Inferred
    } else if (uart_channel == UART_CH2) {
        *RCC_APB1ENR_REG |= (1UL << 17); // USART2EN - Inferred
    } else if (uart_channel == UART_CH6) {
        *RCC_APB2ENR_REG |= (1UL << 5); // USART6EN - Inferred
    }

    // Enable USART (UE bit in CR1)
    *(USART_REG(usart_base, USART_CR1_OFFSET)) |= (1UL << 13);
}

/**
 * @brief Disables a UART channel.
 * @param uart_channel The UART channel to disable.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;

    // Disable USART (UE bit in CR1)
    *(USART_REG(usart_base, USART_CR1_OFFSET)) &= ~(1UL << 13);

    // Disable peripheral clock (RCC)
    WDT_Reset();
    if (uart_channel == UART_CH1) {
        *RCC_APB2ENR_REG &= ~(1UL << 4); // USART1EN - Inferred
    } else if (uart_channel == UART_CH2) {
        *RCC_APB1ENR_REG &= ~(1UL << 17); // USART2EN - Inferred
    } else if (uart_channel == UART_CH6) {
        *RCC_APB2ENR_REG &= ~(1UL << 5); // USART6EN - Inferred
    }
}

/**
 * @brief Updates the status of a UART channel.
 *        This function is a placeholder for any periodic status updates.
 * @param uart_channel The UART channel to update.
 */
void UART_Update(t_uart_channel uart_channel) {
    WDT_Reset();
    // This function can be used to check status flags like RXNE, TXE, TC, etc.
    // For now, it's a placeholder as per API.json.
    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;
    volatile tlong status = *(USART_REG(usart_base, USART_SR_OFFSET));
    (void)status; // Suppress unused variable warning.
}

/**
 * @brief Sends a single byte over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param byte The byte to send.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;

    // Wait until Transmit Data Register Empty (TXE) flag is set
    while (!(*(USART_REG(usart_base, USART_SR_OFFSET)) & (1UL << 7))) {
        WDT_Reset();
    }
    // Write data to Data Register (DR)
    *(USART_REG(usart_base, USART_DR_OFFSET)) = byte;
}

/**
 * @brief Sends a frame of data over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param data Pointer to the data buffer.
 * @param length Length of the data frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset();

    int i;
    for (i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string over a UART channel.
 * @param uart_channel The UART channel to use.
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset();

    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str);
        str++;
    }
}

/**
 * @brief Receives a single byte from a UART channel.
 * @param uart_channel The UART channel to use.
 * @return The received byte.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return 0; // Or handle error

    // Wait until Read Data Register Not Empty (RXNE) flag is set
    while (!(*(USART_REG(usart_base, USART_SR_OFFSET)) & (1UL << 5))) {
        WDT_Reset();
    }
    // Read data from Data Register (DR)
    return (tbyte)(*(USART_REG(usart_base, USART_DR_OFFSET)) & 0xFFUL);
}

/**
 * @brief Receives a frame of data from a UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();

    int i;
    for (i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Receives a null-terminated string from a UART channel.
 * @param uart_channel The UART channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes read.
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset();

    int i = 0;
    while (i < max_length - 1) { // Leave space for null terminator
        buffer[i] = (char)UART_Get_Byte(uart_channel);
        if (buffer[i] == '\0' || buffer[i] == '\n' || buffer[i] == '\r') {
            break;
        }
        i++;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}

/**
 * @brief Clears flags of a UART channel.
 * @param uart_channel The UART channel to clear flags for.
 */
void UART_ClearFlag(t_uart_channel uart_channel) {
    WDT_Reset();

    volatile tlong* usart_base = _get_usart_base(uart_channel);
    if (usart_base == NULL) return;

    // Common flags like RXNE are cleared by reading DR.
    // Some flags like TC (Transmission Complete) are cleared by reading SR then writing to DR.
    // ORE (Overrun Error) is cleared by reading SR then reading DR.
    // This implementation will simply read SR to clear some flags that are cleared on read.
    // For specific flags, a more targeted approach might be needed.
    (void)*(USART_REG(usart_base, USART_SR_OFFSET)); // Read SR
    // If clearing specific flags (e.g., TC), a read-modify-write on SR or specific steps are needed.
    // For simplicity, a generic read of DR after SR clears ORE, NF, FE flags.
    (void)*(USART_REG(usart_base, USART_DR_OFFSET)); // Read DR
}

// =============================================================================
// I2C APIs
// =============================================================================

/**
 * @brief Initializes an I2C channel.
 * @param i2c_channel The I2C channel to initialize.
 * @param i2c_clk_speed Clock speed for the I2C bus (always fast mode).
 * @param i2c_device_address Own device address.
 * @param i2c_ack Acknowledge configuration.
 * @param i2c_datalength Data length (placeholder, I2C is byte-oriented).
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // Enable clock for the specific I2C peripheral (done in I2C_Enable)
    // Enable GPIO clock and configure pins (SCL/SDA alternate function open-drain)
    t_port scl_port, sda_port;
    t_pin scl_pin, sda_pin;
    tbyte af_val = 4; // AF4 for I2C

    switch (i2c_channel) {
        case I2C_CH1: // I2C1 (SCL: PB6/PB8, SDA: PB7/PB9)
            scl_port = PORT_B; scl_pin = PIN6; sda_port = PORT_B; sda_pin = PIN7;
            *RCC_AHB1ENR_REG |= (1UL << 1); // GPIOBEN - Inferred
            break;
        case I2C_CH2: // I2C2 (SCL: PB10/PB3, SDA: PB11)
            scl_port = PORT_B; scl_pin = PIN10; sda_port = PORT_B; sda_pin = PIN11;
            *RCC_AHB1ENR_REG |= (1UL << 1); // GPIOBEN - Inferred
            break;
        case I2C_CH3: // I2C3 (SCL: PA8/PC9, SDA: PB4)
            scl_port = PORT_A; scl_pin = PIN8; sda_port = PORT_B; sda_pin = PIN4;
            *RCC_AHB1ENR_REG |= (1UL << 0) | (1UL << 1); // GPIOAEN, GPIOBEN - Inferred
            break;
        default: return;
    }

    // Configure SCL pin as Alternate Function Open-Drain
    volatile tlong* moder_scl = _get_gpio_reg_ptr(scl_port, GPIO_MODER_OFFSET);
    volatile tlong* otyper_scl = _get_gpio_reg_ptr(scl_port, GPIO_OTYPER_OFFSET);
    volatile tlong* pupdr_scl = _get_gpio_reg_ptr(scl_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_scl = (scl_pin < 8) ? _get_gpio_reg_ptr(scl_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(scl_port, GPIO_AFRH_OFFSET);

    *moder_scl |= (0b10UL << (scl_pin * 2)); // Alternate function mode
    *otyper_scl |= (1UL << scl_pin); // Open-drain
    *pupdr_scl |= (0b01UL << (scl_pin * 2)); // Pull-up (recommended for I2C)

    if (scl_pin < 8) {
        *afr_scl &= ~((0b1111UL) << (scl_pin * 4));
        *afr_scl |= ((tlong)af_val << (scl_pin * 4));
    } else {
        *afr_scl &= ~((0b1111UL) << ((scl_pin - 8) * 4));
        *afr_scl |= ((tlong)af_val << ((scl_pin - 8) * 4));
    }

    // Configure SDA pin as Alternate Function Open-Drain
    volatile tlong* moder_sda = _get_gpio_reg_ptr(sda_port, GPIO_MODER_OFFSET);
    volatile tlong* otyper_sda = _get_gpio_reg_ptr(sda_port, GPIO_OTYPER_OFFSET);
    volatile tlong* pupdr_sda = _get_gpio_reg_ptr(sda_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_sda = (sda_pin < 8) ? _get_gpio_reg_ptr(sda_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(sda_port, GPIO_AFRH_OFFSET);

    *moder_sda |= (0b10UL << (sda_pin * 2)); // Alternate function mode
    *otyper_sda |= (1UL << sda_pin); // Open-drain
    *pupdr_sda |= (0b01UL << (sda_pin * 2)); // Pull-up (recommended for I2C)

    if (sda_pin < 8) {
        *afr_sda &= ~((0b1111UL) << (sda_pin * 4));
        *afr_sda |= ((tlong)af_val << (sda_pin * 4));
    } else {
        *afr_sda &= ~((0b1111UL) << ((sda_pin - 8) * 4));
        *afr_sda |= ((tlong)af_val << ((sda_pin - 8) * 4));
    }

    // Reset I2C peripheral
    // This is typically done via RCC_APB1RSTR_REG
    tlong rst_bit = 0;
    if (i2c_channel == I2C_CH1) rst_bit = (1UL << 21);
    else if (i2c_channel == I2C_CH2) rst_bit = (1UL << 22);
    else if (i2c_channel == I2C_CH3) rst_bit = (1UL << 23);
    *RCC_APB1RSTR_REG |= rst_bit;  // Assert reset - Inferred
    *RCC_APB1RSTR_REG &= ~rst_bit; // Release reset - Inferred

    // Disable I2C peripheral before configuration
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) &= ~(1UL << 0); // Clear PE bit

    // Configure CR2: Peripheral clock frequency (PCLK1 frequency)
    // For F401RC, PCLK1 max is 42MHz. Set FREQ bits to 42.
    *(I2C_REG(i2c_base, I2C_CR2_OFFSET)) &= ~(0x3FUL); // Clear FREQ bits
    *(I2C_REG(i2c_base, I2C_CR2_OFFSET)) |= 42UL; // Set PCLK1 frequency (42MHz) - Inferred

    // Configure CCR: Clock control register (Fast mode, Duty Cycle 2)
    // Rule: Always use fast mode
    // Fast mode calculation: Thigh = CCR * Tpclk1, Tlow = 2 * CCR * Tpclk1 (Duty Cycle 2)
    // For 400kHz: T = 1/400kHz = 2.5us. Thigh = 0.833us, Tlow = 1.667us
    // Assuming PCLK1 = 42MHz, Tpclk1 = 1/42MHz = 23.8ns
    // CCR = Thigh / Tpclk1 = 833ns / 23.8ns = 35.
    // This implies Fast mode (bit 15 = 1), Duty cycle (bit 14 = 1) for 400kHz.
    tlong ccr_val = 0;
    if (i2c_clk_speed == I2C_CLK_SPEED_FAST) {
        ccr_val = 35; // Calculated for 400kHz with Duty cycle 2
        ccr_val |= (1UL << 15); // Fast mode enable
        ccr_val |= (1UL << 14); // Fm mode duty cycle (Tlow/Thigh = 2)
    } else { // Standard mode (100kHz)
        ccr_val = 42000000UL / (2 * 100000); // For 100kHz standard mode, CCR = PCLK1_Freq / (2 * I2C_Freq) = 420/2 = 210
    }
    *(I2C_REG(i2c_base, I2C_CCR_OFFSET)) = ccr_val;

    // Configure TRISE: Max rise time
    // For 400kHz (Fast mode): Max trise = 300ns. TRISE = (300ns / Tpclk1) + 1 = (300/23.8) + 1 = 12.6 + 1 ~ 14.
    *(I2C_REG(i2c_base, I2C_TRISE_OFFSET)) = (tlong)42 * 300 / 1000 + 1; // Calculated for 400kHz. PCLK1 (42MHz) * 300ns = 12.6, round up to 13.
                                                                        // According to doc: TRISE = (Trise_max / PCLK1_period) + 1
                                                                        // For Fm mode, Trise_max = 300ns. 300ns / (1/42MHz) + 1 = 12.6 + 1 = 13.6. Use 14.
                                                                        // The documentation also says TRISE bits should be 0-9. Value should be 1-32.
                                                                        // Given the calculation from the rule, it matches typical config.
    *(I2C_REG(i2c_base, I2C_TRISE_OFFSET)) = 14;

    // Configure Own Address 1 (OAR1)
    *(I2C_REG(i2c_base, I2C_OAR1_OFFSET)) = (tlong)i2c_device_address << 1; // 7-bit address is bits 7:1

    // Configure Acknowledge (CR1.ACK)
    if (i2c_ack == I2C_ACK_ENABLE) {
        *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 10); // ACK bit
    } else {
        *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) &= ~(1UL << 10); // Clear ACK bit
    }

    // Data length: I2C is byte-oriented, t_i2c_datalength is a placeholder for context, no direct register bit
    (void)i2c_datalength; // Suppress unused warning.
}

/**
 * @brief Enables an I2C channel.
 * @param i2c_channel The I2C channel to enable.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // Enable peripheral clock first (RCC)
    WDT_Reset();
    if (i2c_channel == I2C_CH1) {
        *RCC_APB1ENR_REG |= (1UL << 21); // I2C1EN - Inferred
    } else if (i2c_channel == I2C_CH2) {
        *RCC_APB1ENR_REG |= (1UL << 22); // I2C2EN - Inferred
    } else if (i2c_channel == I2C_CH3) {
        *RCC_APB1ENR_REG |= (1UL << 23); // I2C3EN - Inferred
    }

    // Enable I2C (PE bit in CR1)
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 0);
}

/**
 * @brief Disables an I2C channel.
 * @param i2c_channel The I2C channel to disable.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // Disable I2C (PE bit in CR1)
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) &= ~(1UL << 0);

    // Disable peripheral clock (RCC)
    WDT_Reset();
    if (i2c_channel == I2C_CH1) {
        *RCC_APB1ENR_REG &= ~(1UL << 21); // I2C1EN - Inferred
    } else if (i2c_channel == I2C_CH2) {
        *RCC_APB1ENR_REG &= ~(1UL << 22); // I2C2EN - Inferred
    } else if (i2c_channel == I2C_CH3) {
        *RCC_APB1ENR_REG &= ~(1UL << 23); // I2C3EN - Inferred
    }
}

/**
 * @brief Updates the status of an I2C channel.
 * @param i2c_channel The I2C channel to update.
 */
void I2C_Update(t_i2c_channel i2c_channel) {
    WDT_Reset();
    // This function can be used to check status flags like SB, ADDR, BTF, etc.
    // For now, it's a placeholder as per API.json.
    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;
    volatile tlong sr1 = *(I2C_REG(i2c_base, I2C_SR1_OFFSET));
    volatile tlong sr2 = *(I2C_REG(i2c_base, I2C_SR2_OFFSET));
    (void)sr1; // Suppress unused variable warning.
    (void)sr2; // Suppress unused variable warning.
}

/**
 * @brief Sends a single byte over an I2C channel.
 *        Assumes I2C is configured as master.
 * @param i2c_channel The I2C channel to use.
 * @param byte The byte to send.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // A full I2C transaction requires handling START, Address, Data, STOP/ACK.
    // This function only handles sending data *after* START and address phase.
    // Wait for TxE (Transmit Data Register Empty) or BTF (Byte Transfer Finished)
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 7)) && // TXE
           !(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 2))) {  // BTF
        WDT_Reset();
    }
    *(I2C_REG(i2c_base, I2C_DR_OFFSET)) = byte; // Write the byte to DR
}

/**
 * @brief Sends a frame of data over an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param data Pointer to the data buffer.
 * @param length Length of the data frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset();

    // This function needs to handle the I2C master sequence:
    // 1. Generate START condition (CR1.START = 1)
    // 2. Wait for SB (Start Bit) flag in SR1
    // 3. Send slave address (DR) + R/W bit (0 for write)
    // 4. Wait for ADDR (Address sent) flag in SR1
    // 5. Read SR2 to clear ADDR
    // 6. Send data bytes one by one using I2C_send_byte
    // 7. Generate STOP or REPEATED START (CR1.STOP = 1 or CR1.START = 1 again)

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // 1. Generate START condition
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 8); // START bit

    // 2. Wait for SB (Start Bit) flag
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 0))) {
        WDT_Reset(); // Keep WDT happy
    }

    // 3. Send slave address + R/W bit (0 for write)
    *(I2C_REG(i2c_base, I2C_DR_OFFSET)) = (0x00 << 1) & ~0x01; // Slave address assumed to be 0x00, write mode.
                                                              // This needs actual device address!
                                                              // This is a generic implementation, actual slave address should be passed.
                                                              // The API should probably be `I2C_Master_Send(channel, address, data, length)`.
                                                              // Given the API `I2C_send_frame(channel, data, length)`, it implies addressing
                                                              // is handled by the layer or assumes a default slave.
                                                              // The `I2C_Init` takes `i2c_device_address` which is likely own address.

    // Using a placeholder address (e.g., 0x50 for a common EEPROM)
    // For demonstration, let's assume a default slave address like 0x50 (0xA0 for write).
    *(I2C_REG(i2c_base, I2C_DR_OFFSET)) = (0x50 << 1); // Slave address 0x50, write bit = 0

    // 4. Wait for ADDR (Address sent) flag
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 1))) {
        WDT_Reset();
    }

    // 5. Read SR2 to clear ADDR
    (void)*(I2C_REG(i2c_base, I2C_SR2_OFFSET));

    int i;
    for (i = 0; i < length; i++) {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
    }

    // After sending all bytes, wait for BTF (Byte Transfer Finished)
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 2))) {
        WDT_Reset();
    }

    // Generate STOP condition
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 9); // STOP bit
}

/**
 * @brief Sends a null-terminated string over an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset();

    // Use send_frame with string length
    I2C_send_frame(i2c_channel, str, strlen(str));
}

/**
 * @brief Receives a single byte from an I2C channel.
 *        Assumes I2C is configured as master.
 * @param i2c_channel The I2C channel to use.
 * @return The received byte.
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return 0; // Or handle error

    // This function assumes the master has already initiated a read transaction (START + Address + Read bit).
    // Wait for RXNE (Receive Data Register Not Empty)
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 6))) {
        WDT_Reset();
    }

    return (tbyte)*(I2C_REG(i2c_base, I2C_DR_OFFSET)); // Read the byte from DR
}

/**
 * @brief Receives a frame of data from an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // This function needs to handle the I2C master read sequence:
    // 1. Generate START condition (CR1.START = 1)
    // 2. Wait for SB (Start Bit) flag in SR1
    // 3. Send slave address (DR) + R/W bit (1 for read)
    // 4. Wait for ADDR (Address sent) flag in SR1
    // 5. Read SR2 to clear ADDR
    // 6. For N-1 bytes, enable ACK, wait for RXNE, read DR
    // 7. For the last byte, disable ACK, wait for RXNE, read DR, generate STOP

    // Using a placeholder address (e.g., 0x50 for a common EEPROM)
    // For demonstration, let's assume a default slave address like 0x50 (0xA1 for read).

    // 1. Generate START condition
    *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 8); // START bit

    // 2. Wait for SB (Start Bit) flag
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 0))) {
        WDT_Reset();
    }

    // 3. Send slave address + R/W bit (1 for read)
    *(I2C_REG(i2c_base, I2C_DR_OFFSET)) = (0x50 << 1) | 0x01; // Slave address 0x50, read bit = 1

    // 4. Wait for ADDR (Address sent) flag
    while (!(*(I2C_REG(i2c_base, I2C_SR1_OFFSET)) & (1UL << 1))) {
        WDT_Reset();
    }

    // 5. Read SR2 to clear ADDR
    (void)*(I2C_REG(i2c_base, I2C_SR2_OFFSET));

    int i;
    for (i = 0; i < max_length; i++) {
        if (i == max_length - 1) {
            // For the last byte, disable ACK and generate STOP after reading
            *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) &= ~(1UL << 10); // Clear ACK bit
            *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 9); // STOP bit
        } else {
            *(I2C_REG(i2c_base, I2C_CR1_OFFSET)) |= (1UL << 10); // Set ACK bit for non-last bytes
        }

        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
    }
}

/**
 * @brief Receives a null-terminated string from an I2C channel.
 * @param i2c_channel The I2C channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes read.
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset();
    // I2C doesn't inherently transmit null-terminated strings.
    // This implementation will read `max_length - 1` bytes and null-terminate.
    // Or it might imply a protocol where the first byte indicates length.
    // For simplicity, it will read until max_length or a 'null' byte (if protocol supports it).
    int i = 0;
    while (i < max_length - 1) { // Leave space for null terminator
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
        if (buffer[i] == '\0') { // Check for null terminator if protocol uses it
            break;
        }
        i++;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}

/**
 * @brief Clears flags of an I2C channel.
 * @param i2c_channel The I2C channel to clear flags for.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel) {
    WDT_Reset();

    volatile tlong* i2c_base = _get_i2c_base(i2c_channel);
    if (i2c_base == NULL) return;

    // Many I2C flags are cleared by reading SR1 and SR2 in a specific order.
    // Example: ADDR is cleared by reading SR1 followed by reading SR2.
    // OVR is cleared by reading SR1 followed by reading DR.
    // This function performs a general read of SR1 and SR2.
    (void)*(I2C_REG(i2c_base, I2C_SR1_OFFSET));
    (void)*(I2C_REG(i2c_base, I2C_SR2_OFFSET));
}

// =============================================================================
// SPI APIs
// =============================================================================

/**
 * @brief Initializes a SPI channel.
 * @param spi_channel The SPI channel to initialize.
 * @param spi_mode Master or Slave mode.
 * @param spi_cpol Clock Polarity.
 * @param spi_cpha Clock Phase.
 * @param spi_dff Data Frame Format (8-bit or 16-bit).
 * @param spi_bit_order Bit order (MSB first or LSB first).
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // Enable GPIO clock and configure pins (SCK, MISO, MOSI, NSS alternate function)
    t_port sck_port, miso_port, mosi_port, nss_port;
    t_pin sck_pin, miso_pin, mosi_pin, nss_pin;
    tbyte af_val;

    switch (spi_channel) {
        case SPI_CH1: // SPI1 (SCK: PA5/PB3, MISO: PA6/PB4, MOSI: PA7/PB5, NSS: PA4/PA15)
            sck_port = PORT_A; sck_pin = PIN5;
            miso_port = PORT_A; miso_pin = PIN6;
            mosi_port = PORT_A; mosi_pin = PIN7;
            nss_port = PORT_A; nss_pin = PIN4;
            af_val = 5; // AF5 for SPI1
            *RCC_AHB1ENR_REG |= (1UL << 0); // GPIOAEN - Inferred
            break;
        case SPI_CH2: // SPI2 (SCK: PB10/PB13/PC7, MISO: PB14/PC2, MOSI: PB15/PC3, NSS: PB9/PB12)
            sck_port = PORT_B; sck_pin = PIN10;
            miso_port = PORT_B; miso_pin = PIN14;
            mosi_port = PORT_B; mosi_pin = PIN15;
            nss_port = PORT_B; nss_pin = PIN9;
            af_val = 5; // AF5 for SPI2
            *RCC_AHB1ENR_REG |= (1UL << 1); // GPIOBEN - Inferred
            break;
        case SPI_CH3: // SPI3 (SCK: PB3/PC10, MISO: PB4/PC11, MOSI: PB5/PC12, NSS: PA4/PA15)
            sck_port = PORT_B; sck_pin = PIN3;
            miso_port = PORT_B; miso_pin = PIN4;
            mosi_port = PORT_B; mosi_pin = PIN5;
            nss_port = PORT_A; nss_pin = PIN4;
            af_val = 6; // AF6 for SPI3
            *RCC_AHB1ENR_REG |= (1UL << 0) | (1UL << 1); // GPIOAEN, GPIOBEN - Inferred
            break;
        default: return;
    }

    // Helper to configure a single GPIO pin for SPI alternate function
    void _configure_spi_gpio(t_port port, t_pin pin, tbyte af) {
        volatile tlong* moder = _get_gpio_reg_ptr(port, GPIO_MODER_OFFSET);
        volatile tlong* otyper = _get_gpio_reg_ptr(port, GPIO_OTYPER_OFFSET);
        volatile tlong* pupdr = _get_gpio_reg_ptr(port, GPIO_PUPDR_OFFSET);
        volatile tlong* afr = (pin < 8) ? _get_gpio_reg_ptr(port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(port, GPIO_AFRH_OFFSET);

        *moder |= (0b10UL << (pin * 2)); // Alternate function mode
        *otyper &= ~(1UL << pin); // Push-pull
        *pupdr &= ~(0b11UL << (pin * 2)); // No pull-up/pull-down (Floating)

        if (pin < 8) {
            *afr &= ~((0b1111UL) << (pin * 4));
            *afr |= ((tlong)af << (pin * 4));
        } else {
            *afr &= ~((0b1111UL) << ((pin - 8) * 4));
            *afr |= ((tlong)af << ((pin - 8) * 4));
        }
    }

    _configure_spi_gpio(sck_port, sck_pin, af_val);
    _configure_spi_gpio(miso_port, miso_pin, af_val);
    _configure_spi_gpio(mosi_port, mosi_pin, af_val);
    // NSS configuration depends on SW/HW control, here for SW control, NSS pin is GPIO Output
    if (spi_mode == SPI_MODE_MASTER) {
        GPIO_Output_Init(nss_port, nss_pin, 1); // Initialize NSS as output high (chip select inactive)
    } else { // Slave mode, NSS can be input
        _configure_spi_gpio(nss_port, nss_pin, af_val); // If hardware NSS is used
    }


    // Disable SPI before configuration
    *(SPI_REG(spi_base, SPI_CR1_OFFSET)) &= ~(1UL << 6); // Clear SPE bit

    // CR1 Configuration
    tlong cr1_val = 0;

    // SPI Mode (Master/Slave)
    if (spi_mode == SPI_MODE_MASTER) {
        cr1_val |= (1UL << 2); // MSTR (Master Selection)
        // Slave Select always software-controlled:
        cr1_val |= (1UL << 9); // SSM (Software Slave Management)
        cr1_val |= (1UL << 8); // SSI (Internal slave select - sets NSS high for master)
    } else {
        // Master bit is 0 for slave
        // For slave, SSM=1 is also common if NSS pin is not used.
        cr1_val |= (1UL << 9); // SSM (Software Slave Management)
    }

    // Clock Polarity (CPOL)
    if (spi_cpol == SPI_CPOL_HIGH) {
        cr1_val |= (1UL << 1);
    }

    // Clock Phase (CPHA)
    if (spi_cpha == SPI_CPHA_SECOND) {
        cr1_val |= (1UL << 0);
    }

    // Data Frame Format (DFF)
    if (spi_dff == SPI_DFF_16BIT) {
        cr1_val |= (1UL << 11);
    }

    // Bit Order (LSBFIRST)
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        cr1_val |= (1UL << 7);
    }

    // Always use full duplex (BIDIMODE = 0) - default state, no bit set
    cr1_val &= ~(1UL << 15); // Clear BIDIMODE bit

    // Always enable CRC (CRCEN)
    cr1_val |= (1UL << 13); // CRCEN bit

    // Baud Rate (BR[2:0]) - Rule: Always use fast speed. This implies lowest prescaler.
    // PCLK2 for SPI1 (max 84MHz), PCLK1 for SPI2/3 (max 42MHz)
    // Smallest prescaler (div2)
    // For SPI1 (84MHz PCLK2), div2 = 42MHz SPI clock
    // For SPI2/3 (42MHz PCLK1), div2 = 21MHz SPI clock
    cr1_val &= ~(0b111UL << 3); // Clear BR bits
    cr1_val |= (0b000UL << 3);  // PCLK/2 (000 for BR)

    *(SPI_REG(spi_base, SPI_CR1_OFFSET)) = cr1_val;

    // CR2 Configuration
    // Typically used for SS output enable, DMA requests, interrupts
    *(SPI_REG(spi_base, SPI_CR2_OFFSET)) = 0x00; // Reset CR2
}

/**
 * @brief Enables a SPI channel.
 * @param spi_channel The SPI channel to enable.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // Enable peripheral clock first (RCC)
    WDT_Reset();
    if (spi_channel == SPI_CH1) {
        *RCC_APB2ENR_REG |= (1UL << 0); // SPI1EN - Inferred
    } else if (spi_channel == SPI_CH2) {
        *RCC_APB1ENR_REG |= (1UL << 14); // SPI2EN - Inferred
    } else if (spi_channel == SPI_CH3) {
        *RCC_APB1ENR_REG |= (1UL << 15); // SPI3EN - Inferred
    }

    // Enable SPI (SPE bit in CR1)
    *(SPI_REG(spi_base, SPI_CR1_OFFSET)) |= (1UL << 6);
}

/**
 * @brief Disables a SPI channel.
 * @param spi_channel The SPI channel to disable.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // Disable SPI (SPE bit in CR1)
    *(SPI_REG(spi_base, SPI_CR1_OFFSET)) &= ~(1UL << 6);

    // Disable peripheral clock (RCC)
    WDT_Reset();
    if (spi_channel == SPI_CH1) {
        *RCC_APB2ENR_REG &= ~(1UL << 0); // SPI1EN - Inferred
    } else if (spi_channel == SPI_CH2) {
        *RCC_APB1ENR_REG &= ~(1UL << 14); // SPI2EN - Inferred
    } else if (spi_channel == SPI_CH3) {
        *RCC_APB1ENR_REG &= ~(1UL << 15); // SPI3EN - Inferred
    }
}

/**
 * @brief Updates the status of SPI.
 *        This function is a placeholder for any periodic status updates.
 */
void SPI_Update(void) {
    WDT_Reset();
    // This function can be used to check status flags like RXNE, TXE, BSY, etc.
    // For now, it's a placeholder as per API.json.
    // As it doesn't take a channel, it implies a global SPI status or common logic.
    // In a real system, you'd likely pass the channel.
    // No specific global SPI register exists, so this is a no-op placeholder.
}

/**
 * @brief Sends a single byte over a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param byte The byte to send.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // Wait until Transmit Buffer Empty (TXE) flag is set
    while (!(*(SPI_REG(spi_base, SPI_SR_OFFSET)) & (1UL << 1))) {
        WDT_Reset();
    }
    // Write data to Data Register (DR)
    *(SPI_REG(spi_base, SPI_DR_OFFSET)) = (tword)byte; // Write 8-bit to 16-bit DR
}

/**
 * @brief Sends a frame of data over a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param data Pointer to the data buffer.
 * @param length Length of the data frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // For Master mode, optionally pull NSS low here.
    // GPIO_Value_Set(nss_port, nss_pin, 0); // Need to know NSS pin for the channel

    int i;
    for (i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
        // Also need to receive data in full duplex
        while (!(*(SPI_REG(spi_base, SPI_SR_OFFSET)) & (1UL << 0))) { // Wait for RXNE
            WDT_Reset();
        }
        (void)*(SPI_REG(spi_base, SPI_DR_OFFSET)); // Dummy read to clear RXNE
    }

    // Wait until SPI is not busy (BSY flag)
    while ((*(SPI_REG(spi_base, SPI_SR_OFFSET)) & (1UL << 7))) {
        WDT_Reset();
    }

    // For Master mode, optionally pull NSS high here.
    // GPIO_Value_Set(nss_port, nss_pin, 1);
}

/**
 * @brief Sends a null-terminated string over a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param str Pointer to the null-terminated string.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str) {
    WDT_Reset();
    SPI_send_frame(spi_channel, str, strlen(str));
}

/**
 * @brief Receives a single byte from a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @return The received byte.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return 0; // Or handle error

    // To receive, you typically also transmit a dummy byte in full-duplex mode
    SPI_Send_Byte(spi_channel, 0xFF); // Send dummy byte

    // Wait until Receive Buffer Not Empty (RXNE) flag is set
    while (!(*(SPI_REG(spi_base, SPI_SR_OFFSET)) & (1UL << 0))) {
        WDT_Reset();
    }

    return (tbyte)(*(SPI_REG(spi_base, SPI_DR_OFFSET)) & 0xFFUL);
}

/**
 * @brief Receives a frame of data from a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length Maximum length of the buffer.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // For Master mode, optionally pull NSS low here.
    // GPIO_Value_Set(nss_port, nss_pin, 0); // Need to know NSS pin for the channel

    int i;
    for (i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }

    // Wait until SPI is not busy (BSY flag)
    while ((*(SPI_REG(spi_base, SPI_SR_OFFSET)) & (1UL << 7))) {
        WDT_Reset();
    }

    // For Master mode, optionally pull NSS high here.
    // GPIO_Value_Set(nss_port, nss_pin, 1);
}

/**
 * @brief Receives a null-terminated string from a SPI channel.
 * @param spi_channel The SPI channel to use.
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length Maximum length of the buffer.
 * @return Number of bytes read.
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset();
    // SPI doesn't inherently transmit null-terminated strings.
    // This will read `max_length - 1` bytes and null-terminate.
    int i = 0;
    while (i < max_length - 1) { // Leave space for null terminator
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
        if (buffer[i] == '\0') { // Check for null terminator if protocol uses it
            break;
        }
        i++;
    }
    buffer[i] = '\0'; // Null-terminate the string
    return (tbyte)i;
}

/**
 * @brief Clears flags of a SPI channel.
 * @param spi_channel The SPI channel to clear flags for.
 */
void SPI_ClearFlag(t_spi_channel spi_channel) {
    WDT_Reset();

    volatile tlong* spi_base = _get_spi_base(spi_channel);
    if (spi_base == NULL) return;

    // Clear specific SPI flags by reading SR.
    // OVR (Overrun) flag is cleared by reading SR then reading DR.
    // CRCERR (CRC Error) is cleared by writing 0 to the bit.
    // This implementation will read SR.
    (void)*(SPI_REG(spi_base, SPI_SR_OFFSET));

    // For CRCERR, clear it specifically (if desired to reset error state)
    // *(SPI_REG(spi_base, SPI_SR_OFFSET)) &= ~(1UL << 4); // This might not work as intended for all flags
    // Some flags are cleared by specific actions (e.g., reading DR for RXNE, BSY by end of transfer).
}

// =============================================================================
// External Interrupt APIs
// =============================================================================

/**
 * @brief Initializes an External Interrupt channel.
 * @param external_int_channel The EXTI line channel (EXTI_LINE0 to EXTI_LINE15).
 * @param external_int_edge The trigger edge (rising, falling, or both).
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset();

    // Enable SYSCFG clock as it maps EXTI lines to GPIO ports
    *RCC_APB2ENR_REG |= (1UL << 14); // SYSCFGEN - Inferred

    tbyte port_code = 0; // 0 for PAx, 1 for PBx, ...
    tbyte pin_num = (tbyte)external_int_channel;

    // Determine the SYSCFG_EXTICR register and relevant port code
    volatile tlong* exticr_reg;
    tbyte exticr_idx = pin_num / 4; // Which EXTICR register (1-4)
    tbyte exticr_shift = (pin_num % 4) * 4; // Shift within the register (4 bits per pin)

    switch (exticr_idx) {
        case 0: exticr_reg = SYSCFG_EXTICR1_REG; break;
        case 1: exticr_reg = SYSCFG_EXTICR2_REG; break;
        case 2: exticr_reg = SYSCFG_EXTICR3_REG; break;
        case 3: exticr_reg = SYSCFG_EXTICR4_REG; break;
        default: return; // Invalid EXTI line
    }

    // This implementation needs to determine the port_code from the assigned pin in register_json
    // However, the `external_int_channel` is just a line number (0-15).
    // The SYSCFG_EXTICR registers allow *any* GPIO pin (PAx, PBx, etc.) for a specific EXTI line.
    // Without knowing which GPIO port is desired, this must be generic.
    // For simplicity, let's assume Port A for the EXTI source. (0b00)
    // In a real application, the user would specify the GPIO port.
    port_code = 0b00; // Assuming Port A as the source for the EXTI line

    // Clear relevant bits and set port selection
    *exticr_reg &= ~((0b1111UL) << exticr_shift);
    *exticr_reg |= ((tlong)port_code << exticr_shift);

    // Configure trigger edge
    if (external_int_edge == EXTI_EDGE_RISING || external_int_edge == EXTI_EDGE_RISING_FALLING) {
        *EXTI_RTSR_REG |= (1UL << pin_num); // Enable rising trigger
    } else {
        *EXTI_RTSR_REG &= ~(1UL << pin_num); // Disable rising trigger
    }

    if (external_int_edge == EXTI_EDGE_FALLING || external_int_edge == EXTI_EDGE_RISING_FALLING) {
        *EXTI_FTSR_REG |= (1UL << pin_num); // Enable falling trigger
    } else {
        *EXTI_FTSR_REG &= ~(1UL << pin_num); // Disable falling trigger
    }
}

/**
 * @brief Enables an External Interrupt channel.
 * @param external_int_channel The EXTI line channel to enable.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset();

    // Enable peripheral clock for SYSCFG (if not already enabled by init)
    *RCC_APB2ENR_REG |= (1UL << 14); // SYSCFGEN - Inferred

    // Set IMR bit (Interrupt Mask Register) to unmask the interrupt line
    *EXTI_IMR_REG |= (1UL << (tbyte)external_int_channel);
}

/**
 * @brief Disables an External Interrupt channel.
 * @param external_int_channel The EXTI line channel to disable.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset();

    // Clear IMR bit to mask the interrupt line
    *EXTI_IMR_REG &= ~(1UL << (tbyte)external_int_channel);
}

/**
 * @brief Clears the pending flag for an External Interrupt channel.
 * @param external_int_channel The EXTI line channel to clear the flag for.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel) {
    WDT_Reset();

    // Clear pending register (PR) by writing 1 to the bit
    *EXTI_PR_REG = (1UL << (tbyte)external_int_channel);
}

// =============================================================================
// GPIO APIs
// =============================================================================

/**
 * @brief Initializes a GPIO pin as an output.
 * @param port The GPIO port (e.g., PORT_A).
 * @param pin The pin number (e.g., PIN0).
 * @param value Initial value to set for the output pin (0 or 1).
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return;

    // Enable GPIO clock first
    WDT_Reset();
    if (port == PORT_A) *RCC_AHB1ENR_REG |= (1UL << 0); // GPIOAEN - Inferred
    else if (port == PORT_B) *RCC_AHB1ENR_REG |= (1UL << 1); // GPIOBEN - Inferred
    else if (port == PORT_C) *RCC_AHB1ENR_REG |= (1UL << 2); // GPIOCEN - Inferred
    else if (port == PORT_D) *RCC_AHB1ENR_REG |= (1UL << 3); // GPIODEN - Inferred
    else if (port == PORT_E) *RCC_AHB1ENR_REG |= (1UL << 4); // GPIOEEN - Inferred
    else if (port == PORT_H) *RCC_AHB1ENR_REG |= (1UL << 7); // GPIOHEN - Inferred

    volatile tlong* moder_reg = _get_gpio_reg_ptr(port, GPIO_MODER_OFFSET);
    volatile tlong* otyper_reg = _get_gpio_reg_ptr(port, GPIO_OTYPER_OFFSET);
    volatile tlong* ospeedr_reg = _get_gpio_reg_ptr(port, GPIO_OSPEEDR_OFFSET);
    volatile tlong* pupdr_reg = _get_gpio_reg_ptr(port, GPIO_PUPDR_OFFSET);

    // Always set value before setting direction
    GPIO_Value_Set(port, pin, value); // Set initial output value

    // Configure mode to Output (01)
    *moder_reg &= ~(0b11UL << (pin * 2)); // Clear mode bits
    *moder_reg |= (0b01UL << (pin * 2));  // Set to output mode (01)

    // Verify mode with while loop
    while (((*moder_reg >> (pin * 2)) & 0b11UL) != 0b01UL) {
        WDT_Reset();
    }

    // Output Type: Push-pull (0) by default, OTYPER bit cleared
    *otyper_reg &= ~(1UL << pin);

    // Output Speed: High speed (10)
    *ospeedr_reg &= ~(0b11UL << (pin * 2)); // Clear speed bits
    *ospeedr_reg |= (0b10UL << (pin * 2));  // Set to High speed (10) - Inferred for >=20mA sink/source

    // Pull-up/Pull-down: Disabled (00) for output pins
    *pupdr_reg &= ~(0b11UL << (pin * 2)); // Clear pull-up/pull-down bits
}

/**
 * @brief Initializes a GPIO pin as an input.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return;

    // Enable GPIO clock first
    WDT_Reset();
    if (port == PORT_A) *RCC_AHB1ENR_REG |= (1UL << 0); // GPIOAEN - Inferred
    else if (port == PORT_B) *RCC_AHB1ENR_REG |= (1UL << 1); // GPIOBEN - Inferred
    else if (port == PORT_C) *RCC_AHB1ENR_REG |= (1UL << 2); // GPIOCEN - Inferred
    else if (port == PORT_D) *RCC_AHB1ENR_REG |= (1UL << 3); // GPIODEN - Inferred
    else if (port == PORT_E) *RCC_AHB1ENR_REG |= (1UL << 4); // GPIOEEN - Inferred
    else if (port == PORT_H) *RCC_AHB1ENR_REG |= (1UL << 7); // GPIOHEN - Inferred

    volatile tlong* moder_reg = _get_gpio_reg_ptr(port, GPIO_MODER_OFFSET);
    volatile tlong* pupdr_reg = _get_gpio_reg_ptr(port, GPIO_PUPDR_OFFSET);

    // Configure mode to Input (00)
    *moder_reg &= ~(0b11UL << (pin * 2)); // Clear mode bits (already 00, but explicit)

    // Verify mode with while loop
    while (((*moder_reg >> (pin * 2)) & 0b11UL) != 0b00UL) {
        WDT_Reset();
    }

    // Pull-up resistors enabled (01) for all input pins
    *pupdr_reg &= ~(0b11UL << (pin * 2)); // Clear pull-up/pull-down bits
    *pupdr_reg |= (0b01UL << (pin * 2));  // Set to Pull-up (01)

    // No specific "wakeup feature enabled" in GPIO registers, typically done via EXTI.
}

/**
 * @brief Gets the direction of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The direction of the pin (INPUT or OUTPUT).
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return INPUT; // Default to input on error

    volatile tlong* moder_reg = _get_gpio_reg_ptr(port, GPIO_MODER_OFFSET);
    tbyte mode = (tbyte)((*moder_reg >> (pin * 2)) & 0b11UL);

    if (mode == 0b01UL) { // 01 for General purpose output mode
        return OUTPUT;
    } else { // 00 for Input mode, 10 for Alternate function, 11 for Analog
        return INPUT;
    }
}

/**
 * @brief Sets the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param value The value to set (0 for low, 1 for high).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return;

    volatile tlong* bsrr_reg = _get_gpio_reg_ptr(port, GPIO_BSRR_OFFSET);

    if (value == 1) {
        *bsrr_reg = (1UL << pin); // Set bit
    } else {
        *bsrr_reg = (1UL << (pin + 16)); // Reset bit (via BRy)
    }

    // Verify value with while loop (read ODR)
    volatile tlong* odr_reg = _get_gpio_reg_ptr(port, GPIO_ODR_OFFSET);
    if (value == 1) {
        while (!((*odr_reg >> pin) & 1UL)) {
            WDT_Reset();
        }
    } else {
        while (((*odr_reg >> pin) & 1UL)) {
            WDT_Reset();
        }
    }
}

/**
 * @brief Gets the value of a GPIO input pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @return The value of the pin (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return 0;

    volatile tlong* idr_reg = _get_gpio_reg_ptr(port, GPIO_IDR_OFFSET);
    return (tbyte)((*idr_reg >> pin) & 1UL);
}

/**
 * @brief Toggles the value of a GPIO output pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset();

    volatile tlong* gpio_base = _get_gpio_base(port);
    if (gpio_base == NULL) return;

    volatile tlong* odr_reg = _get_gpio_reg_ptr(port, GPIO_ODR_OFFSET);
    volatile tlong* bsrr_reg = _get_gpio_reg_ptr(port, GPIO_BSRR_OFFSET);

    if ((*odr_reg >> pin) & 1UL) {
        // Pin is high, set low
        *bsrr_reg = (1UL << (pin + 16));
    } else {
        // Pin is low, set high
        *bsrr_reg = (1UL << pin);
    }
    // No direct verification for toggle, as it depends on initial state.
    // The previous GPIO_Value_Set verifies single set/reset operations.
}

// =============================================================================
// PWM APIs
// =============================================================================

/**
 * @brief Initializes a PWM channel.
 * @param pwm_channel The PWM channel to initialize.
 * @param pwm_khz_freq PWM frequency in kHz.
 * @param pwm_duty PWM duty cycle (0-100%).
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;
    t_port pwm_port;
    t_pin pwm_pin;
    tbyte af_val;

    // Map PWM channel to specific Timer, Channel number, and GPIO pin
    switch (pwm_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; pwm_port = PORT_A; pwm_pin = PIN8; af_val = 1; break; // PA8/AF1
        case PWM_TIM1_CH2:  timer_channel = TIMER_TIM1; tim_ch_num = 2; pwm_port = PORT_A; pwm_pin = PIN9; af_val = 1; break; // PA9/AF1
        case PWM_TIM1_CH3:  timer_channel = TIMER_TIM1; tim_ch_num = 3; pwm_port = PORT_A; pwm_pin = PIN10; af_val = 1; break; // PA10/AF1
        case PWM_TIM1_CH4:  timer_channel = TIMER_TIM1; tim_ch_num = 4; pwm_port = PORT_A; pwm_pin = PIN11; af_val = 1; break; // PA11/AF1
        case PWM_TIM2_CH1:  timer_channel = TIMER_TIM2; tim_ch_num = 1; pwm_port = PORT_A; pwm_pin = PIN0; af_val = 1; break; // PA0/AF1
        case PWM_TIM2_CH2:  timer_channel = TIMER_TIM2; tim_ch_num = 2; pwm_port = PORT_A; pwm_pin = PIN1; af_val = 1; break; // PA1/AF1
        case PWM_TIM2_CH3:  timer_channel = TIMER_TIM2; tim_ch_num = 3; pwm_port = PORT_A; pwm_pin = PIN2; af_val = 1; break; // PA2/AF1
        case PWM_TIM2_CH4:  timer_channel = TIMER_TIM2; tim_ch_num = 4; pwm_port = PORT_A; pwm_pin = PIN3; af_val = 1; break; // PA3/AF1
        case PWM_TIM3_CH1:  timer_channel = TIMER_TIM3; tim_ch_num = 1; pwm_port = PORT_A; pwm_pin = PIN6; af_val = 2; break; // PA6/AF2
        case PWM_TIM3_CH2:  timer_channel = TIMER_TIM3; tim_ch_num = 2; pwm_port = PORT_A; pwm_pin = PIN7; af_val = 2; break; // PA7/AF2
        case PWM_TIM3_CH3:  timer_channel = TIMER_TIM3; tim_ch_num = 3; pwm_port = PORT_B; pwm_pin = PIN0; af_val = 2; break; // PB0/AF2
        case PWM_TIM3_CH4:  timer_channel = TIMER_TIM3; tim_ch_num = 4; pwm_port = PORT_B; pwm_pin = PIN1; af_val = 2; break; // PB1/AF2
        case PWM_TIM4_CH1:  timer_channel = TIMER_TIM4; tim_ch_num = 1; pwm_port = PORT_D; pwm_pin = PIN12; af_val = 2; break; // PD12/AF2
        case PWM_TIM4_CH2:  timer_channel = TIMER_TIM4; tim_ch_num = 2; pwm_port = PORT_D; pwm_pin = PIN13; af_val = 2; break; // PD13/AF2
        case PWM_TIM4_CH3:  timer_channel = TIMER_TIM4; tim_ch_num = 3; pwm_port = PORT_D; pwm_pin = PIN14; af_val = 2; break; // PD14/AF2
        case PWM_TIM4_CH4:  timer_channel = TIMER_TIM4; tim_ch_num = 4; pwm_port = PORT_D; pwm_pin = PIN15; af_val = 2; break; // PD15/AF2
        case PWM_TIM5_CH1:  timer_channel = TIMER_TIM5; tim_ch_num = 1; pwm_port = PORT_A; pwm_pin = PIN0; af_val = 2; break; // PA0/AF2
        case PWM_TIM5_CH2:  timer_channel = TIMER_TIM5; tim_ch_num = 2; pwm_port = PORT_A; pwm_pin = PIN1; af_val = 2; break; // PA1/AF2
        case PWM_TIM5_CH3:  timer_channel = TIMER_TIM5; tim_ch_num = 3; pwm_port = PORT_A; pwm_pin = PIN2; af_val = 2; break; // PA2/AF2
        case PWM_TIM5_CH4:  timer_channel = TIMER_TIM5; tim_ch_num = 4; pwm_port = PORT_A; pwm_pin = PIN3; af_val = 2; break; // PA3/AF2
        case PWM_TIM9_CH1:  timer_channel = TIMER_TIM9; tim_ch_num = 1; pwm_port = PORT_A; pwm_pin = PIN2; af_val = 3; break; // PA2/AF3
        case PWM_TIM9_CH2:  timer_channel = TIMER_TIM9; tim_ch_num = 2; pwm_port = PORT_A; pwm_pin = PIN3; af_val = 3; break; // PA3/AF3
        case PWM_TIM10_CH1: timer_channel = TIMER_TIM10; tim_ch_num = 1; pwm_port = PORT_B; pwm_pin = PIN8; af_val = 3; break; // PB8/AF3
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; pwm_port = PORT_B; pwm_pin = PIN9; af_val = 3; break; // PB9/AF3
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable Timer clock first
    WDT_Reset();
    if (timer_channel == TIMER_TIM1 || timer_channel == TIMER_TIM9 || timer_channel == TIMER_TIM10 || timer_channel == TIMER_TIM11) {
        // APB2 Timers
        *RCC_APB2ENR_REG |= (1UL << (timer_channel == TIMER_TIM1 ? 0 : (timer_channel == TIMER_TIM9 ? 16 : (timer_channel == TIMER_TIM10 ? 17 : 18)))); // Inferred
    } else {
        // APB1 Timers
        *RCC_APB1ENR_REG |= (1UL << (timer_channel == TIMER_TIM2 ? 0 : (timer_channel == TIMER_TIM3 ? 1 : (timer_channel == TIMER_TIM4 ? 2 : 3)))); // Inferred
    }

    // Configure GPIO pin for Alternate Function Push-Pull
    volatile tlong* moder_gpio = _get_gpio_reg_ptr(pwm_port, GPIO_MODER_OFFSET);
    volatile tlong* otyper_gpio = _get_gpio_reg_ptr(pwm_port, GPIO_OTYPER_OFFSET);
    volatile tlong* pupdr_gpio = _get_gpio_reg_ptr(pwm_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_gpio = (pwm_pin < 8) ? _get_gpio_reg_ptr(pwm_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(pwm_port, GPIO_AFRH_OFFSET);

    *moder_gpio |= (0b10UL << (pwm_pin * 2)); // Alternate function mode
    *otyper_gpio &= ~(1UL << pwm_pin); // Push-pull
    *pupdr_gpio &= ~(0b11UL << (pwm_pin * 2)); // No pull-up/pull-down

    if (pwm_pin < 8) {
        *afr_gpio &= ~((0b1111UL) << (pwm_pin * 4));
        *afr_gpio |= ((tlong)af_val << (pwm_pin * 4));
    } else {
        *afr_gpio &= ~((0b1111UL) << ((pwm_pin - 8) * 4));
        *afr_gpio |= ((tlong)af_val << ((pwm_pin - 8) * 4));
    }

    // Disable timer before configuration
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0); // CEN bit

    // Configure Prescaler (PSC) and Auto-Reload Register (ARR) for desired frequency
    // Assuming Timer Clock is 84MHz (for APB2 timers like TIM1, TIM9,10,11) or 42MHz (for APB1 timers like TIM2-5)
    tlong tim_clk_freq;
    if (timer_channel == TIMER_TIM1 || timer_channel == TIMER_TIM9 || timer_channel == TIMER_TIM10 || timer_channel == TIMER_TIM11) {
        tim_clk_freq = 84000000UL; // APB2 Timers
    } else {
        tim_clk_freq = 42000000UL; // APB1 Timers
    }

    // Period = (ARR + 1) * (PSC + 1) / Clock_Freq
    // Target Frequency (Hz) = Clock_Freq / ((ARR + 1) * (PSC + 1))
    // To achieve pwm_khz_freq: (ARR + 1) * (PSC + 1) = Clock_Freq / (pwm_khz_freq * 1000)
    // Choose PSC for a reasonable ARR value (e.g., ARR max 65535 for 16-bit timers)
    // For 1kHz (pwm_khz_freq=1), Clock=84MHz -> (ARR+1)*(PSC+1) = 84000
    // If PSC = 839, ARR = 99 (840 * 100 = 84000)
    tword prescaler = (tword)((tim_clk_freq / (pwm_khz_freq * 1000UL)) / 1000UL); // Example for PSC ~1000
    if (prescaler == 0) prescaler = 1; // Minimum prescaler
    tword auto_reload = (tword)((tim_clk_freq / (pwm_khz_freq * 1000UL)) / prescaler) - 1;

    *(TIM_REG(tim_base, TIM_PSC_OFFSET)) = prescaler - 1;
    *(TIM_REG(tim_base, TIM_ARR_OFFSET)) = auto_reload;

    // Configure Capture/Compare Mode Register (CCMRx) for PWM Mode 1
    volatile tlong* ccmr_reg;
    volatile tlong* ccr_reg;
    tbyte ccm_shift;

    if (tim_ch_num == 1 || tim_ch_num == 2) {
        ccmr_reg = TIM_REG(tim_base, TIM_CCMR1_OFFSET);
        ccm_shift = (tim_ch_num == 1) ? 0 : 8; // CC1S/OC1M bits
        ccr_reg = TIM_REG(tim_base, TIM_CCR1_OFFSET); // Default
        if (tim_ch_num == 2) ccr_reg = TIM_REG(tim_base, TIM_CCR2_OFFSET);
    } else if (tim_ch_num == 3 || tim_ch_num == 4) {
        ccmr_reg = TIM_REG(tim_base, TIM_CCMR2_OFFSET);
        ccm_shift = (tim_ch_num == 3) ? 0 : 8; // CC3S/OC3M bits
        ccr_reg = TIM_REG(tim_base, TIM_CCR3_OFFSET); // Default
        if (tim_ch_num == 4) ccr_reg = TIM_REG(tim_base, TIM_CCR4_OFFSET);
    } else {
        return; // Invalid channel number
    }
    
    // Set OCxM to PWM Mode 1 (0110)
    *ccmr_reg &= ~(0b111UL << (ccm_shift + 4)); // Clear OCxM bits
    *ccmr_reg |= (0b110UL << (ccm_shift + 4));  // Set OCxM to PWM Mode 1

    *ccmr_reg |= (1UL << (ccm_shift + 3));      // OCxPE (Output compare preload enable)

    // Set Duty Cycle (CCRx)
    tlong pulse_len = (tlong)(((auto_reload + 1) * pwm_duty) / 100UL);
    *ccr_reg = pulse_len;

    // Configure Capture/Compare Enable Register (CCER)
    // Enable Output for selected channel (CCxE bit)
    *(TIM_REG(tim_base, TIM_CCER_OFFSET)) |= (1UL << ((tim_ch_num - 1) * 4)); // CCxE

    // For Advanced Timers (TIM1), enable Main Output Enable (MOE) in BDTR
    if (timer_channel == TIMER_TIM1) {
        *(TIM_REG(tim_base, TIM_BDTR_OFFSET)) |= (1UL << 15); // MOE bit
    }

    // Clear available FREQUENCY Ranges for each channel as comments in PWM_Init()
    // PWM_Channel_TIM1_CH1 //PA8,PE9 (Freq Range: PCLK2/ (PSC+1)(ARR+1) )
    // PWM_Channel_TIM2_CH1 //PA0,PA5,PA15 (Freq Range: PCLK1/ (PSC+1)(ARR+1) )
    // ... similar for all PWM channels and their respective PCLKs
}

/**
 * @brief Starts PWM generation on a channel.
 * @param pwm_channel The PWM channel to start.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;

    // Map PWM channel to specific Timer and Channel number
    switch (pwm_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; break;
        case PWM_TIM1_CH2:  timer_channel = TIMER_TIM1; tim_ch_num = 2; break;
        case PWM_TIM1_CH3:  timer_channel = TIMER_TIM1; tim_ch_num = 3; break;
        case PWM_TIM1_CH4:  timer_channel = TIMER_TIM1; tim_ch_num = 4; break;
        case PWM_TIM2_CH1:  timer_channel = TIMER_TIM2; tim_ch_num = 1; break;
        case PWM_TIM2_CH2:  timer_channel = TIMER_TIM2; tim_ch_num = 2; break;
        case PWM_TIM2_CH3:  timer_channel = TIMER_TIM2; tim_ch_num = 3; break;
        case PWM_TIM2_CH4:  timer_channel = TIMER_TIM2; tim_ch_num = 4; break;
        case PWM_TIM3_CH1:  timer_channel = TIMER_TIM3; tim_ch_num = 1; break;
        case PWM_TIM3_CH2:  timer_channel = TIMER_TIM3; tim_ch_num = 2; break;
        case PWM_TIM3_CH3:  timer_channel = TIMER_TIM3; tim_ch_num = 3; break;
        case PWM_TIM3_CH4:  timer_channel = TIMER_TIM3; tim_ch_num = 4; break;
        case PWM_TIM4_CH1:  timer_channel = TIMER_TIM4; tim_ch_num = 1; break;
        case PWM_TIM4_CH2:  timer_channel = TIMER_TIM4; tim_ch_num = 2; break;
        case PWM_TIM4_CH3:  timer_channel = TIMER_TIM4; tim_ch_num = 3; break;
        case PWM_TIM4_CH4:  timer_channel = TIMER_TIM4; tim_ch_num = 4; break;
        case PWM_TIM5_CH1:  timer_channel = TIMER_TIM5; tim_ch_num = 1; break;
        case PWM_TIM5_CH2:  timer_channel = TIMER_TIM5; tim_ch_num = 2; break;
        case PWM_TIM5_CH3:  timer_channel = TIMER_TIM5; tim_ch_num = 3; break;
        case PWM_TIM5_CH4:  timer_channel = TIMER_TIM5; tim_ch_num = 4; break;
        case PWM_TIM9_CH1:  timer_channel = TIMER_TIM9; tim_ch_num = 1; break;
        case PWM_TIM9_CH2:  timer_channel = TIMER_TIM9; tim_ch_num = 2; break;
        case PWM_TIM10_CH1: timer_channel = TIMER_TIM10; tim_ch_num = 1; break;
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; break;
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) |= (1UL << 0);
}

/**
 * @brief Stops PWM generation on a channel.
 * @param pwm_channel The PWM channel to stop.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;

    // Map PWM channel to specific Timer and Channel number
    switch (pwm_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; break;
        case PWM_TIM1_CH2:  timer_channel = TIMER_TIM1; tim_ch_num = 2; break;
        case PWM_TIM1_CH3:  timer_channel = TIMER_TIM1; tim_ch_num = 3; break;
        case PWM_TIM1_CH4:  timer_channel = TIMER_TIM1; tim_ch_num = 4; break;
        case PWM_TIM2_CH1:  timer_channel = TIMER_TIM2; tim_ch_num = 1; break;
        case PWM_TIM2_CH2:  timer_channel = TIMER_TIM2; tim_ch_num = 2; break;
        case PWM_TIM2_CH3:  timer_channel = TIMER_TIM2; tim_ch_num = 3; break;
        case PWM_TIM2_CH4:  timer_channel = TIMER_TIM2; tim_ch_num = 4; break;
        case PWM_TIM3_CH1:  timer_channel = TIMER_TIM3; tim_ch_num = 1; break;
        case PWM_TIM3_CH2:  timer_channel = TIMER_TIM3; tim_ch_num = 2; break;
        case PWM_TIM3_CH3:  timer_channel = TIMER_TIM3; tim_ch_num = 3; break;
        case PWM_TIM3_CH4:  timer_channel = TIMER_TIM3; tim_ch_num = 4; break;
        case PWM_TIM4_CH1:  timer_channel = TIMER_TIM4; tim_ch_num = 1; break;
        case PWM_TIM4_CH2:  timer_channel = TIMER_TIM4; tim_ch_num = 2; break;
        case PWM_TIM4_CH3:  timer_channel = TIMER_TIM4; tim_ch_num = 3; break;
        case PWM_TIM4_CH4:  timer_channel = TIMER_TIM4; tim_ch_num = 4; break;
        case PWM_TIM5_CH1:  timer_channel = TIMER_TIM5; tim_ch_num = 1; break;
        case PWM_TIM5_CH2:  timer_channel = TIMER_TIM5; tim_ch_num = 2; break;
        case PWM_TIM5_CH3:  timer_channel = TIMER_TIM5; tim_ch_num = 3; break;
        case PWM_TIM5_CH4:  timer_channel = TIMER_TIM5; tim_ch_num = 4; break;
        case PWM_TIM9_CH1:  timer_channel = TIMER_TIM9; tim_ch_num = 1; break;
        case PWM_TIM9_CH2:  timer_channel = TIMER_TIM9; tim_ch_num = 2; break;
        case PWM_TIM10_CH1: timer_channel = TIMER_TIM10; tim_ch_num = 1; break;
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; break;
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Disable counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0);
}

// =============================================================================
// ICU APIs
// =============================================================================

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to initialize.
 * @param icu_prescaller Prescaler value for the timer.
 * @param icu_edge The edge to trigger on (rising, falling, or both).
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;
    t_port icu_port;
    t_pin icu_pin;
    tbyte af_val;

    // Map ICU channel to specific Timer, Channel number, and GPIO pin
    switch (icu_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; icu_port = PORT_A; icu_pin = PIN8; af_val = 1; break; // PA8/AF1
        case PWM_TIM1_CH2:  timer_channel = TIMER_TIM1; tim_ch_num = 2; icu_port = PORT_A; icu_pin = PIN9; af_val = 1; break; // PA9/AF1
        case PWM_TIM1_CH3:  timer_channel = TIMER_TIM1; tim_ch_num = 3; icu_port = PORT_A; icu_pin = PIN10; af_val = 1; break; // PA10/AF1
        case PWM_TIM1_CH4:  timer_channel = TIMER_TIM1; tim_ch_num = 4; icu_port = PORT_A; icu_pin = PIN11; af_val = 1; break; // PA11/AF1
        case PWM_TIM2_CH1:  timer_channel = TIMER_TIM2; tim_ch_num = 1; icu_port = PORT_A; icu_pin = PIN0; af_val = 1; break; // PA0/AF1
        case PWM_TIM2_CH2:  timer_channel = TIMER_TIM2; tim_ch_num = 2; icu_port = PORT_A; icu_pin = PIN1; af_val = 1; break; // PA1/AF1
        case PWM_TIM2_CH3:  timer_channel = TIMER_TIM2; tim_ch_num = 3; icu_port = PORT_A; icu_pin = PIN2; af_val = 1; break; // PA2/AF1
        case PWM_TIM2_CH4:  timer_channel = TIMER_TIM2; tim_ch_num = 4; icu_port = PORT_A; icu_pin = PIN3; af_val = 1; break; // PA3/AF1
        case PWM_TIM3_CH1:  timer_channel = TIMER_TIM3; tim_ch_num = 1; icu_port = PORT_A; icu_pin = PIN6; af_val = 2; break; // PA6/AF2
        case PWM_TIM3_CH2:  timer_channel = TIMER_TIM3; tim_ch_num = 2; icu_port = PORT_A; icu_pin = PIN7; af_val = 2; break; // PA7/AF2
        case PWM_TIM3_CH3:  timer_channel = TIMER_TIM3; tim_ch_num = 3; icu_port = PORT_B; icu_pin = PIN0; af_val = 2; break; // PB0/AF2
        case PWM_TIM3_CH4:  timer_channel = TIMER_TIM3; tim_ch_num = 4; icu_port = PORT_B; icu_pin = PIN1; af_val = 2; break; // PB1/AF2
        case PWM_TIM4_CH1:  timer_channel = TIMER_TIM4; tim_ch_num = 1; icu_port = PORT_D; icu_pin = PIN12; af_val = 2; break; // PD12/AF2
        case PWM_TIM4_CH2:  timer_channel = TIMER_TIM4; tim_ch_num = 2; icu_port = PORT_D; icu_pin = PIN13; af_val = 2; break; // PD13/AF2
        case PWM_TIM4_CH3:  timer_channel = TIMER_TIM4; tim_ch_num = 3; icu_port = PORT_D; icu_pin = PIN14; af_val = 2; break; // PD14/AF2
        case PWM_TIM4_CH4:  timer_channel = TIMER_TIM4; tim_ch_num = 4; icu_port = PORT_D; icu_pin = PIN15; af_val = 2; break; // PD15/AF2
        case PWM_TIM5_CH1:  timer_channel = TIMER_TIM5; tim_ch_num = 1; icu_port = PORT_A; icu_pin = PIN0; af_val = 2; break; // PA0/AF2
        case PWM_TIM5_CH2:  timer_channel = TIMER_TIM5; tim_ch_num = 2; icu_port = PORT_A; icu_pin = PIN1; af_val = 2; break; // PA1/AF2
        case PWM_TIM5_CH3:  timer_channel = TIMER_TIM5; tim_ch_num = 3; icu_port = PORT_A; icu_pin = PIN2; af_val = 2; break; // PA2/AF2
        case PWM_TIM5_CH4:  timer_channel = TIMER_TIM5; tim_ch_num = 4; icu_port = PORT_A; icu_pin = PIN3; af_val = 2; break; // PA3/AF2
        case PWM_TIM9_CH1:  timer_channel = TIMER_TIM9; tim_ch_num = 1; icu_port = PORT_A; icu_pin = PIN2; af_val = 3; break; // PA2/AF3
        case PWM_TIM9_CH2:  timer_channel = TIMER_TIM9; tim_ch_num = 2; icu_port = PORT_A; icu_pin = PIN3; af_val = 3; break; // PA3/AF3
        case PWM_TIM10_CH1: timer_channel = TIMER_TIM10; tim_ch_num = 1; icu_port = PORT_B; icu_pin = PIN8; af_val = 3; break; // PB8/AF3
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; icu_port = PORT_B; icu_pin = PIN9; af_val = 3; break; // PB9/AF3
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable Timer clock first
    WDT_Reset();
    if (timer_channel == TIMER_TIM1 || timer_channel == TIMER_TIM9 || timer_channel == TIMER_TIM10 || timer_channel == TIMER_TIM11) {
        *RCC_APB2ENR_REG |= (1UL << (timer_channel == TIMER_TIM1 ? 0 : (timer_channel == TIMER_TIM9 ? 16 : (timer_channel == TIMER_TIM10 ? 17 : 18)))); // Inferred
    } else {
        *RCC_APB1ENR_REG |= (1UL << (timer_channel == TIMER_TIM2 ? 0 : (timer_channel == TIMER_TIM3 ? 1 : (timer_channel == TIMER_TIM4 ? 2 : 3)))); // Inferred
    }

    // Configure GPIO pin for Alternate Function Input
    volatile tlong* moder_gpio = _get_gpio_reg_ptr(icu_port, GPIO_MODER_OFFSET);
    volatile tlong* pupdr_gpio = _get_gpio_reg_ptr(icu_port, GPIO_PUPDR_OFFSET);
    volatile tlong* afr_gpio = (icu_pin < 8) ? _get_gpio_reg_ptr(icu_port, GPIO_AFRL_OFFSET) : _get_gpio_reg_ptr(icu_port, GPIO_AFRH_OFFSET);

    *moder_gpio |= (0b10UL << (icu_pin * 2)); // Alternate function mode
    *pupdr_gpio &= ~(0b11UL << (icu_pin * 2)); // No pull-up/pull-down

    if (icu_pin < 8) {
        *afr_gpio &= ~((0b1111UL) << (icu_pin * 4));
        *afr_gpio |= ((tlong)af_val << (icu_pin * 4));
    } else {
        *afr_gpio &= ~((0b1111UL) << ((icu_pin - 8) * 4));
        *afr_gpio |= ((tlong)af_val << ((icu_pin - 8) * 4));
    }

    // Disable timer before configuration
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0); // CEN bit

    // Configure Prescaler (PSC)
    *(TIM_REG(tim_base, TIM_PSC_OFFSET)) = icu_prescaller;

    // Configure Capture/Compare Mode Register (CCMRx) for Input Capture
    volatile tlong* ccmr_reg;
    tbyte ccm_shift;

    if (tim_ch_num == 1 || tim_ch_num == 2) {
        ccmr_reg = TIM_REG(tim_base, TIM_CCMR1_OFFSET);
        ccm_shift = (tim_ch_num == 1) ? 0 : 8;
    } else if (tim_ch_num == 3 || tim_ch_num == 4) {
        ccmr_reg = TIM_REG(tim_base, TIM_CCMR2_OFFSET);
        ccm_shift = (tim_ch_num == 3) ? 0 : 8;
    } else {
        return; // Invalid channel number
    }

    // CCxS bits: ICx is mapped on TIx input (01)
    *ccmr_reg &= ~(0b11UL << ccm_shift); // Clear CCxS bits
    *ccmr_reg |= (0b01UL << ccm_shift);  // Set to Input Capture direct mapping (01)

    // Configure Capture/Compare Enable Register (CCER) for polarity
    tbyte ccer_shift = (tim_ch_num - 1) * 4;
    *(TIM_REG(tim_base, TIM_CCER_OFFSET)) &= ~((0b11UL) << (ccer_shift + 1)); // Clear CCxP, CCxNP bits

    if (icu_edge == ICU_EDGE_RISING) {
        *(TIM_REG(tim_base, TIM_CCER_OFFSET)) &= ~(1UL << (ccer_shift + 1)); // CCxP = 0 (rising edge)
    } else if (icu_edge == ICU_EDGE_FALLING) {
        *(TIM_REG(tim_base, TIM_CCER_OFFSET)) |= (1UL << (ccer_shift + 1)); // CCxP = 1 (falling edge)
    } else if (icu_edge == ICU_EDGE_BOTH) {
        *(TIM_REG(tim_base, TIM_CCER_OFFSET)) |= (1UL << (ccer_shift + 1)) | (1UL << (ccer_shift + 3)); // CCxP = 1, CCxNP = 1 (both edges)
    }

    // Enable capture for selected channel (CCxE bit)
    *(TIM_REG(tim_base, TIM_CCER_OFFSET)) |= (1UL << ccer_shift);
}

/**
 * @brief Enables the Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to enable.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;
    // Map ICU channel to specific Timer and Channel number (similar to ICU_init)
    switch (icu_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; break;
        // ... (rest of the mapping similar to ICU_init)
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; break;
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable timer counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) |= (1UL << 0);
    // Enable Capture/Compare Interrupt (CCxIE bit in DIER)
    *(TIM_REG(tim_base, TIM_DIER_OFFSET)) |= (1UL << tim_ch_num);
}

/**
 * @brief Disables the Input Capture Unit (ICU) channel.
 * @param icu_channel The ICU channel to disable.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;
    // Map ICU channel to specific Timer and Channel number (similar to ICU_init)
    switch (icu_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; break;
        // ... (rest of the mapping similar to ICU_init)
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; break;
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Disable timer counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0);
    // Disable Capture/Compare Interrupt (CCxIE bit in DIER)
    *(TIM_REG(tim_base, TIM_DIER_OFFSET)) &= ~(1UL << tim_ch_num);
}

/**
 * @brief Updates frequency for an ICU channel.
 *        This function is a placeholder; frequency calculation typically happens in ISR.
 * @param icu_channel The ICU channel to update.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel) {
    WDT_Reset();
    // This function doesn't directly interact with registers to 'update' frequency.
    // It's a placeholder. Frequency would be calculated in an ISR.
    (void)icu_channel; // Suppress unused warning.
}

/**
 * @brief Gets the frequency measured by an ICU channel.
 * @param icu_channel The ICU channel to get frequency from.
 * @return The measured frequency.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset();
    // This requires specific implementation logic, typically involving timer values captured
    // between edges. The register_json only provides register addresses, not a direct frequency
    // register. This will return a dummy value and indicate the actual implementation is complex.
    (void)icu_channel; // Suppress unused warning.
    return 0; // Placeholder for actual frequency value
}

/**
 * @brief Sets the buffer for remote control keys.
 * @param number_of_keys Number of keys to store.
 * @param key_digits_length Length of digits for each key.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length) {
    WDT_Reset();
    // This is application-specific logic, not direct register manipulation.
    // Placeholder.
    (void)number_of_keys;
    (void)key_digits_length;
}

/**
 * @brief Sets remote control key digits.
 * @param key_num Key number.
 * @param key_array_cell Array cell for the key.
 * @param key_cell_value Value of the key cell.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value) {
    WDT_Reset();
    // This is application-specific logic, not direct register manipulation.
    // Placeholder.
    (void)key_num;
    (void)key_array_cell;
    (void)key_cell_value;
}

/**
 * @brief Updates remote control signal parameters.
 * @param icu_channel The ICU channel.
 * @param strt_bit_us_value Start bit duration in microseconds.
 * @param one_bit_us_value One bit duration in microseconds.
 * @param zero_bit_us_value Zero bit duration in microseconds.
 * @param stop_bit_us_value Stop bit duration in microseconds.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value) {
    WDT_Reset();
    // This is application-specific logic, not direct register manipulation.
    // These parameters would be used by a higher-level module to decode IR signals.
    // Placeholder.
    (void)icu_channel;
    (void)strt_bit_us_value;
    (void)one_bit_us_value;
    (void)zero_bit_us_value;
    (void)stop_bit_us_value;
}

/**
 * @brief Gets the pressed remote control key.
 * @param icu_channel The ICU channel.
 * @return The pressed key.
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel) {
    WDT_Reset();
    // This is application-specific logic that would process captured signal timings.
    // Placeholder.
    (void)icu_channel;
    return 0; // Placeholder for actual key value
}

/**
 * @brief Sets a callback function for ICU events.
 * @param callback Pointer to the callback function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset();
    // In a real system, this would store the callback pointer
    // and call it from the relevant Timer Input Capture interrupt handler.
    // For now, it's a placeholder.
    (void)callback;
}

/**
 * @brief Clears flags for an ICU channel.
 * @param icu_channel The ICU channel to clear flags for.
 */
void ICU_ClearFlag(t_icu_channel icu_channel) {
    WDT_Reset();

    t_timer_channel timer_channel;
    tbyte tim_ch_num;
    // Map ICU channel to specific Timer and Channel number (similar to ICU_init)
    switch (icu_channel) {
        case PWM_TIM1_CH1:  timer_channel = TIMER_TIM1; tim_ch_num = 1; break;
        // ... (rest of the mapping similar to ICU_init)
        case PWM_TIM11_CH1: timer_channel = TIMER_TIM11; tim_ch_num = 1; break;
        default: return;
    }

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Clear Capture/Compare Interrupt Flag (CCxIF in SR)
    *(TIM_REG(tim_base, TIM_SR_OFFSET)) &= ~(1UL << tim_ch_num);
}

// =============================================================================
// Timer APIs
// =============================================================================

/**
 * @brief Initializes a general-purpose timer channel.
 * @param timer_channel The timer channel to initialize.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset();

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable Timer clock first
    WDT_Reset();
    if (timer_channel == TIMER_TIM1 || timer_channel == TIMER_TIM9 || timer_channel == TIMER_TIM10 || timer_channel == TIMER_TIM11) {
        *RCC_APB2ENR_REG |= (1UL << (timer_channel == TIMER_TIM1 ? 0 : (timer_channel == TIMER_TIM9 ? 16 : (timer_channel == TIMER_TIM10 ? 17 : 18)))); // Inferred
    } else {
        *RCC_APB1ENR_REG |= (1UL << (timer_channel == TIMER_TIM2 ? 0 : (timer_channel == TIMER_TIM3 ? 1 : (timer_channel == TIMER_TIM4 ? 2 : 3)))); // Inferred
    }

    // Disable timer before configuration
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0); // CEN bit

    // Set default configuration: Up-counting, no prescaler, auto-reload max
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) = 0; // Reset CR1
    *(TIM_REG(tim_base, TIM_PSC_OFFSET)) = 0; // No prescaler
    *(TIM_REG(tim_base, TIM_ARR_OFFSET)) = 0xFFFFFFFFUL; // Max auto-reload value
    *(TIM_REG(tim_base, TIM_CNT_OFFSET)) = 0; // Reset counter
}

/**
 * @brief Sets a timer to trigger after a specified time in microseconds.
 * @param timer_channel The timer channel.
 * @param time Time in microseconds.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset();

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    tlong tim_clk_freq;
    if (timer_channel == TIMER_TIM1 || timer_channel == TIMER_TIM9 || timer_channel == TIMER_TIM10 || timer_channel == TIMER_TIM11) {
        tim_clk_freq = 84000000UL; // APB2 Timers
    } else {
        tim_clk_freq = 42000000UL; // APB1 Timers
    }

    // Calculate PSC and ARR for the given time.
    // We want (ARR + 1) * (PSC + 1) = Time_us * Clock_Freq_MHz
    // For 1 us resolution, PSC should be (Clock_Freq_MHz - 1).
    tlong prescaler_val = (tim_clk_freq / 1000000UL) - 1; // Prescaler to get 1us tick
    tlong auto_reload_val = time - 1;

    *(TIM_REG(tim_base, TIM_PSC_OFFSET)) = prescaler_val;
    *(TIM_REG(tim_base, TIM_ARR_OFFSET)) = auto_reload_val;
    *(TIM_REG(tim_base, TIM_CNT_OFFSET)) = 0; // Reset counter
}

/**
 * @brief Sets a timer to trigger after a specified time in milliseconds.
 * @param timer_channel The timer channel.
 * @param time Time in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset();
    TIMER_Set_us(timer_channel, time * 1000UL);
}

/**
 * @brief Sets a timer to trigger after a specified time in seconds.
 * @param timer_channel The timer channel.
 * @param time Time in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    TIMER_Set_Time_ms(timer_channel, (tword)time * 1000UL);
}

/**
 * @brief Sets a timer to trigger after a specified time in minutes.
 * @param timer_channel The timer channel.
 * @param time Time in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    TIMER_Set_Time_sec(timer_channel, time * 60UL);
}

/**
 * @brief Sets a timer to trigger after a specified time in hours.
 * @param timer_channel The timer channel.
 * @param time Time in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset();
    TIMER_Set_Time_min(timer_channel, time * 60UL);
}

/**
 * @brief Enables a timer channel.
 * @param timer_channel The timer channel to enable.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset();

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Enable Update Interrupt (UIE bit in DIER)
    *(TIM_REG(tim_base, TIM_DIER_OFFSET)) |= (1UL << 0);
    // Enable counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) |= (1UL << 0);
}

/**
 * @brief Disables a timer channel.
 * @param timer_channel The timer channel to disable.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset();

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Disable counter (CEN bit in CR1)
    *(TIM_REG(tim_base, TIM_CR1_OFFSET)) &= ~(1UL << 0);
    // Disable Update Interrupt (UIE bit in DIER)
    *(TIM_REG(tim_base, TIM_DIER_OFFSET)) &= ~(1UL << 0);
}

/**
 * @brief Clears the pending flag for a timer channel.
 * @param timer_channel The timer channel to clear the flag for.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel) {
    WDT_Reset();

    volatile tlong* tim_base = _get_timer_base(timer_channel);
    if (tim_base == NULL) return;

    // Clear Update Interrupt Flag (UIF bit in SR)
    *(TIM_REG(tim_base, TIM_SR_OFFSET)) &= ~(1UL << 0);
}

// =============================================================================
// ADC APIs
// =============================================================================

/**
 * @brief Initializes the ADC.
 * @param adc_channel The ADC channel to initialize.
 * @param adc_mode ADC conversion mode (single, continuous, scan).
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset();

    // Enable ADC1 clock (if not already enabled by ADC_Enable)
    *RCC_APB2ENR_REG |= (1UL << 8); // ADC1EN - Inferred

    // Enable GPIO clock and configure ADC pins as Analog mode
    t_port adc_port;
    t_pin adc_pin;

    // Example mapping for a few common ADC channels
    switch (adc_channel) {
        case ADC_CHANNEL_0: adc_port = PORT_A; adc_pin = PIN0; break; // PA0
        case ADC_CHANNEL_1: adc_port = PORT_A; adc_pin = PIN1; break; // PA1
        case ADC_CHANNEL_2: adc_port = PORT_A; adc_pin = PIN2; break; // PA2
        case ADC_CHANNEL_3: adc_port = PORT_A; adc_pin = PIN3; break; // PA3
        case ADC_CHANNEL_4: adc_port = PORT_A; adc_pin = PIN4; break; // PA4
        case ADC_CHANNEL_5: adc_port = PORT_A; adc_pin = PIN5; break; // PA5
        case ADC_CHANNEL_6: adc_port = PORT_A; adc_pin = PIN6; break; // PA6
        case ADC_CHANNEL_7: adc_port = PORT_A; adc_pin = PIN7; break; // PA7
        case ADC_CHANNEL_8: adc_port = PORT_B; adc_pin = PIN0; break; // PB0
        case ADC_CHANNEL_9: adc_port = PORT_B; adc_pin = PIN1; break; // PB1
        case ADC_CHANNEL_10: adc_port = PORT_C; adc_pin = PIN0; break; // PC0
        case ADC_CHANNEL_11: adc_port = PORT_C; adc_pin = PIN1; break; // PC1
        case ADC_CHANNEL_12: adc_port = PORT_C; adc_pin = PIN2; break; // PC2
        case ADC_CHANNEL_13: adc_port = PORT_C; adc_pin = PIN3; break; // PC3
        case ADC_CHANNEL_14: adc_port = PORT_C; adc_pin = PIN4; break; // PC4
        case ADC_CHANNEL_15: adc_port = PORT_C; adc_pin = PIN5; break; // PC5
        // Internal channels don't have GPIO. These are configured via ADC_CR2/SMPR/SQR.
        case ADC_CHANNEL_VREFINT:
        case ADC_CHANNEL_TEMPSENSOR:
        case ADC_CHANNEL_VBAT:
        default: return;
    }

    // Configure GPIO pin for Analog mode
    if (adc_channel < ADC_CHANNEL_VREFINT) { // Only configure GPIO for external channels
        volatile tlong* moder_gpio = _get_gpio_reg_ptr(adc_port, GPIO_MODER_OFFSET);
        volatile tlong* pupdr_gpio = _get_gpio_reg_ptr(adc_port, GPIO_PUPDR_OFFSET);

        *moder_gpio |= (0b11UL << (adc_pin * 2)); // Analog mode (11)
        *pupdr_gpio &= ~(0b11UL << (adc_pin * 2)); // No pull-up/pull-down
    }

    // Disable ADC before configuration
    *ADC_CR2_REG &= ~(1UL << 0); // Clear ADON bit

    // Configure CR1: Resolution (12-bit default), Scan mode, Discontinuous mode (optional)
    *ADC_CR1_REG &= ~(0b11UL << 24); // Clear RES bits (12-bit resolution by default)
    *ADC_CR1_REG &= ~(1UL << 8); // Clear SCAN bit (Scan mode disabled by default)
    if (adc_mode == ADC_MODE_SCAN_CONVERSION) {
        *ADC_CR1_REG |= (1UL << 8); // Enable SCAN mode
    }

    // Configure CR2: Continuous conversion, Data alignment
    *ADC_CR2_REG &= ~(1UL << 1); // Clear CONT bit (Single conversion mode by default)
    *ADC_CR2_REG &= ~(1UL << 11); // Clear ALIGN bit (Right alignment by default)
    if (adc_mode == ADC_MODE_CONTINUOUS_CONVERSION) {
        *ADC_CR2_REG |= (1UL << 1); // Enable continuous conversion
    }

    // Configure Sample Time for the channel (SMPRx)
    // Example: 3 cycles sample time (000) for common channels.
    // SMPR1 for channels 10-18, SMPR2 for channels 0-9
    tbyte smp_bits = 0; // 3 cycles
    if (adc_channel >= ADC_CHANNEL_10 && adc_channel <= ADC_CHANNEL_15) {
        *ADC_SMPR1_REG &= ~(0b111UL << ((adc_channel - 10) * 3));
        *ADC_SMPR1_REG |= ((tlong)smp_bits << ((adc_channel - 10) * 3));
    } else if (adc_channel >= ADC_CHANNEL_0 && adc_channel <= ADC_CHANNEL_9) {
        *ADC_SMPR2_REG &= ~(0b111UL << (adc_channel * 3));
        *ADC_SMPR2_REG |= ((tlong)smp_bits << (adc_channel * 3));
    } else if (adc_channel == ADC_CHANNEL_VREFINT) {
        *ADC_SMPR1_REG &= ~(0b111UL << (17 * 3)); // For channel 17 (Vrefint)
        *ADC_SMPR1_REG |= ((tlong)smp_bits << (17 * 3));
        *ADC_CR2_REG |= (1UL << 23); // TSVREFE (Temperature sensor and Vrefint enable)
    } else if (adc_channel == ADC_CHANNEL_TEMPSENSOR) {
        *ADC_SMPR1_REG &= ~(0b111UL << (18 * 3)); // For channel 18 (Temp sensor)
        *ADC_SMPR1_REG |= ((tlong)smp_bits << (18 * 3));
        *ADC_CR2_REG |= (1UL << 23); // TSVREFE
    } else if (adc_channel == ADC_CHANNEL_VBAT) {
        *ADC_SMPR1_REG &= ~(0b111UL << (16 * 3)); // For channel 16 (VBAT)
        *ADC_SMPR1_REG |= ((tlong)smp_bits << (16 * 3));
        *ADC_CCR_REG |= (1UL << 22); // VBATE (VBAT enable in common control register)
    }


    // Configure Regular Sequence (SQRx): Set length and channel order
    // For single channel conversion, sequence length is 1 (L[3:0] in SQR1 = 0000)
    // Add the channel to the first position of the sequence (SQ1[4:0] in SQR3)
    *ADC_SQR1_REG &= ~(0b1111UL << 20); // Clear L bits (sequence length - 1)
    *ADC_SQR3_REG = (tlong)adc_channel; // Set first channel in sequence
}

/**
 * @brief Enables the ADC.
 */
void ADC_Enable(void) {
    WDT_Reset();
    // Enable peripheral clock first (RCC)
    *RCC_APB2ENR_REG |= (1UL << 8); // ADC1EN - Inferred

    // Enable ADC (ADON bit in CR2)
    *ADC_CR2_REG |= (1UL << 0);
    // Add a small delay for ADC to power up and stabilize (typ. a few us)
    volatile int i = 0;
    for(i=0; i<1000; i++) { __asm("nop"); } // Short delay
}

/**
 * @brief Disables the ADC.
 */
void ADC_Disable(void) {
    WDT_Reset();
    // Disable ADC (ADON bit in CR2)
    *ADC_CR2_REG &= ~(1UL << 0);
    // Disable peripheral clock (RCC)
    *RCC_APB2ENR_REG &= ~(1UL << 8); // ADC1EN - Inferred
}

/**
 * @brief Updates the ADC status.
 *        This function is a placeholder for any periodic status updates.
 */
void ADC_Update(void) {
    WDT_Reset();
    // This function can be used to check status flags like EOC, OVR, etc.
    // For now, it's a placeholder as per API.json.
    volatile tlong status = *ADC_SR_REG;
    (void)status; // Suppress unused variable warning.
}

/**
 * @brief Gets the ADC conversion result.
 * @return The 12-bit digital value from the ADC.
 */
tword ADC_Get(void) {
    WDT_Reset();
    // Start ADC conversion (SWSTART bit in CR2)
    *ADC_CR2_REG |= (1UL << 30);

    // Wait for End of Conversion (EOC) flag in SR
    while (!(*ADC_SR_REG & (1UL << 1))) {
        WDT_Reset();
    }
    // Clear EOC flag by reading SR then reading DR
    // (Or by writing 0 to the EOC bit for some ADCs, but F4 requires read sequence)
    // The flag is cleared automatically when DR is read if CONT=0 (single conversion)
    
    return (tword)(*ADC_DR_REG & 0xFFFFUL); // Read data register
}

/**
 * @brief Clears ADC flags.
 */
void ADC_ClearFlag(void) {
    WDT_Reset();
    // Clear End of Conversion (EOC) flag by writing 0 to it (or reading DR in single mode)
    *ADC_SR_REG &= ~(1UL << 1); // Clear EOC
    // Clear Overrun (OVR) flag by writing 0 to it
    *ADC_SR_REG &= ~(1UL << 5); // Clear OVR
}

// =============================================================================
// Internal EEPROM APIs (Mapped to Flash Memory)
// =============================================================================

// Base address for data storage in Flash (e.g., last sector of Flash)
// For STM32F401RC (512KB Flash), Sector 5 is 16KB at 0x08020000.
// Sector 6 (64KB) at 0x08040000. Sector 7 (128KB) at 0x08060000.
// Let's use a small dedicated area, perhaps at the end of Sector 7.
#define FLASH_USER_DATA_START_ADDR  0x08060000UL // Start of Sector 7 (128KB)
#define FLASH_USER_DATA_END_ADDR    (FLASH_USER_DATA_START_ADDR + 0x20000UL) // Up to 0x0807FFFF

/**
 * @brief Initializes the Internal EEPROM (Flash for data storage).
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset();
    // For STM32 Flash, initialization typically involves unlocking Flash controller.
    // Check BSY flag before any operation
    while ((*FLASH_SR_REG & (1UL << 16))) { // Wait for BSY flag to clear (BSY is bit 16)
        WDT_Reset();
    }
    // Unlock Flash (write sequence to FLASH_KEYR)
    if (!(*FLASH_CR_REG & (1UL << 7))) { // If LOCK bit (bit 7) is set
        *FLASH_KEYR_REG = 0x45670123UL;
        *FLASH_KEYR_REG = 0xCDEF89ABUL;
    }
    // Now Flash is unlocked. Further operations (erase/program) can proceed.
}

/**
 * @brief Sets a byte in Internal EEPROM (Flash).
 *        Note: This involves Flash erase/program cycles, which are slow and wear out Flash.
 *        This is a simplified abstraction and doesn't handle full sector erase/rewrite logic.
 * @param address The offset address within the defined user data area.
 * @param data The byte data to write.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset();
    tlong target_addr = FLASH_USER_DATA_START_ADDR + address;

    if (target_addr >= FLASH_USER_DATA_END_ADDR) return; // Address out of bounds

    // STM32 Flash programming requires erase before write if value is not 0xFF
    // A full implementation would involve reading the entire sector, modifying the byte,
    // erasing the sector, then reprogramming the entire sector.
    // For a minimal implementation, we assume we can write directly IF the location is erased (0xFF)
    // or if only changing 1->0. If 0->1, erase is mandatory.
    // For simplicity, we implement a basic half-word programming.

    // Check BSY flag
    while ((*FLASH_SR_REG & (1UL << 16))) {
        WDT_Reset();
    }

    // Set PG bit (Program) in FLASH_CR (bit 0)
    *FLASH_CR_REG |= (1UL << 0);
    // Set PSIZE to Byte programming (00 for x8, 01 for x16, 10 for x32, 11 for x64)
    *FLASH_CR_REG &= ~(0b11UL << 8); // Clear PSIZE bits
    // Use x8 programming if available, or x32/x16 as smallest unit.
    // F401 supports byte, half-word, word, double word. Let's use half-word (x16).
    *FLASH_CR_REG |= (0b01UL << 8); // PSIZE = 01 (x16)

    // Write the data to the target address.
    // If it's a byte, we're writing to a half-word address. This means the other byte will be unknown.
    // For true byte-wise "EEPROM" behavior on Flash, you need a Flash emulation layer.
    *((volatile tbyte*)target_addr) = data;

    // Wait for EOP (End of Operation) flag or BSY flag to clear
    while (!(*FLASH_SR_REG & (1UL << 0)) && (*FLASH_SR_REG & (1UL << 16))) {
        WDT_Reset();
    }

    // Clear EOP flag by writing 1 to it
    *FLASH_SR_REG = (1UL << 0);

    // Clear PG bit
    *FLASH_CR_REG &= ~(1UL << 0);
}

/**
 * @brief Gets a byte from Internal EEPROM (Flash).
 * @param address The offset address within the defined user data area.
 * @return The byte data read.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset();
    tlong target_addr = FLASH_USER_DATA_START_ADDR + address;

    if (target_addr >= FLASH_USER_DATA_END_ADDR) return 0; // Address out of bounds

    // Reading from Flash is a direct memory read.
    return *((volatile tbyte*)target_addr);
}

// =============================================================================
// TT APIs (Time-Triggered OS)
// =============================================================================

static void (*_TT_Callback)(void) = NULL;
// Use TIM2 for the TT_ISR as it is a general purpose 32-bit timer on APB1.
// APB1 timer clock is typically 42MHz.

/**
 * @brief Initializes the Time-Triggered (TT) OS tick.
 * @param tick_time_ms The desired tick time in milliseconds.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset();
    // Configure a timer (e.g., TIM2) to generate an interrupt at tick_time_ms intervals.
    TIMER_Init(TIMER_TIM2);
    TIMER_Set_Time_ms(TIMER_TIM2, tick_time_ms);
    // Note: TIM2 interrupt must be enabled in NVIC for TT_ISR to be called.
    // This is typically done outside MCAL or abstracted via a separate NVIC driver.
    // For now, assume NVIC setup is handled.
    // Enable TIM2 clock in RCC (done in TIMER_Init).
}

/**
 * @brief Starts the Time-Triggered (TT) OS.
 */
void TT_Start(void) {
    WDT_Reset();
    TIMER_Enable(TIMER_TIM2); // Enable TIM2 counter and update interrupt
}

/**
 * @brief Dispatches tasks in the Time-Triggered OS.
 *        This function is application-specific and not directly related to registers.
 */
void TT_Dispatch_task(void) {
    WDT_Reset();
    // This function would typically iterate through a task list and execute tasks
    // whose time has come. It's part of the OS scheduler, not direct hardware.
    // Placeholder.
}

/**
 * @brief Time-Triggered OS Interrupt Service Routine.
 *        This function should be called from the timer's ISR.
 */
void TT_ISR(void) {
    WDT_Reset(); // Keep WDT happy during ISR execution
    // Check and clear the timer's update interrupt flag
    if (*(TIM_REG(_get_timer_base(TIMER_TIM2), TIM_SR_OFFSET)) & (1UL << 0)) {
        TIMER_ClearFlag(TIMER_TIM2);
        if (_TT_Callback != NULL) {
            _TT_Callback(); // Call the registered callback (e.g., to increment OS tick)
        }
    }
}

/**
 * @brief Adds a task to the Time-Triggered OS scheduler.
 * @param task Pointer to the task function.
 * @param period Task execution period.
 * @param delay Initial delay before first execution.
 * @return Index of the added task.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset();
    // This is OS scheduler logic, not direct register manipulation.
    // Placeholder.
    (void)task;
    (void)period;
    (void)delay;
    return 0xFF; // Placeholder
}

/**
 * @brief Deletes a task from the Time-Triggered OS scheduler.
 * @param task_index Index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset();
    // This is OS scheduler logic, not direct register manipulation.
    // Placeholder.
    (void)task_index;
}