/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) source file for STM32F401RC.
 *
 * This file contains the implementations for MCAL functions,
 * providing an abstract interface to the microcontroller's peripherals.
 *
 * @date 2023-10-27
 * @author Your Name/Generator
 */

// global_includes from Rules.json
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
// MCU Specific Includes (if any, e.g., for common bit definitions or CMSIS)

#include "MCAL.h"

// --- Private Helper Functions (if any) ---

/**
 * @brief Gets the base address for a given GPIO port.
 * @param Port The GPIO port enum.
 * @return uint32_t The base address of the GPIO port, or 0 if invalid.
 */
static uint32_t MCAL_GPIO_GetPortBaseAddress(MCAL_GPIO_Port_t Port)
{
    switch (Port)
    {
        case MCAL_GPIO_PORTA: return GPIOA_BASE;
        case MCAL_GPIO_PORTB: return GPIOB_BASE;
        case MCAL_GPIO_PORTC: return GPIOC_BASE;
        case MCAL_GPIO_PORTD: return GPIOD_BASE;
        case MCAL_GPIO_PORTE: return GPIOE_BASE;
        case MCAL_GPIO_PORTH: return GPIOH_BASE;
        default: return 0; // Invalid port
    }
}

/**
 * @brief Gets the base address for a given Timer instance.
 * @param Timer The Timer enum.
 * @return uint32_t The base address of the Timer peripheral, or 0 if invalid.
 */
static uint32_t MCAL_TIMER_GetBaseAddress(MCAL_Timer_t Timer)
{
    switch (Timer)
    {
        case MCAL_TIMER_1:  return TIM1_BASE;
        case MCAL_TIMER_2:  return TIM2_BASE;
        case MCAL_TIMER_3:  return TIM3_BASE;
        case MCAL_TIMER_4:  return TIM4_BASE;
        case MCAL_TIMER_5:  return TIM5_BASE;
        case MCAL_TIMER_9:  return TIM9_BASE;
        case MCAL_TIMER_10: return TIM10_BASE;
        case MCAL_TIMER_11: return TIM11_BASE;
        default: return 0; // Invalid timer
    }
}

/**
 * @brief Gets the base address for a given UART instance.
 * @param Uart The UART enum.
 * @return uint32_t The base address of the UART peripheral, or 0 if invalid.
 */
static uint32_t MCAL_UART_GetBaseAddress(MCAL_UART_t Uart)
{
    switch (Uart)
    {
        case MCAL_UART_1: return USART1_BASE;
        case MCAL_UART_2: return USART2_BASE;
        case MCAL_UART_6: return USART6_BASE;
        default: return 0; // Invalid UART
    }
}

/**
 * @brief Gets the base address for a given I2C instance.
 * @param I2c The I2C enum.
 * @return uint32_t The base address of the I2C peripheral, or 0 if invalid.
 */
static uint32_t MCAL_I2C_GetBaseAddress(MCAL_I2C_t I2c)
{
    switch (I2c)
    {
        case MCAL_I2C_1: return I2C1_BASE;
        case MCAL_I2C_2: return I2C2_BASE;
        case MCAL_I2C_3: return I2C3_BASE;
        default: return 0; // Invalid I2C
    }
}

/**
 * @brief Gets the base address for a given SPI instance.
 * @param Spi The SPI enum.
 * @return uint32_t The base address of the SPI peripheral, or 0 if invalid.
 */
static uint32_t MCAL_SPI_GetBaseAddress(MCAL_SPI_t Spi)
{
    switch (Spi)
    {
        case MCAL_SPI_1: return SPI1_BASE;
        case MCAL_SPI_2: return SPI2_BASE;
        case MCAL_SPI_3: return SPI3_BASE;
        default: return 0; // Invalid SPI
    }
}

// --- API Function Implementations ---

MCAL_Status_t MCAL_MCU_Config_Init(void)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    // Placeholder for common MCU initialization steps
    // This typically includes:
    // 1. Enabling power (PWR) clock (already covered by RCC_EnableClock if needed)
    // 2. Setting Flash Latency (FLASH_ACR)
    // 3. Configuring internal/external clocks, PLL (RCC_CR, RCC_PLLCFGR, RCC_CFGR)

    // Example: Set Flash latency (adjust based on actual clock configuration)
    // For STM32F401RC, 84MHz requires 2 wait states (ACR[2:0] Latency = 010b)
    // Max 42MHz - 1WS, Max 84MHz - 2WS
    // Assuming a system clock up to 84MHz, set 2 wait states.
    // Bits 2:0 - LATENCY[2:0]
    // Clear old latency bits and set new
    *FLASH_ACR_REG = (*FLASH_ACR_REG & ~((uint32_t)0x07)) | ((uint32_t)0x02); // 2 wait states

    // Enable instruction cache (ICEN) and prefetch buffer (PRFTEN)
    *FLASH_ACR_REG |= (1U << 8) | (1U << 9); // Set ICEN and PRFTEN bits

    // Example: Configure RCC (simplified to enable HSI and select it, or external crystal if available)
    // The specifics of full clock tree configuration are complex and would require
    // a more detailed `MCU_Config_Init_t` structure or predefined settings.
    // For demonstration: Ensure HSI is enabled and selected as system clock source.
    // Set HSI ON bit in RCC_CR (HSION)
    *RCC_CR_REG |= (1U << 0);
    // Wait for HSI Ready flag (HSIRDY)
    while (!(*RCC_CR_REG & (1U << 1)))
    {
        /* Wait for HSI to be ready */
    }
    // Select HSI as system clock switch (SWS bits in RCC_CFGR = 00)
    *RCC_CFGR_REG &= ~((uint32_t)0x03); // Clear SW bits to select HSI

    // This is a minimal configuration. A full MCU init would involve
    // detailed PLL, prescaler, bus clock configurations, and potentially
    // enabling specific low power modes if desired.

    return MCAL_OK;
}

MCAL_Status_t MCAL_GPIO_Init(const MCAL_GPIO_Init_t *p_GPIO_Init)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (p_GPIO_Init == NULL)
    {
        return MCAL_INVALID_PARAM;
    }

    uint32_t gpio_base = MCAL_GPIO_GetPortBaseAddress(p_GPIO_Init->Port);
    if (gpio_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    // Enable clock for the GPIO port first
    MCAL_RCC_Peripheral_t rcc_periph = 0;
    MCAL_RCC_Bus_t rcc_bus = MCAL_RCC_AHB1_BUS; // All GPIOs are on AHB1 for STM32F401RC
    switch (p_GPIO_Init->Port) {
        case MCAL_GPIO_PORTA: rcc_periph = MCAL_RCC_AHB1_GPIOA; break;
        case MCAL_GPIO_PORTB: rcc_periph = MCAL_RCC_AHB1_GPIOB; break;
        case MCAL_GPIO_PORTC: rcc_periph = MCAL_RCC_AHB1_GPIOC; break;
        case MCAL_GPIO_PORTD: rcc_periph = MCAL_RCC_AHB1_GPIOD; break;
        case MCAL_GPIO_PORTE: rcc_periph = MCAL_RCC_AHB1_GPIOE; break;
        case MCAL_GPIO_PORTH: rcc_periph = MCAL_RCC_AHB1_GPIOH; break;
        default: return MCAL_INVALID_PARAM;
    }
    MCAL_RCC_EnableClock(rcc_bus, rcc_periph);

    volatile uint32_t *p_moder   = GPIO_REG(gpio_base, GPIO_MODER_OFFSET);
    volatile uint32_t *p_otyper  = GPIO_REG(gpio_base, GPIO_OTYPER_OFFSET);
    volatile uint32_t *p_ospeedr = GPIO_REG(gpio_base, GPIO_OSPEEDR_OFFSET);
    volatile uint32_t *p_pupdr   = GPIO_REG(gpio_base, GPIO_PUPDR_OFFSET);
    volatile uint32_t *p_afrl    = GPIO_REG(gpio_base, GPIO_AFRL_OFFSET);
    volatile uint32_t *p_afrh    = GPIO_REG(gpio_base, GPIO_AFRH_OFFSET);

    uint32_t pin_num;
    for (pin_num = 0; pin_num < 16; pin_num++)
    {
        if ((p_GPIO_Init->Pin >> pin_num) & 0x1U) // Check if this pin is selected
        {
            // Configure Mode (2 bits per pin)
            *p_moder &= ~(0x3U << (pin_num * 2));
            if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_INPUT ||
                p_GPIO_Init->Mode == MCAL_GPIO_MODE_EXTI_IT_RISING ||
                p_GPIO_Init->Mode == MCAL_GPIO_MODE_EXTI_IT_FALLING ||
                p_GPIO_Init->Mode == MCAL_GPIO_MODE_EXTI_IT_RISING_FALLING)
            {
                *p_moder |= (0x0U << (pin_num * 2)); // 00: Input mode
            }
            else if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_OUTPUT_PP ||
                     p_GPIO_Init->Mode == MCAL_GPIO_MODE_OUTPUT_OD)
            {
                *p_moder |= (0x1U << (pin_num * 2)); // 01: General purpose output mode
            }
            else if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_AF_PP ||
                     p_GPIO_Init->Mode == MCAL_GPIO_MODE_AF_OD)
            {
                *p_moder |= (0x2U << (pin_num * 2)); // 10: Alternate function mode
            }
            else if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_ANALOG)
            {
                *p_moder |= (0x3U << (pin_num * 2)); // 11: Analog mode
            }
            else
            {
                return MCAL_INVALID_PARAM; // Unknown mode
            }

            // Configure Output Type (1 bit per pin for push-pull/open-drain)
            if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_OUTPUT_OD ||
                p_GPIO_Init->Mode == MCAL_GPIO_MODE_AF_OD)
            {
                *p_otyper |= (1U << pin_num); // 1: Open-drain
            }
            else
            {
                *p_otyper &= ~(1U << pin_num); // 0: Push-pull
            }

            // Configure Output Speed (2 bits per pin)
            *p_ospeedr &= ~(0x3U << (pin_num * 2));
            *p_ospeedr |= (p_GPIO_Init->Speed << (pin_num * 2));

            // Configure Pull-up/Pull-down (2 bits per pin)
            *p_pupdr &= ~(0x3U << (pin_num * 2));
            *p_pupdr |= (p_GPIO_Init->Pull << (pin_num * 2));

            // Configure Alternate Function (4 bits per pin)
            if (p_GPIO_Init->Mode == MCAL_GPIO_MODE_AF_PP ||
                p_GPIO_Init->Mode == MCAL_GPIO_MODE_AF_OD)
            {
                if (pin_num < 8)
                {
                    *p_afrl &= ~(0xFU << (pin_num * 4));
                    *p_afrl |= (p_GPIO_Init->Alternate << (pin_num * 4));
                }
                else
                {
                    *p_afrh &= ~(0xFU << ((pin_num - 8) * 4));
                    *p_afrh |= (p_GPIO_Init->Alternate << ((pin_num - 8) * 4));
                }
            }
        }
    }

    return MCAL_OK;
}

MCAL_Status_t MCAL_GPIO_WritePin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin, MCAL_GPIO_PinState_t State)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t gpio_base = MCAL_GPIO_GetPortBaseAddress(Port);
    if (gpio_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_bsrr = GPIO_REG(gpio_base, GPIO_BSRR_OFFSET);

    if (State == MCAL_GPIO_HIGH)
    {
        *p_bsrr = (uint32_t)Pin; // Write to the upper 16 bits to set
    }
    else
    {
        *p_bsrr = (uint32_t)Pin << 16; // Write to the lower 16 bits to reset
    }

    return MCAL_OK;
}

MCAL_GPIO_PinState_t MCAL_GPIO_ReadPin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t gpio_base = MCAL_GPIO_GetPortBaseAddress(Port);
    if (gpio_base == 0)
    {
        return (MCAL_GPIO_PinState_t)0xFF; // Indicate error with an invalid state
    }

    volatile uint32_t *p_idr = GPIO_REG(gpio_base, GPIO_IDR_OFFSET);

    if ((*p_idr & (uint32_t)Pin) != (uint32_t)MCAL_GPIO_LOW)
    {
        return MCAL_GPIO_HIGH;
    }
    else
    {
        return MCAL_GPIO_LOW;
    }
}

MCAL_Status_t MCAL_GPIO_TogglePin(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t gpio_base = MCAL_GPIO_GetPortBaseAddress(Port);
    if (gpio_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_odr = GPIO_REG(gpio_base, GPIO_ODR_OFFSET);

    // Read the current output data register and toggle the specified pin
    *p_odr ^= (uint32_t)Pin;

    return MCAL_OK;
}

MCAL_Status_t MCAL_GPIO_SetAlternateFunction(MCAL_GPIO_Port_t Port, MCAL_GPIO_Pin_t Pin, MCAL_GPIO_AlternateFunction_t AlternateFunction)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t gpio_base = MCAL_GPIO_GetPortBaseAddress(Port);
    if (gpio_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_afrl = GPIO_REG(gpio_base, GPIO_AFRL_OFFSET);
    volatile uint32_t *p_afrh = GPIO_REG(gpio_base, GPIO_AFRH_OFFSET);
    volatile uint32_t *p_moder = GPIO_REG(gpio_base, GPIO_MODER_OFFSET);

    uint32_t pin_num;
    for (pin_num = 0; pin_num < 16; pin_num++)
    {
        if ((Pin >> pin_num) & 0x1U) // Check if this pin is selected
        {
            // Set the pin to Alternate Function mode (10)
            *p_moder &= ~(0x3U << (pin_num * 2));
            *p_moder |= (0x2U << (pin_num * 2));

            // Configure Alternate Function (4 bits per pin)
            if (pin_num < 8)
            {
                *p_afrl &= ~(0xFU << (pin_num * 4));
                *p_afrl |= (AlternateFunction << (pin_num * 4));
            }
            else
            {
                *p_afrh &= ~(0xFU << ((pin_num - 8) * 4));
                *p_afrh |= (AlternateFunction << ((pin_num - 8) * 4));
            }
        }
    }

    return MCAL_OK;
}


MCAL_Status_t MCAL_RCC_EnableClock(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    volatile uint32_t *p_enr_reg = NULL;
    uint32_t periph_bit = (uint32_t)Peripheral;

    switch (Bus)
    {
        case MCAL_RCC_AHB1_BUS: p_enr_reg = RCC_AHB1ENR_REG; break;
        case MCAL_RCC_AHB2_BUS: p_enr_reg = RCC_AHB2ENR_REG; break;
        case MCAL_RCC_APB1_BUS: p_enr_reg = RCC_APB1ENR_REG; break;
        case MCAL_RCC_APB2_BUS: p_enr_reg = RCC_APB2ENR_REG; break;
        default: return MCAL_INVALID_PARAM;
    }

    if (p_enr_reg == NULL) return MCAL_ERROR;

    *p_enr_reg |= periph_bit;

    // A small delay or read-back might be needed for some peripherals to ensure clock is stable
    (void)*p_enr_reg; // Read-back to ensure clock is enabled (dummy read)

    return MCAL_OK;
}

MCAL_Status_t MCAL_RCC_DisableClock(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    volatile uint32_t *p_enr_reg = NULL;
    uint32_t periph_bit = (uint32_t)Peripheral;

    switch (Bus)
    {
        case MCAL_RCC_AHB1_BUS: p_enr_reg = RCC_AHB1ENR_REG; break;
        case MCAL_RCC_AHB2_BUS: p_enr_reg = RCC_AHB2ENR_REG; break;
        case MCAL_RCC_APB1_BUS: p_enr_reg = RCC_APB1ENR_REG; break;
        case MCAL_RCC_APB2_BUS: p_enr_reg = RCC_APB2ENR_REG; break;
        default: return MCAL_INVALID_PARAM;
    }

    if (p_enr_reg == NULL) return MCAL_ERROR;

    *p_enr_reg &= ~periph_bit;

    return MCAL_OK;
}

MCAL_Status_t MCAL_RCC_ResetPeripheral(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    volatile uint32_t *p_rstr_reg = NULL;
    uint32_t periph_bit = (uint32_t)Peripheral;

    switch (Bus)
    {
        case MCAL_RCC_AHB1_BUS: p_rstr_reg = RCC_AHB1RSTR_REG; break;
        case MCAL_RCC_AHB2_BUS: p_rstr_reg = RCC_AHB2RSTR_REG; break;
        case MCAL_RCC_APB1_BUS: p_rstr_reg = RCC_APB1RSTR_REG; break;
        case MCAL_RCC_APB2_BUS: p_rstr_reg = RCC_APB2RSTR_REG; break;
        default: return MCAL_INVALID_PARAM;
    }

    if (p_rstr_reg == NULL) return MCAL_ERROR;

    // Set bit to reset, then clear bit to release from reset
    *p_rstr_reg |= periph_bit;
    *p_rstr_reg &= ~periph_bit;

    return MCAL_OK;
}

bool MCAL_RCC_GetClockStatus(MCAL_RCC_Bus_t Bus, MCAL_RCC_Peripheral_t Peripheral)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    volatile uint32_t *p_enr_reg = NULL;
    uint32_t periph_bit = (uint32_t)Peripheral;

    switch (Bus)
    {
        case MCAL_RCC_AHB1_BUS: p_enr_reg = RCC_AHB1ENR_REG; break;
        case MCAL_RCC_AHB2_BUS: p_enr_reg = RCC_AHB2ENR_REG; break;
        case MCAL_RCC_APB1_BUS: p_enr_reg = RCC_APB1ENR_REG; break;
        case MCAL_RCC_APB2_BUS: p_enr_reg = RCC_APB2ENR_REG; break;
        default: return false;
    }

    if (p_enr_reg == NULL) return false;

    return ((*p_enr_reg & periph_bit) != 0U);
}

MCAL_Status_t MCAL_EXTI_Init(MCAL_EXTI_Line_t Line, MCAL_EXTI_PortSource_t PortSource, MCAL_EXTI_Trigger_t Trigger)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable SYSCFG clock
    MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_SYSCFG);

    // 2. Select the source input for the EXTIx external interrupt (SYSCFG_EXTICRx)
    uint32_t exticr_idx = Line / 4; // 0 for EXTI0-3, 1 for EXTI4-7, etc.
    uint33_t exticr_bit_pos = (Line % 4) * 4; // 4 bits per EXTI line

    volatile uint32_t *p_exticr_reg;
    switch (exticr_idx)
    {
        case 0: p_exticr_reg = SYSCFG_EXTICR1_REG; break;
        case 1: p_exticr_reg = SYSCFG_EXTICR2_REG; break;
        case 2: p_exticr_reg = SYSCFG_EXTICR3_REG; break;
        case 3: p_exticr_reg = SYSCFG_EXTICR4_REG; break;
        default: return MCAL_INVALID_PARAM;
    }

    *p_exticr_reg &= ~(0xFU << exticr_bit_pos); // Clear current bits
    *p_exticr_reg |= (PortSource << exticr_bit_pos); // Set new port source

    // 3. Configure trigger (Rising/Falling)
    MCAL_EXTI_SetTrigger(Line, Trigger);

    // 4. Optionally enable/disable Interrupt or Event requests later using MCAL_EXTI_EnableInterrupt/MCAL_EXTI_EnableEvent

    return MCAL_OK;
}

MCAL_Status_t MCAL_EXTI_EnableInterrupt(MCAL_EXTI_Line_t Line)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return MCAL_INVALID_PARAM;
    }

    *EXTI_IMR_REG |= (1U << Line); // Set the corresponding bit in Interrupt Mask Register

    return MCAL_OK;
}

MCAL_Status_t MCAL_EXTI_DisableInterrupt(MCAL_EXTI_Line_t Line)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return MCAL_INVALID_PARAM;
    }

    *EXTI_IMR_REG &= ~(1U << Line); // Clear the corresponding bit in Interrupt Mask Register

    return MCAL_OK;
}

MCAL_Status_t MCAL_EXTI_SetTrigger(MCAL_EXTI_Line_t Line, MCAL_EXTI_Trigger_t Trigger)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return MCAL_INVALID_PARAM;
    }

    // Clear existing trigger bits for the line
    *EXTI_RTSR_REG &= ~(1U << Line);
    *EXTI_FTSR_REG &= ~(1U << Line);

    switch (Trigger)
    {
        case MCAL_EXTI_TRIGGER_RISING:
            *EXTI_RTSR_REG |= (1U << Line);
            break;
        case MCAL_EXTI_TRIGGER_FALLING:
            *EXTI_FTSR_REG |= (1U << Line);
            break;
        case MCAL_EXTI_TRIGGER_RISING_FALLING:
            *EXTI_RTSR_REG |= (1U << Line);
            *EXTI_FTSR_REG |= (1U << Line);
            break;
        default:
            return MCAL_INVALID_PARAM;
    }

    return MCAL_OK;
}

bool MCAL_EXTI_GetPending(MCAL_EXTI_Line_t Line)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return false;
    }

    return ((*EXTI_PR_REG & (1U << Line)) != 0U);
}

MCAL_Status_t MCAL_EXTI_ClearPending(MCAL_EXTI_Line_t tLine)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Line > MCAL_EXTI_LINE_15)
    {
        return MCAL_INVALID_PARAM;
    }

    *EXTI_PR_REG = (1U << Line); // Write 1 to clear the pending bit

    return MCAL_OK;
}

MCAL_Status_t MCAL_ADC_Init(const MCAL_ADC_Init_t *p_ADC_Init)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (p_ADC_Init == NULL)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable ADC1 clock
    MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_ADC1);

    // 2. Disable ADC to configure
    *ADC1_CR2_REG &= ~ADC_CR2_ADON;

    // 3. Configure common ADC parameters (ADC_CCR)
    // Clear previous settings for prescaler (bits 17:16), independent mode (bits 13:10)
    *ADC_CCR_REG &= ~((0x3U << 16) | (0xFU << 16) | (0xFU << 10)); // clear ADCPRE and multi-mode bits
    // This example only configures clock prescaler (ADCPRE bits 17:16)
    // This assumes p_ADC_Init->ClockPrescaler holds the correct bit pattern (0x0, 0x1, 0x2, 0x3 for Div2,4,6,8)
    *ADC_CCR_REG |= (p_ADC_Init->ClockPrescaler << 16);

    // 4. Configure ADC1 control register 1 (ADC1_CR1)
    // Clear previous resolution (bits 25:24)
    *ADC1_CR1_REG &= ~(0x3U << 24);
    // This example only configures resolution (RES bits 25:24)
    // (00: 12-bit, 01: 10-bit, 10: 8-bit, 11: 6-bit)
    *ADC1_CR1_REG |= (p_ADC_Init->Resolution << 24);

    // 5. Configure ADC1 control register 2 (ADC1_CR2)
    // Clear previous conversion mode (CONT bit 1), data alignment (ALIGN bit 11)
    *ADC1_CR2_REG &= ~((1U << 1) | (1U << 11));
    // Set continuous conversion mode if required (CONT bit 1)
    if (p_ADC_Init->ConversionMode == 1U) // Assuming 1U for continuous
    {
        *ADC1_CR2_REG |= (1U << 1); // Set CONT bit
    }
    // Set data alignment (ALIGN bit 11)
    // Assuming 0 for right align, 1 for left align
    *ADC1_CR2_REG |= (p_ADC_Init->DataAlign << 11);

    // 6. Enable ADC
    *ADC1_CR2_REG |= ADC_CR2_ADON;

    return MCAL_OK;
}

MCAL_Status_t MCAL_ADC_StartConversion(void)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    // Start regular conversion by setting SWSTART bit
    *ADC1_CR2_REG |= ADC_CR2_SWSTART;

    return MCAL_OK;
}

uint32_t MCAL_ADC_ReadValue(void)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    // Wait for end of conversion (EOC flag)
    while (!(*ADC1_SR_REG & ADC_SR_EOC))
    {
        // Loop until conversion is complete
    }

    // Clear EOC flag by reading ADC_DR
    // Note: The EOC flag is cleared by software sequence: read ADC_SR then read ADC_DR
    // or by writing 0 to it for some modes. Here, reading DR implicitly clears it.
    
    // Read the converted data from ADC1_DR
    return *ADC1_DR_REG;
}

MCAL_Status_t MCAL_ADC_SetChannel(uint32_t Channel)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Channel > 18) // ADC channels 0-18
    {
        return MCAL_INVALID_PARAM;
    }

    // This function will configure the regular sequence register 3 (SQR3) for a single conversion.
    // For more complex sequences, SQR1 and SQR2 would also be used.
    // L[3:0] bits in SQR1 define the sequence length (0 for 1 conversion, up to F for 16 conversions)
    // SQx bits in SQR3, SQR2, SQR1 define the channels in the sequence.

    // For simplicity, let's assume we are setting up a single channel for regular conversion
    // Set sequence length to 1 conversion (L[3:0] = 0000 in SQR1)
    *ADC1_SQR1_REG &= ~(0xFU << 20); // Clear L[3:0]

    // Clear and set the first channel in the sequence (SQ1[4:0] in SQR3)
    *ADC1_SQR3_REG &= ~(0x1FU << 0); // Clear SQ1[4:0]
    *ADC1_SQR3_REG |= (Channel << 0); // Set new channel

    // Configure Sample Time (SMPR1 for channels 10-18, SMPR2 for channels 0-9)
    // This is a basic example; actual sample time would come from init struct
    uint32_t sample_time_bits = 0x3U; // Example: 28 cycles (0x3)
    if (Channel >= 10)
    {
        // For channels 10-18, use SMPR1
        *ADC1_SMPR1_REG &= ~(0x7U << ((Channel - 10) * 3)); // Clear SMPx bits
        *ADC1_SMPR1_REG |= (sample_time_bits << ((Channel - 10) * 3));
    }
    else
    {
        // For channels 0-9, use SMPR2
        *ADC1_SMPR2_REG &= ~(0x7U << (Channel * 3)); // Clear SMPx bits
        *ADC1_SMPR2_REG |= (sample_time_bits << (Channel * 3));
    }

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_Init(MCAL_Timer_t Timer, uint16_t Prescaler, uint32_t AutoReload)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable timer clock
    switch (Timer)
    {
        case MCAL_TIMER_1:  MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_TIM1); break;
        case MCAL_TIMER_2:  MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_TIM2); break;
        case MCAL_TIMER_3:  MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_TIM3); break;
        case MCAL_TIMER_4:  MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_TIM4); break;
        case MCAL_TIMER_5:  MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_TIM5); break;
        case MCAL_TIMER_9:  MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_TIM9); break;
        case MCAL_TIMER_10: MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_TIM10); break;
        case MCAL_TIMER_11: MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_TIM11); break;
        default: return MCAL_ERROR;
    }

    volatile uint32_t *p_cr1 = TIMER_REG(timer_base, TIMER_CR1_OFFSET);
    volatile uint32_t *p_psc = TIMER_REG(timer_base, TIMER_PSC_OFFSET);
    volatile uint32_t *p_arr = TIMER_REG(timer_base, TIMER_ARR_OFFSET);

    // 2. Stop the timer to configure
    *p_cr1 &= ~(1U << 0); // CEN bit (Counter Enable)

    // 3. Configure Prescaler
    *p_psc = Prescaler;

    // 4. Configure Auto-Reload Register
    *p_arr = AutoReload;

    // 5. Generate an update event to load the prescaler and ARR values
    *TIMER_REG(timer_base, TIMER_EGR_OFFSET) |= (1U << 0); // UG bit (Update Generation)

    // 6. Reset counter
    *TIMER_REG(timer_base, TIMER_CNT_OFFSET) = 0;

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_Start(MCAL_Timer_t Timer)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_cr1 = TIMER_REG(timer_base, TIMER_CR1_OFFSET);

    // Set CEN bit to enable the counter
    *p_cr1 |= (1U << 0);

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_Stop(MCAL_Timer_t Timer)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_cr1 = TIMER_REG(timer_base, TIMER_CR1_OFFSET);

    // Clear CEN bit to disable the counter
    *p_cr1 &= ~(1U << 0);

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_SetPrescaler(MCAL_Timer_t Timer, uint16_t Prescaler)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_psc = TIMER_REG(timer_base, TIMER_PSC_OFFSET);

    *p_psc = Prescaler;

    // Generate an update event to load the prescaler value
    *TIMER_REG(timer_base, TIMER_EGR_OFFSET) |= (1U << 0);

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_SetAutoReload(MCAL_Timer_t Timer, uint32_t AutoReload)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_arr = TIMER_REG(timer_base, TIMER_ARR_OFFSET);

    *p_arr = AutoReload;

    // Generate an update event to load the ARR value
    *TIMER_REG(timer_base, TIMER_EGR_OFFSET) |= (1U << 0);

    return MCAL_OK;
}

MCAL_Status_t MCAL_TIMER_SetCaptureCompare(MCAL_Timer_t Timer, uint8_t Channel, uint32_t CompareValue)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS || Channel == 0 || Channel > 4)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_ccr = NULL;
    volatile uint32_t *p_ccmr_reg = NULL;
    uint32_t ccmr_offset = 0;
    uint32_t ccmr_shift = 0;

    switch (Channel) {
        case 1: p_ccr = TIMER_REG(timer_base, TIMER_CCR1_OFFSET); p_ccmr_reg = TIMER_REG(timer_base, TIMER_CCMR1_OFFSET); ccmr_shift = 0; break;
        case 2: p_ccr = TIMER_REG(timer_base, TIMER_CCR2_OFFSET); p_ccmr_reg = TIMER_REG(timer_base, TIMER_CCMR1_OFFSET); ccmr_shift = 8; break;
        case 3: p_ccr = TIMER_REG(timer_base, TIMER_CCR3_OFFSET); p_ccmr_reg = TIMER_REG(timer_base, TIMER_CCMR2_OFFSET); ccmr_shift = 0; break;
        case 4: p_ccr = TIMER_REG(timer_base, TIMER_CCR4_OFFSET); p_ccmr_reg = TIMER_REG(timer_base, TIMER_CCMR2_OFFSET); ccmr_shift = 8; break;
        default: return MCAL_INVALID_PARAM;
    }

    if (p_ccr == NULL || p_ccmr_reg == NULL) {
        return MCAL_INVALID_PARAM; // Timer instance might not have all 4 channels
    }

    // Configure as output compare mode (e.g., PWM mode 1)
    // Clear CCxS bits (capture/compare selection, bits 1:0 or 9:8 depending on channel)
    // For output compare, CCxS must be '00'.
    *p_ccmr_reg &= ~(0x3U << ccmr_shift);

    // Set output compare mode (OCxM bits, typically bits 6:4 or 14:12 for channels 1/2)
    // Example: PWM mode 1 (0110b or 0x6)
    // Clear OCxM bits
    *p_ccmr_reg &= ~(0x7U << (4 + ccmr_shift));
    // Set to PWM mode 1
    *p_ccmr_reg |= (0x6U << (4 + ccmr_shift));

    // Enable output preload register
    *p_ccmr_reg |= (1U << (3 + ccmr_shift)); // OCxPE bit

    // Set the compare value
    *p_ccr = CompareValue;

    // Enable the capture/compare output
    *TIMER_REG(timer_base, TIMER_CCER_OFFSET) |= (1U << ((Channel - 1) * 4)); // CCxE bit

    // For advanced timers (like TIM1), enable Main Output Enable (MOE) bit in BDTR
    if (Timer == MCAL_TIMER_1)
    {
        volatile uint32_t *p_bdtr = TIMER_REG(timer_base, TIMER_BDTR_OFFSET);
        *p_bdtr |= (1U << 15); // MOE bit
    }

    return MCAL_OK;
}

uint32_t MCAL_TIMER_ReadCounter(MCAL_Timer_t Timer)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t timer_base = MCAL_TIMER_GetBaseAddress(Timer);
    if (timer_base == 0 || Timer >= MCAL_NUM_TIMERS)
    {
        return 0; // Indicate error
    }

    volatile uint32_t *p_cnt = TIMER_REG(timer_base, TIMER_CNT_OFFSET);

    return *p_cnt;
}


MCAL_Status_t MCAL_UART_Init(MCAL_UART_t Uart, const MCAL_UART_Init_t *p_UART_Init)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (p_UART_Init == NULL || Uart >= MCAL_NUM_UARTS)
    {
        return MCAL_INVALID_PARAM;
    }

    uint32_t uart_base = MCAL_UART_GetBaseAddress(Uart);
    if (uart_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable UART peripheral clock
    switch (Uart)
    {
        case MCAL_UART_1: MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_USART1); break;
        case MCAL_UART_2: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_USART2); break;
        case MCAL_UART_6: MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_USART6); break;
        default: return MCAL_ERROR;
    }

    volatile uint32_t *p_cr1 = USART_REG(uart_base, USART_CR1_OFFSET);
    volatile uint32_t *p_brr = USART_REG(uart_base, USART_BRR_OFFSET);
    volatile uint32_t *p_cr2 = USART_REG(uart_base, USART_CR2_OFFSET);
    volatile uint32_t *p_cr3 = USART_REG(uart_base, USART_CR3_OFFSET);

    // 2. Disable UART for configuration
    *p_cr1 &= ~(1U << 13); // UE bit (USART Enable)

    // 3. Configure Baud Rate (BRR)
    // Formula for BRR (assuming PCLK is SystemCoreClock / APBx_Prescaler):
    // USARTDIV = F_PCLK / (16 * BaudRate) for oversampling by 16
    // BRR = (USARTDIV_Mantissa << 4) | USARTDIV_Fraction
    // For simplicity, we'll assume p_UART_Init->BaudRate corresponds to a pre-calculated BRR value
    // or this function is called with F_PCLK and BaudRate to calculate.
    // Given no F_PCLK input, this is a placeholder.
    *p_brr = p_UART_Init->BaudRate; // Placeholder: Assume BaudRate input directly implies BRR value

    // 4. Configure Word Length, Parity, Mode (CR1)
    *p_cr1 &= ~((1U << 12) | (1U << 10) | (0x3U << 2)); // Clear M, PCE, PS, RE, TE bits
    *p_cr1 |= p_UART_Init->WordLength; // M bit (Word length)
    *p_cr1 |= p_UART_Init->Parity;     // PCE, PS bits (Parity enable, Parity selection)
    *p_cr1 |= p_UART_Init->Mode;       // RE, TE bits (Receiver Enable, Transmitter Enable)

    // 5. Configure Stop Bits (CR2)
    *p_cr2 &= ~(0x3U << 12); // Clear STOP bits
    *p_cr2 |= p_UART_Init->StopBits; // STOP bits

    // 6. Configure Flow Control (CR3) (CTS, RTS, DMAT, DMAR, EIE, etc.)
    // For basic UART, usually no flow control is enabled here by default.
    // This example will not set any CR3 bits unless specific configurations are passed.

    // 7. Enable UART
    *p_cr1 |= (1U << 13); // UE bit (USART Enable)

    return MCAL_OK;
}

MCAL_Status_t MCAL_UART_SendByte(MCAL_UART_t Uart, uint8_t Data)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t uart_base = MCAL_UART_GetBaseAddress(Uart);
    if (uart_base == 0 || Uart >= MCAL_NUM_UARTS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_sr = USART_REG(uart_base, USART_SR_OFFSET);
    volatile uint32_t *p_dr = USART_REG(uart_base, USART_DR_OFFSET);

    // Wait for Transmission Data Register Empty (TXE) flag
    while (!(*p_sr & (1U << 7))) {}

    // Write data to the Data Register
    *p_dr = Data;

    // Wait for Transmission Complete (TC) flag (optional, depends on use case)
    while (!(*p_sr & (1U << 6))) {}

    return MCAL_OK;
}

uint8_t MCAL_UART_ReceiveByte(MCAL_UART_t Uart)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t uart_base = MCAL_UART_GetBaseAddress(Uart);
    if (uart_base == 0 || Uart >= MCAL_NUM_UARTS)
    {
        return 0; // Indicate error
    }

    volatile uint32_t *p_sr = USART_REG(uart_base, USART_SR_OFFSET);
    volatile uint32_t *p_dr = USART_REG(uart_base, USART_DR_OFFSET);

    // Wait for Read Data Register Not Empty (RXNE) flag
    while (!(*p_sr & (1U << 5))) {}

    // Read data from the Data Register
    return (uint8_t)(*p_dr & 0xFFU);
}

MCAL_Status_t MCAL_I2C_Init(MCAL_I2C_t I2c, const MCAL_I2C_Init_t *p_I2C_Init)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (p_I2C_Init == NULL || I2c >= MCAL_NUM_I2CS)
    {
        return MCAL_INVALID_PARAM;
    }

    uint32_t i2c_base = MCAL_I2C_GetBaseAddress(I2c);
    if (i2c_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable I2C peripheral clock
    switch (I2c)
    {
        case MCAL_I2C_1: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_I2C1); break;
        case MCAL_I2C_2: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_I2C2); break;
        case MCAL_I2C_3: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_I2C3); break;
        default: return MCAL_ERROR;
    }

    volatile uint32_t *p_cr1 = I2C_REG(i2c_base, I2C_CR1_OFFSET);
    volatile uint33_t *p_cr2 = I2C_REG(i2c_base, I2C_CR2_OFFSET);
    volatile uint32_t *p_oar1 = I2C_REG(i2c_base, I2C_OAR1_OFFSET);
    volatile uint32_t *p_ccr = I2C_REG(i2c_base, I2C_CCR_OFFSET);
    volatile uint32_t *p_trise = I2C_REG(i2c_base, I2C_TRISE_OFFSET);

    // 2. Disable I2C peripheral to configure
    *p_cr1 &= ~(1U << 0); // PE bit (Peripheral Enable)

    // 3. Configure timing parameters (CR2, CCR, TRISE)
    // F_PCLK1 should be used here, assume 42MHz for APB1
    uint32_t pclk1_freq_mhz = 42U; // Example: 42 MHz
    *p_cr2 = pclk1_freq_mhz; // FREQ bits

    // Calculate CCR value (simplified)
    // For Standard Mode (Sm) I2C_CCR = T_PCLK1 / (2 * I2C_ClockSpeed)
    // For Fast Mode (Fm) I2C_CCR = T_PCLK1 / (3 * I2C_ClockSpeed) or T_PCLK1 / (25 * I2C_ClockSpeed) for 16/9 duty cycle
    // This is a placeholder; actual calculation depends on PCLK1 frequency and desired I2C_ClockSpeed
    if (p_I2C_Init->ClockSpeed <= 100000) // Standard Mode
    {
        *p_ccr = pclk1_freq_mhz * 1000000U / (2U * p_I2C_Init->ClockSpeed);
        *p_ccr &= ~(1U << 15); // Clear Fm/Sm bit for Standard Mode
    }
    else // Fast Mode
    {
        *p_ccr = pclk1_freq_mhz * 1000000U / (3U * p_I2C_Init->ClockSpeed); // Assuming 3:1 ratio
        *p_ccr |= (1U << 15); // Set Fm/Sm bit for Fast Mode
        // Configure duty cycle if Fast Mode (DUTY bit 14)
        if (p_I2C_Init->DutyCycle == 1) // Assuming 1 for 16/9, 0 for 2
        {
            *p_ccr |= (1U << 14); // Set DUTY for 16/9
        } else {
            *p_ccr &= ~(1U << 14); // Clear DUTY for 2
        }
    }

    // Configure TRISE (Maximum rise time in master mode)
    // For Standard mode: (1000 ns / T_PCLK1) + 1
    // For Fast mode: (300 ns / T_PCLK1) + 1
    if (p_I2C_Init->ClockSpeed <= 100000) // Standard Mode
    {
        *p_trise = (pclk1_freq_mhz * 1000U / 1000U) + 1U;
    }
    else // Fast Mode
    {
        *p_trise = (pclk1_freq_mhz * 300U / 1000U) + 1U;
    }


    // 4. Configure Own Address 1 (OAR1)
    *p_oar1 &= ~(0x3FFU << 0); // Clear current address bits
    *p_oar1 |= p_I2C_Init->OwnAddress1 << 1; // Set 7-bit address
    *p_oar1 |= p_I2C_Init->AddressingMode; // ADDMODE bit (bit 15) for 10-bit addressing
    *p_oar1 |= (1U << 14); // Always set bit 14 for 7-bit address or 10-bit address. (Datasheet)

    // 5. Configure Dual Address mode (OAR2, CR1)
    // This example only supports single address for now, set by OAR1.

    // 6. Enable I2C peripheral
    *p_cr1 |= (1U << 0); // PE bit

    return MCAL_OK;
}

MCAL_Status_t MCAL_I2C_Start(MCAL_I2C_t I2c)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t i2c_base = MCAL_I2C_GetBaseAddress(I2c);
    if (i2c_base == 0 || I2c >= MCAL_NUM_I2CS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_cr1 = I2C_REG(i2c_base, I2C_CR1_OFFSET);
    volatile uint32_t *p_sr1 = I2C_REG(i2c_base, I2C_SR1_OFFSET);

    // Generate START condition
    *p_cr1 |= (1U << 8); // START bit

    // Wait for SB (Start Bit) flag to be set
    // SB is cleared by reading SR1 followed by writing to DR
    while (!(*p_sr1 & (1U << 0))) {}

    return MCAL_OK;
}

MCAL_Status_t MCAL_I2C_Stop(MCAL_I2C_t I2c)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t i2c_base = MCAL_I2C_GetBaseAddress(I2c);
    if (i2c_base == 0 || I2c >= MCAL_NUM_I2CS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_cr1 = I2C_REG(i2c_base, I2C_CR1_OFFSET);

    // Generate STOP condition
    *p_cr1 |= (1U << 9); // STOP bit

    return MCAL_OK;
}

MCAL_Status_t MCAL_I2C_SendByte(MCAL_I2C_t I2c, uint8_t Data)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t i2c_base = MCAL_I2C_GetBaseAddress(I2c);
    if (i2c_base == 0 || I2c >= MCAL_NUM_I2CS)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_sr1 = I2C_REG(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t *p_dr = I2C_REG(i2c_base, I2C_DR_OFFSET);

    // Wait for TxE (Transmit data register empty) flag
    while (!(*p_sr1 & (1U << 7))) {}

    // Write data to DR
    *p_dr = Data;

    // Wait for BTF (Byte transfer finished) flag (optional, depending on next action)
    while (!(*p_sr1 & (1U << 2))) {}

    return MCAL_OK;
}

uint8_t MCAL_I2C_ReceiveByte(MCAL_I2C_t I2c, bool Ack)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t i2c_base = MCAL_I2C_GetBaseAddress(I2c);
    if (i2c_base == 0 || I2c >= MCAL_NUM_I2CS)
    {
        return 0; // Indicate error
    }

    volatile uint32_t *p_cr1 = I2C_REG(i2c_base, I2C_CR1_OFFSET);
    volatile uint32_t *p_sr1 = I2C_REG(i2c_base, I2C_SR1_OFFSET);
    volatile uint32_t *p_dr = I2C_REG(i2c_base, I2C_DR_OFFSET);

    if (Ack)
    {
        *p_cr1 |= (1U << 10); // Set ACK bit
    }
    else
    {
        *p_cr1 &= ~(1U << 10); // Clear ACK bit (NACK)
    }

    // Wait for RXNE (Receive data register not empty) flag
    while (!(*p_sr1 & (1U << 6))) {}

    uint8_t received_data = (uint8_t)(*p_dr);

    return received_data;
}


MCAL_Status_t MCAL_SPI_Init(MCAL_SPI_t Spi, const MCAL_SPI_Init_t *p_SPI_Init)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (p_SPI_Init == NULL || Spi >= MCAL_NUM_SPIS)
    {
        return MCAL_INVALID_PARAM;
    }

    uint32_t spi_base = MCAL_SPI_GetBaseAddress(Spi);
    if (spi_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    // 1. Enable SPI peripheral clock
    switch (Spi)
    {
        case MCAL_SPI_1: MCAL_RCC_EnableClock(MCAL_RCC_APB2_BUS, MCAL_RCC_APB2_SPI1); break;
        case MCAL_SPI_2: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_SPI2); break;
        case MCAL_SPI_3: MCAL_RCC_EnableClock(MCAL_RCC_APB1_BUS, MCAL_RCC_APB1_SPI3); break;
        default: return MCAL_ERROR;
    }

    volatile uint32_t *p_cr1 = SPI_REG(spi_base, SPI_CR1_OFFSET);
    volatile uint32_t *p_cr2 = SPI_REG(spi_base, SPI_CR2_OFFSET);
    volatile uint32_t *p_i2scfgr = SPI_REG(spi_base, SPI_I2SCFGR_OFFSET);

    // 2. Disable SPI peripheral for configuration
    *p_cr1 &= ~(1U << 6); // SPE bit (SPI Enable)

    // 3. Configure CR1 (Master/Slave, BaudRate, Clock Polarity/Phase, Data Size, First Bit)
    // Clear relevant bits
    *p_cr1 &= ~((1U << 2) | (1U << 1) | (1U << 0) | (0x7U << 3) | (1U << 11) | (1U << 7) | (1U << 8) | (1U << 9) | (1U << 10));
    *p_cr1 &= ~(0x3U << 13); // Clear BIDIMODE, BIDIOE
    *p_cr1 &= ~(1U << 15); // Clear CRCEN

    // Set Master/Slave mode (MSTR bit 2)
    *p_cr1 |= p_SPI_Init->Mode;

    // Set Baud Rate Prescaler (BR bits 5:3)
    *p_cr1 |= (p_SPI_Init->BaudRatePrescaler << 3);

    // Set Clock Polarity (CPOL bit 1) and Clock Phase (CPHA bit 0)
    *p_cr1 |= p_SPI_Init->CLKPolarity;
    *p_cr1 |= p_SPI_Init->CLKPhase;

    // Set Data Size (DFF bit 11)
    *p_cr1 |= p_SPI_Init->DataSize;

    // Set First Bit (LSBFIRST bit 7)
    *p_cr1 |= p_SPI_Init->FirstBit;

    // Set NSS management (SSM bit 9, SSI bit 8)
    if ((p_SPI_Init->NSS & (1U << 9)) != 0) // If SSM is enabled
    {
        *p_cr1 |= (1U << 9); // Set SSM
        if ((p_SPI_Init->NSS & (1U << 8)) != 0) // If SSI is high (master)
        {
            *p_cr1 |= (1U << 8); // Set SSI
        } else { // SSI is low (slave)
            *p_cr1 &= ~(1U << 8); // Clear SSI
        }
    } else {
        *p_cr1 &= ~(1U << 9); // Clear SSM
    }

    // Configure Direction (BIDIMODE bit 15, BIDIOE bit 14)
    *p_cr1 |= p_SPI_Init->Direction;

    // Configure CRC Calculation (CRCEN bit 13, CRCNEXT bit 12)
    if (p_SPI_Init->CRCCalculation != 0U)
    {
        *p_cr1 |= (1U << 13); // Set CRCEN
        // CRC Polynomial (SPI_CRCPR_REG) would be configured here if needed.
        *SPI_REG(spi_base, SPI_CRCPR_OFFSET) = p_SPI_Init->CRCPolynomial;
    }

    // 4. Configure CR2 (RxDMA, TxDMA, SSOW, FRF, ERRIE, etc.)
    // Clear relevant bits
    *p_cr2 &= ~((1U << 0) | (1U << 1) | (1U << 2)); // Clear RXDMAEN, TXDMAEN, SSOE
    *p_cr2 |= p_SPI_Init->TIMode; // TI mode (FRF bit 4)

    // 5. Disable I2S (I2SCFGR) mode explicitly if not using I2S
    *p_i2scfgr &= ~(1U << 11); // I2SE bit (I2S Enable)

    // 6. Enable SPI peripheral
    *p_cr1 |= (1U << 6); // SPE bit

    return MCAL_OK;
}

uint8_t MCAL_SPI_Transfer(MCAL_SPI_t Spi, uint8_t TxData)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    uint32_t spi_base = MCAL_SPI_GetBaseAddress(Spi);
    if (spi_base == 0 || Spi >= MCAL_NUM_SPIS)
    {
        return 0; // Indicate error
    }

    volatile uint32_t *p_sr = SPI_REG(spi_base, SPI_SR_OFFSET);
    volatile uint32_t *p_dr = SPI_REG(spi_base, SPI_DR_OFFSET);

    // Wait until TXE (Transmit buffer empty) is set
    while (!(*p_sr & (1U << 1))) {}

    // Write data to DR
    *p_dr = TxData;

    // Wait until RXNE (Receive buffer not empty) is set
    while (!(*p_sr & (1U << 0))) {}

    // Read data from DR
    return (uint8_t)(*p_dr & 0xFFU);
}

MCAL_Status_t MCAL_SPI_ReadWrite(MCAL_SPI_t Spi, const uint8_t *p_TxData, uint8_t *p_RxData, uint16_t Size)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    if (Spi >= MCAL_NUM_SPIS || Size == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    uint32_t spi_base = MCAL_SPI_GetBaseAddress(Spi);
    if (spi_base == 0)
    {
        return MCAL_INVALID_PARAM;
    }

    volatile uint32_t *p_sr = SPI_REG(spi_base, SPI_SR_OFFSET);
    volatile uint33_t *p_dr = SPI_REG(spi_base, SPI_DR_OFFSET);

    uint16_t i;
    for (i = 0; i < Size; i++)
    {
        uint8_t tx_byte = (p_TxData != NULL) ? p_TxData[i] : 0xFFU; // Send 0xFF if no transmit data (for receiving)

        // Wait until TXE (Transmit buffer empty) is set
        while (!(*p_sr & (1U << 1))) {}

        // Write data to DR
        *p_dr = tx_byte;

        // Wait until RXNE (Receive buffer not empty) is set
        while (!(*p_sr & (1U << 0))) {}

        // Read data from DR
        if (p_RxData != NULL)
        {
            p_RxData[i] = (uint8_t)(*p_dr & 0xFFU);
        }
        else
        {
            (void)*p_dr; // Read to clear RXNE even if not storing
        }
    }
    // Wait for BSY (Busy) flag to clear after the last transfer (optional for some scenarios)
    while ((*p_sr & (1U << 7))) {}

    return MCAL_OK;
}

MCAL_Status_t MCAL_WDT_Init(uint32_t TimeoutMs)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    // Function not implemented: Required watchdog timer registers (e.g., IWDG_KR, IWDG_PR, IWDG_RLR)
    // are not found in the provided Register JSON.
    // As per instruction "ABSOLUTELY DO NOT: Invent or assume any registers, peripherals, or API functions not present in the inputs."
    // a concrete implementation with register access cannot be provided.
    (void)TimeoutMs; // Suppress unused parameter warning
    return MCAL_NOT_IMPLEMENTED;
}

MCAL_Status_t MCAL_WDT_Refresh(void)
{
    // CRITICAL_SECTION_START();
    // CRITICAL_SECTION_END();

    // Function not implemented: Required watchdog timer registers (e.g., IWDG_KR)
    // are not found in the provided Register JSON.
    // As per instruction "ABSOLUTELY DO NOT: Invent or assume any registers, peripherals, or API functions not present in the inputs."
    // a concrete implementation with register access cannot be provided.
    return MCAL_NOT_IMPLEMENTED;
}