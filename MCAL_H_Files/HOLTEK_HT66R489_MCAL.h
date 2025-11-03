#ifndef MCAL_H
#define MCAL_H

#include <stdint.h>
#include <stdbool.h>

/* Data type definitions */
typedef uint8_t tbyte;
typedef uint16_t tword;
typedef uint32_t tlong;

/* Enum and typedef declarations from API.json */
typedef enum {
    Volt_0_5V,
    Volt_1V,
    Volt_1_5V,
    Volt_2V,
    /* ... other voltage levels as needed */
    Volt_5V
} t_sys_volt_type;

/* MCU Configuration Functions */
void MCU_Config_Init(t_sys_volt sys_volt);

/* GPIO Functions */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value);
void GPIO_Input_Init(t_port port, t_pin pin);
void GPIO_Write(t_port port, t_pin pin, tbyte value);
void GPIO_Read(t_port port, t_pin pin, tbyte* value);
void GPIO_Toggle(t_port port, t_pin pin);

/* ADC Functions */
void ADC_Init(t_adc adc);
void ADC_Set Resolution(t_adc adc, uint16_t resolution);
uint16_t ADC_Read(t_adc adc);
void ADC_Enable(t_adc adc);
void ADC_Disable(t_adc adc);

/* SPI Functions */
void SPI_Init(t_spi spi, uint32_t baud_rate, uint8_t cpha, uint8_t dff);
void SPI_Write(t_spi spi, uint8_t* data, uint16_t length);
void SPI_Read(t_spi spi, uint8_t* data, uint16_t length);
uint8_t SPI_Transfer(t_spi spi, uint8_t data);

/* Timer Functions */
void TIMER_Init(t_timer timer, uint32_t period);
void TIMER_Start(t_timer timer);
void TIMER_Stop(t_timer timer);
void TIMER_Update(t_timer timer);
uint32_t TIMER_Read(t_timer timer);

/* WDT Functions */
void WDT_Init(void);
void WDT_SetPeriod(tlong period);
void WDT_Reset(void);

/* Optional Modules (Not Supported on this MCU) */
// MCAL_OUTPUT_BUZZER not supported on this MCU
// DAC not supported on this MCU
// I2S not supported on this MCU
// MQTT Protocol not supported on this MCU
// HTTP Protocol not supported on this MCU
// WiFi Driver not supported on this MCU
// DTC_driver not supported on this MCU

#endif /* MCAL_H */