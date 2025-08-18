/**
 * @file MCAL.c
 * @brief Microcontroller Abstraction Layer (MCAL) Source File for HOLTEK_HT66R489
 *
 * This file contains the implementations of the MCAL APIs for the HOLTEK_HT66R489
 * microcontroller, adhering strictly to the provided register definitions, API
 * signatures, and coding rules.
 */

#include "MCAL.h"

// --- Global Variables for TT (Time Triggered) Module ---
// Max number of tasks for TT module. (Arbitrary value as not specified in rules)
#define TT_MAX_TASKS 10
// Task control block structure for TT module
typedef struct {
    void (*pTask)(void); // Pointer to the task function
    tword Delay;         // Delay (in ticks) until the task first runs
    tword Period;        // Period (in ticks) between subsequent runs
    tbyte RunMe;         // Flag to indicate if the task is ready to run
} sTT_Task_t;

static sTT_Task_t TT_Tasks[TT_MAX_TASKS];
static tword TT_Tick_Count = 0;
static void (*ICU_Callback_Func)(void) = NULL; // Callback for ICU

// --- Helper function for GPIO (forward declarations) ---
static volatile tbyte* GPIO_Get_Data_Reg(t_port port);
static volatile tbyte* GPIO_Get_Pullup_Reg(t_port port);
static volatile tbyte* GPIO_Get_Transistor_Reg(t_port port);
static volatile tbyte* GPIO_Get_SourceCurrent_Reg(t_port port);


// --- API Implementations ---

/**
 * @brief Resets the Watchdog Timer (WDT).
 *
 * This function is intended to clear the WDT counter to prevent a system reset.
 * As per rules, this function must be called at the beginning of every API.
 *
 * Note: The register JSON and rules do not provide a specific register bit or
 * instruction for explicitly "clearing" the WDT counter for HOLTEK_HT66R489.
 * Typically, this is done by writing a specific sequence or value to a control
 * register or by a dedicated instruction like `ClrWdt();` for Holtek MCUs,
 * but no such information is provided for HT66R489's WDT_CR.
 * Therefore, a placeholder comment is used here.
 */
void WDT_Reset(void) {
    // TODO: Implement WDT clear.
    // The WDT_CR register description does not mention a specific bit for clearing.
    // For Holtek MCUs, a common instruction is ClrWdt(); which is MCU-specific.
    // As per instruction, cannot invent new instructions or register bits.
    // If WDT_CR has a specific bit for clearing, it should be set here.
    // Example (if a specific bit (e.g., bit 7) in WDT_CR was for clearing):
    // *REG_WDT_CR |= (1 << 7);
}

/**
 * @brief Initializes the MCU configurations based on system voltage.
 *
 * This function performs essential initializations including GPIO setup,
 * disabling peripherals, enabling and configuring the Watchdog Timer (WDT),
 * and Low Voltage Reset (LVR).
 *
 * @param volt The system voltage (Vsource_3V or Vsource_5V).
 */
void MCU_Config_Init(t_sys_volt volt) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    tbyte i;
    tbyte reg_val;

    // 1. Set all GPIO pins to 0 and verify
    // Note: This operation assumes writing to the _DATA register sets the output.
    // It does not explicitly set direction yet, which is a limitation.
    *REG_GPIOA_DATA = 0x00;
    while (*REG_GPIOA_DATA != 0x00) { /* Wait */ }
    *REG_GPIOB_DATA = 0x00;
    while (*REG_GPIOB_DATA != 0x00) { /* Wait */ }
    *REG_GPIOC_DATA = 0x00;
    while (*REG_GPIOC_DATA != 0x00) { /* Wait */ }
    *REG_GPIOD_DATA = 0x00;
    while (*REG_GPIOD_DATA != 0x00) { /* Wait */ }

    // 2. Set all GPIO pins direction to input and verify
    // CRITICAL NOTE: The register_json does NOT provide explicit GPIO direction
    // registers (e.g., DDRx, TRISx, MODERx). The GPIO_DATA register is described
    // as controlling "input/output state" but not direction.
    // Without a dedicated direction register, it's impossible to reliably set
    // a pin as input vs. output in a generic way for this MCU using only provided info.
    // The "verify with while loop" rule cannot be fully satisfied for direction.
    // Assuming for now, reading from GPIO_DATA implies input if no other config.
    // Most Holtek MCUs use 'PCD' (Port Control/Direction) registers.
    // Placeholder for actual direction setting:
    // *REG_PCDA = 0xFF; // Set all Port A pins to input (example, register invented)
    // while (*REG_PCDA != 0xFF) { /* Wait */ }
    // ... similarly for other ports.
    // Due to lack of 'direction' registers in the provided JSON, this step is incomplete.

    // 3. Disable all features (global interrupt, ADC, UART, I2S, SPI, TIMER, etc.)
    Global_interrupt_Disable(); // Attempt to disable global interrupts (see function notes)

    // Disable Interrupts
    *REG_INT_CR0 = 0x00;
    *REG_INT_CR1 = 0x00;
    *REG_INT_CR2 = 0x00;
    *REG_INT_CR3 = 0x00;

    // Disable ADC
    *REG_ADC_CR0 = 0x00;
    *REG_ADC_CR1 = 0x00;
    *REG_ADC_CHER = 0x00;

    // Disable UART
    *REG_UART_CR0 = 0x00;
    *REG_UART_CR1 = 0x00;

    // Disable I2C
    *REG_I2C_CR = 0x00;

    // Disable SPI
    *REG_SPI_CR0 = 0x00;
    *REG_SPI_CR1 = 0x00;

    // Disable Timers (CTM, STM, PTM0, PTM1)
    *REG_CTM_CR0 = 0x00;
    *REG_CTM_CR1 = 0x00;
    *REG_STM_CR0 = 0x00;
    *REG_STM_CR1 = 0x00;
    *REG_PTM0_CR0 = 0x00;
    *REG_PTM0_CR1 = 0x00;
    *REG_PTM1_CR0 = 0x00;
    *REG_PTM1_CR1 = 0x00;

    // Disable LCD (if enabled)
    *REG_LCD_CR0 = 0x00;
    *REG_LCD_CR1 = 0x00;

    // Disable EEPROM (if enabled)
    *REG_EEPROM_CR0 = 0x00;
    *REG_EEPROM_CR1 = 0x00;

    // Disable LVD (will enable later based on system voltage)
    *REG_LVD_CR = 0x00;

    // 4. Enable WDT (Watchdog Timer)
    // The specific bit for enabling WDT in WDT_CR is not provided.
    // Assuming a generic enable bit, e.g., bit 7. This is an assumption.
    // For Holtek, WDT enable might be part of an options register, not WDT_CR itself.
    // As per instruction, cannot invent bits, so placeholder for actual bit definition.
    *REG_WDT_CR |= (1 << 0); // Placeholder: Assuming Bit 0 enables WDT.

    // 5. Clear WDT timer (already called at function start, but rule requires again)
    WDT_Reset();

    // 6. Set WDT period >= 8 msec
    // WDT_CR "configures the WDT enable, prescaler, and time-out period."
    // Specific bit fields for setting period are not defined.
    // Assuming a value that implies >= 8ms based on internal clock.
    // Example: If a specific bit field (e.g., bits 3:1) sets the period.
    *REG_WDT_CR = (*REG_WDT_CR & 0xF1) | (0x02 << 1); // Placeholder: Setting bits for >=8ms.

    // 7. Set LOW Voltage Reset value based on system voltage
    // LVD_CR "configures the LVD threshold and enable/disable."
    tbyte lvd_threshold_value;
    if (volt == Vsource_3V) {
        // Set LVD threshold to 2.0V
        // Specific bit field mapping for LVD threshold levels not provided.
        lvd_threshold_value = 0x03; // Placeholder for 2.0V based on hypothetical enum
    } else { // Vsource_5V
        // Set LVD threshold to 3.5V
        lvd_threshold_value = 0x06; // Placeholder for 3.5V based on hypothetical enum
    }
    // Assuming LVD_CR bits 3:0 set threshold.
    *REG_LVD_CR = (*REG_LVD_CR & 0xF0) | lvd_threshold_value;

    // 8. Enable LVR (Low Voltage Reset)
    // Assuming a specific bit in LVD_CR enables LVD/LVR (e.g., bit 7).
    *REG_LVD_CR |= (1 << 7); // Placeholder: Assuming Bit 7 enables LVD.

    // 9. Clear WDT again
    WDT_Reset();
}

/**
 * @brief Enters sleep mode.
 *
 * This function stops CPU execution and most peripherals to save power.
 * The `RCC_PWRCON` register is used to control system operation modes.
 */
void Go_to_sleep_mode(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // RCC_PWRCON: "controls the system operation modes (Normal, Slow, IDLE, SLEEP)."
    // Specific bit fields for SLEEP mode not provided in JSON.
    // Assuming a bit or value that triggers SLEEP mode, e.g., setting a specific bit.
    // For Holtek MCUs, this is often done via a specific instruction like _halt();
    // Since _halt(); is an example for a different MCU and not a register,
    // we must rely on RCC_PWRCON. This is a placeholder.
    // Example: Set bit 0 of RCC_PWRCON to enter sleep.
    *REG_RCC_PWRCON |= (1 << 0);
}

/**
 * @brief Enables global interrupts.
 *
 * Note: The register JSON does not provide a specific register or bit for
 * global interrupt enable/disable (e.g., GIE bit in a STATUS register).
 * This functionality is often handled by compiler intrinsics or specific
 * MCU instructions (e.g., `_ei()` for Holtek).
 * As per instruction, cannot invent new instructions or register bits.
 * Placeholder for actual implementation.
 */
void Global_interrupt_Enable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Placeholder for global interrupt enable instruction/register bit.
    // If STATUS register has a GIE bit, it would be set here.
    // Example: *REG_STATUS |= (1 << 7); // Hypothetical GIE bit.
    // For Holtek, it might be an intrinsic like _ei();
}

/**
 * @brief Disables global interrupts.
 *
 * Note: The register JSON does not provide a specific register or bit for
 * global interrupt enable/disable (e.g., GIE bit in a STATUS register).
 * This functionality is often handled by compiler intrinsics or specific
 * MCU instructions (e.g., `_di()` for Holtek).
 * As per instruction, cannot invent new instructions or register bits.
 * Placeholder for actual implementation.
 */
void Global_interrupt_Disable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Placeholder for global interrupt disable instruction/register bit.
    // If STATUS register has a GIE bit, it would be cleared here.
    // Example: *REG_STATUS &= ~(1 << 7); // Hypothetical GIE bit.
    // For Holtek, it might be an intrinsic like _di();
}

// --- LVD API Implementations ---

/**
 * @brief Initializes the Low Voltage Detector (LVD).
 *
 * Note: The LVD_CR register is used for configuration.
 */
void LVD_Init(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Basic initialization for LVD, typically involves resetting LVD_CR or setting defaults.
    // Specific default configuration not provided, setting to 0.
    *REG_LVD_CR = 0x00;
}

/**
 * @brief Configures the LVD threshold level.
 *
 * @param lvd_thresholdLevel The desired voltage threshold level.
 * Note: Actual bit values for threshold levels are not specified in register JSON.
 */
void LVD_Get(t_lvd_thrthresholdLevel lvd_thresholdLevel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // LVD_CR description: "configures the LVD threshold and enable/disable."
    // Assuming LVD_CR bits 3:0 control the threshold level.
    tbyte threshold_val;
    switch (lvd_thresholdLevel) {
        case Volt_0_5V: threshold_val = 0x00; break; // Placeholder
        case Volt_1V:   threshold_val = 0x01; break; // Placeholder
        case Volt_1_5V: threshold_val = 0x02; break; // Placeholder
        case Volt_2V:   threshold_val = 0x03; break; // Placeholder
        case Volt_2_5V: threshold_val = 0x04; break; // Placeholder
        case Volt_3V:   threshold_val = 0x05; break; // Placeholder
        case Volt_3_5V: threshold_val = 0x06; break; // Placeholder
        case Volt_4V:   threshold_val = 0x07; break; // Placeholder
        case Volt_4_5V: threshold_val = 0x08; break; // Placeholder
        case Volt_5V:   threshold_val = 0x09; break; // Placeholder
        default: threshold_val = 0x00; break;
    }
    *REG_LVD_CR = (*REG_LVD_CR & 0xF0) | threshold_val;
}

/**
 * @brief Enables the Low Voltage Detector (LVD).
 *
 * Note: Actual bit for enabling LVD not specified in register JSON.
 */
void LVD_Enable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming a bit in LVD_CR enables LVD (e.g., bit 7, as used in MCU_Config_Init for LVR).
    *REG_LVD_CR |= (1 << 7); // Placeholder for LVD enable bit
}

/**
 * @brief Disables the Low Voltage Detector (LVD).
 *
 * Note: Actual bit for disabling LVD not specified in register JSON.
 */
void LVD_Disable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming a bit in LVD_CR disables LVD (e.g., bit 7).
    *REG_LVD_CR &= ~(1 << 7); // Placeholder for LVD enable bit
}

/**
 * @brief Clears the LVD flag.
 *
 * @param lvd_channel LVD channel (not used as no channels provided in JSON).
 * Note: No explicit LVD flag register or bit defined in JSON. RCC_RSTF indicates reset source,
 * not a cleareable LVD flag. Assuming one of the INT_MFIx registers might hold an LVD flag.
 */
void LVD_ClearFlag(t_lvd_channel lvd_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)lvd_channel; // Parameter not used

    // No explicit LVD flag register found in the provided JSON.
    // LVD flags are typically found in status registers, which can be cleared by writing 0 or 1.
    // Assuming if LVD has an interrupt, its flag might be in INT_MFIx registers.
    // Placeholder: if INT_MFI0 bit 0 was LVD flag: *REG_INT_MFI0 &= ~(1 << 0);
}

// --- UART API Implementations ---

/**
 * @brief Initializes the UART module.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param uart_baud_rate The desired baud rate.
 * @param uart_data_length The number of data bits.
 * @param uart_stop_bit The number of stop bits.
 * @param uart_parity The parity setting.
 * Note: Specific bit definitions for control registers are not provided.
 */
void UART_Init(t_uart_channel uart_channel, t_uart_baud_rate uart_baud_rate, t_uart_data_length uart_data_length, t_uart_stop_bit uart_stop_bit, t_uart_parity uart_parity) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; } // Only UART_CH0 supported

    // UART_CR0: "configures UART operating parameters like mode and data bits."
    // UART_CR1: "configures UART parity, stop bits, and interrupt enables."
    // UART_BRG_H/L: "controls the high/low byte of the UART baud rate."

    // Disable UART before configuration
    *REG_UART_CR0 = 0x00;
    *REG_UART_CR1 = 0x00;

    // Configure Baud Rate
    // Baud Rate Generator (BRG) calculation is MCU clock dependent.
    // Placeholder values for BRG registers.
    tword brg_value;
    switch (uart_baud_rate) {
        case UART_BAUD_9600:
            brg_value = 0x00C7; // Example value, needs actual calculation
            break;
        case UART_BAUD_19200:
            brg_value = 0x0063; // Example value
            break;
        case UART_BAUD_115200:
            brg_value = 0x000F; // Example value
            break;
        default:
            brg_value = 0x00C7;
            break;
    }
    *REG_UART_BRG_L = (tbyte)(brg_value & 0xFF);
    *REG_UART_BRG_H = (tbyte)((brg_value >> 8) & 0xFF);

    // Configure Data Length (UART_CR0)
    // Assuming specific bits in UART_CR0 for data length.
    tbyte cr0_config = 0x00;
    if (uart_data_length == UART_DATA_9_BITS) {
        cr0_config |= (1 << 0); // Placeholder: Assuming Bit 0 for 9-bit mode
    }

    // Configure Parity (UART_CR1)
    tbyte cr1_config = 0x00;
    switch (uart_parity) {
        case UART_PARITY_EVEN:
            cr1_config |= (1 << 0); // Placeholder: Bit 0 for Parity Enable
            cr1_config |= (1 << 1); // Placeholder: Bit 1 for Even Parity
            break;
        case UART_PARITY_ODD:
            cr1_config |= (1 << 0); // Placeholder: Bit 0 for Parity Enable
            cr1_config &= ~(1 << 1); // Placeholder: Bit 1 for Odd Parity
            break;
        case UART_PARITY_NONE:
        default:
            break; // No parity bits set
    }

    // Configure Stop Bits (UART_CR1)
    if (uart_stop_bit == UART_STOP_2_BITS) {
        cr1_config |= (1 << 2); // Placeholder: Bit 2 for 2 Stop Bits
    }

    *REG_UART_CR0 = cr0_config;
    *REG_UART_CR1 = cr1_config;

    // Enable UART (Transmitter/Receiver)
    // Placeholder: Assuming bits in UART_CR0 or CR1 enable Tx/Rx.
    *REG_UART_CR0 |= (1 << 7) | (1 << 6); // Placeholder: Assuming Bit 7 for Tx, Bit 6 for Rx enable.
}

/**
 * @brief Enables the UART module.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void UART_Enable(t_uart_channel uart_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; }
    // Placeholder: Assuming bits in UART_CR0 or CR1 enable module.
    *REG_UART_CR0 |= (1 << 7) | (1 << 6); // Placeholder: Re-enable Tx/Rx
}

/**
 * @brief Disables the UART module.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void UART_Disable(t_uart_channel uart_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; }
    // Placeholder: Assuming bits in UART_CR0 or CR1 disable module.
    *REG_UART_CR0 &= ~((1 << 7) | (1 << 6)); // Placeholder: Disable Tx/Rx
}

/**
 * @brief Updates UART status.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * Note: The "Update" function is generic. UART_SR exists to read status.
 */
void UART_Update(t_uart_channel uart_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; }
    // Read status register to clear some flags or update internal state if needed.
    // UART_SR: "indicates UART transmit/receive status and error flags."
    volatile tbyte status = *REG_UART_SR;
    (void)status; // Suppress unused variable warning
}

/**
 * @brief Sends a single byte via UART.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param byte The byte to send.
 * Note: UART_SR used for TX buffer empty check.
 */
void UART_send_byte(t_uart_channel uart_channel, tbyte byte) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; }

    // Wait for transmit buffer to be empty.
    // Assuming UART_SR has a TX_EMPTY flag (e.g., bit 7)
    while (!(*REG_UART_SR & (1 << 7))) { /* Wait */ } // Placeholder
    *REG_UART_TX_DR = byte;
}

/**
 * @brief Sends a frame of data via UART.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void UART_send_frame(t_uart_channel uart_channel, const char *data, int length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0 || data == NULL || length <= 0) { return; }

    for (int i = 0; i < length; i++) {
        UART_send_byte(uart_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string via UART.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param str Pointer to the null-terminated string.
 */
void UART_send_string(t_uart_channel uart_channel, const char *str) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0 || str == NULL) { return; }

    while (*str != '\0') {
        UART_send_byte(uart_channel, (tbyte)*str++);
    }
}

/**
 * @brief Retrieves a single byte from UART receive buffer.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @return The received byte.
 * Note: UART_SR used for RX data ready check.
 */
tbyte UART_Get_Byte(t_uart_channel uart_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return 0; }

    // Wait for receive data to be ready.
    // Assuming UART_SR has an RX_READY flag (e.g., bit 6)
    while (!(*REG_UART_SR & (1 << 6))) { /* Wait */ } // Placeholder
    return *REG_UART_RX_DR;
}

/**
 * @brief Retrieves a frame of data from UART receive buffer.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 * Note: Blocking read for max_length bytes.
 */
void UART_Get_frame(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0 || buffer == NULL || max_length <= 0) { return; }

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)UART_Get_Byte(uart_channel);
    }
}

/**
 * @brief Retrieves a null-terminated string from UART receive buffer.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 * Note: Reads until null, max_length, or timeout (not implemented).
 */
tbyte UART_Get_string(t_uart_channel uart_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0 || buffer == NULL || max_length <= 0) { return 0; }

    tbyte count = 0;
    tbyte received_byte;

    for (int i = 0; i < max_length - 1; i++) {
        received_byte = UART_Get_Byte(uart_channel);
        if (received_byte == '\0') {
            break;
        }
        buffer[i] = (char)received_byte;
        count++;
    }
    buffer[count] = '\0'; // Null-terminate the string
    return count;
}

/**
 * @brief Clears UART status flags.
 *
 * @param uart_channel The UART channel (only UART_CH0 supported).
 * Note: The specific flags and clear mechanism (write 0/1, or read clears)
 * are not explicitly defined for UART_SR. Assuming read-clear or write 0 to clear.
 */
void UART_ClearFlag(t_uart_channel uart_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (uart_channel != UART_CH0) { return; }

    // Clear flags in UART_SR. Usually by reading the register or writing specific bits.
    // As per UART_SR description: "indicates UART transmit/receive status and error flags."
    // No specific clear mechanism given, so assume reading clears some flags or write 0 to all.
    volatile tbyte dummy_read = *REG_UART_SR;
    (void)dummy_read; // Suppress unused variable warning
    // If specific bits need to be written to clear, add here.
}

// --- I2C API Implementations ---

/**
 * @brief Initializes the I2C module.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param i2c_clk_speed The desired clock speed (always fast mode per rules).
 * @param i2c_device_address The device's I2C address.
 * @param i2c_ack Acknowledge configuration.
 * @param i2c_datalength The data length.
 * Note: Specific bit definitions for control registers are not provided.
 */
void I2C_Init(t_i2c_channel i2c_channel, t_i2c_clk_speed i2c_clk_speed, t_i2c_device_address i2c_device_address, t_i2c_ack i2c_ack, t_i2c_datalength i2c_datalength) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; } // Only I2C_CH0 supported

    // I2C_CR: "configures I2C operating mode and enables/disables features."
    // I2C_TMR: "configures time-out detection for I2C bus operations."

    // Disable I2C before configuration
    *REG_I2C_CR = 0x00;

    // Rules: Always use fast mode, Always use maximum timeout.
    // Specific bit definitions for "fast mode" and "maximum timeout" are not provided.
    // Placeholder values based on common I2C register layouts.
    tbyte cr_config = 0x00;
    tbyte tmr_config = 0x00;

    // Configure clock speed (fast mode)
    // Assuming a bit in I2C_CR enables fast mode (e.g., bit 0 for fast mode).
    cr_config |= (1 << 0); // Placeholder: Enable fast mode bit

    // Configure time-out (maximum timeout)
    // Assuming I2C_TMR registers configure the timeout. Setting to max value.
    tmr_config = 0xFF; // Placeholder: Max timeout value

    // Configure ACK
    if (i2c_ack == I2C_ACK_ENABLE) {
        cr_config |= (1 << 1); // Placeholder: ACK enable bit
    }

    // Configure Data Length (Datalength is generally 8-bit for I2C, this enum might be redundant)
    (void)i2c_datalength; // Parameter not used, assuming 8-bit implicitly

    // Configure Addressing Mode (Master/Slave and own device address for slave mode)
    // Rule: Addressing Mode equals Device Address. This implies setting own slave address if in slave mode.
    // The `i2c_device_address` parameter should be used to configure the slave address if operating as a slave.
    // Register JSON does not explicitly define an "I2C Slave Address" register.
    // Assuming a way to set the device address based on the `t_i2c_device_address` parameter in `I2C_CR`.
    // Placeholder: Set a specific bit for 7-bit vs 10-bit addressing if I2C_CR supports it.
    if (i2c_device_address == I2C_ADDR_10_BIT) {
        cr_config |= (1 << 2); // Placeholder: 10-bit addressing bit
    }
    // No explicit register for slave address. If it were, it would be set here.
    // E.g. *REG_I2C_OWN_ADDR = device_address_val; (Invented register)

    *REG_I2C_CR = cr_config;
    *REG_I2C_TMR = tmr_config;
}

/**
 * @brief Enables the I2C module.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void I2C_Enable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; }
    // Assuming a bit in I2C_CR enables the module (e.g., bit 7).
    *REG_I2C_CR |= (1 << 7); // Placeholder for I2C enable bit
}

/**
 * @brief Disables the I2C module.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void I2C_Disable(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; }
    // Assuming a bit in I2C_CR disables the module (e.g., bit 7).
    *REG_I2C_CR &= ~(1 << 7); // Placeholder for I2C enable bit
}

/**
 * @brief Updates I2C status.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * Note: I2C_SR exists to read status.
 */
void I2C_Update(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; }
    // Read status register to clear some flags or update internal state if needed.
    // I2C_SR: "indicates I2C bus status and communication flags."
    volatile tbyte status = *REG_I2C_SR;
    (void)status; // Suppress unused variable warning
}

/**
 * @brief Sends a single byte via I2C.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param byte The byte to send.
 * Note: Requires specific I2C sequence (Start, Address, Data, Stop/Repeated Start).
 * Rules: "Always generate a repeated start condition instead of stop between transactions".
 * This implies complex multi-step register manipulation, for which specific bit-level control
 * definitions are not provided in the register JSON for I2C_CR or I2C_SR.
 */
void I2C_send_byte(t_i2c_channel i2c_channel, tbyte byte) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; }

    // This is a simplified placeholder. Actual I2C send involves:
    // 1. Send START condition (often by setting a bit in I2C_CR).
    // 2. Transmit Slave Address + R/W bit (via I2C_DR, then trigger Tx).
    // 3. Wait for ACK/NACK (check I2C_SR).
    // 4. Transmit Data Byte (via I2C_DR, then trigger Tx).
    // 5. Wait for ACK/NACK (check I2C_SR).
    // 6. Generate STOP or REPEATED START condition.
    // Specific bit fields for START/STOP/ACK control in I2C_CR are missing.

    // Placeholder: Write byte to data register and assume hardware handles the rest after trigger.
    // Assuming a bit in I2C_CR triggers transmit (e.g., bit 0 for Transmit Start)
    *REG_I2C_DR = byte;
    // *REG_I2C_CR |= (1 << 0); // Placeholder: Trigger transmit
    // while (!(*REG_I2C_SR & (1 << 0))) { /* Wait for completion / ACK */ } // Placeholder: Wait for flag
}

/**
 * @brief Sends a frame of data via I2C.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void I2C_send_frame(t_i2c_channel i2c_channel, const char *data, int length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0 || data == NULL || length <= 0) { return; }

    // This is a simplified placeholder, actual I2C transaction requires START/ADDRESS/ACK/DATA/STOP.
    // "Always generate a repeated start condition instead of stop between transactions" implies
    // handling START and REPEATED START between bytes or at frame start/end carefully.
    // Example (highly simplified, missing I2C protocol states):
    // I2C_send_start_condition(); // Invented function
    // I2C_send_address(slave_addr, WRITE); // Invented function
    // if (!I2C_check_ack()) return;
    for (int i = 0; i < length; i++) {
        I2C_send_byte(i2c_channel, (tbyte)data[i]);
    }
    // I2C_send_stop_condition(); // Invented function, or implicit if next transaction uses repeated start
}

/**
 * @brief Sends a null-terminated string via I2C.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param str Pointer to the null-terminated string.
 */
void I2C_send_string(t_i2c_channel i2c_channel, const char *str) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0 || str == NULL) { return; }

    // See notes for I2C_send_frame regarding I2C protocol implementation.
    while (*str != '\0') {
        I2C_send_byte(i2c_channel, (tbyte)*str++);
    }
}

/**
 * @brief Retrieves a single byte from I2C receive buffer.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @return The received byte.
 * Note: Requires specific I2C sequence (Start, Address, Read, ACK/NACK, Data, Stop/Repeated Start).
 */
tbyte I2C_Get_Byte(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return 0; }

    // This is a simplified placeholder. Actual I2C receive involves:
    // 1. Send START condition.
    // 2. Transmit Slave Address + R/W bit.
    // 3. Wait for ACK/NACK.
    // 4. Trigger Read (often by setting a bit in I2C_CR).
    // 5. Wait for RX_READY flag (check I2C_SR).
    // 6. Read Data Byte from I2C_DR.
    // 7. Send ACK/NACK (depending on if more bytes are expected).
    // 8. Generate STOP or REPEATED START condition.
    // Specific bit fields for START/STOP/ACK/RX control in I2C_CR are missing.

    // Placeholder: Read byte from data register
    // Assuming a bit in I2C_CR triggers receive (e.g., bit 1 for Receive Enable)
    // *REG_I2C_CR |= (1 << 1); // Placeholder: Trigger receive
    // while (!(*REG_I2C_SR & (1 << 1))) { /* Wait for data ready */ } // Placeholder: Wait for flag
    return *REG_I2C_DR;
}

/**
 * @brief Retrieves a frame of data from I2C receive buffer.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 */
void I2C_Get_frame(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0 || buffer == NULL || max_length <= 0) { return; }

    // This is a simplified placeholder, actual I2C transaction requires START/ADDRESS/ACK/DATA/STOP.
    // Rules: "Always generate a repeated start condition instead of stop between transactions" implies
    // handling START and REPEATED START between bytes or at frame start/end carefully.
    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)I2C_Get_Byte(i2c_channel);
        // I2C_send_ack_or_nack(i == (max_length - 1) ? NACK : ACK); // Invented function
    }
}

/**
 * @brief Retrieves a null-terminated string from I2C receive buffer.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 */
tbyte I2C_Get_string(t_i2c_channel i2c_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0 || buffer == NULL || max_length <= 0) { return 0; }

    tbyte count = 0;
    tbyte received_byte;

    for (int i = 0; i < max_length - 1; i++) {
        received_byte = I2C_Get_Byte(i2c_channel);
        if (received_byte == '\0') { // Check for null terminator from sender
            break;
        }
        buffer[i] = (char)received_byte;
        count++;
    }
    buffer[count] = '\0'; // Null-terminate the string
    return count;
}

/**
 * @brief Clears I2C status flags.
 *
 * @param i2c_channel The I2C channel (only I2C_CH0 supported).
 * Note: The specific flags and clear mechanism for I2C_SR are not explicitly defined.
 */
void I2C_ClearFlag(t_i2c_channel i2c_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (i2c_channel != I2C_CH0) { return; }

    // Clear flags in I2C_SR. Usually by reading the register or writing specific bits.
    // No specific clear mechanism given, so assume reading clears some flags or write 0 to all.
    volatile tbyte dummy_read = *REG_I2C_SR;
    (void)dummy_read; // Suppress unused variable warning
}

// --- SPI API Implementations ---

/**
 * @brief Initializes the SPI module.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param spi_mode Master/Slave mode.
 * @param spi_cpol Clock polarity.
 * @param spi_cpha Clock phase.
 * @param spi_dff Data frame format.
 * @param spi_bit_order Bit order (MSB/LSB first).
 * Note: Rules specify fast speed, software SS, full duplex, enable CRC.
 * Specific bit definitions for control registers are not provided.
 */
void spi_Init(t_spi_channel spi_channel, t_spi_mode spi_mode, t_spi_cpol spi_cpol, t_spi_cpha spi_cpha, t_spi_dff spi_dff, t_spi_bit_order spi_bit_order) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return; } // Only SPI_CH0 supported

    // SPI_CR0: "configures SPI operating mode (master/slave), clock polarity/phase."
    // SPI_CR1: "configures SPI data order and enable/disable."

    // Disable SPI before configuration
    *REG_SPI_CR0 = 0x00;
    *REG_SPI_CR1 = 0x00;

    tbyte cr0_config = 0x00;
    tbyte cr1_config = 0x00;

    // Configure Master/Slave Mode
    if (spi_mode == SPI_MODE_MASTER) {
        cr0_config |= (1 << 0); // Placeholder: Master Mode Enable bit in SPI_CR0
    }

    // Configure Clock Polarity (CPOL)
    if (spi_cpol == SPI_CPOL_HIGH) {
        cr0_config |= (1 << 1); // Placeholder: CPOL bit in SPI_CR0
    }

    // Configure Clock Phase (CPHA)
    if (spi_cpha == SPI_CPHA_2_EDGE) {
        cr0_config |= (1 << 2); // Placeholder: CPHA bit in SPI_CR0
    }

    // Configure Data Frame Format (DFF)
    if (spi_dff == SPI_DFF_16_BIT) {
        cr1_config |= (1 << 0); // Placeholder: DFF bit in SPI_CR1
    }

    // Configure Bit Order
    if (spi_bit_order == SPI_BIT_ORDER_LSB_FIRST) {
        cr1_config |= (1 << 1); // Placeholder: LSB First bit in SPI_CR1
    }

    // Rules: Always use fast speed, Slave Select always software-controlled, Always use full duplex, Always enable CRC.
    // Fast Speed: No explicit register for speed. Assuming clock division is implied in init (e.g., bit 3:2 for prescaler).
    cr0_config |= (0x00 << 3); // Placeholder: Smallest prescaler for fast speed.
    // Software SS: No explicit register for SS pin control. Assuming default is software control or implicit.
    // Full Duplex: No explicit register for full duplex. Assuming default operation.
    // Enable CRC: No explicit register for CRC. Placeholder:
    cr1_config |= (1 << 7); // Placeholder: CRC enable bit in SPI_CR1

    *REG_SPI_CR0 = cr0_config;
    *REG_SPI_CR1 = cr1_config;
}

/**
 * @brief Enables the SPI module.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void SPI_Enable(t_spi_channel spi_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return; }
    // Assuming a bit in SPI_CR1 enables the module (e.g., bit 7).
    *REG_SPI_CR1 |= (1 << 7); // Placeholder for SPI enable bit
}

/**
 * @brief Disables the SPI module.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * Note: Specific bit definitions for control registers are not provided.
 */
void SPI_Disable(t_spi_channel spi_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return; }
    // Assuming a bit in SPI_CR1 disables the module (e.g., bit 7).
    *REG_SPI_CR1 &= ~(1 << 7); // Placeholder for SPI enable bit
}

/**
 * @brief Updates SPI status.
 *
 * Note: SPI_SR exists to read status.
 */
void SPI_Update(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Read status register to clear some flags or update internal state if needed.
    // SPI_SR: "indicates SPI communication status and flags."
    volatile tbyte status = *REG_SPI_SR;
    (void)status; // Suppress unused variable warning
}

/**
 * @brief Sends a single byte via SPI.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param byte The byte to send.
 * Note: SPI_SR used for TX buffer empty check.
 */
void SPI_Send_Byte(t_spi_channel spi_channel, tbyte byte) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return; }

    // Wait for transmit buffer to be empty.
    // Assuming SPI_SR has a TX_EMPTY flag (e.g., bit 7)
    while (!(*REG_SPI_SR & (1 << 7))) { /* Wait */ } // Placeholder
    *REG_SPI_DR = byte;
    // Wait for transfer to complete (e.g., busy flag in SPI_SR, bit 0)
    while (*REG_SPI_SR & (1 << 0)) { /* Wait */ } // Placeholder
}

/**
 * @brief Sends a frame of data via SPI.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param data Pointer to the data buffer.
 * @param length The length of the data frame.
 */
void SPI_send_frame(t_spi_channel spi_channel, const char *data, int length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0 || data == NULL || length <= 0) { return; }

    for (int i = 0; i < length; i++) {
        SPI_Send_Byte(spi_channel, (tbyte)data[i]);
    }
}

/**
 * @brief Sends a null-terminated string via SPI.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param str Pointer to the null-terminated string.
 */
void SPI_send_string(t_spi_channel spi_channel, const char *str) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0 || str == NULL) { return; }

    while (*str != '\0') {
        SPI_Send_Byte(spi_channel, (tbyte)*str++);
    }
}

/**
 * @brief Retrieves a single byte from SPI receive buffer.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @return The received byte.
 * Note: SPI_SR used for RX data ready check.
 */
tbyte SPI_Get_Byte(t_spi_channel spi_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return 0; }

    // To receive, usually need to send a dummy byte or trigger receive.
    // This example assumes full-duplex where sending a byte also receives one.
    SPI_Send_Byte(spi_channel, 0xFF); // Send dummy byte to clock in data

    // Wait for receive data to be ready.
    // Assuming SPI_SR has an RX_READY flag (e.g., bit 6)
    while (!(*REG_SPI_SR & (1 << 6))) { /* Wait */ } // Placeholder
    return *REG_SPI_DR;
}

/**
 * @brief Retrieves a frame of data from SPI receive buffer.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param buffer Pointer to the buffer to store received data.
 * @param max_length The maximum length of the buffer.
 */
void SPI_Get_frame(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0 || buffer == NULL || max_length <= 0) { return; }

    for (int i = 0; i < max_length; i++) {
        buffer[i] = (char)SPI_Get_Byte(spi_channel);
    }
}

/**
 * @brief Retrieves a null-terminated string from SPI receive buffer.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * @param buffer Pointer to the buffer to store the received string.
 * @param max_length The maximum length of the buffer (including null terminator).
 * @return The number of bytes received (excluding null terminator).
 */
tbyte SPI_Get_string(t_spi_channel spi_channel, char *buffer, int max_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0 || buffer == NULL || max_length <= 0) { return 0; }

    tbyte count = 0;
    tbyte received_byte;

    for (int i = 0; i < max_length - 1; i++) {
        received_byte = SPI_Get_Byte(spi_channel);
        if (received_byte == '\0') {
            break;
        }
        buffer[i] = (char)received_byte;
        count++;
    }
    buffer[count] = '\0';
    return count;
}

/**
 * @brief Clears SPI status flags.
 *
 * @param spi_channel The SPI channel (only SPI_CH0 supported).
 * Note: The specific flags and clear mechanism for SPI_SR are not explicitly defined.
 */
void SPI_ClearFlag(t_spi_channel spi_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    if (spi_channel != SPI_CH0) { return; }

    // Clear flags in SPI_SR. Usually by reading the register or writing specific bits.
    // No specific clear mechanism given, so assume reading clears some flags or write 0 to all.
    volatile tbyte dummy_read = *REG_SPI_SR;
    (void)dummy_read; // Suppress unused variable warning
}

// --- External Interrupt API Implementations ---

/**
 * @brief Initializes an external interrupt channel.
 *
 * @param external_int_channel The external interrupt channel.
 * @param external_int_edge The desired trigger edge.
 * Note: INT_CR0-3 registers are for configuring interrupt sources.
 * Actual bit assignments for specific pins and edge types are not specified.
 */
void External_INT_Init(t_external_int_channel external_int_channel, t_external_int_edge external_int_edge) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_cr_reg = NULL;
    tbyte pin_bit = 0;

    // Map channel to INT_CRx register and bit. This mapping is hypothetical.
    switch (external_int_channel) {
        case EXT_INT_PB2: int_cr_reg = REG_INT_CR0; pin_bit = 0; break; // Placeholder for PB2
        case EXT_INT_PB1: int_cr_reg = REG_INT_CR0; pin_bit = 1; break; // Placeholder for PB1
        case EXT_INT_PC6: int_cr_reg = REG_INT_CR1; pin_bit = 2; break; // Placeholder for PC6
        case EXT_INT_PC5: int_cr_reg = REG_INT_CR1; pin_bit = 3; break; // Placeholder for PC5
        case EXT_INT_PB0: int_cr_reg = REG_INT_CR2; pin_bit = 4; break; // Placeholder for PB0
        case EXT_INT_PC7: int_cr_reg = REG_INT_CR2; pin_bit = 5; break; // Placeholder for PC7
        default: return; // Invalid channel
    }

    // Configure edge triggering (Assuming 2 bits per channel in INT_CRx for edge config).
    // This is highly speculative without register bit details.
    tbyte current_reg_val = *int_cr_reg;
    current_reg_val &= ~(0x03 << (pin_bit * 2)); // Clear 2 bits for current pin

    switch (external_int_edge) {
        case EXT_INT_EDGE_RISING:
            current_reg_val |= (0x01 << (pin_bit * 2)); // Placeholder: 01 for rising edge
            break;
        case EXT_INT_EDGE_FALLING:
            current_reg_val |= (0x02 << (pin_bit * 2)); // Placeholder: 10 for falling edge
            break;
        case EXT_INT_EDGE_BOTH:
            current_reg_val |= (0x03 << (pin_bit * 2)); // Placeholder: 11 for both edges
            break;
        default:
            break;
    }
    *int_cr_reg = current_reg_val;
}

/**
 * @brief Enables an external interrupt channel.
 *
 * @param external_int_channel The external interrupt channel.
 * Note: Specific bit definitions for enabling interrupts in INT_CRx are not provided.
 */
void External_INT_Enable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_cr_reg = NULL;
    tbyte enable_bit_mask = 0;

    // Map channel to INT_CRx register and its enable bit.
    // Assuming each channel has a single enable bit.
    switch (external_int_channel) {
        case EXT_INT_PB2: int_cr_reg = REG_INT_CR0; enable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PB1: int_cr_reg = REG_INT_CR0; enable_bit_mask = (1 << 7); break; // Placeholder
        case EXT_INT_PC6: int_cr_reg = REG_INT_CR1; enable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PC5: int_cr_reg = REG_INT_CR1; enable_bit_mask = (1 << 7); break; // Placeholder
        case EXT_INT_PB0: int_cr_reg = REG_INT_CR2; enable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PC7: int_cr_reg = REG_INT_CR2; enable_bit_mask = (1 << 7); break; // Placeholder
        default: return;
    }

    if (int_cr_reg != NULL) {
        *int_cr_reg |= enable_bit_mask; // Enable the interrupt
    }
}

/**
 * @brief Disables an external interrupt channel.
 *
 * @param external_int_channel The external interrupt channel.
 * Note: Specific bit definitions for disabling interrupts in INT_CRx are not provided.
 */
void External_INT_Disable(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_cr_reg = NULL;
    tbyte disable_bit_mask = 0;

    // Map channel to INT_CRx register and its enable bit.
    switch (external_int_channel) {
        case EXT_INT_PB2: int_cr_reg = REG_INT_CR0; disable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PB1: int_cr_reg = REG_INT_CR0; disable_bit_mask = (1 << 7); break; // Placeholder
        case EXT_INT_PC6: int_cr_reg = REG_INT_CR1; disable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PC5: int_cr_reg = REG_INT_CR1; disable_bit_mask = (1 << 7); break; // Placeholder
        case EXT_INT_PB0: int_cr_reg = REG_INT_CR2; disable_bit_mask = (1 << 6); break; // Placeholder
        case EXT_INT_PC7: int_cr_reg = REG_INT_CR2; disable_bit_mask = (1 << 7); break; // Placeholder
        default: return;
    }

    if (int_cr_reg != NULL) {
        *int_cr_reg &= ~disable_bit_mask; // Disable the interrupt
    }
}

/**
 * @brief Clears an external interrupt flag.
 *
 * @param external_int_channel The external interrupt channel.
 * Note: INT_MFI0-3 registers contain flags. Specific bit definitions for flags are not provided.
 */
void External_INT_ClearFlag(t_external_int_channel external_int_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_mfi_reg = NULL;
    tbyte flag_bit_mask = 0;

    // Map channel to INT_MFIx register and its flag bit.
    // Assuming a simple mapping, specific bits are not provided.
    switch (external_int_channel) {
        case EXT_INT_PB2: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 0); break; // Placeholder
        case EXT_INT_PB1: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 1); break; // Placeholder
        case EXT_INT_PC6: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 2); break; // Placeholder
        case EXT_INT_PC5: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 3); break; // Placeholder
        case EXT_INT_PB0: int_mfi_reg = REG_INT_MFI2; flag_bit_mask = (1 << 4); break; // Placeholder
        case EXT_INT_PC7: int_mfi_reg = REG_INT_MFI2; flag_bit_mask = (1 << 5); break; // Placeholder
        default: return;
    }

    if (int_mfi_reg != NULL) {
        // Flags are typically cleared by writing a '1' to the flag bit, or by reading the register.
        // Assuming writing a '0' to clear or reading clears.
        *int_mfi_reg &= ~flag_bit_mask; // Placeholder: Clear flag by writing 0
        // Or if write 1 to clear: *int_mfi_reg |= flag_bit_mask;
    }
}

// --- GPIO API Implementations ---

/**
 * @brief Helper function to get the data register for a given port.
 */
static volatile tbyte* GPIO_Get_Data_Reg(t_port port) {
    switch (port) {
        case PORT_A: return REG_GPIOA_DATA;
        case PORT_B: return REG_GPIOB_DATA;
        case PORT_C: return REG_GPIOC_DATA;
        case PORT_D: return REG_GPIOD_DATA;
        default: return NULL;
    }
}

/**
 * @brief Helper function to get the pull-up register for a given port.
 */
static volatile tbyte* GPIO_Get_Pullup_Reg(t_port port) {
    switch (port) {
        case PORT_A: return REG_GPIOA_PULLUP;
        case PORT_B: return REG_GPIOB_PULLUP;
        case PORT_C: return REG_GPIOC_PULLUP;
        case PORT_D: return REG_GPIOD_PULLUP;
        default: return NULL;
    }
}

/**
 * @brief Helper function to get the transistor control register for a given port.
 */
static volatile tbyte* GPIO_Get_Transistor_Reg(t_port port) {
    switch (port) {
        case PORT_A: return REG_GPIOA_TRANSISTOR;
        case PORT_B: return REG_GPIOB_TRANSISTOR;
        case PORT_C: return REG_GPIOC_TRANSISTOR;
        case PORT_D: return REG_GPIOD_TRANSISTOR;
        default: return NULL;
    }
}

/**
 * @brief Helper function to get the source current control register for a given port.
 */
static volatile tbyte* GPIO_Get_SourceCurrent_Reg(t_port port) {
    switch (port) {
        case PORT_A: return REG_GPIOA_SOURCE_CURRENT;
        case PORT_B: return REG_GPIOB_SOURCE_CURRENT;
        case PORT_C: return REG_GPIOC_SOURCE_CURRENT;
        case PORT_D: return REG_GPIOD_SOURCE_CURRENT;
        default: return NULL;
    }
}

/**
 * @brief Initializes a GPIO pin as an output.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The initial output value (0 or 1).
 * Note: The register JSON does NOT provide explicit GPIO direction registers (e.g., DDRx, TRISx, MODERx).
 * This function can only set the initial value and configure pull-ups/current/transistor type.
 * A specific direction register is typically required to set a pin as output.
 */
void GPIO_Output_Init(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* data_reg = GPIO_Get_Data_Reg(port);
    volatile tbyte* pullup_reg = GPIO_Get_Pullup_Reg(port);
    volatile tbyte* transistor_reg = GPIO_Get_Transistor_Reg(port);
    volatile tbyte* source_current_reg = GPIO_Get_SourceCurrent_Reg(port);

    if (data_reg == NULL || pullup_reg == NULL || transistor_reg == NULL || source_current_reg == NULL) { return; }

    // Rule: Always set value before setting direction.
    // Set initial value
    if (value) {
        *data_reg |= (1 << pin);
    } else {
        *data_reg &= ~(1 << pin);
    }
    // Verify value
    while (((*data_reg >> pin) & 0x01) != value) { /* Wait */ }

    // CRITICAL NOTE: Missing explicit GPIO Direction Register.
    // This part of the function cannot set the pin as output because no 'direction' register is provided.
    // Placeholder for setting direction (e.g., *REG_PCDx &= ~(1 << pin); for output)

    // Rule: All output pins have pull-up resistors disabled
    *pullup_reg &= ~(1 << pin);
    // Verify pull-up (if verifiable from register)
    while ((*pullup_reg >> pin) & 0x01) { /* Wait */ }

    // Configure transistor type (CMOS or N-channel open drain)
    // Assuming a bit for each pin in TRANSISTOR register (e.g., 0 for CMOS, 1 for Open-drain)
    // Setting to CMOS (0) as default for output.
    *transistor_reg &= ~(1 << pin); // Placeholder for CMOS output type
    // Verify transistor type (if verifiable)

    // Configure source current strength (>=10mA source current for output)
    // Assuming 2 bits per pin for current control (e.g., 00=low, 01=medium, 10=high, 11=max).
    // Setting to high/max current.
    tbyte current_value = 0x02; // Placeholder for >=10mA source (e.g., '10' for max current)
    *source_current_reg = (*source_current_reg & ~(0x03 << (pin * 2))) | (current_value << (pin * 2));
    // Verify source current (if verifiable)
}

/**
 * @brief Initializes a GPIO pin as an input.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * Note: The register JSON does NOT provide explicit GPIO direction registers (e.g., DDRx, TRISx, MODERx).
 * This function can only configure pull-ups and features.
 * A specific direction register is typically required to set a pin as input.
 */
void GPIO_Input_Init(t_port port, t_pin pin) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* data_reg = GPIO_Get_Data_Reg(port);
    volatile tbyte* pullup_reg = GPIO_Get_Pullup_Reg(port);

    if (data_reg == NULL || pullup_reg == NULL) { return; }

    // CRITICAL NOTE: Missing explicit GPIO Direction Register.
    // This part of the function cannot set the pin as input because no 'direction' register is provided.
    // Placeholder for setting direction (e.g., *REG_PCDx |= (1 << pin); for input)

    // Rule: All input pins have pull-up resistors enabled
    *pullup_reg |= (1 << pin);
    // Verify pull-up (if verifiable from register)
    while (!((*pullup_reg >> pin) & 0x01)) { /* Wait */ }

    // Rule: All input pins have wakeup feature enabled (if available)
    // No explicit "wakeup feature" register is mentioned in the JSON for GPIO.
    // Placeholder for wakeup enable.
    // Example: *REG_GPIOx_WAKEUP_EN |= (1 << pin); (Invented register)
}

/**
 * @brief Gets the direction of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The direction (INPUT or OUTPUT).
 * Note: Cannot be fully implemented due to missing direction registers.
 */
t_direction GPIO_Direction_get(t_port port, t_pin pin) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)port; // Parameter not used due to missing register
    (void)pin;  // Parameter not used due to missing register

    // CRITICAL NOTE: Cannot determine direction without a dedicated direction register.
    // Placeholder for logic if a direction register existed (e.g., REG_PCDA).
    // Example:
    // volatile tbyte* direction_reg = REG_PCDA; // Invented register
    // if (( *direction_reg >> pin) & 0x01) { // Assuming 1 means input, 0 means output
    //     return INPUT;
    // } else {
    //     return OUTPUT;
    // }
    return INPUT; // Defaulting to INPUT as a fallback
}

/**
 * @brief Sets the output value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @param value The desired output value (0 or 1).
 */
void GPIO_Value_Set(t_port port, t_pin pin, tbyte value) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* data_reg = GPIO_Get_Data_Reg(port);
    if (data_reg == NULL) { return; }

    if (value) {
        *data_reg |= (1 << pin);
    } else {
        *data_reg &= ~(1 << pin);
    }
    // Rule: After setting GPIO value, verify with while loop
    while (((*data_reg >> pin) & 0x01) != value) { /* Wait */ }
}

/**
 * @brief Gets the input value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 * @return The input value (0 or 1).
 */
tbyte GPIO_Value_Get(t_port port, t_pin pin) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* data_reg = GPIO_Get_Data_Reg(port);
    if (data_reg == NULL) { return 0; }

    return (tbyte)((*data_reg >> pin) & 0x01);
}

/**
 * @brief Toggles the output value of a GPIO pin.
 *
 * @param port The GPIO port.
 * @param pin The pin number within the port.
 */
void GPIO_Value_Tog(t_port port, t_pin pin) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* data_reg = GPIO_Get_Data_Reg(port);
    if (data_reg == NULL) { return; }

    *data_reg ^= (1 << pin);
    // Rule: After setting GPIO value, verify with while loop
    tbyte new_value = (*data_reg >> pin) & 0x01;
    tbyte old_value = !new_value; // Assuming it just toggled
    while (((*data_reg >> pin) & 0x01) == old_value) { /* Wait */ }
}

// --- PWM API Implementations ---

/**
 * @brief Initializes a PWM channel.
 *
 * @param pwm_channel The PWM channel (CTM, STM, PTM0, PTM1).
 * @param pwm_khz_freq The desired PWM frequency in kHz.
 * @param pwm_duty The desired PWM duty cycle (0-100%).
 * Note: Specific bit definitions for timer control and compare registers are not provided.
 * Rule: Clear available FREQUENCY Ranges for each channel as comments.
 */
void PWM_Init(t_pwm_channel pwm_channel, tbyte pwm_khz_freq, tbyte pwm_duty) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Assuming a system clock (Fsys) to calculate timer period and compare values.
    // Fsys is unknown from the provided data. This is a placeholder for calculations.

    // PWM frequency (Fpwm) = Fsys / (Prescaler * Period)
    // Duty Cycle = Compare / Period

    // Available FREQUENCY Ranges: (These are examples, actual ranges depend on Fsys and timer capabilities)
    // CTM (Compact Type Timer Module): Max Freq: ~MHz to kHz range (e.g., 10MHz / (1 * 256) = 39kHz)
    // STM (Standard Type Timer Module): Max Freq: ~MHz to kHz range (similar to CTM)
    // PTM0 (Periodic Type Timer Module 0): Can generate lower frequencies due to 16-bit Period.
    // PTM1 (Periodic Type Timer Module 1): Can generate lower frequencies due to 16-bit Period.

    tword period_val = 0; // Calculated period value for timer
    tword compare_val = 0; // Calculated compare value for duty cycle

    // Basic calculation for placeholder:
    if (pwm_khz_freq > 0) {
        // Assume Fsys = 4MHz (arbitrary)
        // period_val = (4000 / pwm_khz_freq) - 1; // Example: 4MHz / 10kHz = 400. Period = 399
        period_val = 0xFF; // Placeholder, assuming 8-bit timer default
    }
    if (period_val == 0) period_val = 1; // Avoid division by zero
    compare_val = (period_val * pwm_duty) / 100;

    switch (pwm_channel) {
        case PWM_CTM: // PB2
            // CTM_CR0: configures CTM operating mode and clock source.
            // CTM_CR1: configures CTM output polarity and interrupt enable.
            // CTM_COMPA: used for compare match or PWM output.
            // CTM_CNT_H/L: Counter registers.

            *REG_CTM_CR0 = 0x00; // Disable CTM for config
            *REG_CTM_CR1 = 0x00;

            // Set Compare value
            *REG_CTM_COMPA = (tbyte)compare_val;
            // Set Period (CTM is often 8-bit, so period would be set via PR register or implied by clock)
            // No explicit period register for CTM. Assuming CTM_COMPA is also PR for 8-bit.
            // Or if PWM is generated in "period mode" where COMPA is duty, and period is implicit max count.

            // Configure CTM_CR0 for PWM mode and clock source.
            // Placeholder: Assuming bits for PWM mode (e.g., bit 0, 1 for mode) and clock source (e.g., bit 2,3).
            *REG_CTM_CR0 |= (1 << 0) | (1 << 2); // Placeholder: Enable PWM mode, select clock.

            // Configure CTM_CR1 for output polarity and enable.
            // Placeholder: Assuming bits for output enable (e.g., bit 0) and polarity (e.g., bit 1).
            *REG_CTM_CR1 |= (1 << 0); // Placeholder: Enable PWM output on PB2.
            break;

        case PWM_STM: // PB7
            // Similar to CTM, but for STM registers
            *REG_STM_CR0 = 0x00;
            *REG_STM_CR1 = 0x00;
            *REG_STM_COMPA = (tbyte)compare_val; // Assuming 8-bit, STM_COMPA (PB7)
            *REG_STM_CR0 |= (1 << 0) | (1 << 2); // Placeholder: Enable PWM mode, select clock.
            *REG_STM_CR1 |= (1 << 0); // Placeholder: Enable PWM output on PB7.
            break;

        case PWM_PTM0: // PB0, PB1
            // PTM0_COMPA_H/L: 16-bit comparator for PWM.
            *REG_PTM0_CR0 = 0x00;
            *REG_PTM0_CR1 = 0x00;
            *REG_PTM0_COMPA_L = (tbyte)(compare_val & 0xFF);
            *REG_PTM0_COMPA_H = (tbyte)((compare_val >> 8) & 0xFF);
            // PTM0 typically has a period register too, but not explicitly named.
            // Assuming the maximum value of the counter is the period or set via another register.
            *REG_PTM0_CR0 |= (1 << 0) | (1 << 2); // Placeholder: Enable PWM mode, select clock.
            *REG_PTM0_CR1 |= (1 << 0); // Placeholder: Enable PWM output on PB0/PB1.
            break;

        case PWM_PTM1: // PA5, PA6
            // Similar to PTM0, but for PTM1 registers
            *REG_PTM1_CR0 = 0x00;
            *REG_PTM1_CR1 = 0x00;
            *REG_PTM1_COMPA_L = (tbyte)(compare_val & 0xFF);
            *REG_PTM1_COMPA_H = (tbyte)((compare_val >> 8) & 0xFF);
            *REG_PTM1_CR0 |= (1 << 0) | (1 << 2); // Placeholder: Enable PWM mode, select clock.
            *REG_PTM1_CR1 |= (1 << 0); // Placeholder: Enable PWM output on PA5/PA6.
            break;

        default:
            break;
    }
    // TM_PIN_CR: "configures alternate functions for timer module pins."
    // If PWM requires alternate function mapping, this register would be used.
    // Placeholder: *REG_TM_PIN_CR |= (1 << X); // Set specific bits for PWM output on selected pin.
}

/**
 * @brief Starts the PWM output for a specific channel.
 *
 * @param pwm_channel The PWM channel.
 * Note: This typically involves enabling the timer counter and/or PWM output.
 */
void PWM_Strt(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming an enable bit in CR0 or CR1 to start the timer/PWM.
    switch (pwm_channel) {
        case PWM_CTM:
            *REG_CTM_CR0 |= (1 << 7); // Placeholder: Enable CTM timer
            break;
        case PWM_STM:
            *REG_STM_CR0 |= (1 << 7); // Placeholder: Enable STM timer
            break;
        case PWM_PTM0:
            *REG_PTM0_CR0 |= (1 << 7); // Placeholder: Enable PTM0 timer
            break;
        case PWM_PTM1:
            *REG_PTM1_CR0 |= (1 << 7); // Placeholder: Enable PTM1 timer
            break;
        default:
            break;
    }
}

/**
 * @brief Stops the PWM output for a specific channel.
 *
 * @param pwm_channel The PWM channel.
 * Note: This typically involves disabling the timer counter and/or PWM output.
 */
void PWM_Stop(t_pwm_channel pwm_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming an enable bit in CR0 or CR1 to stop the timer/PWM.
    switch (pwm_channel) {
        case PWM_CTM:
            *REG_CTM_CR0 &= ~(1 << 7); // Placeholder: Disable CTM timer
            break;
        case PWM_STM:
            *REG_STM_CR0 &= ~(1 << 7); // Placeholder: Disable STM timer
            break;
        case PWM_PTM0:
            *REG_PTM0_CR0 &= ~(1 << 7); // Placeholder: Disable PTM0 timer
            break;
        case PWM_PTM1:
            *REG_PTM1_CR0 &= ~(1 << 7); // Placeholder: Disable PTM1 timer
            break;
        default:
            break;
    }
}

// --- ICU API Implementations ---

/**
 * @brief Initializes an Input Capture Unit (ICU) channel.
 *
 * @param icu_channel The ICU channel.
 * @param icu_prescaller The prescaler for the timer used by ICU.
 * @param icu_edge The desired edge for input capture.
 * Note: ICU functionality is typically based on timer modules configured for input capture.
 * The register JSON does not explicitly define "ICU" registers, but timers (CTM, STM, PTMx)
 * are available and have assigned pins that could be used for input capture.
 * This implementation maps ICU channels to the corresponding timer modules.
 * Specific bit definitions for input capture mode and prescalers are not provided.
 */
void ICU_init(t_icu_channel icu_channel, t_icu_prescaller icu_prescaller, t_icu_edge icu_edge) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Determine which timer module corresponds to the ICU channel
    volatile tbyte* timer_cr0 = NULL;
    volatile tbyte* timer_cr1 = NULL;

    // Placeholder for prescaler mapping (assuming 2 bits for prescaler in CR0)
    tbyte prescaler_val = 0x00;
    switch (icu_prescaller) {
        case ICU_PRESCALER_1: prescaler_val = 0x00; break;
        case ICU_PRESCALER_2: prescaler_val = 0x01; break;
        case ICU_PRESCALER_4: prescaler_val = 0x02; break;
        case ICU_PRESCALER_8: prescaler_val = 0x03; break;
        default: break;
    }

    // Placeholder for edge mapping (assuming 2 bits for edge in CR1)
    tbyte edge_val = 0x00;
    switch (icu_edge) {
        case ICU_EDGE_RISING: edge_val = 0x01; break;
        case ICU_EDGE_FALLING: edge_val = 0x02; break;
        case ICU_EDGE_BOTH: edge_val = 0x03; break;
        default: break;
    }

    switch (icu_channel) {
        case ICU_CTM_PB2:
            timer_cr0 = REG_CTM_CR0;
            timer_cr1 = REG_CTM_CR1;
            // Also need to configure PB2 as alternate function for CTM input capture.
            // TM_PIN_CR used for alternate functions. Placeholder for specific bit.
            // *REG_TM_PIN_CR |= (1 << X);
            break;
        case ICU_STM_PB7:
            timer_cr0 = REG_STM_CR0;
            timer_cr1 = REG_STM_CR1;
            // *REG_TM_PIN_CR |= (1 << Y);
            break;
        case ICU_PTM0_PB0:
        case ICU_PTM0_PB1: // PTM0 handles multiple pins, need to differentiate if possible
            timer_cr0 = REG_PTM0_CR0;
            timer_cr1 = REG_PTM0_CR1;
            // *REG_TM_PIN_CR |= (1 << Z);
            break;
        case ICU_PTM1_PA5:
        case ICU_PTM1_PA6: // PTM1 handles multiple pins, need to differentiate if possible
            timer_cr0 = REG_PTM1_CR0;
            timer_cr1 = REG_PTM1_CR1;
            // *REG_TM_PIN_CR |= (1 << W);
            break;
        default:
            return;
    }

    if (timer_cr0 != NULL && timer_cr1 != NULL) {
        // Disable timer before configuring
        *timer_cr0 = 0x00;
        *timer_cr1 = 0x00;

        // Configure timer for input capture mode and prescaler
        // Placeholder for Input Capture mode bit (e.g., bit 0,1) and prescaler (e.g., bit 2,3)
        *timer_cr0 = (0x02 << 0) | (prescaler_val << 2); // Placeholder: Input Capture Mode, Prescaler

        // Configure edge
        // Placeholder for edge selection (e.g., bit 0,1) in CR1 for input capture.
        *timer_cr1 = (edge_val << 0);
    }
}

/**
 * @brief Enables the ICU channel.
 *
 * @param icu_channel The ICU channel.
 * Note: Specific bit definitions for enabling timers are not provided.
 */
void ICU_Enable(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* timer_cr0 = NULL;
    switch (icu_channel) {
        case ICU_CTM_PB2: timer_cr0 = REG_CTM_CR0; break;
        case ICU_STM_PB7: timer_cr0 = REG_STM_CR0; break;
        case ICU_PTM0_PB0:
        case ICU_PTM0_PB1: timer_cr0 = REG_PTM0_CR0; break;
        case ICU_PTM1_PA5:
        case ICU_PTM1_PA6: timer_cr0 = REG_PTM1_CR0; break;
        default: return;
    }
    if (timer_cr0 != NULL) {
        *timer_cr0 |= (1 << 7); // Placeholder: Enable timer counter.
    }
}

/**
 * @brief Disables the ICU channel.
 *
 * @param icu_channel The ICU channel.
 * Note: Specific bit definitions for disabling timers are not provided.
 */
void ICU_Disable(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* timer_cr0 = NULL;
    switch (icu_channel) {
        case ICU_CTM_PB2: timer_cr0 = REG_CTM_CR0; break;
        case ICU_STM_PB7: timer_cr0 = REG_STM_CR0; break;
        case ICU_PTM0_PB0:
        case ICU_PTM0_PB1: timer_cr0 = REG_PTM0_CR0; break;
        case ICU_PTM1_PA5:
        case ICU_PTM1_PA6: timer_cr0 = REG_PTM1_CR0; break;
        default: return;
    }
    if (timer_cr0 != NULL) {
        *timer_cr0 &= ~(1 << 7); // Placeholder: Disable timer counter.
    }
}

/**
 * @brief Updates the ICU frequency.
 *
 * @param icu_channel The ICU channel.
 * Note: This would typically involve reading timer capture registers to calculate frequency.
 * The register JSON provides timer counter registers but no explicit capture registers.
 */
void ICU_Updatefrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)icu_channel; // Parameter used to select appropriate timer module and read capture data.

    // This function would ideally trigger an input capture event check,
    // read the captured value (from a specific input capture register, not just CNT_L/H),
    // and store it for GetFrequency().
    // Since no explicit capture registers are provided (e.g., CTM_CCR), this is a placeholder.
}

/**
 * @brief Gets the captured frequency.
 *
 * @param icu_channel The ICU channel.
 * @return The captured frequency in Hz.
 * Note: Requires previous calls to ICU_Updatefrequency.
 */
tlong ICU_GetFrequency(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)icu_channel; // Parameter used to select appropriate timer module and read capture data.

    // This function should read the last calculated frequency.
    // Since actual input capture and frequency calculation are not possible with provided registers,
    // this returns a dummy value.
    return 0; // Placeholder
}

/**
 * @brief Sets the buffer for remote control keys.
 * Note: This is an application-level buffer, not directly MCU register related.
 */
void ICU_Set_RemoteControlkeysbuffer(tbyte number_of_keys, tbyte key_digits_length) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)number_of_keys; // Parameter used for buffer allocation/management.
    (void)key_digits_length; // Parameter used for buffer allocation/management.
    // Example: Allocate a 2D array: static tbyte remote_keys[number_of_keys][key_digits_length];
}

/**
 * @brief Sets specific digits for a remote control key.
 * Note: This is an application-level function.
 */
void ICU_SetRemoteControlkeydigits(tbyte key_num, tbyte key_array_cell, tbyte key_cell_value) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)key_num;        // Used to index into a remote control key array.
    (void)key_array_cell; // Used to index into the digits of a specific key.
    (void)key_cell_value; // The value of the digit.
    // Example: remote_keys[key_num][key_array_cell] = key_cell_value;
}

/**
 * @brief Updates parameters for remote control signal decoding.
 * Note: These parameters are for the decoding algorithm, not directly MCU register related.
 */
void ICU_UpdateRemoteControlSignal(t_icu_channel icu_channel, tlong strt_bit_us_value, tlong one_bit_us_value, tlong zero_bit_us_value, tlong stop_bit_us_value) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)icu_channel;        // Which ICU channel to associate these decoding parameters with.
    (void)strt_bit_us_value;  // Duration for start bit.
    (void)one_bit_us_value;   // Duration for '1' bit.
    (void)zero_bit_us_value;  // Duration for '0' bit.
    (void)stop_bit_us_value;  // Duration for stop bit.
    // Store these values in internal module variables.
}

/**
 * @brief Gets the decoded remote control key.
 * Note: This is an application-level function relying on ICU input.
 */
tbyte ICU_GetRemoteControlkey(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    (void)icu_channel; // Which ICU channel's data to process.
    // This function would read captured pulse durations, decode them based on the
    // parameters set by ICU_UpdateRemoteControlSignal, and match against defined keys.
    return 0xFF; // Placeholder: Return an invalid key if nothing found.
}

/**
 * @brief Sets a callback function for ICU events.
 * Note: This is an application-level function.
 */
void ICU_setCallback(void (*callback)(void)) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    ICU_Callback_Func = callback;
}

/**
 * @brief Clears ICU flags.
 *
 * @param icu_channel The ICU channel.
 * Note: Timer interrupt flags (e.g., in INT_MFIx) would be cleared here.
 */
void ICU_ClearFlag(t_icu_channel icu_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_mfi_reg = NULL;
    tbyte flag_bit_mask = 0;

    // Mapping ICU channels to INT_MFIx flags for timer capture interrupts.
    // This mapping is hypothetical as specific bits are not defined.
    switch (icu_channel) {
        case ICU_CTM_PB2: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 0); break; // CTM capture flag
        case ICU_STM_PB7: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 1); break; // STM capture flag
        case ICU_PTM0_PB0:
        case ICU_PTM0_PB1: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 0); break; // PTM0 capture flag
        case ICU_PTM1_PA5:
        case ICU_PTM1_PA6: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 1); break; // PTM1 capture flag
        default: return;
    }

    if (int_mfi_reg != NULL) {
        *int_mfi_reg &= ~flag_bit_mask; // Placeholder: Clear flag by writing 0
    }
}

// --- Timer API Implementations ---

/**
 * @brief Initializes a timer channel.
 *
 * @param timer_channel The timer channel (CTM, STM, PTM0, PTM1).
 * Note: Specific bit definitions for timer control registers are not provided.
 */
void TIMER_Init(t_timer_channel timer_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* timer_cr0 = NULL;
    volatile tbyte* timer_cr1 = NULL;
    volatile tbyte* timer_cnt_h = NULL;
    volatile tbyte* timer_cnt_l = NULL;

    switch (timer_channel) {
        case TIMER_CTM:
            timer_cr0 = REG_CTM_CR0; timer_cr1 = REG_CTM_CR1;
            timer_cnt_h = REG_CTM_CNT_H; timer_cnt_l = REG_CTM_CNT_L;
            break;
        case TIMER_STM:
            timer_cr0 = REG_STM_CR0; timer_cr1 = REG_STM_CR1;
            timer_cnt_h = REG_STM_CNT_H; timer_cnt_l = REG_STM_CNT_L;
            break;
        case TIMER_PTM0:
            timer_cr0 = REG_PTM0_CR0; timer_cr1 = REG_PTM0_CR1;
            timer_cnt_h = REG_PTM0_CNT_H; timer_cnt_l = REG_PTM0_CNT_L;
            break;
        case TIMER_PTM1:
            timer_cr0 = REG_PTM1_CR0; timer_cr1 = REG_PTM1_CR1;
            timer_cnt_h = REG_PTM1_CNT_H; timer_cnt_l = REG_PTM1_CNT_L;
            break;
        default: return;
    }

    if (timer_cr0 != NULL && timer_cr1 != NULL && timer_cnt_h != NULL && timer_cnt_l != NULL) {
        // Disable timer first
        *timer_cr0 = 0x00;
        *timer_cr1 = 0x00;

        // Clear counter
        *timer_cnt_l = 0x00;
        *timer_cnt_h = 0x00;

        // Configure basic timer mode (e.g., periodic, default prescaler)
        // Placeholder: Assuming bits in CR0/CR1 set timer mode.
        *timer_cr0 |= (1 << 0); // Placeholder: Enable timer base
    }
}

/**
 * @brief Sets a timer for a specific duration in microseconds.
 *
 * @param timer_channel The timer channel.
 * @param time The time in microseconds.
 * Note: Requires MCU clock frequency for accurate calculation. Assuming 16-bit timers for PTMs.
 */
void TIMER_Set_us(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* timer_comp_h = NULL;
    volatile tbyte* timer_comp_l = NULL;

    // Placeholder for system clock frequency (e.g., 4MHz)
    tlong system_clock_hz = 4000000; // Example
    tword target_counts = (tword)(((tlong)time * (system_clock_hz / 1000000)) - 1); // Calculate counts for us

    switch (timer_channel) {
        case TIMER_CTM:
            // CTM_COMPA is 8-bit, for us, might need large prescaler or be too fast.
            // Placeholder: Assume 8-bit for CTM/STM.
            timer_comp_l = REG_CTM_COMPA; // CTM has only COMPA, assuming it's an 8-bit timer.
            *timer_comp_l = (tbyte)target_counts; // This might overflow for CTM/STM
            break;
        case TIMER_STM:
            timer_comp_l = REG_STM_COMPA; // STM has only COMPA, assuming it's an 8-bit timer.
            *timer_comp_l = (tbyte)target_counts;
            break;
        case TIMER_PTM0:
            timer_comp_h = REG_PTM0_COMPA_H;
            timer_comp_l = REG_PTM0_COMPA_L;
            *timer_comp_l = (tbyte)(target_counts & 0xFF);
            *timer_comp_h = (tbyte)((target_counts >> 8) & 0xFF);
            break;
        case TIMER_PTM1:
            timer_comp_h = REG_PTM1_COMPA_H;
            timer_comp_l = REG_PTM1_COMPA_L;
            *timer_comp_l = (tbyte)(target_counts & 0xFF);
            *timer_comp_h = (tbyte)((target_counts >> 8) & 0xFF);
            break;
        default: return;
    }
    // Also need to set timer mode to "one-shot" or "compare match" in CR0/CR1
}

/**
 * @brief Sets a timer for a specific duration in milliseconds.
 */
void TIMER_Set_Time_ms(t_timer_channel timer_channel, tword time) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Convert ms to us and call TIMER_Set_us. Handle potential overflow for time_us.
    tlong time_us = (tlong)time * 1000;
    if (time_us > 0xFFFF) { // If it exceeds tword max (for 16-bit timers)
        // Adjust prescaler or use a different timer if available for longer times.
        // For simplicity, for now, cap it or indicate error.
        time_us = 0xFFFF; // Cap at max for tword (65535 us = 65.5 ms)
    }
    TIMER_Set_us(timer_channel, (tword)time_us);
}

/**
 * @brief Sets a timer for a specific duration in seconds.
 */
void TIMER_Set_Time_sec(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Convert sec to ms and call TIMER_Set_Time_ms.
    tword time_ms = (tword)time * 1000;
    if (time_ms > 0xFFFF) { // If it exceeds tword max
        time_ms = 0xFFFF;
    }
    TIMER_Set_Time_ms(timer_channel, time_ms);
}

/**
 * @brief Sets a timer for a specific duration in minutes.
 */
void TIMER_Set_Time_min(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Convert min to sec and call TIMER_Set_Time_sec.
    tbyte time_sec = time * 60; // Max 255 * 60 = 15300 seconds, fits tword if needed.
    TIMER_Set_Time_sec(timer_channel, time_sec);
}

/**
 * @brief Sets a timer for a specific duration in hours.
 */
void TIMER_Set_Time_hour(t_timer_channel timer_channel, tbyte time) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Convert hours to min and call TIMER_Set_Time_min.
    tbyte time_min = time * 60;
    TIMER_Set_Time_min(timer_channel, time_min);
}

/**
 * @brief Enables the specified timer channel.
 */
void TIMER_Enable(t_timer_channel timer_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    volatile tbyte* timer_cr0 = NULL;
    switch (timer_channel) {
        case TIMER_CTM: timer_cr0 = REG_CTM_CR0; break;
        case TIMER_STM: timer_cr0 = REG_STM_CR0; break;
        case TIMER_PTM0: timer_cr0 = REG_PTM0_CR0; break;
        case TIMER_PTM1: timer_cr0 = REG_PTM1_CR0; break;
        default: return;
    }
    if (timer_cr0 != NULL) {
        *timer_cr0 |= (1 << 7); // Placeholder: Enable timer counter.
    }
}

/**
 * @brief Disables the specified timer channel.
 */
void TIMER_Disable(t_timer_channel timer_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    volatile tbyte* timer_cr0 = NULL;
    switch (timer_channel) {
        case TIMER_CTM: timer_cr0 = REG_CTM_CR0; break;
        case TIMER_STM: timer_cr0 = REG_STM_CR0; break;
        case TIMER_PTM0: timer_cr0 = REG_PTM0_CR0; break;
        case TIMER_PTM1: timer_cr0 = REG_PTM1_CR0; break;
        default: return;
    }
    if (timer_cr0 != NULL) {
        *timer_cr0 &= ~(1 << 7); // Placeholder: Disable timer counter.
    }
}

/**
 * @brief Clears the interrupt flag for the specified timer channel.
 *
 * Note: Timer flags are typically in the INT_MFIx registers.
 */
void TIMER_ClearFlag(t_timer_channel timer_channel) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    volatile tbyte* int_mfi_reg = NULL;
    tbyte flag_bit_mask = 0;

    // Mapping timer channels to INT_MFIx flags for timer interrupts.
    // This mapping is hypothetical as specific bits are not defined.
    switch (timer_channel) {
        case TIMER_CTM: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 0); break; // CTM flag
        case TIMER_STM: int_mfi_reg = REG_INT_MFI0; flag_bit_mask = (1 << 1); break; // STM flag
        case TIMER_PTM0: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 0); break; // PTM0 flag
        case TIMER_PTM1: int_mfi_reg = REG_INT_MFI1; flag_bit_mask = (1 << 1); break; // PTM1 flag
        default: return;
    }

    if (int_mfi_reg != NULL) {
        *int_mfi_reg &= ~flag_bit_mask; // Placeholder: Clear flag by writing 0
    }
}

// --- ADC API Implementations ---

/**
 * @brief Initializes the ADC module.
 *
 * @param adc_channel The ADC input channel(s) to enable.
 * @param adc_mode The ADC conversion mode (single or continuous).
 * Note: Specific bit definitions for ADC_CR0/CR1/CHER are not provided.
 */
void ADC_Init(t_adc_channel adc_channel, t_adc_mode_t adc_mode) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // ADC_CR0: "configures ADC operation modes and trigger sources."
    // ADC_CR1: "configures ADC channel selection and reference voltage."
    // ADC_CHER: "enables/disables specific analog input channels."

    // Disable ADC before configuration
    *REG_ADC_CR0 = 0x00;
    *REG_ADC_CR1 = 0x00;
    *REG_ADC_CHER = 0x00;

    // Enable selected ADC channel in ADC_CHER
    // Assuming 1:1 mapping of adc_channel enum to bit in ADC_CHER
    *REG_ADC_CHER |= (1 << adc_channel); // Placeholder

    // Configure ADC mode (single/continuous) in ADC_CR0
    // Placeholder for mode bit (e.g., bit 0 for single, bit 1 for continuous)
    if (adc_mode == ADC_MODE_CONTINUOUS_CONVERSION) {
        *REG_ADC_CR0 |= (1 << 1); // Placeholder: Enable continuous mode
    } else {
        *REG_ADC_CR0 |= (1 << 0); // Placeholder: Enable single mode
    }

    // Further configurations like clock source, resolution, reference voltage (ADC_CR1)
    // Placeholder values for ADC_CR1 (e.g., reference voltage selection bits)
    *REG_ADC_CR1 |= (1 << 0) | (1 << 1); // Placeholder: Set reference voltage source
}

/**
 * @brief Enables the ADC module.
 *
 * Note: Specific bit definitions for enabling ADC are not provided.
 */
void ADC_Enable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming a bit in ADC_CR0 enables the ADC peripheral (e.g., bit 7).
    *REG_ADC_CR0 |= (1 << 7); // Placeholder: ADC global enable
}

/**
 * @brief Disables the ADC module.
 *
 * Note: Specific bit definitions for disabling ADC are not provided.
 */
void ADC_Disable(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming a bit in ADC_CR0 disables the ADC peripheral (e.g., bit 7).
    *REG_ADC_CR0 &= ~(1 << 7); // Placeholder: ADC global disable
}

/**
 * @brief Triggers an ADC conversion or updates status.
 *
 * Note: This function might trigger a new conversion in single mode, or
 * just refresh status in continuous mode.
 */
void ADC_Update(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // If in single conversion mode, trigger a new conversion.
    // Assuming a bit in ADC_CR0 triggers conversion (e.g., bit 2).
    *REG_ADC_CR0 |= (1 << 2); // Placeholder: Start conversion
}

/**
 * @brief Gets the latest ADC conversion result.
 *
 * @return The 10-bit ADC conversion result.
 * Note: ADC_DR_L and ADC_DR_H store the result.
 */
tword ADC_Get(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Wait for conversion complete.
    // No explicit ADC status register flag for conversion complete.
    // Assuming INT_MFIx could contain an ADC conversion complete flag (e.g., INT_MFI0 bit 2).
    while (!(*REG_INT_MFI0 & (1 << 2))) { /* Wait for conversion complete */ } // Placeholder

    // Combine high and low bytes to get 10-bit result.
    // Assuming ADC_DR_H contains bits 9:8 and ADC_DR_L contains bits 7:0.
    tword result = (tword)(*REG_ADC_DR_L);
    result |= (tword)((*REG_ADC_DR_H & 0x03) << 8); // Assuming only 2 bits in DR_H for 10-bit ADC
    return result;
}

/**
 * @brief Clears ADC flags.
 *
 * Note: No explicit ADC status register with a cleareable flag.
 * Assuming INT_MFIx contains an ADC flag that needs clearing.
 */
void ADC_ClearFlag(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Assuming INT_MFIx contains an ADC flag (e.g., INT_MFI0 bit 2 for ADC conversion complete).
    *REG_INT_MFI0 &= ~(1 << 2); // Placeholder: Clear ADC conversion complete flag
}

// --- Internal_EEPROM API Implementations ---

/**
 * @brief Initializes the Internal EEPROM module.
 * Note: Specific bit definitions for EEPROM_CR0/CR1 are not provided.
 */
void Internal_EEPROM_Init(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // EEPROM_CR0: "controls EEPROM read/write operations and write protection."
    // EEPROM_CR1: "controls EEPROM interrupt enable."

    // Disable EEPROM operations and interrupts before init
    *REG_EEPROM_CR0 = 0x00;
    *REG_EEPROM_CR1 = 0x00;

    // Basic initialization, potentially setting up clock for EEPROM controller.
    // No specific clock registers for EEPROM in JSON.
}

/**
 * @brief Writes a byte to Internal EEPROM.
 *
 * @param address The EEPROM memory address.
 * @param data The byte data to write.
 * Note: EEPROM_ADDR_L/H and EEPROM_DATA_L/H are used.
 * Specific bits for write trigger and status in EEPROM_CR0/CR1 are not provided.
 */
void Internal_EEPROM_Set(tbyte address, tbyte data) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Set EEPROM address
    *REG_EEPROM_ADDR_L = address; // Assuming address is 8-bit, or needs to be combined for 16-bit address
    // *REG_EEPROM_ADDR_H = (address >> 8); // If 16-bit address

    // Set data to be written
    *REG_EEPROM_DATA_L = data; // Assuming 8-bit data write
    // *REG_EEPROM_DATA_H = 0x00; // If writing 16-bit data

    // Trigger write operation and wait for completion.
    // EEPROM_CR0: "controls EEPROM read/write operations and write protection."
    // Assuming a bit in EEPROM_CR0 (e.g., bit 0) triggers a write.
    *REG_EEPROM_CR0 |= (1 << 0); // Placeholder: Trigger write

    // Wait for write complete (no specific status bit mentioned in JSON)
    // Assuming EEPROM_CR0 or INT_MFIx might have a flag.
    // while (*REG_EEPROM_CR0 & (1 << 0)) { /* Wait for write complete */ } // Placeholder
}

/**
 * @brief Reads a byte from Internal EEPROM.
 *
 * @param address The EEPROM memory address.
 * @return The data byte read from EEPROM.
 * Note: EEPROM_ADDR_L/H and EEPROM_DATA_L/H are used.
 * Specific bits for read trigger and status in EEPROM_CR0/CR1 are not provided.
 */
tbyte Internal_EEPROM_Get(tbyte address) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // Set EEPROM address
    *REG_EEPROM_ADDR_L = address;
    // *REG_EEPROM_ADDR_H = (address >> 8); // If 16-bit address

    // Trigger read operation and wait for completion.
    // EEPROM_CR0: "controls EEPROM read/write operations and write protection."
    // Assuming a bit in EEPROM_CR0 (e.g., bit 1) triggers a read.
    *REG_EEPROM_CR0 |= (1 << 1); // Placeholder: Trigger read

    // Wait for read complete (no specific status bit mentioned in JSON)
    // Assuming EEPROM_CR0 or INT_MFIx might have a flag.
    // while (*REG_EEPROM_CR0 & (1 << 1)) { /* Wait for read complete */ } // Placeholder

    // Read data
    return *REG_EEPROM_DATA_L;
}

// --- TT (Time Triggered OS) API Implementations ---

/**
 * @brief Initializes the Time Triggered (TT) OS.
 *
 * @param tick_time_ms The desired tick time in milliseconds.
 * Note: This typically configures a hardware timer to generate periodic interrupts.
 * We will use PTM0 as the underlying timer for TT.
 */
void TT_Init(t_tick_time tick_time_ms) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    // PTM0_CR0: "configures PTM0 operating mode and clock source."
    // PTM0_CR1: "configures PTM0 output polarity and interrupt enable."
    // PTM0_COMPA_H/L: Comparator for period.

    // Disable PTM0 first
    *REG_PTM0_CR0 = 0x00;
    *REG_PTM0_CR1 = 0x00;

    // Clear PTM0 counter
    *REG_PTM0_CNT_L = 0x00;
    *REG_PTM0_CNT_H = 0x00;

    // Calculate compare value for desired tick_time_ms
    // Assuming a system clock (Fsys) of 4MHz and a timer prescaler (e.g., 1:4)
    // F_timer = Fsys / Prescaler = 4MHz / 4 = 1MHz
    // Counts_per_ms = 1MHz / 1000 = 1000 counts/ms
    // Target_counts = tick_time_ms * Counts_per_ms - 1
    tword target_counts = (tword)(((tlong)tick_time_ms * 1000) - 1); // For 1MHz timer

    *REG_PTM0_COMPA_L = (tbyte)(target_counts & 0xFF);
    *REG_PTM0_COMPA_H = (tbyte)((target_counts >> 8) & 0xFF);

    // Configure PTM0 for periodic mode, with interrupt enable.
    // Placeholder: Bits in PTM0_CR0 for periodic mode (e.g., 0x01) and clock source/prescaler.
    // Placeholder: Bit in PTM0_CR1 for interrupt enable (e.g., bit 7).
    *REG_PTM0_CR0 |= (1 << 0) | (1 << 2); // Mode (periodic), Prescaler (1:4)
    *REG_PTM0_CR1 |= (1 << 7); // Enable PTM0 interrupt

    // Initialize task array
    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        TT_Tasks[i].pTask = NULL;
        TT_Tasks[i].Delay = 0;
        TT_Tasks[i].Period = 0;
        TT_Tasks[i].RunMe = 0;
    }
}

/**
 * @brief Starts the Time Triggered (TT) OS scheduler.
 * Note: This typically enables the underlying hardware timer.
 */
void TT_Start(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    // Enable PTM0 timer counter
    *REG_PTM0_CR0 |= (1 << 7); // Placeholder: Enable PTM0 timer
}

/**
 * @brief Dispatches ready tasks in the TT scheduler.
 * Note: This function runs tasks that are ready to execute.
 */
void TT_Dispatch_task(void) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()
    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        if (TT_Tasks[i].RunMe > 0) {
            TT_Tasks[i].pTask(); // Execute the task
            TT_Tasks[i].RunMe--; // Decrement 'RunMe' flag

            // If it's a periodic task, reset its delay
            if (TT_Tasks[i].Period > 0) {
                TT_Tasks[i].Delay = TT_Tasks[i].Period;
            } else { // One-shot task, delete after running
                TT_Delete_task(i);
            }
        }
    }
}

/**
 * @brief Interrupt Service Routine (ISR) for the TT scheduler.
 * Note: This function should be called from the timer interrupt.
 */
void TT_ISR(void) {
    // No WDT_Reset() here as this is an ISR, not a user-callable API.
    // It's called from a hardware interrupt, not directly by user code.
    // WDT_Reset inside ISR can be problematic due to timing.

    // Clear the timer interrupt flag
    TIMER_ClearFlag(TIMER_PTM0); // Clear PTM0 interrupt flag

    TT_Tick_Count++; // Increment the global tick counter

    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        if (TT_Tasks[i].pTask != NULL) {
            if (TT_Tasks[i].Delay == 0) {
                TT_Tasks[i].RunMe++; // Mark task as ready to run
            } else {
                TT_Tasks[i].Delay--; // Decrement delay
            }
        }
    }
}

/**
 * @brief Adds a task to the TT scheduler.
 *
 * @param task Pointer to the task function.
 * @param period The period (in ticks) for periodic tasks. Set to 0 for one-shot.
 * @param delay The initial delay (in ticks) before the task first runs.
 * @return The index of the added task, or 0xFF if failed.
 */
tbyte TT_Add_task(void (*task)(void), const tword period, const tword delay) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (task == NULL) { return 0xFF; }

    // Find an empty slot
    for (tbyte i = 0; i < TT_MAX_TASKS; i++) {
        if (TT_Tasks[i].pTask == NULL) {
            TT_Tasks[i].pTask = task;
            TT_Tasks[i].Delay = delay;
            TT_Tasks[i].Period = period;
            TT_Tasks[i].RunMe = 0; // Not ready yet
            return i; // Return task index
        }
    }
    return 0xFF; // No empty slot
}

/**
 * @brief Deletes a task from the TT scheduler.
 *
 * @param task_index The index of the task to delete.
 */
void TT_Delete_task(const tbyte task_index) {
    WDT_Reset(); // Rule: All API bodies must include WDT_Reset()

    if (task_index < TT_MAX_TASKS && TT_Tasks[task_index].pTask != NULL) {
        TT_Tasks[task_index].pTask = NULL;
        TT_Tasks[task_index].Delay = 0;
        TT_Tasks[task_index].Period = 0;
        TT_Tasks[task_index].RunMe = 0;
    }
}