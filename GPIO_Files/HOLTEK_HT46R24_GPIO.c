/**
 * @file HOLTEK_HT46R24_GPIO.c
 * @brief GPIO driver implementation for the HOLTEK HT46R24 microcontroller.
 *
 * This file provides the implementation for basic General Purpose Input/Output (GPIO)
 * functions as declared in HOLTEK_HT46R24_GPIO.h. It includes functions for
 * initialization, setting pin direction, setting output state, and reading pin state.
 *
 * @note This code assumes the necessary SFRs (Special Function Registers) like
 *       PA, PB, PAC, PBC, etc., and typedefs/enums like Port_TypeDef,
 *       Direction_TypeDef, State_TypeDef are correctly defined in
 *       HOLTEK_HT46R24_GPIO.h and HOLTEK_HT46R24_MAIN.h.
 *       It also assumes the standard Holtek GPIO control register logic
 *       where 1 = Input (High-Z) and 0 = Output.
 *
 * @author [Your Name/Company Name]
 * @date [Date]
 * @version 1.0
 * @license [Specify License, e.g., MIT, BSD, or Proprietary]
 */

// Include necessary header files
#include "HOLTEK_HT46R24_GPIO.h"
#include "HOLTEK_HT46R24_MAIN.h" // Assumed to contain core types like uint8_t

// --- Assumptions ---
// 1. HOLTEK_HT46R24_GPIO.h defines:
//    - Register definitions (e.g., extern volatile __sfr __at(0x00) PA;) for PA, PB, PC, PD, PE, PF
//    - Control register definitions (e.g., extern volatile __sfr __at(0x08) PAC;) for PAC, PBC, PCC, PDC, PEC, PFC
//    - Pull-up register definitions (e.g., extern volatile __sfr __at(0x10) PAPU;) for PAPU, PBPU, PCPU, PDPU, PEPU, PFPU
//    - An enumeration `Port_TypeDef` (e.g., GPIO_PORT_A, GPIO_PORT_B, ..., GPIO_PORT_F)
//    - An enumeration `Direction_TypeDef` (e.g., GPIO_DIRECTION_INPUT, GPIO_DIRECTION_OUTPUT)
//    - An enumeration `State_TypeDef` (e.g., GPIO_STATE_LOW, GPIO_STATE_HIGH)
//    - Function prototypes matching the implementations below.
// 2. HOLTEK_HT46R24_MAIN.h defines basic types like `uint8_t`.
// 3. The microcontroller's core clock and power systems are initialized prior to calling GPIO_Init().
// 4. The caller provides valid Port_TypeDef values and pinMasks (bits corresponding to actual pins on the port).
//    Masks are represented by a uint8_t where each bit corresponds to a pin (e.g., 0x01 for Pin0, 0x03 for Pin0 & Pin1).
// --- End Assumptions ---

/**
 * @brief Initializes all GPIO ports to a safe default state.
 *
 * Sets all pins across all available ports (A-F) to:
 * - Direction: Input (High-Z)
 * - Pull-up Resistors: Disabled
 * - Data Register Latch: Low (0) - primarily affects behavior if later switched to output
 *
 * This function should typically be called early in the system initialization.
 */
void GPIO_Init(void)
{
    // Set all ports to Input (Control Register = 0xFF)
    // Set all pull-ups to Disabled (Pull-up Register = 0x00)
    // Set all data latches to Low (Data Register = 0x00)

    // Port A
    PAC = 0xFF;  // All Port A pins as Input
    PAPU = 0x00; // Disable Port A pull-ups
    PA = 0x00;   // Set Port A data latch to 0

    // Port B
    PBC = 0xFF;  // All Port B pins as Input
    PBPU = 0x00; // Disable Port B pull-ups
    PB = 0x00;   // Set Port B data latch to 0

    // Port C
    PCC = 0xFF;  // All Port C pins as Input
    PCPU = 0x00; // Disable Port C pull-ups
    PC = 0x00;   // Set Port C data latch to 0

#ifdef PD // Check if Port D exists on this specific variant
    // Port D
    PDC = 0xFF;  // All Port D pins as Input
    PDPU = 0x00; // Disable Port D pull-ups
    PD = 0x00;   // Set Port D data latch to 0
#endif

#ifdef PE // Check if Port E exists on this specific variant
    // Port E
    PEC = 0xFF;  // All Port E pins as Input
    PEPU = 0x00; // Disable Port E pull-ups
    PE = 0x00;   // Set Port E data latch to 0
#endif

#ifdef PF // Check if Port F exists on this specific variant
    // Port F
    PFC = 0xFF;  // All Port F pins as Input
    PFPU = 0x00; // Disable Port F pull-ups
    PF = 0x00;   // Set Port F data latch to 0
#endif

    // Note: Other port configurations (e.g., analog input, alternate functions)
    // would require configuring other specific registers (like PACS, PADS, etc.)
    // This basic init only covers digital input/output and pull-ups.
}

/**
 * @brief Sets the direction (Input or Output) for specified pins on a given port.
 *
 * @param port The GPIO port (e.g., GPIO_PORT_A, GPIO_PORT_B).
 * @param pinMask A mask where each set bit corresponds to a pin to configure
 *                (e.g., 0x01 for Pin0, 0x03 for Pin0 and Pin1).
 * @param direction The desired direction (GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT).
 *
 * @note On HT46R24, Control Register bit = 1 sets the pin as Input,
 *       Control Register bit = 0 sets the pin as Output.
 *       Invalid port values or pinMasks corresponding to non-existent pins
 *       might result in undefined behavior or no action on those invalid bits.
 */
void GPIO_SetDirection(Port_TypeDef port, uint8_t pinMask, Direction_TypeDef direction)
{
    // We need to access the corresponding Control Register (PAC, PBC, etc.)
    // A switch statement is used to select the correct register based on the port.

    // Use temporary volatile pointers to the registers to avoid redundant switch logic
    volatile uint8_t *pCtrlReg;

    // Map port enum to control register pointer
    switch (port)
    {
        case GPIO_PORT_A:
            pCtrlReg = &PAC;
            break;
        case GPIO_PORT_B:
            pCtrlReg = &PBC;
            break;
        case GPIO_PORT_C:
            pCtrlReg = &PCC;
            break;
#ifdef PD
        case GPIO_PORT_D:
            pCtrlReg = &PDC;
            break;
#endif
#ifdef PE
        case GPIO_PORT_E:
            pCtrlReg = &PEC;
            break;
#endif
#ifdef PF
        case GPIO_PORT_F:
            pCtrlReg = &PFC;
            break;
#endif
        default:
            // Handle invalid port if necessary (e.g., return error, assert, or ignore)
            // For a clean implementation assuming valid input, we'll ignore.
            return;
    }

    // Modify the control register based on the desired direction and pin mask
    if (direction == GPIO_DIRECTION_INPUT)
    {
        // Set the corresponding bits in the Control Register to 1 (Input)
        *pCtrlReg |= pinMask;
    }
    else // direction == GPIO_DIRECTION_OUTPUT
    {
        // Clear the corresponding bits in the Control Register to 0 (Output)
        *pCtrlReg &= ~pinMask;
    }
}

/**
 * @brief Sets the output state (High or Low) for specified pins on a given port.
 *
 * This function only affects pins that are configured as Output.
 *
 * @param port The GPIO port (e.g., GPIO_PORT_A, GPIO_PORT_B).
 * @param pinMask A mask where each set bit corresponds to a pin to set state
 *                (e.g., 0x01 for Pin0, 0x03 for Pin0 and Pin1).
 * @param state The desired output state (GPIO_STATE_LOW or GPIO_STATE_HIGH).
 *
 * @note Writing to a pin configured as Input will update its internal data latch,
 *       which will become the output state if the pin is later switched to Output.
 *       This function modifies the data register regardless of the current pin direction.
 *       Caller is responsible for ensuring pins are configured as output if
 *       immediate external state change is expected.
 */
void GPIO_SetPinState(Port_TypeDef port, uint8_t pinMask, State_TypeDef state)
{
    // We need to access the corresponding Data Register (PA, PB, etc.)
    // A switch statement is used to select the correct register based on the port.

    volatile uint8_t *pDataReg;

    // Map port enum to data register pointer
    switch (port)
    {
        case GPIO_PORT_A:
            pDataReg = &PA;
            break;
        case GPIO_PORT_B:
            pDataReg = &PB;
            break;
        case GPIO_PORT_C:
            pDataReg = &PC;
            break;
#ifdef PD
        case GPIO_PORT_D:
            pDataReg = &PD;
            break;
#endif
#ifdef PE
        case GPIO_PORT_E:
            pDataReg = &PE;
            break;
#endif
#ifdef PF
        case GPIO_PORT_F:
            pDataReg = &PF;
            break;
#endif
        default:
            // Handle invalid port if necessary (e.g., return error, assert, or ignore)
            return;
    }

    // Modify the data register based on the desired state and pin mask
    if (state == GPIO_STATE_HIGH)
    {
        // Set the corresponding bits in the Data Register to 1 (High)
        *pDataReg |= pinMask;
    }
    else // state == GPIO_STATE_LOW
    {
        // Clear the corresponding bits in the Data Register to 0 (Low)
        *pDataReg &= ~pinMask;
    }
}

/**
 * @brief Reads the input state of specified pins on a given port.
 *
 * This function reads the current state of the actual physical pins.
 * For pins configured as Output, this function reads the state of the
 * output latch.
 *
 * @param port The GPIO port (e.g., GPIO_PORT_A, GPIO_PORT_B).
 * @param pinMask A mask where each set bit corresponds to a pin to read
 *                (e.g., 0x01 for Pin0, 0x03 for Pin0 and Pin1).
 * @return A uint8_t value representing the state of the specified pins.
 *         Each bit in the return value corresponds to a bit set in pinMask.
 *         A '1' indicates the pin is high, '0' indicates low.
 *         Bits not set in pinMask will be '0' in the return value.
 */
uint8_t GPIO_ReadPinState(Port_TypeDef port, uint8_t pinMask)
{
    // We need to access the corresponding Data Register (PA, PB, etc.)
    // Reading this register reflects the state of the physical pin
    // if configured as input, or the output latch if configured as output.
    // A switch statement is used to select the correct register based on the port.

    volatile uint8_t *pDataReg;

    // Map port enum to data register pointer
    switch (port)
    {
        case GPIO_PORT_A:
            pDataReg = &PA;
            break;
        case GPIO_PORT_B:
            pDataReg = &PB;
            break;
        case GPIO_PORT_C:
            pDataReg = &PC;
            break;
#ifdef PD
        case GPIO_PORT_D:
            pDataReg = &PD;
            break;
#endif
#ifdef PE
        case GPIO_PORT_E:
            pDataReg = &PE;
            break;
#endif
#ifdef PF
        case GPIO_PORT_F:
            pDataReg = &PF;
            break;
#endif
        default:
            // Handle invalid port - return 0 or an error indicator
            return 0x00;
    }

    // Read the entire port register and mask it with the desired pins
    return (*pDataReg) & pinMask;
}

// --- Optional: Add functions for Pull-up enable/disable ---
// These would follow a similar pattern to SetDirection/SetState but access
// the PAPU, PBPU, etc. registers.

/**
 * @brief Enables or disables the internal pull-up resistors for specified pins.
 *
 * @param port The GPIO port (e.g., GPIO_PORT_A, GPIO_PORT_B).
 * @param pinMask A mask where each set bit corresponds to a pin to configure.
 * @param enableState PULLUP_ENABLE or PULLUP_DISABLE.
 *
 * @note On HT46R24, Pull-up Register bit = 1 enables the pull-up,
 *       Pull-up Register bit = 0 disables the pull-up.
 *       Pull-ups are typically only effective when the pin is configured as Input.
 */
/*
void GPIO_SetPullUp(Port_TypeDef port, uint8_t pinMask, PullUp_TypeDef enableState)
{
     volatile uint8_t *pPuReg;

     switch (port)
     {
         case GPIO_PORT_A: pPuReg = &PAPU; break;
         case GPIO_PORT_B: pPuReg = &PBPU; break;
         case GPIO_PORT_C: pPuReg = &PCPU; break;
 #ifdef PD
         case GPIO_PORT_D: pPuReg = &PDPU; break;
 #endif
 #ifdef PE
         case GPIO_PORT_E: pPuReg = &PEPU; break;
 #endif
 #ifdef PF
         case GPIO_PORT_F: pPuReg = &PFPU; break;
 #endif
         default: return; // Invalid port
     }

     if (enableState == PULLUP_ENABLE)
     {
         *pPuReg |= pinMask;
     }
     else // PULLUP_DISABLE
     {
         *pPuReg &= ~pinMask;
     }
}
*/

// --- Optional: Add functions for Analog Select ---
// HT46R24 has specific Analog Select Registers (PACS, PADS, etc.)
// These would need configuration based on which pins are used for Analog Input (ADC).

/*
void GPIO_SetAnalogMode(Port_TypeDef port, uint8_t pinMask, AnalogMode_TypeDef mode)
{
    volatile uint8_t *pAnalogSelReg;

    // Assuming Analog select registers are named PACS, PADS, etc. and
    // that mode enum defines states like ANALOG_ENABLED, ANALOG_DISABLED
    // and mapping these modes to register bit states (often 1 for analog, 0 for digital)

    switch (port)
    {
        case GPIO_PORT_A: pAnalogSelReg = &PACS; break; // Assuming PACS register
        case GPIO_PORT_B: // No analog on PB typically? Check datasheet
             return; // Or handle appropriately if some PB pins have analog
        case GPIO_PORT_C: pAnalogSelReg = &PCCS; break; // Assuming PCCS register
 #ifdef PD
        case GPIO_PORT_D: pAnalogSelReg = &PDS; break; // Assuming PDS register
 #endif
        // ... add other ports with analog capability
        default: return; // Invalid port or port without analog
    }

    if (mode == ANALOG_ENABLED)
    {
        // Assuming setting bit enables analog
        *pAnalogSelReg |= pinMask;
    }
    else // ANALOG_DISABLED
    {
        // Assuming clearing bit disables analog (enables digital)
        *pAnalogSelReg &= ~pinMask;
    }
}
*/

// --- End of File ---