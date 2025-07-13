/***********************************************************************************************************************
* File Name      : pwm.c
* Description    : This file implements PWM driver functions for STM32F401RC.
* Author         : Technology Inovation Software Team
* Tester         : 
* Device(s)      : STM32F401RC
* Creation Date  : 2025-07-13
* Testing Date   : 
* @COPYRIGHT : ELARABY GROUP-TECHNOLOGY & INNOVATION CENTER.
***********************************************************************************************************************/

#include "STM32F401RC_PWM.h"
#include <stdint.h> // For uint32_t, uint16_t
#include <stddef.h> // For NULL

//======================================================================================================================
//--- Private Defines ---
//======================================================================================================================

// Base addresses (Assumed standard CMSIS definitions, not in provided PDF)
#define GPIOA_BASE            (0x40020000U)
#define GPIOB_BASE            (0x40020400U)
#define GPIOC_BASE            (0x40020800U)
#define GPIOD_BASE            (0x40020C00U)
#define GPIOE_BASE            (0x40021000U)
#define GPIOH_BASE            (0x40021C00U) // Per PDF, GPIO H is available except PH0/PH1

#define TIM1_BASE             (0x40010000U)
#define TIM2_BASE             (0x40000000U)
#define TIM3_BASE             (0x40000400U)
#define TIM4_BASE             (0x40000800U)
#define TIM5_BASE             (0x40000C00U)
// TIM6, TIM7 reserved - See comments below.
#define TIM9_BASE             (0x40014000U)
#define TIM10_BASE            (0x40014400U)
#define TIM11_BASE            (0x40014800U)

// RCC Base Address (Assumed standard CMSIS definitions, not in provided PDF)
// Note: RCC register access is required for clock gating, but not described in the provided PDF context.
// The implementation for enabling/disabling clocks will use placeholder logic or assumes external clock setup.
#define RCC_BASE              (0x40023800U)
#define RCC_AHB1ENR_OFFSET    (0x30U) // Offset for AHB1 Enable Register (GPIO clocks)
#define RCC_APB2ENR_OFFSET    (0x44U) // Offset for APB2 Enable Register (TIM1, TIM9-11 clocks)
#define RCC_APB1ENR_OFFSET    (0x40U) // Offset for APB1 Enable Register (TIM2-5 clocks)

// GPIO Register Offsets (from PDF)
#define GPIO_MODER_OFFSET     (0x00U) /* PDF Reference */
#define GPIO_OTYPER_OFFSET    (0x04U) /* PDF Reference */
#define GPIO_OSPEEDR_OFFSET   (0x08U) /* PDF Reference */
#define GPIO_PUPDR_OFFSET     (0x0CU) /* PDF Reference */
#define GPIO_AFRL_OFFSET      (0x20U) /* PDF Reference */
#define GPIO_AFRH_OFFSET      (0x24U) /* PDF Reference */

// GPIO Configuration Values (from PDF)
#define GPIO_MODE_INPUT       ((uint32_t)0x00) /* PDF Reference */
#define GPIO_MODE_OUTPUT      ((uint32_t)0x01) /* PDF Reference */
#define GPIO_MODE_AF          ((uint32_t)0x02) /* PDF Reference */
#define GPIO_MODE_ANALOG      ((uint32_t)0x03) /* PDF Reference */

#define GPIO_OTYPE_PP         ((uint32_t)0x00) /* PDF Reference */
#define GPIO_OTYPE_OD         ((uint32_t)0x01) /* PDF Reference */

#define GPIO_OSPEED_LOW       ((uint32_t)0x00) /* PDF Reference */
#define GPIO_OSPEED_MEDIUM    ((uint32_t)0x01) /* PDF Reference */
#define GPIO_OSPEED_HIGH      ((uint32_t)0x02) /* PDF Reference */
#define GPIO_OSPEED_VERY_HIGH ((uint32_t)0x03) /* PDF Reference */

#define GPIO_PULL_NO          ((uint32_t)0x00) /* PDF Reference */
#define GPIO_PULL_UP          ((uint32_t)0x01) /* PDF Reference */
#define GPIO_PULL_DOWN        ((uint32_t)0x02) /* PDF Reference */

// Timer Register Offsets (from PDF)
#define TIM_CR1_OFFSET        (0x00U) /* PDF Reference */
#define TIM_CR2_OFFSET        (0x04U) /* PDF Reference */
#define TIM_SMCR_OFFSET       (0x08U) /* PDF Reference */
#define TIM_DIER_OFFSET       (0x0CU) /* PDF Reference */
#define TIM_SR_OFFSET         (0x10U) /* PDF Reference */
#define TIM_EGR_OFFSET        (0x14U) /* PDF Reference */
#define TIM_CCMR1_OFFSET      (0x18U) /* PDF Reference */
#define TIM_CCMR2_OFFSET      (0x1CU) /* PDF Reference */ // Only for TIM1, TIM2-5
#define TIM_CCER_OFFSET       (0x20U) /* PDF Reference */
#define TIM_CNT_OFFSET        (0x24U) /* PDF Reference */
#define TIM_PSC_OFFSET        (0x28U) /* PDF Reference */
#define TIM_ARR_OFFSET        (0x2CU) /* PDF Reference */
#define TIM_RCR_OFFSET        (0x30U) /* PDF Reference */ // Only for TIM1
#define TIM_BDTR_OFFSET       (0x44U) /* PDF Reference */ // Only for TIM1
#define TIM_DCR_OFFSET        (0x48U) /* PDF Reference */
#define TIM_DMAR_OFFSET       (0x4CU) /* PDF Reference */
#define TIM_OR_OFFSET         (0x50U) /* PDF Reference */ // Only for TIM2, TIM5, TIM11 (Option registers)

// Timer CR1 Register Bits (from PDF)
#define TIM_CR1_CEN           ((uint32_t)0x0001) /* PDF Reference */
#define TIM_CR1_UDIS          ((uint32_t)0x0002) /* PDF Reference */
#define TIM_CR1_URS           ((uint32_t)0x0004) /* PDF Reference */
#define TIM_CR1_OPM           ((uint32_t)0x0008) /* PDF Reference */
#define TIM_CR1_DIR           ((uint32_t)0x0010) /* PDF Reference */
#define TIM_CR1_CMS_SHIFT     (5U) /* PDF Reference */
#define TIM_CR1_CMS_MASK      ((uint32_t)0x00C0)
#define TIM_CR1_ARPE          ((uint32_t)0x0080) /* PDF Reference */
#define TIM_CR1_CKD_SHIFT     (8U) /* PDF Reference */
#define TIM_CR1_CKD_MASK      ((uint32_t)0x0300)

// Timer CR2 Register Bits (from PDF)
#define TIM_CR2_CCPC          ((uint32_t)0x0001) /* PDF Reference */ // Only TIM1
#define TIM_CR2_CCUS          ((uint32_t)0x0004) /* PDF Reference */ // Only TIM1
#define TIM_CR2_CCDS          ((uint32_t)0x0008) /* PDF Reference */ // Only TIM1, TIM2-5
#define TIM_CR2_MMS_SHIFT     (4U) /* PDF Reference */ // Only TIM1, TIM2-5, TIM9
#define TIM_CR2_MMS_MASK      ((uint32_t)0x0070)
#define TIM_CR2_TI1S          ((uint32_t)0x0080) /* PDF Reference */ // Only TIM1, TIM2-5, TIM9

// Timer CCMRx Register Bits (from PDF) - Note: These bits are per channel (x), need to shift based on channel
// CCMR1: Bits 0-7 for Channel 1, Bits 8-15 for Channel 2
// CCMR2: Bits 0-7 for Channel 3, Bits 8-15 for Channel 4 (TIM1, TIM2-5 only)

// Common to Input/Output mode (CCMR1/CCMR2)
#define TIM_CCMRx_CCxS_SHIFT(channel) (((channel) % 2) * 8) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_CCxS_MASK(channel)  ((uint32_t)0x0003 << TIM_CCMRx_CCxS_SHIFT(channel))

// Output Compare Mode (CCMR1/CCMR2)
#define TIM_CCMRx_OCxFE_SHIFT(channel) (((channel) % 2) * 8 + 2) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_OCxPE_SHIFT(channel) (((channel) % 2) * 8 + 3) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_OCxM_SHIFT(channel)  (((channel) % 2) * 8 + 4) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_OCxM_MASK(channel)   ((uint32_t)0x0007 << TIM_CCMRx_OCxM_SHIFT(channel))
#define TIM_CCMRx_OCxCE_SHIFT(channel) (((channel) % 2) * 8 + 7) /* PDF Reference (CCMR1/CCMR2 Structure) */ // OCxCE is Bit 7 for Ch1, Bit 15 for Ch2 on CCMR1, etc.

// Input Capture Mode (CCMR1/CCMR2)
#define TIM_CCMRx_ICxF_SHIFT(channel)   (((channel) % 2) * 8 + 4) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_ICxF_MASK(channel)    ((uint32_t)0x000F << TIM_CCMRx_ICxF_SHIFT(channel))
#define TIM_CCMRx_ICxPSC_SHIFT(channel) (((channel) % 2) * 8 + 2) /* PDF Reference (CCMR1/CCMR2 Structure) */
#define TIM_CCMRx_ICxPSC_MASK(channel)  ((uint32_t)0x0003 << TIM_CCMRx_ICxPSC_SHIFT(channel))

// Output Compare Mode Values (from PDF)
#define TIM_OCMODE_FROZEN     ((uint32_t)0x000) /* PDF Reference */
#define TIM_OCMODE_ACTIVE     ((uint32_t)0x001) /* PDF Reference */
#define TIM_OCMODE_INACTIVE   ((uint32_t)0x002) /* PDF Reference */
#define TIM_OCMODE_TOGGLE     ((uint32_t)0x003) /* PDF Reference */
#define TIM_OCMODE_FORCE_INAC ((uint32_t)0x004) /* PDF Reference */
#define TIM_OCMODE_FORCE_AC   ((uint32_t)0x005) /* PDF Reference */
#define TIM_OCMODE_PWM1       ((uint32_t)0x006) /* PDF Reference */
#define TIM_OCMODE_PWM2       ((uint32_t)0x007) /* PDF Reference */

// Timer CCER Register Bits (from PDF) - Note: These bits are per channel (x)
// CCER: Bits 0-3 for Channel 1, Bits 4-7 for Channel 2, Bits 8-11 for Channel 3, Bits 12-15 for Channel 4
#define TIM_CCER_CCxE_SHIFT(channel)   (((channel) * 4) + 0) /* PDF Reference */
#define TIM_CCER_CCxP_SHIFT(channel)   (((channel) * 4) + 1) /* PDF Reference */
#define TIM_CCER_CCxNE_SHIFT(channel)  (((channel) * 4) + 2) /* PDF Reference */ // Only TIM1
#define TIM_CCER_CCxNP_SHIFT(channel)  (((channel) * 4) + 3) /* PDF Reference */ // Only TIM1

// Timer BDTR Register Bits (from PDF) - Only TIM1
#define TIM_BDTR_DTG_SHIFT     (0U) /* PDF Reference */
#define TIM_BDTR_DTG_MASK      ((uint32_t)0x00FF)
#define TIM_BDTR_LOCK_SHIFT    (8U) /* PDF Reference */
#define TIM_BDTR_LOCK_MASK     ((uint32_t)0x0300)
#define TIM_BDTR_OSSI          ((uint32_t)0x0400) /* PDF Reference */
#define TIM_BDTR_OSSR          ((uint32_t)0x0800) /* PDF Reference */
#define TIM_BDTR_BKE           ((uint32_t)0x1000) /* PDF Reference */
#define TIM_BDTR_BKP           ((uint32_t)0x2000) /* PDF Reference */
#define TIM_BDTR_AOE           ((uint32_t)0x4000) /* PDF Reference */
#define TIM_BDTR_MOE           ((uint32_t)0x8000) /* PDF Reference */

// Timer EGR Register Bits (from PDF)
#define TIM_EGR_UG             ((uint32_t)0x0001) /* PDF Reference */
#define TIM_EGR_CC1G           ((uint32_t)0x0002) /* PDF Reference */
#define TIM_EGR_CC2G           ((uint32_t)0x0004) /* PDF Reference */
#define TIM_EGR_CC3G           ((uint32_t)0x0008) /* PDF Reference */
#define TIM_EGR_CC4G           ((uint32_t)0x0010) /* PDF Reference */
#define TIM_EGR_COMG           ((uint32_t)0x0020) /* PDF Reference */ // Only TIM1, TIM9
#define TIM_EGR_TG             ((uint32_t)0x0040) /* PDF Reference */ // Only TIM1, TIM2-5, TIM9
#define TIM_EGR_BG             ((uint32_t)0x0080) /* PDF Reference */ // Only TIM1

// Timer Type Constants (for conditional logic)
#define TIMER_TYPE_ADVANCED   1
#define TIMER_TYPE_GENERAL_32 2
#define TIMER_TYPE_GENERAL_16 3
#define TIMER_TYPE_BASIC      4 // Reserved TIM6/7
#define TIMER_TYPE_GENERAL_9  5
#define TIMER_TYPE_GENERAL_10 6
#define TIMER_TYPE_GENERAL_11 7

// Maximum ARR value based on timer width
#define MAX_ARR_16BIT         ((uint32_t)0xFFFF)
#define MAX_ARR_32BIT         ((uint32_t)0xFFFFFFFF)

// Maximum Prescaler value
#define MAX_PSC               ((uint32_t)0xFFFF) /* PDF Reference (Prescaler is 16-bit) */

// Assumed Timer Clock Frequency (This must match actual RCC configuration)
// Assuming APB1 Timer clock = APB1 clock * 2 if APB1 prescaler > 1, else APB1 clock
// Assuming APB2 Timer clock = APB2 clock * 2 if APB2 prescaler > 1, else APB2 clock
// Assuming Max system clock (84MHz) and APB prescalers allow max frequency for timers (e.g. APB1=42MHz, APB2=84MHz, Timer clock = 84MHz)
// THIS VALUE IS A PLACEHOLDER AND MUST BE DEFINED BASED ON ACTUAL CLOCK TREE.
#define TIMER_CLK_FREQ        ((uint32_t)84000000U) /* Assumed Timer Clock Frequency - please verify */

//======================================================================================================================
//--- Private Types ---
//======================================================================================================================

// This struct definition is assumed to be in STM32F401RC_PWM.h as required by the prompt.
// Defining it here for completeness of the .c file.
typedef struct {
    uint32_t TIM_BASE;         // Base address of the Timer peripheral
    uint33_t Channel;          // Timer Channel Number (1 to 4 for TIM1-5, 1 or 2 for TIM9, 1 for TIM10/11)
    uint32_t GPIO_BASE;        // Base address of the GPIO port
    uint32_t Pin;              // GPIO Pin Number (0 to 15)
    uint32_t GPIO_AF;          // GPIO Alternate Function value
    uint32_t TimerType;        // Helper to distinguish timer capabilities
} PWM_Channel_Config_t;


// This enum definition is assumed to be in STM32F401RC_PWM.h as required by the prompt.
// Defining it here for completeness of the .c file.
// User defines these based on their application needs and maps them in pwm_channel_map
typedef enum {
    // Example Channels mapped below
    TRD_CHANNEL_1,
    TRD_CHANNEL_2,
    TRD_CHANNEL_3,
    TRD_CHANNEL_4,
    TRD_CHANNEL_5,
    TRD_CHANNEL_6,
    TRD_CHANNEL_7,
    TRD_CHANNEL_8,
    TRD_CHANNEL_9,
    TRD_CHANNEL_10,
    TRD_CHANNEL_11,
    TRD_CHANNEL_12,
    TRD_CHANNEL_COUNT // Total number of defined channels
} TRD_Channel_t;


//======================================================================================================================
//--- Private Data ---
//======================================================================================================================

// Note: Timers TIM6 and TIM7 are reserved for OS or delay purposes as per requirement.
// They are excluded from the PWM_channel_map below.

// Table mapping TRD_Channel_t to specific hardware configurations.
// Format: { TIM_BASE, ChannelNumber, GPIO_BASE, PinNumber, GPIO_AF, TimerType }
// ChannelNumber is 1-based for TIM1-5, TIM9-11 based on the PDF's CCRx definitions.
// PinNumber is 0-15 based on the PDF's GPIO register descriptions.
// Note: GPIO pin 0 (e.g. PA0, PB0) are excluded as per requirement, even if valid for PWM.
// Alternate Function values (GPIO_AF) are based on typical STM32F401RC mappings, as the PDF did not provide this table.
// PLEASE VERIFY THESE PIN MAPPINGS AND AF VALUES AGAINST THE STM32F401RC DATASHEET FOR YOUR SPECIFIC PACKAGE.
static const PWM_Channel_Config_t pwm_channel_map[] = {
    { TIM1_BASE, 1, GPIOA_BASE, 8,  0x01, TIMER_TYPE_ADVANCED },   /* Assumed PWM config - please verify */
    { TIM1_BASE, 2, GPIOA_BASE, 9,  0x01, TIMER_TYPE_ADVANCED },   /* Assumed PWM config - please verify */
    { TIM2_BASE, 2, GPIOA_BASE, 1,  0x01, TIMER_TYPE_GENERAL_32 }, /* Assumed PWM config - please verify */ // PA1 is TIM2_CH2 AF1
    { TIM2_BASE, 3, GPIOA_BASE, 2,  0x01, TIMER_TYPE_GENERAL_32 }, /* Assumed PWM config - please verify */ // PA2 is TIM2_CH3 AF1
    { TIM3_BASE, 1, GPIOA_BASE, 6,  0x02, TIMER_TYPE_GENERAL_16 }, /* Assumed PWM config - please verify */ // PA6 is TIM3_CH1 AF2
    { TIM3_BASE, 2, GPIOA_BASE, 7,  0x02, TIMER_TYPE_GENERAL_16 }, /* Assumed PWM config - please verify */ // PA7 is TIM3_CH2 AF2
    { TIM4_BASE, 1, GPIOB_BASE, 6,  0x02, TIMER_TYPE_GENERAL_16 }, /* Assumed PWM config - please verify */ // PB6 is TIM4_CH1 AF2
    { TIM4_BASE, 2, GPIOB_BASE, 7,  0x02, TIMER_TYPE_GENERAL_16 }, /* Assumed PWM config - please verify */ // PB7 is TIM4_CH2 AF2
    { TIM5_BASE, 2, GPIOA_BASE, 1,  0x02, TIMER_TYPE_GENERAL_32 }, /* Assumed PWM config - please verify */ // PA1 is TIM5_CH2 AF2 (Note: PA1 is also TIM2_CH2 AF1, user chooses via TRD_Channel_t)
    { TIM9_BASE, 1, GPIOA_BASE, 2,  0x03, TIMER_TYPE_GENERAL_9  }, /* Assumed PWM config - please verify */ // PA2 is TIM9_CH1 AF3 (Note: PA2 is also TIM2_CH3 AF1)
    { TIM10_BASE, 1, GPIOB_BASE, 8, 0x03, TIMER_TYPE_GENERAL_10 }, /* Assumed PWM config - please verify */ // PB8 is TIM10_CH1 AF3 (Note: PB8 is also TIM4_CH3 AF2)
    { TIM11_BASE, 1, GPIOB_BASE, 9, 0x03, TIMER_TYPE_GENERAL_11 }, /* Assumed PWM config - please verify */ // PB9 is TIM11_CH1 AF3 (Note: PB9 is also TIM4_CH4 AF2)
};

// Keep track of initialized timers to avoid re-initialization issues
static uint32_t InitializedTimers = 0; // Bitmask or similar if needed, simpler to track states within init if possible.
// For this implementation, we'll just check if a timer base address has been initialized.
// This requires a state tracking mechanism. A simple approach is to keep track of
// which TRD_Channels have been initialized.
static uint8_t IsChannelInitialized[TRD_CHANNEL_COUNT] = {0};


//======================================================================================================================
/**Functions ===========================================================================*/
//======================================================================================================================


/**
 * @brief Initializes the PWM hardware for a specific channel.
 * @param TRD_Channel: The application-defined PWM channel to initialize.
 */
void PWM_Init(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT)
    {
        // Invalid channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t gpio_base = config->GPIO_BASE;
    uint32_t pin_number = config->Pin;
    uint32_t timer_base = config->TIM_BASE;
    uint32_t timer_channel = config->Channel;
    uint32_t gpio_af = config->GPIO_AF;

    // Prevent re-initialization of the same channel
    if (IsChannelInitialized[TRD_Channel])
    {
        return;
    }

    // --- Enable Clocks (Requires RCC access - not fully described in PDF) ---
    // This part assumes standard RCC register layout for enabling GPIO and Timer clocks.
    // It is a dependency not covered by the provided PDF context.
    volatile uint32_t *rcc_ahb1enr = (volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET);
    volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET);
    volatile uint32_t *rcc_apb2enr = (volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET);

    // Enable GPIO clock for the port
    if (gpio_base == GPIOA_BASE) *rcc_ahb1enr |= (1U << 0); // GPIOAEN
    else if (gpio_base == GPIOB_BASE) *rcc_ahb1enr |= (1U << 1); // GPIOBEN
    else if (gpio_base == GPIOC_BASE) *rcc_ahb1enr |= (1U << 2); // GPIOCEN
    else if (gpio_base == GPIOD_BASE) *rcc_ahb1enr |= (1U << 3); // GPIODEN
    else if (gpio_base == GPIOE_BASE) *rcc_ahb1enr |= (1U << 4); // GPIOEEN
    else if (gpio_base == GPIOH_BASE) *rcc_ahb1enr |= (1U << 7); // GPIOHEN (Assuming bit 7 for GPIOH)

    // Enable Timer clock
    if (timer_base == TIM1_BASE) *rcc_apb2enr |= (1U << 0); // TIM1EN
    else if (timer_base == TIM9_BASE) *rcc_apb2enr |= (1U << 16); // TIM9EN (Assuming bit 16 for TIM9)
    else if (timer_base == TIM10_BASE) *rcc_apb2enr |= (1U << 17); // TIM10EN (Assuming bit 17 for TIM10)
    else if (timer_base == TIM11_BASE) *rcc_apb2enr |= (1U << 18); // TIM11EN (Assuming bit 18 for TIM11)
    else if (timer_base == TIM2_BASE) *rcc_apb1enr |= (1U << 0); // TIM2EN
    else if (timer_base == TIM3_BASE) *rcc_apb1enr |= (1U << 1); // TIM3EN
    else if (timer_base == TIM4_BASE) *rcc_apb1enr |= (1U << 2); // TIM4EN
    else if (timer_base == TIM5_BASE) *rcc_apb1enr |= (1U << 7); // TIM5EN


    // --- Configure GPIO Pin for Alternate Function ---
    volatile uint32_t *gpio_moder = (volatile uint32_t *)(gpio_base + GPIO_MODER_OFFSET); /* PDF Reference */
    volatile uint32_t *gpio_otyper = (volatile uint32_t *)(gpio_base + GPIO_OTYPER_OFFSET); /* PDF Reference */
    volatile uint32_t *gpio_ospeedr = (volatile uint32_t *)(gpio_base + GPIO_OSPEEDR_OFFSET); /* PDF Reference */
    volatile uint32_t *gpio_pupdr = (volatile uint32_t *)(gpio_base + GPIO_PUPDR_OFFSET); /* PDF Reference */
    volatile uint32_t *gpio_afr; // AFRL or AFRH

    // Set pin mode to Alternate Function (MODER = 10)
    *gpio_moder &= ~(GPIO_MODE_ANALOG << (pin_number * 2)); // Clear bits /* PDF Reference */
    *gpio_moder |= (GPIO_MODE_AF << (pin_number * 2));      // Set AF mode bits /* PDF Reference */

    // Set output type to Push-Pull (OTYPER = 0)
    *gpio_otyper &= ~(GPIO_OTYPE_OD << pin_number); /* PDF Reference */

    // Set output speed (High or Very High recommended for PWM)
    *gpio_ospeedr &= ~(GPIO_OSPEED_VERY_HIGH << (pin_number * 2)); // Clear bits /* PDF Reference */
    *gpio_ospeedr |= (GPIO_OSPEED_HIGH << (pin_number * 2));        // Set speed bits /* PDF Reference */

    // Set pull-up/pull-down to No Pull (PUPDR = 00)
    *gpio_pupdr &= ~(GPIO_PULL_DOWN << (pin_number * 2)); // Clear bits /* PDF Reference */
    *gpio_pupdr |= (GPIO_PULL_NO << (pin_number * 2));    // Set no pull bits /* PDF Reference */

    // Set Alternate Function value
    if (pin_number < 8)
    {
        gpio_afr = (volatile uint32_t *)(gpio_base + GPIO_AFRL_OFFSET); /* PDF Reference */
        *gpio_afr &= ~((uint32_t)0x0F << (pin_number * 4));           // Clear AF bits /* PDF Reference */
        *gpio_afr |= (gpio_af << (pin_number * 4));                   // Set AF value /* PDF Reference */ /* Assumed AF value */
    }
    else
    {
        gpio_afr = (volatile uint32_t *)(gpio_base + GPIO_AFRH_OFFSET); /* PDF Reference */
        *gpio_afr &= ~((uint32_t)0x0F << ((pin_number - 8) * 4));     // Clear AF bits /* PDF Reference */
        *gpio_afr |= (gpio_af << ((pin_number - 8) * 4));             // Set AF value /* PDF Reference */ /* Assumed AF value */
    }


    // --- Configure Timer Timebase and Channel ---
    volatile uint32_t *tim_cr1 = (volatile uint32_t *)(timer_base + TIM_CR1_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_ccmr = (volatile uint32_t *)(timer_base + ((timer_channel <= 2) ? TIM_CCMR1_OFFSET : TIM_CCMR2_OFFSET)); /* PDF Reference */
    volatile uint32_t *tim_ccer = (volatile uint32_t *)(timer_base + TIM_CCER_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_psc = (volatile uint32_t *)(timer_base + TIM_PSC_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_arr = (volatile uint32_t *)(timer_base + TIM_ARR_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_ccr = (volatile uint32_t *)(timer_base + TIM_CNT_OFFSET + 0x10 + ((timer_channel - 1) * 0x04)); /* PDF Reference (CCR offsets after ARR/RCR/BDTR) */
    volatile uint32_t *tim_egr = (volatile uint32_t *)(timer_base + TIM_EGR_OFFSET); /* PDF Reference */

    // Disable counter during configuration
    *tim_cr1 &= ~TIM_CR1_CEN; /* PDF Reference */

    // Configure Timebase (Upcounting mode, basic settings)
    *tim_cr1 &= ~TIM_CR1_DIR; // Upcounting /* PDF Reference */
    *tim_cr1 &= ~TIM_CR1_CMS_MASK; // Edge-aligned mode /* PDF Reference */
    *tim_cr1 |= TIM_CR1_ARPE; // Enable auto-reload preload /* PDF Reference */
    *tim_cr1 &= ~TIM_CR1_UDIS; // Enable update events /* PDF Reference */
    *tim_cr1 &= ~TIM_CR1_URS; // Any event can generate update /* PDF Reference */
    *tim_cr1 &= ~TIM_CR1_OPM; // Repetitive mode /* PDF Reference */

    // Set default PSC and ARR (Will be updated by Set_Freq)
    *tim_psc = 0; /* PDF Reference */
    // Set a non-zero ARR default to prevent counter blocking if Set_Freq isn't called immediately
    // Using a large value to make default frequency very low.
    if (config->TimerType == TIMER_TYPE_GENERAL_32)
    {
         *tim_arr = MAX_ARR_32BIT; /* PDF Reference */
    }
    else // 16-bit timers
    {
         *tim_arr = MAX_ARR_16BIT; /* PDF Reference */
    }

    // Configure Channel for PWM Output
    uint32_t channel_shift = (timer_channel - 1);
    uint32_t ccmr_channel_shift = ((timer_channel - 1) % 2) * 8; // Shift within CCMR1/CCMR2

    // Clear existing CCxS, OCxM, OCxPE, OCxFE bits for this channel
    *tim_ccmr &= ~(TIM_CCMRx_CCxS_MASK(channel_shift)); /* PDF Reference */
    *tim_ccmr &= ~(TIM_CCMRx_OCxM_MASK(channel_shift)); /* PDF Reference */
    *tim_ccmr &= ~((1U << TIM_CCMRx_OCxPE_SHIFT(channel_shift))); /* PDF Reference */
    *tim_ccmr &= ~((1U << TIM_CCMRx_OCxFE_SHIFT(channel_shift))); /* PDF Reference */

    // Set channel to Output Mode (CCxS = 00)
    *tim_ccmr |= (GPIO_MODE_OUTPUT << TIM_CCMRx_CCxS_SHIFT(channel_shift)); /* PDF Reference */

    // Set Output Compare Mode to PWM Mode 1 (OCxM = 110)
    *tim_ccmr |= (TIM_OCMODE_PWM1 << TIM_CCMRx_OCxM_SHIFT(channel_shift)); /* PDF Reference */

    // Enable Output Compare Preload (OCxPE = 1)
    *tim_ccmr |= (1U << TIM_CCMRx_OCxPE_SHIFT(channel_shift)); /* PDF Reference */

    // Set default Duty Cycle (0%)
    *tim_ccr = 0; /* PDF Reference */

    // Configure Output Polarity (Active High)
    *tim_ccer &= ~((1U << TIM_CCER_CCxP_SHIFT(channel_shift))); /* PDF Reference */

    // Disable Output Enable initially (CCxE = 0)
    *tim_ccer &= ~((1U << TIM_CCER_CCxE_SHIFT(channel_shift))); /* PDF Reference */

    // Handle TIM1 Specifics (BDTR register)
    if (config->TIM_BASE == TIM1_BASE)
    {
        volatile uint32_t *tim_bdtr = (volatile uint32_t *)(timer_base + TIM_BDTR_OFFSET); /* PDF Reference */
        // Clear MOE, AOE, BKE, BKP, OSSI, OSSR, LOCK (assuming BDTR might have reset value)
        *tim_bdtr &= ~(TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_BKE | TIM_BDTR_BKP | TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_LOCK_MASK); /* PDF Reference */
        // Set MOE later in PWM_Start for TIM1
        // No dead-time or break configured by default.
    }

    // Generate an update event to load all configurations (PSC, ARR, CCRx, etc.)
    // This also clears the counter and prescaler.
    *tim_egr |= TIM_EGR_UG; /* PDF Reference */
    // Wait a few cycles if needed for UG to take effect before re-enabling UDIS
    // (Typically not needed when UG is set by software before CEN=1)
    *tim_cr1 &= ~TIM_CR1_UDIS; // Re-enable update events after UG load /* PDF Reference */


    // Mark channel as initialized
    IsChannelInitialized[TRD_Channel] = 1;
}

/**
 * @brief Sets the desired PWM frequency and duty cycle for the selected channel.
 * @param TRD_Channel: The application-defined PWM channel.
 * @param frequency: Desired frequency in Hz.
 * @param duty: Desired duty cycle in percent (0 to 100).
 */
void PWM_Set_Freq(TRD_Channel_t TRD_Channel, tlong frequency, tbyte duty)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || !IsChannelInitialized[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    if (frequency == 0 || duty > 100)
    {
        // Invalid parameters
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t timer_base = config->TIM_BASE;
    uint32_t timer_channel = config->Channel;
    uint32_t timer_type = config->TimerType;

    volatile uint32_t *tim_cr1 = (volatile uint32_t *)(timer_base + TIM_CR1_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_psc = (volatile uint32_t *)(timer_base + TIM_PSC_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_arr = (volatile uint32_t *)(timer_base + TIM_ARR_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_ccr = (volatile uint32_t *)(timer_base + TIM_CNT_OFFSET + 0x10 + ((timer_channel - 1) * 0x04)); /* PDF Reference (CCR offsets) */
    volatile uint32_t *tim_egr = (volatile uint32_t *)(timer_base + TIM_EGR_OFFSET); /* PDF Reference */

    uint32_t timer_clk = TIMER_CLK_FREQ; /* Assumed Timer Clock Frequency - please verify */
    uint32_t max_arr = (timer_type == TIMER_TYPE_GENERAL_32) ? MAX_ARR_32BIT : MAX_ARR_16BIT;
    uint32_t period_counts = 0;
    uint32_t psc = 0;
    uint32_t arr = 0;
    uint32_t ccr_value = 0;

    // Calculate PSC and ARR for the desired frequency
    // Aim for the highest possible ARR for best resolution
    uint32_t total_counts = timer_clk / frequency;

    if (total_counts == 0)
    {
        // Frequency too high or timer_clk too low
        return;
    }

    // Find smallest PSC that results in ARR <= max_arr
    psc = 0;
    arr = total_counts - 1;
    while ((arr > max_arr) && (psc < MAX_PSC))
    {
        psc++;
        arr = (timer_clk / (frequency * (psc + 1))) - 1;
        if (timer_clk / (frequency * (psc + 1)) == 0) { // Avoid division by zero or large values resulting in 0
             arr = MAX_ARR_32BIT; // Make sure arr is large so the loop continues until psc is max
        }
    }

    if (psc > MAX_PSC || arr > max_arr)
    {
        // Cannot achieve target frequency within PSC/ARR limits
        // Could set to min/max possible frequency or indicate error.
        // For void function, just set to edge limits.
        psc = MAX_PSC;
        arr = max_arr;
        // Re-calculate effective total_counts with max PSC and ARR
        total_counts = (max_arr + 1) * (MAX_PSC + 1);
    } else if ((arr + 1) * (psc + 1) != total_counts) {
        // Due to integer division, recalculate ARR based on chosen PSC
        arr = (timer_clk / (frequency * (psc + 1))) - 1;
         if (timer_clk / (frequency * (psc + 1)) == 0) arr = 0; // handle case where frequency is too high for psc+1
    }

    // Calculate CCR value for the desired duty cycle
    // CCR = (ARR + 1) * duty / 100
    ccr_value = (uint32_t)(((uint64_t)(arr + 1) * duty) / 100); // Use 64-bit for intermediate calculation

    // Apply new PSC, ARR, and CCR values
    // Update disable to prevent UEV while writing
    *tim_cr1 |= TIM_CR1_UDIS; /* PDF Reference */

    *tim_psc = psc; /* PDF Reference */
    *tim_arr = arr; /* PDF Reference */
    *tim_ccr = ccr_value; /* PDF Reference */

    // Generate an update event to immediately load the new values (except for URS=1, but we configured URS=0 in Init)
    *tim_egr |= TIM_EGR_UG; /* PDF Reference */

    // Re-enable update events
    *tim_cr1 &= ~TIM_CR1_UDIS; /* PDF Reference */
}

/**
 * @brief Enables and starts PWM signal generation on the specified channel.
 * @param TRD_Channel: The application-defined PWM channel.
 */
void PWM_Start(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || !IsChannelInitialized[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t timer_base = config->TIM_BASE;
    uint32_t timer_channel = config->Channel;

    volatile uint32_t *tim_ccer = (volatile uint32_t *)(timer_base + TIM_CCER_OFFSET); /* PDF Reference */
    volatile uint32_t *tim_cr1 = (volatile uint32_t *)(timer_base + TIM_CR1_OFFSET); /* PDF Reference */

    // Enable Output Enable (CCxE = 1)
    *tim_ccer |= (1U << TIM_CCER_CCxE_SHIFT(timer_channel - 1)); /* PDF Reference */

    // For TIM1, enable Main Output (MOE = 1)
    if (config->TIM_BASE == TIM1_BASE)
    {
         volatile uint32_t *tim_bdtr = (volatile uint32_t *)(timer_base + TIM_BDTR_OFFSET); /* PDF Reference */
         *tim_bdtr |= TIM_BDTR_MOE; /* PDF Reference */
    }

    // Enable the counter (CEN = 1)
    *tim_cr1 |= TIM_CR1_CEN; /* PDF Reference */
}

/**
 * @brief Stops the PWM signal output on the specified channel.
 * @param TRD_Channel: The application-defined PWM channel.
 */
void PWM_Stop(TRD_Channel_t TRD_Channel)
{
    if (TRD_Channel >= TRD_CHANNEL_COUNT || !IsChannelInitialized[TRD_Channel])
    {
        // Invalid or uninitialized channel
        return;
    }

    const PWM_Channel_Config_t *config = &pwm_channel_map[TRD_Channel];
    uint32_t timer_base = config->TIM_BASE;
    uint32_t timer_channel = config->Channel;

    volatile uint32_t *tim_ccer = (volatile uint32_t *)(timer_base + TIM_CCER_OFFSET); /* PDF Reference */
    // volatile uint32_t *tim_cr1 = (volatile uint32_t *)(timer_base + TIM_CR1_OFFSET); // Not stopping the counter per channel

    // Disable Output Enable (CCxE = 0)
    *tim_ccer &= ~((1U << TIM_CCER_CCxE_SHIFT(timer_channel - 1))); /* PDF Reference */

    // Note: For TIM1, MOE controls all outputs. Disabling only CCxE for a single channel
    // on TIM1 is usually sufficient to stop *that* channel's output. MOE might stay high
    // if other channels are active. If all channels on TIM1 are stopped, MOE could be cleared.
    // This implementation only disables the individual channel output.
}

/**
 * @brief Disables all configured PWM peripherals and outputs to reduce power consumption.
 * This function disables all channels listed in the pwm_channel_map and attempts
 * to gate the clocks and reset GPIOs. Clock gating requires RCC access not fully
 * described in the provided PDF context.
 * @param None
 */
void PWM_PowerOff(void)
{
    volatile uint32_t *tim_cr1;
    volatile uint32_t *tim_ccer;
    volatile uint32_t *tim_bdtr;
    volatile uint32_t *gpio_moder;
    volatile uint32_t *gpio_otyper;
    volatile uint32_t *gpio_ospeedr;
    volatile uint32_t *gpio_pupdr;
    volatile uint32_t *gpio_afr; // AFRL or AFRH

    // --- Disable all configured PWM Channels and Timers ---
    for (TRD_Channel_t i = 0; i < TRD_CHANNEL_COUNT; ++i)
    {
        if (IsChannelInitialized[i])
        {
            const PWM_Channel_Config_t *config = &pwm_channel_map[i];
            uint32_t timer_base = config->TIM_BASE;
            uint32_t timer_channel = config->Channel;
            uint32_t gpio_base = config->GPIO_BASE;
            uint32_t pin_number = config->Pin;

            tim_cr1 = (volatile uint32_t *)(timer_base + TIM_CR1_OFFSET); /* PDF Reference */
            tim_ccer = (volatile uint32_t *)(timer_base + TIM_CCER_OFFSET); /* PDF Reference */

            // Disable the individual channel output
            *tim_ccer &= ~((1U << TIM_CCER_CCxE_SHIFT(timer_channel - 1))); /* PDF Reference */

            // Disable the timer counter
            *tim_cr1 &= ~TIM_CR1_CEN; /* PDF Reference */

            // For TIM1, disable main output
            if (config->TIM_BASE == TIM1_BASE)
            {
                 tim_bdtr = (volatile uint32_t *)(timer_base + TIM_BDTR_OFFSET); /* PDF Reference */
                 *tim_bdtr &= ~TIM_BDTR_MOE; /* PDF Reference */
            }

            // Reset GPIO pin to default Input Floating state (Optional, good for power saving)
            // Clear AF and set to Input mode (MODER = 00)
            gpio_moder = (volatile uint32_t *)(gpio_base + GPIO_MODER_OFFSET); /* PDF Reference */
            *gpio_moder &= ~(GPIO_MODE_ANALOG << (pin_number * 2)); /* PDF Reference */
            *gpio_moder |= (GPIO_MODE_INPUT << (pin_number * 2)); /* PDF Reference */

            // Set output type to Push-Pull (default)
            gpio_otyper = (volatile uint32_t *)(gpio_base + GPIO_OTYPER_OFFSET); /* PDF Reference */
            *gpio_otyper &= ~(GPIO_OTYPE_OD << pin_number); /* PDF Reference */

            // Set output speed to Low Speed (default)
            gpio_ospeedr = (volatile uint32_t *)(gpio_base + GPIO_OSPEEDR_OFFSET); /* PDF Reference */
            *gpio_ospeedr &= ~(GPIO_OSPEED_VERY_HIGH << (pin_number * 2)); /* PDF Reference */
            *gpio_ospeedr |= (GPIO_OSPEED_LOW << (pin_number * 2)); /* PDF Reference */

            // Set pull-up/pull-down to No Pull (default)
            gpio_pupdr = (volatile uint32_t *)(gpio_base + GPIO_PUPDR_OFFSET); /* PDF Reference */
            *gpio_pupdr &= ~(GPIO_PULL_DOWN << (pin_number * 2)); /* PDF Reference */
            *gpio_pupdr |= (GPIO_PULL_NO << (pin_number * 2)); /* PDF Reference */

            // Reset Alternate Function value to AF0 (System function / default)
            if (pin_number < 8)
            {
                gpio_afr = (volatile uint32_t *)(gpio_base + GPIO_AFRL_OFFSET); /* PDF Reference */
                *gpio_afr &= ~((uint32_t)0x0F << (pin_number * 4)); /* PDF Reference */ // Clear AF bits (sets to 0)
            }
            else
            {
                gpio_afr = (volatile uint32_t *)(gpio_base + GPIO_AFRH_OFFSET); /* PDF Reference */
                *gpio_afr &= ~((uint32_t)0x0F << ((pin_number - 8) * 4)); /* PDF Reference */ // Clear AF bits (sets to 0)
            }

            IsChannelInitialized[i] = 0; // Mark channel as uninitialized
        }
    }

    // --- Disable Timer Clocks (Requires RCC access - not fully described in PDF) ---
    // This part assumes standard RCC register layout for disabling Timer clocks.
    // It is a dependency not covered by the provided PDF context.
    volatile uint32_t *rcc_apb1enr = (volatile uint32_t *)(RCC_BASE + RCC_APB1ENR_OFFSET);
    volatile uint32_t *rcc_apb2enr = (volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET);

    // Disable clocks for all timers that *might* have been used for PWM
    // Note: This is an aggressive power-off. If other modules use these timers,
    // their clocks should not be disabled here.
    *rcc_apb2enr &= ~((1U << 0) | (1U << 16) | (1U << 17) | (1U << 18)); // TIM1EN, TIM9EN, TIM10EN, TIM11EN
    *rcc_apb1enr &= ~((1U << 0) | (1U << 1) | (1U << 2) | (1U << 7));    // TIM2EN, TIM3EN, TIM4EN, TIM5EN

    // --- Disable GPIO Clocks (Requires RCC access - not fully described in PDF) ---
    // This part assumes standard RCC register layout for disabling GPIO clocks.
    // It is a dependency not covered by the provided PDF context.
    volatile uint32_t *rcc_ahb1enr = (volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET);

    // Disable clocks for all GPIO ports that *might* have been used for PWM
    // Note: This is an aggressive power-off. If other modules use these GPIO ports,
    // their clocks should not be disabled here.
    *rcc_ahb1enr &= ~((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4) | (1U << 7)); // GPIOA-E, GPIOH
}