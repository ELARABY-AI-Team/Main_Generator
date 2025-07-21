/* Timer Registers for TIM10 and TIM11 */
/* Base Addresses - Assuming standard STM32 Timer addresses */ /* Assumed based on MCU conventions */
#define TIM10_BASE    0x40000000U  /* Base address for TIM10 */
#define TIM11_BASE    0x40010000U  /* Base address for TIM11 */

/* Register Offsets */
/* Control Registers */
#define TIM_CR1_OFFSET   0x00       /* Control register 1 offset */
#define TIM_DIER_OFFSET  0x0C       /* Digital Input Enable Register offset */
#define TIM_SR_OFFSET    0x10       /* Status register offset */
#define TIM_EGR_OFFSET   0x14       /* Event Generator register offset */
#define TIM_CCMR1_OFFSET 0x18      /* Capture/Compare Mode Register 1 offset */
#define TIM_CCER_OFFSET  0x20      /* Capture/Compare Enable Register offset */
#define TIM_CNT_OFFSET   0x24      /* Counter register offset */
#define TIM_PSC_OFFSET   0x28      /* Prescaler register offset */
#define TIM_ARR_OFFSET   0x2C      /* Auto-reload register offset */
#define TIM_CCR1_OFFSET  0x34     /* Capture/Compare register 1 offset */
#define TIM_OR_OFFSET    0x50      /* Option Register 1 offset */

/* Register Definitions */
typedef struct {
    volatile uint16_t CR1;          /* Control Register 1 */
    volatile uint16_t DIER;         /* Digital Input Enable Register */
    volatile uint16_t SR;           /* Status Register */
    volatile uint16_t EGR;          /* Event Generator Register */
    union {
        struct {                    /* Output Compare Mode */
            uint16_t OC1PE : 1;     /* PDF Reference: CCMR1 register */
            uint16_t OC1FE : 1;
            uint16_t CC1S  : 2;
            uint16_t OC1M  : 3;
        };
        struct {                    /* Input Capture Mode */
            uint16_t IC1PSC : 2;    /* PDF Reference: CCMR1 register */
            uint16_t IC1F   : 4;
            uint16_t CC1S   : 2;
        };
    } CCMR1;
    volatile uint16_t CCER;         /* Capture/Compare Enable Register */
    volatile uint16_t CNT;          /* Counter */
    volatile uint16_t PSC;          /* Prescaler */
    volatile uint16_t ARR;          /* Auto-Reload */
    volatile uint16_t CCR1;         /* Capture Compare 1 */
    volatile uint16_t OR;           /* Option Register */
} TIM10_Type, TIM11_Type;

/* Bit Masks */
#define TIM_CR1_CEN           (1 << 0)   /* Counter Enable */
#define TIM_CR1_UDIS          (1 << 1)   /* Update Disable */
#define TIM_CR1_URS          (1 << 2)    /* Update Request Source */
#define TIM_CR1_OPM         (1 << 3)     /* One Pulse Mode */
#define TIM_CR1_ARPE        (1 << 7)     /* Auto-Reload Preload Enable */
#define TIM_DIER_UIE        (1 << 0)      /* Update Interrupt Enable */
#define TIM_SR UIF           (1 << 0)     /* Update Interrupt Flag */
#define TIM_EGR_UG          (1 << 0)      /* Update Generation */
#define TIM_CCMR1_OC1PE    (1 << 0)       /* Output Compare 1 Preload Enable */
#define TIM_CCMR1_OC1FE    (1 << 1)       /* Output Compare 1 Fast Enable */
#define TIM_CCMR1_CC1S     (0x03 << 2)   /* Capture Compare 1 Software */
#define TIM_CCMR1_OC1M     (0x07 << 3)   /* Output Compare 1 Mode */
#define TIM_CCR1_reserved  0xFFFF        /* Reserved bits */

/* Option Register Masks */
#define TIM_OR_TI1_RMP    (0x03 << 0)     /* PDF Reference: OR register */