/***********************************************************************************************************************
* File Name      : ATMEGA32_GPIO.c
* Description    : This file implements device driver for (GPIO)
* Author         : AI
* Tester         :
* Device(s)      : ATMEGA32
* Creation Date  : 2025-06-30
* Testing Date   :
* @COPYRIGHT YYYY El-ARABY Research and Development Center. All rights reserved.
***********************************************************************************************************************/

/** Includes ============================================================================ */
#include "ATMEGA32_GPIO.h"

/** Static Variables ==================================================================== */

/** Functions =========================================================================== */

void GPIO_Value_Set(tport port, tpin pin, tbyte value) {
    WDT_Reset();

    switch(port) {
        case Port_0:
            if (value & (1 << pin)) {
                SET_BIT(P0, pin);
            } else {
                CLR_BIT(P0, pin);
            }
            break;
        case Port_1:
            if (value & (1 << pin)) {
                SET_BIT(P1, pin);
            } else {
                CLR_BIT(P1, pin);
            }
            break;
        case Port_2:
            if (value & (1 << pin)) {
                SET_BIT(P2, pin);
            } else {
                CLR_BIT(P2, pin);
            }
            break;
        case Port_3:
            if (value & (1 << pin)) {
                SET_BIT(P3, pin);
            } else {
                CLR_BIT(P3, pin);
            }
            break;
        default:
            /* Handle invalid port */
            break;
    }
}

tbyte GPIO_Value_Get(tport port, tpin pin) {
    WDT_Reset();

    switch(port) {
        case Port_0:
            return GET_BIT(P0, pin);
        case Port_1:
            return GET_BIT(P1, pin);
        case Port_2:
            return GET_BIT(P2, pin);
        case Port_3:
            return GET_BIT(P3, pin);
        default:
            /* Handle invalid port */
            return 0;
    }
}

void GPIO_Value_Tog(tport port, tpin pin) {
    WDT_Reset();

    switch(port) {
        case Port_0:
            TOG_BIT(P0, pin);
            break;
        case Port_1:
            TOG_BIT(P1, pin);
            break;
        case Port_2:
            TOG_BIT(P2, pin);
            break;
        case Port_3:
            TOG_BIT(P3, pin);
            break;
        default:
            /* Handle invalid port */
            break;
    }
}

t_direction GPIO_Direction_get(tport port, tpin pin) {
    WDT_Reset();

    switch(port) {
        case Port_0:
            return GET_BIT(PM0, pin);
        case Port_1:
            return GET_BIT(PM1, pin);
        case Port_2:
            return GET_BIT(PM2, pin);
        case Port_3:
            return GET_BIT(PM3, pin);
        default:
            /* Handle invalid port */
            return input;
    }
}

void GPIO_Output_Init(tport port, tpin pin, tbyte value, t_usage usage) {
    WDT_Reset();

    do {
    } while(GPIO_Direction_get(port, pin) != output);

    switch(port) {
        case Port_0:
            CLR_BIT(PU0, pin);  // Disable pull-up
            if (value & (1 << pin)) {
                SET_BIT(P0, pin);
            } else {
                CLR_BIT(P0, pin);
            }
            if (usage == communication_usage) {
                SET_BIT(POM0, pin);
            } else {
                CLR_BIT(POM0, pin);
            }
            CLR_BIT(PM0, pin);  // Set as output
            break;
        case Port_1:
            CLR_BIT(PU1, pin);  // Disable pull-up
            if (value & (1 << pin)) {
                SET_BIT(P1, pin);
            } else {
                CLR_BIT(P1, pin);
            }
            if (usage == communication_usage) {
                SET_BIT(POM1, pin);
            } else {
                CLR_BIT(POM1, pin);
            }
            CLR_BIT(PM1, pin);  // Set as output
            break;
        case Port_2:
            CLR_BIT(PU2, pin);  // Disable pull-up
            if (value & (1 << pin)) {
                SET_BIT(P2, pin);
            } else {
                CLR_BIT(P2, pin);
            }
            if (usage == communication_usage) {
                SET_BIT(POM2, pin);
            } else {
                CLR_BIT(POM2, pin);
            }
            CLR_BIT(PM2, pin);  // Set as output
            break;
        case Port_3:
            CLR_BIT(PU3, pin);  // Disable pull-up
            if (value & (1 << pin)) {
                SET_BIT(P3, pin);
            } else {
                CLR_BIT(P3, pin);
            }
            if (usage == communication_usage) {
                SET_BIT(POM3, pin);
            } else {
                CLR_BIT(POM3, pin);
            }
            CLR_BIT(PM3, pin);  // Set as output
            break;
        default:
            /* Handle invalid port */
            break;
    }
}

void GPIO_Input_Init(tport port, tpin pin, t_usage usage) {
    WDT_Reset();

    do {
    } while(GPIO_Direction_get(port, pin) != input);

    switch(port) {
        case Port_0:
            SET_BIT(PU0, pin);  // Enable pull-up
            if (usage == communication_usage) {
                SET_BIT(POM0, pin);
            } else {
                CLR_BIT(POM0, pin);
            }
            SET_BIT(PM0, pin);  // Set as input
            break;
        case Port_1:
            SET_BIT(PU1, pin);  // Enable pull-up
            if (usage == communication_usage) {
                SET_BIT(POM1, pin);
            } else {
                CLR_BIT(POM1, pin);
            }
            SET_BIT(PM1, pin);  // Set as input
            break;
        case Port_2:
            SET_BIT(PU2, pin);  // Enable pull-up
            if (usage == communication_usage) {
                SET_BIT(POM2, pin);
            } else {
                CLR_BIT(POM2, pin);
            }
            SET_BIT(PM2, pin);  // Set as input
            break;
        case Port_3:
            SET_BIT(PU3, pin);  // Enable pull-up
            if (usage == communication_usage) {
                SET_BIT(POM3, pin);
            } else {
                CLR_BIT(POM3, pin);
            }
            SET_BIT(PM3, pin);  // Set as input
            break;
        default:
            /* Handle invalid port */
            break;
    }
}