/*
 * registr_map.h
 *
 *  Created on: Oct 19, 2020
 *      Author: Z13, vehar
 */

#ifndef SRC_REGISTR_MAP_H_
#define SRC_REGISTR_MAP_H_

#include <stdint.h>

#define PA_AMOUNT (1)

typedef __packed struct
{
    uint8_t A2;  // 22
    uint8_t A3;  // 23
    uint8_t B3;  // 24
    uint8_t B5;  // 25
    uint8_t B6;  // 26
    uint8_t B7;  // 27
    uint8_t B13; // 28
    uint8_t B15; // 29
    uint8_t D2;  // 30
} GPIO;

typedef __packed union GPIO_MAP
{
    GPIO Pin;
    uint8_t Idx[sizeof(GPIO)];
} gpioMap_t;

typedef __packed struct
{
    uint8_t FAN_PWM;          // 0
    uint8_t MAIN_CONTROL;     // 1
    uint8_t ALARM_1_FLAG;     // 2
    uint8_t OVER_TEMP_FLAG;   // 3
    uint8_t BTS_RESET;        // 4
    uint8_t POW_OFF_FLAG;     // 5
    uint8_t CURRENT_I;        // 6
    uint8_t CURRENT_F;        // 7
    uint8_t VOLTAGE_28V_I;    // 8
    uint8_t VOLTAGE_28V_F;    // 9
    uint8_t VOLTAGE_SUPPLY_I; // 10
    uint8_t VOLTAGE_SUPPLY_F; // 11
    uint8_t VOLTAGE_12V_I;    // 12
    uint8_t VOLTAGE_12V_F;    // 13
    uint8_t PEAK_TEMPERATURE; // 14
    uint8_t TEMPERATURE_TH;   // 15
    int8_t TEMP_BTS_1;        // 16
    int8_t TEMP_BTS_2;        // 17
    int8_t TEMP_BTS_3;        // 18
    uint8_t FAN_ALARM_FLAG;   // 19
    uint8_t PROTECTION;       // 20
    uint8_t MIN_PWM;          // 21
    uint8_t SWITCH_CONTROL;
    uint8_t SWITCH_MANUAL_POS;
    uint8_t SWITCH_ENTITY_IDX;
    uint8_t SWITCH_DELAY_MS_L;
    uint8_t SWITCH_DELAY_MS_H;
    gpioMap_t IO; // 22..30
} CONTROL_BOARD_INTERNAL_REGS;

typedef __packed struct
{
    uint8_t STATUS;        // 0
    float POWER_mW;        // 1-4
    double SWR;            // 5-12
    int8_t TEMPERATURE;    // 13
    uint8_t POWER_I;       // 14
    uint8_t POWER_F;       // 15
    uint8_t SWR_I;         // 16
    uint8_t SWR_F;         // 17
    uint8_t SHUTDOWN;      // 18
    uint8_t SET_TDD;       // 19
    uint8_t rez3;          // 20
    uint8_t OVER_TEMP_SET; // 21
    uint8_t INST_I2C_ADR;  // 22
    uint8_t rez4[3];       //-->25
} EXTERNAL_PA_REGS;

typedef __packed struct
{
    CONTROL_BOARD_INTERNAL_REGS CB; // 0 - 30
    EXTERNAL_PA_REGS PA[PA_AMOUNT]; // 31 - 186
} CONTROL_BOARD_REGS_t;

enum SWITCH_CONTROL_BITS
{
    SWITCH_CONTROL_RUN = 0, // or Stop
    SWITCH_CONTROL_MANUAL   // or auto switch
};

enum MAIN_CONTROL_BITS
{
    res0 = 0,
    res1,
    FAN_AUTO,
    BTS_3_EN, // 3
    BTS_2_EN,
    BTS_1_EN,
    MAIN_POW_28_EN,
    SUPPLY_EN // 7
};

enum CB_ALARM_1_FLAG_BITS
{
    OVC = 0, // Over current
    UVT = 3, // Under voltage
    OVT = 4  // Over voltage
};

enum CB_OVER_TEMP_FLAG_BITS
{
    BOX_OVT = 0,
    PA_1_OVT,
    PA_2_OVT,
    PA_3_OVT,
    PA_4_OVT,
    PA_5_OVT,
    PA_6_OVT // 6
};

enum CB_BTS_RESET_BITS
{
    BTS_3_RST = 3,
    BTS_2_RST,
    BTS_1_RST
};

enum CB_POW_OFF_FLAG_BITS
{
    POW_OFF = 0,
    BTN_PRESSED
};

enum CB_GPIO
{
    GPIO_VAL = 0,
    GPIO_STATE
};
/////

enum PA_SHUTDOWN_BITS
{
    PA_POW_5_EN = 0,
    PA_POW_28_EN
};

enum PA_SET_TDD_BITS
{
    TDD_EN = 0,
};

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1ULL << (b)))
#define BIT_FLIP(a, b) ((a) ^= (1ULL << (b)))
#define BIT_CHECK(a, b) (!!((a) & (1ULL << (b)))) // '!!' to make sure this returns 0 or 1

/* x=target variable, y=mask */
#define BITMASK_SET(x, y) ((x) |= (y))
#define BITMASK_CLEAR(x, y) ((x) &= (~(y)))
#define BITMASK_FLIP(x, y) ((x) ^= (y))
#define BITMASK_CHECK_ALL(x, y) (!(~(x) & (y)))
#define BITMASK_CHECK_ANY(x, y) ((x) & (y))

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define foreach_pa() for (uint8_t i = 0; i < PA_AMOUNT; i++)
#define foreach_io() for (uint8_t i = 0; i < sizeof(GPIO); i++)

typedef union REG_MAP
{
    CONTROL_BOARD_REGS_t Reg;
    uint8_t i2c1_ram[sizeof(CONTROL_BOARD_REGS_t)];
} map_t;

/*
uint8_t getRegAddress(union REG_MAP *inst, uint8_t *reg)
{
    return (uint8_t) ((uint32_t)((reg)) - (uint32_t)(inst));
}

uint32_t test = getRegAddress(&map, &map.Reg.PA[5].INST_I2C_ADR);
*/

#endif /* SRC_REGISTR_MAP_H_ */
