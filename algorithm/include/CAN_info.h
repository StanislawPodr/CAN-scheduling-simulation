#pragma once

#include <stdint.h>

/*
 * Smallest time unit for the simulation is the time needed to send one bit.
 * Transfer 500kb/s which is 500b/ms
 */
#define SMALLEST_TIME_UNIT_IN_MS            (1.0 / 500.0)

// CAN ID is 11 bit number
#define CAN_MAX_ID                          0x7FF
#define CAN_ILLEGAL_ID                      (CAN_MAX_ID + 1)
#define CAN_ID_ENGINE_TEMPERATURE           0x010
#define CAN_ID_ENGINE_PRESSURE              0x020
#define CAN_ID_POWER_ACC_VOLTAGE            0x100
#define CAN_ID_NAV_GPS_POS                  0x200
#define CAN_ID_DIAGNOSIS_SYSTEM_STAT        0x300

#define CAN_ENGINE_TEMPERATURE_SIZE         4
#define CAN_ENGINE_PRESSURE_SIZE            4
#define CAN_POWER_ACC_VOLTAGE_SIZE          2
#define CAN_NAV_GPS_POS_SIZE                8
#define CAN_DIAGNOSIS_SYSTEM_STAT_SIZE      4

/*
 * Some messages have delivery time constraints
 */
#define CAN_ENGINE_TEMP_TIME_CONSTRAINT_MS 50
#define CAN_ENGINE_PRESS_TIME_CONSTRAINT_MS 10
#define CAN_NAV_GPS_POS_TIME_CONSTRAINT_MS 100

typedef enum CanSenders_e
{
    CAN_ENGINE,
    CAN_POWER,
    CAN_NAVIGATION,
    CAN_DIAGNOSIS,
    CAN_UNKNOWN
} CanSenders_e;


typedef unsigned char u8;

/**
 * @brief Get CAN message sender from CAN ID
 * 
 * @param[in]  canId                Message CAN ID
 * 
 * @return CanSenders_e enum with CAN sender
 */
static inline CanSenders_e getCanSenderFromId(const uint16_t canId)
{
    switch (canId)
    {
    case CAN_ID_ENGINE_TEMPERATURE:
    case CAN_ID_ENGINE_PRESSURE:
        return CAN_ENGINE;
    case CAN_ID_POWER_ACC_VOLTAGE:
        return CAN_POWER;
    case CAN_ID_NAV_GPS_POS:
        return CAN_NAVIGATION;
    case CAN_ID_DIAGNOSIS_SYSTEM_STAT:
        return CAN_DIAGNOSIS;
    default:
        return CAN_UNKNOWN;
    }
}
