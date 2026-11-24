#include <stdint.h>

#include "CAN_driver.h"
#include "CAN_info.h"
#include "CAN_time.h"
#include "queue.h"

#define CAN_ENGINE_TEMPERATURE_FREQ_MS 10
#define CAN_ENGINE_TEMPERATURE_FREQ_SYS ((uint32_t)(CAN_ENGINE_TEMPERATURE_FREQ_MS / SMALLEST_TIME_UNIT_IN_MS))
#define CAN_ENGINE_TEMP_TIME_CONSTRAINT_SYS ((uint32_t)(CAN_ENGINE_TEMP_TIME_CONSTRAINT_MS / SMALLEST_TIME_UNIT_IN_MS))

#define CAN_ENGINE_PRESSURE_FREQ_MS 20
#define CAN_ENGINE_PRESSURE_FREQ_SYS ((uint32_t)(CAN_ENGINE_PRESSURE_FREQ_MS / SMALLEST_TIME_UNIT_IN_MS))
#define CAN_ENGINE_PRESS_TIME_CONSTRAINT_SYS                                                                           \
    ((uint32_t)(CAN_ENGINE_PRESS_TIME_CONSTRAINT_MS / SMALLEST_TIME_UNIT_IN_MS))

#define CAN_POWER_VOLTAGE_FREQ_MS 100
#define CAN_POWER_VOLTAGE_FREQ_SYS ((uint32_t)(CAN_POWER_VOLTAGE_FREQ_MS / SMALLEST_TIME_UNIT_IN_MS))

#define CAN_NAV_GPS_FREQ_MS 200
#define CAN_NAV_GPS_FREQ_SYS ((uint32_t)(CAN_NAV_GPS_FREQ_MS / SMALLEST_TIME_UNIT_IN_MS))
#define CAN_NAV_GPS_POS_TIME_CONSTRAINT_SYS ((uint32_t)(CAN_NAV_GPS_POS_TIME_CONSTRAINT_MS / SMALLEST_TIME_UNIT_IN_MS))

#define CAN_DIAG_SYS_STAT_FREQ_MS 500
#define CAN_DIAG_SYS_STAT_FREQ_SYS ((uint32_t)(CAN_DIAG_SYS_STAT_FREQ_MS / SMALLEST_TIME_UNIT_IN_MS))

#define MAX_ELEMENTS_IN_ONE_CAN_FRAME (CAN_MAX_DATA_SIZE)

void CAN_ENGINE_init(void);
void CAN_ENGINE_addTemperatureData(const _Float32 temperature);
void CAN_ENGINE_addPressureData(const _Float32 pressure);
void CAN_ENGINE_handleTxQueueEmpty(void);

void CAN_POWER_init(void);
void CAN_POWER_addVoltageData(const uint16_t voltage);
void CAN_POWER_handleTxQueueEmpty(void);

void CAN_NAVIGATION_init(void);
void CAN_NAVIGATION_addGpsData(const uint64_t gpsPos);
void CAN_NAVIGATION_handleTxQueueEmpty(void);

void CAN_DIAGNOSIS_init(void);
void CAN_DIAGNOSIS_addSystemData(const uint32_t sysInfo);
void CAN_DIAGNOSIS_handleTxQueueEmpty(void);

typedef struct TxElement_s
{
    uint16_t canId;
    u8 size;
    u8 data[CAN_MAX_DATA_SIZE];
    u8 padding[5]; // We need to have sizeof that divides QUEUE_DATA_SIZE_IN_BYTES
} TxElement_s;

typedef struct SentFrame_s
{
    CanSenders_e sender;
    uint16_t canId;
    uint32_t busBusyTill;
    u8 noElementsInFrame;
    uint32_t messageLateTime[MAX_ELEMENTS_IN_ONE_CAN_FRAME]; // For measuring if message is late. 0 if time is OK.
} SentFrame_s;

// Sim time is measured in smallest data unit (time needed to send one bit)
static uint32_t simTime = 0;

static Queue_s txBufferEngineQ;
static Queue_s txBufferPowerQ;
static Queue_s txBufferNavQ;
static Queue_s txBufferDiagQ;

/*
 * Some data need to be delivered on certain time. To keep the time
 * it was delivered to their module CAN handler we create another queues with
 * delivered times of each message that has time constraint.
 * Messages with time constraints: engine temperature, engine pressure, gps position
 */
static Queue_s engineTempToEngineHandlerTsQ;
static Queue_s enginePressToEngineHandlerTsQ;
static Queue_s navGpsPosToNavHandlerTsQ;

static uint32_t noLateMessages = 0;

// Measure current sent message
static SentFrame_s messageOnBus = {.sender = CAN_UNKNOWN, .canId = CAN_ILLEGAL_ID};

/**
 * @brief Get current time in ms. This function is a mock for queueing algorithm.
 */
double getTime(void)
{
    return simTime * SMALLEST_TIME_UNIT_IN_MS;
}

/**
 * @brief Get time in ms from simulation time.
 */
double getTimeFromSimTs(uint32_t simTime)
{
    return simTime * SMALLEST_TIME_UNIT_IN_MS;
}

/**
 * @brief Initialise the simulation
 */
void SIMULATION_init(void)
{
    // For same results use same seed
    srand(2137);

    // Init tx queues
    queueInit(&txBufferEngineQ, sizeof(TxElement_s));
    queueInit(&txBufferPowerQ, sizeof(TxElement_s));
    queueInit(&txBufferNavQ, sizeof(TxElement_s));
    queueInit(&txBufferDiagQ, sizeof(TxElement_s));

    // Init time queues
    queueInit(&engineTempToEngineHandlerTsQ, sizeof(uint32_t));
    queueInit(&enginePressToEngineHandlerTsQ, sizeof(uint32_t));
    queueInit(&navGpsPosToNavHandlerTsQ, sizeof(uint32_t));

    // Init modules
    CAN_ENGINE_init();
    CAN_POWER_init();
    CAN_NAVIGATION_init();
    CAN_DIAGNOSIS_init();
}

/**
 * @brief Make simulation iteration. Send all necessary signals and update time
 */
void SIMULATION_iteration(void)
{
    // Add new data to be send

    if (simTime % CAN_ENGINE_TEMPERATURE_FREQ_SYS == 0)
    {
        /*
         * Engine temperature will be number 0.0 - 5000.0 celcius. This is only a test value,
         * any value would be fine
         */
        _Float32 engineTemp = ((_Float32)rand() / (_Float32)(RAND_MAX)) * 5000;
        push(&engineTempToEngineHandlerTsQ, &simTime);
        CAN_ENGINE_addTemperatureData(engineTemp);
    }

    if (simTime % CAN_ENGINE_PRESSURE_FREQ_SYS == 0)
    {
        /*
         * Engine pressure will be number 0.0 - 300.0 bars. This is only a test value,
         * any value would be fine
         */
        _Float32 enginePressure = ((_Float32)rand() / (_Float32)(RAND_MAX)) * 300;
        push(&enginePressToEngineHandlerTsQ, &simTime);
        CAN_ENGINE_addPressureData(enginePressure);
    }

    if (simTime % CAN_POWER_VOLTAGE_FREQ_SYS == 0)
    {
        /*
         * Power voltage will be number 0.0 - 30.0 V. This is only a test value,
         * any value would be fine
         */
        uint16_t powerVoltage = ((float)rand() / (float)(RAND_MAX)) * 30;
        CAN_POWER_addVoltageData(powerVoltage);
    }

    if (simTime % CAN_NAV_GPS_FREQ_SYS == 0)
    {
        /*
         * Navigation GPS position will be random 64 bits (whatever it converts to).
         * This is only a test value, any value would be fine
         */
        uint64_t rand1 = rand();
        uint64_t rand2 = rand();
        uint64_t gpsPos = rand1 ^ (rand2 << 32);
        push(&navGpsPosToNavHandlerTsQ, &simTime);
        CAN_NAVIGATION_addGpsData(gpsPos);
    }

    if (simTime % CAN_DIAG_SYS_STAT_FREQ_SYS == 0)
    {
        /*
         * Diagnostic system stats will be number up to RAND_MAX (whatever it converts to).
         * This is only a test value, any value would be fine
         */
        uint32_t sysStats = rand();
        CAN_DIAGNOSIS_addSystemData(sysStats);
    }

    if (messageOnBus.sender == CAN_UNKNOWN)
    {
        // Send data on CAN. We use known prio engine->power->nav->sys
        TxElement_s sendElement;
        uint16_t frameSize;
        if (txBufferEngineQ.size > 0)
        {
            pop(&txBufferEngineQ, &sendElement);
            frameSize = CAN_calculateFrameDurationBits(&sendElement.data[0], sendElement.canId, sendElement.size);

            uint32_t busBusyTill = simTime + frameSize;
            uint16_t canId = sendElement.canId;
            uint32_t recvTime;
            int64_t messageDelayTime = 0;
            // Sometimes one frame can have multiple elements
            if (canId == CAN_ID_ENGINE_TEMPERATURE)
            {
                u8 messageDataCnt = sendElement.size / CAN_ENGINE_TEMPERATURE_SIZE;
                for (u8 dataIdx = 0; dataIdx < messageDataCnt; dataIdx++)
                {
                    pop(&engineTempToEngineHandlerTsQ, &recvTime);
                    messageDelayTime = (int64_t)busBusyTill - (int64_t)recvTime - CAN_ENGINE_TEMP_TIME_CONSTRAINT_SYS;
                    messageOnBus.messageLateTime[dataIdx] = messageDelayTime > 0 ? messageDelayTime : 0;
                }
                messageOnBus.noElementsInFrame = messageDataCnt;
            }
            else if (canId == CAN_ID_ENGINE_PRESSURE)
            {
                u8 messageDataCnt = sendElement.size / CAN_ENGINE_PRESSURE_SIZE;
                for (u8 dataIdx = 0; dataIdx < messageDataCnt; dataIdx++)
                {
                    pop(&enginePressToEngineHandlerTsQ, &recvTime);
                    messageDelayTime = (int64_t)busBusyTill - (int64_t)recvTime - CAN_ENGINE_PRESS_TIME_CONSTRAINT_SYS;
                    messageOnBus.messageLateTime[dataIdx] = messageDelayTime > 0 ? messageDelayTime : 0;
                }
                messageOnBus.noElementsInFrame = messageDataCnt;
            }
            messageOnBus.busBusyTill = busBusyTill;
            messageOnBus.canId = canId;
            messageOnBus.sender = CAN_ENGINE;
        }
        else if (txBufferPowerQ.size > 0)
        {
            pop(&txBufferPowerQ, &sendElement);
            frameSize = CAN_calculateFrameDurationBits(&sendElement.data[0], sendElement.canId, sendElement.size);

            messageOnBus.busBusyTill = simTime + frameSize;
            messageOnBus.canId = sendElement.canId;
            messageOnBus.noElementsInFrame = 1;
            messageOnBus.messageLateTime[0] = 0;
            messageOnBus.sender = CAN_POWER;
        }
        else if (txBufferNavQ.size > 0)
        {
            pop(&txBufferNavQ, &sendElement);
            frameSize = CAN_calculateFrameDurationBits(&sendElement.data[0], sendElement.canId, sendElement.size);

            uint32_t busBusyTill = simTime + frameSize;
            uint16_t canId = sendElement.canId;
            uint32_t recvTime;
            int64_t messageDelayTime = 0;
            if (canId == CAN_ID_NAV_GPS_POS)
            {
                pop(&navGpsPosToNavHandlerTsQ, &recvTime);
                messageDelayTime = (int64_t)busBusyTill - (int64_t)recvTime - CAN_NAV_GPS_POS_TIME_CONSTRAINT_SYS;
            }
            messageDelayTime = messageDelayTime > 0 ? messageDelayTime : 0;
            messageOnBus.messageLateTime[0] = messageDelayTime;
            messageOnBus.busBusyTill = busBusyTill;
            messageOnBus.canId = canId;
            messageOnBus.sender = CAN_NAVIGATION;
            messageOnBus.noElementsInFrame = 1;
        }
        else if (txBufferDiagQ.size > 0)
        {
            pop(&txBufferDiagQ, &sendElement);
            frameSize = CAN_calculateFrameDurationBits(&sendElement.data[0], sendElement.canId, sendElement.size);

            messageOnBus.busBusyTill = simTime + frameSize;
            messageOnBus.canId = sendElement.canId;
            messageOnBus.messageLateTime[0] = 0;
            messageOnBus.sender = CAN_DIAGNOSIS;
            messageOnBus.noElementsInFrame = 1;
        }
    }

    // Manage end of frame transmission
    if (messageOnBus.sender != CAN_UNKNOWN && simTime >= messageOnBus.busBusyTill)
    {
        for (u8 elementIdx = 0; elementIdx < messageOnBus.noElementsInFrame; elementIdx++)
        {
            if (messageOnBus.messageLateTime[elementIdx] > 0)
            {
                noLateMessages++;
                printf("Late id:0x%03x,ts:%lf,timeLate:%lf\n", messageOnBus.canId, getTime(),
                       getTimeFromSimTs(messageOnBus.messageLateTime[elementIdx]));
            }
            else
            {
                printf("Delivered id:0x%03x,ts:%lf\n", messageOnBus.canId, getTime());
            }
        }

        switch (messageOnBus.sender)
        {
        case CAN_ENGINE:
            if (txBufferEngineQ.size == 0)
            {
                CAN_ENGINE_handleTxQueueEmpty();
            }
            break;
        case CAN_POWER:
            if (txBufferPowerQ.size == 0)
            {
                CAN_POWER_handleTxQueueEmpty();
            }
            break;
        case CAN_NAVIGATION:
            if (txBufferNavQ.size == 0)
            {
                CAN_NAVIGATION_handleTxQueueEmpty();
            }
            break;
        case CAN_DIAGNOSIS:
            if (txBufferDiagQ.size == 0)
            {
                CAN_DIAGNOSIS_handleTxQueueEmpty();
            }
            break;

        default:
            fprintf(stderr, "Uknown CAN module\n");
            break;
        }

        memset(&messageOnBus, 0, sizeof(messageOnBus));
        messageOnBus.sender = CAN_UNKNOWN;
        messageOnBus.canId = CAN_ILLEGAL_ID;
        messageOnBus.noElementsInFrame = 0;
    }

    if (simTime + (uint32_t)1 > simTime)
    {
        simTime++;
    }
    else
    {
        fprintf(stderr, "Simulation time limit exceeded!\n");
        exit(1);
    }
}

/**
 * @brief Send message on CAN. This is just a mock function for sim purposes
 *
 * @param[in]   data_p            Pointer to sent data
 * @param[in]   size              Data size in bytes
 * @param[in]   canId             Message CAN ID
 */
void CAN_putToTxBuffer(const u8 *const data_p, const u8 size, const uint16_t canId)
{
    if (size > CAN_MAX_DATA_SIZE)
    {
        fprintf(stderr, "Simulation error! Data size too large for CAN\n");
        exit(1);
    }

    if (canId > CAN_MAX_ID)
    {
        fprintf(stderr, "Simulation error! CAN ID size too large\n");
        exit(1);
    }

    TxElement_s element = {.canId = canId, .size = size};
    memcpy(&element.data, data_p, size);

    Queue_s *queue_p = NULL;

    switch (canId)
    {
    case CAN_ID_ENGINE_TEMPERATURE:
        queue_p = &txBufferEngineQ;
        break;
    case CAN_ID_ENGINE_PRESSURE:
        queue_p = &txBufferEngineQ;
        break;
    case CAN_ID_POWER_ACC_VOLTAGE:
        queue_p = &txBufferPowerQ;
        break;
    case CAN_ID_NAV_GPS_POS:
        queue_p = &txBufferNavQ;
        break;
    case CAN_ID_DIAGNOSIS_SYSTEM_STAT:
        queue_p = &txBufferDiagQ;
        break;
    default:
        fprintf(stderr, "Simulation error! CAN ID not known\n");
        exit(1);
        break;
    }

    if (queue_p->size == CAN_TX_BUFF_SIZE)
    {
        fprintf(stderr, "Simulation error! CAN tx buffer overflow\n");
        exit(1);
    }

    push(queue_p, &element);
}

/**
 * @brief Get tx queue current number of elements. This is just a mock function for sim
 *
 * @param[in]  module                    Module from which the request is sent
 */
u8 CAN_getTxQueueSize(CanSenders_e module)
{
    switch (module)
    {
    case CAN_ENGINE:
        return txBufferEngineQ.size;
    case CAN_POWER:
        return txBufferPowerQ.size;
    case CAN_NAVIGATION:
        return txBufferNavQ.size;
    case CAN_DIAGNOSIS:
        return txBufferDiagQ.size;
    default:
        fprintf(stderr, "CAN_getTxQueueSize, module doesn't exist\n");
        exit(1);
        break;
    }
}

int main(void)
{
    uint32_t simTicks = ~(uint32_t)0U;
    SIMULATION_init();
    while (simTime < simTicks)
    {
        SIMULATION_iteration();
    }
    return 0;
}
