#include "CAN_driver.h"
#include "CAN_time.h"
#include "queue.h"

#define MAX_NO_PRESSURE_DATA_IN_FRAME (CAN_MAX_DATA_SIZE / CAN_ENGINE_PRESSURE_SIZE)
#define MAX_NO_TEMPERATURE_DATA_IN_FRAME (CAN_MAX_DATA_SIZE / CAN_ENGINE_TEMPERATURE_SIZE)

static Queue_s temperatureQ;
static Queue_s temperatureTimeQ;
static Queue_s pressureQ;
static Queue_s pressureTimeQ;

static void fillTxQueue(void);
static void addPressToTxQueue(void);
static void addTempToTxQueue(void);

/**
 * @brief Init Engine data CAN scheduling
 */
void CAN_ENGINE_init(void)
{
    queueInit(&temperatureQ, CAN_ENGINE_TEMPERATURE_SIZE);
    queueInit(&temperatureTimeQ, sizeof(getTime()));
    queueInit(&pressureQ, CAN_ENGINE_PRESSURE_SIZE);
    queueInit(&pressureTimeQ, sizeof(getTime()));
}

/**
 * @brief Handle new (incoming) temperature data
 *
 * @param[in]  temperature             Incoming temperature data
 */
void CAN_ENGINE_addTemperatureData(const _Float32 temperature)
{
    push(&temperatureQ, &temperature);
    typeof(getTime()) time = getTime();
    push(&temperatureTimeQ, &time);
    fillTxQueue();
}

/**
 * @brief Handle new (incoming) pressure data
 *
 * @param[in]  pressure                Incoming pressure data
 */
void CAN_ENGINE_addPressureData(const _Float32 pressure)
{
    push(&pressureQ, &pressure);
    typeof(getTime()) time = getTime();
    push(&pressureTimeQ, &time);
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with pressure and temperature data
 */
void CAN_ENGINE_handleTxQueueEmpty(void)
{
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with data from pressure and temperature
 */
static void fillTxQueue(void)
{

    u8 txQueueSize = CAN_getTxQueueSize(CAN_ENGINE);
    while (txQueueSize < CAN_TX_BUFF_SIZE && (temperatureQ.size > 0 || pressureQ.size > 0))
    {
        if (temperatureQ.size > 0 && pressureQ.size > 0)
        {
            // Decide which next based on estimated delivery time

            double currTime = getTime();
            double timeTemp;
            double timePress;

            peek(&temperatureTimeQ, &timeTemp);
            peek(&pressureTimeQ, &timePress);

            double timeLeftTemp = CAN_ENGINE_TEMP_TIME_CONSTRAINT_MS + timeTemp - currTime;
            double timeLeftPress = CAN_ENGINE_PRESS_TIME_CONSTRAINT_MS + timePress - currTime;

            if (timeLeftTemp < timeLeftPress)
            {
                addTempToTxQueue();
            }
            else
            {
                addPressToTxQueue();
            }
        }
        else if (temperatureQ.size > 0)
        {
            addTempToTxQueue();
        }
        else // pressureQ.size > 0
        {
            addPressToTxQueue();
        }
        txQueueSize++;
    }
}

/**
 * @brief Send one message to tx buffer. The message can possibly have
 *   up to MAX_NO_TEMPERATURE_DATA_IN_FRAME temperatures
 */
static void addTempToTxQueue(void)
{
    // We can merge more than one information in one message (by this we save up some bus time)

    u8 tempToPutToFrame =
        temperatureQ.size > MAX_NO_TEMPERATURE_DATA_IN_FRAME ? MAX_NO_TEMPERATURE_DATA_IN_FRAME : temperatureQ.size;

    u8 mergedTempBuff[CAN_MAX_DATA_SIZE];
    for (u8 tempIdx = 0; tempIdx < tempToPutToFrame; tempIdx++)
    {
        u8 tempInfo[CAN_ENGINE_TEMPERATURE_SIZE];
        pop(&temperatureQ, &tempInfo[0]);
        removeTop(&temperatureTimeQ);

        memcpy(&mergedTempBuff[CAN_ENGINE_TEMPERATURE_SIZE * tempIdx], &tempInfo[0], sizeof(tempInfo));
    }

    CAN_putToTxBuffer(&mergedTempBuff[0], tempToPutToFrame * CAN_ENGINE_TEMPERATURE_SIZE, CAN_ID_ENGINE_TEMPERATURE);
}

/**
 * @brief Send one message to tx buffer. The message can possibly have
 *   up to MAX_NO_PRESSURE_DATA_IN_FRAME pressures
 */
static void addPressToTxQueue(void)
{
    // We can merge more than one information in one message (by this we save up some bus time)

    u8 pressToPutToFrame =
        pressureQ.size > MAX_NO_PRESSURE_DATA_IN_FRAME ? MAX_NO_PRESSURE_DATA_IN_FRAME : pressureQ.size;

    u8 mergedPressBuff[CAN_MAX_DATA_SIZE];
    for (u8 pressIdx = 0; pressIdx < pressToPutToFrame; pressIdx++)
    {
        u8 pressInfo[CAN_ENGINE_PRESSURE_SIZE];
        pop(&pressureQ, &pressInfo[0]);
        removeTop(&pressureTimeQ);

        memcpy(&mergedPressBuff[CAN_ENGINE_PRESSURE_SIZE * pressIdx], &pressInfo[0], sizeof(pressInfo));
    }

    CAN_putToTxBuffer(&mergedPressBuff[0], pressToPutToFrame * CAN_ENGINE_PRESSURE_SIZE, CAN_ID_ENGINE_PRESSURE);
}
