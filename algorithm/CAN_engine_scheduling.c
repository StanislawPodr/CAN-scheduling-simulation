#include "queue.h"
#include "CAN_time.h"
#include "CAN_driver.h"

static Queue_s temperatureQ;
static Queue_s temperatureTimeQ;
static Queue_s pressureQ;
static Queue_s pressureTimeQ;

static u8 txQueueSize = 0;

static void fillTxQueue(void);

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
    //push(&temperatureTimeQ, &time);
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
    //push(&pressureTimeQ, &time);
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with pressure and temperature data
 */
void CAN_ENGINE_handleTxQueueEmpty(void)
{
    txQueueSize = 0;
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with data from voltageQ
 */
static void fillTxQueue(void)
{
    while (temperatureQ.size > 0 && txQueueSize < CAN_TX_BUFF_SIZE)
    {
        u8 buff[CAN_ENGINE_TEMPERATURE_SIZE];
        pop(&temperatureQ, &buff[0]);
        CAN_putToTxBuffer(&buff[0], temperatureQ.elementSize, CAN_ID_ENGINE_TEMPERATURE);
        txQueueSize++;
    }

    while (pressureQ.size > 0 && txQueueSize < CAN_TX_BUFF_SIZE)
    {
        u8 buff[CAN_ENGINE_PRESSURE_SIZE];
        pop(&pressureQ, &buff[0]);
        CAN_putToTxBuffer(&buff[0], pressureQ.elementSize, CAN_ID_ENGINE_PRESSURE);
        txQueueSize++;
    }
}
