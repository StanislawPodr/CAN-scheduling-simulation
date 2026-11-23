#include "CAN_driver.h"
#include "CAN_time.h"
#include "queue.h"

static void fillTxQueue(void);

static Queue_s voltageQ;
static u8 txQueueSize = 0;

/**
 * @brief Init Power CAN scheduling
 */
void CAN_POWER_init(void)
{
    queueInit(&voltageQ, CAN_POWER_ACC_VOLTAGE_SIZE);
}

/**
 * @brief Handle new (incoming) voltage
 *
 * @param[in]  voltage             Incoming power voltage
 */
void CAN_POWER_addVoltageData(const uint16_t voltage)
{
    push(&voltageQ, &voltage);
    fillTxQueue();
}

/**
 * @brief Empty tx buffer interrupt handler
 */
void CAN_POWER_handleTxQueueEmpty(void)
{
    txQueueSize = 0;
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with data from voltageQ
 */
static void fillTxQueue(void)
{
    while (voltageQ.size > 0 && txQueueSize < CAN_TX_BUFF_SIZE)
    {
        u8 buff[CAN_POWER_ACC_VOLTAGE_SIZE];
        pop(&voltageQ, &buff[0]);
        CAN_putToTxBuffer(&buff[0], voltageQ.elementSize, CAN_ID_POWER_ACC_VOLTAGE);
        txQueueSize++;
    }
}
