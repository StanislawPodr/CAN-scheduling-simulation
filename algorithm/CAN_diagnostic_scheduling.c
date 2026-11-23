#include "CAN_driver.h"
#include "CAN_time.h"
#include "queue.h"

static void fillTxQueue(void);

static Queue_s sysStatusQ;
static u8 txQueueSize = 0;

/**
 * @brief Init diagnostic data CAN scheduling
 */
void CAN_DIAGNOSIS_init(void)
{
    queueInit(&sysStatusQ, CAN_DIAGNOSIS_SYSTEM_STAT_SIZE);
}

/**
 * @brief Handle new (incoming) system status data
 *
 * @param[in]  sysInfo             Incoming system data
 */
void CAN_DIAGNOSIS_addSystemData(const uint32_t sysInfo)
{
    push(&sysStatusQ, &sysInfo);
    fillTxQueue();
}

/**
 * @brief Empty tx buffer interrupt handler
 */
void CAN_DIAGNOSIS_handleTxQueueEmpty(void)
{
    txQueueSize = 0;
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with data from sysStatusQ
 */
static void fillTxQueue(void)
{
    while (sysStatusQ.size > 0 && txQueueSize < CAN_TX_BUFF_SIZE)
    {
        u8 buff[CAN_DIAGNOSIS_SYSTEM_STAT_SIZE];
        pop(&sysStatusQ, &buff[0]);
        CAN_putToTxBuffer(&buff[0], sysStatusQ.elementSize, CAN_ID_DIAGNOSIS_SYSTEM_STAT);
        txQueueSize++;
    }
}
