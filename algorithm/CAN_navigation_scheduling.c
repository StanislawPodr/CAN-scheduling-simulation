#include "CAN_driver.h"
#include "CAN_time.h"
#include "queue.h"

static void fillTxQueue(void);

static Queue_s gpsQ;
static u8 txQueueSize = 0;

/**
 * @brief Init Navigation CAN scheduling
 */
void CAN_NAVIGATION_init(void)
{
    queueInit(&gpsQ, CAN_NAV_GPS_POS_SIZE);
}

/**
 * @brief Handle new (incoming) gps position
 *
 * @param[in]  voltage             Incoming gps position
 */
void CAN_NAVIGATION_addGpsData(const uint64_t gpsPos)
{
    push(&gpsQ, &gpsPos);
    fillTxQueue();
}

/**
 * @brief Empty tx buffer interrupt handler
 */
void CAN_NAVIGATION_handleTxQueueEmpty(void)
{
    txQueueSize = 0;
    fillTxQueue();
}

/**
 * @brief Fill Tx queue with data from gpsQ
 */
static void fillTxQueue(void)
{
    while (gpsQ.size > 0 && txQueueSize < CAN_TX_BUFF_SIZE)
    {
        u8 buff[CAN_NAV_GPS_POS_SIZE];
        pop(&gpsQ, &buff[0]);
        CAN_putToTxBuffer(&buff[0], gpsQ.elementSize, CAN_ID_NAV_GPS_POS);
        txQueueSize++;
    }
}
