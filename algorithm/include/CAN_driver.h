#pragma once

#include <stdint.h>
#include "CAN_info.h"

// Simulation size of tx buffer
#define CAN_TX_BUFF_SIZE                    3

#define CAN_MAX_DATA_SIZE                   8

/**
 * @brief Send message on CAN
 *
 * @param[in]   data_p            Pointer to sent data
 * @param[in]   size              Data size in bytes
 * @param[in]   canId             Message CAN ID
 */
void CAN_putToTxBuffer(const u8 *const data_p, const u8 size, const uint16_t canId);
