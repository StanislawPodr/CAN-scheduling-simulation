#pragma once

#include <stdint.h>
#include "CAN_info.h"

double getTime(void);
double CAN_calculateFrameDurationMs(const u8 *data_p, const uint16_t canId, const u8 noBytes);
uint32_t CAN_calculateFrameDurationBits(const u8 *data_p, const uint16_t canId, const u8 noBytes);
