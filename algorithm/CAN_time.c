#include <stdint.h>

#include "CAN_time.h"

static inline void calculateCrcNextBit(const u8 nextBit, uint16_t *crc_p);
static inline void handleStuffing(u8 *lastFiveBits_p, const u8 nextBit, uint16_t *noBits_p);


/**
 * @brief Calculates CAN frame number of bits
 *
 * Base frame = 47 + 8*DLC bits + stuff bits:
 * - Stuffed zone (34 + 8*DLC bits): SOF(1 x 0) + ID(11) + RTR(1 x 0) +
 *                  IDE(1x0) + r0(1 x 0) + DLC(4) + Data(8*DLC) + CRC(15).
 *
 * One opposite stuff bit is inserted after every 5 consecutive
 * identical bits.
 * - Unstuffed tail (13 bits): CRC_DEL(1) + ACK(2) + EOF(7) + Intermission(3).
 * @param[in]  data_p           Pointer to data
 * @param[in]  noBytes          Number of data bytes
 * @param[in]  canId            CAN ID with 11 bits LSB as ID
 */
uint32_t CAN_calculateFrameDurationBits(const u8 *data_p, const uint16_t canId, const u8 noBytes)
{
    uint16_t crc = 0;
    // To not get stuffing from start we start with 5bits set to 1 (SOF is zero)
    u8 lastFiveBits = ~((u8)0);
    // We start from SOF
    uint16_t noBits = 1;
    handleStuffing(&lastFiveBits, 0, &noBits);
    calculateCrcNextBit(0, &crc);

    // Add 11 ID bits
    noBits += 11;
    for (int8_t idBitIdx = 10; idBitIdx >= 0; idBitIdx--)
    {
        u8 bit = (canId >> idBitIdx) & 1;
        handleStuffing(&lastFiveBits, bit, &noBits);
        calculateCrcNextBit(bit, &crc);
    }

    // RTR bit (1 x 0)
    noBits++;
    handleStuffing(&lastFiveBits, 0, &noBits);
    calculateCrcNextBit(0, &crc);

    // IDE bit (1 x 0)
    noBits++;
    handleStuffing(&lastFiveBits, 0, &noBits);
    calculateCrcNextBit(0, &crc);

    // r0 bit (1 x 0)
    noBits++;
    handleStuffing(&lastFiveBits, 0, &noBits);
    calculateCrcNextBit(0, &crc);

    // DLC bits (4)
    noBits += 4;
    for (int8_t dlcBitIdx = 3; dlcBitIdx >= 0; dlcBitIdx--)
    {
        u8 bit = (noBytes >> dlcBitIdx) & 1;
        handleStuffing(&lastFiveBits, bit, &noBits);
        calculateCrcNextBit(bit, &crc);
    }

    // Data bits (8 * DLC)
    noBits += noBytes * 8;
    for (int8_t dataByteIdx = 0; dataByteIdx < noBytes; dataByteIdx++)
    {
        for (int8_t byteBitIdx = 7; byteBitIdx >= 0; byteBitIdx--)
        {
            u8 bit = (data_p[dataByteIdx] >> byteBitIdx) & 1;
            handleStuffing(&lastFiveBits, bit, &noBits);
            calculateCrcNextBit(bit, &crc);
        }
    }

    // Calculated CRC bits (15)
    noBits += 15;
    for (int8_t crcBitIdx = 14; crcBitIdx >= 0; crcBitIdx--)
    {
        u8 bit = (crc >> crcBitIdx) & 1;
        handleStuffing(&lastFiveBits, bit, &noBits);
    }

    // Additional tailing 13 bits CRC_DEL(1) + ACK(2) + EOF(7) + Intermission(3)
    noBits += 13;

    return noBits;
}

/**
 * @brief Calculates CAN frame transmission time in ms
 *
 * Base frame = 47 + 8*DLC bits + stuff bits:
 * - Stuffed zone (34 + 8*DLC bits): SOF(1 x 0) + ID(11) + RTR(1 x 0) +
 *                  IDE(1x0) + r0(1 x 0) + DLC(4) + Data(8*DLC) + CRC(15).
 *
 * One opposite stuff bit is inserted after every 5 consecutive
 * identical bits.
 * - Unstuffed tail (13 bits): CRC_DEL(1) + ACK(2) + EOF(7) + Intermission(3).
 * @param[in]  data_p           Pointer to data
 * @param[in]  noBytes          Number of data bytes
 * @param[in]  canId            CAN ID with 11 bits LSB as ID
 */
double CAN_calculateFrameDurationMs(const u8 *data_p, const uint16_t canId, const u8 noBytes)
{
    return CAN_calculateFrameDurationBits(data_p, canId, noBytes) * SMALLEST_TIME_UNIT_IN_MS;
}

/**
 * @brief Calculate CAN CRC seq
 *
 * @param[in]  bit              Next bit used for CRC calculation
 * @param[out]  crc_p           Pointer to current counted CRC
 *
 */
static inline void calculateCrcNextBit(const u8 nextBit, uint16_t *crc_p)
{
    uint16_t crc = *crc_p;
    u8 crcNext = nextBit ^ ((crc >> 14) & 1);
    crc = (crc << 1) & 0x7FFF;
    if (crcNext)
    {
        crc ^= 0x4599;
    }
    *crc_p = crc;
}

/**
 * @brief Calculate additional stuffing bit for next
 *
 * @param[out] lastFiveBits_p    Pointer to last 5 bits (if same apply stuffing)
 * @param[in]  nextBit           Next bit to see if last 5 need stuffing
 * @param[out] noBits_p          Pointer to number of bits (incremented if stuff
 *                               bit added)
 */
static inline void handleStuffing(u8 *lastFiveBits_p, const u8 nextBit, uint16_t *noBits_p)
{
    uint16_t noBits = *noBits_p;
    u8 lastFiveBits = *lastFiveBits_p;

    // Add next bit. We keep 5 latest bits
    lastFiveBits = (lastFiveBits << 1) | nextBit;
    lastFiveBits &= 0x1F;

    u8 stuffNeeded = ((lastFiveBits + 1U) & 0x1F) < 2;
    lastFiveBits = (lastFiveBits << stuffNeeded) | (stuffNeeded & ((~lastFiveBits) & 1));
    noBits += stuffNeeded;

    *noBits_p = noBits;
    *lastFiveBits_p = lastFiveBits;
}
