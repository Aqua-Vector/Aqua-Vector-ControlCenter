#ifndef CRC_CALCULATOR_H
#define CRC_CALCULATOR_H

#include <cstdint>
#include <cstddef>

class CrcCalculator {
private:
	static const uint8_t crc16_h_table[256];
	static const uint8_t crc16_l_table[256];
	static const uint8_t crc8_table[256];

public:
	static uint8_t CalculateSum(const uint8_t* buf, size_t len);
	static uint8_t CalculateXor(const uint8_t* buf, size_t len);
	static uint8_t CalculateCrc8(const uint8_t* buf, size_t len);
	static uint16_t CalculateCrc16(const uint8_t* buf, size_t len);
};

#endif /* CRC_CALCULATOR_HPP_ */
