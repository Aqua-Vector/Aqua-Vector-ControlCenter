#ifndef CRC_CALCULATOR_H
#define CRC_CALCULATOR_H

#include <cstdint>
#include <cstddef>

// ═══════════════════════════════════════════════
// CRC 체크섬 계산 유틸리티 클래스
// 통신 데이터 무결성 검증용
// ═══════════════════════════════════════════════
class CrcCalculator {
private:
    // CRC16 룩업 테이블 (고속 계산용)
    static const uint8_t crc16_h_table[256];
    static const uint8_t crc16_l_table[256];
    
    // CRC8 룩업 테이블
    static const uint8_t crc8_table[256];

public:
    // ═══════════════════════════════════════════════
    // 체크섬 합계 (Checksum Sum)
    // @param buf: 데이터 버퍼
    // @param len: 데이터 길이
    // @return: 8비트 체크섬 (256 - sum)
    // ═══════════════════════════════════════════════
    static uint8_t CalculateSum(const uint8_t* buf, size_t len);
    
    // ═══════════════════════════════════════════════
    // XOR 체크섬
    // @return: 8비트 XOR 결과
    // ═══════════════════════════════════════════════
    static uint8_t CalculateXor(const uint8_t* buf, size_t len);
    
    // ═══════════════════════════════════════════════
    // CRC8 체크섬
    // @return: 8비트 CRC
    // ═══════════════════════════════════════════════
    static uint8_t CalculateCrc8(const uint8_t* buf, size_t len);
    
    // ═══════════════════════════════════════════════
    // CRC16 체크섬 (Modbus RTU 표준)
    // @return: 16비트 CRC
    // ═══════════════════════════════════════════════
    static uint16_t CalculateCrc16(const uint8_t* buf, size_t len);
};

#endif