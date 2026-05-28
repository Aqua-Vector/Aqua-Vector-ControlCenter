#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <cstddef>
#include <cmath>
#include <vector>
#include <cstring>

#pragma pack(push, 1) 
// =============================================================
// [ 공통 데이터 구조체 ]
// =============================================================
struct ObstaclePoint {
    float x;  // m 단위
    float y;  // m 단위
};

struct ObstacleObject {
    uint16_t pt_count;
    std::vector<ObstaclePoint> points;
};

// =============================================================
// 1. 통제소(Zynq) -> 노트북 GUI 패킷 (가변 길이)
// =============================================================
// 고정 헤더 부분만 구조체로 정의 (가변 payload는 직접 직렬화)

struct GuiPacketHeader {
    uint8_t  sync;        // 0xAA 고정
    uint16_t seq;         // 순환 시퀀스
    uint16_t obj_count;   // 물체 수
    float    torpedo_x;   // m (없으면 NaN)
    float    torpedo_y;   // m (없으면 NaN)
    float    yaw;         // rad (없으면 NaN)
};

// =============================================================
// 2. 노트북 GUI -> 통제소(Zynq) 명령 패킷 (고정 15B 또는 8B)
// =============================================================
struct GuiCommandPacket {
    uint8_t  sync;     // 0xBB 고정
    uint16_t seq;
    uint8_t  type;     // 0x10=목표물, 0x11=개폐, 0x12=발사, 0x13=종말유도
    union {
        struct {       // type=0x10 목표물 좌표 설정 시
            float target_x;
            float target_y;
        };
        struct {       // type=0x11~0x13 명령 하달 시
            uint8_t cmd_data;
        };
    };
    uint16_t crc16;    // CRC16-CCITT
};

// =============================================================
// 3. 통제소 -> 어뢰 Downlink 패킷 (지연 보상 및 조향 포함 v2) - 총 28바이트
// =============================================================
struct DownlinkPacket {
    uint8_t  sync;           // 0xAA
    uint8_t  sync2;          // 0x55
    uint8_t  msg_id;         // 0x00
    uint8_t  length;         // payload 크기 (byte)
    uint16_t seq;            // [4-5]
    float    target_x;       // [6-9]
    float    target_y;       // [10-13]
    float    torpedo_x;      // [14-17]
    float    torpedo_y;      // [18-21]
    int16_t  steer;          // [22-23]
    uint8_t  flags;          // [24]
    uint16_t crc16;          // [25-26]
};

// =============================================================
// 4. 어뢰 -> 통제소 Uplink 패킷 - 총 26바이트
// =============================================================
struct UplinkPacket {
    uint8_t  sync;           // 0xAA
    uint8_t  sync2;          // 0x55
    uint8_t  msg_id;         // 0x00
    uint8_t  length;         // payload 크기 (byte)
    uint16_t seq;            // 시퀀스 번호
    float    p_x;            // 어뢰 현재 위치 X (m)
    float    p_y;            // 어뢰 현재 위치 Y (m)
    float    yaw;            // 어뢰 현재 Heading (rad)
    uint8_t  status_flags;   // 어뢰 내부 상태 플래그
    uint8_t  reserved;       // 바이트 정렬용 예비
    uint16_t crc16;          // CRC16-CCITT
};

// =============================================================
// [ 크기 검증 상수 (Compile-time Assert) ]
// =============================================================
static_assert(sizeof(DownlinkPacket) == 27, "DownlinkPacket size must be 27 bytes");
static_assert(sizeof(UplinkPacket) == 22, "UplinkPacket size must be 22 bytes");

#pragma pack(pop)


// =============================================================
// [ 프로토콜 매크로 및 비트 플래그 정의 ]
// =============================================================
// 1. HMI(GUI) -> 통제소(Zynq) 명령어 Type (TCP)
#define CMD_TARGET    0x10  // 목표물 좌표 설정
#define CMD_OPEN      0x11  // 개폐 명령
#define CMD_FIRE      0x12  // 발사 명령
#define CMD_ENDGUIDE  0x13  // 종말 유도 명령

// 2. 통제소(Zynq) -> 어뢰(Zynq) 유도 모드 Flag (UART 460800)
constexpr uint8_t FLAG_GUIDANCE_MIDCOURSE = 0x01; // 중기유도 (A* + LatencyAwareCtrl)
constexpr uint8_t FLAG_GUIDANCE_TERMINAL  = 0x02; // 종말유도 (어뢰에서 알아서 작동)
constexpr uint8_t FLAG_GUIDANCE_ARRIVE  = 0x03; // 정지 

// =============================================================
// [ 공통 유틸리티 클래스 ]
// =============================================================

class Protocol {
public:
    // CRC 계산
    static uint16_t calculateCRC16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    }

    // 직렬화 헬퍼 (NetworkNode와 UartNode에서 공통 사용)
    static void packFloat(std::vector<uint8_t>& buf, float val) {
        uint8_t tmp[4];
        std::memcpy(tmp, &val, 4);
        buf.insert(buf.end(), tmp, tmp + 4);
    }

    static void packU16(std::vector<uint8_t>& buf, uint16_t val) {
        buf.push_back(val & 0xFF);
        buf.push_back((val >> 8) & 0xFF);
    }
};

#endif // PROTOCOL_H