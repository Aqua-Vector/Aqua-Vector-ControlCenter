#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <cstddef>
#include <cmath>
#include <vector>

#pragma pack(push, 1)

// ─────────────────────────────────────────
// 통제소(Zynq) -> 노트북 GUI (가변길이)
// ─────────────────────────────────────────
struct ObstaclePoint {
    float x;  // m 단위
    float y;  // m 단위
};

struct ObstacleObject {
    uint16_t pt_count;
    std::vector<ObstaclePoint> points;
};

// 고정 헤더 부분만 구조체로 (가변 payload는 직접 직렬화)
struct GuiPacketHeader {
    uint8_t  sync;        // 0xAA
    uint16_t seq;         // 순환 시퀀스
    uint16_t obj_count;   // 물체 수  ← torpedo_x/y 앞으로 이동
    float    torpedo_x;   // m (없으면 NaN)
    float    torpedo_y;   // m (없으면 NaN)
    float    yaw;         // rad (없으면 NaN) ← 추가
};

// ─────────────────────────────────────────
// 노트북 GUI -> 통제소(Zynq)
// ─────────────────────────────────────────
struct GuiCommandPacket {
    uint8_t  sync;     // 0xBB
    uint16_t seq;
    uint8_t  type;     // 0x10=목표물, 0x11=개폐, 0x12=발사, 0x13=종말유도
    union {
        struct {       // type=0x10 목표물 좌표
            float target_x;
            float target_y;
        };
        struct {       // type=0x11~0x13 명령
            uint8_t cmd_data;
        };
    };
    uint16_t crc16;
};

// ─────────────────────────────────────────
// 통제소 -> 어뢰 Downlink (기존 유지)
// ─────────────────────────────────────────
struct DownlinkPacket {
    uint8_t  sync;
    uint32_t timestamp_us;
    uint16_t seq;
    float    target_x;
    float    target_y;
    float    torpedo_x;
    float    torpedo_y;
    uint16_t crc16;
};

// ─────────────────────────────────────────
// 어뢰 -> 통제소 Uplink (기존 유지)
// ─────────────────────────────────────────
struct UplinkPacket {
    uint8_t  sync;
    uint32_t timestamp_us;
    uint16_t seq;
    float    p_x;
    float    p_y;
    float    yaw;
    uint8_t  status_flags;
    uint8_t  reserved;
    uint16_t crc16;
};

#pragma pack(pop)

// Type 정의
#define PKT_TYPE_TARGET   0x10
#define PKT_TYPE_OPEN     0x11
#define PKT_TYPE_FIRE     0x12
#define PKT_TYPE_ENDGUIDE 0x13

class Protocol {
public:
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
};

#endif