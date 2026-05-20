#ifndef UART_NODE_H
#define UART_NODE_H

#include <string>
#include <vector>
#include "DataTypes.h"
#include "Protocol.h"

// ═══════════════════════════════════════════════
// UART 시리얼 통신 클래스
// 어뢰 통신(Downlink/Uplink) 및 GUI 통신 담당
// ═══════════════════════════════════════════════
class UartNode {
private:
    int fd;  // 파일 디스크립터 (시리얼 포트)

public:
    // ═══════════════════════════════════════════════
    // 생성자: 시리얼 포트 열기 및 설정
    // @param port: 디바이스 경로 (예: /dev/ttyPS0)
    // @param baudrate: 통신 속도 (예: 115200)
    // ═══════════════════════════════════════════════
    UartNode(const char* port, int baudrate);
    
    // ═══════════════════════════════════════════════
    // 소멸자: 포트 닫기
    // ═══════════════════════════════════════════════
    ~UartNode();

    // ═══════════════════════════════════════════════
    // 통제소 → 어뢰 Downlink 전송
    // @param target_x/y: 목표물 좌표 (m)
    // @param torpedo_x/y: 어뢰 현재 위치 (m, 라이다 추적)
    // @param timestamp_us: 타임스탬프 (마이크로초)
    // @param seq: 시퀀스 번호
    // ═══════════════════════════════════════════════
    void sendDownlink(float target_x, float target_y, 
                      float torpedo_x, float torpedo_y, 
                      int16_t steer, uint8_t flags, uint16_t seq);

    bool receiveUplinkStatus(UplinkPacket& pkt);

    void sendGuiPacket(float torpedo_x, float torpedo_y,
                       float yaw,
                       const std::vector<std::vector<PointGrid>>& clusters,
                       uint16_t seq);

    bool receiveGuiCommand(GuiCommandPacket& pkt);
};

#endif