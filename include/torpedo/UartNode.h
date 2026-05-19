#ifndef UART_NODE_H
#define UART_NODE_H

#include <string>
#include <vector>
#include "DataTypes.h"
#include "Protocol.h"

class UartNode {
private:
    int fd;
public:
    UartNode(const char* port, int baudrate);
    ~UartNode();

    // 통제소 -> 어뢰
    void sendDownlink(float target_x, float target_y,
                      float torpedo_x, float torpedo_y,
                      uint32_t timestamp_us, uint16_t seq);

    // 어뢰 -> 통제소
    bool receiveUplinkStatus(UplinkPacket& pkt);

    // 통제소 -> GUI (라이다 클러스터 + 어뢰 위치)
    void sendGuiPacket(float torpedo_x, float torpedo_y,
                   float yaw,                              // ← 추가
                   const std::vector<std::vector<PointGrid>>& clusters,
                   uint16_t seq);

    // GUI -> 통제소 명령 수신
    bool receiveGuiCommand(GuiCommandPacket& pkt);
};

#endif