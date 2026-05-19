#ifndef NETWORK_NODE_H
#define NETWORK_NODE_H

#include <stdint.h>
#include <vector>
#include <string>
#include <arpa/inet.h>      // ← 추가 (sockaddr_in)
#include <sys/socket.h>     // ← 추가
#include <netinet/in.h>     // ← 추가
#include "Protocol.h"
#include "DataTypes.h"

class NetworkNode {
private:
    // UDP
    int udp_tx_fd;   // Zynq → 노트북 송신용
    int udp_rx_fd;   // 노트북 → Zynq 수신용
    struct sockaddr_in udp_tx_addr; // 노트북 주소

    // TCP
    int tcp_server_fd;  // TCP 서버 소켓
    int tcp_client_fd;  // 연결된 클라이언트

public:
    NetworkNode(
        const char* pc_ip,        // 노트북 IP
        int udp_tx_port,          // Zynq→PC UDP 포트
        int udp_rx_port,          // PC→Zynq UDP 포트
        int tcp_port              // TCP 서버 포트
    );
    ~NetworkNode();

    // UDP: Zynq → 노트북 (라이다+어뢰위치)
    void sendGuiPacketUDP(float torpedo_x, float torpedo_y,
                          float yaw,
                          const std::vector<std::vector<PointGrid>>& clusters,
                          uint16_t seq);

    // UDP: 노트북 → Zynq (목표물 좌표) 수신
    bool receiveTargetUDP(GuiCommandPacket& pkt);

    // TCP: 클라이언트 접속 대기 (별도 스레드에서 호출)
    void acceptTcpClient();

    // TCP: 명령 수신
    bool receiveCommandTCP(GuiCommandPacket& pkt);

    bool isTcpConnected() { return tcp_client_fd >= 0; }
};

#endif