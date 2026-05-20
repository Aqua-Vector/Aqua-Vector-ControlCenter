#ifndef NETWORK_NODE_H
#define NETWORK_NODE_H

#include <stdint.h>
#include <vector>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "Protocol.h"
#include "DataTypes.h"

// ═══════════════════════════════════════════════
// 네트워크 통신 클래스
// UDP: 실시간 데이터 전송 (Zynq ↔ 노트북)
// TCP: 제어 명령 전송 (노트북 → Zynq)
// ═══════════════════════════════════════════════
class NetworkNode {
private:
    // ─────────────────────────────────────────
    // UDP 소켓 (데이터그램, 비연결형)
    // ─────────────────────────────────────────
    int udp_tx_fd;                      // Zynq → 노트북 송신용
    int udp_rx_fd;                      // 노트북 → Zynq 수신용
    struct sockaddr_in udp_tx_addr;     // 노트북 IP/포트 주소

    // ─────────────────────────────────────────
    // TCP 소켓 (스트림, 연결형)
    // ─────────────────────────────────────────
    int tcp_server_fd;   // TCP 서버 소켓 (listen)
    int tcp_client_fd;   // 연결된 클라이언트 소켓

public:
    // ═══════════════════════════════════════════════
    // 생성자: UDP/TCP 소켓 초기화
    // @param pc_ip: 노트북 IP 주소 (UDP TX 목적지)
    // @param udp_tx_port: Zynq→노트북 UDP 포트
    // @param udp_rx_port: 노트북→Zynq UDP 포트
    // @param tcp_port: TCP 서버 포트
    // ═══════════════════════════════════════════════
    NetworkNode(
        const char* pc_ip,
        int udp_tx_port,
        int udp_rx_port,
        int tcp_port
    );
    
    // ═══════════════════════════════════════════════
    // 소멸자: 모든 소켓 닫기
    // ═══════════════════════════════════════════════
    ~NetworkNode();

    // ═══════════════════════════════════════════════
    // UDP: Zynq → 노트북 패킷 전송
    // UART sendGuiPacket()과 동일한 직렬화 포맷
    // @param torpedo_x/y: 어뢰 위치 (m)
    // @param yaw: 어뢰 방향각 (rad)
    // @param clusters: 라이다 클러스터
    // @param seq: 시퀀스 번호
    // ═══════════════════════════════════════════════
    void sendGuiPacketUDP(float torpedo_x, float torpedo_y,
                          float yaw,
                          const std::vector<std::vector<PointGrid>>& clusters,
                          uint16_t seq);

    // ═══════════════════════════════════════════════
    // UDP: 노트북 → Zynq 목표물 좌표 수신
    // @param pkt: 수신 패킷 저장 버퍼
    // @return: true=수신 성공, false=데이터 없음
    // ═══════════════════════════════════════════════
    bool receiveTargetUDP(GuiCommandPacket& pkt);

    // ═══════════════════════════════════════════════
    // TCP: 클라이언트 접속 대기 (블로킹)
    // 별도 스레드에서 실행 필요
    // 무한 루프로 재접속 처리
    // ═══════════════════════════════════════════════
    void acceptTcpClient();

    // ═══════════════════════════════════════════════
    // TCP: 명령 수신 (Non-blocking)
    // @param pkt: 수신 명령 저장 버퍼
    // @return: true=수신 성공, false=데이터 없음
    // ═══════════════════════════════════════════════
    bool receiveCommandTCP(GuiCommandPacket& pkt);

    // ═══════════════════════════════════════════════
    // TCP 연결 상태 확인
    // @return: true=클라이언트 연결됨, false=연결 없음
    // ═══════════════════════════════════════════════
    bool isTcpConnected() { return tcp_client_fd >= 0; }
};

#endif