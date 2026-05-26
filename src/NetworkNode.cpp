#include "NetworkNode.h"
#include <iostream>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>

// ═══════════════════════════════════════════════
// 생성자: UDP/TCP 소켓 초기화
// ═══════════════════════════════════════════════
NetworkNode::NetworkNode(const char* pc_ip,
                         int udp_tx_port,
                         int udp_rx_port,
                         int tcp_port)
    : udp_tx_fd(-1), udp_rx_fd(-1),
      tcp_server_fd(-1), tcp_client_fd(-1)
{
    // ─────────────────────────────────────────
    // UDP TX 소켓 생성 (Zynq → 노트북)
    // ─────────────────────────────────────────
    udp_tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_tx_fd < 0) {
        fprintf(stderr, "  [Network Error] UDP TX socket failed: %s\n", 
                strerror(errno));
        return;
    }
    
    // 노트북 주소 설정
    memset(&udp_tx_addr, 0, sizeof(udp_tx_addr));
    udp_tx_addr.sin_family      = AF_INET;
    udp_tx_addr.sin_port        = htons(udp_tx_port);
    udp_tx_addr.sin_addr.s_addr = inet_addr(pc_ip);
    
    fprintf(stderr, "  [Network] UDP TX -> %s:%d OK\n", pc_ip, udp_tx_port);
    fflush(stderr);

    // ─────────────────────────────────────────
    // UDP RX 소켓 생성 (노트북 → Zynq)
    // ─────────────────────────────────────────
    udp_rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_rx_fd < 0) {
        fprintf(stderr, "  [Network Error] UDP RX socket failed: %s\n", 
                strerror(errno));
        return;
    }
    
    // 수신 주소 바인딩 (모든 인터페이스)
    struct sockaddr_in rx_addr;
    memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family      = AF_INET;
    rx_addr.sin_port        = htons(udp_rx_port);
    rx_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(udp_rx_fd, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
        fprintf(stderr, "  [Network Error] UDP RX bind failed: %s\n", 
                strerror(errno));
        return;
    }
    
    // Non-blocking 모드 (recv 대기 안 함)
    fcntl(udp_rx_fd, F_SETFL, O_NONBLOCK);
    
    fprintf(stderr, "  [Network] UDP RX port %d OK\n", udp_rx_port);
    fflush(stderr);

    // ─────────────────────────────────────────
    // TCP 서버 소켓 생성
    // ─────────────────────────────────────────
    tcp_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_fd < 0) {
        fprintf(stderr, "  [Network Error] TCP socket failed: %s\n", 
                strerror(errno));
        return;
    }
    
    // SO_REUSEADDR: 포트 재사용 허용 (빠른 재시작)
    int opt = 1;
    setsockopt(tcp_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // 서버 주소 설정
    struct sockaddr_in tcp_addr;
    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family      = AF_INET;
    tcp_addr.sin_port        = htons(tcp_port);
    tcp_addr.sin_addr.s_addr = INADDR_ANY;
    
    // 포트 바인딩
    if (bind(tcp_server_fd, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
        fprintf(stderr, "  [Network Error] TCP bind failed: %s\n", 
                strerror(errno));
        return;
    }
    
    // 리스닝 시작 (대기열 크기 1)
    if (listen(tcp_server_fd, 1) < 0) {
        fprintf(stderr, "  [Network Error] TCP listen failed: %s\n", 
                strerror(errno));
        return;
    }
    
    fprintf(stderr, "  [Network] TCP server port %d OK\n", tcp_port);
    fflush(stderr);
}

// ═══════════════════════════════════════════════
// 소멸자: 모든 소켓 닫기
// ═══════════════════════════════════════════════
NetworkNode::~NetworkNode() {
    if (udp_tx_fd  >= 0) close(udp_tx_fd);
    if (udp_rx_fd  >= 0) close(udp_rx_fd);
    if (tcp_client_fd >= 0) close(tcp_client_fd);
    if (tcp_server_fd >= 0) close(tcp_server_fd);
}

// ═══════════════════════════════════════════════
// UDP: Zynq → 노트북 패킷 전송
// UART sendGuiPacket()과 동일한 직렬화 방식
// ═══════════════════════════════════════════════
void NetworkNode::sendGuiPacketUDP(float torpedo_x, float torpedo_y,
                                    float yaw,
                                    const std::vector<std::vector<PointGrid>>& clusters,
                                    uint16_t seq) {
    std::vector<uint8_t> buf;

    // ─────────────────────────────────────────
    // 헤더 직렬화
    // ─────────────────────────────────────────
    buf.push_back(0xAA);
    Protocol::packU16(buf, seq);
    Protocol::packU16(buf, (uint16_t)clusters.size());
    Protocol::packFloat(buf, torpedo_x);
    Protocol::packFloat(buf, torpedo_y);
    Protocol::packFloat(buf, yaw);

    // ─────────────────────────────────────────
    // 페이로드 직렬화
    // ─────────────────────────────────────────
        for (const auto& cluster : clusters) {
        Protocol::packU16(buf, (uint16_t)cluster.size());
        for (const auto& pg : cluster) {
            Protocol::packFloat(buf, pg.x);
                        Protocol::packFloat(buf, pg.y);
        }
    }

    // ─────────────────────────────────────────
    // CRC 추가
    // ─────────────────────────────────────────
        uint16_t crc = Protocol::calculateCRC16(buf.data(), buf.size());
    Protocol::packU16(buf, crc);

    // ─────────────────────────────────────────
    // UDP 전송 (비연결형, sendto)
    // ─────────────────────────────────────────
    sendto(udp_tx_fd, buf.data(), buf.size(), 0,
           (struct sockaddr*)&udp_tx_addr, sizeof(udp_tx_addr));
}

// ═══════════════════════════════════════════════
// UDP: 노트북 → Zynq 목표물 좌표 수신
// @return: true=수신 성공, false=데이터 없음
// ═══════════════════════════════════════════════
bool NetworkNode::receiveTargetUDP(GuiCommandPacket& pkt) {
    uint8_t buf[256];
    
    // Non-blocking recv
    ssize_t n = recv(udp_rx_fd, buf, sizeof(buf), 0);
    if (n <= 0) return false;

    // Sync 확인
    if (buf[0] != 0xBB) return false;
    
    // 구조체로 복사
    memcpy(&pkt, buf, sizeof(GuiCommandPacket));
    return true;
}

// ═══════════════════════════════════════════════
// TCP: 클라이언트 접속 대기 (블로킹)
// 별도 스레드에서 실행 필요
// 무한 루프로 재접속 처리
// ═══════════════════════════════════════════════
void NetworkNode::acceptTcpClient() {
    while (true) {
        std::cout << "[TCP] Waiting for client..." << std::endl;
        
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        
        // accept() 호출 (블로킹, 연결 대기)
        int new_fd = accept(tcp_server_fd,
                            (struct sockaddr*)&client_addr, &len);
        
        if (new_fd >= 0) {
            // 기존 클라이언트 종료 (1:1 연결만 유지)
            if (tcp_client_fd >= 0) close(tcp_client_fd);
            
            tcp_client_fd = new_fd;
            std::cout << "[TCP] Client connected: "
                      << inet_ntoa(client_addr.sin_addr) << std::endl;
        }
    }
}

// ═══════════════════════════════════════════════
// TCP: 명령 수신 (Non-blocking)
// @return: true=수신 성공, false=데이터 없음
// ═══════════════════════════════════════════════
bool NetworkNode::receiveCommandTCP(GuiCommandPacket& pkt) {
    if (tcp_client_fd < 0) return false;

    uint8_t head;
    
    // ─────────────────────────────────────────
    // 1. Sync 바이트 읽기 (Non-blocking)
    // ─────────────────────────────────────────
    int n = recv(tcp_client_fd, &head, 1, MSG_DONTWAIT);
    if (n <= 0 || head != 0xBB) return false;

    // ─────────────────────────────────────────
    // 2. 공통 헤더 읽기: seq(2) + type(1)
    // ─────────────────────────────────────────
    uint8_t buf[3];
    if (recv(tcp_client_fd, buf, 3, 0) != 3) return false;

    pkt.sync = 0xBB;
    pkt.seq  = buf[0] | (buf[1] << 8);
    pkt.type = buf[2];

    // ─────────────────────────────────────────
    // 3. Type별 페이로드 읽기
    // ─────────────────────────────────────────
    if (pkt.type == CMD_TARGET) {
        // 목표물 좌표: target_x(4) + target_y(4) + crc(2)
        uint8_t payload[10];
        if (recv(tcp_client_fd, payload, 10, 0) != 10) return false;
        memcpy(&pkt.target_x, payload, 4);
        memcpy(&pkt.target_y, payload + 4, 4);
        pkt.crc16 = payload[8] | (payload[9] << 8);
    } else {
        // 단순 명령: cmd_data(1) + crc(2)
        uint8_t payload[3];
        if (recv(tcp_client_fd, payload, 3, 0) != 3) return false;
        pkt.cmd_data = payload[0];
        pkt.crc16 = payload[1] | (payload[2] << 8);
    }

    // ─────────────────────────────────────────
    // 디버깅: 수신 패킷 로그
    // ─────────────────────────────────────────
    fprintf(stderr, "[TCP RX] seq:%u type:0x%02X crc:0x%04X\n",
            pkt.seq, pkt.type, pkt.crc16);

    // ─────────────────────────────────────────
    // CRC 검증 (선택적)
    // ─────────────────────────────────────────
    uint8_t full_pkt[16];
    size_t pkt_len = 0;
    full_pkt[pkt_len++] = 0xBB;
    full_pkt[pkt_len++] = pkt.seq & 0xFF;
    full_pkt[pkt_len++] = (pkt.seq >> 8) & 0xFF;
    full_pkt[pkt_len++] = pkt.type;
    
    if (pkt.type == CMD_TARGET) {
        memcpy(full_pkt + pkt_len, &pkt.target_x, 4); pkt_len += 4;
        memcpy(full_pkt + pkt_len, &pkt.target_y, 4); pkt_len += 4;
    } else {
        full_pkt[pkt_len++] = pkt.cmd_data;
    }
    
    uint16_t calc_crc = Protocol::calculateCRC16(full_pkt, pkt_len);
    fprintf(stderr, "[TCP RX] recv_crc:0x%04X calc_crc:0x%04X %s\n",
            pkt.crc16, calc_crc,
            (pkt.crc16 == calc_crc) ? "OK" : "FAIL");
    fflush(stderr);

    return true;
}