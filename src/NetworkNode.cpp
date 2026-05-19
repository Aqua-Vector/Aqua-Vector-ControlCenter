#include "NetworkNode.h"
#include <iostream>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>

NetworkNode::NetworkNode(const char* pc_ip,
                         int udp_tx_port,
                         int udp_rx_port,
                         int tcp_port)
    : udp_tx_fd(-1), udp_rx_fd(-1),
      tcp_server_fd(-1), tcp_client_fd(-1)
{
    // UDP TX
    udp_tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_tx_fd < 0) {
        fprintf(stderr, "  [Network Error] UDP TX socket failed: %s\n", strerror(errno));
        return;
    }
    memset(&udp_tx_addr, 0, sizeof(udp_tx_addr));
    udp_tx_addr.sin_family      = AF_INET;
    udp_tx_addr.sin_port        = htons(udp_tx_port);
    udp_tx_addr.sin_addr.s_addr = inet_addr(pc_ip);
    fprintf(stderr, "  [Network] UDP TX -> %s:%d OK\n", pc_ip, udp_tx_port);
    fflush(stderr);

    // UDP RX
    udp_rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_rx_fd < 0) {
        fprintf(stderr, "  [Network Error] UDP RX socket failed: %s\n", strerror(errno));
        return;
    }
    struct sockaddr_in rx_addr;
    memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family      = AF_INET;
    rx_addr.sin_port        = htons(udp_rx_port);
    rx_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(udp_rx_fd, (struct sockaddr*)&rx_addr, sizeof(rx_addr)) < 0) {
        fprintf(stderr, "  [Network Error] UDP RX bind failed: %s\n", strerror(errno));
        return;
    }
    fcntl(udp_rx_fd, F_SETFL, O_NONBLOCK);
    fprintf(stderr, "  [Network] UDP RX port %d OK\n", udp_rx_port);
    fflush(stderr);

    // TCP
    tcp_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_fd < 0) {
        fprintf(stderr, "  [Network Error] TCP socket failed: %s\n", strerror(errno));
        return;
    }
    int opt = 1;
    setsockopt(tcp_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in tcp_addr;
    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family      = AF_INET;
    tcp_addr.sin_port        = htons(tcp_port);
    tcp_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(tcp_server_fd, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
        fprintf(stderr, "  [Network Error] TCP bind failed: %s\n", strerror(errno));
        return;
    }
    if (listen(tcp_server_fd, 1) < 0) {
        fprintf(stderr, "  [Network Error] TCP listen failed: %s\n", strerror(errno));
        return;
    }
    fprintf(stderr, "  [Network] TCP server port %d OK\n", tcp_port);
    fflush(stderr);
}

NetworkNode::~NetworkNode() {
    if (udp_tx_fd  >= 0) close(udp_tx_fd);
    if (udp_rx_fd  >= 0) close(udp_rx_fd);
    if (tcp_client_fd >= 0) close(tcp_client_fd);
    if (tcp_server_fd >= 0) close(tcp_server_fd);
}

// UDP: Zynq → 노트북 전송 (UART sendGuiPacket과 동일한 직렬화)
void NetworkNode::sendGuiPacketUDP(float torpedo_x, float torpedo_y,
                                    float yaw,
                                    const std::vector<std::vector<PointGrid>>& clusters,
                                    uint16_t seq) {
    std::vector<uint8_t> buf;

    auto push_float = [&](float val) {
        uint8_t* p = (uint8_t*)&val;
        for (int i = 0; i < 4; i++) buf.push_back(p[i]);
    };
    auto push_u16 = [&](uint16_t val) {
        buf.push_back(val & 0xFF);
        buf.push_back((val >> 8) & 0xFF);
    };

    buf.push_back(0xAA);
    push_u16(seq);
    push_u16((uint16_t)clusters.size());
    push_float(torpedo_x);
    push_float(torpedo_y);
    push_float(yaw);

    for (const auto& cluster : clusters) {
        push_u16((uint16_t)cluster.size());
        for (const auto& pg : cluster) {
            push_float(pg.x * 0.05f);
            push_float(pg.y * 0.05f);
        }
    }

    uint16_t crc = Protocol::calculateCRC16(buf.data(), buf.size());
    push_u16(crc);

    // // ⭐ 전송 패킷 로그
    // fprintf(stderr, "[UDP TX] seq:%u total_bytes:%zu crc:0x%04X\n",
    //         seq, buf.size(), crc);
    // fprintf(stderr, "[UDP TX] hex: ");
    // for (size_t i = 0; i < buf.size() && i < 32; i++) {
    //     fprintf(stderr, "%02X ", buf[i]);
    // }
    // fprintf(stderr, "...\n");
    // fflush(stderr);

    sendto(udp_tx_fd, buf.data(), buf.size(), 0,
           (struct sockaddr*)&udp_tx_addr, sizeof(udp_tx_addr));
}

// UDP: 노트북 → Zynq 목표물 좌표 수신
bool NetworkNode::receiveTargetUDP(GuiCommandPacket& pkt) {
    uint8_t buf[256];
    ssize_t n = recv(udp_rx_fd, buf, sizeof(buf), 0);
    if (n <= 0) return false;

    if (buf[0] != 0xBB) return false;
    memcpy(&pkt, buf, sizeof(GuiCommandPacket));
    return true;
}

// TCP: 클라이언트 접속 대기 (블로킹 - 별도 스레드에서 호출)
void NetworkNode::acceptTcpClient() {
    while (true) {
        std::cout << "[TCP] Waiting for client..." << std::endl;
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);
        int new_fd = accept(tcp_server_fd,
                            (struct sockaddr*)&client_addr, &len);
        if (new_fd >= 0) {
            if (tcp_client_fd >= 0) close(tcp_client_fd);
            tcp_client_fd = new_fd;
            std::cout << "[TCP] Client connected: "
                      << inet_ntoa(client_addr.sin_addr) << std::endl;
        }
    }
}

// TCP: 명령 수신
bool NetworkNode::receiveCommandTCP(GuiCommandPacket& pkt) {
    if (tcp_client_fd < 0) return false;

    uint8_t head;
    int n = recv(tcp_client_fd, &head, 1, MSG_DONTWAIT);
    if (n <= 0 || head != 0xBB) return false;

    uint8_t buf[3];
    if (recv(tcp_client_fd, buf, 3, 0) != 3) return false;

    pkt.sync = 0xBB;
    pkt.seq  = buf[0] | (buf[1] << 8);
    pkt.type = buf[2];

    if (pkt.type == PKT_TYPE_TARGET) {
        uint8_t payload[10];
        if (recv(tcp_client_fd, payload, 10, 0) != 10) return false;
        memcpy(&pkt.target_x, payload, 4);
        memcpy(&pkt.target_y, payload + 4, 4);
        pkt.crc16 = payload[8] | (payload[9] << 8);
    } else {
        uint8_t payload[3];
        if (recv(tcp_client_fd, payload, 3, 0) != 3) return false;
        pkt.cmd_data = payload[0];
        pkt.crc16 = payload[1] | (payload[2] << 8);
    }

    // ⭐ 수신 패킷 로그
    fprintf(stderr, "[TCP RX] seq:%u type:0x%02X crc:0x%04X\n",
            pkt.seq, pkt.type, pkt.crc16);

    // ⭐ CRC 검증 로그
    // 전체 패킷을 버퍼에 모아서 CRC 계산
    uint8_t full_pkt[16];
    size_t pkt_len = 0;
    full_pkt[pkt_len++] = 0xBB;
    full_pkt[pkt_len++] = pkt.seq & 0xFF;
    full_pkt[pkt_len++] = (pkt.seq >> 8) & 0xFF;
    full_pkt[pkt_len++] = pkt.type;
    if (pkt.type == PKT_TYPE_TARGET) {
        memcpy(full_pkt + pkt_len, &pkt.target_x, 4); pkt_len += 4;
        memcpy(full_pkt + pkt_len, &pkt.target_y, 4); pkt_len += 4;
    } else {
        full_pkt[pkt_len++] = pkt.cmd_data;
    }
    uint16_t calc_crc = Protocol::calculateCRC16(full_pkt, pkt_len);
    fprintf(stderr, "[TCP RX] recv_crc:0x%04X calc_crc:0x%04X %s\n",
            pkt.crc16, calc_crc,
            (pkt.crc16 == calc_crc) ? "OK" : "FAIL");
    fprintf(stderr, "[TCP RX] hex: ");
    for (size_t i = 0; i < pkt_len; i++) {
        fprintf(stderr, "%02X ", full_pkt[i]);
    }
    fprintf(stderr, "\n");
    fflush(stderr);



    return true;
}