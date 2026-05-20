#include "UartNode.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <chrono>

// ═══════════════════════════════════════════════
// 생성자: UART 시리얼 포트 초기화
// ═══════════════════════════════════════════════
UartNode::UartNode(const char* port, int baudrate) {
    fprintf(stderr, "  [UART] Opening %s...\n", port);
    fflush(stderr);
    
    // 시리얼 포트 열기
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "  [UART Error] Cannot open %s: %s\n", 
                port, strerror(errno));
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "  [UART] Port opened (fd=%d), configuring...\n", fd);
    fflush(stderr);

    // termios 구조체로 시리얼 설정
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0); // Blocking 모드
    
    fprintf(stderr, "  [UART] Configuration complete\n");
    fflush(stderr);
}

// ═══════════════════════════════════════════════
// 소멸자: UART 포트 닫기
// ═══════════════════════════════════════════════
UartNode::~UartNode() {
    if (fd != -1) close(fd);
}

// ═══════════════════════════════════════════════
// 통제소 → GUI 패킷 전송 (가변 길이)
// 직렬화 로직을 Protocol 클래스로 통합
// ═══════════════════════════════════════════════
void UartNode::sendGuiPacket(float torpedo_x, float torpedo_y,
                              float yaw,
                              const std::vector<std::vector<PointGrid>>& clusters,
                              uint16_t seq) {
    std::vector<uint8_t> buf;

    // 1. 헤더 직렬화
    buf.push_back(0xAA);
    Protocol::packU16(buf, seq);
    Protocol::packU16(buf, (uint16_t)clusters.size());
    Protocol::packFloat(buf, torpedo_x);
    Protocol::packFloat(buf, torpedo_y);
    Protocol::packFloat(buf, yaw);

    // 2. 페이로드 직렬화
    for (const auto& cluster : clusters) {
        Protocol::packU16(buf, (uint16_t)cluster.size());
        for (const auto& pg : cluster) {
            float cx = (pg.x == 0.0f && pg.y == 0.0f) ? std::nanf("") : pg.x;
            float cy = (pg.x == 0.0f && pg.y == 0.0f) ? std::nanf("") : pg.y;
            Protocol::packFloat(buf, cx);
            Protocol::packFloat(buf, cy);
        }
    }

    // 3. CRC16 계산 및 추가
    uint16_t crc = Protocol::calculateCRC16(buf.data(), buf.size());
    Protocol::packU16(buf, crc);

    // 4. UART 전송
    write(fd, buf.data(), buf.size());
}

// ═══════════════════════════════════════════════
// GUI → 통제소 명령 수신
// ═══════════════════════════════════════════════
bool UartNode::receiveGuiCommand(GuiCommandPacket& pkt) {
    uint8_t head;
    if (read(fd, &head, 1) > 0 && head == 0xBB) {
        uint8_t buf[3];
        if (read(fd, buf, 3) == 3) {
            pkt.sync = 0xBB;
            pkt.seq  = buf[0] | (buf[1] << 8);
            pkt.type = buf[2];

            if (pkt.type == PKT_TYPE_TARGET) {
                uint8_t payload[10];
                if (read(fd, payload, 10) == 10) {
                    memcpy(&pkt.target_x, payload, 4);
                    memcpy(&pkt.target_y, payload + 4, 4);
                    pkt.crc16 = payload[8] | (payload[9] << 8);
                    return true;
                }
            } else {
                uint8_t payload[3];
                if (read(fd, payload, 3) == 3) {
                    pkt.cmd_data = payload[0];
                    pkt.crc16 = payload[1] | (payload[2] << 8);
                    return true;
                }
            }
        }
    }
    return false;
}

// ═══════════════════════════════════════════════
// 통제소 → 어뢰 Downlink 전송
// ═══════════════════════════════════════════════
void UartNode::sendDownlink(float target_x, float target_y,
                            float torpedo_x, float torpedo_y,
                            int16_t steer, uint8_t flags,
                            uint16_t seq) {
    if (fd < 0) return;

    DownlinkPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync      = 0xAA;
    pkt.sync2     = 0x55;
    pkt.msg_id    = 0x00;
    pkt.length    = sizeof(DownlinkPacket) - 4;
    pkt.seq       = seq;
    pkt.target_x  = target_x;
    pkt.target_y  = target_y;
    pkt.torpedo_x = torpedo_x;
    pkt.torpedo_y = torpedo_y;
    pkt.steer     = steer;
    pkt.flags     = flags;
    pkt.crc16     = Protocol::calculateCRC16((uint8_t*)&pkt, sizeof(pkt) - 2);

    write(fd, &pkt, sizeof(pkt));

    // ⭐ 로그 추가 (10ms마다 찍히니까 100ms에 1번만 출력)
    int ret = write(fd, &pkt, sizeof(pkt));

    if (seq % 10 == 0) {
        fprintf(stderr, "[Downlink TX] seq:%u ret:%d "
                "target:(%.2f,%.2f) "
                "torpedo:(%.2f,%.2f) "
                "steer:%d flags:0x%02X\n",
                seq, ret,
                target_x, target_y,
                std::isnan(torpedo_x) ? -1.0f : torpedo_x,
                std::isnan(torpedo_y) ? -1.0f : torpedo_y,
                steer, flags);
        fflush(stderr);
    }
}

// ═══════════════════════════════════════════════
// 어뢰 → 통제소 Uplink 수신
// ═══════════════════════════════════════════════
bool UartNode::receiveUplinkStatus(UplinkPacket& pkt) {
    uint8_t head;
    if (read(fd, &head, 1) > 0 && head == 0xBB) {
        // 구조체의 나머지 부분 읽기 (sync 제외)
        uint8_t* payload = ((uint8_t*)&pkt) + 1;
        int total_read = 0;
        int remain = sizeof(UplinkPacket) - 1;
        
        while (total_read < remain) {
            int n = read(fd, payload + total_read, remain - total_read);
            if (n > 0) total_read += n;
        }
        pkt.sync = 0xBB;
        
        // CRC 검증
        uint16_t calc_crc = Protocol::calculateCRC16((uint8_t*)&pkt, sizeof(UplinkPacket) - 2);
        return (calc_crc == pkt.crc16);
    }
    return false;
}