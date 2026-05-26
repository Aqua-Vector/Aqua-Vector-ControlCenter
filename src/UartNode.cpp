#include "UartNode.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <chrono>


static speed_t toSpeedT(int baud) {

    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default:
            fprintf(stderr, "  [UART Warn] Unsupported baudrate %d, falling back to 9600\n", baud);
            return B9600;
    }
}
// ═══════════════════════════════════════════════
// 생성자: UART 시리얼 포트 초기화
// ═══════════════════════════════════════════════
UartNode::UartNode(const char* port, int baudrate) {

    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "  [UART Error] Cannot open %s: %s\n",
                port, strerror(errno));
        fflush(stderr);
        this->fd = -1;
        return;
    }

    struct termios options;
    std::memset(&options, 0, sizeof(options));
    if (tcgetattr(fd, &options) != 0) return ;

    speed_t speed = toSpeedT(baudrate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_iflag |= IGNPAR;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) return ;
    return ;
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

            if (pkt.type == CMD_TARGET) {
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
    pkt.length    = 21;
    pkt.seq       = seq;
    pkt.target_x  = target_x;
    pkt.target_y  = target_y;
    pkt.torpedo_x = torpedo_x;
    pkt.torpedo_y = torpedo_y;
    pkt.steer     = steer;
    pkt.flags     = flags;
    pkt.crc16     = Protocol::calculateCRC16((uint8_t*)&pkt, sizeof(pkt) - 2);

    write(fd, &pkt, sizeof(pkt));
}

// ═══════════════════════════════════════════════
// 어뢰 → 통제소 Uplink 수신
// ═══════════════════════════════════════════════
bool UartNode::receiveUplinkStatus(UplinkPacket& pkt) {
    if (fd < 0) return false;

    // sync 탐색
    uint8_t h1, h2;
    if (read(fd, &h1, 1) <= 0 || h1 != 0xAA) return false;
    if (read(fd, &h2, 1) <= 0 || h2 != 0x55) return false;

    pkt.sync = 0xAA;
        pkt.sync2 = 0x55;

    // header 이후 나머지 바이트를 버퍼로 수신
    constexpr int REMAIN = sizeof(UplinkPacket) - 2; // 20 bytes
    uint8_t buf[REMAIN];
    int total = 0;
    // int retry = 0;
        while (total < REMAIN) {
            int n = read(fd, buf + total, REMAIN - total);
            if (n > 0) {
                total += n;
                // retry = 0;
            } else if (n < 0 && errno != EAGAIN) {
                return false;
            } 
            // else {
            //     usleep(100);
            //     if (++retry > 1000) return false; // 100ms 타임아웃
            // }
        }
        
    fprintf(stderr, "[Uplink RX] Raw bytes: %d", total);
    for (int i = 0; i < REMAIN; i++) {
        fprintf(stderr, " %02X", buf[i]);
    }
    fprintf(stderr, "\n");

    // 버퍼에서 필드별 역직렬화
    int off = 0;
    pkt.msg_id = buf[off++];
    pkt.length = buf[off++];
    memcpy(&pkt.seq,          buf + off, sizeof(pkt.seq));   off += sizeof(pkt.seq);
    memcpy(&pkt.p_x,          buf + off, sizeof(pkt.p_x));   off += sizeof(pkt.p_x);
    memcpy(&pkt.p_y,          buf + off, sizeof(pkt.p_y));   off += sizeof(pkt.p_y);
    memcpy(&pkt.yaw,          buf + off, sizeof(pkt.yaw));   off += sizeof(pkt.yaw);
    pkt.status_flags = buf[off++];
    pkt.reserved     = buf[off++];
    memcpy(&pkt.crc16,        buf + off, sizeof(pkt.crc16));
    pkt.yaw = pkt.yaw * 180.0f / M_PI;
    fprintf(stderr, "[Uplink RX] seq:%u p_x:%.2f p_y:%.2f yaw:%.2f flags:0x%02X\n",
    pkt.seq, pkt.p_x, pkt.p_y, pkt.yaw, pkt.status_flags);
 

    // if (pkt.seq % 500 == 0) {
    //     fprintf(stderr, "[Uplink RX] seq:%u p_x:%.2f p_y:%.2f yaw:%.2f flags:0x%02X\n",
    //             pkt.seq, pkt.p_x, pkt.p_y, pkt.yaw, pkt.status_flags);
    // }


    // CRC 검증 (crc16 필드 제외한 전체 구조체 대상)
    uint16_t calc_crc = Protocol::calculateCRC16((uint8_t*)&pkt, sizeof(UplinkPacket) - sizeof(pkt.crc16));
    return (calc_crc == pkt.crc16);
}