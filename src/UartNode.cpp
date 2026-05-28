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

    // Zynq 명세 기준 고정 사이즈 정의
    constexpr size_t PAYLOAD_SIZE = 16;  // sizeof(Payload) -> seq(4)+p_x(4)+p_y(4)+yaw(4)
    constexpr size_t HEADER_SIZE = 4;   // SYNC1(1) + SYNC2(1) + MSG_ID(1) + LENGTH(1)
    constexpr size_t CRC_SIZE = 2;      // Policy::CRC_SIZE (uint16_t)
    constexpr size_t TOTAL_PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE; // 22 Bytes

    // 1. 헤더 싱크 동기화 탐색
    uint8_t h1, h2;
    if (read(fd, &h1, 1) <= 0 || h1 != 0xAA) return false; // Policy::SYNC1
    if (read(fd, &h2, 1) <= 0 || h2 != 0x55) return false; // Policy::SYNC2

    pkt.sync = 0xAA;
    pkt.sync2 = 0x55;

    // 2. 나머지 패킷 전체(20바이트) 한 번에 수집용 버퍼
    // [msg_id(1) + length(1) + payload(16) + crc(2)]
    constexpr size_t REMAIN_SIZE = TOTAL_PACKET_SIZE - 2;
    uint8_t rx_buf[REMAIN_SIZE];
    
    size_t total = 0;
    while (total < REMAIN_SIZE) {
        int n = read(fd, rx_buf + total, REMAIN_SIZE - total);
        if (n > 0) {
            total += n;
        } else if (n < 0 && errno != EAGAIN) {
            return false;
        }
    }

    // 변수 분리 분기점 데이터 매핑
    uint8_t msg_id = rx_buf[0];
    uint8_t length = rx_buf[1];

    // 3. 🎯 [Zynq 1:1 일치] CRC 검증부 
    // Zynq line 68: buffer + 2 부터 2 + payload_size 만큼만 CRC 계산
    // 즉, rx_buf의 처음(msg_id)부터 페이로드 끝까지가 CRC 계산 대상 범위입니다.
    uint16_t calc_crc = Protocol::calculateCRC16(rx_buf, 2 + PAYLOAD_SIZE);

    // Zynq line 71 방식 역추적: 리틀 엔디안 바이트 스트림 조합으로 수신 CRC 복원
    uint16_t received_crc = static_cast<uint16_t>(rx_buf[REMAIN_SIZE - 2]) | 
                            (static_cast<uint16_t>(rx_buf[REMAIN_SIZE - 1]) << 8);

    // 완벽한 무결성 비교 검증
    if (calc_crc != received_crc) {
        static int debug_cnt = 0;
        if (debug_cnt++ % 200 == 0) {
            fprintf(stderr, "[Uplink CRC Error] 범위 정정 후 검증 미스매치! Calc: 0x%04X, Recv: 0x%04X\n", calc_crc, received_crc);
        }
        return false;
    }

    // 4. 안전한 역직렬화 (Marshaller::deserialize 우회 구조체 안전 복사)
    pkt.msg_id = msg_id;
    pkt.length = length;

    // rx_buf[2] 가 순수 페이로드(packet.payload)의 시작 시점입니다.
    int off = 2; 
    memcpy(&pkt.seq,          rx_buf + off, sizeof(pkt.seq));        off += sizeof(pkt.seq);
    memcpy(&pkt.p_x,          rx_buf + off, sizeof(pkt.p_x));        off += sizeof(pkt.p_x);
    memcpy(&pkt.p_y,          rx_buf + off, sizeof(pkt.p_y));        off += sizeof(pkt.p_y);
    memcpy(&pkt.yaw,          rx_buf + off, sizeof(pkt.yaw));        off += sizeof(pkt.yaw);
    pkt.status_flags = rx_buf[off++];
    pkt.reserved     = rx_buf[off++];
    pkt.crc16        = received_crc;

    // 5. 완벽 동기화 공유 변수 데이터 반영 성공 로그
    static int win_cnt = 0;
    if (win_cnt++ % 50 == 0) {
        fprintf(stderr, "[Uplink RX] 🚀 [Zynq 프로토콜 동기화 완료] seq:%u x:%.2f y:%.2f yaw:%.2f\n", 
                pkt.seq, pkt.p_x, pkt.p_y, pkt.yaw * (180.0f / M_PI));
    }

    return true;
}