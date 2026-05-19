#include "UartNode.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cmath>

// UartNode::UartNode(const char* port, int baudrate) {
//     fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (fd == -1) {
//         std::cerr << "[UART Error] Cannot open " << port << std::endl;
//         return;
//     }

//     struct termios options;
//     tcgetattr(fd, &options);
//     cfsetispeed(&options, B115200);
//     cfsetospeed(&options, B115200);
//     options.c_cflag |= (CLOCAL | CREAD);
//     options.c_cflag &= ~PARENB;
//     options.c_cflag &= ~CSTOPB;
//     options.c_cflag &= ~CSIZE;
//     options.c_cflag |= CS8;
//     tcsetattr(fd, TCSANOW, &options);
//     fcntl(fd, F_SETFL, 0);
// }

UartNode::UartNode(const char* port, int baudrate) {
    fprintf(stderr, "  [UART] Opening %s...\n", port);
    fflush(stderr);
    
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "  [UART Error] Cannot open %s: %s\n", 
                port, strerror(errno));
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "  [UART] Port opened (fd=%d), configuring...\n", fd);
    fflush(stderr);

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0);
    
    fprintf(stderr, "  [UART] Configuration complete\n");
    fflush(stderr);
}


UartNode::~UartNode() {
    if (fd != -1) close(fd);
}

// GUI로 전송 (가변길이 직렬화)
void UartNode::sendGuiPacket(float torpedo_x, float torpedo_y,
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

    // [0] sync
    buf.push_back(0xAA);

    // [1-2] seq
    push_u16(seq);

    // [3-4] obj_count
    uint16_t obj_count = (uint16_t)clusters.size();
    push_u16(obj_count);

    // [5-8] torpedo_x (없으면 NaN)
    push_float(torpedo_x);

    // [9-12] torpedo_y (없으면 NaN)
    push_float(torpedo_y);

    // [13-16] yaw (없으면 NaN)
    push_float(yaw);

    // 각 클러스터: pt_count(2) + x,y(각4) * pt_count
    for (const auto& cluster : clusters) {
        uint16_t pt_count = (uint16_t)cluster.size();
        push_u16(pt_count);

        for (const auto& pg : cluster) {
            // 없으면 NaN, 있으면 50mm 단위 -> m
            float cx = (pg.x == 0 && pg.y == 0) ? std::nanf("") : pg.x * 0.05f;
            float cy = (pg.x == 0 && pg.y == 0) ? std::nanf("") : pg.y * 0.05f;
            push_float(cx);
            push_float(cy);
        }
    }

    // CRC (sync ~ payload 전체)
    uint16_t crc = Protocol::calculateCRC16(buf.data(), buf.size());
    push_u16(crc);

    // ⭐ fd가 -1이면 포트 열기 실패
    // fprintf(stderr, "[UART TX] fd=%d bytes=%zu\n", fd, buf.size());
    fflush(stderr);

    int ret = write(fd, buf.data(), buf.size());

    // ⭐ ret=-1이면 전송 실패
    // fprintf(stderr, "[UART TX] write ret=%d errno=%d\n", ret, errno);
    fflush(stderr);

}

// GUI 명령 수신
bool UartNode::receiveGuiCommand(GuiCommandPacket& pkt) {
    uint8_t head;
    if (read(fd, &head, 1) > 0 && head == 0xBB) {
        // seq(2) + type(1) = 3바이트 읽기
        uint8_t buf[3];
        if (read(fd, buf, 3) == 3) {
            pkt.sync = 0xBB;
            pkt.seq  = buf[0] | (buf[1] << 8);
            pkt.type = buf[2];

            // type별 payload 크기 다름
            if (pkt.type == PKT_TYPE_TARGET) {
                // target_x(4) + target_y(4) + crc(2) = 10바이트
                uint8_t payload[10];
                if (read(fd, payload, 10) == 10) {
                    memcpy(&pkt.target_x, payload, 4);
                    memcpy(&pkt.target_y, payload + 4, 4);
                    pkt.crc16 = payload[8] | (payload[9] << 8);
                    return true;
                }
            } else {
                // cmd_data(1) + crc(2) = 3바이트
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

void UartNode::sendDownlink(float target_x, float target_y,
                             float torpedo_x, float torpedo_y,
                             uint32_t timestamp_us, uint16_t seq) {
    DownlinkPacket pkt;
    pkt.sync         = 0xAA;
    pkt.timestamp_us = timestamp_us;
    pkt.seq          = seq;
    pkt.target_x     = target_x;
    pkt.target_y     = target_y;
    pkt.torpedo_x    = torpedo_x;
    pkt.torpedo_y    = torpedo_y;
    pkt.crc16        = Protocol::calculateCRC16((uint8_t*)&pkt, sizeof(pkt) - 2);
    write(fd, &pkt, sizeof(pkt));
}

bool UartNode::receiveUplinkStatus(UplinkPacket& pkt) {
    uint8_t head;
    // sync 바이트 0xBB 탐색
    if (read(fd, &head, 1) > 0 && head == 0xBB) {
        uint8_t* payload = ((uint8_t*)&pkt) + 1;
        int total_read = 0;
        int remain = sizeof(UplinkPacket) - 1;
        while (total_read < remain) {
            int n = read(fd, payload + total_read, remain - total_read);
            if (n > 0) total_read += n;
        }
        pkt.sync = 0xBB;
        // CRC 검증 (crc16 필드 앞까지)
        uint16_t calc_crc = Protocol::calculateCRC16(
            (uint8_t*)&pkt, sizeof(UplinkPacket) - 2);
        return (calc_crc == pkt.crc16);
    }
    return false;
}