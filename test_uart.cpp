#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <errno.h>

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
    (void)signum;
    g_running = false;
}

// ── CRC16-CCITT ──────────────────────────────
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// ── 패킷 구조체 ──────────────────────────────
struct __attribute__((packed)) DownlinkPacket {
    uint8_t  sync;        // 0xAA
    uint8_t  sync2;       // 0x55
    uint8_t  msg_id;      // 0x00
    uint8_t  length;      // payload 크기
    uint16_t seq;
    float    target_x;
    float    target_y;
    float    torpedo_x;
    float    torpedo_y;
    int16_t  steer;
    uint8_t  flags;
    uint16_t crc16;
};

// ── UART 열기 ────────────────────────────────
static int uart_open(const char* dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfsetispeed(&tio, B460800);
    cfsetospeed(&tio, B460800);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// ── select로 데이터 대기 ─────────────────────
static bool wait_data(int fd, int timeout_ms) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    return select(fd + 1, &rfds, NULL, NULL, &tv) > 0;
}

// ── 정확히 n바이트 읽기 ──────────────────────
static bool read_exact(int fd, uint8_t* buf, int n, int timeout_ms) {
    int total = 0;
    while (total < n && g_running) {
        if (!wait_data(fd, timeout_ms)) return false;
        int r = read(fd, buf + total, n - total);
        if (r > 0) total += r;
        else if (r < 0 && errno != EAGAIN) return false;
    }
    return total == n;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    const char* port = (argc > 1) ? argv[1] : "/dev/ttyS2";

    std::cout << "==================================" << std::endl;
    std::cout << "  Downlink Packet Receiver" << std::endl;
    std::cout << "  Port: " << port << ", Baud: 460800" << std::endl;
    std::cout << "==================================" << std::endl;

    int fd = uart_open(port);
    if (fd < 0) {
        std::cerr << "[ERROR] cannot open " << port
                  << ": " << strerror(errno) << std::endl;
        return 1;
    }
    std::cout << "[OK] Port opened (fd=" << fd << ")" << std::endl;
    std::cout << "Waiting for packets (AA 55)..." << std::endl;

    uint64_t rx_ok  = 0;
    uint64_t rx_err = 0;

    while (g_running) {

        // ── 1. 0xAA 탐색 (select로 대기) ──
        uint8_t byte = 0;
        bool found_aa = false;
        while (g_running) {
            if (!wait_data(fd, 100)) continue;  // 100ms 대기
            if (read(fd, &byte, 1) > 0) {
                if (byte == 0xAA) {
                    found_aa = true;
                    break;
                }
            }
        }
        if (!found_aa) continue;

        // ── 2. 0x55 확인 (5ms 대기) ──
        uint8_t byte2 = 0;
        if (!wait_data(fd, 5)) continue;
        if (read(fd, &byte2, 1) <= 0) continue;

        if (byte2 == 0xAA) {
            // AA 연속 → 다시 탐색
            continue;
        }
        if (byte2 != 0x55) continue;

        // ── 3. 나머지 25바이트 읽기 ──
        DownlinkPacket pkt;
        pkt.sync  = 0xAA;
        pkt.sync2 = 0x55;

        uint8_t* p = (uint8_t*)&pkt + 2;
        int remain = sizeof(DownlinkPacket) - 2;

        if (!read_exact(fd, p, remain, 50)) {  // 50ms 타임아웃
            std::cout << "[WARN] incomplete packet" << std::endl;
            continue;
        }

        // ── 4. CRC 검증 ──
        uint16_t calc_crc = crc16_ccitt(
            (uint8_t*)&pkt, sizeof(DownlinkPacket) - 2);

        if (calc_crc == pkt.crc16) {
            rx_ok++;
            std::cout << "[RX OK #" << rx_ok << "]"
                      << " Seq:"   << std::dec << pkt.seq
                      << " Tgt:("  << std::fixed << std::setprecision(2)
                      << pkt.target_x << "," << pkt.target_y << ")"
                      << " Torp:(" << pkt.torpedo_x << "," << pkt.torpedo_y << ")"
                      << " Steer:" << pkt.steer
                      << " Flags:0x" << std::hex << (int)pkt.flags
                      << std::dec << std::endl;
        } else {
            rx_err++;
            std::cout << "[CRC ERR #" << rx_err << "]"
                      << " Seq:"  << std::dec << pkt.seq
                      << " Recv:0x" << std::hex << pkt.crc16
                      << " Calc:0x" << calc_crc
                      << std::dec << std::endl;
        }
    }

    std::cout << "\n[DONE] RX OK=" << rx_ok
              << " CRC ERR=" << rx_err << std::endl;

    close(fd);
    return 0;
}