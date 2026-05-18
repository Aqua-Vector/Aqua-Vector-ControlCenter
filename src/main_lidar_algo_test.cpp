#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <cmath>    // std::isnan, std::nanf 사용
#include "LidarNode.h"
#include "DataTypes.h"

#pragma pack(push, 1)
struct DownlinkPacket {
    uint8_t  sync;          // [0] 0xAA
    uint32_t timestamp_us;  // [1-4] Control station time (us)
    uint16_t seq;           // [5-6] Sequence
    float    target_x;      // [7-10] Target X (m)
    float    target_y;      // [11-14] Target Y (m)
    float    torpedo_x;     // [15-18] Lidar X (m) or NaN
    float    torpedo_y;     // [19-22] Lidar Y (m) or NaN
    uint16_t crc16;         // [23-24] CRC16-CCITT
};
#pragma pack(pop)

// CRC16-CCITT 계산 함수
uint16_t calculateCRC16(const uint8_t* data, size_t len) {
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

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   [Lidar to Torpedo TX (ttyS2) 10ms]   " << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. 초기화
    LidarNode* lidar = new LidarNode("/dev/ttyUSB0", 460800);
    int uart_fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY);
    
    if (uart_fd == -1) {
        std::cerr << "[Error] Cannot open /dev/ttyS2" << std::endl;
        delete lidar;
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tcsetattr(uart_fd, TCSANOW, &options);

    // 2. 캘리브레이션
    std::cout << "\n[STEP 1] Calibration. Press ENTER to start...";
    std::cin.get();
    lidar->startCalibration(4);
    std::cout << "[DONE] Calibration finished." << std::endl;

    // 3. 추적 및 전송
    std::cout << "\n[STEP 2] Press ENTER to start real-time TX (10ms)...";
    std::cin.get();

    uint16_t sequence = 0;
    Point2D torpedo_pos;
    bool is_found = false; // 타겟 상태 유지 변수 (데이터 홀드용)
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 정밀한 10ms 타이밍 제어를 위해 루프 시작 시간 기록
        auto loop_start = std::chrono::steady_clock::now(); 

        DownlinkPacket pkt;
        Point2D latest_pos;

        // 라이다 드라이버가 새로운 한 바퀴 스캔을 성공적으로 수신/연산했는지 확인
        // 성공 시 데이터 업데이트 및 발견 상태 유지, 실패 시 기존의 데이터와 발견 상태를 재활용(Hold)
        if (lidar->getDynamicCarPosition(latest_pos)) {
            torpedo_pos = latest_pos;
            is_found = true;
        }

        // [0] Sync
        pkt.sync = 0xAA;

        // [1-4] Timestamp (μs)
        auto now = std::chrono::steady_clock::now();
        pkt.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();

        // 모니터링 출력용 시간 계산 (시:분:초.밀리초)
        auto sys_now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(sys_now);
        std::tm* local_tm = std::localtime(&now_c);
        
        auto sys_duration = sys_now.time_since_epoch();
        auto sys_secs = std::chrono::duration_cast<std::chrono::seconds>(sys_duration);
        long ms = std::chrono::duration_cast<std::chrono::milliseconds>(sys_duration - sys_secs).count();

        // [5-6] Sequence
        pkt.seq = sequence++;

        // [7-14] Target Coordinates (가야할 곳, 0.0 고정)
        pkt.target_x = 0.0f;
        pkt.target_y = 0.0f;

        // [15-22] Lidar Measurement (mm 단위를 m 단위로 변환)
        if (is_found) {
            pkt.torpedo_x = torpedo_pos.x / 1000.0f;
            pkt.torpedo_y = torpedo_pos.y / 1000.0f;
            
            printf("[TX 10ms] time: %02d:%02d:%02d.%03ld | SEQ: %u | X: %.3f m, Y: %.3f m\n", 
                   local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec, ms, 
                   pkt.seq, pkt.torpedo_x, pkt.torpedo_y);
        } else {
            // 최초 기동 시 등 한 번도 물체를 못 찾았을 때는 NaN 처리
            pkt.torpedo_x = std::nanf("");
            pkt.torpedo_y = std::nanf("");
            
            printf("[TX 10ms] time: %02d:%02d:%02d.%03ld | SEQ: %u | Target Lost (NaN Sent)\n", 
                   local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec, ms, pkt.seq);
        }

        // [23-24] CRC16-CCITT (마지막 2바이트 제외한 23바이트 계산)
        pkt.crc16 = calculateCRC16((uint8_t*)&pkt, 23);

        // UART 전송 (요구사항대로 10ms마다 무조건 패킷 발송)
        write(uart_fd, &pkt, sizeof(pkt));

        // 10ms 루프 주기를 정밀하게 맞추기 위한 연산 속도 차감 대기 로직
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
        
        long sleep_time = 10000 - elapsed; // 10,000 microsecond = 10 millisecond
        if (sleep_time > 0) {
            usleep(sleep_time); // 남은 시간만큼만 대기
        }
    }

    close(uart_fd);
    delete lidar;
    return 0;
}