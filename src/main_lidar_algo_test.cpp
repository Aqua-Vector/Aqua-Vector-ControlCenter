#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <thread> // 스레드 사용을 위해 추가
#include <mutex>  // 데이터 충돌 방지를 위해 추가
#include "LidarNode.h"
#include "DataTypes.h"
#include "CrcCalculator.h"

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

// ----------------------------------------------------
// [스레드 간 공유 데이터]
std::mutex data_mutex;         // 동시에 데이터 읽기/쓰기 방지용 자물쇠
Point2D shared_torpedo_pos;    // 공유 타겟 위치
bool shared_is_found = false;  // 공유 타겟 발견 여부

// [라이다 전담 스레드 함수]
// 이 함수는 백그라운드에서 50ms마다 계속 돌면서 최신 좌표만 갱신합니다.
void lidarThreadFunction(LidarNode* lidar) {
    Point2D latest_pos;
    while (true) {
        // grabScanDataHq 때문에 여기서 50ms 동안 대기(블로킹)됨
        if (lidar->getDynamicCarPosition(latest_pos)) {
            // 데이터가 나오면 자물쇠를 걸고 안전하게 공유 변수 업데이트
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_torpedo_pos = latest_pos;
            shared_is_found = true;
        }
    }
}
// ----------------------------------------------------

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

    // 3. 라이다 데이터 수집 스레드 실행
    // 백그라운드에서 라이다 데이터만 계속 업데이트하는 스레드 분리 시작
    std::thread lidar_thread(lidarThreadFunction, lidar);
    lidar_thread.detach(); // 메인 루프와 완전히 독립적으로 동작하도록 분리

    // 4. 추적 및 전송 (메인 스레드)
    std::cout << "\n[STEP 2] Press ENTER to start real-time TX (10ms)...";
    std::cin.get();

    uint16_t sequence = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 정밀한 10ms 타이밍 제어를 위해 루프 시작 시간 기록
        auto loop_start = std::chrono::steady_clock::now(); 

        DownlinkPacket pkt;
        
        // 라이다 스레드가 갱신해둔 데이터를 자물쇠 걸고 복사해옴 (아주 찰나의 시간만 소요됨)
        Point2D current_pos;
        bool current_is_found;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_pos = shared_torpedo_pos;
            current_is_found = shared_is_found;
        }

        // [0] Sync
        pkt.sync = 0xAA;

        // [1-4] Timestamp (μs)
        auto now = std::chrono::steady_clock::now();
        pkt.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();

        // 모니터링 출력용 시간 계산
        auto sys_now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(sys_now);
        std::tm* local_tm = std::localtime(&now_c);
        
        auto sys_duration = sys_now.time_since_epoch();
        auto sys_secs = std::chrono::duration_cast<std::chrono::seconds>(sys_duration);
        long ms = std::chrono::duration_cast<std::chrono::milliseconds>(sys_duration - sys_secs).count();

        // [5-6] Sequence
        pkt.seq = sequence++;

        // [7-14] Target Coordinates
        pkt.target_x = 0.0f;
        pkt.target_y = 0.0f;

        // [15-22] Lidar Measurement
        if (current_is_found) {
            pkt.torpedo_x = current_pos.x / 1000.0f;
            pkt.torpedo_y = current_pos.y / 1000.0f;
            
            printf("[TX 10ms] time: %02d:%02d:%02d.%03ld | SEQ: %u | X: %.3f m, Y: %.3f m\n", 
                   local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec, ms, 
                   pkt.seq, pkt.torpedo_x, pkt.torpedo_y);
        } else {
            pkt.torpedo_x = std::nanf("");
            pkt.torpedo_y = std::nanf("");
            
            printf("[TX 10ms] time: %02d:%02d:%02d.%03ld | SEQ: %u | Target Lost (NaN Sent)\n", 
                   local_tm->tm_hour, local_tm->tm_min, local_tm->tm_sec, ms, pkt.seq);
        }

        // [23-24] CRC16-CCITT 
        pkt.crc16 = CrcCalculator::CalculateCrc16((uint8_t*)&pkt, 23);

        // UART 전송 (이제 라이다 블로킹 영향을 받지 않고 무조건 10ms 주기로 실행됨)
        write(uart_fd, &pkt, sizeof(pkt));

        // 10ms 루프 주기 정밀 맞춤 로직
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
        
        long sleep_time = 10000 - elapsed; // 10,000 microsecond = 10 millisecond
        if (sleep_time > 0) {
            usleep(sleep_time);
        }
    }

    close(uart_fd);
    delete lidar;
    return 0;
}