#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include "LidarNode.h"
#include "ServoNode.h"
#include "UartNode.h"

// 공유 자원
std::vector<std::vector<PointGrid>> shared_obstacles; 
std::mutex mtx;

// 하드웨어 포트 (보드 환경에 맞게 수정 필요)
const char* LIDAR_PORT = "/dev/ttyUSB0";
const char* UART_PC_PORT = "/dev/ttyPS0"; // 또는 ttyUSB1 등 PC와 연결된 포트

// 객체 선언 (전역 포인터로 관리하여 스레드에서 접근)
LidarNode* lidar;
UartNode* uart;
ServoNode* servo1;
ServoNode* servo2;

// Thread 1: 라이다 인지
void thread_lidar() {
    while (true) {
        auto grid_clusters = lidar->getGridObstacles(); // 변수명 명확화
        
        { 
            std::lock_guard<std::mutex> lock(mtx);
            shared_obstacles = grid_clusters; // 타입 일치 (vector<vector<PointGrid>>)
        }
        usleep(10000); 
    }
}

// Thread 2: PC로 좌표 전송 (TX)
void thread_uart_tx() {
    while (true) {
        std::vector<std::vector<PointGrid>> temp_clusters; // 타입 수정
        { 
            std::lock_guard<std::mutex> lock(mtx);
            temp_clusters = shared_obstacles;
        }
        
        if (!temp_clusters.empty()) {
            uart->sendObstacles(temp_clusters); // 이제 타입이 맞습니다.
        }
        usleep(100000); 
    }
}

// Thread 3: PC 명령 수신 및 서보 제어 (RX & PWM)
void thread_uart_rx() {
    int servo_id, angle;
    while (true) {
        // read() 함수가 블로킹되어 있으므로 데이터가 올 때까지 대기합니다.
        if (uart->receiveCommand(servo_id, angle)) {
            std::cout << "[UART RX] ID: " << servo_id << " -> Value: " << angle << std::endl;
            
            // [새로 추가된 부분] 특수 명령어 (99,99) 수신 시 라이다 배경 초기화
            if (servo_id == 99 && angle == 99) {
                std::cout << "[System] Calibration Command Received! Mapping static obstacles..." << std::endl;
                lidar->startCalibration(4); // 4초 동안 배경 맵핑
            }
            // [기존 부분 유지] 1번 서보 제어
            else if (servo_id == 1) {
                servo1->setAngle(angle);
            } 
            // [기존 부분 유지] 2번 서보 제어
            else if (servo_id == 2) {
                servo2->setAngle(angle);
            }
        }
    }
}

int main() {
    std::cout << "[System] Multi-thread System Start..." << std::endl;

    // 초기화
    lidar = new LidarNode(LIDAR_PORT, 460800);
    uart = new UartNode(UART_PC_PORT, 115200);
    servo1 = new ServoNode(0, 0); // pwmchip0, pwm0
    servo2 = new ServoNode(0, 1); // pwmchip0, pwm1 (서보2번)

    servo1->setAngle(90);
    servo2->setAngle(90);
    sleep(1);

    // 스레드 가동
    std::thread t1(thread_lidar);
    std::thread t2(thread_uart_tx);
    std::thread t3(thread_uart_rx);

    // 메인 스레드 대기
    t1.join();
    t2.join();
    t3.join();

    return 0;
}