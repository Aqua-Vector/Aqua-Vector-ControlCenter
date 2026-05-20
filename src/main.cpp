#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <arpa/inet.h> 

#include "LidarNode.h"
#include "ServoNode.h"
#include "UartNode.h"
#include "NetworkNode.h"
#include "AStarPlanner.h" 
#include "Protocol.h"
#include "LatencyAwareCtrl.h"

// ── 공유 자원 ─────────────────────────────────────────
std::mutex mtx;
TorpedoPose current_torpedo_state = {{0,0}, 0, 90.0, 0, 0};
Point2D gui_target = {5000, 5000}; // mm 단위
bool is_launched = false;
uint16_t downlink_seq = 0;
uint16_t gui_seq = 0;
std::chrono::steady_clock::time_point system_start_time;
std::vector<std::vector<PointGrid>> shared_grid_obstacles;

// 스레드 간 안전한 데이터 교환을 위한 원자적 변수
std::atomic<float> atomic_tar_x_meters{5.0f};  
std::atomic<float> atomic_tar_y_meters{0.0f};
std::atomic<float> atomic_torpedo_x_m{0.0f};
std::atomic<float> atomic_torpedo_y_m{0.0f};
std::atomic<bool>  atomic_lidar_valid{false};
std::atomic<bool>  atomic_astar_valid{false};
std::atomic<bool>  system_running{true};

// ── 전역 포인터 ─────────────────────────────────────────
LidarNode* lidar = nullptr;
UartNode* uart_torpedo = nullptr;
UartNode* uart_gui = nullptr;
ServoNode* hatch_servo = nullptr;
NetworkNode* network = nullptr;
AStarPlanner* astar_planner = nullptr;
LatencyAwareCtrl* latency_controller = nullptr;

// ─────────────────────────────────────────
// Thread 1: 라이다 데이터 수집 및 공유 (10ms)
// ─────────────────────────────────────────
void thread_lidar_tracking() {
    Point2D car_pos;
    while (system_running.load()) {
        auto loop_start = std::chrono::steady_clock::now();

        // 1. 클러스터 데이터 갱신 (GUI 전송용)
        auto grid_obs = lidar->getGridObstacles();
        {
            std::lock_guard<std::mutex> lock(mtx);
            shared_grid_obstacles = grid_obs;
        }

        // 2. 동적 객체(어뢰) 위치 추적 및 원자적 변수 업데이트 (제어 루프용)
        if (lidar->getDynamicCarPosition(car_pos)) {
            atomic_torpedo_x_m.store(car_pos.x / 1000.0f);
            atomic_torpedo_y_m.store(car_pos.y / 1000.0f);
            atomic_lidar_valid.store(true);
        } else {
            atomic_lidar_valid.store(false);
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        long sleep_us = 10000 - elapsed;
        if (sleep_us > 0) usleep(sleep_us);
    }
}

// ─────────────────────────────────────────
// Thread 2: 어뢰 → 통제소 Uplink 수신
// ─────────────────────────────────────────
void thread_torpedo_rx() {
    UplinkPacket pkt;
    while (system_running.load()) {
        if (uart_torpedo->receiveUplinkStatus(pkt)) {
            std::lock_guard<std::mutex> lock(mtx);
            current_torpedo_state.position.x = pkt.p_x * 1000.0f;
            current_torpedo_state.position.y = pkt.p_y * 1000.0f;
            current_torpedo_state.heading    = pkt.yaw * (180.0f / M_PI);
            
            std::cout << "[Uplink] SEQ:" << pkt.seq
                      << " X:" << pkt.p_x << "m"
                      << " Y:" << pkt.p_y << "m"
                      << " Yaw:" << pkt.yaw << "rad"
                      << " Flags:" << (int)pkt.status_flags << std::endl;
        }
        usleep(5000);
    }
}

// ─────────────────────────────────────────
// Thread 3: GUI → 통제소 명령 수신 (TCP/UART)
// ─────────────────────────────────────────
void thread_gui_rx() {
    GuiCommandPacket pkt;
    while (system_running.load()) {
        bool got_cmd = false;
        
        // UART 수신 확인 (선택 사항)
        // got_cmd = uart_gui->receiveGuiCommand(pkt);

        // TCP 수신 (연결된 경우 기본 처리)
        if (!got_cmd && network->isTcpConnected()) {
            got_cmd = network->receiveCommandTCP(pkt);
        }

        if (got_cmd) {
            switch (pkt.type) {
                case PKT_TYPE_OPEN:
                    hatch_servo->setAngle(pkt.cmd_data ? 180 : 90);
                    std::cout << "[CMD] Hatch " << (pkt.cmd_data ? "OPEN" : "CLOSE") << std::endl;
                    break;
                case PKT_TYPE_FIRE:
                    is_launched = true;
                    std::cout << "[CMD] Torpedo LAUNCHED / Control Loop Activated" << std::endl;
                    break;
                case PKT_TYPE_ENDGUIDE:
                    is_launched = false;
                    std::cout << "[CMD] Guidance TERMINATED" << std::endl;
                    break;
                case PKT_TYPE_TARGET:
                    std::lock_guard<std::mutex> lock(mtx);
                    gui_target.x = pkt.target_x * 1000.0f;
                    gui_target.y = pkt.target_y * 1000.0f;
                    atomic_tar_x_meters.store(pkt.target_x);
                    atomic_tar_y_meters.store(pkt.target_y);
                    std::cout << "[CMD] New Target Set: (" << pkt.target_x << ", " << pkt.target_y << ") m" << std::endl;
                    break;
            }
        }
        usleep(10000);
    }
}

// ─────────────────────────────────────────
// Thread 4: 통제소 → GUI 상태 전송 (50ms)
// ─────────────────────────────────────────
void thread_gui_tx() {
    while (system_running.load()) {
        std::vector<std::vector<PointGrid>> grid_copy;
        float tx_x, tx_y, tx_yaw;
        {
            std::lock_guard<std::mutex> lock(mtx);
            grid_copy = shared_grid_obstacles;
            
            // 어뢰 위치 상태 데이터 미터 단위 매핑
            tx_x = (current_torpedo_state.position.x == 0 && current_torpedo_state.position.y == 0)
                   ? std::nanf("") : current_torpedo_state.position.x / 1000.0f;
            tx_y = (current_torpedo_state.position.x == 0 && current_torpedo_state.position.y == 0)
                   ? std::nanf("") : current_torpedo_state.position.y / 1000.0f;
            tx_yaw = std::isnan(current_torpedo_state.heading)
                     ? std::nanf("") : current_torpedo_state.heading * (M_PI / 180.0f); // rad 변환
        }

        // UART 및 UDP 패킷 송신 일원화
        uart_gui->sendGuiPacket(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);
        network->sendGuiPacketUDP(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);

        gui_seq++;
        usleep(50000);
    }
}

// ─────────────────────────────────────────
// Thread 5: 정밀 지연 보상 제어 루프 및 어뢰 Downlink (10ms)
// ─────────────────────────────────────────
void thread_control_loop() {
    while (system_running.load()) {
        auto loop_start = std::chrono::steady_clock::now();
        float cur_x = atomic_torpedo_x_m.load();
        float cur_y = atomic_torpedo_y_m.load();

        float target_y_compensated = 0.0f;
        if (astar_planner && atomic_astar_valid.load()) {
            target_y_compensated = astar_planner->getNextWaypointY(cur_x, 0.15f);
        }

        TorpedoPose state_copy;
        {
            std::lock_guard<std::mutex> lock(mtx);
            state_copy = current_torpedo_state;
        }
        
        // 제어기 연산 수행을 통해 장치 제어에 매핑할 조타각 물리 수치 획득
                int physical_servo_angle = latency_controller->compute(
                    state_copy.position.y / 1000.0f,   // current_y (m)
                    target_y_compensated,               // target_y (m)
                    0.01f                               // dt = 10ms
                );

        if (uart_torpedo) {
            // 하위 보드로 전송할 실시간 시스템 비트 플래그 조립
            uint8_t tx_flags = 0;
            if (atomic_lidar_valid.load()) tx_flags |= 0x01; // 비트 0: 라이다 트래킹 유효 상태
            if (atomic_astar_valid.load()) tx_flags |= 0x02; // 비트 1: A* 글로벌 경로 계획 유효 상태

            // [동기화 완료] 수정된 UartNode 인터페이스 규격에 부합하도록 순서 및 타입 매핑 호출
            uart_torpedo->sendDownlink(
                atomic_tar_x_meters.load(),
                target_y_compensated,
                atomic_lidar_valid.load() && is_launched ? cur_x : std::nanf(""),
                atomic_lidar_valid.load() && is_launched ? cur_y : std::nanf(""),
                static_cast<int16_t>(physical_servo_angle),
                tx_flags,
                downlink_seq++
            );
        }
        usleep(10000);
    }
}

// ─────────────────────────────────────────
// Thread 6: Network TCP Accept 처리
// ─────────────────────────────────────────
void thread_tcp_accept() {
    while (system_running.load()) {
        network->acceptTcpClient();
        usleep(500000);
    }
}

// ─────────────────────────────────────────
// Thread 7: 노트북 GUI → UDP 수신 명령 처리
// ─────────────────────────────────────────
void thread_udp_rx() {
    GuiCommandPacket pkt;
    while (system_running.load()) {
        if (network->receiveTargetUDP(pkt)) {
            if (pkt.type == PKT_TYPE_TARGET) {
                std::lock_guard<std::mutex> lock(mtx);
                gui_target.x = pkt.target_x * 1000.0f;
                gui_target.y = pkt.target_y * 1000.0f;
                atomic_tar_x_meters.store(pkt.target_x);
                atomic_tar_y_meters.store(pkt.target_y);
                fprintf(stderr, "[UDP RX] Target Set: (%.3f, %.3f) m\n", pkt.target_x, pkt.target_y);
                fflush(stderr);
            }
        }
        usleep(5000);
    }
}

// ─────────────────────────────────────────
// Thread 8: A* 주기적 경로 계획 스레드 (200ms)
// ─────────────────────────────────────────
void thread_astar_path_planning() {
    fprintf(stderr, "[Thread] A* Path Planner active (200ms period)\n");
    while (system_running.load()) {
        auto t_start = std::chrono::steady_clock::now();

        if (astar_planner) {
            // 미터 단위 구조체 매핑 연산 실행
            Waypoint start_pos = { atomic_torpedo_x_m.load(), atomic_torpedo_y_m.load() };
            Waypoint goal_pos  = { atomic_tar_x_meters.load(), atomic_tar_y_meters.load() };
            
            astar_planner->computePath(start_pos, goal_pos);
            atomic_astar_valid.store(true);
        }

        auto t_elapsed = std::chrono::steady_clock::now() - t_start;
        auto sleep_dur = std::chrono::milliseconds(200) - std::chrono::duration_cast<std::chrono::milliseconds>(t_elapsed);
        if (sleep_dur.count() > 0) {
            std::this_thread::sleep_for(sleep_dur);
        }
    }
}

// ── 시스템 메인 진입점 ────────────────────────────────────
int main() {
    system_start_time = std::chrono::steady_clock::now();
    const int LATENCY_MS = 80;
    
    fprintf(stderr, "=== Control Station Booting ===\n");
    fflush(stderr);
    
    // 1. Lidar 초기화
    fprintf(stderr, "[1/7] Initializing Lidar...\n");
    try {
        lidar = new LidarNode("/dev/ttyUSB0", 460800);
        fprintf(stderr, "[1/7] Lidar OK\n");
    } catch (...) {
        fprintf(stderr, "[1/7] Lidar FAILED\n");
        return 1;
    }
    
    // 2. UART Torpedo 초기화
    fprintf(stderr, "[2/7] Initializing UART Torpedo...\n");
    try {
        uart_torpedo = new UartNode("/dev/ttyS2", 460800);
        fprintf(stderr, "[2/7] UART Torpedo OK\n");
    } catch (...) {
        fprintf(stderr, "[2/7] UART Torpedo FAILED\n");
        return 1;
    }
    
    // 3. UART GUI 초기화
    fprintf(stderr, "[3/7] Initializing UART GUI...\n");
    try {
        uart_gui = new UartNode("/dev/ttyS0", 115200);
        fprintf(stderr, "[3/7] UART GUI OK\n");
    } catch (...) {
        fprintf(stderr, "[3/7] UART GUI FAILED\n");
        return 1;
    }
    
    // 4. Servo 초기화
    fprintf(stderr, "[4/7] Initializing Servo...\n");
    try {
        hatch_servo = new ServoNode(0, 0);
        hatch_servo->setAngle(90);
        fprintf(stderr, "[4/7] Servo OK\n");
    } catch (...) {
        fprintf(stderr, "[4/7] Servo FAILED\n");
        return 1;
    }
    
    // 5. LatencyAwareCtrl & A* 플래너 초기화
    fprintf(stderr, "[5/7] Initializing Control & Planner Logic...\n");
    try {
        latency_controller = new LatencyAwareCtrl();
        astar_planner = new AStarPlanner(LATENCY_MS, 0.15f);
        fprintf(stderr, "[5/7] Control & Planner Logic OK\n");
    } catch (...) {
        fprintf(stderr, "[5/7] Control Logic FAILED\n");
        return 1;
    }
    
    // 6. Network 초기화
    fprintf(stderr, "[6/7] Initializing Network...\n");
    try {
        network = new NetworkNode("192.168.1.100", 4000, 4001, 5000);
        fprintf(stderr, "[6/7] Network OK\n");
    } catch (...) {
        fprintf(stderr, "[6/7] Network FAILED\n");
        return 1;
    }
    
    fprintf(stderr, "\n=== All nodes initialized ===\n");
    fprintf(stderr, "Starting threads...\n");
    fflush(stderr);
    
    std::thread t1(thread_lidar_tracking); fprintf(stderr, "Thread 1: Lidar tracking started\n");
    std::thread t2(thread_torpedo_rx);      fprintf(stderr, "Thread 2: Torpedo RX started\n");
    std::thread t3(thread_gui_rx);          fprintf(stderr, "Thread 3: GUI RX started\n");
    std::thread t4(thread_gui_tx);          fprintf(stderr, "Thread 4: GUI TX started\n");
    std::thread t5(thread_control_loop);    fprintf(stderr, "Thread 5: Control loop started\n");
    std::thread t6(thread_tcp_accept);      fprintf(stderr, "Thread 6: TCP Accept started\n");
    std::thread t7(thread_udp_rx);          fprintf(stderr, "Thread 7: UDP RX started\n");
    std::thread t8(thread_astar_path_planning); fprintf(stderr, "Thread 8: A* Planner started\n");
    
    fprintf(stderr, "\n=== System Full-Running ===\n\n");
    fflush(stderr);

    t1.join(); t2.join(); t3.join(); t4.join(); 
    t5.join(); t6.join(); t7.join(); t8.join();
    
    // 객체 파괴 및 메모리 해제
    delete lidar;
    delete uart_torpedo;
    delete uart_gui;
    delete hatch_servo;
    delete latency_controller;
    delete astar_planner;
    delete network;

    return 0;
}