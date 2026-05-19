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
#include "ControlLogic.h"
#include "NetworkNode.h"

// --- 공유 자원 ---
std::mutex mtx;
TorpedoPose current_torpedo_state = {{0,0}, 0, 90.0, 0, 0};
Point2D gui_target = {5000, 5000};
bool is_launched = false;
uint16_t downlink_seq = 0;
uint16_t gui_seq = 0;
std::chrono::steady_clock::time_point system_start_time;
std::vector<std::vector<PointGrid>> shared_grid_obstacles;

// --- 전역 포인터 ---
LidarNode*    lidar = nullptr;
UartNode*     uart_torpedo = nullptr;
UartNode*     uart_gui = nullptr;
ServoNode*    hatch_servo = nullptr;
ControlLogic* brain = nullptr;
NetworkNode*  network = nullptr;

// ─────────────────────────────────────────
// Thread 1: 라이다 → 클러스터 공유 + 어뢰로 Downlink (10ms)
// ─────────────────────────────────────────
void thread_lidar_tracking() {
    Point2D car_pos;
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        // 클러스터 데이터 갱신 (GUI용)
        auto grid_obs = lidar->getGridObstacles();
        {
            std::lock_guard<std::mutex> lock(mtx);
            shared_grid_obstacles = grid_obs;
        }

        uint32_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
            loop_start - system_start_time).count();

        Point2D target_copy;
        {
            std::lock_guard<std::mutex> lock(mtx);
            target_copy = gui_target;
        }

        float tx = target_copy.x / 1000.0f;
        float ty = target_copy.y / 1000.0f;

        // 어뢰로 Downlink 전송
        if (is_launched && lidar->getDynamicCarPosition(car_pos)) {
            uart_torpedo->sendDownlink(tx, ty,
                car_pos.x / 1000.0f, car_pos.y / 1000.0f,
                ts, downlink_seq++);
        } else {
            uart_torpedo->sendDownlink(tx, ty,
                std::nanf(""), std::nanf(""),
                ts, downlink_seq++);
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
    while (true) {
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
// Thread 3: GUI → 통제소 명령 수신
// ─────────────────────────────────────────
void thread_gui_rx() {
    GuiCommandPacket pkt;
    while (true) {
        bool got_cmd = false;
        // UART 수신
        // got_cmd = uart_gui->receiveGuiCommand(pkt);

        // TCP 수신 (연결된 경우)
        if (!got_cmd && network->isTcpConnected()) {
            got_cmd = network->receiveCommandTCP(pkt);
        }

        if (got_cmd) {
            switch (pkt.type) {
                case PKT_TYPE_OPEN:
                    hatch_servo->setAngle(pkt.cmd_data ? 180 : 90);
                    std::cout << "[CMD] Hatch "
                              << (pkt.cmd_data ? "OPEN" : "CLOSE")
                              << std::endl;
                    break;
                case PKT_TYPE_FIRE:
                    std::cout << "[CMD] FIRE (미구현)" << std::endl;
                    break;
                case PKT_TYPE_ENDGUIDE:
                    std::cout << "[CMD] END GUIDE (미구현)" << std::endl;
                    break;
                case PKT_TYPE_TARGET:
                    std::cout << "[CMD] TARGET (미구현)" << std::endl;
                    break;
            }
        }
        usleep(10000);
    }
}
void thread_gui_tx() {
    while (true) {
        std::vector<std::vector<PointGrid>> grid_copy;
        float tx_x, tx_y, tx_yaw;
        {
            std::lock_guard<std::mutex> lock(mtx);
            grid_copy = shared_grid_obstacles;
            tx_x = (current_torpedo_state.position.x == 0 &&
                    current_torpedo_state.position.y == 0)
                   ? std::nanf("")
                   : current_torpedo_state.position.x / 1000.0f;
            tx_y = (current_torpedo_state.position.x == 0 &&
                    current_torpedo_state.position.y == 0)
                   ? std::nanf("")
                   : current_torpedo_state.position.y / 1000.0f;
            tx_yaw = std::isnan(current_torpedo_state.heading)
                     ? std::nanf("")
                     : current_torpedo_state.heading;
        }

        // UART 전송
        uart_gui->sendGuiPacket(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);
        // fprintf(stderr, "[GUI TX] seq:%u torpedo:(%.2f,%.2f) clusters:%zu\n",
        //         gui_seq,
        //         std::isnan(tx_x) ? -1.0f : tx_x,
        //         std::isnan(tx_y) ? -1.0f : tx_y,
        //         grid_copy.size());
        // fflush(stderr);

        // UDP 전송 (동일 패킷)
        network->sendGuiPacketUDP(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);

        gui_seq++;
        usleep(50000);
    }
}

void thread_control_loop() {
    while (true) {
        if (is_launched) {
            TorpedoPose state_copy;
            Point2D target_copy;
            {
                std::lock_guard<std::mutex> lock(mtx);
                state_copy  = current_torpedo_state;
                target_copy = gui_target;
            }
            int steering = brain->calculateSteering(state_copy, target_copy);
            fprintf(stderr, "[Control] Steering: %d\n", steering);
            fflush(stderr);
        }
        usleep(500000);
    }
}

void thread_tcp_accept() {
    network->acceptTcpClient();
}

void thread_udp_rx() {
    GuiCommandPacket pkt;
    while (true) {
        if (network->receiveTargetUDP(pkt)) {
            if (pkt.type == PKT_TYPE_TARGET) {
                std::lock_guard<std::mutex> lock(mtx);
                gui_target.x = pkt.target_x * 1000.0f;
                gui_target.y = pkt.target_y * 1000.0f;
                fprintf(stderr, "[UDP] Target: (%.3f, %.3f) m\n", pkt.target_x, pkt.target_y);
                fflush(stderr);
            }
        }
        usleep(5000);
    }
}

int main() {
    system_start_time = std::chrono::steady_clock::now();
    
    fprintf(stderr, "=== Control Station Booting ===\n");
    fflush(stderr);
    
    // 1. Lidar 초기화
    fprintf(stderr, "[1/6] Initializing Lidar...\n");
    fflush(stderr);
    try {
        lidar = new LidarNode("/dev/ttyUSB0", 460800);
        fprintf(stderr, "[1/6] Lidar OK\n");
    } catch (...) {
        fprintf(stderr, "[1/6] Lidar FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    // 2. UART Torpedo 초기화
    fprintf(stderr, "[2/6] Initializing UART Torpedo...\n");
    fflush(stderr);
    try {
        uart_torpedo = new UartNode("/dev/ttyS2", 460800);
        fprintf(stderr, "[2/6] UART Torpedo OK\n");
    } catch (...) {
        fprintf(stderr, "[2/6] UART Torpedo FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    // 3. UART GUI 초기화
    fprintf(stderr, "[3/6] Initializing UART GUI...\n");
    fflush(stderr);
    try {
        uart_gui = new UartNode("/dev/ttyS0", 115200);
        fprintf(stderr, "[3/6] UART GUI OK\n");
    } catch (...) {
        fprintf(stderr, "[3/6] UART GUI FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    // 4. Servo 초기화
    fprintf(stderr, "[4/6] Initializing Servo...\n");
    fflush(stderr);
    try {
        hatch_servo = new ServoNode(0, 0);
        hatch_servo->setAngle(90);
        fprintf(stderr, "[4/6] Servo OK\n");
    } catch (...) {
        fprintf(stderr, "[4/6] Servo FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    // 5. Control Logic 초기화
    fprintf(stderr, "[5/6] Initializing Control Logic...\n");
    fflush(stderr);
    try {
        brain = new ControlLogic();
        fprintf(stderr, "[5/6] Control Logic OK\n");
    } catch (...) {
        fprintf(stderr, "[5/6] Control Logic FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    // 6. Network 초기화
    fprintf(stderr, "[6/6] Initializing Network...\n");
    fflush(stderr);
    try {
        network = new NetworkNode(
            "169.254.190.22",    // ← 노트북 IP
            4000,               // UDP TX 포트 (Zynq → 노트북)
            4001,               // UDP RX 포트 (노트북 → Zynq)
            5000                // TCP 서버 포트
        );
        fprintf(stderr, "[6/6] Network OK\n");
    } catch (...) {
        fprintf(stderr, "[6/6] Network FAILED\n");
        return 1;
    }
    fflush(stderr);
    
    fprintf(stderr, "\n=== All nodes initialized ===\n");
    fprintf(stderr, "Starting threads...\n");
    fflush(stderr);
    
    std::thread t1(thread_lidar_tracking);
    fprintf(stderr, "Thread 1: Lidar tracking started\n");
    fflush(stderr);
    
    std::thread t2(thread_torpedo_rx);
    fprintf(stderr, "Thread 2: Torpedo RX started\n");
    fflush(stderr);
    
    std::thread t3(thread_gui_rx);
    fprintf(stderr, "Thread 3: GUI RX started\n");
    fflush(stderr);
    
    std::thread t4(thread_gui_tx);
    fprintf(stderr, "Thread 4: GUI TX started\n");
    fflush(stderr);
    
    std::thread t5(thread_control_loop);
    fprintf(stderr, "Thread 5: Control loop started\n");
    fflush(stderr);
    
    std::thread t6(thread_tcp_accept);
    fprintf(stderr, "Thread 6: TCP accept started\n");
    fflush(stderr);
    
    std::thread t7(thread_udp_rx);
    fprintf(stderr, "Thread 7: UDP RX started\n");
    fflush(stderr);
    
    fprintf(stderr, "\n=== System Running ===\n\n");
    fflush(stderr);

    t1.join(); t2.join(); t3.join(); t4.join();
    t5.join(); t6.join(); t7.join();
    
    return 0;
}