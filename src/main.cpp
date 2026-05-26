#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <arpa/inet.h> 
#include <memory> // 스마트 포인터 사용을 위해 추가

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

// 목표물 좌표 (GUI로 수신)
std::atomic<float> atomic_tar_x_meters{5.0f};  
std::atomic<float> atomic_tar_y_meters{0.0f};

// 라이다 추적 결과
std::atomic<float> atomic_torpedo_x_m{0.0f};
std::atomic<float> atomic_torpedo_y_m{0.0f};
std::atomic<bool>  atomic_lidar_valid{false};

// 경로 계획 상태
std::atomic<bool>  atomic_astar_valid{false};

// 유도 모드 제어
std::atomic<bool>  is_terminal_guidance{false}; 
std::atomic<bool> system_running{true};

// ── 전역 포인터 (스마트 포인터로 변경) ──────────────────────────────────
std::unique_ptr<LidarNode> lidar;
std::unique_ptr<UartNode> uart_torpedo;
std::unique_ptr<UartNode> uart_gui;
std::unique_ptr<ServoNode> hatch_servo;
std::unique_ptr<NetworkNode> network;
std::unique_ptr<AStarPlanner> astar_planner;
std::unique_ptr<LatencyAwareCtrl> latency_controller;

// ─────────────────────────────────────────
// Thread 1: 라이다 데이터 수집 및 공유 (10ms)
// ─────────────────────────────────────────
void thread_lidar_tracking() {
    Point2D car_pos;
    std::vector<std::vector<PointGrid>> local_grid_obs;

    while (system_running.load()) {
        auto loop_start = std::chrono::steady_clock::now();

        // 통합 함수 딱 하나만 호출하여 데이터와 좌표를 동시에 획득!
        if (lidar->processLidarFrame(car_pos, local_grid_obs)) {
            
            // 1. GUI 전송용 공유 격자 맵 업데이트
            {
                std::lock_guard<std::mutex> lock(mtx);
                shared_grid_obstacles = local_grid_obs;
            }

            // 2. 제어 루프용 어뢰 좌표 원자적 업데이트
            atomic_torpedo_x_m.store(car_pos.x / 1000.0f);
            atomic_torpedo_y_m.store(car_pos.y / 1000.0f);
            atomic_lidar_valid.store(true);
        } else {
            // 어뢰를 놓쳤을 때 처리
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

        if (!got_cmd && network->isTcpConnected()) {
            got_cmd = network->receiveCommandTCP(pkt);
        }

        if (got_cmd) {
            switch (pkt.type) {
                case CMD_OPEN:  // 0x11
                    hatch_servo->setAngle(pkt.cmd_data ? 180 : 90);
                    fprintf(stderr, "[CMD] Hatch %s\n",
                            pkt.cmd_data ? "OPEN" : "CLOSE");
                    break;

                case CMD_FIRE:  // 0x12 → 중기유도 시작
                    is_launched = true;
                    is_terminal_guidance.store(false);
                    fprintf(stderr, "[CMD] FIRE → 중기유도(0x01) 시작\n");
                    break;

                case CMD_ENDGUIDE:  // 0x13 → 종말유도 트리거
                    is_terminal_guidance.store(true);
                    fprintf(stderr, "[CMD] ENDGUIDE → 종말유도(0x02) 전송\n");
                    break;

                case CMD_TARGET:  // 0x10 → 목표 좌표 업데이트
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    gui_target.x = pkt.target_x * 1000.0f;
                    gui_target.y = pkt.target_y * 1000.0f;
                    atomic_tar_x_meters.store(pkt.target_x);
                    atomic_tar_y_meters.store(pkt.target_y);
                    fprintf(stderr, "[CMD] Target: (%.2f, %.2f) m\n",
                            pkt.target_x, pkt.target_y);
                    break;
                }
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
// void thread_control_loop() {
//     while (system_running.load()) {
//         auto loop_start = std::chrono::steady_clock::now();

//         if (uart_torpedo) {
//             uint8_t tx_flags = 0;
            
//             if (is_terminal_guidance.load()) {
//                 // ── [종말유도 상태] 어뢰 자체 제어 위임 ──
//                 tx_flags = FLAG_GUIDANCE_TERMINAL; // 0x02

//                 uart_torpedo->sendDownlink(
//                     std::nanf(""),  // target_x 없음
//                     std::nanf(""),  // target_y 없음
//                     std::nanf(""),  // torpedo_x 없음
//                     std::nanf(""),  // torpedo_y 없음
//                     0,              // steer 없음
//                     tx_flags,       // 0x02 전송
//                     downlink_seq++
//                 );
//             } else if (is_launched) {
//                 // ── [중기유도 상태] 통제소에서 조향각 계산 ──
//                 tx_flags = FLAG_GUIDANCE_MIDCOURSE; // 0x01

//                 float cur_x = atomic_torpedo_x_m.load();
//                 float cur_y = atomic_torpedo_y_m.load();
//                 float target_y_compensated = 0.0f;

//                 if (astar_planner && atomic_astar_valid.load()) {
//                     target_y_compensated = astar_planner->getNextWaypointY(cur_x, 0.15f);
//                 }

//                 TorpedoPose state_copy;
//                 {
//                     std::lock_guard<std::mutex> lock(mtx);
//                     state_copy = current_torpedo_state;
//                 }

//                 int steer = latency_controller->compute(
//                     state_copy.position.y / 1000.0f,
//                     target_y_compensated,
//                     0.01f
//                 );

//                 uart_torpedo->sendDownlink(
//                     atomic_tar_x_meters.load(),
//                     target_y_compensated,
//                     atomic_lidar_valid.load() ? cur_x : std::nanf(""),
//                     atomic_lidar_valid.load() ? cur_y : std::nanf(""),
//                     static_cast<int16_t>(steer),
//                     tx_flags,       // 0x01 전송
//                     downlink_seq++
//                 );
//             }
//         }

//         auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
//             std::chrono::steady_clock::now() - loop_start).count();
//         long sleep_us = 10000 - elapsed;
//         if (sleep_us > 0) usleep(sleep_us);
//     }
// }

// ─────────────────────────────────────────
// Thread 5: 정밀 지연 보상 제어 루프 및 어뢰 Downlink (10ms)
// ─────────────────────────────────────────
// void thread_control_loop() {
//     while (system_running.load()) {
//         auto loop_start = std::chrono::steady_clock::now();

//         if (uart_torpedo) {
//             uint8_t tx_flags = 0;
            
//             if (is_terminal_guidance.load()) {
//                 // ── [종말유도 상태] 어뢰 자체 제어 위임 ──
//                 tx_flags = FLAG_GUIDANCE_TERMINAL; // 0x02

//                 uart_torpedo->sendDownlink(
//                     std::nanf(""),  // target_x 없음
//                     std::nanf(""),  // target_y 없음
//                     std::nanf(""),  // torpedo_x 없음
//                     std::nanf(""),  // torpedo_y 없음
//                     0,              // steer 없음
//                     tx_flags,       // 0x02 전송
//                     downlink_seq++
//                 );
//         //     } else if (is_launched) {
//         //         // ── [중기유도 상태] 통제소에서 조향각 계산 ──
//         //         tx_flags = FLAG_GUIDANCE_MIDCOURSE; // 0x01

//         //         // 💡 1. 라이다가 측정한 생(Raw) 좌표 읽기 (어뢰 다운링크 전송용)
//         //         float lidar_x = atomic_torpedo_x_m.load();
//         //         float lidar_y = atomic_torpedo_y_m.load();

//         //         // 💡 2. 뮤텍스 락을 위로 이동: 어뢰가 보낸 업링크 데이터 최우선 복사
//         //         TorpedoPose state_copy;
//         //         {
//         //             std::lock_guard<std::mutex> lock(mtx);
//         //             state_copy = current_torpedo_state;
//         //         }

//         //         // 💡 3. 제어에 사용할 어뢰 피드백 위치 추출 (미터 단위 변환)
//         //         // ※ 만약 Uplink 수신부에서 이미 미터 단위로 저장했다면 /1000.0f을 지워주세요!
//         //         float uplink_x_m = state_copy.position.x;
//         //         float uplink_y_m = state_copy.position.y;

//         //         // float uplink_yaw = state_copy.yaw; // 필요 시 제어기 확장용

//         //         // 💡 4. [변경] 라이다 X 대신 '어뢰 업링크 X'를 기준으로 A* 다음 웨이포인트 탐색
//         //         float target_y_compensated = 0.0f;
//         //         if (astar_planner && atomic_astar_valid.load()) {
//         //             target_y_compensated = astar_planner->getNextWaypointY(uplink_x_m, 0.15f);
//         //         }

//         //         // 💡 5. [변경] '어뢰 업링크 Y'와 경로점 오차를 계산하여 Latency_Aware 제어 수행
//         //         int steer = latency_controller->compute(
//         //             uplink_y_m,
//         //             target_y_compensated,
//         //             0.01f
//         //         );

//         //         // 💡 6. 다운링크 전송: torpedo_x/y 자리에는 라이다 생(Raw) 측정값을 실어서 송신
//         //         uart_torpedo->sendDownlink(
//         //             atomic_tar_x_meters.load(),
//         //             target_y_compensated,
//         //             atomic_lidar_valid.load() ? lidar_x : std::nanf(""),
//         //             atomic_lidar_valid.load() ? lidar_y : std::nanf(""),
//         //             static_cast<int16_t>(steer),
//         //             tx_flags,       // 0x01 전송
//         //             downlink_seq++
//         //         );

//         //         // 💡 7. [추가] 어뢰로 나가는 데이터 터미널 출력 (100ms에 한 번씩 샘플링)
//         //         static int tx_log_counter = 0;
//         //         if (tx_log_counter++ % 100 == 0) {
//         //             float current_tar_x = atomic_tar_x_meters.load();
//         //             fprintf(stdout, "\n==================================================\n");
//         //             fprintf(stdout, "[Zynq -> Torpedo Downlink Packet Transmitted]\n");
//         //             fprintf(stdout, "  |- [SEQ]       : %u\n", downlink_seq - 1);
//         //             fprintf(stdout, "  |- [TARGET]    : X = %.2f m, Y = %.2f m  (From GUI)\n", current_tar_x, target_y_compensated);
//         //             fprintf(stdout, "  |- [LIDAR_EST] : X = %.2f m, Y = %.2f m  (To Torpedo)\n", 
//         //                     atomic_lidar_valid.load() ? lidar_x : 0.0f, 
//         //                     atomic_lidar_valid.load() ? lidar_y : 0.0f);
//         //             fprintf(stdout, "  |- [STEER]     : %d deg\n", static_cast<int16_t>(steer));
//         //             fprintf(stdout, "  |- [FLAGS]     : 0x%02X  (0x01:MID, 0x02:TERM)\n", tx_flags);
//         //             fprintf(stdout, "==================================================\n");
//         //             fprintf(stdout, "[Torpedo -> Zynq RX] Ctrl_Uplink:(%.2f, %.2f)\n",uplink_x_m, uplink_y_m);
//         //         }
//         //     }

//             } else if (is_launched) {
//                 // ── [중기유도 상태] 통제소에서 조향각 계산 ──
//                 tx_flags = FLAG_GUIDANCE_MIDCOURSE; // 0x01

//                 float lidar_x = atomic_torpedo_x_m.load();
//                 float lidar_y = atomic_torpedo_y_m.load();

//                 TorpedoPose state_copy;
//                 {
//                     std::lock_guard<std::mutex> lock(mtx);
//                     state_copy = current_torpedo_state;
//                 }

//                 // [임시] 통신 전이므로 피드백 기준은 라이다로 설정
//                 float control_x = lidar_x; 
//                 float control_y = lidar_y; 

//                 // 1. Zynq 내부 제어용 웨이포인트 Y 계산 (어뢰 전송용 X)
//                 float target_y_compensated = 0.0f;
//                 if (astar_planner && atomic_astar_valid.load()) {
//                     target_y_compensated = astar_planner->getNextWaypointY(control_x, 0.15f);
//                 }

//                 // 2. Zynq 내부 조향각 계산 (임시 15cm 앞 웨이포인트 기준)
//                 int steer = latency_controller->compute(
//                     control_y,
//                     target_y_compensated,
//                     0.01f
//                 );

//                 // 3. 🎯 다운링크 전송 수정
//                 // target_x, target_y 자리에는 "GUI 원본 최종 목적지"를 그대로 넣어줍니다.
//                 float raw_target_x = atomic_tar_x_meters.load();
//                 float raw_target_y = atomic_tar_y_meters.load(); // 💡 GUI 원본 Y 축 원자적 변수 사용

//                 uart_torpedo->sendDownlink(
//                     raw_target_x,                                       // 어뢰가 기억할 최종 목적지 X
//                     raw_target_y,                                       // 어뢰가 기억할 최종 목적지 Y (보정값 X)
//                     atomic_lidar_valid.load() ? lidar_x : std::nanf(""),
//                     atomic_lidar_valid.load() ? lidar_y : std::nanf(""),
//                     static_cast<int16_t>(steer),                        // Zynq가 계산한 중기유도 조향 명령
//                     tx_flags,
//                     downlink_seq++
//                 );

//                 // 4. 로그 출력 부분도 매칭되도록 수정
//                 static int tx_log_counter = 0;
//                 if (tx_log_counter++ % 100 == 0) {
//                     fprintf(stdout, "\n==================================================\n");
//                     fprintf(stdout, "[Zynq -> Torpedo Downlink Packet Transmitted]\n");
//                     fprintf(stdout, "  |- [SEQ]       : %u\n", downlink_seq - 1);
//                     fprintf(stdout, "  |- [FINAL TG]  : X = %.2f m, Y = %.2f m  (To Torpedo Terminal)\n", raw_target_x, raw_target_y);
//                     fprintf(stdout, "  |- [ZYNQ WP Y] : %.2f m (Internal Midcourse Look-ahead)\n", target_y_compensated);
//                     fprintf(stdout, "  |- [LIDAR_EST] : X = %.2f m, Y = %.2f m  [Valid: %s]\n", lidar_x, lidar_y, atomic_lidar_valid.load() ? "OK" : "FAIL");
//                     fprintf(stdout, "  |- [STEER]     : %d deg\n", static_cast<int16_t>(steer));
//                     fprintf(stdout, "  |- [FLAGS]     : 0x%02X  (0x01:MID, 0x02:TERM)\n", tx_flags);
//                     fprintf(stdout, "==================================================\n");
//                 }
//             }
//         }


//         auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
//             std::chrono::steady_clock::now() - loop_start).count();
//         long sleep_us = 10000 - elapsed;
//         if (sleep_us > 0) usleep(sleep_us);
//     }
// }
void thread_control_loop() {
    while (system_running.load()) {
        auto loop_start = std::chrono::steady_clock::now();

        if (uart_torpedo) {
            uint8_t tx_flags = 0;
            
            if (is_terminal_guidance.load()) {
                // ── [종말유도 상태] 생략 (기존 코드 유지) ──
                tx_flags = FLAG_GUIDANCE_TERMINAL;
                uart_torpedo->sendDownlink(std::nanf(""), std::nanf(""), std::nanf(""), std::nanf(""), 0, tx_flags, downlink_seq++);
            } 
            else if (is_launched) {
                // ── [중기유도 상태] 통제소 제어 및 데이터 융합 ──
                tx_flags = FLAG_GUIDANCE_MIDCOURSE; // 0x01

                // 라이다 동적 객체 데이터 추출 -> 어뢰의 위치 추정(참고용)을 위해 다운링크로 쏴줌
                float lidar_x = atomic_torpedo_x_m.load();
                float lidar_y = atomic_torpedo_y_m.load();

                TorpedoPose state_copy;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    state_copy = current_torpedo_state; // 어뢰가 보낸 최신 Uplink 패킷 복사
                }

                // 1. 🎯 피드백 기준점 설정: "어뢰 자체 추정 좌표" 활용
                float control_x = state_copy.position.x; 
                float control_y = state_copy.position.y; 

                // 2. A* 기반 지연 선행 보정 웨이포인트 Y 계산 (어뢰의 X 좌표 기준)
                float target_y_compensated = 0.0f;
                if (astar_planner && atomic_astar_valid.load()) {
                    target_y_compensated = astar_planner->getNextWaypointY(control_x, 0.15f);
                }

                // 3. 기하학적 목표 헤딩(Target Yaw) 계산 (라디안 단위 결과 도출)
                float look_ahead_dist = (0.15f * 0.08f) + 0.12f; 
                float target_yaw = std::atan2(target_y_compensated - control_y, look_ahead_dist);

                // 4. 🎯 어뢰 실제 헤딩 추출 (헤더 정의에 따라 degree -> radian 변환 적용)
                float current_yaw = state_copy.heading * (M_PI / 180.0f);

                // 5. 🎯 등속도 운행 기반 융합 제어 알고리즘 호출 -> 조향각(steer) 도출
                int steer = latency_controller->compute(
                    control_y,
                    target_y_compensated,
                    0.01f,          // dt (10ms)
                    current_yaw,    // 어뢰 실제 헤딩 (라디안)
                    target_yaw      // 목적지 헤딩 (라디안)
                );

                // 6. 다운링크 전송 (원 목적지 + 라이다 참조 좌표 + 계산된 steer 명령)
                float raw_target_x = atomic_tar_x_meters.load();
                float raw_target_y = atomic_tar_y_meters.load(); 

                uart_torpedo->sendDownlink(
                    raw_target_x,                                       
                    raw_target_y,                                       
                    atomic_lidar_valid.load() ? lidar_x : std::nanf(""), // 어뢰 전송용 라이다 X
                    atomic_lidar_valid.load() ? lidar_y : std::nanf(""), // 어뢰 전송용 라이다 Y
                    static_cast<int16_t>(steer),                         // 어뢰를 움직일 조향 제어 명령
                    tx_flags,
                    downlink_seq
                );

                // 7. 🎯 100개 단위(1초) TX & RX 데이터 대시보드 출력
                static int tx_log_counter = 0;
                if (tx_log_counter++ % 100 == 0) {
                    fprintf(stdout, "\n==================================================\n");
                    fprintf(stdout, "[Zynq <-> Torpedo 100-Loop Sync Dashboard]\n");
                    
                    // --- RX (어뢰에서 들어온 값 구조체 매핑 확인) ---
                    fprintf(stdout, " [RX UPLINK]  SEQ: %u | Flags: 0x%02X\n", state_copy.seq, state_copy.flags);
                    fprintf(stdout, "  |- Torpedo Pos: X = %.2f m, Y = %.2f m\n", state_copy.position.x, state_copy.position.y);
                    fprintf(stdout, "  |- Torpedo Yaw: %.2f deg (Internal: %.4f rad)\n", state_copy.heading, current_yaw);
                    
                    fprintf(stdout, " -------------------------------------------------\n");
                    
                    // --- TX (어뢰로 날아가는 값 구조체 매핑 확인) ---
                    fprintf(stdout, " [TX DOWNLINK] SEQ: %u | Flags: 0x%02X\n", downlink_seq, tx_flags);
                    fprintf(stdout, "  |- Target WP  : X = %.2f m, Y = %.2f m (Comp Y: %.2f m)\n", raw_target_x, raw_target_y, target_y_compensated);
                    fprintf(stdout, "  |- LiDAR Sync : X = %.2f m, Y = %.2f m\n", lidar_x, lidar_y);
                    fprintf(stdout, "  |- Target Yaw : %.2f deg\n", target_yaw * (180.0f / M_PI));
                    fprintf(stdout, "  |- STEER CMD  : %d deg (Mapped: -45 to +45)\n", static_cast<int16_t>(steer));
                    fprintf(stdout, "==================================================\n");
                }
                
                downlink_seq++;
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        long sleep_us = 10000 - elapsed;
        if (sleep_us > 0) usleep(sleep_us);
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
            if (pkt.type == CMD_TARGET) {
                std::lock_guard<std::mutex> lock(mtx);
                gui_target.x = pkt.target_x * 1000.0f;
                gui_target.y = pkt.target_y * 1000.0f;
                atomic_tar_x_meters.store(pkt.target_x);
                atomic_tar_y_meters.store(pkt.target_y);
                // fprintf(stderr, "[UDP RX] Target Set: (%.3f, %.3f) m\n", pkt.target_x, pkt.target_y);
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
        // lidar = std::make_unique<LidarNode>("/dev/ttyUSB0", 460800);
        lidar = std::make_unique<LidarNode>("/dev/ttyPS1", 460800);
        fprintf(stderr, "[1/7] Lidar OK\n");
        std::cout << "\n👉 [Lidar] 주변을 비운 뒤, 배경 학습을 시작하려면 [Enter] 키를 누르세요..." << std::endl;
        std::string dummy;
        std::getline(std::cin, dummy);
        // 💡 [필수 추가] 스레드 켜기 전에 3초 동안 가만히 있는 상태에서 배경 학습!
        lidar->startCalibration(3); 
        
    } catch (...) {
        fprintf(stderr, "[1/7] Lidar FAILED\n");
        return 1;
    }
    
    // 2. UART Torpedo 초기화
    fprintf(stderr, "[2/7] Initializing UART Torpedo...\n");
    try {
        uart_torpedo = std::make_unique<UartNode>("/dev/ttyS2", 115200);
        fprintf(stderr, "[2/7] UART Torpedo OK\n");
    } catch (...) {
        fprintf(stderr, "[2/7] UART Torpedo FAILED\n");
        return 1;
    }
    
    // 3. UART GUI 초기화
    fprintf(stderr, "[3/7] Initializing UART GUI...\n");
    try {
        uart_gui = std::make_unique<UartNode>("/dev/ttyS0", 115200);
        fprintf(stderr, "[3/7] UART GUI OK\n");
    } catch (...) {
        fprintf(stderr, "[3/7] UART GUI FAILED\n");
        return 1;
    }
    
    // 4. Servo 초기화
    fprintf(stderr, "[4/7] Initializing Servo...\n");
    try {
        hatch_servo = std::make_unique<ServoNode>(0, 0);
        hatch_servo->setAngle(90);
        fprintf(stderr, "[4/7] Servo OK\n");
    } catch (...) {
        fprintf(stderr, "[4/7] Servo FAILED\n");
        return 1;
    }
    
    // 5. LatencyAwareCtrl & A* 플래너 초기화
    fprintf(stderr, "[5/7] Initializing Control & Planner Logic...\n");
    try {
        latency_controller = std::make_unique<LatencyAwareCtrl>();
        astar_planner = std::make_unique<AStarPlanner>(LATENCY_MS, 0.15f);
        fprintf(stderr, "[5/7] Control & Planner Logic OK\n");
    } catch (...) {
        fprintf(stderr, "[5/7] Control Logic FAILED\n");
        return 1;
    }
    
    // 6. Network 초기화
    fprintf(stderr, "[6/7] Initializing Network...\n");
    try {
        network = std::make_unique<NetworkNode>("10.0.0.1", 4000, 4001, 5000);
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

    return 0;
}