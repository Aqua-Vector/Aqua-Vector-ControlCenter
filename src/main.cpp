#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <arpa/inet.h> 
#include <memory> 
#include <csignal>

#include "LidarNode.h"
#include "ServoNode.h"
#include "UartNode.h"
#include "NetworkNode.h"
#include "AStarPlanner.h" 
#include "Protocol.h"
#include "LatencyAwareCtrl.h"

// ── 공유 자원 ─────────────────────────────────────────
std::mutex mtx;
TorpedoPose current_torpedo_state = {{0,0}, 0, 0.0, 0, 0};
std::atomic<bool> is_launched{false}; 
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

// ── 전역 포인터 (스마트 포인터) ──────────────────────────────────
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

        if (lidar->processLidarFrame(car_pos, local_grid_obs)) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                shared_grid_obstacles = local_grid_obs;
            }
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
// Thread 2: 어뢰 → 통제소 Uplink 수신 (수정본)
// ─────────────────────────────────────────
void thread_torpedo_rx() {
    UplinkPacket pkt;
    int consecutive_fail_cnt = 0; 

    while (system_running.load()) {
        if (uart_torpedo->receiveUplinkStatus(pkt)) {
            consecutive_fail_cnt = 0; 

            std::lock_guard<std::mutex> lock(mtx);
            
            // 🐛 [버그 수정]: 원본 데이터(m)를 1000배 튀기지 않고 그대로 주입합니다.
            current_torpedo_state.position.x = pkt.p_x; 
            current_torpedo_state.position.y = pkt.p_y;
            
            current_torpedo_state.heading    = pkt.yaw;  
            current_torpedo_state.seq        = pkt.seq;
            current_torpedo_state.flags      = pkt.status_flags;

            // 디버그 출력
            static int rx_sync_cnt = 0;
            if (rx_sync_cnt++ % 100 == 0) {
                fprintf(stdout, "[Thread 2 디버그] 공유 변수 주입 성공! seq: %u, x: %.2f, yaw: %.2f\n", 
                        current_torpedo_state.seq, current_torpedo_state.position.x, current_torpedo_state.heading);
                fflush(stdout);
            }
        } else {
            consecutive_fail_cnt++;

            if (consecutive_fail_cnt >= 100) {
                fprintf(stderr, "[Thread 2 경고] 🚨 통신 끊김 감지! 0.5초간 Uplink 패킷 수신 불가능.\n");
                consecutive_fail_cnt = 80; 
            }
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
                case CMD_OPEN:  
                    hatch_servo->setAngle(pkt.cmd_data ? 180 : 90);
                    fprintf(stderr, "[CMD] Hatch %s\n", pkt.cmd_data ? "OPEN" : "CLOSE");
                    break;

                case CMD_FIRE:  
                    is_launched.store(true); 
                    is_terminal_guidance.store(false);
                    fprintf(stderr, "[CMD] FIRE → 중기유도(0x01) 시작\n");
                    break;

                case CMD_ENDGUIDE:  
                    is_terminal_guidance.store(true);
                    fprintf(stderr, "[CMD] ENDGUIDE → 종말유도(0x02) 전송\n");
                    break;

                case CMD_TARGET:  
                    {
                        atomic_tar_x_meters.store(pkt.target_x);
                        atomic_tar_y_meters.store(pkt.target_y);
                        fprintf(stderr, "[CMD] Target: (%.2f, %.2f) m\n", pkt.target_x, pkt.target_y);
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
            
            tx_x = (current_torpedo_state.position.x == 0 && current_torpedo_state.position.y == 0)
                   ? std::nanf("") : current_torpedo_state.position.x / 1000.0f;
            tx_y = (current_torpedo_state.position.x == 0 && current_torpedo_state.position.y == 0)
                   ? std::nanf("") : current_torpedo_state.position.y / 1000.0f;
            tx_yaw = std::isnan(current_torpedo_state.heading)
                     ? std::nanf("") : current_torpedo_state.heading * (M_PI / 180.0f); 
        }

        uart_gui->sendGuiPacket(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);
        network->sendGuiPacketUDP(tx_x, tx_y, tx_yaw, grid_copy, gui_seq);

        gui_seq++;
        usleep(50000);
    }
}


// ─────────────────────────────────────────
// Thread 5: 정밀 지연 보상 제어 루프 및 어뢰 Downlink (10ms)
// ─────────────────────────────────────────

// 어뢰 데이터로 제어 
void thread_control_loop() {
    static bool is_arrived = false; // 🌟 통신 모드가 바뀌어도 정지 상태가 유지되도록 스레드 내 래치 플래그 선언

    while (system_running.load()) {
        auto loop_start = std::chrono::steady_clock::now();

        if (uart_torpedo) {
            uint8_t tx_flags = 0;
            
            // 1️⃣ [도달 정지 모드 최우선 처리] 이미 목표에 도달했다면 제어 연산을 건너뛰고 정지 패킷만 송신
            if (is_arrived) {
                tx_flags = FLAG_GUIDANCE_ARRIVE; // 0x03 정지 flag 설정
                int16_t steer = 0;               // 조타 정중앙 고정
                
                float raw_target_x = atomic_tar_x_meters.load();
                float raw_target_y = atomic_tar_y_meters.load(); 
                float lidar_x = atomic_torpedo_x_m.load();
                float lidar_y = atomic_torpedo_y_m.load();

                uart_torpedo->sendDownlink(
                    raw_target_x, raw_target_y, 
                    atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
                    atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
                    steer, tx_flags, downlink_seq
                );

                static int arrive_log_counter = 0;
                if (arrive_log_counter++ % 80 == 0) {
                    TorpedoPose log_state;
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        log_state = current_torpedo_state;
                    }
                    fprintf(stdout, "\n==================================================\n");
                    fprintf(stdout, "[Thread 5] 🏁 TARGET ARRIVED -> MOTOR STOP ACTIVE (0x03)\n");
                    fprintf(stdout, "   |- Target WP : X = %.2f m, Y = %.2f m\n", raw_target_x, raw_target_y);
                    fprintf(stdout, "   |- Current(Torpedo) : X = %.2f m, Y = %.2f m\n", log_state.position.x, log_state.position.y);
                    fprintf(stdout, "   |- Current(LiDAR)   : X = %.2f m, Y = %.2f m\n", lidar_x, lidar_y);
                    fprintf(stdout, "==================================================\n");
                    fflush(stdout);
                }
                downlink_seq++;
            }
            else if (is_terminal_guidance.load()) {
                tx_flags = FLAG_GUIDANCE_TERMINAL;
                uart_torpedo->sendDownlink(std::nanf(""), std::nanf(""), std::nanf(""), std::nanf(""), 0, tx_flags, downlink_seq);
                downlink_seq++;
            } 
            else if (is_launched.load()) { 
                tx_flags = FLAG_GUIDANCE_MIDCOURSE;

                // 🔄 순수 어뢰 데이터를 안전하게 획득하기 위해 뮤텍스 락 범위를 상단으로 이동
                TorpedoPose state_copy;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    state_copy = current_torpedo_state; 
                }

                // 🎯 [제어 입력 소스 전환] 제어 및 탐색 인풋을 '순수 어뢰 데이터'로 변경
                float control_x = state_copy.position.x;
                float control_y = state_copy.position.y;

                // 라이다 데이터는 다운링크 피드백 및 로그용으로만 활용
                float lidar_x = atomic_torpedo_x_m.load();
                float lidar_y = atomic_torpedo_y_m.load();

                float raw_target_x = atomic_tar_x_meters.load();
                float raw_target_y = atomic_tar_y_meters.load(); 

                // 🎯 [실시간 Target WP 기준 오차 판정] (순수 어뢰 좌표 기준 20cm 판정)
                if (std::abs(control_x - raw_target_x) <= 0.20f && std::abs(control_y - raw_target_y) <= 0.20f) {
                    is_arrived = true;               // 정지 플래그 온
                    tx_flags = FLAG_GUIDANCE_ARRIVE; // 0x03 변경
                    int16_t steer = 0;               // 모터 정지 시 조타 정중앙

                    uart_torpedo->sendDownlink(
                        raw_target_x, raw_target_y, 
                        atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
                        atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
                        steer, tx_flags, downlink_seq
                    );
                    downlink_seq++;
                    
                    // 정밀 주기 유지를 위해 잔여 시간 계산 후 탈출
                    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - loop_start).count();
                    long sleep_us = 10000 - elapsed;
                    if (sleep_us > 0) usleep(sleep_us);
                    continue; 
                }

                // 1️⃣ [A* Planner 검증 지점] (어뢰 기반 x 입력으로 경로 탐색 연동)
                float target_y_compensated = 0.0f;
                if (astar_planner && atomic_astar_valid.load()) {
                    target_y_compensated = astar_planner->getNextWaypointY(control_x, 0.20f);
                }

                // 1️⃣ [LookAhead 거리 계산 지점 보완] (어뢰 기반 y 오차 계산)
                float y_error = std::abs(target_y_compensated - control_y);

                // 🎯 최소 하한선(0.8m) 보장 로직
                float look_ahead_dist = 0.5f + (y_error * 0.4f); 

                // 상한선 제한
                if (look_ahead_dist > 1.2f) look_ahead_dist = 1.2f;

                // 하한선 제한 (목적지에 다 와도 최소 0.8m 유지하여 오실레이션 방지)
                if (look_ahead_dist < 0.8f) look_ahead_dist = 0.8f;

                float target_x_cmd = raw_target_x;
                
                // 2️⃣ [Geometric Math]
                float dx = target_x_cmd - control_x;  
                float target_yaw = std::atan2(dx, look_ahead_dist); 
                float current_yaw = state_copy.heading;

                // 🌟 [교정] 물리적으로 올바른 헤딩 오차(Yaw Error) 계산 및 Wrap-around 처리 (-PI ~ +PI)
                float yaw_error = target_yaw - current_yaw;
                while (yaw_error > M_PI)  yaw_error -= 2.0f * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;


                // 3️⃣ [Latency Aware Controller 호출]
                int pure_controller_steer = latency_controller->compute(
                    control_y, target_y_compensated, 0.01f, current_yaw, target_yaw
                );

                // 🌟 [교정] 제어기 내부 부호 뒤집힘 방지 가드 (헤딩 오차 방향과 부호 강제 동기화)
                pure_controller_steer = std::abs(pure_controller_steer) * (yaw_error >= 0.0f ? 1 : -1);


                // 4️⃣ [Cross-Track 보정항 - 종착지 감쇄 기능 동적화]
                float x_error = target_x_cmd - control_x; 
                float x_correction = x_error * 8.0f; // 제어 불감대가 사라지므로 게인을 8.0f 정도로 안정적 상향

                // 실시간 목표치(raw_target_y) 기준 종착지 0.5m 전부터 감쇄
                float fade_factor = 1.0f;
                if (control_y > (raw_target_y - 0.5f)) {
                    fade_factor = (raw_target_y - control_y) / 0.5f;
                    if (fade_factor < 0.0f) fade_factor = 0.0f;
                }
                x_correction = x_correction * fade_factor;
                x_correction = std::max(-10.0f, std::min(10.0f, x_correction)); 


                // 5️⃣ [최종 조향 연산 - Float 제어 불감대(Deadzone) 제거]
                float scaled_controller_steer = static_cast<float>(pure_controller_steer);
                
                float yaw_error_deg = yaw_error * (180.0f / M_PI);
                float yaw_error_abs = std::abs(yaw_error_deg);
                if (yaw_error_abs < 15.0f) {
                    float scaling_factor = yaw_error_abs / 15.0f;
                    scaled_controller_steer *= scaling_factor; 
                }

                // 소수점 상태에서 두 제어 인풋을 병합한 뒤 최종 반올림 처리
                float final_steer_float = scaled_controller_steer + x_correction;
                int steer = static_cast<int>(std::round(final_steer_float));

                // 🌟 초기 Y축 60cm(0.6m) 이내 구간 직선주행 강제 로직 (유지)
                if (control_y < 0.6f) {
                    steer = 0;
                    pure_controller_steer = 0;
                    x_correction = 0.0f;
                    target_yaw = 0.0f;
                    yaw_error = 0.0f; // 초기화 가드 추가
                }

                bool invert_hardware_steering = false;
                if (invert_hardware_steering) {
                    steer = -steer;
                }

                // Saturation (전체 기구학적 하드웨어 상한선 제한: ±30도)
                int pre_clamp_steer = steer;
                if (steer > 30) steer = 30;
                if (steer < -30) steer = -30;

                // Rate Limiter (루프당 최대 변위 제한으로 서보 모터 보호 및 급격한 요동 방지)
                static int last_steer = 0;
                constexpr int MAX_STEER_DIFF = 4; 
                int steer_diff = steer - last_steer;
                if (steer_diff > MAX_STEER_DIFF)  steer = last_steer + MAX_STEER_DIFF;
                if (steer_diff < -MAX_STEER_DIFF) steer = last_steer - MAX_STEER_DIFF;
                last_steer = steer;

                // 시스템 기동 초기나 A* 맵이 유효하지 않을 때 직진 고정 가드
                if (state_copy.seq == 0 || !atomic_astar_valid.load()) {
                    steer = 0;
                    target_yaw = 0.0f;
                }

                // 하드웨어로 최종 패킷 송신
                uart_torpedo->sendDownlink(
                    raw_target_x, raw_target_y, 
                    atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
                    atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
                    static_cast<int16_t>(steer), tx_flags, downlink_seq
                );

                // 대시보드 출력
                static int tx_log_counter = 0;
                if (tx_log_counter++ % 100 == 0) {
                    fprintf(stdout, "\n==================================================\n");
                    fprintf(stdout, "[Zynq <-> Torpedo 100-Loop Sync Dashboard]\n");
                    fprintf(stdout, "  [RX UPLINK]  SEQ: %u | Flags: 0x%02X\n", state_copy.seq, state_copy.flags);
                    fprintf(stdout, "   |- Raw Zynq RX Pos : X = %.2f m, Y = %.2f m\n", state_copy.position.x, state_copy.position.y);
                    fprintf(stdout, "   |- Control Pos(Torpedo): X = %.2f m, Y = %.2f m\n", control_x, control_y); // 🛠️ 로그 명칭 명확화
                    fprintf(stdout, "   |- Torpedo Yaw     : %.2f deg (Internal: %.4f rad)\n", state_copy.heading * (180.0f / M_PI), state_copy.heading);
                    fprintf(stdout, "  -------------------------------------------------\n");
                    
                    if (control_y < 0.6f) {
                        fprintf(stdout, "  [🔧 DEEP DIAGNOSTIC BREAKDOWN] 🚀 [STRAIGHT RUN MODE ACTIVE (< 0.6m)]\n");
                    } else {
                        fprintf(stdout, "  [🔧 DEEP DIAGNOSTIC BREAKDOWN]\n");
                    }
                    fprintf(stdout, "   1) A* Planner  -> Comp Y: %.4f m (Target Y: %.2f)\n", target_y_compensated, raw_target_y);
                    fprintf(stdout, "   2) Geometry    -> Target Yaw: %.2f deg | True Error: %.2f deg\n", target_yaw * (180.0f / M_PI), yaw_error * (180.0f / M_PI));
                    fprintf(stdout, "   3) LatencyCtrl -> Pure Controller Output: %d (Signed/Scaled)\n", pure_controller_steer);
                    fprintf(stdout, "   4) Thread5 Math-> X Pos: %.3f m | x_error: %.3f | x_correction: %.2f\n", control_x, x_error, x_correction);
                    fprintf(stdout, "   5) Total Sum   -> Pre-Clamp: %d -> Post-Clamp: %d\n", pre_clamp_steer, steer);
                    fprintf(stdout, "  -------------------------------------------------\n");
                    
                    fprintf(stdout, "  [TX DOWNLINK] SEQ: %u | Flags: 0x%02X\n", downlink_seq, tx_flags);
                    fprintf(stdout, "   |- Target WP  : X = %.2f m, Y = %.2f m (Comp Y: %.2f m)\n", raw_target_x, raw_target_y, target_y_compensated);
                    fprintf(stdout, "   |- LiDAR Sync : X = %.2f m, Y = %.2f m\n", lidar_x, lidar_y);
                    fprintf(stdout, "   |- Target Yaw : %.2f deg (LookAhead: %.2fm)\n", target_yaw * (180.0f / M_PI), look_ahead_dist);
                    fprintf(stdout, "   |- STEER CMD  : %d deg\n", static_cast<int16_t>(steer));
                    fprintf(stdout, "==================================================\n");
                }
                
                downlink_seq++;
            }
            else {
                static int ready_log_counter = 0;
                if (ready_log_counter++ % 100 == 0) {
                    fprintf(stdout, "[Thread 5] Control Loop Alive - Waiting for GUI CMD_FIRE\n");
                    fflush(stdout);
                }
            }
        }

        // 10ms (10000us) 정밀 제어 주기 동기화
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        long sleep_us = 10000 - elapsed;
        if (sleep_us > 0) usleep(sleep_us);
    }
}


// 순수 라이다만 이용 
// void thread_control_loop() {
//     static bool is_arrived = false; // 🌟 통신 모드가 바뀌어도 정지 상태가 유지되도록 스레드 내 래치 플래그 선언

//     while (system_running.load()) {
//         auto loop_start = std::chrono::steady_clock::now();

//         if (uart_torpedo) {
//             uint8_t tx_flags = 0;
            
//             // 1️⃣ [도달 정지 모드 최우선 처리] 이미 목표에 도달했다면 제어 연산을 건너뛰고 정지 패킷만 송신
//             if (is_arrived) {
//                 tx_flags = FLAG_GUIDANCE_ARRIVE; // 0x03 정지 flag 설정
//                 int16_t steer = 0;               // 조타 정중앙 고정
                
//                 float raw_target_x = atomic_tar_x_meters.load();
//                 float raw_target_y = atomic_tar_y_meters.load(); 
//                 float lidar_x = atomic_torpedo_x_m.load();
//                 float lidar_y = atomic_torpedo_y_m.load();

//                 uart_torpedo->sendDownlink(
//                     raw_target_x, raw_target_y, 
//                     atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
//                     atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
//                     steer, tx_flags, downlink_seq
//                 );

//                 static int arrive_log_counter = 0;
//                 if (arrive_log_counter++ % 80 == 0) {
//                     fprintf(stdout, "\n==================================================\n");
//                     fprintf(stdout, "[Thread 5] 🏁 TARGET ARRIVED -> MOTOR STOP ACTIVE (0x03)\n");
//                     fprintf(stdout, "   |- Target WP : X = %.2f m, Y = %.2f m\n", raw_target_x, raw_target_y);
//                     fprintf(stdout, "   |- Current   : X = %.2f m, Y = %.2f m\n", 
//                             (atomic_lidar_valid.load() ? lidar_x : current_torpedo_state.position.x),
//                             (atomic_lidar_valid.load() ? lidar_y : current_torpedo_state.position.y));
//                     fprintf(stdout, "==================================================\n");
//                     fflush(stdout);
//                 }
//                 downlink_seq++;
//             }
//             else if (is_terminal_guidance.load()) {
//                 tx_flags = FLAG_GUIDANCE_TERMINAL;
//                 uart_torpedo->sendDownlink(std::nanf(""), std::nanf(""), std::nanf(""), std::nanf(""), 0, tx_flags, downlink_seq);
//                 downlink_seq++;
//             } 
//             else if (is_launched.load()) { 
//                 tx_flags = FLAG_GUIDANCE_MIDCOURSE;

//                 float lidar_x = atomic_torpedo_x_m.load();
//                 float lidar_y = atomic_torpedo_y_m.load();

//                 TorpedoPose state_copy;
//                 {
//                     std::lock_guard<std::mutex> lock(mtx);
//                     state_copy = current_torpedo_state; 
//                 }

//                 // 🎯 제어 인풋은 '무조건' LiDAR 값으로 고정
//                 float control_x = lidar_x;
//                 float control_y = lidar_y;

//                 float raw_target_x = atomic_tar_x_meters.load();
//                 float raw_target_y = atomic_tar_y_meters.load(); 

//                 // 🎯 [실시간 Target WP 기준 오차 판정] (LiDAR 기준 판정)
//                 if (std::abs(control_x - raw_target_x) <= 0.20f && std::abs(control_y - raw_target_y) <= 0.20f) {
//                     is_arrived = true;               // 정지 플래그 온
//                     tx_flags = FLAG_GUIDANCE_ARRIVE; // 0x03 변경
//                     int16_t steer = 0;               // 모터 정지 시 조타 정중앙

//                     uart_torpedo->sendDownlink(
//                         raw_target_x, raw_target_y, 
//                         atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
//                         atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
//                         steer, tx_flags, downlink_seq
//                     );
//                     downlink_seq++;
                    
//                     // 정밀 주기 유지를 위해 잔여 시간 계산 후 탈출
//                     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
//                         std::chrono::steady_clock::now() - loop_start).count();
//                     long sleep_us = 10000 - elapsed;
//                     if (sleep_us > 0) usleep(sleep_us);
//                     continue; 
//                 }

//                 // 1️⃣ [A* Planner 검증 지점] (LiDAR 기반 x 입력)
//                 float target_y_compensated = 0.0f;
//                 if (astar_planner && atomic_astar_valid.load()) {
//                     target_y_compensated = astar_planner->getNextWaypointY(control_x, 0.20f);
//                 }

//                 // 1️⃣ [LookAhead 거리 계산 지점 보완] (LiDAR 기반 y 오차 계산)
//                 float y_error = std::abs(target_y_compensated - control_y);

//                 // 🎯 최소 하한선(0.8m) 보장 로직
//                 float look_ahead_dist = 0.5f + (y_error * 0.4f); 

//                 // 상한선 제한
//                 if (look_ahead_dist > 1.2f) look_ahead_dist = 1.2f;

//                 // 하한선 제한 (목적지에 다 와도 최소 0.8m 유지하여 오실레이션 방지)
//                 if (look_ahead_dist < 0.8f) look_ahead_dist = 0.8f;

//                 float target_x_cmd = raw_target_x;
                
//                 // 2️⃣ [Geometric Math]
//                 float dx = target_x_cmd - control_x;  
//                 float target_yaw = std::atan2(dx, look_ahead_dist); 
//                 float current_yaw = state_copy.heading;

//                 // 🌟 [교정] 물리적으로 올바른 헤딩 오차(Yaw Error) 계산 및 Wrap-around 처리 (-PI ~ +PI)
//                 float yaw_error = target_yaw - current_yaw;
//                 while (yaw_error > M_PI)  yaw_error -= 2.0f * M_PI;
//                 while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;


//                 // 3️⃣ [Latency Aware Controller 호출]
//                 int pure_controller_steer = latency_controller->compute(
//                     control_y, target_y_compensated, 0.01f, current_yaw, target_yaw
//                 );

//                 // 🌟 [교정] 제어기 내부 부호 뒤집힘 방지 가드 (헤딩 오차 방향과 부호 강제 동기화)
//                 pure_controller_steer = std::abs(pure_controller_steer) * (yaw_error >= 0.0f ? 1 : -1);


//                 // 4️⃣ [Cross-Track 보정항 - 종착지 감쇄 기능 동적화]
//                 float x_error = target_x_cmd - control_x; 
//                 float x_correction = x_error * 8.0f; // 제어 불감대가 사라지므로 게인을 8.0f 정도로 안정적 상향

//                 // 실시간 목표치(raw_target_y) 기준 종착지 0.5m 전부터 감쇄
//                 float fade_factor = 1.0f;
//                 if (control_y > (raw_target_y - 0.5f)) {
//                     fade_factor = (raw_target_y - control_y) / 0.5f;
//                     if (fade_factor < 0.0f) fade_factor = 0.0f;
//                 }
//                 x_correction = x_correction * fade_factor;
//                 x_correction = std::max(-10.0f, std::min(10.0f, x_correction)); 


//                 // 5️⃣ [최종 조향 연산 - Float 제어 불감대(Deadzone) 제거]
//                 // 🌟 중복 선언 제거 및 float 공간에서 미세 타각 소수점 유지 통합
//                 float scaled_controller_steer = static_cast<float>(pure_controller_steer);
                
//                 float yaw_error_deg = yaw_error * (180.0f / M_PI);
//                 float yaw_error_abs = std::abs(yaw_error_deg);
//                 if (yaw_error_abs < 15.0f) {
//                     float scaling_factor = yaw_error_abs / 15.0f;
//                     scaled_controller_steer *= scaling_factor; 
//                 }

//                 // 소수점 상태에서 두 제어 인풋을 병합한 뒤 최종 반올림 처리
//                 float final_steer_float = scaled_controller_steer + x_correction;
//                 int steer = static_cast<int>(std::round(final_steer_float));

//                 // 🌟 초기 Y축 60cm(0.6m) 이내 구간 직선주행 강제 로직 (유지)
//                 if (control_y < 0.6f) {
//                     steer = 0;
//                     pure_controller_steer = 0;
//                     x_correction = 0.0f;
//                     target_yaw = 0.0f;
//                     yaw_error = 0.0f; // 초기화 가드 추가
//                 }

//                 bool invert_hardware_steering = false;
//                 if (invert_hardware_steering) {
//                     steer = -steer;
//                 }

//                 // Saturation (전체 기구학적 하드웨어 상한선 제한: ±30도)
//                 int pre_clamp_steer = steer;
//                 if (steer > 30) steer = 30;
//                 if (steer < -30) steer = -30;

//                 // Rate Limiter (루프당 최대 변위 제한으로 서보 모터 보호 및 급격한 요동 방지)
//                 static int last_steer = 0;
//                 constexpr int MAX_STEER_DIFF = 4; 
//                 int steer_diff = steer - last_steer;
//                 if (steer_diff > MAX_STEER_DIFF)  steer = last_steer + MAX_STEER_DIFF;
//                 if (steer_diff < -MAX_STEER_DIFF) steer = last_steer - MAX_STEER_DIFF;
//                 last_steer = steer;

//                 // 시스템 기동 초기나 A* 맵이 유효하지 않을 때 직진 고정 가드
//                 if (state_copy.seq == 0 || !atomic_astar_valid.load()) {
//                     steer = 0;
//                     target_yaw = 0.0f;
//                 }

//                 // 하드웨어로 최종 패킷 송신
//                 uart_torpedo->sendDownlink(
//                     raw_target_x, raw_target_y, 
//                     atomic_lidar_valid.load() ? lidar_x : std::nanf(""), 
//                     atomic_lidar_valid.load() ? lidar_y : std::nanf(""), 
//                     static_cast<int16_t>(steer), tx_flags, downlink_seq
//                 );

//                 // 대시보드 출력
//                 static int tx_log_counter = 0;
//                 if (tx_log_counter++ % 100 == 0) {
//                     fprintf(stdout, "\n==================================================\n");
//                     fprintf(stdout, "[Zynq <-> Torpedo 100-Loop Sync Dashboard]\n");
//                     fprintf(stdout, "  [RX UPLINK]  SEQ: %u | Flags: 0x%02X\n", state_copy.seq, state_copy.flags);
//                     fprintf(stdout, "   |- Raw Zynq RX Pos : X = %.2f m, Y = %.2f m\n", state_copy.position.x, state_copy.position.y);
//                     fprintf(stdout, "   |- Control Pos(LiDAR): X = %.2f m, Y = %.2f m\n", control_x, control_y);
//                     fprintf(stdout, "   |- Torpedo Yaw     : %.2f deg (Internal: %.4f rad)\n", state_copy.heading * (180.0f / M_PI), state_copy.heading);
//                     fprintf(stdout, "  -------------------------------------------------\n");
                    
//                     if (control_y < 0.6f) {
//                         fprintf(stdout, "  [🔧 DEEP DIAGNOSTIC BREAKDOWN] 🚀 [STRAIGHT RUN MODE ACTIVE (< 0.6m)]\n");
//                     } else {
//                         fprintf(stdout, "  [🔧 DEEP DIAGNOSTIC BREAKDOWN]\n");
//                     }
//                     fprintf(stdout, "   1) A* Planner  -> Comp Y: %.4f m (Target Y: %.2f)\n", target_y_compensated, raw_target_y);
//                     fprintf(stdout, "   2) Geometry    -> Target Yaw: %.2f deg | True Error: %.2f deg\n", target_yaw * (180.0f / M_PI), yaw_error * (180.0f / M_PI));
//                     fprintf(stdout, "   3) LatencyCtrl -> Pure Controller Output: %d (Signed/Scaled)\n", pure_controller_steer);
//                     fprintf(stdout, "   4) Thread5 Math-> X Pos: %.3f m | x_error: %.3f | x_correction: %.2f\n", control_x, x_error, x_correction);
//                     fprintf(stdout, "   5) Total Sum   -> Pre-Clamp: %d -> Post-Clamp: %d\n", pre_clamp_steer, steer);
//                     fprintf(stdout, "  -------------------------------------------------\n");
                    
//                     fprintf(stdout, "  [TX DOWNLINK] SEQ: %u | Flags: 0x%02X\n", downlink_seq, tx_flags);
//                     fprintf(stdout, "   |- Target WP  : X = %.2f m, Y = %.2f m (Comp Y: %.2f m)\n", raw_target_x, raw_target_y, target_y_compensated);
//                     fprintf(stdout, "   |- LiDAR Sync : X = %.2f m, Y = %.2f m\n", lidar_x, lidar_y);
//                     fprintf(stdout, "   |- Target Yaw : %.2f deg (LookAhead: %.2fm)\n", target_yaw * (180.0f / M_PI), look_ahead_dist);
//                     fprintf(stdout, "   |- STEER CMD  : %d deg\n", static_cast<int16_t>(steer));
//                     fprintf(stdout, "==================================================\n");
//                 }
                
//                 downlink_seq++;
//             }
//             else {
//                 static int ready_log_counter = 0;
//                 if (ready_log_counter++ % 100 == 0) {
//                     fprintf(stdout, "[Thread 5] Control Loop Alive - Waiting for GUI CMD_FIRE\n");
//                     fflush(stdout);
//                 }
//             }
//         }

//         // 10ms (10000us) 정밀 제어 주기 동기화
//         auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
//             std::chrono::steady_clock::now() - loop_start).count();
//         long sleep_us = 10000 - elapsed;
//         if (sleep_us > 0) usleep(sleep_us);
//     }
// }



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
                atomic_tar_x_meters.store(pkt.target_x);
                atomic_tar_y_meters.store(pkt.target_y);
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

// 🎯 버그 수정: Linux 시그널 수신 시 안전 탈출을 유도하는 핸들러 정의
void signalHandler(int signum) {
    fprintf(stderr, "\n[SIGNAL] Shutdown signal (%d) received. Cleaning up threads...\n", signum);
    system_running.store(false);
}

// ── 시스템 메인 진입점 ────────────────────────────────────
int main() {
    // 🎯 버그 수정: 메인 함수 시작 시그널 핸들러 등록
    std::signal(SIGINT, signalHandler);  
    std::signal(SIGTERM, signalHandler); 

    system_start_time = std::chrono::steady_clock::now();
    const int LATENCY_MS = 80;
    
    fprintf(stderr, "=== Control Station Booting ===\n");
    fflush(stderr);
     
    // 1. Lidar 초기화
    fprintf(stderr, "[1/7] Initializing Lidar...\n");
    try {
        lidar = std::make_unique<LidarNode>("/dev/ttyPS1", 460800);
        fprintf(stderr, "[1/7] Lidar OK\n");
        std::cout << "\n👉 [Lidar] 주변을 비운 뒤, 배경 학습을 시작하려면 [Enter] 키를 누르세요..." << std::endl;
        std::string dummy;
        std::getline(std::cin, dummy);
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

// accept 블로킹 상태인 t6은 독립(detach)시켜 프로세스 종료 시 자동 수거되도록 함
    t6.detach(); 

    // 나머지 스레드들은 루프를 빠져나올 때까지 안전하게 join
    t1.join(); t2.join(); t3.join(); t4.join(); 
    t5.join(); t7.join(); t8.join();

    return 0;

}