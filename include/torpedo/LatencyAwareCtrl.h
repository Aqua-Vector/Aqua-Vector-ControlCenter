#ifndef LATENCY_AWARE_CTRL_H
#define LATENCY_AWARE_CTRL_H

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class LatencyAwareCtrl {
private:
    int   last_steer_packet = 0; 
    float steer_filtered = 0.0f;
    float prev_y = 0.0f;
    float dy_filt = 0.0f;

    static constexpr float NOISE_DECAY   = 0.25f;  
    static constexpr int   DEAD_ZONE     = 2;      
    static constexpr float BASE_VELOCITY = 0.15f;  
    static constexpr float LATENCY_SEC   = 0.08f;  

    // ── [🚨 좌표계 제어 핵심 플래그] ──
    // 실험 환경에 따라 P항 오차 부호가 반대여야 로그의 -45 현상이 설명됨.
    // 만약 수정 후에도 반대로 돌면 이 값을 false로 바꾸면 돼.
    static constexpr bool INVERT_Y_CONTROL = true; 

public:
    LatencyAwareCtrl() { reset(); }

    void reset() {
        last_steer_packet = 0;
        steer_filtered = 0.0f;
        prev_y = 0.0f;
        dy_filt = 0.0f;
    }

int compute(float current_y, float target_y, float dt,
            float current_yaw_rad, float target_yaw_rad) {

    if (prev_y == 0.0f) prev_y = current_y;

    // 1. Y 변화율 필터링 속도 업 (0.92 -> 0.85로 현재 변화 반영율을 높임)
    float dy_dt = (dt > 0.0f) ? (current_y - prev_y) / dt : 0.0f;
    dy_filt = 0.85f * dy_filt + 0.15f * dy_dt; 
    prev_y = current_y;

    float noise_est = std::abs(dy_filt);
    float noise_scale = std::exp(-noise_est / (NOISE_DECAY * 1.5f));

    float v_y = BASE_VELOCITY * std::sin(current_yaw_rad);
    float estimated_y = current_y + (v_y * LATENCY_SEC * 0.7f * noise_scale);

    float error_y = target_y - estimated_y;

    // 2. 기본 P, D 게인 연산
    float p_term = error_y * 65.0f;
    float d_term = -dy_filt * 2.5f;

    // 3. Yaw항 제어 및 Saturation(포화 제한) 추가로 과도한 회전 방지
    float yaw_error = target_yaw_rad - current_yaw_rad;
    while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
    float yaw_term = yaw_error * 5.0f;
    yaw_term = std::max(-8.0f, std::min(8.0f, yaw_term)); // 지나친 꺾임 방지

    float steer_raw = p_term + d_term + yaw_term;
    steer_raw = std::max(-45.0f, std::min(45.0f, steer_raw));

    // 4. 🔥 핵심: 오차 크기에 따른 동적 LPF Alpha 적용
    float error_abs = std::abs(error_y);
    float alpha;
    if (error_abs < 0.1f) {
        alpha = 0.3f;   // 오차가 작아지면 필터를 느슨하게 해서 제어값을 빠르게 0 근처로 복귀
    } else if (error_abs < 0.3f) {
        alpha = 0.2f;
    } else {
        alpha = 0.12f;  // 오차가 커도 기존 0.05보다는 훨씬 빠르게 반응하도록 설정
    }
    steer_filtered = (1.0f - alpha) * steer_filtered + alpha * steer_raw;

    // 5. 데드존 및 Slew Rate 제어 (기존 로직 유지)
    int steer_int = static_cast<int>(std::round(steer_filtered));
    if (std::abs(steer_int - last_steer_packet) < DEAD_ZONE) {
        return last_steer_packet;
    }

    float slew_f = 2.5f + (std::min(0.6f, std::max(0.0f, error_abs)) * 12.0f);
    int dynamic_slew = static_cast<int>(slew_f);
    int diff = steer_int - last_steer_packet;
    if (std::abs(diff) > dynamic_slew) {
        steer_int = last_steer_packet + (diff > 0 ? dynamic_slew : -dynamic_slew);
    }

    last_steer_packet = std::max(-45, std::min(45, steer_int));
    return last_steer_packet;
}
};

#endif // LATENCY_AWARE_CTRL_H