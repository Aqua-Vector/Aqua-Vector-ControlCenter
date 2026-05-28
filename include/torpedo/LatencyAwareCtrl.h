#ifndef LATENCY_AWARE_CTRL_H
#define LATENCY_AWARE_CTRL_H

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class LatencyAwareCtrl {
private:
    int   last_steer_packet = 0; // Zynq 송신 규격: -45 ~ +45
    float steer_filtered = 0.0f;
    float prev_y = 0.0f;
    float dy_filt = 0.0f;

    // ── 시스템 매개변수 설정 (초속 15cm 등속도 수조 환경 최적화) ──
    static constexpr float NOISE_DECAY   = 0.25f;  // 노이즈 감쇄 스케일
    static constexpr int   DEAD_ZONE     = 2;      // 미세 진동 차단 데드존 (2도)
    static constexpr float BASE_VELOCITY = 0.15f;  // 어뢰 구동 속도 (0.15 m/s)
    static constexpr float LATENCY_SEC   = 0.08f;  // 시스템 총 지연 시간 (80ms 가정)

public:
    LatencyAwareCtrl() { reset(); }

    void reset() {
        last_steer_packet = 0;
        steer_filtered = 0.0f;
        prev_y = 0.0f;
        dy_filt = 0.0f;
    }

    /**
     * @brief 위치(Y)와 헤딩(Yaw) 센서를 융합한 지연 보상 제어 루프
     * @param current_y       현재 라이다/Uplink 측정 Y 좌표 (m)
     * @param target_y        A* 플래너가 계산한 선행 보정 목표 Y 좌표 (m)
     * @param dt              제어 주기 (10ms = 0.01f)
     * @param current_yaw_rad Uplink로 수신한 어뢰 현재 헤딩 각도 (단위: radian)
     * @param target_yaw_rad  현재 위치에서 목표점을 바라보는 이상적인 헤딩 각도 (단위: radian)
     */
    int compute(float current_y, float target_y, float dt, float current_yaw_rad, float target_yaw_rad) {
        if (prev_y == 0.0f) prev_y = current_y;

        // ── [치트키 1] 관성 추정 필터 (Y축 실시간 변화율 계산) ──
        float dy_dt = (dt > 0.0f) ? (current_y - prev_y) / dt : 0.0f;
        dy_filt = 0.92f * dy_filt + 0.08f * dy_dt; 
        prev_y = current_y;

        // ── [치트키 2] 지수 감쇄형 노이즈 필터 (noise_scale) ──
        // Y축 출렁임(노이즈)이 커지면 데드 레커닝 예측 신뢰도를 낮추어 오버슈트를 방지합니다.
        float noise_est = std::abs(dy_filt);
        float noise_scale = std::exp(-noise_est / (NOISE_DECAY * 1.5f));

        // ── [치트키 3] Yaw 데이터 융합형 현재 위치 미래 예측 (State Projection) ──
        // 단순히 과거 미분치에 의존하지 않고, 물리 모델(v * sin(yaw))을 결합하여 지연 시간 동안 전진했을 실제 현재 위치를 추정합니다.
        float v_y = BASE_VELOCITY * std::sin(current_yaw_rad);
        float estimated_y = current_y + (v_y * LATENCY_SEC * 0.7f * noise_scale);

        // ── 제어 입력 산출 (위치 오차 + 헤딩 오차 복합 제어) ──
        float error_y = target_y - estimated_y;
        
        // 1. 위치 오차 기반 P항 (Zynq 스케일 65.0f 유지)
        float p_term = error_y * 65.0f; 

        // 2. 미세 진동 감쇄를 위한 D항 (Damping)
        float d_term = -dy_filt * 2.5f;

        // 3. 헤딩 오차 보정항 (Cross-Track Error를 빠르게 정렬하는 핵심)
        float yaw_error = target_yaw_rad - current_yaw_rad;
        while (yaw_error >  M_PI) yaw_error -= 2.0f * M_PI; // -π ~ +π 정규화
        while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
        
        float k_yaw = 5.0f; // 수조 실험을 통해 최적화할 Yaw 게인
        float yaw_term = yaw_error * k_yaw;
        yaw_term = std::max(-10.0f, std::min(10.0f, yaw_term));
        
        // ── 조향각 최종 계산 및 물리 리밋 적용 (-45 ~ +45 범위 매핑) ──
        float steer_raw = p_term + d_term + yaw_term;
        steer_raw = std::max(-45.0f, std::min(45.0f, steer_raw));

        // ── Adaptive LPF (안정 구간에서 서보 지터 제거) ──
        float error_abs = std::abs(error_y);
        float alpha = 0.05f + (std::min(0.5f, std::max(0.0f, error_abs - 0.1f)) * 1.0f);
        steer_filtered = (1.0f - alpha) * steer_filtered + alpha * steer_raw;

        // ── 데드존 필터링 (2도 이내의 미세 떨림 명령 방출 차단) ──
        int steer_int = static_cast<int>(std::round(steer_filtered));
        if (std::abs(steer_int - last_steer_packet) < DEAD_ZONE) {
            return last_steer_packet;
        }

        // ── [치트키 4] 다이내믹 슬루 레이트 (Dynamic Slew Rate) ──
        // 오차가 클 때는 빠르게(최대 10도/step) 조향을 열고, 경로에 안착하면 촘촘하게(최소 2.5도/step) 조여 지터를 물리적으로 박살냅니다.
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