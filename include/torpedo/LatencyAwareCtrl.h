#ifndef LATENCY_AWARE_CTRL_H
#define LATENCY_AWARE_CTRL_H

#include <cmath>
#include <algorithm>

class LatencyAwareCtrl {
private:
    int   last_steer_packet = 0; // 패킷 전송용 범위: -45 ~ +45
    float steer_filtered = 0.0f;
    
    // 서보 구동 하드웨어 물리 보호용 슬루레이트 한계 상수
    const float MAX_SLEW_RATE = 4.0f; 
    const int   DEAD_ZONE = 1;

public:
    LatencyAwareCtrl() { reset(); }

    void reset() {
        last_steer_packet = 0;
        steer_filtered = 0.0f;
    }

    // 오차 계산 및 비선형 댐핑을 가미한 조향 제어 로직 호출
    // 반환값: 패킷 전송 규격에 부합하는 정수 데이터 (-45 ~ +45)
    int compute(float current_y, float target_y, float dt) {
        float error = target_y - current_y;

        // 지연 환경 특화 매개변수 설정 (P게인 및 적응형 LPF 가중치 산출)
        float kp = 65.0f;
        float p_term = error * kp;
        
        // 미세 진동 감쇠를 위한 미분 결합 제어 계산
        float d_term = -((current_y - steer_filtered) / dt) * 2.5f;

        float steer_raw = p_term + d_term;
        // 소프트웨어 안전 각도 소프트 리밋 적용
        steer_raw = std::max(-45.0f, std::min(45.0f, steer_raw));

        // 지연 보충 적응형 저주파 필터 (Adaptive LPF)
        float error_abs = std::abs(error);
        float alpha = 0.08f + std::min(0.42f, std::max(0.0f, error_abs - 0.05f));
        steer_filtered = (1.0f - alpha) * steer_filtered + alpha * steer_raw;

        int steer_int = static_cast<int>(std::round(steer_filtered));
        
        // 불필요한 미세 진동 출력 차단을 위한 데드존 필터링
        if (std::abs(steer_int - last_steer_packet) < DEAD_ZONE) {
            return last_steer_packet;
        }

        // 급격한 출력 방지를 위한 Slew Rate 제한 수행
        float diff = steer_int - last_steer_packet;
        if (std::abs(diff) > MAX_SLEW_RATE) {
            steer_int = last_steer_packet + (diff > 0 ? MAX_SLEW_RATE : -MAX_SLEW_RATE);
        }

        last_steer_packet = steer_int;
        return last_steer_packet;
    }
};

#endif // LATENCY_AWARE_CTRL_H