#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <cmath>
#include <atomic>
#include <mutex>
#include "DataTypes.h"

/**
 * @brief A* 플래너 내부에서 사용하는 독립적 웨이포인트 구조체
 * @note 미터(m) 단위를 표준으로 사용합니다.
 */

class AStarPlanner {
private:
    std::vector<Waypoint> path; // 변수명을 path로 통일
    mutable std::mutex path_mtx; // const 함수 내에서 lock을 걸기 위해 mutable 추가
    std::atomic<bool> is_path_valid{false};
    
    float latency_sec;    // 초 단위 제어 지연 시간 (예: 80ms -> 0.08f)
    float base_velocity;  // 기본 구동 속도 (m/s)

public:
    AStarPlanner(int latency_ms, float default_vel = 0.15f) {
        latency_sec = static_cast<float>(latency_ms) / 1000.0f;
        base_velocity = default_vel;
    }

    // 200ms 주기 알고리즘 스레드가 호출하는 경로 갱신 함수 (미터 단위 입력)
    void computePath(Waypoint start, Waypoint goal) {
        std::lock_guard<std::mutex> lock(path_mtx);
        path.clear();
        
        // 최소 직선 경로 생성 뼈대 로직 (추후 실제 알고리즘 탐색부 확장 가능)
        int steps = 20;
        for (int i = 0; i <= steps; ++i) {
            float t = static_cast<float>(i) / steps;
            path.push_back({start.x + t * (goal.x - start.x), start.y + t * (goal.y - start.y)});
        }
        is_path_valid.store(true);
    }

    bool isReady() const { return is_path_valid.load(); }

        float getNextWaypointY(float current_x_meters, float current_vel_ms = 0.0f) const {
        if (!is_path_valid.load()) return 0.0f;

        std::lock_guard<std::mutex> lock(path_mtx);
        if (path.size() < 2) return 0.0f;

        float vel = (current_vel_ms > 0.001f) ? current_vel_ms : base_velocity;
        float look_ahead_dist = (vel * latency_sec) + 0.12f;

        // ── X 이동 방향 자동 감지 ──
        float dx = path.back().x - path.front().x;
        float dir = (dx >= 0.0f) ? 1.0f : -1.0f;
        float target_x = current_x_meters + dir * look_ahead_dist;

        // ── 경계 처리 ──
        float x_min = std::min(path.front().x, path.back().x);
        float x_max = std::max(path.front().x, path.back().x);
        target_x = std::max(x_min, std::min(x_max, target_x));

        // ── 선형 보간 ──
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            float xa = path[i].x, xb = path[i+1].x;
            float lo = std::min(xa, xb), hi = std::max(xa, xb);
            if (target_x >= lo && target_x <= hi) {
                float segment_len = std::abs(xb - xa);
                if (segment_len < 1e-5f) return path[i].y;
                        
                // 🌟 절대 거리 기준 비율 계산 (부호 버그 원천 차단)
                float ratio = std::abs(target_x - xa) / segment_len;
                return path[i].y + ratio * (path[i+1].y - path[i].y);
                }
        }
        return path.back().y;
    }
};

#endif // ASTAR_PLANNER_H