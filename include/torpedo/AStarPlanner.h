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

    // 10ms 주기 제어 루프가 호출하는 지연 시간 선행 보정 경로 보간 로직
    float getNextWaypointY(float current_x_meters, float current_vel_ms = 0.0f) const {
        if (!is_path_valid.load()) return 0.0f;

        float vel = (current_vel_ms > 0.001f) ? current_vel_ms : base_velocity;
        
        // 핵심 물리 수식: 지연 시간(Latency) 동안 어뢰가 전진할 물리적 선행 보정 거리 계산
        float look_ahead_dist = (vel * latency_sec) + 0.12f; 
        float target_x = current_x_meters + look_ahead_dist;

        std::lock_guard<std::mutex> lock(path_mtx);
        if (path.size() < 2) return 0.0f;

        // 타겟 X에 인접한 웨이포인트 구간 선형 보간(Linear Interpolation) 처리
        if (target_x <= path.front().x) return path.front().y;
        if (target_x >= path.back().x) return path.back().y;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (target_x >= path[i].x && target_x <= path[i+1].x) {
                float ratio = (target_x - path[i].x) / (path[i+1].x - path[i].x);
                return path[i].y + ratio * (path[i+1].y - path[i].y);
            }
        }
        return path.back().y;
    }
};

#endif // ASTAR_PLANNER_H