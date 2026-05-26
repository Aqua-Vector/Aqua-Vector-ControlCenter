#ifndef DATATYPES_H
#define DATATYPES_H

#include <cmath>
#include <cstdint> // uint32_t, uint8_t 사용을 위해 추가

/**
 * @brief 2D 공간상의 좌표를 나타내는 표준 구조체
 * @note 프로젝트 표준 단위: 미터(m)
 */
struct Point2D {
    float x; // 위치 X [m]
    float y; // 위치 Y [m]
};

/**
 * @brief A* 알고리즘 및 경로 계획용 웨이포인트 구조체
 * @note 프로젝트 표준 단위: 미터(m)
 */
struct Waypoint {
    float x; // 웨이포인트 X [m]
    float y; // 웨이포인트 Y [m]
};

/**
 * @brief 라이다 격자 지형도(Grid Map)의 단일 셀 상태
 * @note 프로젝트 표준 단위: 미터(m)
 */
struct PointGrid {
    float x;           // 격자 중심 X [m]
    float y;           // 격자 중심 Y [m]
    bool is_obstacle;  // 장애물 여부 (true: 장애물, false: 빈 공간)
};

/**
 * @brief 어뢰의 동적 상태(Pose) 정보를 담는 구조체
 */
struct TorpedoPose {
    Point2D position;    // 현재 위치 (x, y) [m]
    float velocity;      // 현재 선속도 [m/s]
    float heading;       // 현재 헤딩 각도 [degree] -> 단, 제어기 내부 연산 시 필요에 따라 라디안 변환 사용
    float angular_vel;   // 각속도 [rad/s]
    float latency_comp;  // 지연 보상값 또는 제어 입력 가중치
    uint32_t seq;        // 수신 패킷 시퀀스 번호
    uint8_t flags;       // 수신 상태 플래그
};

#endif // DATATYPES_H