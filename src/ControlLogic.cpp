#include "ControlLogic.h"
#include <cmath>

ControlLogic::ControlLogic() {}

void ControlLogic::setStaticMap(const std::vector<Point2D>& map) {
    static_map = map;
}

int ControlLogic::calculateSteering(const TorpedoPose& current, const Point2D& target) {
    // 1. 타겟을 향한 절대 각도 계산
    float dx = target.x - current.position.x;
    float dy = target.y - current.position.y;
    float target_angle_rad = std::atan2(dy, dx);
    float target_angle_deg = target_angle_rad * (180.0f / M_PI);

    // [향후 추가] A* 경로점 계산 후, target_angle_deg를 A*의 Next Step 각도로 대체
    // 여기서는 기본 P-제어(비례제어) 조향 뼈대 제공
    
    // 2. 현재 내 Heading 각도와의 오차 도출
    float error_angle = target_angle_deg - current.heading;

    // 3. 서보모터 조향각 변환 (90도가 직진)
    float kp = 1.0f; // 비례 게인
    int steering = 90 + static_cast<int>(error_angle * kp);

    // 하드웨어 제한 (Soft Limit)
    if(steering > 150) steering = 150;
    if(steering < 30) steering = 30;

    return steering;
}