#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <string>

// ═══════════════════════════════════════════════
// 서보 모터 제어 클래스 (해치 개폐용)
// Linux sysfs PWM 인터페이스 사용
// ═══════════════════════════════════════════════
class ServoNode {
private:
    std::string pwm_path;  // PWM 디바이스 경로 (/sys/class/pwm/...)
    
    // sysfs 파일 쓰기 헬퍼 함수
    void writeSysfs(const std::string& path, const std::string& value);

public:
    // ═══════════════════════════════════════════════
    // 생성자: PWM 채널 초기화
    // @param pwm_chip: PWM 칩 번호 (보통 0)
    // @param pwm_num: PWM 채널 번호 (보통 0)
    // ═══════════════════════════════════════════════
    ServoNode(int pwm_chip, int pwm_num);
    
    // ═══════════════════════════════════════════════
    // 서보 각도 설정
    // @param angle: 목표 각도 (0~180도)
    //   - 90도: 해치 닫힘
    //   - 180도: 해치 열림
    // ═══════════════════════════════════════════════
    void setAngle(int angle);
};

#endif