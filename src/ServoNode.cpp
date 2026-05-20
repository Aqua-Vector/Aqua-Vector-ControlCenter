#include "ServoNode.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

// ═══════════════════════════════════════════════
// 생성자: PWM 초기화 및 서보 설정
// ═══════════════════════════════════════════════
ServoNode::ServoNode(int pwm_chip, int pwm_num) {
    fprintf(stderr, "  [Servo] Initializing PWM chip %d, num %d...\n", 
            pwm_chip, pwm_num);
    fflush(stderr);
    
    // PWM 디바이스 경로 생성
    pwm_path = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip) + 
               "/pwm" + std::to_string(pwm_num);
    
    // PWM 채널 export (커널에 등록)
    std::string export_path = "/sys/class/pwm/pwmchip" + 
                              std::to_string(pwm_chip) + "/export";
    writeSysfs(export_path, std::to_string(pwm_num));
    usleep(100000);  // 100ms 대기 (sysfs 파일 생성 시간)

    // PWM 파라미터 설정
    writeSysfs(pwm_path + "/period", "20000000");      // 20ms 주기 (50Hz)
    writeSysfs(pwm_path + "/polarity", "inversed");    // 극성 반전
    writeSysfs(pwm_path + "/enable", "1");             // PWM 활성화
    
    fprintf(stderr, "  [Servo] PWM configured\n");
    fflush(stderr);
}

// ═══════════════════════════════════════════════
// 서보 각도 설정
// PWM duty cycle 계산: 0.5ms~2.5ms (0~180도)
// ═══════════════════════════════════════════════
void ServoNode::setAngle(int angle) {
    // 각도 제한
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Duty cycle 계산 (나노초 단위)
    // 0도: 500,000ns (0.5ms)
    // 180도: 2,500,000ns (2.5ms)
    long duty_cycle = 500000 + (angle * (2000000 / 180));
    writeSysfs(pwm_path + "/duty_cycle", std::to_string(duty_cycle));
}

// ═══════════════════════════════════════════════
// sysfs 파일 쓰기 헬퍼 함수
// ═══════════════════════════════════════════════
void ServoNode::writeSysfs(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << value;
        file.close();
    } else {
        fprintf(stderr, "  [Servo Error] Failed to open: %s\n", path.c_str());
        fflush(stderr);
    }
}