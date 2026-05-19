#include "ServoNode.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

ServoNode::ServoNode(int pwm_chip, int pwm_num) {
    fprintf(stderr, "  [Servo] Initializing PWM chip %d, num %d...\n", 
            pwm_chip, pwm_num);
    fflush(stderr);
    
    pwm_path = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip) + 
               "/pwm" + std::to_string(pwm_num);
    
    std::string export_path = "/sys/class/pwm/pwmchip" + 
                              std::to_string(pwm_chip) + "/export";
    writeSysfs(export_path, std::to_string(pwm_num));
    usleep(100000);

    writeSysfs(pwm_path + "/period", "20000000");
    writeSysfs(pwm_path + "/polarity", "inversed");
    writeSysfs(pwm_path + "/enable", "1");
    
    fprintf(stderr, "  [Servo] PWM configured\n");
    fflush(stderr);
}

void ServoNode::setAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    long duty_cycle = 500000 + (angle * (2000000 / 180));
    writeSysfs(pwm_path + "/duty_cycle", std::to_string(duty_cycle));
}
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