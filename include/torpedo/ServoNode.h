#ifndef SERVO_NODE_H
#define SERVO_NODE_H

#include <string>

class ServoNode {
private:
    std::string pwm_path;
    void writeSysfs(const std::string& path, const std::string& value);

public:
    ServoNode(int pwm_chip, int pwm_num);
    void setAngle(int angle);
};

#endif