#ifndef UART_NODE_H
#define UART_NODE_H

#include <string>
#include <vector>
#include "DataTypes.h"

class UartNode {
private:
    int fd;
public:
    UartNode(const char* port, int baudrate);
    ~UartNode();
    
    // 반드시 vector가 두 번 겹쳐진 형태여야 합니다!
    void sendObstacles(const std::vector<std::vector<PointGrid>>& clusters);
    
    bool receiveCommand(int& servo_id, int& angle);
};

#endif