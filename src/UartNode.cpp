#include "UartNode.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

UartNode::UartNode(const char* port, int baudrate) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "[UART Error] Cannot open " << port << std::endl;
        return;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200); // PC와 115200으로 통신
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &options);
    fcntl(fd, F_SETFL, 0); // Blocking read
}

UartNode::~UartNode() {
    if (fd != -1) close(fd);
}

void UartNode::sendObstacles(const std::vector<std::vector<PointGrid>>& clusters) {
    if (fd == -1 || clusters.empty()) return;
    
    std::string msg = "";
    for (const auto& cluster : clusters) {
        msg += "[";
        for (size_t i = 0; i < cluster.size(); ++i) {
            msg += "[" + std::to_string(cluster[i].x) + "," + std::to_string(cluster[i].y) + "]";
            if (i != cluster.size() - 1) msg += ",";
        }
        msg += "]";
    }
    msg += "\n"; // \\n 대신 \n 사용
    
    ssize_t bytes_written = write(fd, msg.c_str(), msg.length());
    if (bytes_written < 0) {
        std::cerr << "[UART Error] Write failed." << std::endl;
    }
}

bool UartNode::receiveCommand(int& servo_id, int& angle) {
    if (fd == -1) return false;

    char buffer[64];
    memset(buffer, 0, sizeof(buffer));
    int n = read(fd, buffer, sizeof(buffer)-1);

    if (n > 0) {
        // "1,60\r\n" 처럼 뒤에 개행문자가 붙어도 무시하고 숫자만 파싱하도록 수정
        if (sscanf(buffer, "%d,%d", &servo_id, &angle) >= 2) {
            return true;
        }
    }
    return false;
}