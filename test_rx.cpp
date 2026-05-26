#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main(int argc, char** argv) {
    std::string port = "/dev/ttyS2";
    if (argc > 1) port = argv[1];

    std::cout << "[RX TEST] Opening " << port << " at 115200..." << std::endl;
    // 읽기 전용, Non-blocking 모드로 오픈
    int fd = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Open failed: " << strerror(errno) << std::endl;
        return 1;
    }

    // 최소한의 순수 RAW 모드 설정
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);

    std::cout << "Waiting for data... Press Ctrl+C to stop." << std::endl;

    uint8_t buffer;
    while (true) {
        int n = read(fd, &buffer, 1);
        if (n > 0) {
            std::cout << "[RX] Recv Hex: 0x" << std::hex << (int)buffer 
                      << " | Char: " << (char)(buffer >= 32 && buffer <= 126 ? buffer : '.') 
                      << std::dec << std::endl;
        } else {
            // 데이터가 없으면 아주 잠깐 쉬어서 CPU 독점 방지
            usleep(1000); 
        }
    }

    close(fd);
    return 0;
}