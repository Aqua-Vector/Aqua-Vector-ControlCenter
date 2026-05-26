#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main(int argc, char** argv) {
    std::string port = "/dev/ttyS2";
    if (argc > 1) port = argv[1];

    std::cout << "[TX TEST] Opening " << port << " at 115200..." << std::endl;
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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

    std::cout << "Sending 'A' (0x41) every 1 second. Press Ctrl+C to stop." << std::endl;

    char data = 'A';
    while (true) {
        int bytes_written = write(fd, &data, 1);
        if (bytes_written > 0) {
            std::cout << "[TX] Sent: " << data << " (0x" << std::hex << (int)data << std::dec << ")" << std::endl;
        } else {
            std::cerr << "Write error!" << std::endl;
        }
        sleep(1);
    }

    close(fd);
    return 0;
}