#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

int main(int argc, char** argv) {
    // 기본값을 ttyPS1으로 설정 (잘 되던 포트)
    std::string port = "/dev/ttyPS1";
    if (argc > 1) port = argv[1];

    std::cout << "[Fixed Non-Blocking Test] Opening " << port << " at 115200..." << std::endl;

    // 검증된 O_NDELAY 방식으로 오픈
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Open failed: " << strerror(errno) << std::endl;
        return 1;
    }

    // 터미널 속성 초기화 및 설정
    struct termios options;
    tcgetattr(fd, &options);
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag |= (CLOCAL | CREAD | CS8);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    
    // 이 부분이 핵심: 시스템이 임의로 패리티/프레임 에러를 0x00으로 변환하는 것을 방지
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    
    // fcntl로 확실하게 노딜레이 유지 확인
    fcntl(fd, F_SETFL, O_NDELAY);

    std::cout << "⚠️  보드의 [TX 핀]과 [RX 핀]이 점퍼선으로 연결되어 있는지 확인하세요!" << std::endl;
    std::cout << "시작하려면 엔터(Enter) 키를 누르세요..." << std::endl;
    std::cin.get();

    char tx_data = 'A'; // 0x41
    int loop_counter = 0;

    while (true) {
        // 1. 송신 파트: 5ms * 200 = 1000ms (1초) 마다 전송
        if (loop_counter >= 200) {
            int bytes_written = write(fd, &tx_data, 1);
            if (bytes_written > 0) {
                std::cout << "[TX] 보냄: " << tx_data << " (0x" << std::hex << (int)tx_data << std::dec << ")" << std::endl;
            }
            loop_counter = 0;
        }

        // 2. 수신 파트: 매 루프(5ms) 마다 데이터 확인
        uint8_t rx_buf;
        int n = read(fd, &rx_buf, 1);
        
        // O_NDELAY 모드에서는 데이터가 정말로 있어야만 n > 0이 됩니다.
        if (n > 0) {
            std::cout << "        [RX] 수신 성공! Hex: 0x" << std::hex << (int)rx_buf 
                      << " | Char: '" << (char)(rx_buf >= 32 && rx_buf <= 126 ? rx_buf : '.') 
                      << "'" << std::dec << std::endl;
        }

        // 5ms 대기
        usleep(5000); 
        loop_counter++;
    }

    close(fd);
    return 0;
}