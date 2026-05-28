/*
 * linkcheck.c — 통제소 FPGA 연동 확인 프로그램 (실행 전 사전 점검용)
 *
 * 테스트 Mode:
 *   [ 라이다 ] 장치 파일(/dev/ttyUSB0) 존재 + 열기 가능 여부 확인  (실제 동작 X)
 *   [어뢰UART] 실제 소형 패킷 송수신으로 통신 확인
 *   [  PWM   ] sysfs 경로(/sys/class/pwm/pwmchip0) 존재 여부 확인  (실제 동작 X)
 *
 * 장치 매핑:
 *   /dev/ttyUSB0  RPLIDAR C1,     460800 bps
 *   /dev/ttyS2    AXI UART 16550, 460800 bps — 어뢰FPGA UART1 (primary  ,   JD Pin1,2)
 *   /dev/ttyS3    AXI UART 16550, 460800 bps — 어뢰FPGA UART2 (redundant, JD Pin7,8)
 *   pwmchip0/pwm0 pwm
 * 
 * 빌드 방법:
 *   aarch64-linux-gnu-gcc -O2 -Wall -o linkcheck linkcheck.c
 *
 * 실행 방법:
 *   sudo ./linkcheck
 *
 * 로그 경로:
 *   /var/log/linkcheck.log  (append)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>


  /* ── 장치 경로 ── */
#define LIDAR_DEV        "/dev/ttyUSB0"
#define TORPEDO_UART1    "/dev/ttyS2"
#define TORPEDO_UART2    "/dev/ttyS3"

/* ── 어뢰 UART 보레이트 ── */
#define TORPEDO_BAUD     B460800

/*
 * ── 어뢰FPGA 핑 패킷 (UART1 / UART2 공용) ──
 *   헤더(0xAA) + 커맨드(0x01, PING) + 데이터(22바이트) + 체크섬(1바이트) = 총 25바이트
 */
#define TORPEDO_PACKET_SIZE 25
static uint8_t TORPEDO_PING[TORPEDO_PACKET_SIZE];

static void init_ping_packet(void)
{
    memset(TORPEDO_PING, 0, TORPEDO_PACKET_SIZE);
    TORPEDO_PING[0] = 0xAA;
    TORPEDO_PING[1] = 0x01;
    /* 데이터 영역은 0으로 채움 (index 2~23) */
    TORPEDO_PING[TORPEDO_PACKET_SIZE - 1] = 0xFE;
}

#define TORPEDO_MIN_RESP  TORPEDO_PACKET_SIZE
#define UART_TIMEOUT_SEC  2

/* ── PWM sysfs 확인 경로 ── */
#define PWM_CHIP_ID   0                /* pwmchip0 */

/* ── 로그 파일 ── */
#define LOG_PATH  "/var/log/linkcheck.log"

/* ═══════════════════════════════════════════════════════════════
 *  내부 공통
 * ═══════════════════════════════════════════════════════════════ */

typedef enum { PASS = 0, FAIL } Result;
static FILE* g_log = NULL;

static void log_print(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
    if (g_log) {
        va_start(ap, fmt); vfprintf(g_log, fmt, ap); va_end(ap);
        fflush(g_log);
    }
}

static void log_timestamp(void)
{
    time_t t = time(NULL);
    struct tm* tm = localtime(&t);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm);
    log_print("[%s]\n", buf);
}

static void print_result(const char* name, Result r, const char* detail)
{
    const char* mark = (r == PASS) ? "[ PASS ]" : "[ FAIL ]";
    log_print("  %s  %-44s  %s\n", mark, name, detail ? detail : "");
}

/* ═══════════════════════════════════════════════════════════════
 *  1. 라이다 — 장치 파일 존재 + 열기 가능 여부
 *     (실제 커맨드 전송 없음 — 실행 전 점검용)
 * ═══════════════════════════════════════════════════════════════ */
static Result check_lidar(void)
{
    char detail[128] = "";

    /* stat()으로 파일 존재 확인 */
    struct stat st;
    if (stat(LIDAR_DEV, &st) < 0) {
        snprintf(detail, sizeof(detail),
            "device file not found (%s): %s", LIDAR_DEV, strerror(errno));
        print_result("LiDAR device file (ttyUSB0)", FAIL, detail);
        return FAIL;
    }

    /* 문자 장치(character device) 여부 확인 */
    if (!S_ISCHR(st.st_mode)) {
        snprintf(detail, sizeof(detail),
            "%s is not a character device", LIDAR_DEV);
        print_result("LiDAR device file (ttyUSB0)", FAIL, detail);
        return FAIL;
    }

    /* 실제 열기 가능 여부 확인 */
    int fd = open(LIDAR_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        snprintf(detail, sizeof(detail),
            "open failed (%s): %s", LIDAR_DEV, strerror(errno));
        print_result("LiDAR device file (ttyUSB0)", FAIL, detail);
        return FAIL;
    }
    close(fd);

    snprintf(detail, sizeof(detail), "device file OK (%s)", LIDAR_DEV);
    print_result("LiDAR device file (ttyUSB0)", PASS, detail);
    return PASS;
}

/* ═══════════════════════════════════════════════════════════════
 *  2. 어뢰FPGA UART — 실제 소형 패킷 송수신
 * ═══════════════════════════════════════════════════════════════ */

static int uart_open(const char* dev, speed_t baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    return fd;
}

static Result uart_handshake(int fd,
    const uint8_t* ping, size_t ping_len,
    int min_resp,
    char* errbuf, size_t errsz)
{
    struct timespec ts_start, ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    /* Tx: 핑 패킷 전송 */
    if (write(fd, ping, ping_len) != (ssize_t)ping_len) {
        snprintf(errbuf, errsz, "Tx failed: %s", strerror(errno));
        return FAIL;
    }

    uint8_t rx_buf[512];
    int rx_len = 0;
    
    /* 타임아웃 내에서 데이터가 총 25바이트 이상 쌓일 때까지 select 루프 반복 */
    while (rx_len < min_resp) {
        fd_set rfds;
        struct timeval tv = { UART_TIMEOUT_SEC, 0 }; // 루프마다 타임아웃 재설정
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            snprintf(errbuf, errsz, "select error: %s", strerror(errno));
            return FAIL;
        }
        if (ret == 0) {
            snprintf(errbuf, errsz, "timeout (no response or incomplete: %d/%d bytes)", rx_len, min_resp);
            return FAIL;
        }

        // 남은 버퍼 공간만큼 이어 받기
        ssize_t r = read(fd, rx_buf + rx_len, sizeof(rx_buf) - rx_len);
        if (r > 0) {
            rx_len += r;
        } else if (r < 0 && errno != EAGAIN) {
            snprintf(errbuf, errsz, "read error: %s", strerror(errno));
            return FAIL;
        }
    }

    /* 수신 완료 후 시간 측정 */
    clock_gettime(CLOCK_MONOTONIC, &ts_end);

    /* ACK 패킷 검증 추가 (0xAA 0x06 확인) */
    if (rx_buf[0] != 0xAA || rx_buf[1] != 0x06) {
        snprintf(errbuf, errsz, "Invalid ACK header (0x%02X 0x%02X)", rx_buf[0], rx_buf[1]);
        return FAIL;
    }

    double elapsed_ms = (ts_end.tv_sec - ts_start.tv_sec) * 1000.0 +
                        (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000.0;

    snprintf(errbuf, errsz, "Tx/Rx OK (%d bytes), RTT: %.3f ms", rx_len, elapsed_ms);
    return PASS;
}

static Result check_torpedo(int ch, const char* dev)
{
    char name[56], detail[128] = "";
    snprintf(name, sizeof(name), "TorpedoFPGA UART%d (%s, %s)",
        ch, dev, (ch == 1) ? "primary  " : "redundant");

    int fd = uart_open(dev, TORPEDO_BAUD);
    if (fd < 0) {
        snprintf(detail, sizeof(detail), "device open failed: %s", strerror(errno));
        print_result(name, FAIL, detail);
        return FAIL;
    }

    Result r = uart_handshake(fd,
        TORPEDO_PING, TORPEDO_PACKET_SIZE,
        TORPEDO_MIN_RESP, detail, sizeof(detail));
    close(fd);
    print_result(name, r, detail);
    return r;
}

/* ═══════════════════════════════════════════════════════════════
 *  3. PWM — sysfs 경로 존재 여부만 확인
 *     (export / enable 없음 — 실행 전 점검용)
 * ═══════════════════════════════════════════════════════════════ */
static Result check_pwm(void)
{
    char detail[128] = "";
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d", PWM_CHIP_ID);

    struct stat st;
    if (stat(path, &st) < 0) {
        snprintf(detail, sizeof(detail), "path not found (%s): %s", path, strerror(errno));
        print_result("PWM motor sysfs (pwmchip0)", FAIL, detail);
        return FAIL;
    }

    snprintf(detail, sizeof(detail), "path OK (%s)", path);
    print_result("PWM motor sysfs (pwmchip0)", PASS, detail);
    return PASS;
}

/* ═══════════════════════════════════════════════════════════════
 *  main
 * ═══════════════════════════════════════════════════════════════ */
int main(void)
{
    init_ping_packet();

    g_log = fopen(LOG_PATH, "a");
    if (!g_log)
        fprintf(stderr, "WARNING: cannot open log file (%s) -- stdout only\n", LOG_PATH);

    log_print("\n");
    log_print("╔══════════════════════════════════════════════╗\n");
    log_print("║       Control FPGA Link Check Program     ║\n");
    log_print("║                (Pre-run Verification)          ║\n");
    log_print("╚══════════════════════════════════════════════╝\n");
    log_timestamp();
    log_print("\n");
    log_print("  %-6s  %-44s  %s\n", "Mode", "항목", "Detail");
    log_print("  %-6s  %-44s  %s\n", "------", "--------------------------------------------", "--------");
    log_print("  %-6s  %s\n", "[FILE]", "LiDAR/PWM   : check device/sysfs path existence");
    log_print("  %-6s  %s\n", "[UART]", "TorpedoFPGA : verify with actual packet Tx/Rx  ");
    log_print("\n");

    int total = 4, failed = 0;

    if (check_lidar() != PASS) failed++;
    if (check_torpedo(1, TORPEDO_UART1) != PASS) failed++;
    if (check_torpedo(2, TORPEDO_UART2) != PASS) failed++;
    if (check_pwm() != PASS) failed++;

    log_print("\n");
    log_print("──────────────────────────────────────────────\n");
    log_print("  Result: %d / %d passed", total - failed, total);
    if (failed == 0)
        log_print("  --> All OK, ready to run\n");
    else
        log_print("  --> %d item(s) FAILED, check required\n", failed);
    log_print("──────────────────────────────────────────────\n\n");

    if (g_log) fclose(g_log);
    return (failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}