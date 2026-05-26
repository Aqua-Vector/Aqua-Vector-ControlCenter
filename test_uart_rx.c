/*
 * linkcheck_slave.c — TorpedoFPGA UART ping responder
 *
 * 동작:
 *   ttyS2, ttyS3 동시 대기 (select)
 *   ping 수신 (0xAA 0x01 0xFE) → ACK 응답 (0xAA 0x06 0xF9)
 *   통제소 LinkCheck 테스트가 끝나면 자동 종료
 *
 * 장치 매핑:
 *   /dev/ttyS2  AXI UART 16550, 460800 bps — UART1 (primary)
 *   /dev/ttyS3  AXI UART 16550, 460800 bps — UART2 (redundant)
 *
 * 빌드:
 *   aarch64-linux-gnu-gcc -O2 -Wall -o linkcheck_slave linkcheck_slave.c
 *
 * 실행 순서:
 *   1) 어뢰FPGA 에서 먼저 실행: ./linkcheck_slave
 *   2) 통제소 FPGA 에서 실행:   ./LinkCheck
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <time.h>
#include <errno.h>

/* ═══════════════════════════════════════════════════════════════
 *  설정
 * ═══════════════════════════════════════════════════════════════ */

#define UART1_DEV   "/dev/ttyS2"
#define UART2_DEV   "/dev/ttyS3"
#define UART_BAUD   B460800

/* 통제소가 보내는 ping 패킷 */
static const uint8_t PING[] = { 0xAA, 0x01, 0xFE };
#define PING_LEN  3

/* 어뢰가 보내는 ACK 패킷 */
static const uint8_t ACK[]  = { 0xAA, 0x06, 0xF9 };
#define ACK_LEN   3

/* 각 UART 별로 ping을 1회 수신하면 완료 */
/* 전체 대기 타임아웃 (통제소 LinkCheck 실행 안 할 경우 자동 종료) */
#define TOTAL_TIMEOUT_SEC  300

/* ═══════════════════════════════════════════════════════════════
 *  UART 유틸리티
 * ═══════════════════════════════════════════════════════════════ */

static int uart_open(const char *dev, speed_t baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }
    return fd;
}

/* ═══════════════════════════════════════════════════════════════
 *  main
 * ═══════════════════════════════════════════════════════════════ */

int main(void)
{
    printf("\n");
    printf("==============================================\n");
    printf("  TorpedoFPGA LinkCheck Slave\n");
    printf("  Waiting for ping on ttyS2, ttyS3 ...\n");
    printf("  (timeout: %ds)\n", TOTAL_TIMEOUT_SEC);
    printf("==============================================\n\n");

    /* 두 UART 열기 */
    int fd1 = uart_open(UART1_DEV, UART_BAUD);
    int fd2 = uart_open(UART2_DEV, UART_BAUD);

    if (fd1 < 0) {
        fprintf(stderr, "[ERROR] cannot open %s: %s\n", UART1_DEV, strerror(errno));
        return EXIT_FAILURE;
    }
    if (fd2 < 0) {
        fprintf(stderr, "[ERROR] cannot open %s: %s\n", UART2_DEV, strerror(errno));
        close(fd1);
        return EXIT_FAILURE;
    }

    printf("  [OPEN] %s (UART1 primary)\n", UART1_DEV);
    printf("  [OPEN] %s (UART2 redundant)\n\n", UART2_DEV);

    int done1 = 0, done2 = 0;  /* 각 UART 완료 플래그 */
    uint8_t buf[64];

    time_t start = time(NULL);

    while (!done1 || !done2) {

        /* 전체 타임아웃 체크 */
        if (time(NULL) - start >= TOTAL_TIMEOUT_SEC) {
            printf("\n[TIMEOUT] %ds elapsed.\n", TOTAL_TIMEOUT_SEC);
            if (!done1) printf("  [FAIL] UART1 (%s): no ping received\n", UART1_DEV);
            if (!done2) printf("  [FAIL] UART2 (%s): no ping received\n", UART2_DEV);
            break;
        }

        /* select로 두 fd 동시 감시 */
        fd_set rfds;
        FD_ZERO(&rfds);
        if (!done1) FD_SET(fd1, &rfds);
        if (!done2) FD_SET(fd2, &rfds);
        int maxfd = (fd1 > fd2 ? fd1 : fd2) + 1;

        struct timeval tv = { 1, 0 };  /* 1초마다 타임아웃 체크 반복 */
        int ret = select(maxfd, &rfds, NULL, NULL, &tv);
        if (ret <= 0) continue;

        /* ── UART1 수신 ── */
        if (!done1 && FD_ISSET(fd1, &rfds)) {
            ssize_t r = read(fd1, buf, sizeof(buf));
            if (r >= (ssize_t)PING_LEN &&
                memcmp(buf, PING, PING_LEN) == 0) {
                /* ACK 응답 */
                { ssize_t w1 = write(fd1, ACK, ACK_LEN); (void)w1; }
                printf("  [UART1] ping received -> ACK sent (%s)\n", UART1_DEV);
                done1 = 1;
            } else if (r > 0) {
                /* 잘못된 패킷 — 무시하고 계속 대기 */
                printf("  [UART1] unknown data (%zd bytes), waiting...\n", r);
            }
        }

        /* ── UART2 수신 ── */
        if (!done2 && FD_ISSET(fd2, &rfds)) {
            ssize_t r = read(fd2, buf, sizeof(buf));
            if (r >= (ssize_t)PING_LEN &&
                memcmp(buf, PING, PING_LEN) == 0) {
                { ssize_t w2 = write(fd2, ACK, ACK_LEN); (void)w2; }
                printf("  [UART2] ping received -> ACK sent (%s)\n", UART2_DEV);
                done2 = 1;
            } else if (r > 0) {
                printf("  [UART2] unknown data (%zd bytes), waiting...\n", r);
            }
        }
    }

    printf("\n");
    if (done1 && done2)
        printf("  Result: UART1 + UART2 both OK\n");
    else if (done1)
        printf("  Result: UART1 OK / UART2 FAIL\n");
    else if (done2)
        printf("  Result: UART1 FAIL / UART2 OK\n");
    else
        printf("  Result: UART1 FAIL / UART2 FAIL\n");
    printf("==============================================\n\n");

    close(fd1);
    close(fd2);
    return (done1 && done2) ? EXIT_SUCCESS : EXIT_FAILURE;
}