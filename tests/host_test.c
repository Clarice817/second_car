/**
  * @file    host_test.c
  * @brief   PC 上的协议单元测试（不需要开发板）
  *
  *  编译并运行（在工程根目录）：
  *    gcc -std=c11 -I Core/Inc -I tests/inc tests/host_test.c Core/Src/motor_proto.c Core/Src/pc_cmd.c -o test.exe
  *    test.exe
  *
  *  作用：验证 $spd:..# 等帧拼得对不对、$MSPD:..# 等回传解析得对不对。
  *  改了协议相关代码后跑一下，比烧板子快得多。
  */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "uart_io.h"
#include "motor_proto.h"
#include "pc_cmd.h"

/* ---------- 假的串口 + 假的时间 ---------- */

static char pc_in[512];
static int  pc_len, pc_pos;
static char mot_in[512];
static int  mot_len, mot_pos;
static char tx_log[4096]; /* 记录“发给驱动板”的字节 */
static int  tx_len;

static int fails;

uint32_t HAL_GetTick(void)
{
    static uint32_t t = 1000;
    return t += 10;
}

void uart_io_init(void) {}
void uart_io_write(uart_io_id_t id, const void *data, uint16_t len)
{
    (void)id;
    memcpy(&tx_log[tx_len], data, len);
    tx_len += len;
    tx_log[tx_len] = '\0';
}
void uart_io_puts(uart_io_id_t id, const char *s) { uart_io_write(id, s, (uint16_t)strlen(s)); }
uint8_t uart_io_read(uart_io_id_t id, uint8_t *b)
{
    if (id == UART_IO_PC)
    {
        if (pc_pos < pc_len) { *b = (uint8_t)pc_in[pc_pos++]; return 1; }
        return 0;
    }
    if (mot_pos < mot_len) { *b = (uint8_t)mot_in[mot_pos++]; return 1; }
    return 0;
}

/* ---------- 断言小工具 ---------- */

static void tx_reset(void) { tx_len = 0; tx_log[0] = '\0'; }

static void feed_motor(const char *s)
{
    memcpy(&mot_in[mot_len], s, strlen(s));
    mot_len += (int)strlen(s);
}

static void feed_pc(const char *s)
{
    memcpy(&pc_in[pc_len], s, strlen(s));
    pc_len += (int)strlen(s);
}

static void expect_tx(const char *what, const char *want)
{
    if (strcmp(tx_log, want) != 0)
    {
        printf("FAIL %s\n   want: %s\n   got : %s\n", what, want, tx_log);
        fails++;
    }
    else
    {
        printf("ok   %s -> %s\n", what, tx_log);
    }
}

static void expect_int(const char *what, long want, long got)
{
    if (want != got) { printf("FAIL %s: want %ld got %ld\n", what, want, got); fails++; }
    else { printf("ok   %s = %ld\n", what, got); }
}

static void expect_f(const char *what, float want, float got)
{
    float d = want - got;
    if (d < 0) { d = -d; }
    if (d > 0.01f) { printf("FAIL %s: want %.2f got %.2f\n", what, want, got); fails++; }
    else { printf("ok   %s = %.2f\n", what, got); }
}

int main(void)
{
    printf("===== 1. 协议打包（STM32 -> 驱动板）=====\n");
    tx_reset();
    motor_send_speed(300, -300, 0, 50);
    expect_tx("spd 正常值", "$spd:300,-300,0,50#");

    tx_reset();
    motor_send_speed(5000, -5000, 0, 0);
    expect_tx("spd 超范围钳位到 1000", "$spd:1000,-1000,0,0#");

    tx_reset();
    motor_send_pwm(1200, -3600, 0, 9999);
    expect_tx("pwm 钳位到 3600", "$pwm:1200,-3600,0,3600#");

    tx_reset();
    motor_send_upload(0, 1, 1);
    expect_tx("upload", "$upload:0,1,1#");

    tx_reset();
    motor_send_pid(0.5f, 0.01f, 0.1f);
    expect_tx("pid 三位小数", "$MPID:0.500,0.010,0.100#");

    tx_reset();
    motor_send_wdiameter(65.0f);
    expect_tx("wdiameter", "$wdiameter:65.000#");

    tx_reset();
    motor_send_cmd("read_vol", "");
    expect_tx("read_vol", "$read_vol#");

    printf("\n===== 2. 回传解析（驱动板 -> STM32）=====\n");
    feed_motor("$MTEP:12,-3,0,4#");
    feed_motor("$MSPD:120.50,-3.25,0.00,7.75#");
    feed_motor("$MAll:1000,2000,-3000,4#");
    feed_motor("$flash:1,2#"); /* 不认识的标签要被忽略 */
    motor_proto_poll();

    expect_int("delta[0]", 12, g_motor.delta[0]);
    expect_int("delta[1]", -3, g_motor.delta[1]);
    expect_int("delta[3]", 4, g_motor.delta[3]);
    expect_f("speed[0]", 120.50f, g_motor.speed[0]);
    expect_f("speed[1]", -3.25f, g_motor.speed[1]);
    expect_int("total[2]", -3000, g_motor.total[2]);
    expect_int("有效帧数", 3, (long)g_motor.frames);

    printf("\n===== 3. 电脑命令 -> 协议帧 =====\n");
    tx_reset();
    feed_pc("spd 300\r");
    pc_cmd_poll();
    expect_tx("spd 300", "$spd:300,300,300,300#");

    tx_reset();
    feed_pc("spd -300,200,0,50\r\n");
    pc_cmd_poll();
    expect_tx("spd 四路", "$spd:-300,200,0,50#");

    tx_reset();
    feed_pc("stop\r");
    pc_cmd_poll();
    expect_tx("stop", "$spd:0,0,0,0#");

    tx_reset();
    feed_pc("pid 0.5,0.01,0.1\r");
    pc_cmd_poll();
    expect_tx("pid 命令", "$MPID:0.500,0.010,0.100#");

    tx_reset();
    feed_pc("wdiameter 65\r");
    pc_cmd_poll();
    expect_tx("wdiameter 命令", "$wdiameter:65.000#");

    tx_reset();
    feed_pc("raw $read_vol#\r");
    pc_cmd_poll();
    expect_tx("raw 原样转发", "$read_vol#");

    tx_reset();
    feed_pc("mtype 2\r");
    pc_cmd_poll();
    expect_tx("mtype", "$mtype:2#");

    tx_reset();
    feed_pc("MPID 1,2,3\r");
    pc_cmd_poll();
    expect_tx("未知命令按 $name:args# 发", "$MPID:1,2,3#");

    printf("\n===== 结果：%s =====\n", (fails == 0) ? "全部通过" : "有失败");
    return fails;
}
