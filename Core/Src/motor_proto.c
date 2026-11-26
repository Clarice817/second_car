/**
  ******************************************************************************
  * @file    motor_proto.c
  * @brief   驱动板串口协议：打包下发命令 + 解析回传数据
  *
  * 协议格式： '$' 命令名 ':' 参数1,参数2,... '#'
  *   例：$spd:300,-300,0,0#   表示 M1 正转 300、M2 反转 300、M3/M4 停
  *
  * 注意：本工程用的是 newlib-nano，printf 不支持 %f（要加 -u _printf_float
  *       才行）。所以下面用 put_fixed() 做定点小数，既能省 flash 又够用。
  ******************************************************************************
  */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_proto.h"
#include "uart_io.h"
#include "stm32f1xx_hal.h"

#define MOTOR_UART  UART_IO_MOTOR /* 驱动板接在 USART2 上 */
#define SPD_LIMIT   1000          /* 闭环速度范围 ±1000 */
#define PWM_LIMIT   3600          /* 开环 PWM 范围 ±3600 */

/* 协议里的 PID 命令名：文档写的是大写 $MPID:，
   如果实测不生效，改成 "$mpid:" 试试（不同固件版本大小写不一致） */
#define CMD_PID "$MPID:"

motor_feedback_t g_motor;

/* 收帧状态机 */
static char    s_frame[64]; /* 一帧的内容，不含 '$' 和 '#' */
static uint8_t s_len;
static uint8_t s_in_frame;

/* ---------------- 小工具 ---------------- */

static int16_t clamp16(int32_t v, int32_t limit)
{
    if (v > limit)
    {
        v = limit;
    }
    if (v < -limit)
    {
        v = -limit;
    }
    return (int16_t)v;
}

/* 把浮点数写成定点字符串，追加到 p 后面，返回新的写入位置。
   dec 只能是 2 或 3，分别表示两位/三位小数。 */
static char *put_fixed(char *p, float v, uint8_t dec)
{
    int32_t scale = (dec == 3) ? 1000 : 100;
    int32_t x = (int32_t)(v * (float)scale + (v < 0.0f ? -0.5f : 0.5f));
    int32_t whole = x / scale;
    int32_t frac = x % scale;
    char frac_buf[8];
    uint8_t i;

    if (frac < 0)
    {
        frac = -frac;
    }
    /* 手工补前导零：0.05 -> "050"，避免依赖 printf 的宽度格式 */
    for (i = 0; i < dec; i++)
    {
        frac_buf[dec - 1u - i] = (char)('0' + (frac % 10));
        frac /= 10;
    }
    frac_buf[dec] = '\0';

    if ((x < 0) && (whole == 0))
    {
        *p++ = '-'; /* -0.005 这种小负数，别漏掉负号 */
    }
    p += sprintf(p, "%ld.%s", (long)whole, frac_buf);
    return p;
}

static void send_line(const char *s)
{
    uart_io_puts(MOTOR_UART, s);
}

/* ---------------- 初始化 ---------------- */

void motor_proto_init(void)
{
    memset(&g_motor, 0, sizeof(g_motor));
    g_motor.last_ms = HAL_GetTick();
    s_len = 0;
    s_in_frame = 0;

    /* 默认让驱动板回传“10ms 增量 + 速度”。
       想看累计编码器就发 $upload:1,1,1#（数据量会变大） */
    motor_send_upload(0, 1, 1);
}

/* ---------------- 下发命令 ---------------- */

void motor_send_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    char buf[40];
    (void)snprintf(buf, sizeof(buf), "$spd:%d,%d,%d,%d#",
                   clamp16(m1, SPD_LIMIT), clamp16(m2, SPD_LIMIT),
                   clamp16(m3, SPD_LIMIT), clamp16(m4, SPD_LIMIT));
    send_line(buf);
}

void motor_send_speed_all(int16_t spd)
{
    motor_send_speed(spd, spd, spd, spd);
}

void motor_send_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    char buf[40];
    (void)snprintf(buf, sizeof(buf), "$pwm:%d,%d,%d,%d#",
                   clamp16(m1, PWM_LIMIT), clamp16(m2, PWM_LIMIT),
                   clamp16(m3, PWM_LIMIT), clamp16(m4, PWM_LIMIT));
    send_line(buf);
}

void motor_send_upload(uint8_t total_en, uint8_t delta_en, uint8_t speed_en)
{
    char buf[32];
    (void)snprintf(buf, sizeof(buf), "$upload:%d,%d,%d#",
                   total_en ? 1 : 0, delta_en ? 1 : 0, speed_en ? 1 : 0);
    send_line(buf);
}

void motor_send_mtype(uint8_t type)
{
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "$mtype:%d#", type);
    send_line(buf);
}

void motor_send_mline(uint16_t line)
{
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "$mline:%d#", line);
    send_line(buf);
}

void motor_send_mphase(uint16_t ratio)
{
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "$mphase:%d#", ratio);
    send_line(buf);
}

void motor_send_deadzone(uint16_t dz)
{
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "$deadzone:%d#", dz);
    send_line(buf);
}

void motor_send_wdiameter(float mm)
{
    char buf[32];
    char *p = buf;
    p += sprintf(p, "$wdiameter:");
    p = put_fixed(p, mm, 3);
    *p++ = '#';
    *p = '\0';
    send_line(buf);
}

void motor_send_pid(float p_, float i_, float d_)
{
    char buf[56];
    char *p = buf;
    p += sprintf(p, CMD_PID);
    p = put_fixed(p, p_, 3);
    *p++ = ',';
    p = put_fixed(p, i_, 3);
    *p++ = ',';
    p = put_fixed(p, d_, 3);
    *p++ = '#';
    *p = '\0';
    send_line(buf);
}

void motor_send_cmd(const char *name, const char *args)
{
    uart_io_puts(MOTOR_UART, "$");
    uart_io_puts(MOTOR_UART, name);
    if (args != NULL && args[0] != '\0')
    {
        uart_io_puts(MOTOR_UART, ":");
        uart_io_puts(MOTOR_UART, args);
    }
    uart_io_puts(MOTOR_UART, "#");
}

void motor_send_frame(const char *frame)
{
    send_line(frame);
}

void motor_format_fixed(char *out, uint16_t size, float v, uint8_t dec)
{
    char tmp[24];
    char *p = put_fixed(tmp, v, dec);
    *p = '\0';
    (void)strncpy(out, tmp, (size_t)(size - 1u));
    out[size - 1u] = '\0';
}

/* ---------------- 接收解析 ---------------- */

static void parse_int4(char *s, int32_t *dst)
{
    uint8_t i;
    for (i = 0; i < MOTOR_NUM; i++)
    {
        char *tok = strtok((i == 0) ? s : NULL, ", ");
        dst[i] = (tok != NULL) ? (int32_t)atol(tok) : 0;
    }
}

static void parse_short4(char *s, int16_t *dst)
{
    uint8_t i;
    for (i = 0; i < MOTOR_NUM; i++)
    {
        char *tok = strtok((i == 0) ? s : NULL, ", ");
        dst[i] = (tok != NULL) ? (int16_t)atoi(tok) : 0;
    }
}

static void parse_float4(char *s, float *dst)
{
    uint8_t i;
    for (i = 0; i < MOTOR_NUM; i++)
    {
        char *tok = strtok((i == 0) ? s : NULL, ", ");
        dst[i] = (tok != NULL) ? (float)atof(tok) : 0.0f;
    }
}

/* 处理一帧，形如 "MSPD:120.50,120.00,0.00,0.00" */
static void handle_frame(char *frame)
{
    char *colon = strchr(frame, ':');
    char *tag;
    char *vals;

    if (colon == NULL)
    {
        /* 没有冒号（比如 $ok#），原样打印出来，方便排查 */
        printf("[motor] 未知帧: $%s#\r\n", frame);
        return;
    }
    *colon = '\0';
    tag = frame;
    vals = colon + 1;

    if (strcmp(tag, "MAll") == 0)
    {
        parse_int4(vals, g_motor.total);
    }
    else if (strcmp(tag, "MTEP") == 0)
    {
        parse_short4(vals, g_motor.delta);
    }
    else if (strcmp(tag, "MSPD") == 0)
    {
        parse_float4(vals, g_motor.speed);
    }
    else
    {
        /* 不认识的标签（比如 $flash:...#、$vol:...#）：原样打印，方便调试。
           调完可以把它注释掉，避免刷屏。 */
        printf("[motor] 未知帧: $%s:%s#\r\n", tag, vals);
        return;
    }

    g_motor.frames++;
    g_motor.last_ms = HAL_GetTick();
}

void motor_proto_poll(void)
{
    uint8_t ch;

    while (uart_io_read(MOTOR_UART, &ch))
    {
        if (ch == '$')
        {
            /* 帧头：重新开始攒 */
            s_in_frame = 1;
            s_len = 0;
            continue;
        }
        if (!s_in_frame)
        {
            continue; /* 帧外乱码，丢掉 */
        }
        if (ch == '#')
        {
            /* 帧尾：交给解析 */
            s_in_frame = 0;
            s_frame[s_len] = '\0';
            handle_frame(s_frame);
            continue;
        }
        if (ch == '\r' || ch == '\n')
        {
            s_in_frame = 0; /* 只收到半帧就换行了，丢弃 */
            continue;
        }
        if (s_len < (sizeof(s_frame) - 1u))
        {
            s_frame[s_len++] = (char)ch;
        }
        else
        {
            s_in_frame = 0; /* 超长帧，丢弃 */
        }
    }
}
