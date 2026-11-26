/**
  ******************************************************************************
  * @file    pc_cmd.c
  * @brief   电脑端命令解析 + 编码器数据打印
  *
  * 数据流：
  *   电脑 --"spd 300"--> USART1 --> 本文件解析 --> motor_send_speed() --> USART2 --> 驱动板
  *   驱动板 --"$MSPD:..."--> USART2 --> motor_proto_poll() --> g_motor --> 本文件打印 --> USART1 --> 电脑
  ******************************************************************************
  */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> /* strcasecmp / strncasecmp */

#include "pc_cmd.h"
#include "uart_io.h"
#include "motor_proto.h"
#include "stm32f1xx_hal.h"

#define PC_LINE_MAX      80   /* 一行命令最长多少字节 */
#define REPORT_PERIOD_MS 100  /* 编码器数据多久往电脑打一次（毫秒） */
#define LINK_TIMEOUT_MS  1000 /* 多久没收到驱动板回传就报警 */

static char     s_line[PC_LINE_MAX];
static uint8_t  s_len;
static uint32_t s_last_report_ms;
static uint32_t s_last_frames;

/* 上一次打印的值，用来做“只打印变化”的比较 */
static int32_t s_prev_speed[MOTOR_NUM]; /* 速度乘 100 存成整数，避免浮点比较 */
static int16_t s_prev_delta[MOTOR_NUM];
static int32_t s_prev_total[MOTOR_NUM];
static uint8_t s_have_prev;
static uint8_t  s_link_lost;

/* ---------------- 打印小工具 ---------------- */

/* newlib-nano 的 printf 不支持 %f，用 motor_format_fixed() 转成字符串再打印 */
static void print_fixed_val(float v, uint8_t dec)
{
    char b[16];
    motor_format_fixed(b, sizeof(b), v, dec);
    printf(" %s", b);
}

static void print_help(void)
{
    printf("\r\n");
    printf("================ second_car 电机调试命令 ================\r\n");
    printf(" spd 300            四路都正转 300（范围 -1000~1000）\r\n");
    printf(" spd -300           四路都反转 300\r\n");
    printf(" spd 300,-300,0,0   分别设置 M1~M4（正负号=转向）\r\n");
    printf(" pwm 1200           开环 PWM（范围 -3600~3600）\r\n");
    printf(" stop               停车（速度全部清零）\r\n");
    printf(" upload 0,1,1       上报开关: 总编码器 / 10ms增量 / 速度\r\n");
    printf(" get                立刻打印一次编码器数据\r\n");
    printf(" mtype 2            电机型号 1:520 2:310 3:TT带编码器 4:TT\r\n");
    printf(" mline 13           编码器线数\r\n");
    printf(" mphase 30          减速比\r\n");
    printf(" deadzone 1600      PWM 死区\r\n");
    printf(" wdiameter 65       轮子直径 mm\r\n");
    printf(" pid 0.5,0.01,0.1   PID 参数\r\n");
    printf(" read_flash         查询驱动板里的配置\r\n");
    printf(" read_vol           查询电池电压\r\n");
    printf(" reset              恢复出厂设置\r\n");
    printf(" raw $read_vol#     直接发一帧（调试未知命令用）\r\n");
    printf(" help               显示本帮助\r\n");
    printf("========================================================\r\n");
}

/* ---------------- 打印编码器数据 ---------------- */

static void print_feedback(void)
{
    uint8_t i;

    printf("[%lu ms] ", (unsigned long)HAL_GetTick());

    /* 速度 mm/s */
    printf("spd(mm/s):");
    for (i = 0; i < MOTOR_NUM; i++)
    {
        print_fixed_val(g_motor.speed[i], 2);
    }

    /* 10ms 编码器增量 */
    printf(" | d:");
    for (i = 0; i < MOTOR_NUM; i++)
    {
        printf(" %d", (int)g_motor.delta[i]);
    }

    /* 累计编码器 */
    printf(" | tot:");
    for (i = 0; i < MOTOR_NUM; i++)
    {
        printf(" %ld", (long)g_motor.total[i]);
    }
    printf("\r\n");
}

/* 把当前回传值和“上次打印的值”比较，有变化才返回 1
   这样车停着的时候屏幕完全安静，一动就有输出 */
static uint8_t feedback_changed(void)
{
    uint8_t i;

    if (!s_have_prev)
    {
        return 1; /* 上电后第一帧一定要打印 */
    }

    for (i = 0; i < MOTOR_NUM; i++)
    {
        int32_t spd = (int32_t)(g_motor.speed[i] * 100.0f +
                                ((g_motor.speed[i] < 0.0f) ? -0.5f : 0.5f));

        if ((spd != s_prev_speed[i]) ||
            (g_motor.delta[i] != s_prev_delta[i]) ||
            (g_motor.total[i] != s_prev_total[i]))
        {
            return 1;
        }
    }
    return 0;
}

/* 记下这次打印的值 */
static void feedback_save(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_NUM; i++)
    {
        s_prev_speed[i] = (int32_t)(g_motor.speed[i] * 100.0f +
                                    ((g_motor.speed[i] < 0.0f) ? -0.5f : 0.5f));
        s_prev_delta[i] = g_motor.delta[i];
        s_prev_total[i] = g_motor.total[i];
    }
    s_have_prev = 1;
}

void pc_report_poll(void)
{
    uint32_t now = HAL_GetTick();

    /* 超时没回传：只提醒一次，避免刷屏 */
    if ((uint32_t)(now - g_motor.last_ms) > LINK_TIMEOUT_MS)
    {
        if (!s_link_lost)
        {
            s_link_lost = 1;
            s_have_prev = 0; /* 恢复后重新打印第一帧 */
            printf("[warn] %d ms 没收到驱动板回传：检查接线/共地/波特率，或重新发 upload 0,1,1\r\n", LINK_TIMEOUT_MS);
        }
        return;
    }
    if (s_link_lost)
    {
        s_link_lost = 0;
        printf("[ok] 驱动板回传恢复正常\r\n");
    }

    /* 1) 最多每 REPORT_PERIOD_MS 检查一次
       2) 有新帧
       3) 数值和上次打印的不一样 —— 三条都满足才打印 */
    if ((uint32_t)(now - s_last_report_ms) < REPORT_PERIOD_MS)
    {
        return;
    }
    s_last_report_ms = now;

    if (g_motor.frames == s_last_frames)
    {
        return; /* 没有新数据 */
    }
    s_last_frames = g_motor.frames;

    if (!feedback_changed())
    {
        return; /* 数据没变化：不打印，屏幕保持安静 */
    }

    feedback_save();
    print_feedback();
}

/* ---------------- 命令解析 ---------------- */

/* 把 argv[1..argc-1] 用逗号拼起来，例如 pid 0.5 0.01 0.1 -> "0.5,0.01,0.1"
   缓冲区不够就截断，绝不越界 */
static void join_args(int argc, char **argv, char *out, uint16_t out_size)
{
    uint16_t used = 0;
    int i;

    out[0] = '\0';
    for (i = 1; i < argc; i++)
    {
        uint16_t n = (uint16_t)strlen(argv[i]);
        uint16_t need = (uint16_t)(n + ((i > 1) ? 1u : 0u));

        if ((uint32_t)used + need + 1u > out_size)
        {
            break; /* 放不下就停，避免溢出 */
        }
        if (i > 1)
        {
            out[used++] = ',';
        }
        memcpy(&out[used], argv[i], n);
        used = (uint16_t)(used + n);
        out[used] = '\0';
    }
}

/* spd / pwm 两个命令共用的解析 */
static void drive_cmd(int argc, char **argv, uint8_t is_pwm)
{
    int16_t v[MOTOR_NUM];

    if (argc == 2)
    {
        /* 只给一个值：四路都一样 */
        v[0] = v[1] = v[2] = v[3] = (int16_t)atoi(argv[1]);
    }
    else if (argc >= 5)
    {
        v[0] = (int16_t)atoi(argv[1]);
        v[1] = (int16_t)atoi(argv[2]);
        v[2] = (int16_t)atoi(argv[3]);
        v[3] = (int16_t)atoi(argv[4]);
    }
    else
    {
        printf("! 用法: %s 300  或者  %s 300,-300,0,0\r\n", argv[0], argv[0]);
        return;
    }

    if (is_pwm)
    {
        motor_send_pwm(v[0], v[1], v[2], v[3]);
        printf("-> $pwm:%d,%d,%d,%d#\r\n", v[0], v[1], v[2], v[3]);
    }
    else
    {
        motor_send_speed(v[0], v[1], v[2], v[3]);
        printf("-> $spd:%d,%d,%d,%d#\r\n", v[0], v[1], v[2], v[3]);
    }
}

static void exec_line(char *line)
{
    char *argv[6];
    int argc = 0;
    char *tok;
    char args[48];

    /* raw 要原样转发，不能按空格/逗号切分，先单独处理 */
    if (strncasecmp(line, "raw ", 4) == 0)
    {
        const char *frame = line + 4;
        while (*frame == ' ')
        {
            frame++;
        }
        if (*frame != '$')
        {
            printf("! 用法: raw $read_vol#\r\n");
            return;
        }
        motor_send_frame(frame);
        printf("-> %s\r\n", frame);
        return;
    }

    /* 按空格和逗号切分成若干参数 */
    for (tok = strtok(line, " ,\t"); (tok != NULL) && (argc < 6); tok = strtok(NULL, " ,\t"))
    {
        argv[argc++] = tok;
    }
    if (argc == 0)
    {
        return;
    }

    if ((strcasecmp(argv[0], "help") == 0) || (strcmp(argv[0], "?") == 0))
    {
        print_help();
    }
    else if (strcasecmp(argv[0], "spd") == 0)
    {
        drive_cmd(argc, argv, 0);
    }
    else if (strcasecmp(argv[0], "pwm") == 0)
    {
        drive_cmd(argc, argv, 1);
    }
    else if (strcasecmp(argv[0], "stop") == 0)
    {
        motor_send_speed_all(0);
        printf("-> $spd:0,0,0,0#\r\n");
    }
    else if (strcasecmp(argv[0], "upload") == 0)
    {
        if (argc >= 4)
        {
            motor_send_upload((uint8_t)atoi(argv[1]), (uint8_t)atoi(argv[2]), (uint8_t)atoi(argv[3]));
            printf("-> $upload:%d,%d,%d#\r\n", atoi(argv[1]) ? 1 : 0, atoi(argv[2]) ? 1 : 0, atoi(argv[3]) ? 1 : 0);
        }
        else
        {
            printf("! 用法: upload 0,1,1\r\n");
        }
    }
    else if (strcasecmp(argv[0], "get") == 0)
    {
        print_feedback();
    }
    else if (strcasecmp(argv[0], "mtype") == 0)
    {
        if (argc >= 2)
        {
            motor_send_mtype((uint8_t)atoi(argv[1]));
            printf("-> $mtype:%d#\r\n", atoi(argv[1]));
        }
    }
    else if (strcasecmp(argv[0], "mline") == 0)
    {
        if (argc >= 2)
        {
            motor_send_mline((uint16_t)atoi(argv[1]));
            printf("-> $mline:%d#\r\n", atoi(argv[1]));
        }
    }
    else if (strcasecmp(argv[0], "mphase") == 0)
    {
        if (argc >= 2)
        {
            motor_send_mphase((uint16_t)atoi(argv[1]));
            printf("-> $mphase:%d#\r\n", atoi(argv[1]));
        }
    }
    else if (strcasecmp(argv[0], "deadzone") == 0)
    {
        if (argc >= 2)
        {
            motor_send_deadzone((uint16_t)atoi(argv[1]));
            printf("-> $deadzone:%d#\r\n", atoi(argv[1]));
        }
    }
    else if (strcasecmp(argv[0], "wdiameter") == 0)
    {
        if (argc >= 2)
        {
            float mm = (float)atof(argv[1]);
            char s[16];
            motor_send_wdiameter(mm);
            motor_format_fixed(s, sizeof(s), mm, 3);
            printf("-> $wdiameter:%s#\r\n", s);
        }
    }
    else if (strcasecmp(argv[0], "pid") == 0)
    {
        if (argc >= 4)
        {
            float kp = (float)atof(argv[1]);
            float ki = (float)atof(argv[2]);
            float kd = (float)atof(argv[3]);
            char sp[16], si[16], sd[16];
            motor_send_pid(kp, ki, kd);
            motor_format_fixed(sp, sizeof(sp), kp, 3);
            motor_format_fixed(si, sizeof(si), ki, 3);
            motor_format_fixed(sd, sizeof(sd), kd, 3);
            printf("-> $MPID:%s,%s,%s#\r\n", sp, si, sd);
        }
        else
        {
            printf("! 用法: pid 0.5,0.01,0.1\r\n");
        }
    }
    else if ((strcasecmp(argv[0], "read_flash") == 0) || (strcasecmp(argv[0], "read_vol") == 0))
    {
        motor_send_cmd(argv[0], "");
        printf("-> $%s#\r\n", argv[0]);
    }
    else if (strcasecmp(argv[0], "reset") == 0)
    {
        motor_send_cmd("flash_reset", "");
        printf("-> $flash_reset#\r\n");
    }
    else
    {
        /* 其它没见过的命令：原样拼成 $name:参数# 发出去，方便试协议 */
        join_args(argc, argv, args, sizeof(args));
        motor_send_cmd(argv[0], args);
        printf("-> $%s%s%s#\r\n", argv[0], (args[0] != '\0') ? ":" : "", args);
    }
}

/* ---------------- 对外接口 ---------------- */

void pc_cmd_init(void)
{
    s_len = 0;
    s_last_report_ms = HAL_GetTick();
    s_last_frames = 0;
    s_link_lost = 0;
    s_have_prev = 0; /* 第一帧一定要打印出来 */

    printf("\r\n\r\n==== second_car : STM32 + 驱动板 串口调试 ====\r\n");
    printf("USART1(PA9/PA10) 115200 -> 电脑    USART2(PA2/PA3) 115200 -> 驱动板\r\n");
    printf("输入 help 查看命令\r\n");
}

void pc_cmd_poll(void)
{
    uint8_t ch;

    while (uart_io_read(UART_IO_PC, &ch))
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            if (s_len > 0)
            {
                s_line[s_len] = '\0';
                s_len = 0;
                exec_line(s_line);
            }
        }
        else if (s_len < (PC_LINE_MAX - 1u))
        {
            s_line[s_len++] = (char)ch;
        }
        else
        {
            s_len = 0; /* 太长，丢掉这一行 */
        }
    }
}
