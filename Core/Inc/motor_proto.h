/**
  ******************************************************************************
  * @file    motor_proto.h
  * @brief   驱动板串口通信协议（ASCII 帧：$命令:参数,参数,...#）
  *
  * 一帧长这样（全部是可见字符，串口助手里能直接看懂）：
  *      '$' + 命令名 + ':' + 参数 + '#'
  *
  * 下发给驱动板：
  *      $spd:300,-300,0,0#     闭环速度控制，范围 -1000~1000（正负号决定正反转）
  *      $pwm:1200,-1200,0,0#   开环 PWM，范围 -3600~3600
  *      $upload:0,1,1#         打开回传：总编码器 / 10ms增量 / 速度
  *
  * 驱动板回传（STM32 解析后放在 g_motor 里）：
  *      $MAll:1,2,3,4#         四路累计编码器数
  *      $MTEP:1,2,3,4#         四路 10ms 内的编码器增量
  *      $MSPD:120.5,120.0,0,0# 四路速度，单位 mm/s
  ******************************************************************************
  */
#ifndef __MOTOR_PROTO_H
#define __MOTOR_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_NUM 4 /* 驱动板有 4 路电机接口 M1~M4 */

/* 驱动板回传的编码器数据 */
typedef struct
{
    int32_t total[MOTOR_NUM]; /* 累计编码器数（$MAll） */
    int16_t delta[MOTOR_NUM]; /* 10ms 内的编码器增量（$MTEP） */
    float speed[MOTOR_NUM];   /* 速度 mm/s（$MSPD） */
    uint32_t frames;          /* 一共收到多少帧有效回传 */
    uint32_t last_ms;         /* 最近一次收到回传的时刻 */
} motor_feedback_t;

extern motor_feedback_t g_motor;

/* 初始化（清状态 + 打开回传开关） */
void motor_proto_init(void);

/* 主循环里一直调用：把 USART2 收到的字节拼成帧并解析 */
void motor_proto_poll(void);

/* -------- 下发命令 -------- */

/* 闭环速度控制：正数正转、负数反转，范围 -1000~1000 */
void motor_send_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void motor_send_speed_all(int16_t spd);

/* 开环 PWM：范围 -3600~3600 */
void motor_send_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/* 回传开关：a=总编码器 b=10ms增量 c=速度，1 打开 0 关闭 */
void motor_send_upload(uint8_t total_en, uint8_t delta_en, uint8_t speed_en);

/* 配置类命令 */
void motor_send_mtype(uint8_t type);   /* 1:520 2:310 3:TT带编码器 4:TT */
void motor_send_mline(uint16_t line);  /* 编码器线数 */
void motor_send_mphase(uint16_t ratio);/* 减速比 */
void motor_send_deadzone(uint16_t dz); /* PWM 死区 */
void motor_send_wdiameter(float mm);   /* 轮子直径 mm */
void motor_send_pid(float p, float i, float d);

/* 通用命令：把 name 和 args 原样拼成 "$name:args#"，例如 ("flash_reset","") */
void motor_send_cmd(const char *name, const char *args);

/* 原样发送一帧，例如 "$read_vol#"（自己带 $ 和 #） */
void motor_send_frame(const char *frame);

/* 把浮点数写成定点字符串（dec 取 2 或 3）：65.0f -> "65.000"
   因为 newlib-nano 的 printf 不支持 %f，打印/拼接都用这个 */
void motor_format_fixed(char *out, uint16_t size, float v, uint8_t dec);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PROTO_H */
