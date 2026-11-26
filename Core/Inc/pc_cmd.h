/**
  ******************************************************************************
  * @file    pc_cmd.h
  * @brief   电脑端（USART1）命令解析：把串口助手里敲的一行命令翻译成驱动板协议帧
  *
  * 用法：串口助手（115200 8N1）里敲命令 + 回车，例如
  *      spd 300        四路电机都正转 300
  *      spd -300       反转
  *      stop           停车
  *      help           看全部命令
  ******************************************************************************
  */
#ifndef __PC_CMD_H
#define __PC_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化：打印欢迎信息和命令表 */
void pc_cmd_init(void);

/* 主循环里一直调用：解析 USART1 收到的命令 */
void pc_cmd_poll(void);

/* 主循环里一直调用：按固定周期把编码器数据打印到电脑串口 */
void pc_report_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __PC_CMD_H */
