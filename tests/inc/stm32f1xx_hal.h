/* 主机测试用的假 HAL 头文件：让 motor_proto.c / pc_cmd.c 能在 PC 上编译 */
#ifndef __FAKE_STM32F1XX_HAL_H
#define __FAKE_STM32F1XX_HAL_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif
