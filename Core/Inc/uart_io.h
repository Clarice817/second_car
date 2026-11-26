/**
  ******************************************************************************
  * @file    uart_io.h
  * @brief   两路串口的收发封装（中断收 + 阻塞发 + 环形缓冲区）
  *
  *   USART1 : 接电脑（调试 / 发命令），printf 也走这里
  *   USART2 : 接电机驱动板（发 $spd:..# 命令，收回传的编码器数据）
  ******************************************************************************
  */
#ifndef __UART_IO_H
#define __UART_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 两路串口的编号，调用函数时用它区分 */
typedef enum
{
    UART_IO_PC = 0,   /* USART1 -> 电脑 */
    UART_IO_MOTOR = 1 /* USART2 -> 驱动板 */
} uart_io_id_t;

/* 初始化：打开两路串口的“单字节接收中断” */
void uart_io_init(void);

/* 发数据（阻塞式，发完才返回）。数据量很小时用这个最简单 */
void uart_io_write(uart_io_id_t id, const void *data, uint16_t len);

/* 发字符串 */
void uart_io_puts(uart_io_id_t id, const char *str);

/* 从缓冲区取一个字节：返回 1=取到了，0=当前没有数据（非阻塞） */
uint8_t uart_io_read(uart_io_id_t id, uint8_t *byte);

#ifdef __cplusplus
}
#endif

#endif /* __UART_IO_H */
