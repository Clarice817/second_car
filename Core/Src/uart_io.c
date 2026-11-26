/**
  ******************************************************************************
  * @file    uart_io.c
  * @brief   两路串口收发实现
  *
  * 思路（最简单可靠的做法）：
  *   1. 每收到 1 个字节进一次中断，把字节丢进“环形缓冲区”；
  *   2. 主循环里用 uart_io_read() 慢慢取，不阻塞、不丢数据；
  *   3. 发送用阻塞式 HAL_UART_Transmit，一帧只有几十字节，够用。
  ******************************************************************************  
  */
#include <string.h>

#include "uart_io.h"
#include "usart.h" /* 里面声明了 huart1 / huart2 */
#include "stm32f1xx_hal.h"

#define RB_SIZE 256 /* 每路串口环形缓冲区大小，必须是 2 的幂或任意都可以（这里用取模） */

typedef struct
{
    uint8_t buf[RB_SIZE];
    volatile uint16_t head; /* 写指针（中断里写） */
    volatile uint16_t tail; /* 读指针（主循环里写） */
} ring_t;

static ring_t s_rb_pc;    /* USART1 */
static ring_t s_rb_motor; /* USART2 */

static uint8_t s_byte_pc;    /* 中断接收用的 1 字节临时变量 */
static uint8_t s_byte_motor;

/* ---------------- 环形缓冲区 ---------------- */

static void rb_put(ring_t *rb, uint8_t b)
{
    uint16_t next = (uint16_t)((rb->head + 1u) % RB_SIZE);
    if (next != rb->tail) /* 缓冲区满则丢弃新字节，不覆盖旧数据 */
    {
        rb->buf[rb->head] = b;
        rb->head = next;
    }
}

static uint8_t rb_get(ring_t *rb, uint8_t *b)
{
    if (rb->head == rb->tail)
    {
        return 0;
    }
    *b = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1u) % RB_SIZE);
    return 1;
}

/* ---------------- 初始化 ---------------- */

void uart_io_init(void)
{
    /* USART1 的中断在 CubeMX 里已经打开了（usart.c 的 MspInit 里）。
       USART2 的中断 CubeMX 没勾选，这里手动打开。
       如果你的 CubeMX 里勾选了 “USART2 global interrupt”，
       那么 stm32f1xx_it.c 会自动生成 USART2_IRQHandler，
       请把本文件最下面的同名函数删掉，否则会重复定义。 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_UART_Receive_IT(&huart1, &s_byte_pc, 1);
    HAL_UART_Receive_IT(&huart2, &s_byte_motor, 1);
}

/* ---------------- 接收：中断回调 ---------------- */

/* 每收完 1 个字节 HAL 都会调用这里，存起来并重新挂上接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        rb_put(&s_rb_pc, s_byte_pc);
        HAL_UART_Receive_IT(&huart1, &s_byte_pc, 1);
    }
    else if (huart->Instance == USART2)
    {
        rb_put(&s_rb_motor, s_byte_motor);
        HAL_UART_Receive_IT(&huart2, &s_byte_motor, 1);
    }
}

/* USART2 中断服务函数：交给 HAL 处理（清标志、进回调） */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* ---------------- 发送 ---------------- */

void uart_io_write(uart_io_id_t id, const void *data, uint16_t len)
{
    UART_HandleTypeDef *h = (id == UART_IO_PC) ? &huart1 : &huart2;
    if (len == 0u)
    {
        return;
    }
    HAL_UART_Transmit(h, (uint8_t *)data, len, 100); /* 超时 100ms，防止死等 */
}

void uart_io_puts(uart_io_id_t id, const char *str)
{
    uart_io_write(id, str, (uint16_t)strlen(str));
}

uint8_t uart_io_read(uart_io_id_t id, uint8_t *byte)
{
    return rb_get((id == UART_IO_PC) ? &s_rb_pc : &s_rb_motor, byte);
}

/* ---------------- printf 重定向到 USART1 ---------------- */

/* syscalls.c 里的 _write 是 weak 的，这里提供更强的版本，
   于是 printf(...) 的数据就会从 USART1 发到电脑串口助手 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    uart_io_write(UART_IO_PC, ptr, (uint16_t)len);
    return len;
}

/* 有些库会用 __io_putchar，一并提供，保证一定能链接通过 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    uart_io_write(UART_IO_PC, &c, 1);
    return ch;
}
