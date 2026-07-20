/**
 * @file serial_print.c
 * @brief 串口打印模块实现
 *
 * 依赖 BSP USART �?(bsp_usart.c) �?HAL USART 句柄 (Src/usart.c)�?
 * 使用 USART1 @ 921600，DMA 发�?+ FreeRTOS 互斥量防重入�?
 */
#include "serial_print.h"

#include "bsp_usart.h"   /* USARTRegister, USARTSend, USARTIsReady ... */
#include "usart.h"       /* extern huart1 */
#include "bsp_log.h"     /* LOGINFO / LOGERROR */
#include "FreeRTOS.h"    /* FreeRTOS base types (BaseType_t, TickType_t, etc.) */

#include <stdarg.h>      /* va_list, va_start, va_end */
#include <stdio.h>       /* vsnprintf */
#include <string.h>      /* strlen */

#include "semphr.h"      /* FreeRTOS mutex */

/* ---- 常量 ---- */
#define PRINT_BUFFER_SIZE 256           /* 格式化缓冲区大小，可按需调整 */
#define USART_RX_BUF_SIZE  128          /* 本模块只发不收，�?BSP 层注册时需�?*/

/* ---- 静态变�?---- */
static USARTInstance    *print_instance;                    /* BSP USART 实例 */
static uint8_t           print_tx_buf[PRINT_BUFFER_SIZE];   /* 发送缓冲区 (DMA 使用) */
static SemaphoreHandle_t print_mutex;                        /* 互斥量，保护缓冲区与 DMA 发送序�?*/

/**
 * @brief 串口接收回调（本模块只发不收，留空即可）
 */
static void SerialPrintRxCallback(void)
{
    /* 不做任何解析，仅用于满足 USARTRegister 的回调要�?*/
}

/**
 * @brief 初始化串口打印模�?
 *
 * 使用 USART1 (921600-8-N-1) 作为打印端口�?
 * 该串口已�?MX_USART1_UART_Init() 中由 HAL 初始化，
 * 这里只需调用 USARTRegister 将其注册�?BSP USART 管理层�?
 */
void SerialPrintInit(void)
{
    USART_Init_Config_s conf;
    conf.usart_handle    = &huart1;              /* USART1, 921600-8-N-1 */
    conf.recv_buff_size  = USART_RX_BUF_SIZE;
    conf.module_callback = SerialPrintRxCallback; /* 只发不收，回调为�?*/

    print_instance = USARTRegister(&conf);
    if (print_instance == NULL)
    {
        LOGERROR("[serial_print] USARTRegister failed!");
        return;
    }

    print_mutex = xSemaphoreCreateMutex();
    if (print_mutex == NULL)
    {
        LOGERROR("[serial_print] mutex create failed!");
        return;
    }

    LOGINFO("[serial_print] init done on USART1 @ 921600");
}

/**
 * @brief 格式化并发送字符串
 *
 * 流程: 取互斥量 -> vsnprintf -> USARTIsReady 轮询 -> USARTSend (DMA 模式) -> 放互斥量�?
 * 互斥量保护整�?"格式�?等待-发�? 序列，确保多任务调用安全�?
 * 921600 波特率下 256 字节�?2.8ms 完成,DMA 期间 CPU 不被阻塞�?
 */
void SerialPrintf(const char *fmt, ...)
{
    if (print_instance == NULL || print_mutex == NULL) return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)print_tx_buf, PRINT_BUFFER_SIZE, fmt, args);
    va_end(args);

    /* vsnprintf 返回负值表示格式化错误 */
    if (len < 0)
    {
        xSemaphoreGive(print_mutex);
        return;
    }
    /* 如果超出缓冲，截断到缓冲区末�?*/
    if (len >= PRINT_BUFFER_SIZE) len = PRINT_BUFFER_SIZE - 1;

    HAL_UART_Transmit(print_instance->usart_handle, print_tx_buf, (uint16_t)len, 100);

    xSemaphoreGive(print_mutex);
}

/**
 * @brief 发送原始字节数�?
 *
 * @note 调用者需保证 data 指向的内存在 DMA 传输期间有效�?
 */
void SerialPrint(const uint8_t *data, uint16_t len)
{
    if (print_instance == NULL || data == NULL || len == 0 || print_mutex == NULL) return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    HAL_UART_Transmit(print_instance->usart_handle, (uint8_t *)data, len, 100);

    xSemaphoreGive(print_mutex);
}
