/**
 * @file serial_print.h
 * @brief 基于 BSP USART 层的串口打印模块
 *
 * 使用 USART1 (921600-8-N-1 + DMA) 提供类似 printf 的串口输出功能。
 * 调用 SerialPrintInit() 注册到 BSP USART 层后，
 * 即可通过 SerialPrintf() / SerialPrint() 在任何 FreeRTOS 任务中打印调试信息。
 *
 * @note 发送采用 DMA 模式 (USART_TRANSFER_DMA)，
 *       连续调用时内部通过 USARTIsReady 轮询等待上一次传输完成，
 *       因此调用者无需关心 DMA 重入问题。
 *
 * @example
 *    // ---- 初始化 (在 RobotInit/BSPInit 之后调用一次) ----
 *    SerialPrintInit();
 *
 *    // ---- 任意任务中打印 ----
 *    SerialPrintf("Hello! tick = %lu\r\n", xTaskGetTickCount());
 *    SerialPrintf("motor: rpm=%d, temp=%.1f\r\n", rpm, temp);
 *    SerialPrint((uint8_t *)"OK\r\n", 4);
 */
#ifndef SERIAL_PRINT_H
#define SERIAL_PRINT_H

#include <stdint.h>

/**
 * @brief 初始化串口打印模块，注册 USART1 到 BSP USART 层
 */
void SerialPrintInit(void);

/**
 * @brief 通过串口发送格式化字符串（类似 printf）
 *
 * @note 内部使用 vsnprintf 格式化到静态缓冲区 (256 字节)，
 *       然后通过 USARTSend (DMA 模式) 发送；
 *       发送前会自动等待上一次 DMA 传输完成，安全可重入。
 *
 * @param fmt  格式化字符串
 * @param ...  可变参数列表
 */
void SerialPrintf(const char *fmt, ...);

/**
 * @brief 通过串口发送原始字节
 *
 * @param data 要发送的数据指针
 * @param len  数据长度 (字节)
 */
void SerialPrint(const uint8_t *data, uint16_t len);

#endif // SERIAL_PRINT_H
