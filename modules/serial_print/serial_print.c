/**
 * @file serial_print.c
 * @brief 涓插彛鎵撳嵃妯″潡瀹炵幇
 *
 * 渚濊禆 BSP USART 灞?(bsp_usart.c) 鍜?HAL USART 鍙ユ焺 (Src/usart.c)銆? * 浣跨敤 USART1 @ 921600锛孌MA 鍙戦€?+ FreeRTOS 浜掓枼閲忛槻閲嶅叆銆? */
#include "serial_print.h"

#include "bsp_usart.h"   /* USARTRegister, USARTSend, USARTIsReady ... */
#include "usart.h"       /* extern huart1 */
#include "bsp_log.h"     /* LOGINFO / LOGERROR */

#include <stdarg.h>      /* va_list, va_start, va_end */
#include "FreeRTOS.h"    /* FreeRTOS base types (BaseType_t, TickType_t, etc.) */
#include <stdio.h>       /* vsnprintf */
#include <string.h>      /* strlen */

#include "semphr.h"      /* FreeRTOS mutex */

/* ---- 甯搁噺 ---- */
#define PRINT_BUFFER_SIZE 256           /* 鏍煎紡鍖栫紦鍐插尯澶у皬锛屽彲鎸夐渶璋冩暣 */
#define USART_RX_BUF_SIZE  128          /* 鏈ā鍧楀彧鍙戜笉鏀讹紝浣?BSP 灞傛敞鍐屾椂闇€瑕?*/

/* ---- 闈欐€佸彉閲?---- */
static USARTInstance    *print_instance;                    /* BSP USART 瀹炰緥 */
static uint8_t           print_tx_buf[PRINT_BUFFER_SIZE];   /* 鍙戦€佺紦鍐插尯 (DMA 浣跨敤) */
static SemaphoreHandle_t print_mutex;                        /* 浜掓枼閲忥紝淇濇姢缂撳啿鍖轰笌 DMA 鍙戦€佸簭鍒?*/

/**
 * @brief 涓插彛鎺ユ敹鍥炶皟锛堟湰妯″潡鍙彂涓嶆敹锛岀暀绌哄嵆鍙級
 */
static void SerialPrintRxCallback(void)
{
    /* 涓嶅仛浠讳綍瑙ｆ瀽锛屼粎鐢ㄤ簬婊¤冻 USARTRegister 鐨勫洖璋冭姹?*/
}

/**
 * @brief 鍒濆鍖栦覆鍙ｆ墦鍗版ā鍧? *
 * 浣跨敤 USART1 (921600-8-N-1) 浣滀负鎵撳嵃绔彛銆? * 璇ヤ覆鍙ｅ凡鍦?MX_USART1_UART_Init() 涓敱 HAL 鍒濆鍖栵紝
 * 杩欓噷鍙渶璋冪敤 USARTRegister 灏嗗叾娉ㄥ唽鍒?BSP USART 绠＄悊灞傘€? */
void SerialPrintInit(void)
{
    USART_Init_Config_s conf;
    conf.usart_handle    = &huart1;              /* USART1, 921600-8-N-1 */
    conf.recv_buff_size  = USART_RX_BUF_SIZE;
    conf.module_callback = SerialPrintRxCallback; /* 鍙彂涓嶆敹锛屽洖璋冧负绌?*/

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
 * @brief 鏍煎紡鍖栧苟鍙戦€佸瓧绗︿覆
 *
 * 娴佺▼: 鍙栦簰鏂ラ噺 -> vsnprintf -> USARTIsReady 杞 -> USARTSend (DMA 妯″紡) -> 鏀句簰鏂ラ噺銆? * 浜掓枼閲忎繚鎶ゆ暣涓?"鏍煎紡鍖?绛夊緟-鍙戦€? 搴忓垪锛岀‘淇濆浠诲姟璋冪敤瀹夊叏銆? * 921600 娉㈢壒鐜囦笅 256 瀛楄妭绾?2.8ms 瀹屾垚,DMA 鏈熼棿 CPU 涓嶈闃诲銆? */
void SerialPrintf(const char *fmt, ...)
{
    if (print_instance == NULL || print_mutex == NULL) return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)print_tx_buf, PRINT_BUFFER_SIZE, fmt, args);
    va_end(args);

    /* vsnprintf 杩斿洖璐熷€艰〃绀烘牸寮忓寲閿欒 */
    if (len < 0)
    {
        xSemaphoreGive(print_mutex);
        return;
    }
    /* 濡傛灉瓒呭嚭缂撳啿锛屾埅鏂埌缂撳啿鍖烘湯灏?*/
    if (len >= PRINT_BUFFER_SIZE) len = PRINT_BUFFER_SIZE - 1;

    /* 绛夊緟涓婁竴娆?DMA 浼犺緭瀹屾垚,闃叉闈欐€佺紦鍐插尯琚鐩?*/
    while (!USARTIsReady(print_instance))
    {
        /* busy-wait: 921600 娉㈢壒鐜囦笅鏈€鍧忕瓑寰?< 3ms */
    }

    USARTSend(print_instance, print_tx_buf, (uint16_t)len, USART_TRANSFER_DMA);

    xSemaphoreGive(print_mutex);
}

/**
 * @brief 鍙戦€佸師濮嬪瓧鑺傛暟鎹? *
 * @note 璋冪敤鑰呴渶淇濊瘉 data 鎸囧悜鐨勫唴瀛樺湪 DMA 浼犺緭鏈熼棿鏈夋晥銆? */
void SerialPrint(const uint8_t *data, uint16_t len)
{
    if (print_instance == NULL || data == NULL || len == 0 || print_mutex == NULL) return;

    xSemaphoreTake(print_mutex, portMAX_DELAY);

    while (!USARTIsReady(print_instance)) {}  /* 闃查噸鍏?*/
    USARTSend(print_instance, (uint8_t *)data, len, USART_TRANSFER_DMA);

    xSemaphoreGive(print_mutex);
}
