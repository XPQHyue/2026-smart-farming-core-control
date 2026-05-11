/**
 * @file    bsp_wit_uart4.h
 * @brief   维特智能陀螺仪 UART4 驱动 (PC10/PC11)
 * @note    适配 STM32F407
 */

#ifndef __BSP_WIT_UART4_H
#define __BSP_WIT_UART4_H

#include "sys.h"

/* UART4 初始化 */
void WitUart4Init(u32 bound);

/* UART4 发送数据 */
void Uart4Send(uint8_t *p_data, uint32_t uiSize);

#endif /* __BSP_WIT_UART4_H */
