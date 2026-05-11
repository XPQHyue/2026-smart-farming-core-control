/**
 * @file    bsp_wit_uart4.c
 * @brief   维特智能陀螺仪 UART4 驱动 (PC10/PC11)
 * @note    适配 STM32F407
 */

#include "bsp_wit_uart4.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "wit_c_sdk.h"

/**
 * @brief  UART4 初始化 (PC10=TX, PC11=RX)
 * @param  bound: 波特率
 */
void WitUart4Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 GPIOC 和 UART4 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* PC10=TX, PC11=RX 复用配置 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    /* GPIO 配置 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* USART 配置 */
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);

    /* 中断配置 */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能接收中断 */
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

/**
 * @brief  UART4 中断服务函数
 */
void UART4_IRQHandler(void)
{
    uint8_t ucTemp;
    
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        ucTemp = USART_ReceiveData(UART4);
        WitSerialDataIn(ucTemp);  /* 喂给维特SDK解析 */
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}

/**
 * @brief  UART4 发送数据
 * @param  p_data: 数据指针
 * @param  uiSize: 数据长度
 */
void Uart4Send(uint8_t *p_data, uint32_t uiSize)
{
    uint32_t i;
    
    for (i = 0; i < uiSize; i++)
    {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        USART_SendData(UART4, *p_data++);
    }
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}
