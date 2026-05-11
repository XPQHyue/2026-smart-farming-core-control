/**
 * @file    wit_sdk_example.h
 * @brief   维特智能陀螺仪 SDK 使用示例
 * @note    将此代码片段添加到你的 main.c 中使用
 * 
 * 使用步骤：
 * 1. 在 main.c 开头添加: #include "wit_c_sdk.h" 和 #include "bsp_wit_uart4.h"
 * 2. 在 main() 开头调用 Wit_Init() 进行初始化
 * 3. 在主循环中调用 Wit_Process() 读取数据
 * 4. 使用 Wit_GetAngle(), Wit_GetGyro(), Wit_GetAcc() 获取数据
 */

#ifndef __WIT_SDK_EXAMPLE_H
#define __WIT_SDK_EXAMPLE_H

/* ============================================
   在 main.c 开头添加以下代码
   ============================================ */

#include "wit_c_sdk.h"
#include "bsp_wit_uart4.h"

/* 数据更新标志 */
#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define ANGLE_UPDATE    0x04
#define MAG_UPDATE      0x08

static volatile char s_cDataUpdate = 0;

/* 波特率表 */
static const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

/* ============================================
   以下函数添加到 main.c 中
   ============================================ */

/**
 * @brief  陀螺仪数据发送回调
 */
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    Uart4Send(p_data, uiSize);
}

/**
 * @brief  延时回调
 */
static void Delayms(uint16_t ucMs)
{
    delay_ms(ucMs);
}

/**
 * @brief  数据更新回调
 */
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for (i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                break;
        }
        uiReg++;
    }
}

/**
 * @brief  自动扫描传感器波特率
 */
static void AutoScanSensor(void)
{
    int i, iRetry;
    
    for (i = 1; i < 10; i++)
    {
        WitUart4Init(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            delay_ms(100);
            if (s_cDataUpdate != 0)
            {
                printf("WIT: %d baud find sensor\r\n", c_uiBaud[i]);
                return;
            }
            iRetry--;
        } while (iRetry);
    }
    printf("WIT: can not find sensor\r\n");
}

/**
 * @brief  维特陀螺仪初始化 - 在 main() 开头调用
 */
void Wit_Init(void)
{
    /* 初始化 SDK */
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    
    /* 注册回调函数 */
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    
    /* 自动扫描波特率 */
    AutoScanSensor();
}

/**
 * @brief  获取角度数据 (度)
 * @param  index: 0=Roll, 1=Pitch, 2=Yaw
 * @return 角度值 (度)
 */
float Wit_GetAngle(int index)
{
    return sReg[Roll + index] / 32768.0f * 180.0f;
}

/**
 * @brief  获取角速度数据 (度/秒)
 * @param  index: 0=GX, 1=GY, 2=GZ
 * @return 角速度值 (度/秒)
 */
float Wit_GetGyro(int index)
{
    return sReg[GX + index] / 32768.0f * 2000.0f;
}

/**
 * @brief  获取加速度数据 (g)
 * @param  index: 0=AX, 1=AY, 2=AZ
 * @return 加速度值 (g)
 */
float Wit_GetAcc(int index)
{
    return sReg[AX + index] / 32768.0f * 16.0f;
}

/**
 * @brief  处理陀螺仪数据 - 在主循环中调用
 * @note   可以在此函数中添加数据处理逻辑
 */
void Wit_Process(void)
{
    if (s_cDataUpdate)
    {
        if (s_cDataUpdate & ANGLE_UPDATE)
        {
            /* 读取角度数据 */
            float yaw = Wit_GetAngle(2);    /* Yaw 角度 */
            float pitch = Wit_GetAngle(1);  /* Pitch 角度 */
            float roll = Wit_GetAngle(0);   /* Roll 角度 */
            
            /* 在这里使用角度数据... */
            printf("Angle: Yaw=%.2f, Pitch=%.2f, Roll=%.2f\r\n", yaw, pitch, roll);
            
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        
        if (s_cDataUpdate & GYRO_UPDATE)
        {
            /* 读取角速度数据 */
            float gz = Wit_GetGyro(2);  /* Z轴角速度 */
            
            /* 在这里使用角速度数据... */
            
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
        
        if (s_cDataUpdate & ACC_UPDATE)
        {
            /* 读取加速度数据 */
            float az = Wit_GetAcc(2);  /* Z轴加速度 */
            
            /* 在这里使用加速度数据... */
            
            s_cDataUpdate &= ~ACC_UPDATE;
        }
    }
}

/* ============================================
   在 main.c 中的使用示例:
   ============================================ */

#if 0  // 示例代码，仅供参考

int main(void)
{
    /* 系统初始化 */
    delay_init(168);
    
    /* 初始化维特陀螺仪 (PC10/PC11) */
    Wit_Init();
    
    while (1)
    {
        /* 处理陀螺仪数据 */
        Wit_Process();
        
        /* 其他任务... */
    }
}

#endif

#endif /* __WIT_SDK_EXAMPLE_H */
