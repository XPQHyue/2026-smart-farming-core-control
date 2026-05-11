/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 2026国际大学生农业装备竞赛B类
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * @project        : 大学生农业装备竞赛B类智能小车
  * @authors        : XPQH CQX LSL WFS LQQ
  * @date           : 2026.3.22
  * @version        : V1.0 
  * 
  * @description    : STM32F407VET6的智能小车
  *                   
  *
  * @hardware       : BSP层VL53L0X多传感器模块
  *
  * @update_log     : V1.0版本主要更新内容：
  *					
  *
  * @control_system : 当前支持4路VL53L0X传感器的初始化和读取，适用于多传感器环境

  ******************************************************************************
  */

#include "bsp_vl53l0x.h"
#include "vl53l0x.h"
#include "vl53l0x_gen.h"
#include "delay.h"
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////////////
// VL53L0X BSP层 - 简化接口实现
//////////////////////////////////////////////////////////////////////////////////

// 全局距离数据
volatile uint16_t vl53l0x_distance[VL53L0X_MAX_SENSORS] = {0};
volatile uint8_t vl53l0x_ready[VL53L0X_MAX_SENSORS] = {0};

// 传感器地址配置
static const uint8_t vl53l0x_addresses[VL53L0X_MAX_SENSORS] = {
    VL53L0X_ADDR_1,  // 0x54
    VL53L0X_ADDR_2,  // 0x56
    VL53L0X_ADDR_3,  // 0x58
    VL53L0X_ADDR_4   // 0x5A
};

// 默认工作模式
static uint8_t vl53l0x_mode = HIGH_SPEED;

// 启用掩码(bit0~bit3对应传感器1~4)
static uint8_t vl53l0x_active_mask = VL53L0X_ACTIVE_MASK_ALL;

// 非阻塞读取状态
#define VL53L0X_NONBLOCK_STUCK_LIMIT_MIN      10
#define VL53L0X_NONBLOCK_STUCK_LIMIT_MAX      1000
#define VL53L0X_NONBLOCK_STUCK_LIMIT_DEFAULT  50
static uint16_t vl53l0x_nonblock_stuck_limit = VL53L0X_NONBLOCK_STUCK_LIMIT_DEFAULT;
static uint8_t vl53l0x_measure_active[VL53L0X_MAX_SENSORS] = {0};
static uint16_t vl53l0x_wait_count[VL53L0X_MAX_SENSORS] = {0};

// 串扰抑制与有效帧过滤参数
#define VL53L0X_RANGE_VALID_STATUS             0U
#define VL53L0X_RAW_MIN_MM                     20U
#define VL53L0X_RAW_OUT_OF_RANGE_MM            8000U
#define VL53L0X_SUSPECT_NEAR_MM                150U
#define VL53L0X_EMPTY_SCENE_CONFIRM_FRAMES     3U

static uint8_t vl53l0x_empty_scene_latch[VL53L0X_MAX_SENSORS] = {0};
static uint8_t vl53l0x_near_confirm_count[VL53L0X_MAX_SENSORS] = {0};

// 判断传感器是否启用
static uint8_t bsp_sensor_enabled(uint8_t id)
{
    if(id >= VL53L0X_MAX_SENSORS)
    {
        return 0;
    }
    return (uint8_t)((vl53l0x_active_mask & (1U << id)) != 0U);
}

//////////////////////////////////////////////////////////////////////////////////
// 内部函数
//////////////////////////////////////////////////////////////////////////////////

// 设置XSHUT引脚状态
static void bsp_xshut_set(uint8_t id, uint8_t state)
{
    switch(id)
    {
        case 0: VL53L0X_XSHUT_1 = state; break;
        case 1: VL53L0X_XSHUT_2 = state; break;
        case 2: VL53L0X_XSHUT_3 = state; break;
        case 3: VL53L0X_XSHUT_4 = state; break;
        default: break;
    }
}

// 初始化XSHUT引脚GPIO
static void bsp_xshut_gpio_init(void)
{
    static uint8_t gpio_inited = 0;
    
    if(gpio_inited) return;
    gpio_inited = 1;
    
    // 使能GPIO时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // 配置PA6, PA7
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置PC0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 复位非阻塞测量状态
static void bsp_measure_state_reset(void)
{
    for(uint8_t i = 0; i < VL53L0X_MAX_SENSORS; i++)
    {
        vl53l0x_measure_active[i] = 0;
        vl53l0x_wait_count[i] = 0;
        vl53l0x_empty_scene_latch[i] = 0;
        vl53l0x_near_confirm_count[i] = 0;
    }
}

// 过滤无效帧与空旷场景下的瞬时近距离误报
static uint16_t bsp_filter_range_measurement(uint8_t id, uint16_t raw_mm, uint8_t range_status)
{
    if(id >= VL53L0X_MAX_SENSORS)
    {
        return 0xFFFF;
    }

    if((range_status != VL53L0X_RANGE_VALID_STATUS) ||
       (raw_mm < VL53L0X_RAW_MIN_MM) ||
       (raw_mm >= VL53L0X_RAW_OUT_OF_RANGE_MM))
    {
        vl53l0x_empty_scene_latch[id] = 1;
        vl53l0x_near_confirm_count[id] = 0;
        return 0xFFFF;
    }

    if(vl53l0x_empty_scene_latch[id] && (raw_mm <= VL53L0X_SUSPECT_NEAR_MM))
    {
        if(vl53l0x_near_confirm_count[id] < 0xFF)
        {
            vl53l0x_near_confirm_count[id]++;
        }

        if(vl53l0x_near_confirm_count[id] < VL53L0X_EMPTY_SCENE_CONFIRM_FRAMES)
        {
            return 0xFFFF;
        }
    }

    vl53l0x_empty_scene_latch[id] = 0;
    vl53l0x_near_confirm_count[id] = 0;
    return raw_mm;
}

void VL53L0X_Set_ActiveMask(uint8_t mask)
{
    vl53l0x_active_mask = (uint8_t)(mask & VL53L0X_ACTIVE_MASK_ALL);
}

uint8_t VL53L0X_Get_ActiveMask(void)
{
    return vl53l0x_active_mask;
}

void VL53L0X_Set_NonblockStuckLimit(uint16_t limit)
{
    if(limit < VL53L0X_NONBLOCK_STUCK_LIMIT_MIN)
    {
        limit = VL53L0X_NONBLOCK_STUCK_LIMIT_MIN;
    }
    else if(limit > VL53L0X_NONBLOCK_STUCK_LIMIT_MAX)
    {
        limit = VL53L0X_NONBLOCK_STUCK_LIMIT_MAX;
    }

    vl53l0x_nonblock_stuck_limit = limit;
}

uint16_t VL53L0X_Get_NonblockStuckLimit(void)
{
    return vl53l0x_nonblock_stuck_limit;
}

//////////////////////////////////////////////////////////////////////////////////
// 公开接口实现
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  初始化单个VL53L0X传感器
 */
uint8_t VL53L0X_Sensor_Init(uint8_t id, uint8_t debug)
{
    VL53L0X_Error status;

    // 初始化GPIO
    bsp_xshut_gpio_init();
    
    if(id >= VL53L0X_MAX_SENSORS)
    {
        if(debug) printf("VL53L0X: Invalid sensor ID %d\r\n", id);
        return 1;
    }

    if(!bsp_sensor_enabled(id))
    {
        bsp_xshut_set(id, 0);
        vl53l0x_ready[id] = 0;
        vl53l0x_distance[id] = 0xFFFF;
        if(debug) printf("VL53L0X #%d disabled by mask\r\n", id + 1);
        return 1;
    }
    
    bsp_measure_state_reset();
    
    // 先将所有传感器复位
    for(uint8_t i = 0; i < VL53L0X_MAX_SENSORS; i++)
    {
        bsp_xshut_set(i, 0);
    }
    delay_ms(10);
    
    // 使能目标传感器
    bsp_xshut_set(id, 1);
    delay_ms(10);
    
    // 初始化传感器设备结构
    vl53l0x_devices[id].I2cDevAddr = VL53L0X_Addr;
    vl53l0x_devices[id].comms_type = 1;
    
    // 执行初始化
    status = vl53l0x_init(&vl53l0x_devices[id]);
    if(status != VL53L0X_ERROR_NONE)
    {
        if(debug) printf("VL53L0X #%d init failed!\r\n", id + 1);
        vl53l0x_ready[id] = 0;
        return 1;
    }
    
    // 设置工作模式
    status = vl53l0x_set_mode(&vl53l0x_devices[id], vl53l0x_mode);
    if(status != VL53L0X_ERROR_NONE)
    {
        if(debug) printf("VL53L0X #%d set mode failed!\r\n", id + 1);
        vl53l0x_ready[id] = 0;
        return 1;
    }
    
    // 修改I2C地址
    status = VL53L0X_SetDeviceAddress(&vl53l0x_devices[id], vl53l0x_addresses[id]);
    if(status != VL53L0X_ERROR_NONE)
    {
        if(debug) printf("VL53L0X #%d set address failed!\r\n", id + 1);
        vl53l0x_ready[id] = 0;
        return 1;
    }
    vl53l0x_devices[id].I2cDevAddr = vl53l0x_addresses[id];
    
    vl53l0x_ready[id] = 1;
    if(debug) printf("VL53L0X #%d init OK, Addr: 0x%02X\r\n", id + 1, vl53l0x_addresses[id]);
    
    return 0;
}

/**
 * @brief  初始化所有VL53L0X传感器
 */
uint8_t VL53L0X_All_Init(uint8_t debug)
{
    uint8_t success_count = 0;
    
    if(debug) printf("\r\n=== VL53L0X Multi-Sensor Init ===\r\n");
    
    // 初始化GPIO
    bsp_xshut_gpio_init();
    bsp_measure_state_reset();
    
    // 将所有传感器复位
    for(uint8_t i = 0; i < VL53L0X_MAX_SENSORS; i++)
    {
        bsp_xshut_set(i, 0);
    }
    delay_ms(10);
    
    // 逐个初始化传感器
    for(uint8_t i = 0; i < VL53L0X_MAX_SENSORS; i++)
    {
        if(!bsp_sensor_enabled(i))
        {
            bsp_xshut_set(i, 0);
            vl53l0x_ready[i] = 0;
            vl53l0x_distance[i] = 0xFFFF;
            continue;
        }

        // 使能当前传感器
        bsp_xshut_set(i, 1);
        delay_ms(10);
        
        // 初始化设备结构
        vl53l0x_devices[i].I2cDevAddr = VL53L0X_Addr;
        vl53l0x_devices[i].comms_type = 1;
        
        // 执行初始化
        VL53L0X_Error status = vl53l0x_init(&vl53l0x_devices[i]);
        if(status != VL53L0X_ERROR_NONE)
        {
            if(debug) printf("VL53L0X #%d init failed!\r\n", i + 1);
            vl53l0x_ready[i] = 0;
            continue;
        }
        
        // 设置工作模式
        status = vl53l0x_set_mode(&vl53l0x_devices[i], vl53l0x_mode);
        if(status != VL53L0X_ERROR_NONE)
        {
            if(debug) printf("VL53L0X #%d set mode failed!\r\n", i + 1);
            vl53l0x_ready[i] = 0;
            continue;
        }
        
        // 修改I2C地址
        status = VL53L0X_SetDeviceAddress(&vl53l0x_devices[i], vl53l0x_addresses[i]);
        if(status != VL53L0X_ERROR_NONE)
        {
            if(debug) printf("VL53L0X #%d set address failed!\r\n", i + 1);
            vl53l0x_ready[i] = 0;
            continue;
        }
        vl53l0x_devices[i].I2cDevAddr = vl53l0x_addresses[i];
        
        vl53l0x_ready[i] = 1;
        success_count++;
        if(debug) printf("VL53L0X #%d init OK, Addr: 0x%02X\r\n", i + 1, vl53l0x_addresses[i]);
    }
    
    if(debug) 
    {
        printf("=== Init Complete: %d/%d sensors ready ===\r\n\r\n", success_count, VL53L0X_MAX_SENSORS);
    }
    
    return success_count;
}

/**
 * @brief  读取单个传感器距离
 */
uint16_t VL53L0X_Read_Single(uint8_t id)
{
    if(id >= VL53L0X_MAX_SENSORS || !bsp_sensor_enabled(id) || !vl53l0x_ready[id])
    {
        return 0xFFFF;
    }
    
    VL53L0X_Error status;
    
    status = VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_devices[id], &vl53l0x_datas[id]);
    if(status == VL53L0X_ERROR_NONE)
    {
        vl53l0x_distance[id] = bsp_filter_range_measurement(id,
                                                            vl53l0x_datas[id].RangeMilliMeter,
                                                            vl53l0x_datas[id].RangeStatus);
        return vl53l0x_distance[id];
    }
    
    return 0xFFFF;
}

/**
 * @brief  读取所有传感器距离
 */
void VL53L0X_Read_All(void)
{
    for(uint8_t i = 0; i < VL53L0X_MAX_SENSORS; i++)
    {
        if(!bsp_sensor_enabled(i) || !vl53l0x_ready[i])
        {
            vl53l0x_distance[i] = 0xFFFF;
            vl53l0x_measure_active[i] = 0;
            vl53l0x_wait_count[i] = 0;
            vl53l0x_empty_scene_latch[i] = 1;
            vl53l0x_near_confirm_count[i] = 0;
            continue;
        }

        // 未启动则发起单次测量，立即返回给主循环，不等待结果
        if(!vl53l0x_measure_active[i])
        {
            VL53L0X_Error status = VL53L0X_SetDeviceMode(&vl53l0x_devices[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);
            if(status == VL53L0X_ERROR_NONE)
            {
                status = VL53L0X_StartMeasurement(&vl53l0x_devices[i]);
            }

            if(status == VL53L0X_ERROR_NONE)
            {
                vl53l0x_measure_active[i] = 1;
                vl53l0x_wait_count[i] = 0;
            }
            else
            {
                vl53l0x_distance[i] = 0xFFFF;
                vl53l0x_measure_active[i] = 0;
                vl53l0x_wait_count[i] = 0;
                vl53l0x_empty_scene_latch[i] = 1;
                vl53l0x_near_confirm_count[i] = 0;
            }

            continue;
        }

        // 已启动则只检查就绪状态，不做阻塞等待
        uint8_t data_ready = 0;
        VL53L0X_Error status = VL53L0X_GetMeasurementDataReady(&vl53l0x_devices[i], &data_ready);
        if(status != VL53L0X_ERROR_NONE)
        {
            vl53l0x_distance[i] = 0xFFFF;
            vl53l0x_measure_active[i] = 0;
            vl53l0x_wait_count[i] = 0;
            vl53l0x_empty_scene_latch[i] = 1;
            vl53l0x_near_confirm_count[i] = 0;
            continue;
        }

        if(data_ready == 0)
        {
            if(vl53l0x_wait_count[i] < 0xFFFF)
            {
                vl53l0x_wait_count[i]++;
            }

            // 长时间未就绪则重置该路状态，防止异常卡死
            if(vl53l0x_wait_count[i] >= vl53l0x_nonblock_stuck_limit)
            {
                vl53l0x_distance[i] = 0xFFFF;
                vl53l0x_measure_active[i] = 0;
                vl53l0x_wait_count[i] = 0;
                vl53l0x_empty_scene_latch[i] = 1;
                vl53l0x_near_confirm_count[i] = 0;
            }
            continue;
        }

        status = VL53L0X_GetRangingMeasurementData(&vl53l0x_devices[i], &vl53l0x_datas[i]);
        if(status == VL53L0X_ERROR_NONE)
        {
            vl53l0x_distance[i] = bsp_filter_range_measurement(i,
                                                               vl53l0x_datas[i].RangeMilliMeter,
                                                               vl53l0x_datas[i].RangeStatus);
        }
        else
        {
            vl53l0x_distance[i] = 0xFFFF;
            vl53l0x_empty_scene_latch[i] = 1;
            vl53l0x_near_confirm_count[i] = 0;
        }

        (void)VL53L0X_ClearInterruptMask(&vl53l0x_devices[i], 0);
        vl53l0x_measure_active[i] = 0;
        vl53l0x_wait_count[i] = 0;
    }

}
