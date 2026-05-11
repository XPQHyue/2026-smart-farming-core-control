#ifndef __BSP_VL53L0X_H
#define __BSP_VL53L0X_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////
// VL53L0X BSP层 - 简化接口
// 提供简洁的传感器初始化和读取接口
//////////////////////////////////////////////////////////////////////////////////

// 传感器数量定义
#define VL53L0X_MAX_SENSORS  4

// 传感器ID定义
#define VL53L0X_ID_1   0
#define VL53L0X_ID_2   1
#define VL53L0X_ID_3   2
#define VL53L0X_ID_4   3

// 调试开关
#define VL53L0X_DEBUG_OFF  0
#define VL53L0X_DEBUG_ON   1

// 启用掩码默认值(bit0~bit3对应传感器1~4)
#define VL53L0X_ACTIVE_MASK_ALL  0x0F

// 全局距离数据数组 (单位: mm)
// 使用方式: vl53l0x_distance[0] 是传感器1的距离值
extern volatile uint16_t vl53l0x_distance[VL53L0X_MAX_SENSORS];

// 传感器初始化状态
extern volatile uint8_t vl53l0x_ready[VL53L0X_MAX_SENSORS];

//////////////////////////////////////////////////////////////////////////////////
// 简化接口函数
//////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  设置启用传感器掩码
 * @param  mask: bit0~bit3对应传感器1~4, 1=启用, 0=禁用
 * @note   建议在VL53L0X_All_Init()之前调用
 * @usage   VL53L0X_Set_ActiveMask((1<<VL53L0X_ID_1) | (1<<VL53L0X_ID_3) | (1<<VL53L0X_ID_4));
 */
void VL53L0X_Set_ActiveMask(uint8_t mask);

/**
 * @brief  获取当前启用传感器掩码
 * @retval 当前掩码(bit0~bit3)
 */
uint8_t VL53L0X_Get_ActiveMask(void);

/**
 * @brief  设置非阻塞测量卡住阈值(按主循环调用次数计)
 * @param  limit: 阈值, 建议范围[10, 1000]
 */
void VL53L0X_Set_NonblockStuckLimit(uint16_t limit);

/**
 * @brief  获取非阻塞测量卡住阈值
 * @retval 当前阈值
 */
uint16_t VL53L0X_Get_NonblockStuckLimit(void);

/**
 * @brief  初始化单个VL53L0X传感器
 * @param  id: 传感器ID (0-3, 对应传感器1-4)
 * @param  debug: 调试开关 (0=关闭调试输出, 1=开启调试输出)
 * @retval 0=成功, 1=失败
 * @usage   VL53L0X_Sensor_Init(0, 1);  // 初始化传感器1, 开启调试
 */
uint8_t VL53L0X_Sensor_Init(uint8_t id, uint8_t debug);

/**
 * @brief  初始化所有VL53L0X传感器
 * @param  debug: 调试开关 (0=关闭调试输出, 1=开启调试输出)
 * @retval 初始化成功的传感器数量
 * @usage   VL53L0X_All_Init(0);  // 初始化所有传感器, 关闭调试
 *          VL53L0X_All_Init(1);  // 初始化所有传感器, 开启调试
 */
uint8_t VL53L0X_All_Init(uint8_t debug);

/**
 * @brief  读取单个传感器距离
 * @param  id: 传感器ID (0-3)
 * @retval 距离值(mm), 0xFFFF表示读取失败
 * @usage   uint16_t dist = VL53L0X_Read_Single(0);
 */
uint16_t VL53L0X_Read_Single(uint8_t id);

/**
 * @brief  读取所有传感器距离
 * @note   非阻塞轮询接口: 每次调用只推进测量状态机, 不会等待测量完成
 * @retval 无, 最新结果存储在vl53l0x_distance[]数组中
 * @usage   VL53L0X_Read_All();
 *          printf("Dist1=%d, Dist2=%d\n", vl53l0x_distance[0], vl53l0x_distance[1]);
 */
void VL53L0X_Read_All(void);

//////////////////////////////////////////////////////////////////////////////////
// 兼容旧接口的宏定义
//////////////////////////////////////////////////////////////////////////////////

// 简写形式 - 初始化单个传感器
#define VL53L0X_1()       VL53L0X_Sensor_Init(0, 0)
#define VL53L0X_2()       VL53L0X_Sensor_Init(1, 0)
#define VL53L0X_3()       VL53L0X_Sensor_Init(2, 0)
#define VL53L0X_4()       VL53L0X_Sensor_Init(3, 0)

// 简写形式 - 初始化所有传感器 (带调试参数)
#define VL53L0X_All(debug)  VL53L0X_All_Init(debug)

#endif /* __BSP_VL53L0X_H */
