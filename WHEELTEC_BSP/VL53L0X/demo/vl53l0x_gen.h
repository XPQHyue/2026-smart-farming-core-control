#ifndef __VL53L0X_GEN_H
#define __VL53L0X_GEN_H

#include "vl53l0x.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK MiniV3 STM32开发板
//VL53L0X-普通测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  

extern VL53L0X_RangingMeasurementData_t vl53l0x_data;
extern vu16 Distance_data;//保存测距数据

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf);
void vl53l0x_general_start(VL53L0X_Dev_t *dev,u8 mode);
void vl53l0x_general_test(VL53L0X_Dev_t *dev);


// 多传感器支持
extern VL53L0X_Dev_t vl53l0x_devices[VL53L0X_SENSOR_COUNT];  // 多传感器设备数组
extern VL53L0X_RangingMeasurementData_t vl53l0x_datas[VL53L0X_SENSOR_COUNT];  // 多传感器数据数组
extern vu16 Distance_datas[VL53L0X_SENSOR_COUNT];  // 多传感器距离数据

// 多传感器初始化函数
VL53L0X_Error vl53l0x_multi_init(void);  // 初始化所有传感器
VL53L0X_Error vl53l0x_multi_read(vu16 *distances);  // 读取所有传感器距离

#endif

