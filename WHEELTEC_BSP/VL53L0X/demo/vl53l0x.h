#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "sys.h"
#include "delay.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

////////////////////////////////////////////////////////////////////////////////// 	
//本程序只供学习使用，未经作者许可，不得用于其它任何用途。
//ALIENTEK MiniV3 STM32开发板
//VL53L0X-功能测量 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved						  
////////////////////////////////////////////////////////////////////////////////// 

// 多传感器配置 - 必须在包含vl53l0x_gen.h之前定义
#define VL53L0X_SENSOR_COUNT  4

// 多传感器XSHUT引脚定义
#define VL53L0X_XSHUT_1  PAout(6)  // 传感器1 - PA6
#define VL53L0X_XSHUT_2  PAout(7)  // 传感器2 - PA7
#define VL53L0X_XSHUT_3  PCout(1)  // 传感器3 - PC1  原PA8，蜂鸣器使用，故修改
#define VL53L0X_XSHUT_4  PCout(0)  // 传感器4 - PC0

// 多传感器I2C地址定义
#define VL53L0X_ADDR_1   0x54      // 传感器1地址
#define VL53L0X_ADDR_2   0x56      // 传感器2地址
#define VL53L0X_ADDR_3   0x58      // 传感器3地址
#define VL53L0X_ADDR_4   0x5A      // 传感器4地址

#include "vl53l0x_gen.h"
#include "vl53l0x_cali.h"
#include "vl53l0x_it.h"

//VL53L0X传感器上电后的默认IIC地址为0X52(不算最低位)
#define VL53L0X_Addr 0x52

//使用2.8V IO电平模式
#define USE_I2C_2V8  1

//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

//vl53l0x模式设置参数结构体
typedef __packed struct
{
	FixPoint1616_t signalLimit;    //Signal速率极限值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限极限值
	uint32_t timingBudget;         //测量时间预算
	uint8_t preRangeVcselPeriod ;  //VCSEL预脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL最终脉冲范围
	
}mode_data;


extern mode_data Mode_data[];
extern uint8_t AjustOK;

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);//初始化vl53l0x
void print_pal_error(VL53L0X_Error Status);//错误信息打印
void mode_string(u8 mode,char *buf);//模式字符串显示
void vl53l0x_test(void);//vl53l0x测试
void vl53l0x_reset(VL53L0X_Dev_t *dev);//vl53l0x复位

void vl53l0x_info(void);//获取vl53l0x设备ID信息
void One_measurement(u8 mode);//获取一次测量数据
#endif
