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
  *
  * 轮子布局 (俯视图):
  *       Vx^+ (前)
  *       |
  *   B   |   C     
  *       |
  * ------+------->Vy+ (右)
  *       |
  *	   主控位置	
  *   A   |   D
  *       |
  * @project        : 大学生农业装备竞赛B类智能小车
  * @authors        : XPQH CQX LSL WFS LQQ
  * @date           : 2026.3.22
  * @version        : V1.0 
  * 
  * @description    : STM32F407VET6的小车
  *                   
  *
  * @hardware       : STM32F407VET6 + ICM20948 + 互补滤波(角速度加速度) + 三串级PID控制(角度，加速度，编码器)并联距离PID控制
  *
  * @update_log     : V1.0版本主要更新内容：
  *						预期设计角度和角速度PID控制，沿用pid_study.c(改)
  *
  * 				  V2.0版本主要更新：
  *						移除激光传感器，改为超声波传感器
  *						新增角度控制独立PID函数，解决角度180度跳变问题  
  *
  *
  * @control_system : 三环串级PID控制并联距离PID控制架构  超声波FD07

  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __GPIO_INOUT_H
#define __GPIO_INOUT_H
#include "sys.h"

void GPIO_IN_Init(void);
void GPIO_OUT_Init(void);
//超声波 PC0有问题，暂时不用
#define LSonic PCin(1)
#define RSonic PCin(2)
#define FSonic PCin(3)

//openmv 0代表未检测到目标，1代表检测到目标
#define OpenMV_P5 PCin(5)
#endif /* __BSP_LED_H */

