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
  * @project        : 大学生农业装备竞赛B类智能小车
  * @authors        : XPQH CQX LSL WFS LQQ
  * @date           : 2026.3.22
  * @version        : V1.0 
  * 
  * @description    : STM32F407VET6的智能小车
  *                   
  *
  * @hardware       : STM32F407VET6 + ICM20948 + 互补滤波(角速度加速度) + 三串级PID控制(角度，加速度，编码器)并联距离PID控制
  *
  * @update_log     : V1.0版本主要更新内容：
  *						预期设计角度和角速度PID控制，沿用pid_study.c(改)
  *
  * @control_system : 三环串级PID控制并联距离PID控制架构

  ******************************************************************************
  */
/* USER CODE END Header */

//注意，当前文件及其.h文件未被使用

#include "robot_state.h"
