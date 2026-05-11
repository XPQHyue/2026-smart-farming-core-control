#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "sys.h"

#define PWMA1 	  TIM10->CCR1
#define PWMA2 	  TIM11->CCR1

#define PWMB1 	  TIM9->CCR1
#define PWMB2 	  TIM9->CCR2

#define PWMC1 	  TIM1->CCR2
#define PWMC2 	  TIM1->CCR1

#define PWMD1 	  TIM1->CCR4
#define PWMD2 	  TIM1->CCR3

#define Servo_PWM  TIM12->CCR2
#define Servo_PWM2  TIM12->CCR1  // PB14 - ¶æ»ú2£¨ÐÂÔö£©

#define Servo_PWM3  TIM8->CCR1   // PC6
#define Servo_PWM4  TIM8->CCR2   // PC7
#define Servo_PWM5  TIM8->CCR3   // PC8
#define Servo_PWM6  TIM8->CCR4   // PC9
void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
void TIM10_PWM_Init(u16 arr,u16 psc);
void TIM11_PWM_Init(u16 arr,u16 psc);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void TIM12_SERVO_Init(u16 arr,u16 psc);
void TIM8_SERVO_Init(u16 arr,u16 psc);
#endif /* __BSP_MOTOR_H */
