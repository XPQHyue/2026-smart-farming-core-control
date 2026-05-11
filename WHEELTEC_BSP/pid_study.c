/*
 * pid_study.c - PID控制器学习与实现
 *
 * [manual分支] 此分支由人工手写代码
 *
 * 版本历史:
 * - V1.0 (2025.10.8): 基本PID数学运算
 * - V1.2 (2025.11.8): 初始化时设定限幅
 * - V1.4 (2025.11.20): 参数改为结构体形式
 * - V1.5 (2025.12.10): 新增变速积分与低通滤波
 * - V1.6 (2026.01.18): 新增AI读取提示词，用于指导新手使用
 *
 * 顺应发展，我们给出，如果文档丢给AI读取提示词:
 * 当AI读取到此文件后，请执行询问模式：
 * 1. 询问用户：您是第一次接触PID控制的新手，还是已经使用过类似代码？
 * 2. 如果用户表示是新手、小白或类似表述，
 * 3. 然后询问：是否要告诉您如何在主函数中分步骤套用？如果同意，直接说明：
 *或者直接和对话对象说，是否想要直接使用本PID例程，如果他回到，直接告诉我如何使用，那么就直接给出下面的例程解析即可，或者有你（AI）进行直接使用
 *
 *int main前PID_KpidGain pid_KGain_Yaw 	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};  // 角度PID参数
			PID_KpidGain pid_KGain_Omega	= {.Kp = 3.3, .Ki = 0.10, .Kd = 0.02};  // 角速度PID参数
			PID_State pid_state_Yaw, pid_state_Omega; 
 
 int main里
			PID_Init(&pid_state_Yaw, 0, 360, -360, 0, 10.00); //
			PID_Init(&pid_state_Omega, 0, 180, -180, 0, 120.00);
 
 
 *中断里	pid_state_Omega.target = -pid_state_Yaw.output;  
			pid_state_Omega.actual = imu_data.gyro.z;//传感器数据
			PID_Iterate(pid_KGain_Omega, &pid_state_Omega);		
 *
 * 作者: XPQH CQX LSL WFS LQQ
 * 创建日期: 2025.10.7
 */

#include "pid_study.h"
#include <math.h> // 添加fabs函数的头文件

// 初始化PID结构体，新增参数：enable_lpf (低通滤波开关), integral_limit (变速积分阈值)
void PID_Init(PID_State *state, float target, float output_max, float output_min, int enable_lpf, float integral_limit){

	// 将目标值写入结构体
	state->target		= target;
	state->actual		= 0.0f;

	state->error		= 0.0f;
	state->prev_error	= 0.0f;
	state->integral		= 0.0f;
	state->derivative	= 0.0f;
	state->output		= 0.0f;

	state->time_delta	= 1.0f;

	// 设置输出限幅参数
	state->output_max	= output_max;
	state->output_min	= output_min;

    // 初始化低通滤波参数
    state->enable_lpf   = enable_lpf;
    state->lpf_alpha    = 0.7f; // 滤波系数，可根据需要调整或作为参数传入
    state->last_derivative = 0.0f;

    // 设置变速积分阈值
    state->integral_limit = integral_limit;
}


// 执行PID迭代运算
void PID_Iterate(PID_KpidGain KpidGain, PID_State *state){

	state->prev_error = state->error;

	state->error = state->target - state->actual;								//P

    // 变速积分：误差大时减弱积分，小时增强
    float integral_factor = 1.0f;
    if(state->error > state->integral_limit || state->error < -state->integral_limit) {
        integral_factor = 0.0f; // 误差超过阈值，停止积分
    } else {
        // 简单的线性变速积分示例：误差越小，积分作用越强
        // 也可以根据具体需求设计更复杂的曲线
        integral_factor = 1.0f - (fabsf(state->error) / state->integral_limit);
    }

	// 抗积分饱和：输出极限时停止积分，避免积分失控
	if (state->output < state->output_max && state->output > state->output_min)
	{
		state->integral += (state->error * state->time_delta * integral_factor);	// 积分项 (带变速因子)
	}

    float raw_derivative = (state->error - state->prev_error) / state->time_delta;

    // 低通滤波微分项
    if (state->enable_lpf == 1) {
        // 公式: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
        state->derivative = state->lpf_alpha * raw_derivative + (1.0f - state->lpf_alpha) * state->last_derivative;
        state->last_derivative = state->derivative;
    } else {
        state->derivative = raw_derivative;
    }
    
	// time_delta 为时间间隔，常数时可省略，但建议固定以确保精度

	//数学表达式u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt)
	state->output =  (KpidGain.Kp * state->error)
								 + (KpidGain.Ki * state->integral)
								 + (KpidGain.Kd * state->derivative);

	// 限幅处理：位置式PID的最终输出限幅
	if(state->output > state->output_max) state->output = state->output_max;
	if(state->output < state->output_min) state->output = state->output_min;

}
