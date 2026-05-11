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
  *   A   |   C     
  *       |
  * ------+------->Vy+ (右)
  *       |
  *	   主控位置	
  *   B   |   D
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
#include "sys.h"
#include "pid_study.h"
#include "bsp_bluetooth.h"

#include "wit_c_sdk.h"
#include "bsp_wit_uart4.h"
#include "gpio_inout.h"

/******************************* 维特陀螺仪 *************************************/
   // 维特陀螺仪数据更新标志
#define WIT_ACC_UPDATE      0x01
#define WIT_GYRO_UPDATE     0x02
#define WIT_ANGLE_UPDATE    0x04

#define WIT_AUTO_ZZERO_ENABLE  1   // 1: 上电自动置零, 0: 关闭
   
static volatile char s_cWitDataUpdate = 0;


   // 维特陀螺仪数据
volatile  float WitAcc[3] = {0};    // 加速度
volatile  float WitGyro[3] = {0};   // 角速度
volatile  float WitAngle[3] = {0};  // 角度

   // 维特陀螺仪串口命令处理
   static uint8_t s_ucWitCmdData[20] = {0};
   static uint8_t s_ucWitCmdCnt = 0;


   // 维特陀螺仪串口发送回调
   static void WitSensorUartSend(uint8_t *p_data, uint32_t uiSize)
   {
       Uart4Send(p_data, uiSize);
   }

   // 维特陀螺仪延时回调
   static void WitDelayms(uint16_t ucMs)
   {
       delay_ms(ucMs);
   }

   // 维特陀螺仪数据更新回调
   static void WitSensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
   {
       int i;
       for (i = 0; i < uiRegNum; i++)
       {
           switch (uiReg)
           {
               case AZ:
                   s_cWitDataUpdate |= WIT_ACC_UPDATE;
                   break;
               case GZ:
                   s_cWitDataUpdate |= WIT_GYRO_UPDATE;
                   break;
               case Yaw:
                   s_cWitDataUpdate |= WIT_ANGLE_UPDATE;
                   break;
               default:
                   break;
           }
           uiReg++;
       }
   }

   // 维特陀螺仪自动扫描波特率
   static void WitAutoScanSensor(void)
   {
       int i, iRetry;
       const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

       for (i = 1; i < 10; i++)
       {
           WitUart4Init(c_uiBaud[i]);
           iRetry = 2;
           do
           {
               s_cWitDataUpdate = 0;
               WitReadReg(AX, 3);
               delay_ms(100);
               if (s_cWitDataUpdate != 0)
               {
                   return;  // 找到传感器
               }
               iRetry--;
           } while (iRetry);
       }
   }

   // 维特陀螺仪数据处理函数（在主循环中调用）
   static void Wit_Process(void)
   {
       if (s_cWitDataUpdate)
       {
           if (s_cWitDataUpdate & WIT_ANGLE_UPDATE)
           {
               // 读取角度 (Roll=0, Pitch=1, Yaw=2)
               WitAngle[0] = sReg[Roll] / 32768.0f * 180.0f;
               WitAngle[1] = sReg[Pitch] / 32768.0f * 180.0f;
               WitAngle[2] = sReg[Yaw] / 32768.0f * 180.0f;
               s_cWitDataUpdate &= ~WIT_ANGLE_UPDATE;
           }

           if (s_cWitDataUpdate & WIT_GYRO_UPDATE)
           {
               // 读取角速度
               WitGyro[0] = sReg[GX] / 32768.0f * 2000.0f;
               WitGyro[1] = sReg[GY] / 32768.0f * 2000.0f;
               WitGyro[2] = sReg[GZ] / 32768.0f * 2000.0f;
               s_cWitDataUpdate &= ~WIT_GYRO_UPDATE;
           }

           if (s_cWitDataUpdate & WIT_ACC_UPDATE)
           {
               // 读取加速度
               WitAcc[0] = sReg[AX] / 32768.0f * 16.0f;
               WitAcc[1] = sReg[AY] / 32768.0f * 16.0f;
               WitAcc[2] = sReg[AZ] / 32768.0f * 16.0f;
               s_cWitDataUpdate &= ~WIT_ACC_UPDATE;
           }
       }
   }

   /**
    * @brief  维特陀螺仪命令处理
    * @note   支持命令: Z0=Z轴置零, AUTOZ0=关闭校准, AUTOZ1=开启校准
    */
   static void Wit_CmdProcess(uint8_t ucData)
   {
       s_ucWitCmdData[s_ucWitCmdCnt++] = ucData;

       if (s_ucWitCmdCnt < 2) return;
       if (s_ucWitCmdCnt >= 20) s_ucWitCmdCnt = 0;

       // 检查命令结束符 "\r\n" 或 "\n"
       if (s_ucWitCmdCnt >= 2)
       {
           if ((s_ucWitCmdData[s_ucWitCmdCnt-2] == '\r' && s_ucWitCmdData[s_ucWitCmdCnt-1] == '\n') ||
               (s_ucWitCmdData[s_ucWitCmdCnt-1] == '\n'))
           {
               // 解析命令
               s_ucWitCmdData[s_ucWitCmdCnt-1] = '\0';  // 字符串结束
               if (s_ucWitCmdData[s_ucWitCmdCnt-2] == '\r') s_ucWitCmdData[s_ucWitCmdCnt-2] = '\0';

               // Z0 - Z轴置零
               if (strcmp((char*)s_ucWitCmdData, "Z0") == 0)
               {
                   if (WitStartIYAWCali() == WIT_HAL_OK)
                       printf("WIT: Z-Axis Zero OK\r\n");
                   else
                       printf("WIT: Z-Axis Zero Error\r\n");
               }
               // AUTOZ0 - 关闭自动零偏校准
               else if (strcmp((char*)s_ucWitCmdData, "AUTOZ0") == 0)
               {
                   if (WitStopRKMODECali() == WIT_HAL_OK)
                       printf("WIT: Auto Cali OFF OK\r\n");
                   else
                       printf("WIT: Auto Cali OFF Error\r\n");
               }
               // AUTOZ1 - 开启自动零偏校准
               else if (strcmp((char*)s_ucWitCmdData, "AUTOZ1") == 0)
               {
                   if (WitStartRKMODECali() == WIT_HAL_OK)
                       printf("WIT: Auto Cali ON OK\r\n");
                   else
                       printf("WIT: Auto Cali ON Error\r\n");
               }
               // HELP - 帮助
               else if (strcmp((char*)s_ucWitCmdData, "HELP") == 0)
               {
                   printf("\r\n=== WIT Command List ===\r\n");
                   printf("Z0      - Z-Axis Zero\r\n");
                   printf("AUTOZ0  - Auto Cali OFF\r\n");
                   printf("AUTOZ1  - Auto Cali ON\r\n");
                   printf("HELP    - Show Help\r\n");
               }

               s_ucWitCmdCnt = 0;
               memset(s_ucWitCmdData, 0, 20);
           }
       }
   }

   
float AnglePID(float Angactual, float Angtarget);

volatile uint8_t ADC_StartFlag = 0;
volatile uint8_t BuzzerTipsTime = 0;

/*******************************不修改*****************************************/
//4路电机控制PWM值
volatile int MotorA=0,MotorB=0,MotorC=0,MotorD=0;
//4路编码器值
volatile int EncoderA=0,EncoderB=0,EncoderC=0,EncoderD=0;
//电机转速
volatile float MotorArpm = 0,MotorBrpm = 0,MotorCrpm = 0,MotorDrpm = 0;
/*******************************不修改*****************************************/
//电机目标转速	最大转速 360
#define X_Wheel 0.085f
#define Y_Wheel 0.0930f
volatile float TargetArpm = 0,TargetBrpm = 0,TargetCrpm = 0,TargetDrpm = 0;
volatile float VxRpm = 0, VyRpm = 0,VzRpm = 0, VxRpmTaget = 0, VyRpmTaget = 0; //期望速度和实际速度
volatile float Vx_Step = 30.0f; //匀加速或匀减速时的“加速度”
volatile float VyRpm_L0 = 0, VyRpm_R0 = 0;

// Encoder position accumulators (int32_t, safe for long runs)
volatile int32_t EncoderTotal_A = 0, EncoderTotal_B = 0, EncoderTotal_C = 0, EncoderTotal_D = 0;
// Position PID control (distance-based stop, wraps velocity PI)
volatile float PositionTarget_mm = 0, DistanceTraveled_mm = 0;
volatile uint8_t PositionPID_Enable = 0;
volatile uint8_t PositionPID_Done  = 0;
volatile uint8_t EncoderResetFlag  = 0;
PID_KpidGain pid_KGain_Position;
PID_State    pid_state_Position; //左右两侧距离PID输出的期望速度分量

////RPM转速度公式： RPM/60 = N r/s  N * R = V m/s    R为轮子半径
volatile uint32_t Time0 = 0, Time1 = 0, Time0Flage = 0;
volatile uint32_t TimeSonicL = 0, TimeSonicR = 0, TimeSonicF = 0;
volatile uint8_t TimeSonicFlage = 0;
//电池电压
float robotVol;
//使用本例程时，请根据电机和编码器的实际参数配置以下的宏定义
#define MOTOR_REDUCTION_RATION  30  //电机减速比
#define ENCODER_ACCURACY        13  //编码器精度,GMR编码器是500,霍尔编码器是13
#define CONTROL_FREQ            100 //PID控制器的执行频率,也就是控制频率,跟中断配置相关,本例程配置的是200Hz
#define PULSES_PER_REVOLUTION    (30 * 13 * 4)  // 1560
#define WHEEL_DIAMETER_MM        75.0f
#define MM_PER_PULSE             (3.14159f * WHEEL_DIAMETER_MM / (float)PULSES_PER_REVOLUTION)

// 单点模式配置: 0=比赛模式 1=调试模式
#define SYSTEM_MODE_DEBUG       1

#if SYSTEM_MODE_DEBUG
#define MAIN_LOOP_HZ_MONITOR    1   // 调试模式: 开启主循环频率监测
#define VOFA_SEND_PERIOD_MS     100 // 调试模式: 10Hz上报
#define VL53_STUCK_LIMIT_CFG    60  // 调试模式: 放宽超时判定
#else
#define MAIN_LOOP_HZ_MONITOR    0   // 比赛模式: 关闭监测串口输出
#define VOFA_SEND_PERIOD_MS     200 // 比赛模式: 5Hz上报(可按需提高)
#define VL53_STUCK_LIMIT_CFG    45  // 比赛模式: 约300ms快速恢复
#endif

volatile uint8_t Omega_TIM_ONE = 100;
volatile float Omega_Test_PID = 0;
#define OMEGAKP 10.231
#define OMEGAKI 0.431
#define OMEGAKD 15.41
PID_KpidGain pid_KGain_Omega;  // 角速度PID参数
PID_State pid_state_Omega; 

volatile float Velocity_KP = 5.4f,Velocity_KI = 1.1f;	//PI控制器 第三级 直接输出PWM

volatile float Angle_Kp = 5.1f, Angle_Ki = 0.0084f, Angle_Kd = 18.202f; //角度PID参数

volatile float YawTarget = 0, YawAct = 0; 
volatile float  YawAngle = 0, YawAng_out = 0;  //转为360度制的负角度值
volatile int Car_ON = 0; 	//小车启动标志位,按键单击触发
volatile int Sonic_ON = 9;	//超声波控制标志
char app_page1[1024]={ 0 };	//发送到APP首页的数据缓冲区
/*******************************以上无需修改*****************************************/

volatile uint32_t NEXT_ROW = 0, NEXT_ROWFlag = 0;

float target_angle_X = 170.0f;   //初始状态  		数据大的为逆时针
float target_angle_Y = 201.0f;//109					数据越大越往上

float target_angle_D = 50.0f;   // TIM8_CH4 PC9 R_X	数据大的为逆时针			
float target_angle_C = 20.0f;   // TIM8_CH3 PC8 R_Y	数据越大越往下

float target_angle_A = 133.0f;   // TIM8_CH1 PC6 	预留
float target_angle_B = 133.0f;   // TIM8_CH2 PC7	预留

//target_angle_X = 202.0f;   		水平下压
//target_angle_Y = 101.0f;			109
//target_angle_X = 102.0f;   		扭起来
//target_angle_X = 140.0f;   		140放置土豆

//target_angle_D = 10.0f;   	 	水平下压	
//target_angle_C = 141.0f;   		数据越大越往下
//target_angle_D = 112.0f;   		扭起来R_X 	
  

#define ROW_PLAY 	0
#define ROW_1 		0
#define S_X_1		0
#define S_Y_1		0

#define ROW_2 		0
#define ROW_2_1 	0
#define S_X_2		0
#define S_Y_2		0

#define ROW_3 		0
#define S_X_3		0
#define S_Y_3		0

#define ROW_4 		0
#define S_X_4		0
#define S_Y_4		0
#define ROW_4_1		0
#define S_X_4_1		0
#define S_Y_4_1		0

#define ROW_5 		0
#define S_X_5		0
#define S_Y_5		0

#define ROW_6		0
#define S_X_6		0
#define S_Y_6		0

#define ROW_OVER 	0



//定义使用的舵机是180度还是270度舵机
#define _270_SERVO  1
#define _180_SERVO  0


int main(void)
{
	//设置系统中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化SysTick,配置其频率为1000Hz
	SysTick_Init(1000);
	uart2_init(9600); //用于蓝牙
	LED_Init();   //LED初始化
	GPIO_IN_Init();

	delay_ms(200);

   // 初始化维特陀螺仪 (PC10/PC11)
        WitInit(WIT_PROTOCOL_NORMAL,0x50);
        WitSerialWriteRegister(WitSensorUartSend);
        WitRegisterCallBack(WitSensorDataUpdata);
        WitDelayMsRegister(WitDelayms);
        WitUart4Init(115200);  // 波特率根据你的陀螺仪设置
        delay_ms(100);
        WitAutoScanSensor(); // 自动扫描波特率
		
   #if WIT_AUTO_ZZERO_ENABLE
       // 等待传感器数据稳定
       delay_ms(500);

       // 执行Z轴自动置零
       if (WitStartIYAWCali() == WIT_HAL_OK)
       {
           printf("WIT: Auto Z-Axis Zero OK\r\n");
       }
       else
       {
           printf("WIT: Auto Z-Axis Zero Failed\r\n");
       }
       delay_ms(100);  // 等待置零生效
   #endif

	delay_ms(200);

	Adc_Init();   //ADC初始化
	KEY_Init();   //用户按键、使能开关初始化
	Buzzer_Init();//蜂鸣器初始化
	
	//4路电机PWM初始化,PWM频率10KHz.一路电机控制需要2路PWM

		//电机启动
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);
	
	//4路编码器初始化
	Encoder_Init_TIM2();
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	Encoder_Init_TIM5();


#if MAIN_LOOP_HZ_MONITOR
	uint32_t loop_cnt = 0;
	uint32_t loop_hz = 0;
	uint32_t loop_ts_ms = User_GetSysTickTime();
#endif
	uint32_t vofa_ts_ms = User_GetSysTickTime();
	PID_Init(&pid_state_Omega, 0, 820.00f, -820.00f, 0, 100.00f);
	pid_KGain_Omega.Kp = OMEGAKP;
	pid_KGain_Omega.Ki = OMEGAKI;
	pid_KGain_Omega.Kd = OMEGAKD;
	
		// Position PID init (output=RPM ceiling, P-only first to avoid overshoot)
		PID_Init(&pid_state_Position, 0, 300.0f, -300.0f, 0, 300.0f);
		pid_KGain_Position.Kp = 4.231f;//通过Z-N整定法，快速给到了一个超调1/4的系统
		pid_KGain_Position.Ki = 0.01741f;//极快加快了调节PID速度
		pid_KGain_Position.Kd = 97.25f;
		
//		pid_KGain_Position.Kp = 4.371f;//单云台时PID
//		pid_KGain_Position.Ki = 0.01281f;//
//		pid_KGain_Position.Kd = 93.85f;		
		

	// 先计算目标初始偏角度的PWM，防止底层死值导致抽搐
	#if _270_SERVO
		target_angle_X = target_limit_float(target_angle_X,0,270);
		target_angle_Y = target_limit_float(target_angle_Y,0,270);

		target_angle_A = target_limit_float(target_angle_A,0,270);
		target_angle_B = target_limit_float(target_angle_B,0,270);
		target_angle_C = target_limit_float(target_angle_C,0,270);
		target_angle_D = target_limit_float(target_angle_D,0,270);
	#endif
	
	// 在PWM未开启前，提前算出初始PWM该多少
	int init_pwm_x = Angle_to_PWM(target_angle_X);
	int init_pwm_y = Angle_to_PWM(target_angle_Y);

	int init_pwm_a = Angle_to_PWM(target_angle_A);
	int init_pwm_b = Angle_to_PWM(target_angle_B);
	int init_pwm_c = Angle_to_PWM(target_angle_C);
	int init_pwm_d = Angle_to_PWM(target_angle_D);
	//舵机PWM初始化,初始化为50Hz,0-19099对应 0 到 100% 占空比
	//2.5%~12.5%(对应数值500~2500)占空比对应舵机的0-180或0-270度
	TIM12_SERVO_Init(19999,83);

	TIM8_SERVO_Init(19999,167);	
	//把真正的需求赋值进去，避免初始化瞬间脉冲不对导致跳变
	Servo_PWM = init_pwm_x;
	Servo_PWM2 = init_pwm_y;

	Servo_PWM3 = init_pwm_a;
	Servo_PWM4 = init_pwm_b;
	Servo_PWM5 = init_pwm_c;
	Servo_PWM6 = init_pwm_d;	
	delay_ms(500);
	TIM7_Init(83,4999);	
	uint8_t Servo_ON = 0;
	
	uint8_t FSonic_TEST = 0, FSonic_TEST_Flage = 0;
	while(1)
	{
				
#if MAIN_LOOP_HZ_MONITOR
		loop_cnt++;
#endif	
		
		//电池电压采集
		if( ADC_StartFlag==1 )
		{
			ADC_StartFlag=0;
			robotVol = (float)Get_Adc(Battery_Ch)/4095.0f * 3.3f * 11.0f;
		}
		
		//蜂鸣器提示
		if( BuzzerTipsTime!=0 )
		{
			BuzzerTipsTime--;
			Buzzer=1;
		}
		else Buzzer = 0;
					
#if MAIN_LOOP_HZ_MONITOR
		{
			uint32_t now_ms = User_GetSysTickTime();
			uint32_t dt_ms = now_ms - loop_ts_ms;
			if(dt_ms >= 1000)
			{
				loop_hz = (loop_cnt * 1000U) / dt_ms;
				printf("[loop] hz=%lu dt=%lums\r\n", (unsigned long)loop_hz, (unsigned long)dt_ms);
				loop_cnt = 0;
				loop_ts_ms = now_ms;
			}
		}
#endif
		if((User_GetSysTickTime() - vofa_ts_ms) >= VOFA_SEND_PERIOD_MS)
		{
		


			vofa_ts_ms = User_GetSysTickTime();
//			sprintf((char*)app_page1,"%.2f,%.2f,%.2f,%.2f,%.2f\n", WitAngle[2],PositionTarget_mm,VxRpm,VxRpmTaget,DistanceTraveled_mm);

//			sprintf((char*)app_page1,"%.2f,%.2f,%.2f,%.2f,%d\n", WitAngle[2],YawTarget,PositionTarget_mm,DistanceTraveled_mm,Omega_TIM_ONE);
							
			sprintf((char*)app_page1,"%.2f,%d,%d\n", WitAngle[2],FSonic,Omega_TIM_ONE);	
			
//			sprintf((char*)app_page1,"%d,%d\n",RSonic_TSTE,Omega_TIM_ONE);
			
			AppSendData(app_page1,strlen(app_page1));//将数据发送到APP(VOFA)
		}
		
		
		#if _180_SERVO
		//目标值限幅
		target_angle = target_limit_float(target_angle,0,180);
		#endif
		
		#if _270_SERVO
		//270度舵机
		target_angle_X = target_limit_float(target_angle_X,0,270);
		target_angle_Y = target_limit_float(target_angle_Y,0,270);

		target_angle_A = target_limit_float(target_angle_A,0,270);
		target_angle_B = target_limit_float(target_angle_B,0,270);
		target_angle_C = target_limit_float(target_angle_C,0,270);
		target_angle_D = target_limit_float(target_angle_D,0,270);
		#endif
		
		//将目标角度转换为PWM并赋值
		Servo_PWM = Angle_to_PWM(target_angle_X);
		Servo_PWM2 = Angle_to_PWM(target_angle_Y);

		Servo_PWM3 = Angle_to_PWM(target_angle_A);
		Servo_PWM4 = Angle_to_PWM(target_angle_B);
		Servo_PWM5 = Angle_to_PWM(target_angle_C);
		Servo_PWM6 = Angle_to_PWM(target_angle_D);
		/***********************单独程序测试口**************************/
				//超声波 F 方向测试
			switch (FSonic_TEST)
			{
			case 0:
					PositionPID_Enable = 1;	
					PositionTarget_mm = 346.50f;//346.50f
					if(PositionPID_Done == 1){FSonic_TEST = 1;}
			break;

			case 1: 
					if(FSonic == 0){ FSonic_TEST = 2; }

			break;

			case 2:	EncoderResetFlag = 1; PositionPID_Enable = 1;
					PositionTarget_mm = 30.0f; PositionPID_Done = 0;
					FSonic_TEST = 3;break;


			}

		/***********************单独程序测试口*************************/
			switch (Car_ON)
			{
				case 0: break;
				//直行，旋转1											起点，直行
				case 1:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 346.50f;//346.50f
							if(PositionPID_Done == 1){Car_ON = 2;}
				break;
				case 2: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0;
						Car_ON = 3;break;
				case 3:
							
							YawTarget = -88.5f;
							if(fabsf(YawAct - YawTarget) <= 2.1f){NEXT_ROWFlag = 1;}
							if(NEXT_ROW >= 2){ Car_ON = 4; }
							
				break;

				//直行，旋转 2    										垄1跑
				case 4:
							NEXT_ROWFlag = 0;
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2580.00f;
							if(PositionPID_Done == 1){Car_ON = 5;}
				break;
				case 5: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 6;break;
				case 6:
							YawTarget = 0;
							if(fabsf(YawAct - YawTarget) <= 2.1f){Car_ON = 7;}
				break;	
							
				//直行，旋转 3    
				case 7:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 700.00f;
							if(PositionPID_Done == 1){Car_ON = 8;}
				break;
				case 8: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 9;break;
				case 9:
							YawTarget = 91.5f;
							target_angle_X = 202.0f;		//水平下压
							target_angle_Y = 111.0f;		//数据越大越往上
								
							target_angle_D = 10.0f;   	 	//水平下压	
							target_angle_C = 133.0f;   		//数据越大越往下						
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 10;}
				break;

				//直行，旋转 4     										垄2跑
				case 10:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2580.00f;
							if(DistanceTraveled_mm >= 150.0f & Servo_ON == 0)
							{
								
								target_angle_X = 202.0f;		//水平下压
								target_angle_Y = 102.0f;		//数据越大越往上
								
								target_angle_D = 10.0f;   	 	//水平下压	
								target_angle_C = 140.0f;   		//数据越大越往下	
								
								Servo_ON = 1;
							}else if(DistanceTraveled_mm >= 2400.0f & Servo_ON == 1)
							{
								target_angle_X = 102.0f;		//扭起来
								target_angle_D = 112.0f;   		//扭起来R_X 								
								Servo_ON = 2;
							}else if(DistanceTraveled_mm >= 2490.0f & Servo_ON == 2)
							{
								target_angle_X = 170.0f;   //初始状态  		数据大的为逆时针
								target_angle_Y = 201.0f;//109					数据越大越往上

								target_angle_D = 50.0f;   // TIM8_CH4 PC9 R_X	数据大的为逆时针			
								target_angle_C = 20.0f;   // TIM8_CH3 PC8 R_Y	数据越大越往下
								

								Servo_ON = 3;
							}								
				
							if(PositionPID_Done == 1){Car_ON = 11;}
				break;
				case 11: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 12;break;
				case 12:
							YawTarget = 0;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 13;}
				break;	

				//直行，旋转 5    										
				case 13:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 700.00f;
							if(PositionPID_Done == 1){Car_ON = 14;}
				break;
				case 14: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 15;break;
				case 15:
							YawTarget = -88.5f;					
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 16;}
				break;	

				//直行，旋转 6    							垄3跑	 				
				case 16:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2605.00f;
							if(PositionPID_Done == 1){Car_ON = 17;}
				break;
				case 17: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 18;break;
				case 18:
							YawTarget = 0;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 19;}
				break;	

				//直行，旋转 7    								 				
				case 19:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 700.00f;
							if(PositionPID_Done == 1){Car_ON = 20;}
				break;
				case 20: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 21;break;
				case 21:
							YawTarget = 91.5f;
							target_angle_X = 202.0f;		//水平下压
							target_angle_Y = 111.0f;		//数据越大越往上
								
							target_angle_D = 10.0f;   	 	//水平下压	
							target_angle_C = 130.0f;   		//数据越大越往下
							Servo_ON  = 0;				
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 22;}
				break;

				//直行，旋转 8    							垄4跑	 				
				case 22:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2605.00f;
				
							if(DistanceTraveled_mm >= 150.0f & Servo_ON == 0)
							{
								
								target_angle_X = 202.0f;		//水平下压
								target_angle_Y = 102.0f;		//数据越大越往上
								
								target_angle_D = 10.0f;   	 	//水平下压	
								target_angle_C = 140.0f;   		//数据越大越往下	
								
								Servo_ON = 1;
							}else if(DistanceTraveled_mm >= 2500.0f & Servo_ON == 1)
							{
								target_angle_X = 102.0f;		//扭起来
								target_angle_D = 112.0f;   		//扭起来R_X 						
								Servo_ON = 2;
							}else if(DistanceTraveled_mm >= 2600.0f & Servo_ON == 2)
							{
								target_angle_X = 170.0f;   //初始状态  		数据大的为逆时针
								target_angle_Y = 201.0f;//109					数据越大越往上
								target_angle_D = 50.0f;   // TIM8_CH4 PC9 R_X	数据大的为逆时针			
								target_angle_C = 20.0f;   // TIM8_CH3 PC8 R_Y	数据越大越往下
								

								Servo_ON = 3;
							}				
				
							if(PositionPID_Done == 1){Car_ON = 23;}
				break;
				case 23: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 24;break;
				case 24:
							YawTarget = 0;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 25;}
				break;	
							
				//直行，旋转 9    								 				
				case 25:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 700.00f;
							if(PositionPID_Done == 1){Car_ON = 26;}
				break;
				case 26: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 27;break;
				case 27:
							YawTarget = -88.5f;
							target_angle_X = 202.0f;		//水平下压
							target_angle_Y = 121.0f;		//数据越大越往上
								
							target_angle_D = 10.0f;   	 	//水平下压	
							target_angle_C = 125.0f;   		//数据越大越往下
							Servo_ON = 0;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 28;}
				break;

				//直行，旋转 10   							垄5跑	 				
				case 28:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2605.00f;
				
				
							if(DistanceTraveled_mm >= 150.0f & Servo_ON == 0)
							{
								
								target_angle_X = 202.0f;		//水平下压
								target_angle_Y = 101.5f;		//数据越大越往上
								
								target_angle_D = 10.0f;   	 	//水平下压	
								target_angle_C = 140.5f;   		//数据越大越往下	
								
								Servo_ON = 1;
							}else if(DistanceTraveled_mm >= 2500.0f & Servo_ON == 1)
							{
								target_angle_X = 102.0f;		//扭起来
								target_angle_D = 112.0f;   		//扭起来R_X 
								
								
								Servo_ON = 2;
							}else if(DistanceTraveled_mm >= 2590.0f & Servo_ON == 2)
							{
								target_angle_X = 170.0f;   //初始状态  		数据大的为逆时针
								target_angle_Y = 201.0f;//109					数据越大越往上

								target_angle_D = 50.0f;   // TIM8_CH4 PC9 R_X	数据大的为逆时针			
								target_angle_C = 20.0f;   // TIM8_CH3 PC8 R_Y	数据越大越往下
								

								Servo_ON = 3;
							}				
				
							if(PositionPID_Done == 1){Car_ON = 29;}
				break;
				case 29: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 30;break;
				case 30:
							YawTarget = 0;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 31;}
				break;	

				//直行，旋转 11    								 				
				case 31:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 700.00f;
							if(PositionPID_Done == 1){Car_ON = 32;}
				break;
				case 32: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 33;break;
				case 33:
							YawTarget = 92.0f;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 34;}
				break;

				//直行，旋转 12   							垄6跑	 				
				case 34:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 2620.00f;
							if(PositionPID_Done == 1){Car_ON = 35;}
				break;
				case 35: EncoderResetFlag = 1; PositionPID_Enable = 0;
						PositionTarget_mm = 0; PositionPID_Done = 0; Car_ON = 36;break;
				case 36:
							YawTarget = -180;
							if(fabsf(YawAct - YawTarget) < 2.1f){Car_ON = 37;}
				break;	

				//直行，旋转 13    								 				
				case 37:
							
							PositionPID_Enable = 1;	
							PositionTarget_mm = 4100.00f;
							if(PositionPID_Done == 1){Car_ON = 38;}
				break;					
							
			}
			 

	}
	
}

//定时器7更新中断服务函数,根据配置的运行频率触发
void TIM7_IRQHandler(void)
{
	static uint32_t LedTickCnt = 0;
	
	static uint32_t adc_taskCnt = 0;
	
	static uint8_t EncoderTaskFlag = 0;
	static uint8_t arrivalCount = 0;
	static uint32_t TIME_ONE_TickCnt = 0;
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	
		
		// 处理维特陀螺仪数据
        Wit_Process();	
		
		
			switch(Time0Flage)
			{
				
				case 0:  Time0 = 0; break;
				case 1:  Time0++; break;
				case 2:  Time0 = 0;break;				
			
			}
			
			//L R F控制
			TimeSonicL++;
			TimeSonicR++;
			switch (TimeSonicFlage)
			{
				//
				case 0: VxRpm = VxRpm;  break;
				case 1: VxRpm = VxRpm; break;
			}
		
		
		//LED闪烁,1秒闪烁1次.辅助检查频率是否正确
		LedTickCnt++;
		if( LedTickCnt > 200 * 1 ) 
		{
			LedTickCnt=0;
			LED = !LED;

		}
		
		//标准1s计数
		TIME_ONE_TickCnt++;
		if( TIME_ONE_TickCnt > 200 * 1 ) 
		{
			TIME_ONE_TickCnt=0;
			Omega_TIM_ONE ^= 100;
		}	

		//快速到达设计 5ms为一个计次
		if(NEXT_ROWFlag == 1 )
		{
			NEXT_ROW++;
		}else
			{
				NEXT_ROW = 0;
			
			}


		//ADC任务
		adc_taskCnt++;
		if( adc_taskCnt == 10 )
		{
			adc_taskCnt = 0;
			ADC_StartFlag=1;
		}

		//为了简单学习麦克纳姆轮逆运动学，有必要给出我们的逆运动学方案 在balance.c	
		//读取编码器原始值,降频至100Hz读取  
		EncoderTaskFlag = !EncoderTaskFlag;
		if( EncoderTaskFlag==1 )
		{
			
			//读取编码器数值
			EncoderA = Read_Encoder(2);
			EncoderB = -Read_Encoder(3);
			EncoderC = Read_Encoder(4);
			EncoderD = -Read_Encoder(5);
			
			//电机旋转一圈编码器读数 = 电机减速比*编码器精度*4
			//电机转速RPM = 当前读取到的值/电机旋转一圈编码器读数 * 控制频率 * 60
			MotorArpm = (float)EncoderA/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorBrpm = (float)EncoderB/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorCrpm = (float)EncoderC/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorDrpm = (float)EncoderD/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;

			// === Encoder accumulation for distance odometry ===
			if(EncoderResetFlag)
			{
				EncoderTotal_A = 0; EncoderTotal_B = 0;
				EncoderTotal_C = 0; EncoderTotal_D = 0;
				EncoderResetFlag = 0;
			}
			EncoderTotal_A += EncoderA;
			EncoderTotal_B += EncoderB;
			EncoderTotal_C += EncoderC;
			EncoderTotal_D += EncoderD;

			// Forward distance via average of 4 wheels (mecanum odometry)
			DistanceTraveled_mm = (float)(EncoderTotal_A - EncoderTotal_B - EncoderTotal_C + EncoderTotal_D) / 4.0f * MM_PER_PULSE;

			// === Position PID (distance-to-speed outer loop) ===
			if(PositionPID_Enable == 1)
			{
				pid_state_Position.target = PositionTarget_mm;
				pid_state_Position.actual = DistanceTraveled_mm;
				PID_Iterate(pid_KGain_Position, &pid_state_Position);
				VxRpmTaget = pid_state_Position.output;

				// 到达判断：误差<6.5mm持续50ms（Ki自动收敛）
				if(fabsf(PositionTarget_mm - DistanceTraveled_mm) < 6.5f)
				{
					arrivalCount++;
					if(arrivalCount >= 2)
					{
						VxRpmTaget = 0;
						PositionPID_Enable = 0;
						PositionPID_Done = 1;
						arrivalCount = 0;
					}
				}
				else
				{
					arrivalCount = 0;
				}
			}

			//角度PID
			YawAct = WitAngle[2]; 			
			YawAng_out = AnglePID (YawAct,YawTarget);
			
			// 角速度Omega PID (100Hz, synced with encoder)
			pid_state_Omega.target = YawAng_out + Omega_Test_PID;
			pid_state_Omega.actual = WitGyro[2] /** 57.2957795f*/; //单位 度/s
			PID_Iterate(pid_KGain_Omega, &pid_state_Omega);
			
			VzRpm = pid_state_Omega.output/* * (X_Wheel + Y_Wheel)*/;

			if(VxRpm < VxRpmTaget)//匀加速直线运动
			{
				VxRpm += Vx_Step;
				if(VxRpm > VxRpmTaget)
				{
 					VxRpm = VxRpmTaget;
				}
			}
			else if(VxRpm > VxRpmTaget)//匀减速
			{
				VxRpm -= Vx_Step;
				if(VxRpm < VxRpmTaget)
				{
					VxRpm = VxRpmTaget;
				} 
			}

			//本质上是简化版逆运动学对应 Vy + Vx + Vz;  因为小车编码器BC原因，BC要反转极性(当前已反转)
			//由于辊子的转动，产生的侧向的力，推进小车的侧向运动
			TargetArpm = + VyRpm + VxRpm - VzRpm;	//并联PID
			TargetBrpm = + VyRpm - VxRpm + VzRpm;
			TargetCrpm = + VyRpm - VxRpm - VzRpm;
			TargetDrpm = + VyRpm + VxRpm + VzRpm;
	

			//4路电机PI控制器  直接接入角速度输出  逆时针为正
			MotorA = Incremental_PI_A(MotorArpm,TargetArpm);
			MotorB = Incremental_PI_B(MotorBrpm,TargetBrpm);
			MotorC = Incremental_PI_C(MotorCrpm,TargetCrpm);
			MotorD = Incremental_PI_D(MotorDrpm,TargetDrpm);	
		}	
		
		//按键扫描
		uint8_t userKeyState = KEY_Scan(200,0);
		switch( userKeyState )
		{
			case single_click://单击
				Car_ON = 1;
				if( robotVol < 10.0f ) BuzzerTipsTime = 10; //电池电压不足以驱动电机,蜂鸣器提示
				break;
			case double_click://双击
				Car_ON = 1;
				if( robotVol < 10.0f ) BuzzerTipsTime = 10; //电池电压不足以驱动电机,蜂鸣器提示
				break;
			case long_click://长按
				Car_ON = 1;
				break;
		}
				
		//PWM限幅 [manual-GB2312]
		MotorA = target_limit_int(MotorA,-16799,16799);
		MotorB = target_limit_int(MotorB,-16799,16799);
		MotorC = target_limit_int(MotorC,-16799,16799);
		MotorD = target_limit_int(MotorD,-16799,16799);
		
		//发送PWM到电机函数, 0-16799对应电机0到最大转速
		//0-16799对应电机0到最大转速 负数时电机转向相反
		Set_Pwm(MotorA,MotorB,MotorC,MotorD);
		
		
	}
}


//针对角度跳变优化PID
float AnglePID (float AngActual,float AngTarget)
{ 	
	
	static volatile float error=0.00f,prev_error=0.00f,output=0.00f,time_delta=1.00f,integral=0.00f,derivative=0.00f;
	const float output_max = 200.00f;
	const float output_min = -200.00f;
	const float integral_max = 3.00f;
	const float integral_min = -3.00f;
	volatile float p_term = 0.00f, d_term = 0.00f, output_unsat = 0.00f;
	volatile float integral_candidate = 0.00f;

	prev_error = error;
	error = AngTarget - AngActual;			//P
	if(error > 180)
	{
		error -= 360.00f;
	}else if(error < -180)
		{
		error += 360.00f;
		}	
	derivative = (error - prev_error) / time_delta;//D
	p_term = Angle_Kp * error;
	d_term = Angle_Kd * derivative;
	output_unsat = p_term + (Angle_Ki * integral) + d_term;

	// 条件积分抗饱和: 饱和且误差同向时停止积分，误差反向时恢复积分
	integral_candidate = integral + (error * time_delta);
	if((output_unsat < output_max && output_unsat > output_min)
		|| (output_unsat >= output_max && error < 0.00f)
		|| (output_unsat <= output_min && error > 0.00f))
	{
		integral = integral_candidate;
	}

	// 积分限幅，防止积分项长期累积
	if(integral > integral_max) {
		integral = integral_max;
	} else if(integral < integral_min) {
		integral = integral_min;
	}

	output = p_term + (Angle_Ki * integral) + d_term;
	if(output > output_max) {
		output = output_max;
	} else if(output < output_min) {
		output = output_min;
	}
	return output;
	
}



/***********************PI控制器端********************************************/
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}


//PI控制器
int Incremental_PI_A (float Encoder,float Target)
{ 	
	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差	
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
	 if(Pwm>16799)Pwm=16799;
	 if(Pwm<-16799)Pwm=-16799;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差
	 return Pwm;    
	
}


int Incremental_PI_B (float Encoder,float Target)
{ 	
	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16799)Pwm=16799;
	 if(Pwm<-16799)Pwm=-16799;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;   
	
}


int Incremental_PI_C (float Encoder,float Target)
{ 	
	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16799)Pwm=16799;
	 if(Pwm<-16799)Pwm=-16799;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;   
	
}


int Incremental_PI_D (float Encoder,float Target)
{ 	
	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16799)Pwm=16799;
	 if(Pwm<-16799)Pwm=-16799;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
	
}



//串口2接收中断,接收手机APP通过蓝牙模块发送过来的数据
void USART2_IRQHandler(void)
{	
	uint8_t Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//判断是否接收到数据
	{	
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		Usart_Receive = USART_ReceiveData(USART2);
		
		
           // 维特陀螺仪命令处理
           Wit_CmdProcess(Usart_Receive);		
		
		
		//将接收到的串口数据传入解码函数,对手机APP的数据进行解析
		//解析后的数据存放在 结构体 wheeltecApp 处
		BlueToothAPPDecode(Usart_Receive);
	}
}



//串口2发送函数,用于发送数据到APP
void AppSendData(char* buffer,uint8_t Len)
{
	for(uint8_t i=0;i<Len;i++)
	{
		while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
		USART2->DR = (u8) buffer[i];  
	}
}

//舵机角度转化为PWM函数
int Angle_to_PWM(float Angle)
{
	//PWM 500~2500 对应占空比 2.5%~12.5%，对应舵机 0-180度
	
	#if _180_SERVO
	// 180度舵机角度转换
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    
    // 线性映射：PWM = 500 + (2500-500)*Angle/180
    // 简化后：PWM = 500 + 2000*Angle/180
    int PWM = (int)(500 + (2000.0f * Angle / 180.0f));
    
    // 确保PWM值在有效范围内
    if (PWM < 500) PWM = 500;
    if (PWM > 2500) PWM = 2500;
    
    return PWM;
	#endif
	
	//如果是270度舵机，则使用下面内容
	#if _270_SERVO
	// 限制角度范围在0~270度
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 270.0f) Angle = 270.0f;
    
    // 线性映射：PWM = 500 + (2500-500)*Angle/270
    // 简化后：PWM = 500 + 2000*Angle/270
    int PWM = (int)(500 + (2000.0f * Angle / 270.0f));
    
    // 确保PWM值在有效范围内
    if (PWM < 500) PWM = 500;
    if (PWM > 2500) PWM = 2500;
    
    return PWM;
	#endif
}

float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
