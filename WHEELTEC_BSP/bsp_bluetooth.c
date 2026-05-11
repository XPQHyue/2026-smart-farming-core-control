#include "bsp_bluetooth.h"
#include "pid_study.h"
#include <string.h>

extern volatile float YawTarget;
extern volatile float VxRpm, VyRpm;
extern float target_angle_X, target_angle_Y;

#define PID_GAIN_SCALE 1000.0f

//来自APP的控制键值
WHEELTEC_APPKey_t wheeltecApp = { 0 };

void uart2_init(u32 bound)
{  	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          

	//UsartNVIC configuration
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	//USART Initialization Settings
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE); 
}


//JDY-33蓝牙AT指令集过滤
//当蓝牙连接或断开时，蓝牙模块会主动上报AT反馈信息，这里统一过滤
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0";
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;
				isFilter = 1;
				filterIndex = 2;
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;
				isFilter = 1;
				filterIndex = 2;
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;
				isFilter = 1;
				filterIndex = 2;
			}
			else if( recv=='C'&&lastrecv!='C' )
			{
				isFilter = 1;
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )
			{
				isFilter = 1;
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
        			  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )
			{
				isFilter = 1;
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )
			{
				isFilter = 1;
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )
			{
				isFilter = 1;
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}

//手机APP数据解码
void BlueToothAPPDecode(uint8_t recv)
{
	//APP控制页面临时变量
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
	
	//过滤蓝牙模块AT指令反馈
	uint8_t ATFilter = 0;
	ATFilter += ATCommandFeedBack_JDY33(recv);
	if( ATFilter > 0 ) return; //过滤
	
	/* APP控制页面切换 */
	if( recv == 'K' ) wheeltecApp.page = 2;      //按键页面
	else if( recv == 'J' ) wheeltecApp.page = 1; //摇杆页面
	else if( recv == 'I' ) wheeltecApp.page = 0; //参数设置页面
	
	//方向按键
	wheeltecApp.dirkey = recv-0x40;
	
	//APP控制页面 数据格式: {?:?}
	if(recv==0x7B) paramFlag=1;        //APP参数指令起始位
	else if(recv==0x7D) paramFlag=2;   //APP参数指令结束位
	
	if(paramFlag==1) //采集数据
	{
		paramReceive[(param_i%50)]=recv;
		param_i++;
	}
	else if(paramFlag==2) //解析数据
	{
		if(paramReceive[3]==0x50)  // {Q:P} 读取设备参数
		{
			wheeltecApp.reportparam = 1;
		}
			
		else if( paramReceive[3]==0x57 ) // {Q:W} 写入掉电保存参数
		{
			wheeltecApp.saveflash = 1;
		}				
		
		// ===== 修复: 普通参数分支 支持负数 =====
		else if(paramReceive[1]!=0x23)
		{
			uint8_t isNegative = 0;
			uint8_t startIdx = 3;  // 数据起始位置，跳过参数头
			
			// 检测负号
			if(paramReceive[3] == '-')
			{
				isNegative = 1;
				startIdx = 4;  // 有负号时从索引4开始
			}
			
			// 正向解析数字，避免pow精度问题
			for(param_j = startIdx; param_j < param_i; param_j++)
			{
				if(paramReceive[param_j] >= '0' && paramReceive[param_j] <= '9')
				{
					paramData = paramData * 10 + (paramReceive[param_j] - '0');
				}
			}
			
			// 应用符号
			if(isNegative) paramData = -paramData;
			
			switch(paramReceive[1])
			{
				// 0/1/2 通道固定绑定 VL53L0X 前向PID: Kp/Ki/Kd
				case 0x30: 	YawTarget = paramData; 		break;
				case 0x31:  target_angle_X = paramData; break;
				case 0x32:  target_angle_Y = paramData; break;
				case 0x33: 	VxRpm = paramData; 			break;
				case 0x34: break;
				case 0x35: break;
				case 0x36: break;
				case 0x37: break;
				case 0x38: break;
			}
		}
		// ===== 修复: 打包模式 支持负数 =====
		else if( paramReceive[1]==0x23 )
		{
			float num=0;
			uint8_t dataIndex=0;
			float dataArray[9]={0};
			uint8_t isNegative = 0;  // 负数标志

			if( param_i<=50 )
			{
				paramReceive[param_i]='}';

				for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
				{
					// 检测负号
					if( paramReceive[kk]=='-' )
					{
						isNegative = 1;
					}
					else if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
					{
						num = num*10 + ( paramReceive[kk] - '0' );
					}
					else if( paramReceive[kk]==':' )
					{
						// 应用符号后保存
						if(isNegative) num = -num;
						dataArray[dataIndex++] = num;
						num = 0;
						isNegative = 0;  // 重置负数标志
					}
				}
				// 处理最后一项
				if(isNegative) num = -num;
				dataArray[dataIndex] = num;
				
				// 批量模式前三个参数同样绑定到VL53L0X前向PID
				YawTarget = dataArray[0];
				target_angle_X = dataArray[1];
				target_angle_Y = dataArray[2];
						 VxRpm = dataArray[3];
			}
		}
		
		//相关标志位清零
		paramFlag=0;param_i=0;param_j=0;paramData=0;
		memset(paramReceive, 0, sizeof(uint8_t)*50);
	}
}
