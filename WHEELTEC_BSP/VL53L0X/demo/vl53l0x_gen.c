#include "vl53l0x_gen.h"

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

VL53L0X_RangingMeasurementData_t vl53l0x_data;//测距测量结构体
vu16 Distance_data=0;//保存测距数据

//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//0：默认;1:高精度;2:长距离;3:高速
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,u8 mode)
{
	
	 VL53L0X_Error status = VL53L0X_ERROR_NONE;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	 vl53l0x_reset(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit(dev);
	 
	 
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if(status!=VL53L0X_ERROR_NONE) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if(status!=VL53L0X_ERROR_NONE) goto error;
        delay_ms(2);		 	 
	 
	 
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);//使能单次测量模式
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//使能SIGMA范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//使能信号速率范围检查
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//设定SIGMA范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//设定信号速率范围范围
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//设定完整测距最长时间
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		print_pal_error(status);
		return status;
	 }
	 return status;
	
}	

//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf)
{

	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE) return status;
   
	RangeStatus = pdata->RangeStatus;//获取当前测量状态
    memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus,buf);//根据测量状态读取状态字符串
	
	Distance_data = pdata->RangeMilliMeter;//保存最近一次测距测量数据

    return status;
}


//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离
void vl53l0x_general_start(VL53L0X_Dev_t *dev,u8 mode)
{
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;//工作状态
	u8 i=0;
	
	//mode_string(mode,buf);//显示当前配置的模式
	while(vl53l0x_set_mode(dev,mode))//配置测量模式
	{
		printf("Mode Set Error!!!\r\n");
		i++;
		if(i==2) return;
	}
	
	while(Status==VL53L0X_ERROR_NONE)
	{
		Status = vl53l0x_start_single_test(dev,&vl53l0x_data,buf);//执行一次测量
		printf("d: %4imm\r\n",Distance_data);//打印测量距离
	}
	delay_ms(500);
	printf("Measurement is Error,Program Continue!\r\n");   //如果出错会回到上面，重新开始
	
}

//vl53l0x普通测量模式测试
//dev:设备I2C参数结构体
void vl53l0x_general_test(VL53L0X_Dev_t *dev)
{
	u8 mode=0;
	
	vl53l0x_general_start(dev,mode);
	/*
	while(1)
	{	
		vl53l0x_general_start(dev,mode);
		mode=0;
		
		delay_ms(500);
	}
	*/
}
// 多传感器设备数组

// 多传感器设备数组
VL53L0X_Dev_t vl53l0x_devices[VL53L0X_SENSOR_COUNT];
VL53L0X_RangingMeasurementData_t vl53l0x_datas[VL53L0X_SENSOR_COUNT];
vu16 Distance_datas[VL53L0X_SENSOR_COUNT];

// XSHUT引脚控制
static void vl53l0x_xshut_set(u8 index, u8 state)
{
    switch(index)
    {
        case 0: VL53L0X_XSHUT_1 = state; break;
        case 1: VL53L0X_XSHUT_2 = state; break;
        case 2: VL53L0X_XSHUT_3 = state; break;
        case 3: VL53L0X_XSHUT_4 = state; break;
    }
}

// 多传感器初始化
VL53L0X_Error vl53l0x_multi_init(void)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    u8 i;
    u8 addresses[VL53L0X_SENSOR_COUNT] = {VL53L0X_ADDR_1, VL53L0X_ADDR_2, VL53L0X_ADDR_3, VL53L0X_ADDR_4};
    
    VL53L0X_i2c_init();
    delay_ms(10);
    
    for(i = 0; i < VL53L0X_SENSOR_COUNT; i++)
    {
        vl53l0x_xshut_set(i, 0);
    }
    delay_ms(10);
    
    for(i = 0; i < VL53L0X_SENSOR_COUNT; i++)
    {
        vl53l0x_xshut_set(i, 1);
        delay_ms(10);
        
        vl53l0x_devices[i].I2cDevAddr = VL53L0X_Addr;
        vl53l0x_devices[i].comms_type = 1;
        
        status = VL53L0X_DataInit(&vl53l0x_devices[i]);
        if(status != VL53L0X_ERROR_NONE) 
        {
            printf("Sensor %d DataInit failed\r\n", i+1);
            continue;
        }
        
        status = VL53L0X_SetDeviceAddress(&vl53l0x_devices[i], addresses[i]);
        if(status != VL53L0X_ERROR_NONE)
        {
            printf("Sensor %d SetAddress failed\r\n", i+1);
            continue;
        }
        vl53l0x_devices[i].I2cDevAddr = addresses[i];
        
        printf("Sensor %d init OK, Addr: 0x%02X\r\n", i+1, addresses[i]);
    }
    
    return status;
}

// 读取所有传感器距离
VL53L0X_Error vl53l0x_multi_read(vu16 *distances)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    static char buf[VL53L0X_MAX_STRING_LENGTH];
    u8 i;
    
    for(i = 0; i < VL53L0X_SENSOR_COUNT; i++)
    {
        status = vl53l0x_start_single_test(&vl53l0x_devices[i], &vl53l0x_datas[i], buf);
        if(status == VL53L0X_ERROR_NONE)
        {
            distances[i] = vl53l0x_datas[i].RangeMilliMeter;
        }
        else
        {
            distances[i] = 0xFFFF;
        }
    }
    
    return status;
}
