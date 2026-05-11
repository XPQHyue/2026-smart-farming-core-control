#include "bsp_key.h"

void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//用户按键
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//暂停按键
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
} 

//按键扫描函数
//在内部执行该函数的频率,和滤波次数
//返回值：long_click、double_click、single_click、key_stateless
u8 KEY_Scan(u16 Frequency,u16 filter_times)
{
    static u16 time_core;//计时变量
    static u16 long_press_time;//长按计时
    static u8 press_flag=0;//按下是否标记
    static u8 check_once=0;//是否已经标记1次标志
    static u16 delay_mini_1;
    static u16 delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//运行1次需要多少毫秒

    if(check_once)//已经标记,变量清空
    {
        press_flag=0;//按下1时标志清空
        time_core=0;//按下1时标志时间清空
        long_press_time=0;//按下1时标志时间清空
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY_PIN==1) check_once=0; //全扫描按键释放后，下一次才是一次扫描

    if(KEY_PIN==0&&check_once==0)//按下扫描
    {
        press_flag=1;//标记按下1次
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(u16)(600.0f/Count_time))// 长按1次
    {	
        check_once=1;//标记已标志
        return long_click; //返回
    }

    //按下后等待1次值的峰，可能在后面时间
    if(press_flag&&KEY_PIN==1)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(500.0f/Count_time)))//50~700ms内在次按下
    {
        if(KEY_PIN==0) //检测再次按下
        {
            check_once=1;//标记已标志
            return double_click; //记为双击
        }
    }
    else if(press_flag&&time_core>(u16)(500.0f/Count_time))
    {
        check_once=1;//标记已标志
        return single_click; //800ms没再按下，单击的
    }

    return key_stateless;
}
