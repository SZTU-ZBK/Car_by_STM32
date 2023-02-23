#include "./BSP/ENCODER/encoder.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/MOTOR_CONTROL/motor_control.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/HR04/HR04.h"
#include "./BSP/uart/uart.h"
extern uint8_t rx_buffer[1];
int over_count_tim6=0;
int right_flag=0;
int left_flag=0;
int flag_r=0;
int flag_l=0;
int Encoder_Overflow_Count_l=0;//左轮定时器3溢出次数
int Capture_Count_l = 0;//左轮定时器3计数器值
int last_Count_l = 0;//上一周期定时器3计数器值
int Encoder_Overflow_Count_r=0;
int Capture_Count_r = 0;//右轮定时器2计数器值
int last_Count_r = 0;
float Velocity_l = 0;//左轮当前速度
float Velocity_r = 0;
int len=0;
int left_len=0;
int right_len=0;
uint8_t stop=0;
uint8_t slam=0;
uint8_t al_lft=0;
uint8_t stop_set_zero=0;
int stop_set_zero_count=0;
TIM_HandleTypeDef htim_2;//右轮定时器句柄
TIM_HandleTypeDef htim_3;//左轮定时器句柄
extern TIM_HandleTypeDef g_tim4_pwm_chy_handle;//定时器四句柄，用于生成PWM控制左右两个电机
extern TIM_HandleTypeDef g_tim8_pwm_chy_handle;
TIM_HandleTypeDef htim_6;//定时器6句柄,用于定时修改PWM占空比和串口打印速度
TIM_HandleTypeDef htim_7;
int Target_l=0;//左轮目标速度
int Target_V_L=0;
int Target_V_R=0;
int out_r,out_l=0;
float current_r,current_l,tar_pos_r,tar_pos_l=0;
void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef gpio_init_struct = {0};//定义一个引脚初始化的结构体  
    TIM_Encoder_InitTypeDef Tim_2_EncoderConfig; //定义一个定时器编码器模式初始化的结构体
	
	__HAL_RCC_GPIOA_CLK_ENABLE();//使能GPIOA的时钟
	__HAL_RCC_TIM2_CLK_ENABLE();//使能定时器2的时钟
	gpio_init_struct.Pin = GPIO_PIN_0;   
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;        /* 推挽式复用功能 */
	gpio_init_struct.Pull = GPIO_PULLUP;          /* 上拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);
	gpio_init_struct.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);
	
	
    htim_2.Instance = TIM2; 
    htim_2.Init.Prescaler = 0;
    htim_2.Init.CounterMode = TIM_COUNTERMODE_UP; // 模式为向上计数
    htim_2.Init.Period = 60000;                   //溢出值为60000
    htim_2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  //默认为TIM_CLOCKDIVISION_DIV1
    htim_2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;// 使用自动重装载
	
	Tim_2_EncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;//编码器模式3双通道计数
	Tim_2_EncoderConfig.IC1Polarity = TIM_ENCODERINPUTPOLARITY_RISING;//输入极性选择
	Tim_2_EncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;//通道选择，编码器模式线只能为此值
	Tim_2_EncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;//分频系数设置为1
	Tim_2_EncoderConfig.IC1Filter     = 10;            //滤波参数
	Tim_2_EncoderConfig.IC2Polarity = TIM_ENCODERINPUTPOLARITY_RISING;//输入极性选择
	Tim_2_EncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;//通道选择，编码器模式线只能为此值
	Tim_2_EncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;//分频系数设置为1
	Tim_2_EncoderConfig.IC2Filter     = 10;            //滤波参数
	
	HAL_TIM_Encoder_Init(&htim_2,  &Tim_2_EncoderConfig);//编码器模式初始化
	__HAL_TIM_CLEAR_IT(&htim_2,TIM_IT_UPDATE);/* 清零中断标志位 */
	__HAL_TIM_SET_COUNTER(&htim_2, 0);//清零计数器
    __HAL_TIM_ENABLE_IT(&htim_2,TIM_IT_UPDATE);/* 使能定时器的更新事件中断 */
    __HAL_TIM_URS_ENABLE(&htim_2);/* 设置更新事件请求源为：定时器溢出 */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 1);/* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);/* 使能定时器中断 */
    HAL_TIM_Encoder_Start(&htim_2, TIM_CHANNEL_ALL);/* 使能编码器接口 */
}
void Encoder_Init_TIM3(void)
{
	GPIO_InitTypeDef gpio_init_struct = {0};//定义一个引脚初始化的结构体  
    TIM_Encoder_InitTypeDef Tim_3_EncoderConfig; //定义一个定时器编码器模式初始化的结构体
	
	__HAL_RCC_GPIOA_CLK_ENABLE();//使能GPIOA的时钟
	__HAL_RCC_TIM3_CLK_ENABLE();//使能定时器三的时钟
	gpio_init_struct.Pin = GPIO_PIN_6;   
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;        /* 推挽式复用功能 */
	gpio_init_struct.Pull = GPIO_PULLUP;          /* 上拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);
	gpio_init_struct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);
	
	
    htim_3.Instance = TIM3; 
    htim_3.Init.Prescaler = 0;
    htim_3.Init.CounterMode = TIM_COUNTERMODE_UP; // 模式为向上计数
    htim_3.Init.Period = 60000;                   //溢出值为60000
    htim_3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  //默认为TIM_CLOCKDIVISION_DIV1
    htim_3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;// 使用自动重装载
	
	Tim_3_EncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;//编码器模式3双通道计数
	Tim_3_EncoderConfig.IC1Polarity = TIM_ENCODERINPUTPOLARITY_RISING;//输入极性选择
	Tim_3_EncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;//通道选择，编码器模式线只能为此值
	Tim_3_EncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;//分频系数设置为1
	Tim_3_EncoderConfig.IC1Filter     = 10;            //滤波参数
	Tim_3_EncoderConfig.IC2Polarity = TIM_ENCODERINPUTPOLARITY_RISING;//输入极性选择
	Tim_3_EncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;//通道选择，编码器模式线只能为此值
	Tim_3_EncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;//分频系数设置为1
	Tim_3_EncoderConfig.IC2Filter     = 10;            //滤波参数
	
	HAL_TIM_Encoder_Init(&htim_3,  &Tim_3_EncoderConfig);//编码器模式初始化
	__HAL_TIM_CLEAR_IT(&htim_3,TIM_IT_UPDATE);/* 清零中断标志位 */
	__HAL_TIM_SET_COUNTER(&htim_3, 0);//清零计数器
    __HAL_TIM_ENABLE_IT(&htim_3,TIM_IT_UPDATE);/* 使能定时器的更新事件中断 */
    __HAL_TIM_URS_ENABLE(&htim_3);/* 设置更新事件请求源为：定时器溢出 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 1);/* 设置中断优先级 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);/* 使能定时器中断 */
    HAL_TIM_Encoder_Start(&htim_3, TIM_CHANNEL_ALL);/* 使能编码器接口 */
}
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
   /* 判断当前计数器计数方向 */
    if (htim->Instance == TIM6)
    {
		over_count_tim6++;
		len = Get_Len();
		if(stop_set_zero_count==over_count_tim6)
		{
				stop_set_zero=0;
				stop=0;
			    al_lft=0;
		}
		if(over_count_tim6%4==0)
		{
			if(stop==1&&slam==1)
			{
				if(al_lft==0)
				{
					__HAL_TIM_SET_COMPARE(&g_tim8_pwm_chy_handle, TIM_CHANNEL_1, 124);
					al_lft=1;
				}
				else if(al_lft==1)
				{
					left_len=len;
					__HAL_TIM_SET_COMPARE(&g_tim8_pwm_chy_handle, TIM_CHANNEL_1, 24);
					al_lft=2;
				}
				else if(al_lft==2&&stop_set_zero==0)
				{
					right_len=len;
					if(left_len>right_len)
					{
						left_flag=1;
						flag_l=1;
					}
					else
					{
						right_flag=1;
						flag_r=1;
					}
					stop_set_zero=1;
					stop_set_zero_count=over_count_tim6+15;
					__HAL_TIM_SET_COMPARE(&g_tim8_pwm_chy_handle, TIM_CHANNEL_1, 74);
				}
			}
		}
		printf("len=%d___________________________________________________\r\n",len);
		if(len<250&&rx_buffer[0]=='f')
		{
			Target_V_R=0;
			Target_V_L=0;
		}
		else if((len<250&&slam==1)&&(al_lft==0))
		{
			Target_V_R=0;
			Target_V_L=0;
		}
	}	
	else if(htim->Instance == TIM7)
	{
		if(slam==1&&stop==0)
		{
			Target_V_R=200;
			Target_V_L=200;
		}
		Capture_Count_l =__HAL_TIM_GET_COUNTER(&htim_3) + (Encoder_Overflow_Count_l * 60000);/* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * 计数器溢出值  */
		Velocity_l=(Capture_Count_l-last_Count_l)*3.1416*65/(60*50);
		last_Count_l=Capture_Count_l;		
		Capture_Count_r =__HAL_TIM_GET_COUNTER(&htim_2) + (Encoder_Overflow_Count_r * 60000);/* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * 计数器溢出值  */
		Velocity_r=((Capture_Count_r-last_Count_r)*3.1416*65)/(60*50);
		last_Count_r=Capture_Count_r;
		printf("当前右轮编码器数值：%d\r\n", Capture_Count_r);
		printf("当前右轮电机转速：%fm/s\r\n", Velocity_r);
		out_r = Motor_PI_R(Velocity_r,Target_V_R);
		current_l=__HAL_TIM_GET_COUNTER(&htim_3) + (Encoder_Overflow_Count_l * 60000);
		out_l = Motor_PI(Velocity_l,Target_V_L);
		if(right_flag==1&&flag_r==1)
		{
			tar_pos_l=Capture_Count_l+71000;
			flag_r=0;
		}
		if(right_flag==1)
		{
			if(tar_pos_l-current_l<2000&&tar_pos_l-current_l>-2000)
			{
				Target_V_L=0;
				right_flag=0;
			}
			else
			{
				Target_V_L=Pos_PI_L(current_l,tar_pos_l);				
			}
		}
		current_r=__HAL_TIM_GET_COUNTER(&htim_2) + (Encoder_Overflow_Count_r * 60000);
		if(left_flag==1&&flag_l==1)
		{
			tar_pos_r=Capture_Count_r+68000;
			flag_l=0;
		}
		if(left_flag==1)
		{
			if(tar_pos_r-current_r<2000&&tar_pos_r-current_r>-2000)
			{
				Target_V_R=0;
				left_flag=0;
			}
			else
			{
				Target_V_R=Pos_PI_R(current_r,tar_pos_r);				
			}
		}
		if(out_l>0)
		{
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_1, (7199-out_l));
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_2, 7199);
		}
		else if(out_l<=0)
		{
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_1, 7199);
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_2, (7199+out_l));
		}
		if(out_r>0)
		{
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_3, (7199-out_r));
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_4, 7199);
		}
		else if(out_r<=0)
		{
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_3, 7199);
			__HAL_TIM_SET_COMPARE(&g_tim4_pwm_chy_handle, TIM_CHANNEL_4, (7199+out_r));
		}
    }
	if(htim->Instance == TIM3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim_3))
		{
			Encoder_Overflow_Count_l--;
		}
        else if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim_3)!=1)
		{
			Encoder_Overflow_Count_l++;
	    }
	}
	else if(htim->Instance == TIM2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim_2))
		{
			Encoder_Overflow_Count_r--;
		}
        else if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim_2)!=1)
		{
			printf("进入中断————————————————————————————");
			Encoder_Overflow_Count_r++;
	    }
	}
	//printf("进入中断\r\n");
}
/* 定时器中断初始化函数 */
void tim6_init(uint16_t arr, uint16_t psc)
{
    htim_6.Instance = TIM6;
    htim_6.Init.Prescaler = psc;
    htim_6.Init.Period = arr;
    HAL_TIM_Base_Init(&htim_6);//定时器初始化

    HAL_TIM_Base_Start_IT(&htim_6);//使能中断
}
void tim7_init(uint16_t arr, uint16_t psc)
{
    htim_7.Instance = TIM7;
    htim_7.Init.Prescaler = psc;
    htim_7.Init.Period = arr;
    HAL_TIM_Base_Init(&htim_7);//定时器初始化

    HAL_TIM_Base_Start_IT(&htim_7);//使能中断
}
/* 定时器基础MSP初始化函数 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(TIM6_IRQn);
	    __HAL_TIM_SET_COUNTER(&htim_6, 0);//清零计数器
		__HAL_TIM_CLEAR_IT(&htim_6,TIM_IT_UPDATE);/* 清零中断标志位 */
		__HAL_TIM_ENABLE_IT(&htim_6,TIM_IT_UPDATE);/* 使能定时器的更新事件中断 */
		__HAL_TIM_URS_ENABLE(&htim_6);/* 设置更新事件请求源为：定时器溢出 */
    }
    if (htim->Instance == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 2, 1);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
	    __HAL_TIM_SET_COUNTER(&htim_7, 0);//清零计数器
		__HAL_TIM_CLEAR_IT(&htim_7,TIM_IT_UPDATE);/* 清零中断标志位 */
		__HAL_TIM_ENABLE_IT(&htim_7,TIM_IT_UPDATE);/* 使能定时器的更新事件中断 */
		__HAL_TIM_URS_ENABLE(&htim_7);/* 设置更新事件请求源为：定时器溢出 */
    }
}


/* 定时器6中断服务函数 */
void TIM6_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_6);
}
void TIM7_IRQHandler(void)
{
	printf("OK\r\n");
    HAL_TIM_IRQHandler(&htim_7);
}
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_3);
}
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim_2);
}
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_tim4_pwm_chy_handle);
}
