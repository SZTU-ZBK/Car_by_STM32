#include "./BSP/MOTOR_CONTROL/motor_control.h"
TIM_HandleTypeDef g_tim4_pwm_chy_handle;
TIM_OC_InitTypeDef tim4_oc_pwm_chy; 
TIM_HandleTypeDef g_tim8_pwm_chy_handle;
TIM_OC_InitTypeDef tim8_oc_pwm_chy;
float Velocity_KP=7.0;
float Velocity_KI=3.6;
float Velocity_KD=0;
int Bias_R,Pwm_R,Last_bias_R,Bias_I_R=0;
int Bias,Pwm,Last_bias,Bias_I=0;

float Pos_KP=0.008;
float Pos_KI=0.0;
float Pos_KD=0;

int Pos_Bias_R,Pos_V_R,Pos_Last_bias_R,Pos_Bias_I_R=0;
int Pos_Bias_L,Pos_V_L,Pos_Last_bias_L,Pos_Bias_I_L=0;
int Motor_PI_R (float current,float Target)
{ 	
	 Bias_R=Target-current;                //计算偏差
	 printf("Bias_R=%d\r\n",Bias_R);
	 Bias_I_R+=Bias_R;
	 if(Bias_I_R>8500)
	 {
		 Bias_I_R=8500;
	 }
	 else if(Bias_I_R<-8000)
	 {
		 Bias_I_R=-8000;
	 }
	 Pwm_R=Velocity_KP*Bias_R+Velocity_KI*(Bias_I_R)+Velocity_KD*(Bias_R-Last_bias_R);   //PI控制器
	 printf("Pwm_R=%d\r\n",Pwm_R);
	 Last_bias_R=Bias_R;
	if(Pwm_R>7100)
	 {
		 Pwm_R=7100;
	 }
	 else if(Pwm_R<-7100)
	 {
		 Pwm_R=-7100;
	 }
	 return Pwm_R;                         //输出PWM
}
int Motor_PI (float current,float Target)
{ 	
	 Bias=Target-current;                //计算偏差
	 printf("Bias=%d\r\n",Bias);
	 Bias_I+=Bias;
	 if(Bias_I>8500)
	 {
		 Bias_I=8500;
	 }
	 else if(Bias_I<-8000)
	 {
		 Bias_I=-8000;
	 }
	 Pwm=Velocity_KP*Bias+Velocity_KI*(Bias_I)+Velocity_KD*(Bias-Last_bias);   //PI控制器
	 printf("Pwm=%d\r\n",Pwm);
	 Last_bias=Bias;
	if(Pwm>7100)
	 {
		 Pwm=7100;
	 }
	 else if(Pwm<-7100)
	 {
		 Pwm=-7100;
	 }
	 return Pwm;                         //输出PWM
}
int Pos_PI_L (float current,float Target)
{ 	
	 Pos_Bias_L=Target-current;                //计算偏差
	 Pos_Bias_I_L+=Pos_Bias_L;
	 Pos_V_L=Pos_KP*Pos_Bias_L;   //PI控制器
	 Pos_Last_bias_L=Pos_Bias_L;
	if(Pos_V_L>300)
	 {
		 Pos_V_L=300;
	 }
	 else if(Pos_V_L<-300)
	 {
		 Pos_V_L=-300;
	 }
	 return Pos_V_L;                         //输出速度
}
int Pos_PI_R (float current,float Target)
{ 	
	 Pos_Bias_R=Target-current;                //计算偏差
	 printf("Pos_Bias_R=%d\r\n",Pos_Bias_R);
	 Pos_Bias_I_R+=Pos_Bias_R;
	 Pos_V_R=Pos_KP*Pos_Bias_R;   //PI控制器
	 Pos_Last_bias_R=Pos_Bias_R;
	if(Pos_V_R>300)
	 {
		 Pos_V_R=300;
	 }
	 else if(Pos_V_R<-300)
	 {
		 Pos_V_R=-300;
	 }
	 return Pos_V_R;                         //输出速度
}
void Motor_Control_Init(void)
{
    g_tim4_pwm_chy_handle.Instance = TIM4;
    g_tim4_pwm_chy_handle.Init.Prescaler = 0;
    g_tim4_pwm_chy_handle.Init.Period = 7199;//设置PWM频率10KHz
    g_tim4_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	g_tim4_pwm_chy_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	g_tim4_pwm_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&g_tim4_pwm_chy_handle);
    
    tim4_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;
    tim4_oc_pwm_chy.Pulse = 0;
    tim4_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;
	
    HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_chy_handle, &tim4_oc_pwm_chy, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_tim4_pwm_chy_handle, TIM_CHANNEL_1);		
    HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_chy_handle, &tim4_oc_pwm_chy, TIM_CHANNEL_2);	
    HAL_TIM_PWM_Start(&g_tim4_pwm_chy_handle, TIM_CHANNEL_2);	
    HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_chy_handle, &tim4_oc_pwm_chy, TIM_CHANNEL_3);	
    HAL_TIM_PWM_Start(&g_tim4_pwm_chy_handle, TIM_CHANNEL_3);	
    HAL_TIM_PWM_ConfigChannel(&g_tim4_pwm_chy_handle, &tim4_oc_pwm_chy, TIM_CHANNEL_4);	
    HAL_TIM_PWM_Start(&g_tim4_pwm_chy_handle, TIM_CHANNEL_4);


    g_tim8_pwm_chy_handle.Instance = TIM8;
    g_tim8_pwm_chy_handle.Init.Prescaler = 1439;
    g_tim8_pwm_chy_handle.Init.Period = 999;//设置PWM频率50Hz
    g_tim8_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	g_tim8_pwm_chy_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	g_tim8_pwm_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&g_tim8_pwm_chy_handle);
    
    tim8_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;
    tim8_oc_pwm_chy.Pulse = 74;//24right 74 124left
    tim8_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;
	
    HAL_TIM_PWM_ConfigChannel(&g_tim8_pwm_chy_handle, &tim8_oc_pwm_chy, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&g_tim8_pwm_chy_handle, TIM_CHANNEL_1);		
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        GPIO_InitTypeDef gpio_init_struct;
        __HAL_RCC_TIM4_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();                                

		gpio_init_struct.Pin = GPIO_PIN_6;                   /* LED0引脚 */
		gpio_init_struct.Mode = GPIO_MODE_AF_PP;;            /* 推挽输出 */
		gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
		HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化LED0引脚 */
		gpio_init_struct.Pin = GPIO_PIN_7;                   /* LED0引脚 */     
		HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化LED0引脚 */	
		gpio_init_struct.Pin = GPIO_PIN_8;                   /* LED0引脚 */     
		HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化LED0引脚 */	
		gpio_init_struct.Pin = GPIO_PIN_9;                   /* LED0引脚 */     
		HAL_GPIO_Init(GPIOB, &gpio_init_struct);       /* 初始化LED0引脚 */	
    }
    if(htim->Instance == TIM8)
    {
        GPIO_InitTypeDef gpio_init_struct;
        __HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();                                

		gpio_init_struct.Pin = GPIO_PIN_6;                   /* LED0引脚 */
		gpio_init_struct.Mode = GPIO_MODE_AF_PP;;            /* 推挽输出 */
		gpio_init_struct.Pull = GPIO_PULLDOWN;                    /* 下拉 */
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
		HAL_GPIO_Init(GPIOC, &gpio_init_struct);       /* 初始化LED0引脚 */
		printf("_________________________________________________________-");
    }
}
