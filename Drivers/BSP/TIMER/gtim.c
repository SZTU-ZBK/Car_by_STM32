#include "./BSP/TIMER/gtim.h"


TIM_HandleTypeDef g_timx_cnt_chy_handle;        /* 定时器x句柄 */

 /* 通用定时器通道y 脉冲计数 初始化函数 */
void gtim_timx_cnt_chy_init(uint16_t psc)
{
    TIM_SlaveConfigTypeDef tim_salve_config = {0};
    
    g_timx_cnt_chy_handle.Instance = TIM2;
    g_timx_cnt_chy_handle.Init.Prescaler = psc;
    g_timx_cnt_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_cnt_chy_handle.Init.Period = 65535;
    HAL_TIM_IC_Init(&g_timx_cnt_chy_handle);
    
    tim_salve_config.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;//外部时钟模式1
    tim_salve_config.InputTrigger = TIM_TS_TI1F_ED;//上升沿和下降沿都触发
    tim_salve_config.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;//向上计数
    tim_salve_config.TriggerFilter = 0;//滤波器设置
    HAL_TIM_SlaveConfigSynchro(&g_timx_cnt_chy_handle, &tim_salve_config);

    HAL_TIM_IC_Start(&g_timx_cnt_chy_handle, TIM_CHANNEL_1);
}

/* 定时器 输入捕获 MSP初始化函数 */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8)
    {
        GPIO_InitTypeDef gpio_init_struct;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_TIM8_CLK_ENABLE();

        gpio_init_struct.Pin = GPIO_PIN_6;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;        /* 推挽式复用功能 */
        gpio_init_struct.Pull = GPIO_PULLDOWN;          /* 下拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;  /* 高速 */
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    }
}






