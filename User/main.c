#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/uart/uart.h"
#include "./BSP/LED/led.h"
#include "./BSP/TIMER/gtim.h"
#include "./BSP/ENCODER/encoder.h"
#include "./BSP/MOTOR_CONTROL/motor_control.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/HR04/HR04.h"
extern TIM_HandleTypeDef g_timx_cnt_chy_handle;        /* 定时器x句柄 */
extern TIM_HandleTypeDef g_tim4_pwm_chy_handle; 
extern TIM_HandleTypeDef g_tim8_pwm_chy_handle; 
extern int Target_V_L;
extern int Target_V_R;
extern int len;
extern uint8_t slam;
extern uint8_t stop;
extern uint8_t al_lft;
int main(void)
{
    
    HAL_Init();                                 /* 初始化HAL库 */
	SR04_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟, 72Mhz */
    delay_init(72);                             /* 延时初始化 */
	usart_init(115200);                             /* 串口初始化为115200 */
	Encoder_Init_TIM3();
	Encoder_Init_TIM2();
	exti_init();
	tim6_init(9999, 719);
	tim7_init(9999,359);
	Motor_Control_Init();
	//delay_ms(10);
	uart4_init(9600);
    while (1)
    {  
			if(len<250)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
				if(al_lft==0)
				{
					stop=1;
				}
			}
			else if(len>=250)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
			}
    }
}

