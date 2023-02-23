#include "./BSP/HR04/HR04.h"
#include "./BSP/ENCODER/encoder.h"
#include "./SYSTEM/delay/delay.h"
extern int over_count_tim6;
extern TIM_HandleTypeDef htim_6;
float last,current,length=0;
float Get_Len(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	delay_us(15);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==0);
	last=(float)(__HAL_TIM_GET_COUNTER(&htim_6))/100.0+ (float)over_count_tim6*100;
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)==1);
	current=(float)(__HAL_TIM_GET_COUNTER(&htim_6))/100.0+ over_count_tim6*100;
	length=(float)(current-last)*340/2.0;
	return length;
}