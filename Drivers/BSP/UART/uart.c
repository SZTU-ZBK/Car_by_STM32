#include "./BSP/uart/uart.h"
UART_HandleTypeDef uart4;
uint8_t rx_buffer[1];
void uart4_init(uint32_t bau)
{
	uart4.Instance=UART4;
	uart4.Init.BaudRate=bau;
	uart4.Init.WordLength= UART_WORDLENGTH_8B;
	uart4.Init.StopBits=UART_STOPBITS_1;
	uart4.Init.Parity=UART_PARITY_NONE;
    uart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart4.Init.Mode=UART_MODE_TX_RX;
	HAL_UART_Init(&uart4);
	HAL_UART_Receive_IT(&uart4,(uint8_t*)rx_buffer,1);

}

void UART4_IRQHandler(void)
{
	HAL_UART_IRQHandler(&uart4);/**接收中断会失能中断，所以需要重新使能一�?*/
	HAL_UART_Receive_IT(&uart4,(uint8_t*)rx_buffer,1);
	//printf("OK___________________________________________________________________________--");
}
