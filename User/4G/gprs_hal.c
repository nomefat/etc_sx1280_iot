#include "stm32f4xx_hal.h"
#include "string.h"
#include "gprs_hal.h"
#include "main.h"



#define GPRS_RCV_DMA_BUFF_LENGTH (1024+512)  // gprs串口接收


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t gprs_receive_dma_buff[GPRS_RCV_DMA_BUFF_LENGTH];



extern void gprs_str_copy_to_queue(unsigned short len,char* p_data);


/**********************************************************************************************
***func: 用于单片第一次开启DMA接收
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_gprs_dma_receive()
{
	SET_BIT((&huart2)->Instance->CR1, USART_CR1_IDLEIE);  //打开串口空闲中断
	HAL_UART_Receive_DMA(&huart2,gprs_receive_dma_buff,GPRS_RCV_DMA_BUFF_LENGTH);	 //打开DMA接收
}




/**********************************************************************************************
***func:串口空闲中断回调函数
***     空闲中断用来判断一包数据的结束
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_gprs_idle_callback()
{
	HAL_DMA_Abort((&huart2)->hdmarx);
	huart2.RxState = HAL_UART_STATE_READY;
	huart2.hdmarx->State = HAL_DMA_STATE_READY;
	gprs_str_copy_to_queue(GPRS_RCV_DMA_BUFF_LENGTH-DMA1_Stream5->NDTR,(char*)gprs_receive_dma_buff);

	//HAL_UART_Transmit_DMA(&huart3,gprs_receive_dma_buff,GPRS_RCV_DMA_BUFF_LENGTH-DMA1_Stream5->NDTR);	
	
	HAL_UART_Receive_DMA(&huart2,gprs_receive_dma_buff,GPRS_RCV_DMA_BUFF_LENGTH);	 //打开DMA接收


	
}



void grps_power_off()
{
	HAL_GPIO_WritePin(L506_PWR_EN_GPIO_Port,L506_PWR_EN_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_1_8V_GPIO_Port,EN_1_8V_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(L506_LOGIC_OE_GPIO_Port,L506_LOGIC_OE_Pin,GPIO_PIN_SET);
}

void grps_power_on()
{
	HAL_GPIO_WritePin(EN_1_8V_GPIO_Port,EN_1_8V_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(L506_LOGIC_OE_GPIO_Port,L506_LOGIC_OE_Pin,GPIO_PIN_SET);	
	HAL_GPIO_WritePin(L506_PWR_EN_GPIO_Port,L506_PWR_EN_Pin,GPIO_PIN_SET);

	HAL_GPIO_WritePin(L506_PWR_KEY_GPIO_Port,L506_PWR_KEY_Pin,GPIO_PIN_RESET);
	

}








void led_1_close()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}









