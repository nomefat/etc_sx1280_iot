#ifndef __IO_LED_H
#define __IO_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}IO_PIN_STRUCT;

typedef struct 
{
	GPIO_TypeDef* GPIOx_G;
	uint16_t GPIO_Pin_G;
	
}CH_LED;	

typedef struct 
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
}TEST_LED;	

typedef enum
{
	RED = 1,
	GREEN,
	LEDOFF
}LED_ST;

typedef enum
{
	SON = 1,
	SOFF
}IO_ST;

#ifdef __cplusplus
}
#endif

void Led_Test();
void All_Channel_Led_Off();
void ALL_Channel_IO_Off();
void Sensor_Norm_Proc(uint8_t ch);
void Sensor_RfTimeout_Proc(uint16_t timeout_count);
void Channel_Led_Op(uint8_t ch, LED_ST led_st);
void IO_Op(uint8_t ch, IO_ST io_st);
void Channel_IO_Proc();
void Watch_Dog_Reset();
void test_led_switch(uint8_t led,uint8_t on_off);



#endif


