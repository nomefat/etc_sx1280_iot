#ifndef __USB_KEY_H
#define __USB_KEY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

#define USB_KEY_TEST         0
#define IIC_SIMULATION       1   // 1 - 使用IO模拟IIC  0 - 使用MCU的IIC接口
	 
#pragma pack(1)
typedef	struct _id_crc{
		uint16_t s_id;
		uint8_t crc;
	}*pstruct_id_crc;	 
#pragma pack()	 
	 
typedef  struct 
{
	uint16_t GPIO_Pin;
	GPIO_TypeDef* GPIOx;
}USB_KEY;

typedef enum 
{ 
	PWR_ON = 1,
	PWR_OFF
}PWR_STATE;

typedef enum 
{ 
	USB_KEY1 = 0,
	USB_KEY2,
	USB_KEY3,
	USB_KEY4,
	USB_KEY5,
	USB_KEY6,
	USB_KEY7
}USB_KEY_NO_;

#ifdef __cplusplus
}
#endif

void AT_24C01_init();
void UsbKey_Poll();
void SetUseKey_All();
void Rs232_Cmd_Proc();
void start_rs232_dma_receive();
void uart7_from_debug_idle_callback();
#endif

