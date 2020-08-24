/**
  ******************************************************************************
  * File Name          : usb_key.c
  * Description        : This file provides code for the AT24C01C reading and
	*                      writing by I2C.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "usb_key.h"
#include "etc_rf.h"


#define USB_KEY_NUM           7
static USB_KEY  USB_KEY_POOL[USB_KEY_NUM] =
{
	{USB_KEY_VCC_EN1_Pin, USB_KEY_VCC_EN1_GPIO_Port},
	{USB_KEY_VCC_EN2_Pin, USB_KEY_VCC_EN2_GPIO_Port},
	{USB_KEY_VCC_EN3_Pin, USB_KEY_VCC_EN3_GPIO_Port},
	{USB_KEY_VCC_EN4_Pin, USB_KEY_VCC_EN4_GPIO_Port},
	{USB_KEY_VCC_EN5_Pin, USB_KEY_VCC_EN5_GPIO_Port},
	{USB_KEY_VCC_EN6_Pin, USB_KEY_VCC_EN6_GPIO_Port},
	{USB_KEY_VCC_EN7_Pin, USB_KEY_VCC_EN7_GPIO_Port}

};

#define I2C_SCL_H  HAL_GPIO_WritePin(USB_KEY_IIC2_SCL_GPIO_Port, USB_KEY_IIC2_SCL_Pin, GPIO_PIN_SET)
#define I2C_SDA_H  HAL_GPIO_WritePin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin, GPIO_PIN_SET)
#define I2C_SCL_L  HAL_GPIO_WritePin(USB_KEY_IIC2_SCL_GPIO_Port, USB_KEY_IIC2_SCL_Pin, GPIO_PIN_RESET)
#define I2C_SDA_L  HAL_GPIO_WritePin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA   HAL_GPIO_ReadPin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin)


// AT24C01 Ïà¹ØµØÖ·
#define    READ_DATA_CMD    0xA1
#define    WRTIE_DATA_CMD   0xA0

#define    WORD_ADDRESS     0x00     // Bit 7 is a "don't care" bit on the AT24C01C


extern uint8_t ap_param_write_flash_flag;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t RxBuf[16];
extern uint8_t TxBuf[16] ;
extern uint8_t usb_key_5sec;
/**
  * Func: ¸÷Â·USB_KEYµÄµçÔ´¿ØÖÆ
	* Param: usb_key_num        - USB_KEY µÄÐòºÅ(USB_KEY1 ~ USB_KEY7)
	*        usb_key_pwr_state  - USB_KEYµÄµçÔ´¿ØÖÆ×´Ì¬(PWR_ON - PWR_OFF)
	*/
extern void Pwr_Ctl(USB_KEY_NO_ usb_key_num, PWR_STATE usb_key_pwr_state);

extern struct_sensor_list etc_sensor_list[IO_COUNT];
//struct_sensor_list etc_sensor_list_1[IO_COUNT];
static USB_KEY_NO_ usb_key_list[USB_KEY_NUM] = {USB_KEY1, USB_KEY2, USB_KEY3, USB_KEY4, USB_KEY5, USB_KEY6, USB_KEY7};






void I2C_SDA_OUT()
{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};

//	__HAL_RCC_GPIOH_CLK_ENABLE();
//	/**I2C2 GPIO Configuration    
//	PH4     ------> I2C2_SCL
//	PH5     ------> I2C2_SDA 
//	*/
//	GPIO_InitStruct.Pin = USB_KEY_IIC2_SCL_Pin|USB_KEY_IIC2_SDA_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}



void I2C_SDA_IN(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};

//	__HAL_RCC_GPIOH_CLK_ENABLE();
//	/**I2C2 GPIO Configuration    
//	PH4     ------> I2C2_SCL
//	PH5     ------> I2C2_SDA 
//	*/
//	GPIO_InitStruct.Pin = USB_KEY_IIC2_SCL_Pin|USB_KEY_IIC2_SDA_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

void delay_us(uint32_t ti)
{
	uint32_t tm = 0;
	while(tm++<=ti*168*2);
}

void I2C_Start(void)
{
	I2C_SDA_OUT();
 
    I2C_SDA_H;
    I2C_SCL_H;
    delay_us(5);
    I2C_SDA_L;
    delay_us(6);
    I2C_SCL_L;
}


void I2C_Stop(void)
{
	I2C_SDA_OUT();
	 
	 
	I2C_SCL_L;
	I2C_SDA_L;
	I2C_SCL_H;
	delay_us(6);
	I2C_SDA_H;
	delay_us(6);
}


void I2C_Ack(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_L;
   delay_us(2);
   I2C_SCL_H;
   delay_us(5);
   I2C_SCL_L;
}

void I2C_NAck(void)
{
   I2C_SCL_L;
   I2C_SDA_OUT();
   I2C_SDA_H;
   delay_us(2);
   I2C_SCL_H;
   delay_us(5);
   I2C_SCL_L;
}



uint8_t I2C_Wait_Ack(void)
{
    uint8_t tempTime=0;
 
    I2C_SDA_IN();
    I2C_SDA_H;
    delay_us(1);
    I2C_SCL_H;
    delay_us(1);
 
 
    while(READ_SDA)
    {
        tempTime++;
        if(tempTime>250)
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_SCL_L;
    return 0;
}



void I2C_Send_Byte(uint8_t txd)
{
    uint8_t i=0;
 
    I2C_SDA_OUT();
    I2C_SCL_L;
 
    for(i=0;i<8;i++)
    {
        if((txd&0x80)>0) //0x80  1000 0000
            I2C_SDA_H;
        else
            I2C_SDA_L;
 
        txd<<=1;
        I2C_SCL_H;
        delay_us(2); 
        I2C_SCL_L;
        delay_us(2);
    }
}


uint8_t I2C_Read_Byte(uint8_t ack)
{
	uint8_t i=0,receive=0;
	 
	I2C_SDA_IN();
	for(i=0;i<8;i++)
	{
			 I2C_SCL_L;
					delay_us(2);
					I2C_SCL_H;
					receive<<=1;
					if(READ_SDA)
					 receive++;
					delay_us(1); 
	}
	if(ack==0)
   I2C_NAck();
    else
      I2C_Ack();
    return receive;
}


uint8_t AA24x_ReadOneByte(uint8_t addr)
{
    uint8_t temp=0;
    I2C_Start(); 
    I2C_Send_Byte(0xA0);//1010000
    I2C_Wait_Ack();
    I2C_Send_Byte(addr);
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte(0xA1);//10100001
    I2C_Wait_Ack();
    temp=I2C_Read_Byte(0); 
    I2C_NAck();
    I2C_Stop(); 
    return temp; 

}



//Ë³Ðò¶ÁÈ¡¶à¸ö×Ö½Ú
uint8_t AA24x_seq_read(uint8_t data_addr, uint8_t *data, uint8_t da_len)
{
	uint8_t rd_data = 0;
	
	if (da_len == 0)
		return 0;
	I2C_Start();
	I2C_Send_Byte(WRTIE_DATA_CMD);
	I2C_Wait_Ack();
	
	I2C_Send_Byte(data_addr);
  I2C_Wait_Ack();
	
	I2C_Start();
	I2C_Send_Byte(READ_DATA_CMD);
	I2C_Wait_Ack();
	
	while(da_len--)
	{
		*data = I2C_Read_Byte(1);
		data++;
		rd_data++;
		//I2C_Ack();
	}
	I2C_NAck();
	I2C_Stop();
	return rd_data;	
}











