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

#define IIC_SCL_H  HAL_GPIO_WritePin(USB_KEY_IIC2_SCL_GPIO_Port, USB_KEY_IIC2_SCL_Pin, GPIO_PIN_SET)
#define IIC_SDA_H  HAL_GPIO_WritePin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin, GPIO_PIN_SET)
#define IIC_SCL_L  HAL_GPIO_WritePin(USB_KEY_IIC2_SCL_GPIO_Port, USB_KEY_IIC2_SCL_Pin, GPIO_PIN_RESET)
#define IIC_SDA_L  HAL_GPIO_WritePin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin, GPIO_PIN_RESET)
#define READ_SDA   HAL_GPIO_ReadPin(USB_KEY_IIC2_SDA_GPIO_Port, USB_KEY_IIC2_SDA_Pin)


// AT24C01 Ïà¹ØµØÖ·
#define    READ_DATA_CMD    0xA1
#define    WRTIE_DATA_CMD   0xA0

#define    WORD_ADDRESS     0x00     // Bit 7 is a "don't care" bit on the AT24C01C

extern uint8_t ap_param_write_flash_flag;
extern I2C_HandleTypeDef hi2c2;
uint8_t RxBuf[16];
uint8_t TxBuf[16] = {1,2,3,4,5,6,7,8};
uint8_t usb_key_5sec = 1;
/**
  * Func: ¸÷Â·USB_KEYµÄµçÔ´¿ØÖÆ
	* Param: usb_key_num        - USB_KEY µÄÐòºÅ(USB_KEY1 ~ USB_KEY7)
	*        usb_key_pwr_state  - USB_KEYµÄµçÔ´¿ØÖÆ×´Ì¬(PWR_ON - PWR_OFF)
	*/
void Pwr_Ctl(USB_KEY_NO_ usb_key_num, PWR_STATE usb_key_pwr_state)
{
	if(usb_key_pwr_state == PWR_ON)
		HAL_GPIO_WritePin(USB_KEY_POOL[usb_key_num].GPIOx, USB_KEY_POOL[usb_key_num].GPIO_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(USB_KEY_POOL[usb_key_num].GPIOx, USB_KEY_POOL[usb_key_num].GPIO_Pin, GPIO_PIN_RESET);
}

extern struct_sensor_list etc_sensor_list[IO_COUNT];
//struct_sensor_list etc_sensor_list_1[IO_COUNT];
static USB_KEY_NO_ usb_key_list[USB_KEY_NUM] = {USB_KEY1, USB_KEY2, USB_KEY3, USB_KEY4, USB_KEY5, USB_KEY6, USB_KEY7};

#if IIC_SIMULATION
// ------- Ê¹ÓÃIOÄ£Äâ I2C µÄÊ±Ðò¶ÁÐ´AT24C01 ---------//

void delau_us(uint32_t ti)
{
	uint32_t tm = 0;
	while(tm++<=ti*168);
}
uint8_t Xor_Cal(uint8_t *da, uint8_t len)
{
	uint8_t i;
	uint8_t xor = *da;
	
	for(i=1; i<len; i++)
	{
		xor ^= *(++da);
	}
	return xor;
}
//IICÆðÊ¼ÐÅºÅ
void IIC_Start(void)
{
    IIC_SCL_H;
    IIC_SDA_H;
    delau_us(5);
    IIC_SDA_L;
    delau_us(5);
    IIC_SCL_L;
}

//IICÖÕÖ¹ÐÅºÅ
void IIC_Stop(void)
{
   IIC_SDA_L;
   IIC_SCL_L;
   delau_us(5);
	
   IIC_SCL_H;
   delau_us(4);
   IIC_SDA_H;
   delau_us(5);
}

//µÈ´ýACK

uint8_t IIC_Wait_Ack(void)
{
    uint8_t flag_ack=0;
    IIC_SCL_L;
    IIC_SDA_H;
    delau_us(5);
	
    IIC_SCL_H;
    delau_us(5);
    while(READ_SDA && flag_ack<4)
    {
       flag_ack++;
       delau_us(1);
    }
    if(flag_ack>=4)
    {
     IIC_Stop();
     return 1;
 
    }
    IIC_SCL_L;
    return 0;
}

// ²úÉúÓ¦´ðÐÅºÅ

void IIC_Ack(void)
{
   IIC_SCL_L;
   IIC_SDA_L;
   delau_us(5);
	 
   IIC_SCL_H;
   delau_us(5);
	 IIC_SDA_H;
 
}

void IIC_NAck(void)
{
    IIC_SCL_L;
    IIC_SDA_H;
    delau_us(5);
	
    IIC_SCL_H;
    delau_us(5); 
}

// ·¢ËÍ-½ÓÊÕ

void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t=0; 
    /*Ê±ÖÓÐÅºÅÀ­µÍ£¬Êý¾Ý×¼±¸ºÃÔÙÀ­¸ß½øÐÐ´«Êä*/    
    for(t = 0; t < 8; t++)
    {     
			 IIC_SCL_L;
			 if(txd&0x80)
					IIC_SDA_H;
			 else
					IIC_SDA_L;      
			 delau_us(5);   
        
			/*SCLÀ­¸ß´«ÊäÊý¾Ý*/
			IIC_SCL_H;
			delau_us(5);
			txd <<= 1;   
    }	 
}
 
uint8_t IIC_Read_Byte(void)
{
   uint8_t receive = 0;
   uint8_t i=8; 
   for(i = 0; i < 8; i++ )
   {
			/*SCLÀ­µÍ*/
			IIC_SCL_L;   
			delau_us(4);  
			/*À­¸ßscl²úÉúÒ»¸öÓÐÐ§µÄÊ±ÖÓÐÅºÅ*/
			IIC_SCL_H;
			/*¶ÁÈ¡×ÜÏßÉÏµÄÊý¾Ý*/
			receive<<=1;
			
			if(READ_SDA)
				receive|=1; 
			delau_us(4); 
    }
    return receive;

}

/**
 * Func: µ¥×Ö½Ú¶ÁÐ´
 * Param: data_addr µØÖ·    data  Êý¾Ý
 */
void AT_24c01_write_data(uint8_t data_addr,uint8_t data)
{
   IIC_Start();
	 IIC_Send_Byte(WRTIE_DATA_CMD);
	 IIC_Wait_Ack();
	
	 IIC_Send_Byte(data_addr);
	 IIC_Wait_Ack();
	
	 IIC_Send_Byte(data);
	 IIC_Wait_Ack();
	 IIC_Stop();
}
uint8_t AT_24c01_read_data(uint8_t data_addr)
{
  uint8_t data=0;
	
	IIC_Start();
	IIC_Send_Byte(WRTIE_DATA_CMD);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(data_addr);
  IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(READ_DATA_CMD);
	IIC_Wait_Ack();
	
	data=IIC_Read_Byte();
	IIC_NAck();
	IIC_Stop();
	return data;	
}

//°´Ò³Ð´Èë£¬×î¶à8¸ö×Ö½Ú
void AT_24c01_write_page(uint8_t data_addr,uint8_t *data, uint8_t da_len)
{
	 if(da_len > 8)
		 da_len = 8;
	 
   IIC_Start();
	 IIC_Send_Byte(WRTIE_DATA_CMD);
	 IIC_Wait_Ack();
	
	 IIC_Send_Byte(data_addr);
	 IIC_Wait_Ack();
	
	 while(da_len--)
	 {
		 IIC_Send_Byte(*data++);
		 IIC_Wait_Ack();
	 }
	 IIC_Stop();
}
//Ë³Ðò¶ÁÈ¡¶à¸ö×Ö½Ú
uint8_t AT_24c01_seq_read(uint8_t data_addr, uint8_t *data, uint8_t da_len)
{
	uint8_t rd_data = 0;
	
	if (da_len == 0)
		return 0;
	IIC_Start();
	IIC_Send_Byte(WRTIE_DATA_CMD);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(data_addr);
  IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(READ_DATA_CMD);
	IIC_Wait_Ack();
	
	while(da_len--)
	{
		*data = IIC_Read_Byte();
		data++;
		rd_data++;
		IIC_Ack();
	}
	IIC_NAck();
	IIC_Stop();
	return rd_data;	
}

void AT_24C01_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOH_CLK_ENABLE();
	/**I2C2 GPIO Configuration    
	PH4     ------> I2C2_SCL
	PH5     ------> I2C2_SDA 
	*/
	GPIO_InitStruct.Pin = USB_KEY_IIC2_SCL_Pin|USB_KEY_IIC2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}
#else
// ------- Ê¹ÓÃÐ¾Æ¬µÄI2C½Ó¿Ú¶ÁÐ´AT24C01 ---------//
// £¿----  Ä¿Ç°²âÊÔÓÐÎÊÌâ£¬¶Á³öÀ´È«ÊÇFF ---- £¿//
/**
* Func: AT24C01Êý¾Ý¶ÁÈ¡
  *
  */
int8_t Read_AT2401(uint8_t usb_key_no, uint8_t *pData, uint8_t Size)
{
	HAL_StatusTypeDef StatusRet;
	int8_t res = -1;
	uint32_t timeout = 1;
	
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_ON);
	
	//ÏÈÐ´Èë Word AddressByte
	TxBuf[0] = WORD_ADDRESS;	
//	HAL_I2C_Master_Transmit_IT(&hi2c2, WRTIE_DATA_CMD, TxBuf, 1);
	HAL_I2C_Master_Transmit(&hi2c2, WRTIE_DATA_CMD, TxBuf, 1, timeout);
	
	//¶ÁÈ¡Ö¸¶¨ Word AddressByte Êý¾Ý
//	StatusRet = HAL_I2C_Master_Receive_IT(&hi2c2, READ_DATA_CMD, RxBuf, 2);
	StatusRet = HAL_I2C_Master_Receive(&hi2c2, READ_DATA_CMD, pData, Size, timeout);
	if(StatusRet == HAL_OK)
	{
		res = StatusRet;
	}
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_OFF);
	return res;
}

/**
* Func: ÏòAT24C01Ð´ÈëÊý¾Ý
  *
  */
int8_t Write_AT2401(uint8_t usb_key_no, uint8_t *pData, uint8_t Size)
{
	HAL_StatusTypeDef StatusRet;
	int8_t res = -1;
	
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_ON);
	
	//Ð´Èë ¡®¡¯Word AddressByte + Êý¾Ý ¡®¡¯
	TxBuf[0] = WORD_ADDRESS;
	memcpy(&TxBuf[1], pData, Size);
	HAL_I2C_Master_Transmit(&hi2c2, WRTIE_DATA_CMD, TxBuf, Size+1,1);

	if(StatusRet == HAL_OK)
	{
		res = StatusRet;
	}
	
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_OFF);
	return res;
}
#endif

#if USB_KEY_TEST

#if IIC_SIMULATION
uint8_t Test_AT2401(void)
{
	uint8_t key_flag = 0;
	AT_24C01_init();
	
	Pwr_Ctl(USB_KEY4, PWR_ON);
//	AT_24c01_write_page(0x00,TxBuf,8);
//  HAL_Delay(10);
	memset(RxBuf, 0, sizeof(RxBuf));
	key_flag=AT_24c01_seq_read(0x00,RxBuf,8); //Ë³Ðò¶ÁÈ¡¶à¸ö×Ö½Ú
//	key_flag=AT_24c01_read_data(0x02);// ¶ÁÈ¡µ¥×Ö½Ú
	
	Pwr_Ctl(USB_KEY4, PWR_OFF);
	return key_flag;
}
#else
uint8_t Test_AT2401(void)
{
	Write_AT2401(0, (uint8_t *)&etc_sensor_list[0].sensor_id, 2);
	Read_AT2401(3, RxBuf, 8);
}
#endif

#endif

// ----------------- USB_KEY OPERATER ---------------------//

//ÂÖÑ°7Â·usb_key,»ñÈ¡ sensor_id
void UsbKey_Poll()
{
	uint8_t i,j,k;
	
	pstruct_id_crc p_id_crc= (pstruct_id_crc)RxBuf;
	if(usb_key_5sec == 0)
		return;
	usb_key_5sec = 0;
	
	for(i=0; i<IO_COUNT; i++)
	{
		Pwr_Ctl(usb_key_list[i], PWR_ON);
		memset(RxBuf, 0, sizeof(RxBuf));
		
		#if IIC_SIMULATION
		delau_us(1000);
		//AT_24c01_seq_read(WORD_ADDRESS,RxBuf,15); //Á¬Ðø¶ÁÈ¡ÓÐÊ±ºò»á³ö´
		for(j=0;j<15;j++)
		{
			RxBuf[j] = AT_24c01_read_data(j);
		}
		#else
		//
		#endif
		for(k=0;k<5;k++)
		{
			if(p_id_crc[k].crc == Xor_Cal((uint8_t *)&p_id_crc[k].s_id, 2) && p_id_crc[k].s_id != 0xffff)
				break;
		}
		if(k<5)
		{
			p_id_crc[k].s_id = (p_id_crc[k].s_id>>8) | (p_id_crc[k].s_id<<8);
			if(p_id_crc[k].s_id != 0x0000 && p_id_crc[k].s_id != 0xffff)
			{
				etc_sensor_list[i].sensor_id = p_id_crc[k].s_id;
				etc_sensor_list[i].usb_key_error_times = 0;
				if(p_id_crc[k].s_id != etc_sys_param.etc_sensor_param[i].sensor_id)
				{
					etc_sys_param.etc_sensor_param[i].sensor_id = p_id_crc[k].s_id;
					etc_sys_param.etc_ap_param.sensor_num = 0;
					ap_param_write_flash_flag = 1;
					for(j=0;j<7;j++)
					{
						if(etc_sys_param.etc_sensor_param[j].sensor_id!=0)
						{
							etc_sys_param.etc_ap_param.sensor_num++;
						}
					}
				}
			}
		}
		else
		{
			etc_sensor_list[i].sensor_id = 0;
			etc_sensor_list[i].usb_key_error_times++;
			if(etc_sys_param.etc_sensor_param[i].sensor_id !=0)
			{
				etc_sys_param.etc_sensor_param[i].sensor_id = 0;
				etc_sys_param.etc_ap_param.sensor_num = 0;
				ap_param_write_flash_flag = 1;
				for(j=0;j<7;j++)
				{
					if(etc_sys_param.etc_sensor_param[j].sensor_id!=0)
					{
						etc_sys_param.etc_ap_param.sensor_num++;
					}
				}	
			}			
		}
		Pwr_Ctl(usb_key_list[i], PWR_OFF);
	}
}

//½«sensor_id Ð´Èëusb_key
void Sid2UsbKey(uint16_t s_id, uint8_t usb_key_no)
{
	memset(TxBuf, 0, sizeof(TxBuf));
	TxBuf[0] = (s_id >> 8);
	TxBuf[1] = (s_id & 0xff);
	
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_ON);
	
	#if IIC_SIMULATION
	//AT_24c01_write_page(WORD_ADDRESS,TxBuf,2);
	AT_24c01_write_data(0x00,TxBuf[0]);
	HAL_Delay(2);
	AT_24c01_write_data(0x01,TxBuf[1]);
	HAL_Delay(2);
	#else
	//
	#endif
	Pwr_Ctl(usb_key_list[usb_key_no], PWR_OFF);
}

//ÉèÖÃËùÓÐusb_keyµÄsensor_id
void SetUseKey_All(uint16_t *s_Id)
{
	uint8_t i;
	
	for(i=0; i<IO_COUNT; i++)
	{
		Sid2UsbKey(s_Id[i], i);
	}
}

#define RECV_LEN   256
uint8_t Recv[RECV_LEN];
char rev_buf[RECV_LEN];
__IO uint8_t rev_data232_flag;
extern uint8_t Test_Flag;
/**********************************************************************************
* func: ¶ÔRs232½ÓÊÕµÄÏà¹ØÃüÁî½øÐÐ´¦Àí
* descripe: 
  *
 **********************************************************************************/
void Rs232_Cmd_Proc()
{
	uint16_t usbkey_info[IO_COUNT];
	char Ack[] = "Usb_key info set OK";
	
	/****ÉèÖÃusb_key´æ´¢ÐÅÏ¢ ****/
	//CMD£ºwr_usb_key:f001,f002,f003,f004,f005,f006,f007
	if(strstr(rev_buf,"wr_usb_key") != NULL)
	{
		sscanf(rev_buf,"wr_usb_key:%04x,%04x,%04x,%04x,%04x,%04x,%04x",
								(uint32_t *)&usbkey_info[0],(uint32_t *)&usbkey_info[1],(uint32_t *)&usbkey_info[2],(uint32_t *)&usbkey_info[3],
		             (uint32_t *)&usbkey_info[4],(uint32_t *)&usbkey_info[5],(uint32_t *)&usbkey_info[6]);
		
		SetUseKey_All(usbkey_info);
	
		HAL_UART_Transmit(&huart7, (uint8_t*)Ack, strlen(Ack), 1);
	}
	/**** LEDµÆ¼° IO¿ÚÂÖÑ¯²âÊÔ ****/
	//CMD£ºled_test
	else if(strstr(rev_buf,"led_test") != NULL)
	{
		Test_Flag = 1;
	}
}

/**********************************************************************************
* func: ÆôÓÃRS232 (uart7) ÓÃÓÚUSB_KEY²ÎÊýµÄÉèÖÃ
  *
  *
 **********************************************************************************/
void start_rs232_dma_receive()
{
	SET_BIT((&huart7)->Instance->CR1, USART_CR1_IDLEIE);  //´ò¿ª´®¿Ú¿ÕÏÐÖÐ¶Ï
	HAL_UART_Receive_DMA(&huart7,Recv,RECV_LEN);	 //´ò¿ªDMA½ÓÊÕ
}

/**********************************************************************************************
***func:´®¿Ú¿ÕÏÐÖÐ¶Ï»Øµ÷º¯Êý
***     ¿ÕÏÐÖÐ¶ÏÓÃÀ´ÅÐ¶ÏÒ»°üÊý¾ÝµÄ½áÊø
*** nome
***********************************************************************************************/
void uart7_from_debug_idle_callback()
{	
	HAL_DMA_Abort((&huart7)->hdmarx);
	huart7.RxState = HAL_UART_STATE_READY;
	huart7.hdmarx->State = HAL_DMA_STATE_READY;
	
	memset(rev_buf, 0, sizeof(rev_buf));
	memcpy(rev_buf,Recv,RECV_LEN-DMA1_Stream3->NDTR);
	
	rev_data232_flag = 0x55;
	
	HAL_UART_Receive_DMA(&huart7,Recv,RECV_LEN);	 //´ò¿ªDMA½ÓÊÕ

}

