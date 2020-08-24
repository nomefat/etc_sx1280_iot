#include "main.h"
#include "stm32f4xx_hal.h"
#include "flash.h"
#include "string.h"
#include "gprs_4g_app.h"
#include "etc_def.h"


#define FLASH_HEAD 0X12345678

extern CRC_HandleTypeDef hcrc;



uint8_t flash_buff[256] = {0};
extern uint8_t ap_param_write_flash_flag ;
extern unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);
extern void init_ap_param(void);

void read_ap_param_flash(void)
{
	struct_flash_head_crc *p_head1;
	struct_flash_head_crc *p_head2;
	struct_etc_sys_param * psysparam = (struct_etc_sys_param * )(FLASH_AP_PARAM_BEGIN_1+8);
	uint32_t crc;
	uint16_t crc16_;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	int i = 0;
	int8_t times = 0;
	
	uint8_t *p_sys_param = (uint8_t *)&etc_sys_param;
	
	p_head1 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_1;
	p_head2 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_2;
	
	

read:

	times++;
	if(times>3)
		return;
	
	if(p_head1->head == FLASH_HEAD)
	{
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(etc_sys_param)/4);
		if(crc == p_head1->crc)
		{
			memcpy(&etc_sys_param,(uint8_t*)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(etc_sys_param));
			crc16_ = crc16(0,(uint8_t *)&etc_sys_param.etc_ap_param.param_updata_time,sizeof(etc_sys_param)-2);
			crc16_ = (crc16_>>8 | crc16_<<8);
			if(etc_sys_param.etc_ap_param.param_crc == crc16_)
			{
				etc_sys_param.etc_ap_param.ap_software_version = AP_VERSION;
				return;
			}
		}
		
	}
	if(p_head2->head == FLASH_HEAD)
	{
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)(FLASH_AP_PARAM_BEGIN_2+8),sizeof(etc_sys_param)/4);
		if(crc == p_head1->crc)
		{
			memcpy(&etc_sys_param,(uint8_t*)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(etc_sys_param));
			crc16_ = crc16(0,(uint8_t *)&etc_sys_param.etc_ap_param.param_updata_time,sizeof(etc_sys_param)-2);
			crc16_ = (crc16_>>8 | crc16_<<8);
			if(etc_sys_param.etc_ap_param.param_crc == crc16_)
			{
				etc_sys_param.etc_ap_param.ap_software_version = AP_VERSION;
				return;
			}
		}
	}

	
write:
	init_ap_param();  //读出参数错误 先初始化 再写入
	crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)&etc_sys_param,sizeof(etc_sys_param)/4);
	
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Banks = FLASH_BANK_2;
	EraseInit.NbSectors = 1;
	EraseInit.Sector = FLASH_SECTOR_12;
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);

	EraseInit.Sector = FLASH_SECTOR_13;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);	
	
	HAL_FLASH_Unlock();
	
	for(i=0;i<sizeof(etc_sys_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_1+8+i,p_sys_param[i]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1,FLASH_HEAD);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1+4,crc);	
	
	for(i=0;i<sizeof(etc_sys_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_2+8+i,p_sys_param[i]);
	}	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2,FLASH_HEAD);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2+4,crc);		
	
	HAL_FLASH_Lock();
	goto read;
	
}




void write_ap_param_flash(void)
{
	struct_flash_head_crc *p_head1;
	struct_flash_head_crc *p_head2;
	
	uint32_t crc;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	int i = 0;
	
	int8_t times = 0;
	uint8_t *p_sys_param = (uint8_t *)&etc_sys_param;	
	
	p_head1 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_1;
	p_head2 = (struct_flash_head_crc *)FLASH_AP_PARAM_BEGIN_2;


write:
	times++;
	if(times>3)
		return;

	crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)&etc_sys_param,sizeof(etc_sys_param)/4);
	
	HAL_FLASH_Unlock();
	
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Banks = FLASH_BANK_2;
	EraseInit.NbSectors = 1;
	EraseInit.Sector = FLASH_SECTOR_12;
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);

	EraseInit.Sector = FLASH_SECTOR_13;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);	
	
	
	
	for(i=0;i<sizeof(etc_sys_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_1+8+i,p_sys_param[i]);
	}
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1,FLASH_HEAD);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_1+4,crc);	
	
	for(i=0;i<sizeof(etc_sys_param);i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,FLASH_AP_PARAM_BEGIN_2+8+i,p_sys_param[i]);
	}	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2,FLASH_HEAD);	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_AP_PARAM_BEGIN_2+4,crc);	

	HAL_FLASH_Lock();
	
	if(p_head1->head == FLASH_HEAD)
	{

		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(etc_sys_param)/4);
		if(crc == p_head1->crc)
		{
//			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param));
			ap_param_write_flash_flag = 0;
			return;
		}
		
	}
	if(p_head2->head == FLASH_HEAD)
	{
		crc = HAL_CRC_Calculate(&hcrc,(uint32_t *)(FLASH_AP_PARAM_BEGIN_1+8),sizeof(etc_sys_param)/4);
		if(crc == p_head2->crc)
		{
//			memcpy(&ap_param,flash_buff,sizeof(struct_ap_param)); ddd
			ap_param_write_flash_flag = 0;
			return;
		}
	}
	
	goto write;
	
	
}

HAL_StatusTypeDef sd;
int32_t write_bin_flash(uint32_t address,uint8_t *pdata,uint32_t size)
{
	int i = 0;
	uint8_t *p_readdata = (uint8_t *)address;
	uint8_t re_write_times = 0;
	
	if(address < 0x08004000)
		return -1;
	
__disable_irq() ;  //关总中断
	HAL_FLASH_Unlock();
	

	for(i=0;i<size;i++)
	{
		
		sd = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,address+i,pdata[i]);
		if(p_readdata[i] != pdata[i] )
		{
			if(re_write_times<2)
			{
				re_write_times++;
				i--;
				continue;
			}
			return -1;		
		}
	}	
	
	HAL_FLASH_Lock();
	__enable_irq() ; //开总中断		
	for(i=0;i<size;i++)
	{
		if(p_readdata[i] != pdata[i])
			return -1;
	}
	
	return 0;
}








