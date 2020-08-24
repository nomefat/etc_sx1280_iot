#include "gprs_4g_app.h"
#include "gprs_comm.h"
#include "etc_rf.h"
#include "etc_def.h"
#include "string.h"
#include "stdio.h"
#include "usb_key.h"
#include "io_led.h"
#include "flash.h"



#define FIRM_HEAD_ERROR 0XFFFE
#define FIRM_LINES_ERROR 0XFFFD
#define FIRM_AP     1
#define FIRM_SENSOR 2

const uint32_t flash_begin[3] = {0,FLASH_AP_FIRMWARE_BEGIN,FLASH_SENSOR_FIRMWARE_BEGIN};

extern struct_etc_sensor_car_data etc_sensor_car_data_wait_ack[]; //已经发送过的实时数据等待ack
extern struct_etc_4g_timer_data etc_4g_timer_data_send_data;
extern char debug_send_buff[256];
extern struct_etc_sensor_updata_status etc_sensor_updata_status;
extern uint8_t ap_param_write_flash_flag;
extern CRC_HandleTypeDef hcrc;
extern uint32_t etc_sec;
extern int8_t etc_realdata_read_queue(struct_etc_sensor_car_data *pdata);
extern unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);
extern int copy_string_to_double_buff(char *pstr);
void read_firmware_sensor_head();
int io_data_rev_ack_delete_data(uint32_t number,uint8_t index);
uint32_t timer_reboot_system;
uint32_t ap_id;

struct_client_data_buff client_data_buff_0;
struct_client_data_buff client_data_buff_1;

FW_HEADER_t_5xx pheader_5xx;
struct_to_4g_task to_4g_task;

struct__etc_io_on_off etc_io_on_off;

extern int8_t gprs_print_rx_tx_data_enable;
uint8_t io_4g_fun;

//初始化系统参数表
void init_ap_param(void)
{
	uint16_t crc;
	
	etc_sys_param.etc_ap_param.server_ip[0] = 0x4a53efdb;
	etc_sys_param.etc_ap_param.server_port[0] = 40009;	
	
	etc_sys_param.etc_ap_param.param_updata_time =  get_sec_from_rtc();
	etc_sys_param.etc_ap_param.realtime_data_switch = 1;
	etc_sys_param.etc_ap_param.data_status_save_timer = 300;
	etc_sys_param.etc_ap_param.sensor_num = 0;
	etc_sys_param.etc_ap_param.ap_software_version = AP_VERSION;
	etc_sys_param.etc_sensor_param[0].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[1].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[2].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[3].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[4].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[5].off_to_on_min_t_ms = 400;
	etc_sys_param.etc_sensor_param[6].off_to_on_min_t_ms = 400;
	read_firmware_sensor_head();
	
	crc = crc16(0,(uint8_t *)&etc_sys_param.etc_ap_param.param_updata_time,sizeof(etc_sys_param)-2);
	etc_sys_param.etc_ap_param.param_crc = (crc>>8 | crc<<8);

}

//初始化应用
void app_init()
{
	uint8_t i;
	ap_id = *(uint32_t *)0x0800001c;
	if(ap_id == 0)
		ap_id = HAL_CRC_Calculate(&hcrc,(uint32_t *)0x1fff7a10,3);	
	read_ap_param_flash(); //读系统参数表
//	init_sensor_list();
	for(i=0;i<5;i++)  //读取固件头
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xx+i*32),(unsigned char * )FLASH_SENSOR_FIRMWARE_BEGIN+i*37+5,32);
	}
	
	for(i=0;i<7;i++)  //读入sensor ID
	{
		etc_sensor_list[i].no_data_times = 121;
		etc_sensor_list[i].usb_key_error_times = 1;
		etc_sensor_list[i].sensor_id = etc_sys_param.etc_sensor_param[i].sensor_id;
	}
	ALL_Channel_IO_Off();  //关闭输出
}


//发送实时数据
int to_4g_realtime_data()
{
	struct_etc_4g_server_protocol *p;
	uint16_t i,j;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	uint8_t realtime_data_num;
	uint8_t resend_data_num;
	struct_etc_sensor_car_data *p_car_data;
	
	if(to_4g_task.realtime_data_flag > 0 && gprs_stat.con_client[0].connect_ok &&(etc_sensor_car_data_wait_ack[0].sensor_id || etc_realdata_queue_has_data()))  //定时发送标志
	{		
		pdata_buff = get_4g_data_buff_max(0,&buff_len);   //获取最大缓冲
		if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
			return -1;
		
		to_4g_task.realtime_data_flag = 0;	//清除定式标志	
		if(buff_len>(sizeof(struct_etc_sensor_car_data)+20)) //4g有空
		{
			realtime_data_num = (buff_len-60)/(sizeof(struct_etc_sensor_car_data));
			p = (struct_etc_4g_server_protocol *)pdata_buff;
			p->head = 0x584daaaa;
			p->ap_id = ap_id;
			p->ap_ip = 0x01010101;
			j = 0;
			copy_string_to_double_buff("实时数据重传NO=[");
			p_car_data = (struct_etc_sensor_car_data *)&p->data[3];
			for(i=0;i<realtime_data_num;i++)   //先发等待ack的数据 确保全部收到ack 才会发送新数据
			{
				if(etc_sensor_car_data_wait_ack[i].sensor_id)
				{
					p_car_data[j] = etc_sensor_car_data_wait_ack[i];
					p_car_data[j].sys_time =  get_sec_from_rtc();
					j++;
					sprintf(debug_send_buff," %d ",etc_sensor_car_data_wait_ack[i].number);
				copy_string_to_double_buff(debug_send_buff);			
				}
			}
			copy_string_to_double_buff("]        ");
			copy_string_to_double_buff("实时数据发送NO=[");
			resend_data_num = j;
			for(;j<realtime_data_num;j++)   //等待ack的数据不够50个 补上新数据
			{							
				if(etc_realdata_read_queue(&p_car_data[j]) == -1)
				{
					break;
				}
				p_car_data[j].sys_time =  get_sec_from_rtc();
				sprintf(debug_send_buff," %d ",(p_car_data[j].number));
				copy_string_to_double_buff(debug_send_buff);
			}
			copy_string_to_double_buff("]\r\n");
			if(j!=0)
			{
				p->data[0] = 0x88;
				p->data[1] = 1;
				p->data[2] = j;
				p->len = sizeof(struct_etc_sensor_car_data)*j+3;
				crc = crc16(0, (uint8_t*)&p->ap_id, sizeof(struct_etc_4g_server_protocol)+sizeof(struct_etc_sensor_car_data)*j - 4 +2);
				p->data[1+j*sizeof(struct_etc_sensor_car_data)+2] = crc>>8;
				p->data[1+j*sizeof(struct_etc_sensor_car_data)+2+1] = crc;

				//p->packet_seq++;
				add_4g_data_len(0,sizeof(struct_etc_4g_server_protocol)+sizeof(struct_etc_sensor_car_data)*j+2+2);
				sprintf(debug_send_buff,"发送实时数据: %d 其中重发=%d\r\n",j,resend_data_num);
				copy_string_to_double_buff(debug_send_buff);					
			}			
		}
	}
}


//删除已经收到ack的实时数据
void etc_sensor_car_data_get_ack_and_delete(uint32_t number)
{
	uint16_t i;
	
	for(i=0;i<256;i++)
	{
		if(etc_sensor_car_data_wait_ack[i].number == number)
		{
			memcpy(&etc_sensor_car_data_wait_ack[i],&etc_sensor_car_data_wait_ack[i+1],(256-i-1)*sizeof(struct_etc_sensor_car_data));
		}
	}	
}


//发送定时数据
int to_4g_timer_data()
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	if(to_4g_task.timer_data_flag == 0)
		return -1;
	
	//有未发送成功的定时数据
	if(etc_4g_timer_data_send_data.struct_etc_sys_status.data_time != 0 && gprs_stat.con_client[0].connect_ok)
	{
		pdata_buff = get_4g_data_buff(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_4g_timer_data_send_data)+2+2);
		if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
			return -2;
		
		{
			to_4g_task.timer_data_flag = 0;
			p = (struct_etc_4g_server_protocol *)pdata_buff;
			p->head = 0x584daaaa;
			p->ap_id = ap_id;
			p->ap_ip = 0x01010101;		
			etc_4g_timer_data_send_data.struct_etc_sys_status.sys_time =  get_sec_from_rtc();
			p->data[0] = 0x87;
			p->data[1] = 1;
			p->data[2] = 1;
			memcpy(&p->data[3],&etc_4g_timer_data_send_data,sizeof(etc_4g_timer_data_send_data));
			p->len = sizeof(etc_4g_timer_data_send_data) + 3;
			crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)+sizeof(etc_4g_timer_data_send_data) - 4+2);
			p->data[3+sizeof(etc_4g_timer_data_send_data)] = crc>>8;
			p->data[3+sizeof(etc_4g_timer_data_send_data)+1] = crc;
			
			//p->packet_seq++;			
			add_4g_data_len(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_4g_timer_data_send_data)+2+2);
			sprintf(debug_send_buff,"发送定时数据: %d\r\n",etc_4g_timer_data_send_data.struct_etc_sys_status.number);
			copy_string_to_double_buff(debug_send_buff);					
		}
	}
}



//发送io data 2020-4-22 add
int to_4g_io_data()
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	uint16_t data_len;
	
	if(etc_io_on_off.index <50)
		return -1;
	
	if(to_4g_task.io_data_flag == 0)
		return -1;	
	
	//有未发送成功的定时数据
	if(gprs_stat.con_client[0].connect_ok)
	{
		data_len = 5+11*etc_io_on_off.index;
		pdata_buff = get_4g_data_buff(0,sizeof(struct_etc_4g_server_protocol)+data_len+1+2);
		if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
			return -2;
		io_4g_fun = 1;
		{
			to_4g_task.io_data_flag = 0;
			p = (struct_etc_4g_server_protocol *)pdata_buff;
			p->head = 0x584daaaa;
			p->ap_id = ap_id;
			p->ap_ip = 0x01010101;		
			p->data[0] = 0xa1;
			p->data[1] = 1;
			memcpy(&p->data[2],&etc_io_on_off,data_len);
			p->len = data_len + 2;
			crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)+data_len - 4+1);
			p->data[2+data_len] = crc>>8;
			p->data[2+data_len+1] = crc;
			
			//p->packet_seq++;			
			add_4g_data_len(0,sizeof(struct_etc_4g_server_protocol)+data_len+1+2);
			sprintf(debug_send_buff,"发送IO数据: %d\r\n",etc_io_on_off.index);
			copy_string_to_double_buff(debug_send_buff);					
		}
		io_4g_fun = 0;
	}
}







//发送系统参数    不自动重发
int to_4g_sys_param_data()
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	if(to_4g_task.param_flag == 0)
		return -1;
	
	//有未发送成功的定时数据
	if(1)
	{
		pdata_buff = get_4g_data_buff(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sys_param)+1+2);
		if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
			return -2;
		
		{
			to_4g_task.param_flag = 0;
			p = (struct_etc_4g_server_protocol *)pdata_buff;
			p->head = 0x584daaaa;
			p->ap_id = ap_id;
			p->ap_ip = 0x01010101;		
			p->data[0] = 0x83;
			p->data[1] = 1;
			
			memcpy(&p->data[2],&etc_sys_param,sizeof(etc_sys_param));
			p->len = sizeof(etc_sys_param) + 2;
			crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sys_param) - 4+1);
			p->data[2+sizeof(etc_sys_param)] = crc>>8;
			p->data[2+sizeof(etc_sys_param)+1] = crc;
			
			//p->packet_seq++;			
			add_4g_data_len(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sys_param)+1+2);
			sprintf(debug_send_buff,"发送系统参数: %d\r\n",1);
			copy_string_to_double_buff(debug_send_buff);					
		}
	}
}

//发送sensor升级状态
int to_4g_updata_status_data()
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	if(to_4g_task.updata_flag == 0)
		return -1;
	
	//有未发送成功的定时数据
	if(1)
	{
		pdata_buff = get_4g_data_buff(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sensor_updata_status)+1+2);
		if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
			return -2;
		
		{
			to_4g_task.updata_flag = 0;
			p = (struct_etc_4g_server_protocol *)pdata_buff;
			p->head = 0x584daaaa;
			p->ap_id = ap_id;
			p->ap_ip = 0x01010101;	
			etc_sensor_updata_status.sys_time =  get_sec_from_rtc();			
			p->data[0] = 0x86;
			p->data[1] = 1;
			memcpy(&p->data[2],&etc_sensor_updata_status,sizeof(etc_sensor_updata_status));
			p->len = sizeof(etc_sensor_updata_status) + 2;
			crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sensor_updata_status) - 4+1);
			p->data[2+sizeof(etc_sensor_updata_status)] = crc>>8;
			p->data[2+sizeof(etc_sensor_updata_status)+1] = crc;
			
			//p->packet_seq++;			
			add_4g_data_len(0,sizeof(struct_etc_4g_server_protocol)+sizeof(etc_sensor_updata_status)+1+2);
			sprintf(debug_send_buff,"4g发送升级状态包\r\n");
			copy_string_to_double_buff(debug_send_buff);					
		}
	}
}


//发送联网包
int32_t send_4g_heart_pacekt(int32_t whitch_client)
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	pdata_buff = get_4g_data_buff(whitch_client,sizeof(struct_etc_4g_server_protocol)+2);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -2;
		

	p = (struct_etc_4g_server_protocol *)pdata_buff;
	p->head = 0x584daaaa;
	p->ap_id = ap_id;
	p->ap_ip = 0x01010101;		
	p->data[0] = 0x81;
	p->len = 1;
	crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)- 4);
	p->data[1] = crc>>8;
	p->data[2] = crc;
	
	//p->packet_seq++;			
	add_4g_data_len(whitch_client,sizeof(struct_etc_4g_server_protocol)+2);
	sprintf(debug_send_buff,"4g发送连接包: %d\r\n",0);
	copy_string_to_double_buff(debug_send_buff);	

}

//发送升级检测器ack
int32_t send_4g_updata_sensor_ack_pacekt(int32_t whitch_client)
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	pdata_buff = get_4g_data_buff(whitch_client,sizeof(struct_etc_4g_server_protocol)+2+2);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -2;
		

	p = (struct_etc_4g_server_protocol *)pdata_buff;
	p->head = 0x584daaaa;
	p->ap_id = ap_id;
	p->ap_ip = 0x01010101;		
	p->data[0] = 0x85;
	p->data[1] = 1;
	p->data[2] = 0xac;
	p->len = 3;
	crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)- 4+2);
	p->data[3] = crc>>8;
	p->data[4] = crc;
	
	//p->packet_seq++;			
	add_4g_data_len(whitch_client,sizeof(struct_etc_4g_server_protocol)+4);
	sprintf(debug_send_buff,"4g发送升级sensor ack包%d\r\n",0);
	copy_string_to_double_buff(debug_send_buff);	

}



//发送重启ack包
int32_t send_4g_reboot_ack_pacekt(int32_t whitch_client)
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	pdata_buff = get_4g_data_buff(whitch_client,sizeof(struct_etc_4g_server_protocol)+2+1);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -2;
		

	p = (struct_etc_4g_server_protocol *)pdata_buff;
	p->head = 0x584daaaa;
	p->ap_id = ap_id;
	p->ap_ip = 0x01010101;		
	p->data[0] = 0x8b;
	p->data[1] = 0xac;
	p->len = 2;
	crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)- 4+1);
	p->data[2] = crc>>8;
	p->data[3] = crc;
	
	//p->packet_seq++;			
	add_4g_data_len(whitch_client,sizeof(struct_etc_4g_server_protocol)+3);
	sprintf(debug_send_buff,"4g重启ack包: %d\r\n",0);
	copy_string_to_double_buff(debug_send_buff);	

}


//发送系统参数校验
int32_t send_4g_sys_param_crc(int32_t whitch_client,uint16_t param_crc)
{
	struct_etc_4g_server_protocol *p ;
	uint16_t crc;
	uint8_t *pdata_buff;
	uint16_t buff_len = 0;
	
	pdata_buff = get_4g_data_buff(whitch_client,sizeof(struct_etc_4g_server_protocol)+3+2);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -2;
		

	p = (struct_etc_4g_server_protocol *)pdata_buff;
	p->head = 0x584daaaa;
	p->ap_id = ap_id;
	p->ap_ip = 0x01010101;		
	p->data[0] = 0x83;
	p->data[1] = 0x02;
	p->data[2] = param_crc>>8;
	p->data[3] = param_crc;
	p->len = 4;
	crc = crc16(0, (uint8_t *)&p->ap_id, sizeof(struct_etc_4g_server_protocol)- 4+3);
	p->data[4] = crc>>8;
	p->data[5] = crc;
	
	//p->packet_seq++;			
	add_4g_data_len(whitch_client,sizeof(struct_etc_4g_server_protocol)+5);
	sprintf(debug_send_buff,"4g发送写系统参数ack\r\n");
	copy_string_to_double_buff(debug_send_buff);	

}


int32_t send_4g_firmware_ack(int32_t whitch_client,uint16_t p_count,uint16_t p_seq,char *file_name)
{
	struct_etc_4g_server_protocol *p ;
	uint8_t *pdata_buff;
	uint16_t crc;
	uint16_t len;
	
	if(whitch_client > CLIENT_NUM)
		return -1;
	
	len = sizeof(struct_etc_4g_server_protocol) + 8 + strlen(file_name);
	pdata_buff = get_4g_data_buff(0,len);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
	{
		copy_string_to_double_buff("4g发送固件ack失败  没有空间\r\n");			
		return -1;	
	}

	p = (struct_etc_4g_server_protocol *)pdata_buff;
	p->head = 0x584daaaa;
	p->ap_id = ap_id;
	p->ap_ip = 0x01010101;		
	p->data[0] = 0x84;
	p->data[1] = 0x01;
	p->data[2] = 0xa1;
	p->data[3] = p_count;
	p->data[4] = p_count>>8;
	p->data[5] = p_seq;
	p->data[6] = p_seq>>8;
	strcpy(&p->data[7],file_name);
	p->len = 7+strlen(file_name);
	crc = crc16(0, (uint8_t *)&p->ap_id, len - 4 -2);
	p->data[7+strlen(file_name)] = crc>>8;
	p->data[7+strlen(file_name)+1] = crc;
	
	//p->packet_seq++;			
	add_4g_data_len(whitch_client,len);
	sprintf(debug_send_buff,"4g发送固件ack %d->%d\r\n",p_count,p_seq);
	copy_string_to_double_buff(debug_send_buff);	
	
	return 0;
}

void erase_ap_firm_flash()
{
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	__disable_irq() ;  //关总中断
	HAL_FLASH_Unlock();
	EraseInit.Sector = FLASH_SECTOR_19;
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInit.Banks = FLASH_BANK_2;
	EraseInit.NbSectors = 1;		
	EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASHEx_Erase(&EraseInit,&SectorError);
	__enable_irq() ; //开总中断	
}

void erase_ap_firm_flash1()
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;
	uint32_t Address = 0;	
	
	__disable_irq() ;  //关总中断	
	HAL_FLASH_Unlock();
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_19;
	EraseInitStruct.Banks = 2;
	EraseInitStruct.NbSectors = 1;
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{ 
		HAL_FLASH_Lock();
		Error_Handler();
	}
	HAL_FLASH_Lock();
	__enable_irq() ; //开总中断	
	
}


void erase_sensor_firm_flash()
{
		FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	
		//__disable_irq() ;  //关总中断
		HAL_FLASH_Unlock();
		EraseInit.Sector = FLASH_SECTOR_18;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);
		HAL_FLASH_Lock();
		//__enable_irq() ; //开总中断	
}

uint32_t find_firm_is_ap_or_sensor(char *file_name)
{
	uint32_t i = 0;
	
	while(file_name[i] != 0 && i<64)
	{
		if(file_name[i] == 'n' && file_name[i+1] == '1')
			return FIRM_AP;
		if(file_name[i] == 's' && file_name[i+1] == 'e'&& file_name[i+2] == 'n'&& file_name[i+3] == 's'&& file_name[i+4] == 'o')
			return FIRM_SENSOR;
		i++;
	}
	return 0;
}



/*
固件存储方式：[4bytes(固件地址)1bytes(行校验)32bytes(固件内容) ] X (n行)

*/
void read_firmware_sensor_head()
{
	unsigned char * begin_addr = (unsigned char * )FLASH_SENSOR_FIRMWARE_BEGIN;
	uint16_t i,j;
	uint32_t check_bytes_add = 0;
	uint16_t lines_size = 0;
	uint8_t *p_data;
	static uint32_t last_addr = 0;
	uint32_t addr = 0;
	
	for(i=0;i<5;i++)  //读取固件头
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xx+i*32),begin_addr+i*37+5,32);
	}
	
	if(pheader_5xx.ulId != 0xdada10af || pheader_5xx.ucCfgComb != 3)
	{
		etc_sys_param.etc_ap_param.sensor_firmware_software_version = FIRM_HEAD_ERROR;	
		return;
	}
	
	begin_addr += 37*8;
	lines_size = pheader_5xx.uiFwSize/32;
	
	for(i=0;i<lines_size;i++)
	{
		p_data = begin_addr+i*37;  //指向每行的起始地址
		for(j=5;j<37;j++)
		{
			check_bytes_add +=p_data[j];
		}
		addr = *((uint32_t *)p_data);
		if(addr == last_addr)
		{
			check_bytes_add = 0;
			break;
		}
		last_addr = addr;				
	}
	if(check_bytes_add == pheader_5xx.ulCheckSum)
	{
		if(etc_sys_param.etc_ap_param.sensor_firmware_software_version != pheader_5xx.uiFwVer)
		{
			etc_sys_param.etc_ap_param.sensor_firmware_software_version = pheader_5xx.uiFwVer;
			ap_param_write_flash_flag = 1;
		}
	}
	else
	{
		etc_sys_param.etc_ap_param.sensor_firmware_software_version = FIRM_LINES_ERROR;
		ap_param_write_flash_flag = 1;		
	}
}




void rev_4g_server_ap_firmware(int32_t whitch_client,uint8_t *pdata, uint16_t p_count,uint16_t p_seq,int16_t len)
{
	struct_gprs_4g_firmware_data *p_firmware = (struct_gprs_4g_firmware_data *)pdata;
	static uint32_t now_firmware_len = 0;   //记录当前flash写了的固件长度
	static uint16_t now_seq = 0xffff;   //记录当前flash写了的固件长度
	static uint16_t get_crc = 0;  //服务器下发的文件crc
	static char file_name[64];  //文件名
	static int8_t ap_or_sensor = 0;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	uint16_t crc;

	
	if(p_seq == 0)  //服务器下发第0包   如果本地不是第0包 则需要执行Flash擦除
	{
		now_firmware_len = 0;		
		memcpy(file_name,&pdata[2],len-2);
		file_name[len-2] = 0;
		get_crc = pdata[0] <<8 | pdata[1];
		ap_or_sensor = find_firm_is_ap_or_sensor(file_name);
		if(now_seq != 0) //本地记录不是第0包  需要执行擦除 如果本地已经是第0包 说明已经擦除过了 不操作
		{
			now_seq = 0;
			now_firmware_len = 0;
			if(ap_or_sensor == FIRM_AP)
				erase_ap_firm_flash1();	
			else if(ap_or_sensor == FIRM_SENSOR)
				erase_sensor_firm_flash();			
		}
	}
	
	if(p_seq == (now_seq+1) && p_seq <=p_count)  //服务器下发的固件位置等于本地记录的位置  才会写flash
	{
		if(len < 1500)
		{
			if(write_bin_flash(flash_begin[ap_or_sensor]+now_firmware_len,pdata,len) == 0) //写正确
			{
				now_firmware_len += len;
				now_seq = p_seq;
			}
			else
			{
				now_firmware_len = 0;
				now_seq = 0;
				erase_ap_firm_flash1();	
				erase_sensor_firm_flash();					
			}			
		}
	}
	
	
	//返回服务器ack   包含本地已经写到的固件位置， 便于服务器确认固件下发的对错，及时调整固件位置
	if(now_seq <=p_count)
		send_4g_firmware_ack(whitch_client,p_count,now_seq+1,file_name);	

	if(p_count == now_seq)
	{
		crc = crc16(0,(uint8_t *)flash_begin[ap_or_sensor],now_firmware_len);
		if(crc == get_crc)
		{
			if(ap_or_sensor == FIRM_AP)
			{
				write_bin_flash(FLASH_AP_FIRMWARE_HEAD,(uint8_t *)&now_firmware_len,4);
				write_bin_flash(FLASH_AP_FIRMWARE_HEAD+4,(uint8_t *)&crc,2);
				timer_reboot_system = etc_sec  + 5;
			}
			else if(ap_or_sensor == FIRM_SENSOR)
			{
				read_firmware_sensor_head();
			}
		}
	}	
	
}



void poll_4g_task()
{
	to_4g_timer_data();
	to_4g_io_data();
	to_4g_realtime_data();
	to_4g_updata_status_data();
	to_4g_sys_param_data();
}

/*
解析4G服务器下发数据
	该函数接收到的数据为服务器下发的数据，可能包含多条或者不足一条

*/
int32_t server_4g_data_hanle(int32_t whitch_client)
{
	uint16_t crc,crc1;
	uint16_t index;
	struct_etc_4g_server_protocol *phead;
	uint8_t data_from_witch_client = -1;
	struct_client_data_buff *client_buff[3] = {&client_data_buff_0,&client_data_buff_1};
	uint32_t i = 0;
	uint32_t *seq = 0;
	uint32_t *p_number;
	struct_etc_sys_param* p_param;
	struct_s *s;
	
	
	if(whitch_client<0 || whitch_client>1)
		return -1;
	
	while(1)
	{	
		for(index=0;index<client_buff[whitch_client]->index;index++)
		{
			phead = (struct_etc_4g_server_protocol *)&(client_buff[whitch_client]->buff[index]);
			if(phead->head == 0x584daaaa)   //从数据buff中找协议头
			{
				if(phead->len+17 > GPRS_DATA_BUFF_LENGH) //协议中长度超过范围 数据作废
				{
					client_buff[whitch_client]->index = 0;
					return -2;					
				}
				else if(phead->len+17 <= client_buff[whitch_client]->index-index)  //一条协议数据长度小于总数据长度，说明存在一条完整的协议数据
				{
					index += phead->len+15;
					goto work;
				}
				else   //没有一条完整的协议数据，继续接收数据
				{
					if(index > 0)  //消除前面的废弃数据
					{
						memcpy(&(client_buff[whitch_client]->buff[0]),&(client_buff[whitch_client]->buff[index]),client_buff[whitch_client]->index-index);
					}
					return -1;
				}
			}

		}
		//没有找到协议头
		client_buff[whitch_client]->index = 0;
		return -1;
		


work:	
		crc = (phead->data[phead->len]<<8) + phead->data[phead->len+1];
		crc1 = crc16(0,(uint8_t *)&phead->ap_id,phead->len+11);
		if(gprs_print_rx_tx_data_enable)
		{
			sprintf(debug_send_buff,"4g_hanle_data: len=%d %d\r\n",index,crc-crc1);
			copy_string_to_double_buff(debug_send_buff);
		}
		if(crc == crc1)  //crc校验
		{				
			switch(phead->data[0])
			{
				case 0x87:  //定时流量ack
					if(phead->data[1] == 1)
					{
						p_number = (uint32_t *)&phead->data[2];
						for(i=0;i<(phead->len-2)/4;i++)
						{
							if(p_number[i] == etc_4g_timer_data_send_data.struct_etc_sys_status.number)
							{
								etc_4g_timer_data_send_data.struct_etc_sys_status.data_time = 0;
							}
						}
						sprintf(debug_send_buff,"ACK定时数据: %d\r\n",p_number[0]);
						copy_string_to_double_buff(debug_send_buff);	
					}
					break;
					
				case 0x88:  //实时流量ack
					if(phead->data[1] == 1)
					{
						p_number = (uint32_t *)&phead->data[2];
						sprintf(debug_send_buff,"ACK实时数据: %d [",(phead->len-2)/4);
						copy_string_to_double_buff(debug_send_buff);	
						for(i=0;i<(phead->len-2)/4;i++)
						{
							etc_sensor_car_data_get_ack_and_delete(p_number[i]);
							sprintf(debug_send_buff," %d ",p_number[i]);
							copy_string_to_double_buff(debug_send_buff);	
						}			
						copy_string_to_double_buff("]\r\n");				
					}			
					break;
				case 0xa1:
					if(phead->data[1] == 1)
					{
						p_number = (uint32_t *)&phead->data[2];
						io_data_rev_ack_delete_data(*p_number,phead->data[6]);
						sprintf(debug_send_buff,"4g rev io ack: %d->%d\r\n",*p_number,phead->data[6]);
						copy_string_to_double_buff(debug_send_buff);						
					}
					break;
				case 0x85:  //启动sensor升级
					if(phead->data[1] == 1)
					{
						s = (struct_s *)&phead->data[2];
						for(i=0;i<s->sensor_num;i++)
						{
							etc_updata_set_id(s->sensor_id[i]);
						}
						send_4g_updata_sensor_ack_pacekt(0);
						sprintf(debug_send_buff,"4g启动升级sensor: %d\r\n",s->sensor_num);
						copy_string_to_double_buff(debug_send_buff);					
					}			
					break;	
				case 0x8b:  //服务器重启命令
					send_4g_reboot_ack_pacekt(0);
					timer_reboot_system = etc_sec  + 5;
					break;
				case 0xa0:  //校准sensor
					if(phead->data[1] == 1)
					{
						s = (struct_s *)&phead->data[2];
						for(i=0;i<s->sensor_num;i++)
						{
							etc_adjust_set_id(s->sensor_id[i]);
						}
						sprintf(debug_send_buff,"4g收到校准sensor命令: %d\r\n",s->sensor_num);
						copy_string_to_double_buff(debug_send_buff);					
					}			
					break;					
				case 0x83:  
					if(phead->data[1] == 1) //服务器读系统参数
					{
						to_4g_task.param_flag++;
						copy_string_to_double_buff("服务器读系统参数\r\n");					
					}	
					else if(phead->data[1] == 2) //系统写参数
					{
						crc = crc16(0,&phead->data[4],sizeof(etc_sys_param)-2);
						if( phead->data[2] == (crc>>8) && phead->data[3] == (crc&0xff))
						{
							p_param = (struct_etc_sys_param*)&phead->data[2];
							if(p_param->etc_ap_param.server_ip[0] != etc_sys_param.etc_ap_param.server_ip[0] ||
							p_param->etc_ap_param.server_port[0] != etc_sys_param.etc_ap_param.server_port[0])
							{
								gprs_stat.con_client[0].connect_ok = 0;
							}
							if(p_param->etc_ap_param.server_ip[1] != etc_sys_param.etc_ap_param.server_ip[1] ||
							p_param->etc_ap_param.server_port[1] != etc_sys_param.etc_ap_param.server_port[1])
							{
								gprs_stat.con_client[1].connect_ok = 0;
							}							
							memcpy(&etc_sys_param,&phead->data[2],sizeof(etc_sys_param));
							etc_sys_param.etc_ap_param.ap_software_version = AP_VERSION;
							for(i=0;i<7;i++)  //读入sensor ID
							{
								etc_sensor_list[i].no_data_times = 121;
								etc_sensor_list[i].usb_key_error_times = 1;
								etc_sensor_list[i].sensor_id = etc_sys_param.etc_sensor_param[i].sensor_id;
							}
							ap_param_write_flash_flag = 1;
							copy_string_to_double_buff("服务器写系统参数成功\r\n");
						}						
						else
							copy_string_to_double_buff("服务器写系统参数失败\r\n");
						send_4g_sys_param_crc(0,crc);
					}					
				break;
				case 0x84:  //服务器下发固件
					if(phead->data[1] == 1) 
					{
						sprintf(debug_send_buff,"4g收到固件: %d->%d\r\n",(phead->data[3] | phead->data[4]<<8),(phead->data[5] | phead->data[6]<<8));
						copy_string_to_double_buff(debug_send_buff);							
						rev_4g_server_ap_firmware(0,&phead->data[7],(phead->data[3] | phead->data[4]<<8),(phead->data[5] | phead->data[6]<<8),phead->len-7);
					}		
				break;					
				default:break;
			}			
		}
			
		if(client_buff[whitch_client]->index-index > 0) //还有未处理完的数据
		{
			memcpy(&client_buff[whitch_client]->buff[0],&client_buff[whitch_client]->buff[index],client_buff[whitch_client]->index-index);
			client_buff[whitch_client]->index = client_buff[whitch_client]->index-index;
		}
		else
			client_buff[whitch_client]->index = 0;
	}
}



int insert_io_data(uint8_t io,uint8_t on_off)
{
	if(etc_io_on_off.index>99)
		return -1;
	
	etc_io_on_off.io_data[etc_io_on_off.index].io = io;
	etc_io_on_off.io_data[etc_io_on_off.index].ms = slot_count%128*8;
	etc_io_on_off.io_data[etc_io_on_off.index].second = slot_count/128;
	etc_io_on_off.io_data[etc_io_on_off.index].on_off = on_off;
	etc_io_on_off.io_data[etc_io_on_off.index].sys_time = get_sec_from_rtc();
	
	etc_io_on_off.index++;
}


int io_data_rev_ack_delete_data(uint32_t number,uint8_t index)
{
	
	if(number != etc_io_on_off.number)
		return -1;
	
	if(index > etc_io_on_off.index)
		return -2;
	
	io_4g_fun = 2;
	if(index>=99)
		etc_io_on_off.index = 0;
	else{
		memcpy(&etc_io_on_off.io_data[0],&etc_io_on_off.io_data[index],sizeof(etc_io_on_off.io_data[0])*(etc_io_on_off.index-index));
		etc_io_on_off.index = etc_io_on_off.index-index;
	}
	etc_io_on_off.number++;
	
	io_4g_fun = 0;
	return 0;
	
}


uint8_t test_flash_write_buff[1024];

uint32_t test_flash_write_times;

void test_flash_write()
{
	uint32_t i = 0;
	erase_ap_firm_flash1();
	
	for(i=0;i<1024;i++)
	{
		test_flash_write_buff[i] = test_flash_write_times;
	}
	
	for(i=0;i<100;i++)
	{
		if(write_bin_flash(flash_begin[1]+1024*i,test_flash_write_buff,1024) != 0)
		{
			while(1);
		}
	}
}







