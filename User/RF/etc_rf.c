#include "main.h"
#include "stm32f4xx_hal.h"
#include "etc_rf.h"
#include "etc_def.h"
#include "radio.h"
#include "string.h"
#include "stdio.h"
#include "io_led.h"
#include "flash.h"
#include "gprs_4g_app.h"




#define OFF_TO_ON_TIME 400
#define UPDATA_TIME_OUT (128*61)   //61秒


extern uint32_t ap_id;
extern uint8_t ap_param_write_flash_flag ;
extern uint8_t usb_key_5sec;
extern volatile uint8_t radio_switch;
extern char debug_send_buff[256];
extern volatile PacketStatus_t pktStatus ;
extern strcut_radio_status radio_status[2];
extern struct_to_4g_task to_4g_task;
extern unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);
extern int copy_string_to_double_buff(char *pstr);
extern void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel );

void etc_rev_data(void *p_rf_data,uint8_t slot,uint8_t index);
void scan_button();
void etc_else_ap_or_sensor_packet_handle(void *p_rf_data,uint8_t slot);
void etc_make_updata_packet();
void etc_make_ack();
void make_realtime_data_and_insert_queue(uint8_t index,uint32_t ontime);
void make_on_data_and_insert_queue(uint8_t index,uint16_t event,uint8_t seq,uint8_t rt);
void make_timer_data_and_status_and_insert_queue();
void etc_led_out(uint8_t io,uint8_t on_off);
uint32_t timer_has_go_time();
extern void get_nodata_rssi();

extern int8_t nodata_rssi[2];
int32_t interrupt_or_main;
uint32_t real_data_number;
int8_t start_sys_timer ;
uint8_t now_slot;
uint8_t now_ch;
volatile uint32_t slot_count;
int32_t button_status;

struct_etc_update_s_manage etc_updata_s_manage;

struct_sensor_list etc_sensor_list[IO_COUNT] = {{0},{0},{0},{0},{0},{0},{0},};

struct_test_rev_rf_rate test_rev_rf_rate[2];

#define ETC_REALTIME_DATA_QUEUE_LENGH 1024

uint32_t etc_realdata_queue_write;
uint32_t etc_realdata_queue_read;
struct_etc_sensor_car_data etc_sensor_car_data[ETC_REALTIME_DATA_QUEUE_LENGH]; //实时数据缓冲

struct_etc_sensor_car_data etc_sensor_car_data_wait_ack[256]; //已经发送过的实时数据等待ack

struct_etc_sensor_car_data etc_sensor_car_data_one; //实时过车数据

struct_etc_sensor_on_event etc_sensor_on_event; // ON状态上报

struct_etc_4g_timer_data etc_4g_timer_data; //定时数据

struct_etc_4g_timer_data etc_4g_timer_data_send_data;  //定时数据待发送4g

union_etc_jump_channel etc_jump_channel;

struct_etc_ack etc_ack;

int32_t timer_rssi;

struct_etc_sys_param etc_sys_param; //系统参数


struct_etc_sensor_updata_status etc_sensor_updata_status;

uint8_t led_sec;
uint32_t etc_sec; //存活秒数

int32_t enable_send_ack_forever = -1;

uint8_t enable_rev_1000;
uint16_t rev_id_1000;
uint16_t rev_1000_count;
uint16_t rev_1000_lost;
uint8_t rev_1000_now_seq;
/*
 * 功能：往队列中写入一个字节
 * 失败：返回-1
 * 成功：返回0
*/
int8_t etc_realdata_write_queue(struct_etc_sensor_car_data *data)
{
	if((etc_realdata_queue_write+1)%ETC_REALTIME_DATA_QUEUE_LENGH==etc_realdata_queue_read)
		return -1;
	etc_sensor_car_data[etc_realdata_queue_write] = *data;
	etc_realdata_queue_write = (etc_realdata_queue_write+1)%ETC_REALTIME_DATA_QUEUE_LENGH;
	return 0;
}

/*
 * 功能：从队列中读取一个字节
 * 失败：返回-1
 * 成功：返回0
*/
 int8_t etc_realdata_read_queue(struct_etc_sensor_car_data *pdata)
{
	uint16_t i=0;
	if(etc_realdata_queue_write==etc_realdata_queue_read)
		return -1;		
	*pdata = etc_sensor_car_data[etc_realdata_queue_read];
	for(i=0;i<256;i++)
	{
		if(etc_sensor_car_data_wait_ack[i].sensor_id == 0)
		{
			etc_sensor_car_data_wait_ack[i] = etc_sensor_car_data[etc_realdata_queue_read];
			break;
		}
	}
	etc_realdata_queue_read = (etc_realdata_queue_read+1)%ETC_REALTIME_DATA_QUEUE_LENGH;
	return 0;
}


int8_t etc_realdata_queue_has_data()
{
	if(etc_realdata_queue_write==etc_realdata_queue_read)
		return 0;	
	else
		return 1;
}

//初始化sensor id 为0时候的通道
void init_sensor_list()
{
	uint32_t i = 0;
	uint8_t *p_char = (uint8_t *)0x1fff7a10;
	
	
	for(i=0;i<IO_COUNT;i++)
	{
		
		etc_sensor_list[i].sensor_id_is_zero_ch = p_char[i];
	}
}


//清除帧标志位
void clear_sensor_flag_frame()
{
	uint32_t i = 0;
	
	for(i=0;i<IO_COUNT;i++)
	{
		etc_sensor_list[i].frame_rev_evnet = 0;
	}	
}



//通过sensor列表中的ID 计算跳频通道，sensor id更新时候调用
void calc_ch_from_sensor_list(struct_sensor_list *p_etc_sensor_list,struct _etc_jump_channel *etc_jump_channel)
{
	uint32_t i;
	
	etc_jump_channel->ch0 = 0;
	etc_jump_channel->ch1 = 0;

	etc_jump_channel->ch0 |= (p_etc_sensor_list[0].sensor_id!=0 ? ((p_etc_sensor_list[0].sensor_id%CH_COUNT)&0x1f) : (p_etc_sensor_list[0].sensor_id_is_zero_ch&0x1f)) |
	(p_etc_sensor_list[1].sensor_id!=0 ? (((p_etc_sensor_list[1].sensor_id%CH_COUNT)&0x1f)<<5) : ((p_etc_sensor_list[1].sensor_id_is_zero_ch&0x1f)<<5))|
	(p_etc_sensor_list[2].sensor_id!=0 ? (((p_etc_sensor_list[2].sensor_id%CH_COUNT)&0x1f)<<10) : ((p_etc_sensor_list[2].sensor_id_is_zero_ch&0x1f)<<10))|
	(p_etc_sensor_list[3].sensor_id!=0 ? (((p_etc_sensor_list[3].sensor_id%CH_COUNT)&0x1f)<<15) : ((p_etc_sensor_list[3].sensor_id_is_zero_ch&0x1f)<<15))|
	(p_etc_sensor_list[4].sensor_id!=0 ? (((p_etc_sensor_list[4].sensor_id%CH_COUNT)&0x1f)<<20) : ((p_etc_sensor_list[4].sensor_id_is_zero_ch&0x1f)<<20))|
	(p_etc_sensor_list[5].sensor_id!=0 ? (((p_etc_sensor_list[5].sensor_id%CH_COUNT)&0x1f)<<25) : ((p_etc_sensor_list[5].sensor_id_is_zero_ch&0x1f)<<25))|
	(((ap_id&0x03)<<30));

	etc_jump_channel->ch1 |=(((ap_id&0x1c)>>2));
	etc_jump_channel->ch1 |= ((ap_id&0x1f)<<3);
	
}

//接收RF 数据  区分是否本机所带的Sensor
void etc_rev_rf_data(void *p_rf_data,uint8_t slot)
{
	struct_etc_event *p_etc_event = (struct_etc_event *)p_rf_data;
	struct_etc_stat *p_etc_stat = (struct_etc_stat *)p_rf_data;
	struct_etc_sensor_ack *p_etc_sensor_ack = (struct_etc_sensor_ack *)p_rf_data;	
		
	uint32_t i;
	
	
	for(i=0;i<IO_COUNT;i++)
	{
		if(etc_sensor_list[i].sensor_id == p_etc_event->etc_head.dev_id)
			goto MY_SENSOR;
	}
	goto ELSE_SENSOR;
		
	
MY_SENSOR:	
	etc_rev_data(p_rf_data,now_slot,i);
	return;
	
	
ELSE_SENSOR: //非列表中的sensor 或者其他AP 或者其他设备的数据
	etc_else_ap_or_sensor_packet_handle(p_rf_data,now_slot);
	return;
	
}

void get_event_to_hanle(struct_etc_event *p_etc_event,uint8_t index,uint8_t valid_event_num,uint8_t flag)
{
	uint8_t event_num;
	int8_t i;
	int32_t event_from_next_time;
	int32_t event_from_next_time_1;

	if(valid_event_num>4)
		valid_event_num = 4;
	for(i=valid_event_num-1;i>=0;i--)
	{
		if(p_etc_event->event[i].uiAll == 0xffff || p_etc_event->event[i].bmSec >16) //无效事件
			continue;
		
		if(etc_sensor_list[index].now_on_off_status == NONE)  //第一个事件 直接记录状态
		{
			if(p_etc_event->event[i].blIsOn == 1) //on事件
			{
				etc_sensor_list[index].now_on_off_status = ON;                       //    ON
				etc_led_out(index,etc_sensor_list[index].now_on_off_status);
				etc_sensor_list[index].car_count++;
				make_on_data_and_insert_queue(index,p_etc_event->event[i].uiAll,p_etc_event->etc_head.seq,p_etc_event->resend_times);
				sprintf(debug_send_buff,"<--ON-->: id=[%04X] t=%d.%d\r\n",etc_sensor_list[index].sensor_id, p_etc_event->event[i].bmSec,p_etc_event->event[i].bmMs);
				copy_string_to_double_buff(debug_send_buff);					
			}
			else
			{
				etc_sensor_list[index].now_on_off_status = OFF;                      //    OFF
				etc_led_out(index,etc_sensor_list[index].now_on_off_status);
				sprintf(debug_send_buff,"<--OFF-->: id=[%04X] t=%d.%d\r\n",etc_sensor_list[index].sensor_id,p_etc_event->event[i].bmSec,p_etc_event->event[i].bmMs);
				copy_string_to_double_buff(debug_send_buff);					
			}

			
			etc_sensor_list[index].now_event = p_etc_event->event[i];
			etc_sensor_list[index].on_off_time_tick = slot_count;
		}
		else   //不是第一个事件
		{
			event_from_next_time = p_etc_event->event[i].bmSec*1024+p_etc_event->event[i].bmMs - (etc_sensor_list[index].now_event.bmSec*1024+etc_sensor_list[index].now_event.bmMs);
			if(event_from_next_time<0)
				event_from_next_time += 16*1024;
			
			event_from_next_time = event_from_next_time*1000/1024;
			
			if(slot_count-etc_sensor_list[index].on_off_time_tick >= 128*16)   //实际时间已经大于16秒
			{
				event_from_next_time += ((slot_count-etc_sensor_list[index].on_off_time_tick)/(128*16))*16000; //加上实际过的时间	
			}
			
			if(etc_sensor_list[index].now_on_off_status == OFF)  //现在是OFF
			{
				if(p_etc_event->event[i].blIsOn == 1)//on事件
				{
					if(event_from_next_time > etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms)  //off->on阈值
					{
						//满足阈值   切换为ON
						etc_sensor_list[index].car_count++;
						etc_sensor_list[index].off_time_ms += event_from_next_time;
						if(event_from_next_time/1000 > timer_has_go_time())		
							etc_sensor_list[index].off_time_ms_timer_use += timer_has_go_time()*1000;
						else
							etc_sensor_list[index].off_time_ms_timer_use += event_from_next_time;						
						
						sprintf(debug_send_buff,"<--ON-->: id=[%04X] t=%dms\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);								
						etc_sensor_list[index].now_on_off_status = ON;                 //   ON
						etc_led_out(index,etc_sensor_list[index].now_on_off_status);
						make_on_data_and_insert_queue(index,p_etc_event->event[i].uiAll,p_etc_event->etc_head.seq,p_etc_event->resend_times);
						etc_4g_timer_data.struct_etc_data_and_status[index].car_count++;
						etc_sensor_list[index].now_event = p_etc_event->event[i];
						etc_sensor_list[index].on_off_time_tick = slot_count;						
					}
					else
					{
						//不满足阈值  ON事件丢弃
						sprintf(debug_send_buff,"<--no_time off to on-->: id=[%04X] %d\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);								
					}
					
				}
				else //off事件  OFF->OFF  不更新
				{
						sprintf(debug_send_buff,"<--off to off-->: id=[%04X] \r\n",etc_sensor_list[index].sensor_id);
						copy_string_to_double_buff(debug_send_buff);						
//					etc_sensor_list[index].now_event = p_etc_event->event[i];
//					etc_sensor_list[index].on_off_time_tick = slot_count;					
				}
			}
			else//现在是ON
			{
				if(p_etc_event->event[i].blIsOn == 1)//又来一个on事件
				{
					if(etc_sensor_list[index].no_sure_off_time_tick !=0) //有一个未确认的OFF事件
					{
						//OFF持续的时间
						event_from_next_time = p_etc_event->event[i].bmSec*1024+p_etc_event->event[i].bmMs - (etc_sensor_list[index].no_sure_off_event.bmSec*1024+etc_sensor_list[index].no_sure_off_event.bmMs);
						if(event_from_next_time<0)
							event_from_next_time += 16*1024;
						
						event_from_next_time = event_from_next_time*1000/1024;
						
						if(slot_count-etc_sensor_list[index].no_sure_off_time_tick >= 128*16)   //实际时间已经大于16秒
						{
								event_from_next_time += ((slot_count-etc_sensor_list[index].no_sure_off_time_tick)/(128*16))*16000; //加上实际过的时间	
						}						
						if(event_from_next_time > etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms)  //满足off->on阈值
						{
							//未确认的临时OFF减去之前ON的时间   这个时间是之前ON持续的时间
							event_from_next_time_1 = (etc_sensor_list[index].no_sure_off_event.bmSec*1024+etc_sensor_list[index].no_sure_off_event.bmMs)
							- (etc_sensor_list[index].now_event.bmSec*1024+etc_sensor_list[index].now_event.bmMs);
							if(event_from_next_time_1<0)
								event_from_next_time_1 += 16*1024;	
							
							event_from_next_time_1 = event_from_next_time_1*1000/1024;
							if(slot_count-etc_sensor_list[index].on_off_time_tick >= 128*16)   //实际时间已经大于16秒
							{
								event_from_next_time_1 += ((slot_count-etc_sensor_list[index].on_off_time_tick)/(128*16))*16000; //加上实际过的时间	
							}								
							
							etc_sensor_list[index].on_time_ms += event_from_next_time_1;		
							if(event_from_next_time/1000 > timer_has_go_time())		
								etc_sensor_list[index].on_time_ms_timer_use += timer_has_go_time()*1000;
							else
								etc_sensor_list[index].on_time_ms_timer_use += event_from_next_time_1;	
							etc_sensor_list[index].now_on_off_status = OFF;							
							make_realtime_data_and_insert_queue(index,event_from_next_time_1);
							//确认上一个OFF事件，此处需要做OFF计数统计 但是实际状态已经到ON 错过了输出时间       //   OFF
							
							
							sprintf(debug_send_buff,"<--OFF--no time out>: id=[%04X] on->off:%dms 有一个未确认的OFF事件\r\n",etc_sensor_list[index].sensor_id,event_from_next_time_1);
							copy_string_to_double_buff(debug_send_buff);		
							//满足阈值   切换为ON
								
							etc_sensor_list[index].car_count++;
							etc_sensor_list[index].off_time_ms += event_from_next_time;
						if(event_from_next_time/1000 > timer_has_go_time())		
							etc_sensor_list[index].off_time_ms_timer_use += timer_has_go_time()*1000;
						else
							etc_sensor_list[index].off_time_ms_timer_use += event_from_next_time;	
							etc_sensor_list[index].now_on_off_status = ON;
							etc_4g_timer_data.struct_etc_data_and_status[index].car_count++;
							etc_led_out(index,etc_sensor_list[index].now_on_off_status);
							etc_sensor_list[index].now_event = p_etc_event->event[i];
							etc_sensor_list[index].on_off_time_tick = slot_count;	
							etc_sensor_list[index].no_sure_off_event.uiAll = 0xffff;
							etc_sensor_list[index].no_sure_off_time_tick = 0;	
							sprintf(debug_send_buff,"<--ON-->: id=[%04X] off->on:%dms \r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
							copy_string_to_double_buff(debug_send_buff);								
						}						
						else  //不满足阈值  取消上一个off事件  不更新这个ON状态
						{
								//不满足阈值  ON事件丢弃
								sprintf(debug_send_buff,"<--no_time off to on-->: id=[%04X] %dms 不满足阈值 丢弃\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
								copy_string_to_double_buff(debug_send_buff);															
						}
					}
					else
					{
					//ON->ON
						sprintf(debug_send_buff,"<--on to on-->: id=[%04X] %d 丢弃\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);						
						//etc_sensor_list[index].now_event = p_etc_event->event[i];
						//etc_sensor_list[index].on_off_time_tick = slot_count;	
					}
				}
				else //off事件  先写入中间状态 等待超时或者下一个事件确认		
				{
					etc_sensor_list[index].no_sure_off_event = p_etc_event->event[i];
					etc_sensor_list[index].no_sure_off_time_tick = slot_count;
				}				
			}
		}
	}
	
}

void get_ack_event_to_hanle(struct_etc_sensor_ack *p_etc_event,uint8_t index,uint8_t valid_event_num,uint8_t flag)
{
	uint8_t event_num;
	int8_t i;
	int32_t event_from_next_time;
	int32_t event_from_next_time_1;
	
	valid_event_num = 1;
	
	for(i=valid_event_num-1;i>=0;i--)
	{
		if(p_etc_event->event[i].uiAll == 0xffff || p_etc_event->event[i].bmSec >16) //无效事件
			continue;
		
		if(etc_sensor_list[index].now_on_off_status == NONE)  //第一个事件 直接记录状态
		{
			if(p_etc_event->event[i].blIsOn == 1) //on事件
			{
				etc_sensor_list[index].now_on_off_status = ON;                       //    ON
				etc_led_out(index,etc_sensor_list[index].now_on_off_status);
				etc_sensor_list[index].car_count++;
				make_on_data_and_insert_queue(index,p_etc_event->event[i].uiAll,p_etc_event->etc_head.seq,p_etc_event->resend_times);
				sprintf(debug_send_buff,"<--ON-->: id=[%04X] t=%d.%d\r\n",etc_sensor_list[index].sensor_id, p_etc_event->event[i].bmSec,p_etc_event->event[i].bmMs);
				copy_string_to_double_buff(debug_send_buff);					
			}
			else
			{
				etc_sensor_list[index].now_on_off_status = OFF;                      //    OFF
				etc_led_out(index,etc_sensor_list[index].now_on_off_status);
				sprintf(debug_send_buff,"<--OFF-->: id=[%04X] t=%d.%d\r\n",etc_sensor_list[index].sensor_id,p_etc_event->event[i].bmSec,p_etc_event->event[i].bmMs);
				copy_string_to_double_buff(debug_send_buff);					
			}

			
			etc_sensor_list[index].now_event = p_etc_event->event[i];
			etc_sensor_list[index].on_off_time_tick = slot_count;
		}
		else   //不是第一个事件
		{
			event_from_next_time = p_etc_event->event[i].bmSec*1024+p_etc_event->event[i].bmMs - (etc_sensor_list[index].now_event.bmSec*1024+etc_sensor_list[index].now_event.bmMs);
			if(event_from_next_time<0)
				event_from_next_time += 16*1024;
			
			event_from_next_time = event_from_next_time*1000/1024;
			
			if(slot_count-etc_sensor_list[index].on_off_time_tick >= 128*16)   //实际时间已经大于16秒
			{
				event_from_next_time += ((slot_count-etc_sensor_list[index].on_off_time_tick)/(128*16))*16000; //加上实际过的时间	
			}
			
			if(etc_sensor_list[index].now_on_off_status == OFF)  //现在是OFF
			{
				if(p_etc_event->event[i].blIsOn == 1)//on事件
				{
					if(event_from_next_time > etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms)  //off->on阈值
					{
						//满足阈值   切换为ON
						etc_sensor_list[index].car_count++;
						etc_sensor_list[index].off_time_ms += event_from_next_time;
						if(event_from_next_time/1000 > timer_has_go_time())		
							etc_sensor_list[index].off_time_ms_timer_use += timer_has_go_time()*1000;
						else
							etc_sensor_list[index].off_time_ms_timer_use += event_from_next_time;						
						
						sprintf(debug_send_buff,"<--ON-->: id=[%04X] t=%dms\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);								
						etc_sensor_list[index].now_on_off_status = ON;                 //   ON
						etc_led_out(index,etc_sensor_list[index].now_on_off_status);
						make_on_data_and_insert_queue(index,p_etc_event->event[i].uiAll,p_etc_event->etc_head.seq,p_etc_event->resend_times);
						etc_4g_timer_data.struct_etc_data_and_status[index].car_count++;
						etc_sensor_list[index].now_event = p_etc_event->event[i];
						etc_sensor_list[index].on_off_time_tick = slot_count;						
					}
					else
					{
						//不满足阈值  ON事件丢弃
						sprintf(debug_send_buff,"<--no_time off to on-->: id=[%04X] %d\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);								
					}
					
				}
				else //off事件  OFF->OFF  不更新
				{
						sprintf(debug_send_buff,"<--off to off-->: id=[%04X] \r\n",etc_sensor_list[index].sensor_id);
						copy_string_to_double_buff(debug_send_buff);						
//					etc_sensor_list[index].now_event = p_etc_event->event[i];
//					etc_sensor_list[index].on_off_time_tick = slot_count;					
				}
			}
			else//现在是ON
			{
				if(p_etc_event->event[i].blIsOn == 1)//又来一个on事件
				{
					if(etc_sensor_list[index].no_sure_off_time_tick !=0) //有一个未确认的OFF事件
					{
						//OFF持续的时间
						event_from_next_time = p_etc_event->event[i].bmSec*1024+p_etc_event->event[i].bmMs - (etc_sensor_list[index].no_sure_off_event.bmSec*1024+etc_sensor_list[index].no_sure_off_event.bmMs);
						if(event_from_next_time<0)
							event_from_next_time += 16*1024;
						
						event_from_next_time = event_from_next_time*1000/1024;
						
						if(slot_count-etc_sensor_list[index].no_sure_off_time_tick >= 128*16)   //实际时间已经大于16秒
						{
								event_from_next_time += ((slot_count-etc_sensor_list[index].no_sure_off_time_tick)/(128*16))*16000; //加上实际过的时间	
						}						
						if(event_from_next_time > etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms)  //满足off->on阈值
						{
							//未确认的临时OFF减去之前ON的时间   这个时间是之前ON持续的时间
							event_from_next_time_1 = (etc_sensor_list[index].no_sure_off_event.bmSec*1024+etc_sensor_list[index].no_sure_off_event.bmMs)
							- (etc_sensor_list[index].now_event.bmSec*1024+etc_sensor_list[index].now_event.bmMs);
							if(event_from_next_time_1<0)
								event_from_next_time_1 += 16*1024;	
							
							event_from_next_time_1 = event_from_next_time_1*1000/1024;
							if(slot_count-etc_sensor_list[index].on_off_time_tick >= 128*16)   //实际时间已经大于16秒
							{
								event_from_next_time_1 += ((slot_count-etc_sensor_list[index].on_off_time_tick)/(128*16))*16000; //加上实际过的时间	
							}								
							
							etc_sensor_list[index].on_time_ms += event_from_next_time_1;		
							if(event_from_next_time/1000 > timer_has_go_time())		
								etc_sensor_list[index].on_time_ms_timer_use += timer_has_go_time()*1000;
							else
								etc_sensor_list[index].on_time_ms_timer_use += event_from_next_time_1;	
							etc_sensor_list[index].now_on_off_status = OFF;							
							make_realtime_data_and_insert_queue(index,event_from_next_time_1);
							//确认上一个OFF事件，此处需要做OFF计数统计 但是实际状态已经到ON 错过了输出时间       //   OFF
							
							
							sprintf(debug_send_buff,"<--OFF--no time out>: id=[%04X] on->off:%dms 有一个未确认的OFF事件\r\n",etc_sensor_list[index].sensor_id,event_from_next_time_1);
							copy_string_to_double_buff(debug_send_buff);		
							//满足阈值   切换为ON
								
							etc_sensor_list[index].car_count++;
							etc_sensor_list[index].off_time_ms += event_from_next_time;
						if(event_from_next_time/1000 > timer_has_go_time())		
							etc_sensor_list[index].off_time_ms_timer_use += timer_has_go_time()*1000;
						else
							etc_sensor_list[index].off_time_ms_timer_use += event_from_next_time;	
							etc_sensor_list[index].now_on_off_status = ON;
							etc_4g_timer_data.struct_etc_data_and_status[index].car_count++;
							etc_led_out(index,etc_sensor_list[index].now_on_off_status);
							etc_sensor_list[index].now_event = p_etc_event->event[i];
							etc_sensor_list[index].on_off_time_tick = slot_count;	
							etc_sensor_list[index].no_sure_off_event.uiAll = 0xffff;
							etc_sensor_list[index].no_sure_off_time_tick = 0;	
							sprintf(debug_send_buff,"<--ON-->: id=[%04X] off->on:%dms \r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
							copy_string_to_double_buff(debug_send_buff);								
						}						
						else  //不满足阈值  取消上一个off事件  不更新这个ON状态
						{
								//不满足阈值  ON事件丢弃
								sprintf(debug_send_buff,"<--no_time off to on-->: id=[%04X] %dms 不满足阈值 丢弃\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
								copy_string_to_double_buff(debug_send_buff);															
						}
					}
					else
					{
					//ON->ON
						sprintf(debug_send_buff,"<--on to on-->: id=[%04X] %d 丢弃\r\n",etc_sensor_list[index].sensor_id,event_from_next_time);
						copy_string_to_double_buff(debug_send_buff);						
						//etc_sensor_list[index].now_event = p_etc_event->event[i];
						//etc_sensor_list[index].on_off_time_tick = slot_count;	
					}
				}
				else //off事件  先写入中间状态 等待超时或者下一个事件确认		
				{
					etc_sensor_list[index].no_sure_off_event = p_etc_event->event[i];
					etc_sensor_list[index].no_sure_off_time_tick = slot_count;
				}				
			}
		}
	}
	
}


//轮训检查sensor的中间状态off是否已经满足超时确认条件
void  time_out_make_sure_off()
{
	uint8_t index;
	int32_t event_from_next_time;
	int32_t real_time_ms;
	int32_t sensor_time_ms;

	int32_t st_slot = slot_count;
	
	for(index=0;index<7;index++)
	{
		if(interrupt_or_main)
		{
			if(SysTick->VAL/168<1000)
				return;
		}
		if(etc_sensor_list[index].no_sure_off_time_tick !=0) //有一个未确认的OFF事件
		{
			if((slot_count - etc_sensor_list[index].no_sure_off_time_tick) >= (etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms/8))
			{				
				event_from_next_time = (etc_sensor_list[index].no_sure_off_event.bmSec*1024+etc_sensor_list[index].no_sure_off_event.bmMs)
				- (etc_sensor_list[index].now_event.bmSec*1024+etc_sensor_list[index].now_event.bmMs);
				if(event_from_next_time<0)
					event_from_next_time += 16*1024;
				
				event_from_next_time = event_from_next_time*1000/1024;
				sensor_time_ms = event_from_next_time;
				real_time_ms = (slot_count-etc_sensor_list[index].on_off_time_tick)*8-etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms;
				if(slot_count-etc_sensor_list[index].on_off_time_tick > (128*16+etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms/8))   //实际时间已经大于16秒
				{
					event_from_next_time += ((slot_count-etc_sensor_list[index].on_off_time_tick-etc_sys_param.etc_sensor_param[index].off_to_on_min_t_ms/8)/(128*16))*16000; //加上实际过的时间
				}
			
				etc_sensor_list[index].now_on_off_status = OFF;                                      //OFF
				etc_led_out(index,etc_sensor_list[index].now_on_off_status);
				etc_sensor_list[index].now_event = etc_sensor_list[index].no_sure_off_event;
				
				etc_sensor_list[index].no_sure_off_event.uiAll = 0xffff;
				etc_sensor_list[index].on_time_ms += event_from_next_time;
				if(event_from_next_time/1000 > timer_has_go_time())		
					etc_sensor_list[index].on_time_ms_timer_use += timer_has_go_time()*1000;
				else
					etc_sensor_list[index].on_time_ms_timer_use += event_from_next_time;	
				make_realtime_data_and_insert_queue(index,event_from_next_time);
				sprintf(debug_send_buff,"<--OFF--超时>: id=[%04X] t=%dms delay=%dms 检测器计算时间=%d 实际间隔时间=%d[%d->%d] %d\r\n",etc_sensor_list[index].sensor_id,event_from_next_time,(slot_count - etc_sensor_list[index].no_sure_off_time_tick)*8,
				sensor_time_ms,real_time_ms,slot_count,etc_sensor_list[index].on_off_time_tick,st_slot);
				copy_string_to_double_buff(debug_send_buff);
				etc_sensor_list[index].on_off_time_tick = etc_sensor_list[index].no_sure_off_time_tick;	
				etc_sensor_list[index].no_sure_off_time_tick = 0;						
			}

		}
		
	}
}



//收到事件包 状态包之后回复ACK 修改对应bit 
//事件包时间槽错误或者状态包参数不匹配,发送设置参数ack包
//或者需要设置升级 校准的时候发送设置包
void etc_rev_data(void *p_rf_data,uint8_t slot,uint8_t index)
{
	struct_etc_event *p_etc_event = (struct_etc_event *)p_rf_data;
	struct_etc_stat *p_etc_stat = (struct_etc_stat *)p_rf_data;
	struct_etc_sensor_ack *p_etc_sensor_ack = (struct_etc_sensor_ack *)p_rf_data;	
	struct_etc_sensor_net_in *p_etc_net_in = (struct_etc_sensor_net_in *)p_rf_data;	
	struct_etc_d_mode * p_etc_d_mode = (struct_etc_d_mode *)p_rf_data;	
	
	
	uint8_t i,event_num=0;
	uint16_t crc;
	uint8_t cha = 0;
	
	if(etc_sensor_list[index].frame_rev_evnet !=0) //一帧已经收到同样sensorid的数据   多余的不处理
	{
		if(pktStatus.Params.LoRa.RssiPkt > etc_sensor_list[index].rssi)
		{
			etc_sensor_list[index].rssi = pktStatus.Params.LoRa.RssiPkt;
			etc_4g_timer_data.struct_etc_data_and_status[index].rssi = etc_sensor_list[index].rssi;
		}
		sprintf(debug_send_buff,"rf%d-->[%d]\r\n",radio_switch+1,pktStatus.Params.LoRa.RssiPkt);
		copy_string_to_double_buff(debug_send_buff);	
		return;
	}
	etc_sensor_list[index].frame_rev_evnet = 1;
	
	//置位ACK
	if(slot < 8)
		etc_ack.ack_bit |= 1<<slot;
	
	
	//事件包
	if(p_etc_event->etc_head.type == SENSOR_EVENT_PACKET)
	{
		etc_sensor_list[index].no_data_times = 0;//清除无数据超时计数
		if(etc_sensor_list[index].updata_status == 2) //收到事件包  表示升级失败
		{
			etc_sensor_list[index].updata_status = 0;
		}
		
		if(slot != index) //时间槽错误  设置时间槽 ， 后一个sensor会替代前面的
		{
			etc_ack.etc_head.type = MASTER_ACK_PACKET;
			etc_ack.etc_head.cmd = SET_PARAM_CMD;
			etc_ack.etc_set_param.sensor_id = etc_sensor_list[index].sensor_id;
			etc_ack.etc_set_param.slot = index;
			etc_ack.etc_set_param.ch = etc_jump_channel;
		}
		//第一包
		if(etc_sensor_list[index].now_etc_event_packet.etc_head.dev_id == 0)
		{
			//记录当前数据包
			etc_sensor_list[index].now_etc_event_packet = *p_etc_event;
			etc_sensor_list[index].event_count = 1;
			etc_sensor_list[index].event_lost_count = 0;
			etc_sensor_list[index].event_resend_count = 0;
			get_event_to_hanle(p_etc_event,index,4,0);
		}
		else
		{
			//包序号相同 重传次数更新
			if(p_etc_event->etc_head.seq == etc_sensor_list[index].now_etc_event_packet.etc_head.seq) 
			{
				
			}
			else //新的包序号
			{
				etc_sensor_list[index].timer_rssi += etc_sensor_list[index].rssi;
				etc_sensor_list[index].all_add_rssi += etc_sensor_list[index].rssi;
				etc_sensor_list[index].event_count++;				
				etc_sensor_list[index].rssi = pktStatus.Params.LoRa.RssiPkt;
			
				
				//计算丢包率 去掉重复事件 计算出这一包里有几个有效事件
				cha = p_etc_event->etc_head.seq - etc_sensor_list[index].now_etc_event_packet.etc_head.seq;
				
				if(cha < 0)
					cha += 256;
				
				for(i=0;i<4;i++)
				{
					if(p_etc_event->event[i].uiAll != 0xffff && p_etc_event->event[i].bmSec <=16) //无效事件
						event_num++;
				}
	
				
				if(cha - event_num>0)
					etc_sensor_list[index].event_lost_count += cha - event_num;
				
				etc_4g_timer_data.struct_etc_data_and_status[index].packet_count++;
				etc_4g_timer_data.struct_etc_data_and_status[index].packet_lost_count += cha - event_num;
				
				get_event_to_hanle(p_etc_event,index,cha,1);
				

			}
			//	记录重传次数
			if((p_etc_event->resend_times == 0 && etc_sensor_list[index].now_etc_event_packet.resend_times != 0)||
				(etc_sensor_list[index].now_etc_event_packet.resend_times != 0 && p_etc_event->etc_head.seq != etc_sensor_list[index].now_etc_event_packet.etc_head.seq))
			{
				etc_4g_timer_data.struct_etc_data_and_status[index].packet_resend_count += etc_sensor_list[index].now_etc_event_packet.resend_times;
				etc_sensor_list[index].event_resend_count += etc_sensor_list[index].now_etc_event_packet.resend_times;
				if(etc_sensor_list[index].now_etc_event_packet.resend_times <= 9)
				{
					etc_sensor_list[index].repeat_packet_times_count[etc_sensor_list[index].now_etc_event_packet.resend_times-1]++;
				}
				else
					etc_sensor_list[index].repeat_packet_times_count[9]++;
			}	
			//记录当前数据包
			etc_sensor_list[index].now_etc_event_packet = *p_etc_event;			
			
		}
		sprintf(debug_send_buff,"%d rev_event_%d: id=[%04X] [%04X:%d->%d.%d %04X:%d->%d.%d %04X:%d->%d.%d %04X:%d->%d.%d] slot=%d %d->%d seq=%d re_s_times=%d\r\n",
		pktStatus.Params.LoRa.RssiPkt,radio_switch+1,p_etc_event->etc_head.dev_id,p_etc_event->event[0].uiAll,p_etc_event->event[0].blIsOn,p_etc_event->event[0].bmSec,p_etc_event->event[0].bmMs,
			p_etc_event->event[1].uiAll,p_etc_event->event[1].blIsOn,p_etc_event->event[1].bmSec,p_etc_event->event[1].bmMs,
		p_etc_event->event[2].uiAll, p_etc_event->event[2].blIsOn,p_etc_event->event[2].bmSec,p_etc_event->event[2].bmMs, p_etc_event->event[3].uiAll,p_etc_event->event[3].blIsOn,p_etc_event->event[3].bmSec,p_etc_event->event[3].bmMs,
		etc_ack.etc_head.seq,slot,SysTick->VAL/168,p_etc_event->etc_head.seq,p_etc_event->resend_times);
		copy_string_to_double_buff(debug_send_buff);		
		
	}//状态包
	else if(p_etc_event->etc_head.type == SENSOR_STAT_PACKET)
	{
		etc_sensor_list[index].no_data_times = 0;
		if(slot != index || p_etc_stat->band_id != etc_ack.etc_head.dev_id) 
			//时间槽错误 或者绑定id不同 设置时间槽 ， 后一个sensor会替代前面的
		{
			etc_ack.etc_head.type = MASTER_ACK_PACKET;
			etc_ack.etc_head.cmd = SET_PARAM_CMD;
			etc_ack.etc_set_param.sensor_id = etc_sensor_list[index].sensor_id;
			etc_ack.etc_set_param.slot = index;
			etc_ack.etc_set_param.ch = etc_jump_channel;
		}		
		if(etc_sensor_list[index].updata_status == 2) //收到包  表示升级失败
		{
			etc_sensor_list[index].updata_status = 0;
		}		
		if(p_etc_stat->adjust_flag) //校准成功
		{
			etc_sensor_list[index].adjust_count++;
			etc_sensor_list[index].adjust_status = ADJUST_NONE;
			test_led_switch(index,0);
		}
		etc_sensor_list[index].rssi = pktStatus.Params.LoRa.RssiPkt;
		etc_4g_timer_data.struct_etc_data_and_status[index].rssi = etc_sensor_list[index].rssi;
		etc_4g_timer_data.struct_etc_data_and_status[index].voltage = p_etc_stat->battery;
		etc_4g_timer_data.struct_etc_data_and_status[index].sensor_rev_rssi = p_etc_stat->rssi;
		etc_4g_timer_data.struct_etc_data_and_status[index].sensor_software_version = p_etc_stat->s_version;
		sprintf(debug_send_buff,"%d rev_stat_%d: id=[%04X] slot=%d %d seq=%d bandid=%04X s_rev_rssi=%d,V=%d s_v=%d flag=%d\r\n",pktStatus.Params.LoRa.RssiPkt,radio_switch+1,
			p_etc_stat->etc_head.dev_id,etc_ack.etc_head.seq,slot,p_etc_event->etc_head.seq,p_etc_stat->band_id,p_etc_stat->rssi,p_etc_stat->battery,p_etc_stat->s_version, p_etc_stat->adjust_flag);
		copy_string_to_double_buff(debug_send_buff);				
		
	}//sensor ack包
	else if(p_etc_event->etc_head.type == SENSOR_UPDATA_ACK_PACKET)
	{
		etc_sensor_list[index].no_data_times = 0;
		if(etc_sensor_list[index].updata_status == 1) //进入升级模式
		{
			etc_sensor_list[index].updata_status = 2;
		}				
		if(etc_sensor_list[index].updata_status == 2) //记录sensor升级进度地址
		{
			etc_sensor_list[index].sensor_addr = p_etc_sensor_ack->address;
			etc_sensor_updata_status.sensor_updata_status[index].sensor_id = etc_sensor_list[index].sensor_id;
			if(etc_sensor_list[index].sensor_addr >= 0x7100)
				etc_sensor_updata_status.sensor_updata_status[index].updata_percent = (etc_sensor_list[index].sensor_addr - 0x7100)*100/(pheader_5xx.uiFwSize);
			else
				etc_sensor_updata_status.sensor_updata_status[index].updata_percent = 0;
		}		
		sprintf(debug_send_buff,"%d rev_updata_ack_%d: id=[%04X] slot=%d %d seq=%d addr=%X event=%04X\r\n",pktStatus.Params.LoRa.RssiPkt,radio_switch+1,
			p_etc_stat->etc_head.dev_id,etc_ack.etc_head.seq,slot,p_etc_event->etc_head.seq,p_etc_sensor_ack->address,p_etc_sensor_ack->event[0].uiAll);
		copy_string_to_double_buff(debug_send_buff);	
	//第一包
		if(etc_sensor_list[index].now_etc_event_packet.etc_head.dev_id == 0)
		{
			//记录当前数据包
			etc_sensor_list[index].now_etc_event_packet = *p_etc_event;
			etc_sensor_list[index].event_count = 1;
			etc_sensor_list[index].event_lost_count = 0;
			etc_sensor_list[index].event_resend_count = 0;
			get_ack_event_to_hanle((struct_etc_sensor_ack*)p_etc_event,index,3,0);
		}
		else
		{
			//包序号相同 重传次数更新
			if(p_etc_event->etc_head.seq == etc_sensor_list[index].now_etc_event_packet.etc_head.seq) 
			{
				
			}
			else //新的包序号
			{
				etc_sensor_list[index].timer_rssi += etc_sensor_list[index].rssi;
				etc_sensor_list[index].all_add_rssi += etc_sensor_list[index].rssi;
				etc_sensor_list[index].event_count++;				
				etc_sensor_list[index].rssi = pktStatus.Params.LoRa.RssiPkt;
			
				
				//计算丢包率 去掉重复事件 计算出这一包里有几个有效事件
				cha = p_etc_event->etc_head.seq - etc_sensor_list[index].now_etc_event_packet.etc_head.seq;
				
				if(cha < 0)
					cha += 256;
				
				for(i=0;i<4;i++)
				{
					if(p_etc_event->event[i].uiAll != 0xffff && p_etc_event->event[i].bmSec <=16) //无效事件
						event_num++;
				}
	
				
				if(cha - event_num>0)
					etc_sensor_list[index].event_lost_count += cha - event_num;
				
				etc_4g_timer_data.struct_etc_data_and_status[index].packet_count++;
				etc_4g_timer_data.struct_etc_data_and_status[index].packet_lost_count += cha - event_num;
				
				get_ack_event_to_hanle((struct_etc_sensor_ack*)p_etc_event,index,cha,1);
				

			}
		}		
		//记录当前数据包
		etc_sensor_list[index].now_etc_event_packet = *p_etc_event;		
	}	//sensor net in包
	else if(p_etc_event->etc_head.type == SENSOR_NET_IN_PACKET)
	{
		if(index != p_etc_net_in->slot || p_etc_net_in->band_id != etc_ack.etc_head.dev_id) //时间槽错误  设置时间槽 ， 后一个sensor会替代前面的
		{
			etc_ack.etc_head.type = MASTER_ACK_PACKET;
			etc_ack.etc_head.cmd = SET_PARAM_CMD;
			etc_ack.etc_set_param.sensor_id = etc_sensor_list[index].sensor_id;
			etc_ack.etc_set_param.slot = index;
			etc_ack.etc_set_param.ch = etc_jump_channel;
		}		
		etc_4g_timer_data.struct_etc_data_and_status[index].net_out_times++;
		etc_4g_timer_data.struct_etc_data_and_status[index].net_out_time += p_etc_net_in->net_out_time;
		etc_sensor_list[index].net_in_times += p_etc_net_in->net_in_times;
		etc_sensor_list[index].net_out_time += p_etc_net_in->net_out_time;
		sprintf(debug_send_buff,"%d rev_net_in_%d: id=[%04X] slot=%d %d seq=%d bandid=%04X netin_times=%d,netin_time=%d\r\n",
		pktStatus.Params.LoRa.RssiPkt,radio_switch+1,p_etc_net_in->etc_head.dev_id, p_etc_net_in->slot ,slot,p_etc_net_in->etc_head.seq,
			p_etc_net_in->band_id,p_etc_net_in->net_in_times,p_etc_net_in->net_out_time);
		copy_string_to_double_buff(debug_send_buff);	

	}	
	else if(p_etc_event->etc_head.type == SENSOR_D_MODE_DATA)
	{

		sprintf(debug_send_buff,"%d rev_d_mode_%d: id=[%04X] slot=%d seq=%d %d %d %d %d %d %d %d %d %d\r\n",
		pktStatus.Params.LoRa.RssiPkt,radio_switch+1,p_etc_net_in->etc_head.dev_id, slot,p_etc_d_mode->etc_head.seq,
			p_etc_d_mode->data[0],p_etc_d_mode->data[1],p_etc_d_mode->data[2],p_etc_d_mode->data[3],p_etc_d_mode->data[4],
		p_etc_d_mode->data[5],p_etc_d_mode->data[6],p_etc_d_mode->data[7],p_etc_d_mode->data[8]);
		copy_string_to_double_buff(debug_send_buff);	

	}		
	
	
}

uint8_t rf_wave_en = 0;


extern int syn_disable_0;
extern int syn_disable_1;

void etc_send_ack(uint32_t rf_index)
{
	uint32_t delay = 16800;
	uint32_t tick;
	while(delay--);
	
	if(rf_index == 0)
	{
		if(!syn_disable_0)
			return;
	}
	if(rf_index == 1)
	{
		if(!syn_disable_1)
			return;
	}	
	
	if(rf_index == 0)
		etc_make_ack();
	
	
	if(etc_ack.ack_bit != 0 || rf_wave_en)  //有ack标志位   启动rf发送  发送后清除
	{
		etc_ack.etc_head.rf_index = rf_index;
		if(rf_index == 0)
		{			
			Radio_write_data((uint8_t*)&etc_ack,RF1,sizeof(etc_ack));
			if(rf_wave_en == 0)
				Radio_start_send(RF1);
			else if(rf_wave_en == 1)
			{
				rf_wave_en = 2;
				radio_switch = RF1;
				Radio.SetTxContinuousWave();
			}
		}
		else if(rf_index == 1)
		{
			Radio_write_data((uint8_t*)&etc_ack,RF2,sizeof(etc_ack));			
			if(rf_wave_en == 0)
				Radio_start_send(RF2);
			else if(rf_wave_en == 2)
			{
				rf_wave_en = 3;
				radio_switch = RF2;
				Radio.SetTxContinuousWave();
			}
		}

		tick = SysTick->VAL/168;
		if(etc_ack.etc_head.type == MASTER_UPDATA)
		{
			sprintf(debug_send_buff,"updata_ack_[%d:%d]->%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d addr=%X \r\n",
			nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd, etc_ack.etc_updata.address);
			copy_string_to_double_buff(debug_send_buff);			
		}
		else if(etc_ack.etc_head.cmd == NO_CMD)
		{
			  sprintf(debug_send_buff,"send_ack_[%d:%d]%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d ch8=%04x%0x\r\n",
				nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd,
			  etc_ack.etc_comm.ch.ch0,etc_ack.etc_comm.ch.ch1);
			  copy_string_to_double_buff(debug_send_buff);
		}
		else if(etc_ack.etc_head.cmd == SET_PARAM_CMD)
		{
			sprintf(debug_send_buff,"send_ack_[%d:%d]->%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d,s_id=%04X slot=%d\r\n",
				nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd,
					etc_ack.etc_set_param.sensor_id,etc_ack.etc_set_param.slot);
			copy_string_to_double_buff(debug_send_buff);
		}		
		else if(etc_ack.etc_head.cmd == ADJUST_CMD)
		{
			sprintf(debug_send_buff,"adjust_ack_[%d:%d]->%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d,s_id=%04X s_id=%04X s_id=%04X s_id=%04X\r\n",
				nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd,
					etc_ack.etc_set_jiaozhun_or_updata.sensor_id[0],etc_ack.etc_set_jiaozhun_or_updata.sensor_id[1],etc_ack.etc_set_jiaozhun_or_updata.sensor_id[2],
			etc_ack.etc_set_jiaozhun_or_updata.sensor_id[3]);
			copy_string_to_double_buff(debug_send_buff);
		}			
		else if(etc_ack.etc_head.cmd == UPDATA_CMD)
		{
			sprintf(debug_send_buff,"updata_cmd_ack_[%d:%d]->%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d,s_id=%04X s_id=%04X s_id=%04X s_id=%04X\r\n",
				nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd,
					etc_ack.etc_set_jiaozhun_or_updata.sensor_id[0],etc_ack.etc_set_jiaozhun_or_updata.sensor_id[1],etc_ack.etc_set_jiaozhun_or_updata.sensor_id[2],
			etc_ack.etc_set_jiaozhun_or_updata.sensor_id[3]);
			copy_string_to_double_buff(debug_send_buff);
		}				
		else if(etc_ack.etc_head.cmd == SET_ELSE_PARAM)
		{
			sprintf(debug_send_buff,"set_elseparam_cmd_ack_[%d:%d]%d: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d,s_id=%04X data=%d\r\n",
				nodata_rssi[0],nodata_rssi[1],tick,etc_ack.etc_head.dev_id,slot_count,etc_ack.etc_head.seq,now_ch,etc_ack.ack_bit,etc_ack.etc_head.cmd,
					etc_ack.etc_set_else_param.sensor_id,etc_ack.etc_set_else_param.data[0]);
			copy_string_to_double_buff(debug_send_buff);
		}	
	}
	else
	{
		get_nodata_rssi();
	}

	if(rf_index == 1 || syn_disable_1 == 0)
	{
		etc_ack.ack_bit = 0;
		etc_ack.etc_head.type = MASTER_ACK_PACKET;
		etc_ack.etc_head.cmd = NO_CMD;		
		etc_ack.etc_set_param.ch = etc_jump_channel;
		if(interrupt_or_main == 0)
			time_out_make_sure_off();		
	}

	

}

//执行跳频动作  检查如果需要复位射频 执行复位
int32_t etc_jump_ch(uint32_t index)
{
	uint32_t i = 0;
	
	if(index>7)
		return -1;
		
	if(index < 6)
	{
		now_ch = (etc_sensor_list[index].sensor_id!=0 ? ((etc_sensor_list[index].sensor_id%CH_COUNT)&0x1f) : (etc_sensor_list[index].sensor_id_is_zero_ch&0x1f));
	}
	else if(index == 7 || index == 6)
	{
		now_ch = (ap_id&0x1f);		
	}
	
	if(radio_status[RF1].reboot_status == WAIT_REBOOT) //有复位标志
	{
		etc_reboot_rf(RF1);
		radio_status[RF1].reboot_status = WORK;
	}
	else
		Radio_set_rf_ch(RF1,now_ch);
	
	if(radio_status[RF2].reboot_status == WAIT_REBOOT) //有复位标志
	{
		etc_reboot_rf(RF2);
		radio_status[RF2].reboot_status = WORK;
	}
	else	
		Radio_set_rf_ch(RF2,now_ch);	
}

int32_t etc_jump_ch_monitor(uint32_t index)
{
	uint32_t i = 0;
	
	if(index>7)
		return -1;
		
	if(index < 6)
	{
		now_ch = (etc_jump_channel.ch0>>(index*5))&0x1f;
		Radio_set_rf_ch(RF1,now_ch);
		
		Radio_set_rf_ch(RF2,now_ch);
	}
	else if(index == 6)
	{
		now_ch = ((etc_jump_channel.ch0>>30)&0x03)|((etc_jump_channel.ch1&0x07)<<2);
		Radio_set_rf_ch(RF1,now_ch);
		
		Radio_set_rf_ch(RF2,now_ch);	
	}
	else if(index == 7)
	{
		now_ch = ((etc_jump_channel.ch1>>3)&0x1f);
		Radio_set_rf_ch(RF1,now_ch);
		
		Radio_set_rf_ch(RF2,now_ch);	
	}	
}




void etc_else_ap_or_sensor_packet_handle(void *p_rf_data,uint8_t slot)
{
	struct_etc_event *p_etc_event = (struct_etc_event *)p_rf_data;
	struct_etc_stat *p_etc_stat = (struct_etc_stat *)p_rf_data;
	struct_etc_sensor_ack *p_etc_sensor_ack = (struct_etc_sensor_ack *)p_rf_data;	
	struct_etc_ack *p_etc_ap_ack = (struct_etc_ack *)p_rf_data;	
	
	int8_t lost_count;
//	if(p_etc_event->etc_head.type == SENSOR_EVENT_PACKET)
//	{
//		
//		sprintf(debug_send_buff,"%d ch=%d -------------------------rev_event: id=%04X seq=%d slot=%d:%d  re_s_times=%d\r\n",
//		((signed char *)p_rf_data)[14]-76,now_ch,p_etc_event->etc_head.dev_id,etc_ack.etc_head.seq,slot,SysTick->VAL/168,p_etc_event->etc_head.seq,p_etc_event->resend_times);
//		copy_string_to_double_buff(debug_send_buff);		
//		
//		
//	}//状态包
//	else if(p_etc_event->etc_head.type == SENSOR_STAT_PACKET)
//	{
//		
//		sprintf(debug_send_buff,"%d ch=%d -------------------------rev_stat: id=%04X slot=%d %d seq=%d bandid=%04X s_rev_rssi=%d,V=%d\r\n",((signed char *)p_rf_data)[14]-76,
//			now_ch,p_etc_event->etc_head.dev_id,etc_ack.etc_head.seq,slot,p_etc_event->etc_head.seq,p_etc_stat->band_id,p_etc_stat->rssi,p_etc_stat->battery);
//		copy_string_to_double_buff(debug_send_buff);				
//		
//		
//	}//sensor ack包
//	else 
if(p_etc_event->etc_head.type == MASTER_ACK_PACKET && 
	(ap_id & 0x0000ffff) == p_etc_event->etc_head.dev_id &&
	enable_send_ack_forever>0) //本机的ack包
	{
//		etc_ack.etc_head.seq = p_etc_ap_ack->etc_head.seq;
		if(p_etc_ap_ack->etc_head.rf_index == 0)
		{
			if(test_rev_rf_rate[0].rev_packet_count == 0)
			{
				test_rev_rf_rate[0].rev_packet_count = 1;
				test_rev_rf_rate[0].real_rev_packet_count = 1;
				test_rev_rf_rate[0].seq = p_etc_ap_ack->etc_head.seq;
			}
			else if(test_rev_rf_rate[0].rev_packet_count<100)
			{
				lost_count = p_etc_ap_ack->etc_head.seq - test_rev_rf_rate[0].seq;
				if(lost_count<0)
					lost_count += 256;
				test_rev_rf_rate[0].real_rev_packet_count++;
				test_rev_rf_rate[0].rssi += pktStatus.Params.LoRa.RssiPkt;
				test_rev_rf_rate[0].rev_packet_count += lost_count;
				test_rev_rf_rate[0].lost_packet_count += lost_count-1;
				test_rev_rf_rate[0].seq = p_etc_ap_ack->etc_head.seq;
			}
			else if(test_rev_rf_rate[0].rev_packet_count == 100)
			{
				test_rev_rf_rate[0].rev_packet_count = 101;
				test_rev_rf_rate[0].rssi = test_rev_rf_rate[0].rssi/test_rev_rf_rate[0].real_rev_packet_count;
				sprintf(debug_send_buff,"test_rf2_rev wish=%d real=%d rssi=%d\r\n",test_rev_rf_rate[0].rev_packet_count-1,test_rev_rf_rate[0].real_rev_packet_count,test_rev_rf_rate[0].rssi);
				copy_string_to_double_buff(debug_send_buff);
			}
		}
		else
		{
		if(test_rev_rf_rate[1].rev_packet_count == 0)
			{
				test_rev_rf_rate[1].rev_packet_count = 1;
				test_rev_rf_rate[1].real_rev_packet_count = 1;
				test_rev_rf_rate[1].seq = p_etc_ap_ack->etc_head.seq;
			}
			else if(test_rev_rf_rate[1].rev_packet_count<100)
			{
				lost_count = p_etc_ap_ack->etc_head.seq - test_rev_rf_rate[1].seq;
				if(lost_count<0)
					lost_count += 256;
				test_rev_rf_rate[1].real_rev_packet_count++;
				test_rev_rf_rate[1].rssi += pktStatus.Params.LoRa.RssiPkt;
				test_rev_rf_rate[1].rev_packet_count += lost_count;
				test_rev_rf_rate[1].lost_packet_count += lost_count-1;
				test_rev_rf_rate[1].seq = p_etc_ap_ack->etc_head.seq;
			}
			else if(test_rev_rf_rate[1].rev_packet_count == 100)
			{
				test_rev_rf_rate[1].rev_packet_count = 101;
				test_rev_rf_rate[1].rssi = test_rev_rf_rate[1].rssi/test_rev_rf_rate[1].real_rev_packet_count;
				sprintf(debug_send_buff,"test_rf1_rev wish=%d real=%d rssi=%d\r\n",test_rev_rf_rate[1].rev_packet_count-1,test_rev_rf_rate[1].real_rev_packet_count,test_rev_rf_rate[1].rssi);
				copy_string_to_double_buff(debug_send_buff);
			}	
		if(test_rev_rf_rate[1].rev_packet_count == 101 && test_rev_rf_rate[0].rev_packet_count == 101)
			enable_send_ack_forever = -1;
		}
		if(p_etc_ap_ack->etc_head.cmd == NO_CMD)
			{
//				etc_jump_channel.ch0 = p_etc_ap_ack->etc_comm.ch.ch0;
//				etc_jump_channel.ch1 = p_etc_ap_ack->etc_comm.ch.ch1;
				sprintf(debug_send_buff,"%d ch=%d-------------------------rev_ack: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d ch8=%04x%0x [seq=%d time=%d:%d]\r\n",
					pktStatus.Params.LoRa.RssiPkt,now_ch,p_etc_ap_ack->etc_head.dev_id,slot_count,p_etc_ap_ack->etc_head.seq,now_ch,p_etc_ap_ack->ack_bit,p_etc_ap_ack->etc_head.cmd,
						p_etc_ap_ack->etc_comm.ch.ch0,p_etc_ap_ack->etc_comm.ch.ch1,etc_ack.etc_head.seq,slot,SysTick->VAL/168);
				copy_string_to_double_buff(debug_send_buff);
			}
			else if(p_etc_ap_ack->etc_head.cmd == SET_PARAM_CMD)
			{
//				etc_jump_channel.ch0 = p_etc_ap_ack->etc_set_param.ch.ch0;
//				etc_jump_channel.ch1 = p_etc_ap_ack->etc_set_param.ch.ch1;				
				sprintf(debug_send_buff,"%d ch=%d-------------------------rev_ack: id=%04X seq=%d:%d ch=%d ack=%02X cmd=%d,s_id=%04X slot=%d [seq=%d time=%d:%d]\r\n",
					pktStatus.Params.LoRa.RssiPkt,now_ch,p_etc_ap_ack->etc_head.dev_id,slot_count,p_etc_ap_ack->etc_head.seq,now_ch,p_etc_ap_ack->ack_bit,p_etc_ap_ack->etc_head.cmd,
						p_etc_ap_ack->etc_set_param.sensor_id,p_etc_ap_ack->etc_set_param.slot,etc_ack.etc_head.seq, slot,SysTick->VAL/168);
				copy_string_to_double_buff(debug_send_buff);
			}						
//	else if(p_etc_event->etc_head.cmd == UPDATA_CMD)
//	{
//		
//		sprintf(debug_send_buff,"rev_updata_cmd_ack: id=%04X seq=%d slot=%d:%d ch=%d ack=%02X cmd=%d addr=%d \r\n",
//				p_etc_ap_ack->etc_head.dev_id,etc_ack.etc_head.seq,slot,SysTick->VAL/168,now_ch,p_etc_ap_ack->ack_bit,p_etc_ap_ack->etc_head.cmd,
//		p_etc_ap_ack->etc_updata.address);
//			copy_string_to_double_buff(debug_send_buff);				
//		
//		
//	}//sensor ack包			
	}	
	else if(p_etc_event->etc_head.type == MASTER_ACK_PACKET && enable_rev_1000)
	{
		if(rev_id_1000 == 0)
			rev_id_1000 = p_etc_event->etc_head.dev_id;
		
		if(rev_id_1000 == p_etc_event->etc_head.dev_id && rev_1000_now_seq != p_etc_event->etc_head.seq)
		{
			if(rev_1000_count > 1000)
			{
				rev_1000_count = 0;
				rev_1000_lost = 0;
			}
			if(rev_1000_count == 0)
			{
				rev_1000_now_seq = p_etc_event->etc_head.seq;
				rev_1000_count = 1;
			}
			else
			{
				lost_count = p_etc_event->etc_head.seq - rev_1000_now_seq;
				if(lost_count<0)
					lost_count += 256;
				rev_1000_now_seq = p_etc_event->etc_head.seq;
				rev_1000_count += lost_count;
				rev_1000_lost += lost_count-1;
			}
		}
		sprintf(debug_send_buff,"%d [ch:%d]rev_1000: id=%04X seq=%d all=%d lost=%d\r\n",
			pktStatus.Params.LoRa.RssiPkt,now_ch,p_etc_ap_ack->etc_head.dev_id,p_etc_ap_ack->etc_head.seq,rev_1000_count,rev_1000_lost);
		copy_string_to_double_buff(debug_send_buff);		
	}
//	else if(p_etc_event->etc_head.type == MASTER_UPDATA)
//	{
//		
//		sprintf(debug_send_buff,"rev_updata_ack: id=%04X seq=%d:%d slot=%d:%d ch=%d ack=%02X cmd=%d addr=%d \r\n",
//				p_etc_ap_ack->etc_head.dev_id,slot_count,p_etc_ap_ack->etc_head.seq,slot,SysTick->VAL/168,now_ch,p_etc_ap_ack->ack_bit,p_etc_ap_ack->etc_head.cmd,
//		p_etc_ap_ack->etc_updata.address);
//			copy_string_to_double_buff(debug_send_buff);				
//		
//		
//	}//sensor ack包			
//	else
//	{
//		sprintf(debug_send_buff,"%d ch=%d-------------------------rev_xxxxxxxxx: id=%04X slot=%d %d seq=%d bandid=%04X slot=%d:%d s_rev_rssi=%d,V=%d\r\n",
//		((signed char *)p_rf_data)[14]-76,now_ch,p_etc_event->etc_head.dev_id,etc_ack.etc_head.seq,slot,p_etc_event->etc_head.seq,p_etc_stat->band_id,
//			slot,SysTick->VAL/168,p_etc_stat->rssi,p_etc_stat->battery);
//		copy_string_to_double_buff(debug_send_buff);				
//	}	
}


extern uint8_t gprs_sec_flag; 

//中断里每秒调用 用来置位一些任务标志
void etc_sec_set_flag()
{
	static uint32_t n_sec = 0;
	uint16_t now_sec;
	
	n_sec++;
	gprs_sec_flag = 1;
	radio_status[RF1].rf_rev_nothing_time++;
	radio_status[RF2].rf_rev_nothing_time++;	
	led_sec = 1;
	etc_sec++;
	if(enable_send_ack_forever>0)
		enable_send_ack_forever--;
	if(n_sec == timer_reboot_system)
	{
		HAL_NVIC_SystemReset();
	}
		
	if(n_sec%5 == 0)
	{
to_4g_task.realtime_data_flag = 1;
		to_4g_task.timer_data_flag = 1;		
		to_4g_task.io_data_flag = 1;
		usb_key_5sec = 1;
		if(etc_updata_s_manage.update_s_enable == START_UPDATA)
		{
			to_4g_task.updata_flag = 1;
		}
	}	

	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);	
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);	
	now_sec = rtc_time.Seconds + rtc_time.Minutes*60;
	if(now_sec%etc_sys_param.etc_ap_param.data_status_save_timer == 0)
	{
		to_4g_task.timer_data_make_flag = 1;
	}	
}


uint32_t timer_has_go_time()
{
	uint16_t now_sec;
	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);	
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);	
	now_sec = rtc_time.Seconds + rtc_time.Minutes*60;
	return(now_sec%etc_sys_param.etc_ap_param.data_status_save_timer);	
}

void HAL_SYSTICK_Callback(void)
{
	etc_ack.etc_head.dev_id = ap_id;
	
	
	if(start_sys_timer == 0)
		return;
	if(enable_send_ack_forever>0)
		etc_ack.ack_bit |= 0x80;  //test for RF


	if(slot_count%(128) == 1)
	{	
		etc_sec_set_flag();
	}
	
	now_slot++;
	slot_count++;
	now_slot &= 0x07;
	
	if(now_slot == 7 || now_slot == 6)
	{
		etc_send_ack(now_slot-6);
	}
	
	if(now_slot== 0 && rf_wave_en == 0)
	{
		etc_ack.etc_head.seq++;
		if(enable_send_ack_forever <= 0 && enable_rev_1000 == 0)
			etc_jump_ch(etc_ack.etc_head.seq%8);	 //跳频		

		clear_sensor_flag_frame();

	}		
			
	calc_ch_from_sensor_list(etc_sensor_list,&etc_jump_channel);


	scan_button();
}


void clear_etc()
{
	uint32_t i = 0;
	uint32_t j = 0;
	
	for(i=0;i<IO_COUNT;i++)
	{
		etc_sensor_list[i].rssi = 0;
		etc_sensor_list[i].all_add_rssi = 0;
		etc_sensor_list[i].event_count = 0;
		etc_sensor_list[i].event_lost_count = 0;
		etc_sensor_list[i].event_resend_count = 0;
		for(j=0;j<10;j++)
			etc_sensor_list[i].repeat_packet_times_count[j] = 0;
		
		etc_sensor_list[i].car_count = 0;
		etc_sensor_list[i].on_time_ms = 0;
		etc_sensor_list[i].off_time_ms = 0;
		
		etc_sensor_list[i].net_in_times = 0;
		etc_sensor_list[i].net_out_time = 0;
		
	}	
}


void	etc_adjust_(uint32_t index)
{
	etc_sensor_list[index].adjust_status = ADJUST_ENABLE;
	test_led_switch(index,1);
}

void	etc_updata_(uint32_t index)
{
	etc_sensor_list[index].updata_status = UPDATA_ENABLE;
	etc_updata_s_manage.update_s_enable = ENABLE_UPDATA;
	etc_updata_s_manage.updata_send_cmd_timeout = slot_count + 128*61;
}

void etc_updata_set_id(uint16_t sensor_id)
{
	uint8_t i;
	for(i=0;i<7;i++)
	{
		if(etc_sensor_list[i].sensor_id == sensor_id)
		{
			etc_sensor_list[i].updata_status = UPDATA_ENABLE;		
			etc_updata_s_manage.update_s_enable = ENABLE_UPDATA;
			etc_updata_s_manage.updata_send_cmd_timeout = slot_count + 128*61;			
		}
	}
}

void etc_adjust_set_id(uint16_t sensor_id)
{
	uint8_t i;
	for(i=0;i<7;i++)
	{
		if(etc_sensor_list[i].sensor_id == sensor_id)
		{
			etc_sensor_list[i].adjust_status = ADJUST_ENABLE;	
			test_led_switch(i,1);			
		}
	}
}

void etc_set_else_param_fun(uint32_t index,uint8_t data)
{
	if(index<7)	
		{
			etc_sensor_list[index].else_param[0] = data;				
		}
}


void etc_make_ack()
{
	uint8_t i;
	uint8_t j = 0;
	
	if(etc_ack.ack_bit)
	{
		if(etc_ack.etc_head.cmd == NO_CMD)
		{
			for(i=0;i<7;i++)
			{
				if(etc_sensor_list[i].updata_status == UPDATA_ENABLE && (etc_ack.ack_bit & (1<<i)))
				{
					if(j>3)
						return;
					etc_ack.etc_head.type = MASTER_ACK_PACKET;
					etc_ack.etc_head.cmd = UPDATA_CMD;
					etc_ack.etc_set_jiaozhun_or_updata.sensor_id[j++] = etc_sensor_list[i].sensor_id;
				}
			}
		}
		
		if(etc_ack.etc_head.cmd == NO_CMD)
		{
			for(i=0;i<7;i++)
			{
				if(etc_sensor_list[i].adjust_status == ADJUST_ENABLE && (etc_ack.ack_bit & (1<<i)))
				{
					if(j>3)
						return;
					etc_ack.etc_head.type = MASTER_ACK_PACKET;
					etc_ack.etc_head.cmd = ADJUST_CMD;
					etc_ack.etc_set_jiaozhun_or_updata.sensor_id[j++] = etc_sensor_list[i].sensor_id;
				}
			}
		}	
		if(etc_ack.etc_head.cmd == NO_CMD)
		{
			for(i=0;i<7;i++)
			{
				if(etc_sensor_list[i].else_param[0] !=0 && (etc_ack.ack_bit & (1<<i)))
				{
					etc_ack.etc_head.type = MASTER_ACK_PACKET;
					etc_ack.etc_head.cmd = SET_ELSE_PARAM;
					etc_ack.etc_set_else_param.sensor_id = etc_sensor_list[i].sensor_id;
					etc_ack.etc_set_else_param.data[0] = etc_sensor_list[i].else_param[0];
					etc_sensor_list[i].else_param[0] = 0;
				}
			}
		}			
		
		if(etc_ack.etc_head.cmd == ADJUST_CMD || etc_ack.etc_head.cmd == UPDATA_CMD)
		{
			for(;j<4;j++)
			{		
				etc_ack.etc_set_jiaozhun_or_updata.sensor_id[j] = 0;
			}
		}
	}
	if(etc_ack.etc_head.cmd == NO_CMD)
	{
		etc_make_updata_packet();
	}		
	
}



//收到升级指令
void etc_do_updata_cmd()
{
	
}

#define FLASH_SENSOR_FIRMWARE_BEGIN   0x08140000     //sensor固件起始地址


void etc_make_updata_packet()
{
	
	uint8_t i,flag=0;
	uint8_t sensor_not_good = 0;
	static uint8_t firm_end_sendtimes = 0;
	
	if(etc_updata_s_manage.update_s_enable != START_UPDATA)
		return;
	
	for(i=0;i<7;i++)
	{
		if(etc_sensor_list[i].updata_status == UPDATA_IN)
		{
			flag = 1;
			if(etc_sensor_list[i].sensor_addr != etc_updata_s_manage.now_update_addr+0x7100) //有一个没有满足ack
			{
					sensor_not_good = 1;	
					if(etc_updata_s_manage.now_send_times > (16*10)) //超时还灭有满足的
					{
						etc_sensor_list[i].updata_status = UPDATA_NONE;
					}
			}									
		}
	}	
	if(flag==0)
	{
		etc_updata_s_manage.update_s_enable = UPDATA_NONE;
		etc_updata_s_manage.now_update_addr = 0;
		etc_updata_s_manage.now_send_times = 0;
		return;
	}
	if(sensor_not_good == 0)
	{
		etc_updata_s_manage.now_update_addr += 4;
		etc_updata_s_manage.now_send_times = 0;	
	}
	
	
	etc_ack.etc_updata.address = *((uint32_t *)(FLASH_SENSOR_FIRMWARE_BEGIN + (etc_updata_s_manage.now_update_addr/32)*37)) + (etc_updata_s_manage.now_update_addr%32);
	memcpy(etc_ack.etc_updata.data,(uint8_t *)(FLASH_SENSOR_FIRMWARE_BEGIN + (etc_updata_s_manage.now_update_addr/32)*37 + 5+etc_updata_s_manage.now_update_addr%32),4);
	etc_ack.etc_head.type = MASTER_UPDATA;

	etc_ack.ack_bit |= 0x80;
	etc_ack.etc_updata.crc = crc16(0, etc_ack.etc_updata.data, 4);
	etc_updata_s_manage.now_send_times++;
	
	if(etc_ack.etc_updata.address == 0xffff)
	{
		if(etc_updata_s_manage.now_send_times > 16)
		{
			etc_updata_s_manage.update_s_enable = UPDATA_NONE;
			etc_updata_s_manage.now_update_addr = 0;
			etc_updata_s_manage.now_send_times = 0;			
		}
	}
	
	
}

//超时或者所有sensor进入升级模式后 启动升级
void etc_check_updata_sensor()
{
		uint8_t i ,flag=0;
	
		if(etc_updata_s_manage.update_s_enable != ENABLE_UPDATA)
			return;
		
		//升级指令已经超时   开始升级已经进入升级模式的sensor
		if(slot_count > etc_updata_s_manage.updata_send_cmd_timeout)
		{
			for(i=0;i<7;i++) //没有进入升级模式得sensor 取消升级
			{
				if(etc_sensor_list[i].updata_status != UPDATA_IN)
				{
					etc_sensor_list[i].updata_status = UPDATA_NONE;
				}
			}
			
			for(i=0;i<7;i++)  //只要有一个进入升级模式  启动升级
			{
				if(etc_sensor_list[i].updata_status == UPDATA_IN)
				{//启动升级
					etc_updata_s_manage.update_s_enable = START_UPDATA;
					return;
				}
			}			
			//一个都没有进入升级模式  取消升级任务
			etc_updata_s_manage.update_s_enable = DISABLE;
									
		}
		
		for(i=0;i<7;i++) //全部需要升级的sensor 都已经进入升级模式
		{
			if(etc_sensor_list[i].updata_status == UPDATA_ENABLE)
			{
				return;
			}
			if(etc_sensor_list[i].updata_status == UPDATA_IN)
			{
				flag++;
			}	
		}	
		if(flag) //启动升级
			etc_updata_s_manage.update_s_enable = START_UPDATA;
}



//结算定时数据并发送
void make_timer_data_and_status_and_insert_queue()
{
	uint32_t number = etc_4g_timer_data_send_data.struct_etc_sys_status.number;
	uint8_t i;
	int32_t tm_ms;
	
	for(i=0;i<7;i++)
	{
		if(etc_sensor_list[i].now_on_off_status == ON) //剩余时间当做ON时间
		{
			tm_ms = etc_sys_param.etc_ap_param.data_status_save_timer*1000 - 
			(etc_sensor_list[i].on_time_ms_timer_use+etc_sensor_list[i].off_time_ms_timer_use);
			
			etc_4g_timer_data.struct_etc_data_and_status[i].on_times = (tm_ms + etc_sensor_list[i].on_time_ms_timer_use)*100/etc_sys_param.etc_ap_param.data_status_save_timer*1000;
			if(etc_4g_timer_data.struct_etc_data_and_status[i].on_times > 100)
				etc_4g_timer_data.struct_etc_data_and_status[i].on_times = 100;
		}
		else if(etc_sensor_list[i].now_on_off_status == OFF)
		{
			etc_4g_timer_data.struct_etc_data_and_status[i].on_times = etc_sensor_list[i].on_time_ms_timer_use*100/etc_sys_param.etc_ap_param.data_status_save_timer*1000;	
			if(etc_4g_timer_data.struct_etc_data_and_status[i].on_times > 100)
				etc_4g_timer_data.struct_etc_data_and_status[i].on_times = 100;			
		}
		if(etc_4g_timer_data.struct_etc_data_and_status[i].packet_count)
			etc_4g_timer_data.struct_etc_data_and_status[i].rssi = etc_sensor_list[i].timer_rssi/etc_4g_timer_data.struct_etc_data_and_status[i].packet_count;
		etc_sensor_list[i].timer_rssi = 0;
		etc_4g_timer_data.struct_etc_data_and_status[i].sensor_id = etc_sensor_list[i].sensor_id;
		etc_sensor_list[i].off_time_ms_timer_use = 0;
		etc_sensor_list[i].on_time_ms_timer_use = 0;
	}
	

	etc_4g_timer_data_send_data = etc_4g_timer_data;
	memset(&etc_4g_timer_data.struct_etc_data_and_status[0],0,sizeof(etc_4g_timer_data.struct_etc_data_and_status));
	etc_4g_timer_data_send_data.struct_etc_sys_status.data_time = get_sec_from_rtc();
	etc_4g_timer_data_send_data.struct_etc_sys_status.number = number + 1;
}



//制作实时数据并写入缓冲队列
void make_realtime_data_and_insert_queue(uint8_t index,uint32_t ontime)
{
	
	if(index >7)
		return;
	etc_sensor_car_data_one.sensor_id = etc_sensor_list[index].sensor_id;
	if(etc_sys_param.etc_ap_param.realtime_data_switch > 0)
		etc_sensor_car_data_one.number = real_data_number++;
	etc_sensor_car_data_one.position = index;
	etc_sensor_car_data_one.data_time = get_sec_from_rtc();
	etc_sensor_car_data_one.sys_time = 0;
	etc_sensor_car_data_one.on_time = ontime;
	if(etc_sys_param.etc_ap_param.realtime_data_switch > 0)
		etc_realdata_write_queue(&etc_sensor_car_data_one);
	
}

//制作ON状态数据并写入缓冲队列
void make_on_data_and_insert_queue(uint8_t index,uint16_t event,uint8_t seq,uint8_t rt)
{
	
	if(index >7)
		return;
	etc_sensor_on_event.sensor_id = etc_sensor_list[index].sensor_id;
	if((etc_sys_param.etc_ap_param.realtime_data_switch & 0xf0) > 0)
		etc_sensor_on_event.number = real_data_number++;
	etc_sensor_on_event.position = index | 0x80;
	etc_sensor_on_event.data_time = get_sec_from_rtc();
	etc_sensor_on_event.sys_time = 0;
	etc_sensor_on_event.event = event;
	etc_sensor_on_event.event_packet_seq = seq;
	etc_sensor_on_event.resend_times = rt;
	if((etc_sys_param.etc_ap_param.realtime_data_switch & 0xf0) > 0)
		etc_realdata_write_queue((struct_etc_sensor_car_data*)&etc_sensor_on_event);
	
}


void calc_result_is_on()
{
	
	
}


void calc_result_is_off()
{
	
	
}



//监控SX1280 收发失败计数 重启RF模块
void etc_momitor_rf()
{
	uint32_t i;
	for(i=0;i<2;i++)
	{
		if(/*radio_status[i].rf_send_error_times > 100 | 
			radio_status[i].rf_send_timeout_times > 100 |*/
			radio_status[i].rf_rev_nothing_time>30 && radio_status[i].rf_rev_nothing_time%30 == 0
		)
		{
			radio_status[i].rf_rev_nothing_time++;	
			radio_status[i].reboot_status = WAIT_REBOOT;
			radio_status[i].reboot_times++;
			if(radio_status[i].rf_rev_nothing_time > 1800) //复位系统
				timer_reboot_system = etc_sec  + 5;
		}
			
	}
}



void etc_led_out(uint8_t io,uint8_t on_off)
{
	if(etc_sensor_list[io].sensor_id!=0)
	{
		if(on_off == ON)
			Channel_Led_Op(io,RED);
		else if(on_off == OFF)
			Channel_Led_Op(io,GREEN);				
	}
	IO_Op(io,on_off);
}


void etc_led_flash()
{
	uint8_t i;
	if(led_sec)
	{

		led_sec = 0;
		for(i=0; i<IO_COUNT; i++)
		{
			etc_sensor_list[i].no_data_times++;
			if(etc_sensor_list[i].sensor_id!=0 &&  //读到USBkey  RF 120秒没有数据
				etc_sensor_list[i].no_data_times>120)
			{
				if(etc_sec%2)
					Channel_Led_Op(i,RED);	              //红灯闪烁
				else
					Channel_Led_Op(i,LEDOFF);	
			}
			if(etc_sensor_list[i].sensor_id==0)
			{
				Channel_Led_Op(i,LEDOFF);
			}
			else if(etc_sensor_list[i].sensor_id!=0 &&  //读到USBkey  RF 120秒没有数据
				etc_sensor_list[i].no_data_times<120)
			{
				if(etc_sensor_list[i].now_on_off_status == ON)
					Channel_Led_Op(i,RED);
				else
					Channel_Led_Op(i,LEDOFF);
			}
		}
	}
}

void scan_button()
{
	uint8_t i;
	if(HAL_GPIO_ReadPin(SENSOR_CAL_KEY_GPIO_Port,SENSOR_CAL_KEY_Pin) == GPIO_PIN_RESET)
	{	
		button_status++;
		if(button_status == 20)
		{
			for(i=0; i<IO_COUNT; i++)
			{
				if(etc_sensor_list[i].sensor_id)
				{
				etc_sensor_list[i].adjust_status = ADJUST_ENABLE;
				test_led_switch(i,1);
				}
			}
			copy_string_to_double_buff("校准所有检测器\r\n");
		}
	}
	else
		button_status = 0;
}


uint8_t Is_Leap_Year(uint16_t year)
{  
	if(year%4==0)
	{  
		return 1;
	}
	else return 0;
}  


const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};     


/**************************************************
Func:
		读取RTC时间 并返回对1970年的秒数


***************************************************/
uint32_t get_sec_from_rtc()
{
	uint16_t t;
	uint32_t seccount=0;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	
	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);	
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);
	
	
	year = rtc_date.Year + 2000;
	
	if(year<1970||year>2099)              
		return 1;   
	
	for(t=1970;t<year;t++)                                     
	{
		if(Is_Leap_Year(t))
			seccount+=31622400;                     
		else 
			seccount+=31536000;                                  
	}
	month = rtc_date.Month-1; 
	day = rtc_date.Date-1;
	for(t=0;t<month;t++)                                               
	{
		seccount+=(uint32_t)mon_table[t]*86400;                                
		if(Is_Leap_Year(year)&&t==1)
			seccount+=86400;                     
	}
	seccount+=(uint32_t)(day)*86400;                           
	seccount+=(uint32_t)rtc_time.Hours*3600;                                        
	seccount+=(uint32_t)rtc_time.Minutes*60;                                       
	seccount+=rtc_time.Seconds;                                              
	seccount-=8*3600; 	
	
	return seccount;    
}





extern uint32_t ap_id;
void print_sys_status()
{
	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);	
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);
	
	sprintf(debug_send_buff,"\r\n\r\n\r\n*******************************************meirtdataplus etc_sys *************************************************\r\n\
RTC=[%d %d %d %d:%d:%d] id=%04X V=%d:%d S_V=%02X sys_life=%d时%d分%d秒 RF1->[复位次数=%d 发送错误次数=%d 无接收时间=%d] RF2->[[复位次数=%d 发送错误次数=%d 无接收时间=%d]\r\n\
4g_clinet_1[%d.%d.%d.%d:%d][%d:%d] GPS[%f:%f] %d->%d\r\n",
	rtc_date.Year+2000,rtc_date.Month,rtc_date.Date,rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds,ap_id,etc_sys_param.etc_ap_param.ap_software_version>>4, etc_sys_param.etc_ap_param.ap_software_version&0x0f,
	etc_sys_param.etc_ap_param.sensor_firmware_software_version,etc_sec/3600,etc_sec%3600/60,etc_sec%3600%60,radio_status[RF1].reboot_times,radio_status[RF1].rf_send_error_times,radio_status[RF1].rf_rev_nothing_time,
	radio_status[RF2].reboot_times,radio_status[RF2].rf_send_error_times,radio_status[RF2].rf_rev_nothing_time,etc_sys_param.etc_ap_param.server_ip[0]&0xff,(etc_sys_param.etc_ap_param.server_ip[0]>>8)&0xff,(etc_sys_param.etc_ap_param.server_ip[0]>>16)&0xff,
	etc_sys_param.etc_ap_param.server_ip[0]>>24,etc_sys_param.etc_ap_param.server_port[0],
	gprs_stat.csq,gprs_stat.con_client[0].connect_ok, etc_4g_timer_data.struct_etc_sys_status.gps_n_e[0],etc_4g_timer_data.struct_etc_sys_status.gps_n_e[1],
	slot_count,SysTick->VAL/168);
	copy_string_to_double_buff(debug_send_buff);				
}

void test_timeout_send_result()
{
	if(enable_send_ack_forever == 0)
	{
		test_rev_rf_rate[0].rssi = test_rev_rf_rate[0].rssi/test_rev_rf_rate[0].real_rev_packet_count;
		test_rev_rf_rate[1].rssi = test_rev_rf_rate[1].rssi/test_rev_rf_rate[1].real_rev_packet_count;
		sprintf(debug_send_buff,"test_rf2_rev wish=%d real=%d rssi=%d\r\n",test_rev_rf_rate[0].rev_packet_count-1,test_rev_rf_rate[0].real_rev_packet_count,test_rev_rf_rate[0].rssi);
		copy_string_to_double_buff(debug_send_buff);	
		sprintf(debug_send_buff,"test_rf1_rev wish=%d real=%d rssi=%d\r\n",test_rev_rf_rate[1].rev_packet_count-1,test_rev_rf_rate[1].real_rev_packet_count,test_rev_rf_rate[1].rssi);
		copy_string_to_double_buff(debug_send_buff);	
		enable_send_ack_forever = -1;
	}
}




extern void poll_get_nodata_rssi();



void etc_rf_main_call()
{
	uint16_t crc;
	interrupt_or_main = 1;
	time_out_make_sure_off(); //超时结算过车OFF
	interrupt_or_main = 0;
	etc_check_updata_sensor();	
	etc_momitor_rf();      //
	etc_led_flash();
	//poll_get_nodata_rssi();  读出来都是0xff
	test_timeout_send_result();
	
	if(to_4g_task.timer_data_make_flag == 1)
	{
		to_4g_task.timer_data_make_flag = 0;
		make_timer_data_and_status_and_insert_queue();	
	}
	if(ap_param_write_flash_flag)
	{
		crc = crc16(0,(uint8_t *)&etc_sys_param.etc_ap_param.param_updata_time,sizeof(etc_sys_param)-2);
		etc_sys_param.etc_ap_param.param_crc = (crc>>8 | crc<<8);
		sprintf(debug_send_buff,"开始保存参数:%d->%d\r\n",slot_count,SysTick->VAL/168);
		copy_string_to_double_buff(debug_send_buff);
		write_ap_param_flash();
		sprintf(debug_send_buff,"结束保存参数:%d->%d\r\n",slot_count,SysTick->VAL/168);
		copy_string_to_double_buff(debug_send_buff);		
	}	
}






