#ifndef _ETC_4G_DEF_
#define _ETC_4G_DEF_



#include "stm32f4xx_hal.h"

#pragma anon_unions
#pragma pack(1)

//系统参数
typedef struct _etc_sys_param
{
	struct _etc_ap_param
	{
		uint16_t param_crc;  //参数校验
		uint32_t param_updata_time;  //参数更新时间
		uint8_t ap_software_version;  //版本号
		uint16_t sensor_firmware_software_version;  //存放的sensor固件版本号
		uint8_t sensor_num; //sensor数量

		uint32_t server_ip[2];          //公司服务器ip
		uint16_t server_port[2];        //公司服务器端口		
		uint16_t data_status_save_timer;  //定时状态和数据存储间隔时间 秒
		uint8_t realtime_data_switch; //实时数据开关
		uint8_t nouse[100];
	}etc_ap_param;

	struct _etc_sensor_param
	{
		uint16_t sensor_id;  //sensor id
		uint16_t off_to_on_min_t_ms;
		uint8_t d[6];
	}etc_sensor_param[7];	
	
}struct_etc_sys_param;



//定时数据
typedef struct _etc_4g_timer_data
{
	//系统状态
	struct _etc_sys_status
	{
		uint32_t number;    //流水号 递增
		uint32_t sys_time; //系统时间 ，传送时候填写
		uint32_t data_time;  //数据时间
		float gps_n_e[2];
		uint8_t d[24];
	}struct_etc_sys_status;	
	//sensor状态和数据（定时）
	struct _etc_data_and_status
	{
		uint16_t sensor_id;   
		uint16_t sensor_software_version;  //sensor版本号
		int8_t rssi;        //平均信号强度
		uint16_t packet_count;     //包总数
		uint16_t packet_lost_count;   //丢包数
		uint16_t packet_resend_count;  //重传数
		
		
		uint16_t car_count; //过车统计
		uint8_t on_times;    //占有率
		uint8_t voltage;
		int8_t sensor_rev_rssi;
		uint8_t net_out_times; //掉网次数
		uint16_t net_out_time; //掉网时间 累积
		uint8_t d1[3];
		
	}struct_etc_data_and_status[7];	
}struct_etc_4g_timer_data;



//sensor 升级状态包上报
typedef struct _etc_sensor_updata_status
{
	uint32_t sys_time; //系统时间 ，传送时候填写
	struct{
	uint16_t sensor_id;
	int8_t updata_percent; 
	}sensor_updata_status[7];
}struct_etc_sensor_updata_status;


//sensor 实时过车数据
typedef struct _etc_sensor_car_data
{
	uint16_t sensor_id;
	uint32_t number; //流水号
	uint32_t sys_time; //系统时间 ，传送时候填写
	uint32_t data_time;  //数据时间	
	uint8_t position;  //位置 1-7 IO
	uint32_t on_time;    //压站时间
}struct_etc_sensor_car_data;


//sensor 车辆ON状态上报
typedef struct _etc_sensor_on_event
{
	uint16_t sensor_id;
	uint32_t number; //流水号
	uint32_t sys_time; //系统时间 ，传送时候填写
	uint32_t data_time;  //数据时间	
	uint8_t position;  //位置 1-7 IO
	uint8_t event_packet_seq;
	uint8_t resend_times;
	uint16_t event;
}struct_etc_sensor_on_event;



//IO on off
typedef struct _etc_io_on_off
{	
	uint8_t index;
	uint32_t number; //流水号

	struct 
	{
		uint8_t io;  //0-11
		uint32_t sys_time; //系统时间 ，传送时候填写
		uint32_t second; //系统时间 ，传送时候填写
		struct 
		{
			uint16_t ms:15;
			uint16_t on_off:1;
		};
	}io_data[100];
}struct__etc_io_on_off;



typedef struct _etc_4g_server_protocol
{
	uint32_t head;
	uint32_t ap_id;
	uint32_t ap_ip;
	uint16_t len;
	uint8_t packet_seq;
	uint8_t data[1];
}struct_etc_4g_server_protocol;



typedef struct _to_4g_task
{
	uint32_t realtime_data_flag;  //实时数据发送标志
	uint32_t timer_data_flag;    //定时数据发送标志
	uint8_t timer_data_make_flag;
	uint8_t param_flag;      //参数发送标志
	uint8_t updata_flag;
	uint8_t io_data_flag;
}struct_to_4g_task;


typedef struct _test_rev_rf_rate
{
	uint16_t rev_packet_count;
	uint16_t real_rev_packet_count;
	uint16_t lost_packet_count;
	uint8_t seq;
	int32_t rssi;
}struct_test_rev_rf_rate;


#pragma pack()



extern struct_etc_sys_param etc_sys_param;






#endif



