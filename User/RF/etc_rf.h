#ifndef ETC_RF_H_
#define ETC_RF_H_


#include "stm32f4xx_hal.h"
#include "etc_def.h"
#include "rtc.h"



#pragma anon_unions
#pragma pack(1)



#define IO_COUNT 7
#define CH_COUNT 32


//Э��ͷ
typedef struct _etc_head
{
	uint16_t crc;
	uint16_t dev_id;
	struct 
	{
		uint8_t type:4;
#define MASTER_ACK_PACKET 				1
#define MASTER_UPDATA 						2
#define SENSOR_EVENT_PACKET 			9
#define SENSOR_STAT_PACKET 				10
#define SENSOR_UPDATA_ACK_PACKET  11
#define SENSOR_NET_IN_PACKET      12	
#define SENSOR_D_MODE_DATA        13				
		uint8_t cmd:3;
#define NO_CMD 						0
#define SET_PARAM_CMD 		1
#define ADJUST_CMD 				2
#define UPDATA_CMD 				3
#define SET_ELSE_PARAM 		4	
		uint8_t rf_index:1;		
#define RF1_P             0
#define RF2_p             1
	};
	uint8_t seq;
}struct_etc_head;



typedef struct _etc_jump_channel
	{
		uint32_t ch0;
		uint8_t ch1;
	}union_etc_jump_channel;




//ͨ��ack������
typedef struct _etc_comm
{
	uint16_t ch_crc;
	union_etc_jump_channel ch;
	uint8_t no_use;
}struct_etc_comm;

//����ack������
typedef struct _etc_set_param
{
	uint16_t sensor_id;
	union_etc_jump_channel ch;
	uint8_t slot;
}struct_etc_set_param;

//������������������
typedef struct _etc_set_else_param
{
	uint16_t sensor_id;
	uint8_t data[6];
}struct_etc_set_else_param;

//��������У׼ack������
typedef struct _etc_set_jiaozhun_or_updata
{
	uint16_t sensor_id[4];
}struct_etc_set_jiaozhun_or_updata;

//����������
typedef struct _etc_updata
{
	uint16_t address;
	uint8_t data[4];
	uint16_t crc;
}struct_etc_updata;


//ack��
typedef struct _etc_ack
{
	struct_etc_head etc_head;
	uint8_t ack_bit;
	union 
	{
		struct_etc_comm etc_comm;
		struct_etc_set_param etc_set_param;
		struct_etc_set_jiaozhun_or_updata etc_set_jiaozhun_or_updata; 
		struct_etc_updata etc_updata;
		struct_etc_set_else_param etc_set_else_param;
	};
//	uint8_t fuck[10];
}struct_etc_ack;

// ????.
typedef union _SNP_EVENT_t
{
	uint16_t		uiAll;
	struct
	{
		uint16_t	bmMs	:	10,		// ???, 'SNP_REF_TIME_US'?32?: 0.9765625ms.
				bmSec	:	5,		// ??.
				blIsOn	:	1;		// ??'ON'/'OFF', 'ON' = 1.
	};
} SNP_EVENT_t;

//event��
typedef struct _etc_event
{
	struct_etc_head etc_head;
	
	SNP_EVENT_t event[4];
	uint8_t resend_times;
}struct_etc_event;

//Dģʽ����
typedef struct _etc_d_mode
{
	struct_etc_head etc_head;
	uint8_t data[9];
}struct_etc_d_mode;


//stat��
typedef struct _etc_stat
{
	struct_etc_head etc_head;
	uint16_t band_id;
	int8_t rssi;
	uint8_t battery;
	uint8_t t;   //�¶�
	uint8_t h;    //ʪ��
	uint8_t h_version;
	uint8_t s_version;
	uint8_t adjust_flag;

}struct_etc_stat;

//sensor_ack��
typedef struct _etc_sensor_ack
{
	struct_etc_head etc_head;
	uint16_t address;
	SNP_EVENT_t event[3];
	uint8_t resend_times;
}struct_etc_sensor_ack;


//sensor_net_in��
typedef struct _etc_sensor_net_in
{
	struct_etc_head etc_head;
	uint16_t band_id;
	uint16_t net_in_times;
	uint16_t net_out_time;
	uint8_t slot;
	uint16_t nouse;
}struct_etc_sensor_net_in;



typedef struct _sensor_list
{
	uint16_t sensor_id;
	uint32_t usb_key_error_times;
	uint8_t sensor_id_is_zero_ch;  //��û��sensorʱ��  ��chipid����ȡͨ��
	uint8_t frame_rev_evnet;  //һ֡�Ѿ��յ�ͬһ��sensor�����ݣ�Ĭ�������Ĳ�����  ��������RF�յ����ظ���
	uint8_t slot;
	int8_t rssi;
	int32_t timer_rssi;
	int32_t all_add_rssi;
	uint8_t battery;
	struct_etc_event now_etc_event_packet;
	uint8_t now_stat_seq;
	uint32_t event_count;
	uint32_t event_resend_count;
	uint32_t event_lost_count;
	uint32_t repeat_packet_times_count[10];	
	
	uint32_t no_data_times;
	uint32_t net_in_times;
	uint32_t net_out_time;
	
	struct{
volatile		int8_t updata_status;   //����״̬
#define UPDATA_NONE    0
#define UPDATA_ENABLE  1
#define UPDATA_IN      2		
volatile		int8_t adjust_status;  //У׼״̬
#define ADJUST_NONE    0
#define ADJUST_ENABLE  1
volatile		uint32_t adjust_count; //У׼����
		
volatile uint16_t sensor_addr;     //sensor �ϱ��������̼���ַ
	
uint8_t else_param[6];     //sensor �ϱ��������̼���ַ	
	};
	
	struct{
		SNP_EVENT_t now_event;      //��ǰ�¼�
volatile		uint8_t now_on_off_status;  //��ǰON OFF ״̬
		#define NONE          0
		#define ON            1
		#define OFF           2
		uint32_t on_off_time_tick;  //on off ����ʱ���tick��
		
		SNP_EVENT_t no_sure_off_event;   //��ȷ����off�¼�
volatile		uint32_t no_sure_off_time_tick;
		
	};
	
	struct
	{
		uint32_t on_time_ms;   //ѹռʱ��
		uint32_t off_time_ms;		//����ʱ��
		uint32_t on_time_ms_timer_use;   //ѹռʱ�� ����ͳ�ƶ�ʱ����
		uint32_t off_time_ms_timer_use;		//����ʱ��	 ����ͳ�ƶ�ʱ����	
		uint32_t car_count;    //��������
		
	};
}struct_sensor_list;



typedef struct _etc_update_s_manage
{
	uint8_t update_s_enable;
	#define DISABLE       0
	#define ENABLE_UPDATA 1
	#define START_UPDATA  2
	uint32_t updata_send_cmd_timeout;   //��������ģʽ��ʱʱ��  ��ʱ��ֱ����������
	uint16_t now_update_addr;    //�����������ڼ���
	uint16_t now_send_times;            //һ�����������͵Ĵ���

}struct_etc_update_s_manage;


#pragma pack()


extern void etc_check_updata_sensor(void);

extern void  time_out_make_sure_off();

extern void etc_rf_main_call(void);


extern volatile uint32_t slot_count;

extern struct_etc_4g_timer_data etc_4g_timer_data;

extern struct_etc_sys_param etc_sys_param;

extern void etc_adjust_set_id(uint16_t sensor_id);

extern void etc_updata_set_id(uint16_t sensor_id);
extern void etc_set_else_param(uint16_t sensor_id,uint8_t data);
extern void init_sensor_list();
extern void etc_led_flash(void);

extern uint32_t get_sec_from_rtc(void);
extern void print_sys_status();
extern RTC_TimeTypeDef rtc_time;
extern RTC_DateTypeDef rtc_date;
int8_t etc_realdata_queue_has_data();
extern struct_sensor_list etc_sensor_list[IO_COUNT];




#endif







