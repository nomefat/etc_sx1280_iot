#ifndef GPRS_4G_APP_H_
#define GPRS_4G_APP_H_













#define GPRS_DATA_BUFF_LENGH (1024 + 800)      //���ڴӶ�����ȡ��������һ������������� (����ʵ�ʲ��� ���ܺ��м�������)


#include "stm32f4xx_hal.h"
#include "gprs_comm.h"


#pragma pack(1)


struct _gps_date
{
	uint16_t year;
	uint8_t month;
	uint8_t date;
	uint8_t hour;
	uint8_t minter;
	uint8_t sec;
};

typedef struct _gprs_4g_packet
{
		uint32_t head;
		uint32_t ap_id;
		uint16_t len;
		uint8_t data[1];
}struct_gprs_4g_packet_head;

typedef struct _gprs_4g_heart_packet
{
		uint32_t head;
		uint32_t ap_id;
		uint16_t len;
		uint16_t cmd;                         //������
		float gps_n_e[2];                  //gps
		struct _gps_date rtc_date_time;                    //rtcʱ��
		uint32_t ap_live_time;                //AP���ʱ��
		uint32_t client0_send_bytes;
	  uint32_t client1_send_bytes;
		int8_t gprs_rssi;                    //4G�ź�ǿ��
		struct
		{
			uint32_t id1;
			uint32_t id2;
			uint32_t id3;
		}chip_id;
		uint16_t battery_mv;
		uint16_t crc;
}struct_gprs_4g_heart_packet;

typedef struct _gprs_4g_firmware_data
{
	uint32_t firmware_len;         //�̼��ܳ���
	uint32_t firmware_now_len;     //�̼���ǰ���ĳ���
	uint8_t data[1];
}struct_gprs_4g_firmware_data;



#pragma pack()




typedef struct {
	uint32_t index;
	uint8_t buff[GPRS_DATA_BUFF_LENGH];   //���ڴӶ�����ȡ��������һ������������� 
}struct_client_data_buff;



#define  CMD1_4G_SYS_PARAM_TABLE             0XB0     //4g����������1  ϵͳ������
#define  CMD2_4G_SYS_PARAM_TABLE_WRITE       0X01     //4g����������2  дϵͳ������
#define  CMD2_4G_SYS_PARAM_TABLE_READ        0X02     //4g����������2  ��ϵͳ������
#define  CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK   0x03     //4g����������2  д��ACK
#define  SYS_PARAM_TABLE_WRITE_ACK_F1        -1
#define  SYS_PARAM_TABLE_WRITE_ACK_F2        -2
#define  SYS_PARAM_TABLE_WRITE_ACK_F0        0

#define CMD1_4G_AUTO_CFG                      0XB1      //4G������1 �Զ��������
#define CMD2_4G_START_AUTO_CFG                0X01      //4G������2 �����Զ�����  (����������)
#define CMD2_4G_START_ACK_BEGIN               0X02      //4G������2 �����Զ����� (AP����)
#define CMD2_4G_AUTO_CFG_END                  0X03      //4G������2 �������
#define CMD2_4G_AUTO_CFG_TIMEOUT              0X04      //4G������2 ���ó�ʱδ���
#define CMD2_4G_AUTO_CFG_SERVER_ACK           0XF0      //4G������2 �������յ�����֮�󷢸�ACK


#define CMD1_4G_DOWNLOAD_FIRMWARE             0XB2      //4G������1 �������·��̼����
#define CMD2_4G_AP_FIRMWARE                   0X01      //4G������2 AP�̼�  (����������)
#define CMD2_4G_RP_FIRMWARE                   0X02      //4G������2 RP�̼�   (����������)
#define CMD2_4G_S_FIRMWARE                    0X03      //4G������2 Sensor�̼� (����������)
#define CMD2_4G_AP_FIRMWARE_ACK               0X11      //4G������2 ap�̼�ack (����������)
#define CMD2_4G_RP_FIRMWARE_ACK               0X12      //4G������2 ap�̼�ack (����������)
#define CMD2_4G_S_FIRMWARE_ACK                0X13      //4G������2 ap�̼�ack (����������)


#define CMD1_4G_UPDATA_SENSOR_RP              0XB3      //4G������1 Sensor RP �̼��������
#define CMD2_4G_UPDATA_SENSOR_RP_STAT         0X01      //4G������2 ����S RP����״̬




#define GPRS_4G_RESEND_TIME  5000  //����

typedef struct _gprs_4g_task
{
	int8_t sys_param_table_task;
	int8_t sys_param_table_task_resend_times;
	int8_t sys_param_table_task_resend_timeout; //5s �ط����
	int8_t sys_param_table_task_ack_value; 
	
	uint16_t timer_data_task;  //��ʱ��������
	uint16_t timer_stat_task;  //��ʱ״̬����
	uint16_t timer_stat_qianfang_task; //��ʱǧ��״̬����
	
	uint32_t timer_data_task_timeslot;  //��ʱ����������ʱ���
	uint32_t timer_stat_task_timeslot;  //��ʱ״̬������ʱ���
	uint32_t timer_stat_qianfang_task_timeslot; //��ʱǧ��״̬����	����ʱ���
	
}struct_gprs_4g_task;








typedef struct _auto_cfg
{
	uint8_t auto_cfg_switch;   //�Զ����ÿ���
	uint32_t auto_cfg_time;    //�Զ����ü�ʱ
	uint32_t send_to_4g_time;  //��¼���͵�ʱ��� ������ʱ�ط�
	uint16_t re_send_times;    //�ط�����
	#define AUTO_CFG_TIMEOUT    600  //  ��ʱʱ��  ��
	uint8_t to_4g_data[156];
	
}struct_auto_cfg;


#pragma pack(1)

typedef struct _FW_HEADER_t_5xx
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[64u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
	uint16_t d[8];
} FW_HEADER_t_5xx;

typedef struct _FW_HEADER_t_5xxRP
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[16u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
} FW_HEADER_t_5xxRP;


typedef	struct _s{
		uint8_t sensor_num;
		uint16_t sensor_id[1];
	}struct_s;

#pragma pack()

extern uint32_t timer_reboot_system;
extern FW_HEADER_t_5xx pheader_5xx;
extern struct_gprs_stat gprs_stat;
extern struct_client_data_buff client_data_buff_0;
extern struct_client_data_buff client_data_buff_1;

int32_t server_4g_data_hanle(int32_t whitch_client);
void gprs_4g_task_poll(void);
void load_sys_param_to_use_table(void);

int32_t send_4g_heart_pacekt(int32_t whitch_client);

void poll_4g_task(void);
void app_init(void);

#endif


