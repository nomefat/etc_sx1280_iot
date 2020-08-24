#ifndef GPRS_COMM_H_
#define GPRS_COMM_H_

#include "stm32f4xx_hal.h"


#define CLIENT_NUM 2

typedef struct _struct_gprs_stat{
	signed char ati_ok;
	signed char creg_ok;       
	signed char cgreg_ok;  	
	signed char cereg_ok;          //4G����ע���־λ
	signed char set_timeout;         //���ó�ʱʱ���־λ
	signed char netopen;             //�������־λ
	signed char now_who_rx;         //���һ���������ĸ������յ���
	unsigned short now_rx_len;      //���һ�����ݵĳ���
	signed char now_read_who_txrx;    //�����ڶ�ȡ�ĸ����ӵ��շ����ݳ���
	volatile signed char now_who_working;  //ָʾ�����Ǹ��������ڷ�������
	volatile signed char send_steps;       //ָʾ�������ݵ�״̬   
	#define GPRS_SEND_WAIT_S 1           //�Ѿ����� cipsend����ȴ�>
	#define GPRS_SEND_WAIT_SENDOK 2      //�Ѿ���������  �ȴ�ģ�鷵��send ok  +cipsend:success,1,20,4
	#define GPRS_SEND_OK            3     //�������
 	char csq;              //�ź�����
	char reboot_flag;
#define GPRS_REBOOT_SETP0 0               //ִ�йر�GSMģ���Դָ��
#define GPRS_REBOOT_SETP1 1               //ִ�д�GSMģ���Դָ��
#define GPRS_REBOOT_SETP2 4               //GSM�Ѿ��ϵ� ��ִ�в���	
	struct {
		char connect_ok;
		uint8_t connect_fail_times;	//����ʧ�ܵĴ���
		uint8_t send_cmd_timeout; //������������ĳ�ʱ
		uint8_t send_cmd_times;    //������������Ĵ���
		uint32_t send_no_timeout;            //���ͺ�ֱ��û���յ� > ��ʱ
		uint8_t send_no_times;
		int gprs_send_error_timeout;   // tcp��������û�гɹ�����	
		uint32_t tx_len;              //�����˶����ֽ�
		uint32_t rx_len;  		        //�����˶����ֽ�
		uint16_t send_len;
		uint32_t last_data_send_time;
		#define _4G_BUFF_LEN 1400
		uint8_t buf[_4G_BUFF_LEN];
	}con_client[CLIENT_NUM];

}struct_gprs_stat;


typedef struct _gprs_one_cmd{
	uint8_t index;
	uint8_t data[256]; //���ڴӶ�����ȡ��������һ������
}struct_gprs_one_cmd;


void dtu_cmd_param(char *pstr , unsigned short length);
void gprs_get_cmd_param(char *pstr , unsigned short length);
void gprs_main_call(void);



extern uint8_t * get_4g_data_buff(int8_t client,uint16_t len);
extern uint8_t * get_4g_data_buff_max(int8_t client,uint16_t *len);
extern int32_t add_4g_data_len(int8_t client,int16_t len);




#endif

