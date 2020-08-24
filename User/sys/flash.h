#ifndef FLASH_H_
#define FLASH_H_


#include "stm32f4xx_hal.h"



#define AP_VERSION 0x19
//V:7  ����USB KEY   XORУ��  �洢5��   ��������һ�μ�OK
//v:10 ȥ��USB KEY���� ���Ӵ�������sensor id  ��Զ������sensor id �Ĺ���
//v:0x11  ����ָ��ʹ��rf1 rf2����ack��    ������һ��rf��ack��  ��ӡ���
//V:0X12 ����ÿ���賿gps��ʱ�ᵼ��������һ�죬�޸�Ϊÿ�����5��Ż��ʱ
//v:0x13 ÿ���賿8��֮ǰ���ڻ���һ�죬������gps��ʱСʱ=8 ���24��ʱ��û���ֶ���������
//...... �޸���Ƶ��������������IO6ʵ��û��sesor id  ���������id�����Ȳ���sensor id����ͨ���·���sensor ��ap��Ƶû���� ����ͨ����ƥ���ղ�������
//V:0X14 �޸�gps��ʱ��bug ����gps��ȡ��ʱ��һֱ��19:34:36 ���¶�ʱ����  ����Ϊ�Ա�2��gpsʱ��  ��ͬ��ʱ����Ϊ��Ч���� ����gpsʱ����ת��Ϊ���ټ�8Сʱ�������ٶ�ʱ
//V:0X15 ������ rev_1000 0/1 syn_forever 0/1 send_wave 0/1 ������  ����1000��  ��������  ���ز�����
//v:0x16 ȥ��4Gģ���ѯ CREG������   ����0 3 ע�ᱻ�ܾ�  ʵ���ϲ���ѯ�� Ҳ���������� 

//v:0x17 ���Ӵ���4����io������������ӡ��4g�ش�
//V:0X18 IO ON OFF AND SHUNXU
//V:0X19 add test rf   timeout send test result








#define FLASH_AP_PARAM_BEGIN_1          0x08100000                     //AP������ʼ��ַ1
#define FLASH_AP_PARAM_BEGIN_2          0x08104000                     //AP������ʼ��ַ2


#define FLASH_RP_FIRMWARE_BEGIN       0x08120000         //RP�̼���ʼ��ַ
#define FLASH_SENSOR_FIRMWARE_BEGIN   0x08140000     //sensor�̼���ʼ��ַ
#define FLASH_AP_FIRMWARE_BEGIN       0x08160008     //ap�̼���ʼ��ַ
#define FLASH_AP_FIRMWARE_HEAD       0x08160000     //ap�̼���ʼ��ַ


typedef struct _flash_head_crc{

	unsigned int head;
	unsigned int crc;

}struct_flash_head_crc;





void read_ap_param_flash(void);
void write_ap_param_flash(void);
int32_t write_bin_flash(uint32_t address,uint8_t *pdata,uint32_t size);







#endif


