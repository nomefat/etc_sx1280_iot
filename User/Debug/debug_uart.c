#include "main.h"
#include "debug_uart.h"
#include "string.h"
#include "math.h"
#include "debug.h"
#include "stdio.h"
#include "etc_def.h"
#include "etc_rf.h"
#include "gprs_comm.h"
#include "flash.h"




extern UART_HandleTypeDef huart3;
extern int8_t nodata_rssi[2];

extern void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel );
extern void print_sys_status();


#define Q_LEN 512           //���г���
char debug_uart_dma_buff[Q_LEN];       //��������

char debug_uart_buff[Q_LEN+1];

char debug_send_buff[Q_LEN];


struct_debug_double_buff debug_send_buff1;
struct_debug_double_buff debug_send_buff2;



extern int32_t enable_send_ack_forever;



struct _cmd_param_int{
	int param_num;
	int param[10];
}cmd_param_int;


struct _cmd_list{
	char *cmd;
	void (*func)(char *param);
};

#define CMD_CALLBACK_LIST_BEGIN struct _cmd_list cmd_list[] = {NULL,NULL,
#define CMD_CALLBACK_LIST_END NULL,NULL};
#define CMD_CALLBACK(cmd_string,callback)	cmd_string,callback,



extern struct_etc_sys_param etc_sys_param; //ϵͳ����
extern uint8_t ap_param_write_flash_flag;

extern uint8_t rf_scan_channel_enable;

extern uint8_t rf_send_1000_p_enable; 

extern unsigned char rf_send_data[];

extern int enable_print_sensor_event;   //ʹ��rf ������¼���ӡ����
	
extern int gprs_print_rx_tx_data_enable;   //ʹ��gprs�շ����ݴ�ӡ����

extern struct_sensor_list etc_sensor_list[IO_COUNT];




int copy_string_to_double_buff(char *pstr)
{
	
	int len = strlen(pstr);
	
	if(debug_send_buff1.len != 0xffffffff)
	{
		if(len<(DEBUG_DOUBLE_BUFF_LEN-debug_send_buff1.len))
		{			
			memcpy(debug_send_buff1.data+debug_send_buff1.len,pstr,len);
			debug_send_buff1.len += len;
			
			return 0;
		}	
	}
	
	if(debug_send_buff2.len != 0xffffffff)
	{
		if(len<(DEBUG_DOUBLE_BUFF_LEN-debug_send_buff2.len))
		{			
			memcpy(debug_send_buff2.data+debug_send_buff2.len,pstr,len);
			debug_send_buff2.len += len;
			return 0;
		}	
	}
	return -1;
}

/*****************

��main�������� ˫���������ݾ�����DMA����


******************/
void debug_send_double_buff_poll(void)
{
	int len;
	
	if(huart3.gState == HAL_UART_STATE_READY)
	{
		if(debug_send_buff1.len == 0xffffffff)
		{
			debug_send_buff1.len = 0;
		}
		else if(debug_send_buff2.len == 0xffffffff)
			debug_send_buff2.len = 0;
	}
	else
		return;
	
	
	//��������������Ҫ����  2�����岢û�����ڷ���
	//if(debug_send_buff1.len != 0 && debug_send_buff1.len != 0xffffffff && debug_send_buff2.len != 0xffffffff)
	if(debug_send_buff1.len != 0 && debug_send_buff1.len != 0xffffffff && huart3.gState == HAL_UART_STATE_READY)
	{		
		
		len = debug_send_buff1.len;
		debug_send_buff1.len = 0xffffffff;  //��������������ڷ��ͣ����ܲ���
		HAL_UART_Transmit_DMA(&huart3,(uint8_t *)debug_send_buff1.data,len);		
	}
	//else if(debug_send_buff2.len != 0 && debug_send_buff2.len != 0xffffffff && debug_send_buff1.len != 0xffffffff)
	else if(debug_send_buff2.len != 0 && debug_send_buff2.len != 0xffffffff && huart3.gState == HAL_UART_STATE_READY)
	{		
		len = debug_send_buff2.len;
		debug_send_buff2.len = 0xffffffff;
		HAL_UART_Transmit_DMA(&huart3,(uint8_t *)debug_send_buff2.data,len);		
	}
}



void _copy_string_to_double_buff(char *pstr)
{
	int num = 0;
	char *ptr = pstr;
	while(*ptr++)
	{
		num++;
		if(num>5000)return;
	}
	
	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)pstr,num);
	
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		//HAL_GPIO_WritePin(general_led_5_GPIO_Port,general_led_5_Pin,GPIO_PIN_SET);	
	}
}





/**********************************************************************************************
***func: ���ڵ�Ƭ��һ�ο���DMA����
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_debug_dma_receive()
{
	SET_BIT((&huart3)->Instance->CR1, USART_CR1_IDLEIE);  //�򿪴��ڿ����ж�
	HAL_UART_Receive_DMA(&huart3,debug_uart_dma_buff,Q_LEN);	 //��DMA����
}




/**********************************************************************************************
***func:���ڿ����жϻص�����
***     �����ж������ж�һ�����ݵĽ���
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_debug_idle_callback()
{
	HAL_DMA_Abort((&huart3)->hdmarx);
	huart3.RxState = HAL_UART_STATE_READY;
	huart3.hdmarx->State = HAL_DMA_STATE_READY;
	debug_uart_buff[0] = Q_LEN-DMA1_Stream1->NDTR;
	memcpy(debug_uart_buff+1,(char*)debug_uart_dma_buff,Q_LEN-DMA1_Stream1->NDTR);
	
	HAL_UART_Receive_DMA(&huart3,debug_uart_dma_buff,Q_LEN);	 //��DMA����

}


/*
 * ���ܣ��Ӷ�����ȡ��һ���������ַ������
 * ʧ�ܣ�����-1
 * �ɹ�������0
 *cmd ��������ָ�룬param ��Ų�����ָ�롣
*/
int get_q_string(char *cmd,char *param)
{
	int i = 1;
	int timeout = 0;


	for(;;){
		if(debug_uart_buff[i] == ' ')
		{
			cmd[i-1] = 0; 
			i++;
			break;
		}
		else if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			cmd[i-1] = 0; 
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;
		cmd[i-1] = debug_uart_buff[i];
		i++;
				
	}

	for(;;){

		if(debug_uart_buff[i] == '\r' || debug_uart_buff[i] == '\n')
		{
			debug_uart_buff[0] = 0;
			*param = 0; 
			return 0;
		}
		if(i>debug_uart_buff[0])
			return -1;	
		
		*param++ = debug_uart_buff[i];
		i++;	
	}

	return 0;
}

int str_to_int(char *str)
{
	int i = 0,j = 0;
	int ret = 0;
	for(;;){
	if(str[i++]==0||i>20)
		break;
	}
	j = i = i-2;
	for(;i>=0;i--)
	{
		ret += (str[i]-'0')*(pow(10,(j-i)));
	}
	return ret;
}

//
struct _cmd_param_int* get_str_param(char *param)
{
	char *ptr_now = param;
	char *ptr_pre = param;
	int i = 0;
	
	cmd_param_int.param_num = 0;
	for(;;){                     //�ָ���������տո�

		if(*ptr_now==' ')
		{
			*ptr_now = 0;
			cmd_param_int.param[i++] = str_to_int(ptr_pre);
			ptr_pre = ptr_now+1;
			cmd_param_int.param_num++;
		}
		ptr_now++;		
		if(*ptr_now==0)
		{
			cmd_param_int.param[i] = str_to_int(ptr_pre);
			cmd_param_int.param_num++;
			return &cmd_param_int;
		}
		if(ptr_now-param>100)		
			return &cmd_param_int;
	}
	
}

void print_now_rfmode()
{

}

void help(char *param)
{
	copy_string_to_double_buff("*******************************************************\r\n\
1.?                       ��ӡ��������ĵ�\r\n\
2.gprs                    ��ӡ4G�豸״̬\r\n\
3.setrfch                 ����rfͨ��\r\n\
4.rf                      ��ӡRF�豸״̬\r\n\
5.h                       ��ӡsensor״̬\r\n\
6.clear                   ���sensor��¼״̬\r\n\
7.ap                      ��ӡAP�豸״̬\r\n\
8.syn [0/1]               ����ack��ʹ�ܻ���ʧ��\r\n\
9.etc_j [0-6]             У׼sensor ����������\r\n\
10.etc_u [0-6]            ����sensor ����������\r\n\
11.etc_f [0-6] [d]        ����sensor ����������\r\n\
12.rssi                   ��ӡrssiֵ\r\n\
13.set_s_id [0-6] [id]    ����sensor ����������\r\n\
8.syn_forever [0/1]       ����ack������ʹ�ܻ���ʧ��\r\n\
*******************************************************\r\n");

}

extern void Radio_set_rf_ch(uint8_t rf,uint8_t ch);
extern uint8_t now_ch;
void setrfch(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	
	if(ps_pram->param[0]>4 || ps_pram->param[0]<1)
	{
		
		copy_string_to_double_buff("ѡ��RF����1-4��\r\n");
		return;
	}	
	if(ps_pram->param[1]>31 || ps_pram->param[1]<0)
	{
		
		copy_string_to_double_buff("RFͨ������0-2��\r\n");
		return;
	}	
	if(ps_pram->param[0] == 1)
		Radio_set_rf_ch(ps_pram->param[0]-1,now_ch=ps_pram->param[1]);
	if(ps_pram->param[0] == 2)
		Radio_set_rf_ch(ps_pram->param[0]-1,now_ch=ps_pram->param[1]);
	sprintf(debug_send_buff,"setrfch ok %d %d\r\n",ps_pram->param[0],ps_pram->param[1]);
	copy_string_to_double_buff(debug_send_buff);	
}



extern char* make_gprs_stat();
void get_gprs_stat()
{
//	copy_string_to_double_buff(make_gprs_stat());	
}

char* make_rf_stat();
void get_rf_stat()
{
//	copy_string_to_double_buff(make_rf_stat());		
}





void restart_sensor(char *param)
{
	//re_start_sensor_event_record();
	copy_string_to_double_buff("restart sensor event record \r\n");
}


extern struct_gprs_stat gprs_stat;
extern volatile uint32_t slot_count;
extern char debug_sensor_event_str[8][43];
void get_sensor(char *param)
{
	print_sys_status();
	debug_etc_sensor_event_to_str();
	copy_string_to_double_buff(&debug_sensor_event_str[0][0]);
	sprintf(debug_send_buff,"t=%d->%d\r\n",slot_count,SysTick->VAL/168);
	copy_string_to_double_buff(debug_send_buff);
	
}



void change_server_port(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	if(ps_pram->param[0]  > 0 )
	{
		etc_sys_param.etc_ap_param.server_port[0] = ps_pram->param[0];	
		gprs_stat.con_client[0].connect_ok = 0;
		ap_param_write_flash_flag = 1;
		sprintf(debug_send_buff,"c0_port:%d \r\n",etc_sys_param.etc_ap_param.server_port[0]);
		copy_string_to_double_buff(debug_send_buff);
	}	
}


void print_version()
{

//	if(RF_DEFAULT_MODE == ENABLE_NOMODULATE_CARRIER)
//	{
//		if(rf_scan_channel_enable == 0)
//			copy_string_to_double_buff("033ap:���ز�ģʽ \r\n");
//		else
//			copy_string_to_double_buff("033ap:���ز�ɨƵģʽ \r\n");
//	}
//	else if(RF_DEFAULT_MODE == ENABLE_MODULATE_CARRIER)
//	{
//		if(rf_scan_channel_enable == 0)
//			copy_string_to_double_buff("033ap:���Ʋ�ģʽ \r\n");
//		else
//			copy_string_to_double_buff("033ap:���Ʋ�ɨƵģʽ \r\n");
//	}	
//	else if(RF_DEFAULT_MODE == RF_WORK)
//	{
//		sprintf(debug_send_buff,"033ap:����ģʽ version=%d.%d\r\n",AP_VERSION>>8,AP_VERSION&0xff);
//		copy_string_to_double_buff(debug_send_buff);
//	}		
//	else if(RF_DEFAULT_MODE == RF_IDLE)
//	{
//		copy_string_to_double_buff("033ap:����ģʽģʽ(������1000���շ�����) \r\n");
//	}			
}

extern uint32_t ap_id;
void get_ap_param(char *param)
{
		sprintf(debug_send_buff,"etc_ap:version=%d apid=%x\r\n",AP_VERSION,ap_id);
	copy_string_to_double_buff(debug_send_buff);	
}





int syn_disable_0 = 1;
int syn_disable_1 = 1;
void enable_syn(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	if(ps_pram->param[0]  == 0 )
	{
		if(ps_pram->param[1]  == 0 )
			syn_disable_0 = 0;
		else
			syn_disable_0 = 1;
		sprintf(debug_send_buff,"rf1 ack en:%d \r\n",syn_disable_0);
		copy_string_to_double_buff(debug_send_buff);
	}
	else if(ps_pram->param[0]  == 1 )
	{
		if(ps_pram->param[1]  == 0 )
			syn_disable_1 = 0;
		else
			syn_disable_1 = 1;
		sprintf(debug_send_buff,"rf2 ack en:%d \r\n",syn_disable_1);
		copy_string_to_double_buff(debug_send_buff);
	}		
}




extern void etc_set_else_param_fun(uint32_t index,uint8_t data);

extern void clear_etc();

extern void	etc_adjust_(uint32_t index);

extern void	etc_updata_(uint32_t index);

void etc_adjust(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	uint32_t i;
	
	for(i=0;i<ps_pram->param_num;i++)
	{
		etc_adjust_(ps_pram->param[i]);
	sprintf(debug_send_buff,"adjust:%d\r\n",ps_pram->param[i]);
		copy_string_to_double_buff(debug_send_buff)	;
	}	
;
}


void etc_set_elseparam(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	uint32_t i;
	
	etc_set_else_param_fun(ps_pram->param[0],ps_pram->param[1]);	
	sprintf(debug_send_buff,"���ò���:%d->%d\r\n",ps_pram->param[0],ps_pram->param[1]);
	copy_string_to_double_buff(debug_send_buff);				

}

void etc_updata(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);	
	uint8_t i;
	
	for(i=0;i<ps_pram->param_num;i++)
	{
		etc_updata_(ps_pram->param[i]);
		sprintf(debug_send_buff,"updata:%d\r\n",ps_pram->param[i]);
		copy_string_to_double_buff(debug_send_buff);		
	}

}


void get_rssi(char *param)
{
	sprintf(debug_send_buff,"rssi:%d:%d\r\n",nodata_rssi[0],nodata_rssi[1]);
	copy_string_to_double_buff(debug_send_buff);		
}

void set_sensor_id(char *param)
{
	uint32_t index;
	uint32_t s_id;
	uint32_t j;
	
	sscanf(param,"%d %x",&index,&s_id);
	
	if(index > 6)
	{
		copy_string_to_double_buff("index ����6 �������\r\n");
		return;
	}
	etc_sensor_list[index].sensor_id = s_id;
	etc_sys_param.etc_sensor_param[index].sensor_id = s_id;
	etc_sys_param.etc_ap_param.sensor_num = 0;
	ap_param_write_flash_flag = 1;
	for(j=0;j<7;j++)
	{
		if(etc_sys_param.etc_sensor_param[j].sensor_id!=0)
		{
			etc_sys_param.etc_ap_param.sensor_num++;
		}
	}		
	copy_string_to_double_buff("s_id set ok\r\n");
}

extern struct_test_rev_rf_rate test_rev_rf_rate[2];
void enable_send_ack_forever_fun(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	
	if(ps_pram->param[0] == 0 )
	{
		enable_send_ack_forever = 0;
		copy_string_to_double_buff("disable send ack forever\r\n");
		return;
	}	
	else
	{
		enable_send_ack_forever = ps_pram->param[0];
		memset(test_rev_rf_rate,0,sizeof(test_rev_rf_rate));
		copy_string_to_double_buff("enable send ack forever\r\n");
		return;
	}	
	
	
}

extern uint8_t enable_rev_1000;
void en_rev_1000(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	
	if(ps_pram->param[0] == 0 )
	{
		enable_rev_1000 = 0;
		copy_string_to_double_buff("disable rev 1000 packet\r\n");
		return;
	}	
	else
	{
		enable_rev_1000 = 1;
		copy_string_to_double_buff("enable  rev 1000 packet\r\n");
		return;
	}	
		
}
extern uint8_t rf_wave_en ;
void enable_rf_wave(char *param)
{
	struct _cmd_param_int* ps_pram = get_str_param(param);
	

	
	if(ps_pram->param[0] == 0 )
	{
		rf_wave_en = 0;
		copy_string_to_double_buff("disable ���ز�����\r\n");
		return;
	}	
	else
	{
		rf_wave_en = 1;
		copy_string_to_double_buff("enable  ���ز�����\r\n");
		return;
	}	
			
}

//�ڴ˴������������ַ����ͻص�����
CMD_CALLBACK_LIST_BEGIN


CMD_CALLBACK("?",help)		
CMD_CALLBACK("gprs",get_gprs_stat)		
CMD_CALLBACK("setrfch",setrfch)
CMD_CALLBACK("rf",get_rf_stat)
CMD_CALLBACK("h",get_sensor)
CMD_CALLBACK("clear",clear_etc)
CMD_CALLBACK("ap",get_ap_param)
CMD_CALLBACK("syn",enable_syn)
CMD_CALLBACK("etc_j",etc_adjust)
CMD_CALLBACK("etc_u",etc_updata)
CMD_CALLBACK("etc_f",etc_set_elseparam)
CMD_CALLBACK("rssi",get_rssi)
CMD_CALLBACK("set_s_id",set_sensor_id)
CMD_CALLBACK("syn_forever",enable_send_ack_forever_fun)
CMD_CALLBACK("rev_1000",en_rev_1000)
CMD_CALLBACK("send_wave",enable_rf_wave)
CMD_CALLBACK("ch_port",change_server_port)


CMD_CALLBACK_LIST_END




//uint8_t uart4_buff[7];




char cmd[100];
char param[32];
int get_cmd(void)
{
	int i = 0;
	if(get_q_string(cmd,param) == -1)
		return 0;
	
	for(;;){
		if(strcmp(cmd,cmd_list[i].cmd)==0)
			return i;	
		if(cmd_list[++i].cmd==NULL)
			return 0;
	}
}



void debug_cmd_handle(void)
{
	int func_index = get_cmd();
	if(func_index<=0)
		return;
	cmd_list[func_index].func(param);
}







