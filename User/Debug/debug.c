#include "debug.h"
#include "string.h"
#include "math.h"
#include "etc_rf.h"
#include "stdio.h"

char debug_sensor_event_str[8][129];




extern struct_sensor_list etc_sensor_list[7] ;

void debug_etc_sensor_event_to_str()
{
	int i = 0;
	int j = 0;
	int index = 0;
	uint8_t rssi;
	
	char hex_str[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	
	memcpy(&debug_sensor_event_str[0][0]	,"s_id slot rssi all_cout lost_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10 in_tims out_t car_num on_ms->off_ms\r\n",
	sizeof("s_id slot rssi all_count rev_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10 in_tims out_t car_num on_ms->off_ms\r\n"));
//	index += sizeof("s_id slot rssi all_count rev_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10\r\n");
	
	for(i=0;i<7;i++)
	{
//		if(etc_sensor_list[i].sensor_id == 0)
//			continue;
		
		debug_sensor_event_str[i+1][0] = hex_str[(etc_sensor_list[i].sensor_id>>12)];
		debug_sensor_event_str[i+1][1] = hex_str[(etc_sensor_list[i].sensor_id>>8) & 0x0f];		
		debug_sensor_event_str[i+1][2] = hex_str[(etc_sensor_list[i].sensor_id &0xff)>>4];
		debug_sensor_event_str[i+1][3] = hex_str[(etc_sensor_list[i].sensor_id & 0x000f)];			
		debug_sensor_event_str[i+1][4] = ' ';

		debug_sensor_event_str[i+1][5] = ' ';
		debug_sensor_event_str[i+1][6] = hex_str[etc_sensor_list[i].slot/100];
		debug_sensor_event_str[i+1][7] = hex_str[etc_sensor_list[i].slot%100/10];		
		debug_sensor_event_str[i+1][8] = hex_str[etc_sensor_list[i].slot%10];	
		debug_sensor_event_str[i+1][9] = ' ';	
		
		rssi = abs(etc_sensor_list[i].all_add_rssi)/(etc_sensor_list[i].event_count-1);
		debug_sensor_event_str[i+1][10] = '-';
		debug_sensor_event_str[i+1][11] = hex_str[rssi/100];
		debug_sensor_event_str[i+1][12] = hex_str[rssi%100/10];		
		debug_sensor_event_str[i+1][13] = hex_str[rssi%10];	
		debug_sensor_event_str[i+1][14] = ' ';	

		debug_sensor_event_str[i+1][15] = hex_str[etc_sensor_list[i].event_count/100000000];
		debug_sensor_event_str[i+1][16] = hex_str[etc_sensor_list[i].event_count%100000000/10000000];
		debug_sensor_event_str[i+1][17] = hex_str[etc_sensor_list[i].event_count%10000000/1000000];		
		debug_sensor_event_str[i+1][18] = hex_str[etc_sensor_list[i].event_count%1000000/100000];	
		debug_sensor_event_str[i+1][19] = hex_str[etc_sensor_list[i].event_count%100000/10000];		
		debug_sensor_event_str[i+1][20] = hex_str[etc_sensor_list[i].event_count%10000/1000];
		debug_sensor_event_str[i+1][21] = hex_str[etc_sensor_list[i].event_count%1000/100];		
		debug_sensor_event_str[i+1][22] = hex_str[etc_sensor_list[i].event_count%100/10];	
		debug_sensor_event_str[i+1][23] = hex_str[etc_sensor_list[i].event_count%10];
		debug_sensor_event_str[i+1][24] = ' ';			

		debug_sensor_event_str[i+1][25] = hex_str[etc_sensor_list[i].event_lost_count/100000000];
		debug_sensor_event_str[i+1][26] = hex_str[etc_sensor_list[i].event_lost_count%100000000/10000000];
		debug_sensor_event_str[i+1][27] = hex_str[etc_sensor_list[i].event_lost_count%10000000/1000000];		
		debug_sensor_event_str[i+1][28] = hex_str[etc_sensor_list[i].event_lost_count%1000000/100000];	
		debug_sensor_event_str[i+1][29] = hex_str[etc_sensor_list[i].event_lost_count%100000/10000];		
		debug_sensor_event_str[i+1][30] = hex_str[etc_sensor_list[i].event_lost_count%10000/1000];
		debug_sensor_event_str[i+1][31] = hex_str[etc_sensor_list[i].event_lost_count%1000/100];		
		debug_sensor_event_str[i+1][32] = hex_str[etc_sensor_list[i].event_lost_count%100/10];	
		debug_sensor_event_str[i+1][33] = hex_str[etc_sensor_list[i].event_lost_count%10];
		debug_sensor_event_str[i+1][34] = ' ';	
		
		debug_sensor_event_str[i+1][35] = hex_str[etc_sensor_list[i].event_resend_count%1000000/100000];	
		debug_sensor_event_str[i+1][36] = hex_str[etc_sensor_list[i].event_resend_count%100000/10000];		
		debug_sensor_event_str[i+1][37] = hex_str[etc_sensor_list[i].event_resend_count%10000/1000];
		debug_sensor_event_str[i+1][38] = hex_str[etc_sensor_list[i].event_resend_count%1000/100];		
		debug_sensor_event_str[i+1][39] = hex_str[etc_sensor_list[i].event_resend_count%100/10];	
		debug_sensor_event_str[i+1][40] = hex_str[etc_sensor_list[i].event_resend_count%10];
		
		for(j=0;j<10;j++)
		{
			debug_sensor_event_str[i+1][41+j*5] = ' ';
			debug_sensor_event_str[i+1][42+j*5] = hex_str[etc_sensor_list[i].repeat_packet_times_count[j]%10000/1000];
			debug_sensor_event_str[i+1][43+j*5] = hex_str[etc_sensor_list[i].repeat_packet_times_count[j]%1000/100];		
			debug_sensor_event_str[i+1][44+j*5] = hex_str[etc_sensor_list[i].repeat_packet_times_count[j]%100/10];	
			debug_sensor_event_str[i+1][45+j*5] = hex_str[etc_sensor_list[i].repeat_packet_times_count[j]%10];
		}
		debug_sensor_event_str[i+1][91] = ' ';
		debug_sensor_event_str[i+1][92] = hex_str[etc_sensor_list[i].net_in_times%1000000/100000];	
		debug_sensor_event_str[i+1][93] = hex_str[etc_sensor_list[i].net_in_times%100000/10000];		
		debug_sensor_event_str[i+1][94] = hex_str[etc_sensor_list[i].net_in_times%10000/1000];
		debug_sensor_event_str[i+1][95] = hex_str[etc_sensor_list[i].net_in_times%1000/100];		
		debug_sensor_event_str[i+1][96] = hex_str[etc_sensor_list[i].net_in_times%100/10];	
		debug_sensor_event_str[i+1][97] = hex_str[etc_sensor_list[i].net_in_times%10];		
		debug_sensor_event_str[i+1][98] = ' ';
		
		debug_sensor_event_str[i+1][99] = hex_str[etc_sensor_list[i].net_out_time%1000000/100000];	
		debug_sensor_event_str[i+1][100] = hex_str[etc_sensor_list[i].net_out_time%100000/10000];		
		debug_sensor_event_str[i+1][101] = hex_str[etc_sensor_list[i].net_out_time%10000/1000];
		debug_sensor_event_str[i+1][102] = hex_str[etc_sensor_list[i].net_out_time%1000/100];		
		debug_sensor_event_str[i+1][103] = hex_str[etc_sensor_list[i].net_out_time%100/10];	
		debug_sensor_event_str[i+1][104] = hex_str[etc_sensor_list[i].net_out_time%10];
		debug_sensor_event_str[i+1][105] = ' ';
		
		debug_sensor_event_str[i+1][106] = hex_str[etc_sensor_list[i].car_count%1000000/100000];	
		debug_sensor_event_str[i+1][107] = hex_str[etc_sensor_list[i].car_count%100000/10000];		
		debug_sensor_event_str[i+1][108] = hex_str[etc_sensor_list[i].car_count%10000/1000];
		debug_sensor_event_str[i+1][109] = hex_str[etc_sensor_list[i].car_count%1000/100];		
		debug_sensor_event_str[i+1][110] = hex_str[etc_sensor_list[i].car_count%100/10];	
		debug_sensor_event_str[i+1][111] = hex_str[etc_sensor_list[i].car_count%10];
		debug_sensor_event_str[i+1][112] = ' ';
		
		debug_sensor_event_str[i+1][113] = hex_str[etc_sensor_list[i].on_time_ms/1000%1000000/100000];	
		debug_sensor_event_str[i+1][114] = hex_str[etc_sensor_list[i].on_time_ms/1000%100000/10000];		
		debug_sensor_event_str[i+1][115] = hex_str[etc_sensor_list[i].on_time_ms/1000%10000/1000];
		debug_sensor_event_str[i+1][116] = hex_str[etc_sensor_list[i].on_time_ms/1000%1000/100];		
		debug_sensor_event_str[i+1][117] = hex_str[etc_sensor_list[i].on_time_ms/1000%100/10];	
		debug_sensor_event_str[i+1][118] = hex_str[etc_sensor_list[i].on_time_ms/1000%10];
		debug_sensor_event_str[i+1][119] = '-';	
		debug_sensor_event_str[i+1][120] = '>';			
		debug_sensor_event_str[i+1][121] = hex_str[etc_sensor_list[i].off_time_ms/1000%1000000/100000];	
		debug_sensor_event_str[i+1][122] = hex_str[etc_sensor_list[i].off_time_ms/1000%100000/10000];		
		debug_sensor_event_str[i+1][123] = hex_str[etc_sensor_list[i].off_time_ms/1000%10000/1000];
		debug_sensor_event_str[i+1][124] = hex_str[etc_sensor_list[i].off_time_ms/1000%1000/100];		
		debug_sensor_event_str[i+1][125] = hex_str[etc_sensor_list[i].off_time_ms/1000%100/10];	
		debug_sensor_event_str[i+1][126] = hex_str[etc_sensor_list[i].off_time_ms/1000%10];		
		
		
		debug_sensor_event_str[i+1][127] = '\r';	
		debug_sensor_event_str[i+1][128] = '\n';	
	}
	
	debug_sensor_event_str[i+1][0] = 0;

}



/*** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
unsigned short const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

unsigned short  crc16_byte(unsigned short crc, const unsigned char data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}
/***
 * crc16 - compute the CRC-16 for the data buffer
 * @crc:	previous CRC value
 * @buffer:	data pointer
 * @len:	number of bytes in the buffer
 *
 * Returns the updated CRC value.
 */
unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}


extern UART_HandleTypeDef huart4;
uint8_t uart4_buff[70];



/**********************************************************************************************
***func: 用于单片第一次开启DMA接收
***     
***date: 2017/6/9
*** nome
***********************************************************************************************/
void start_from_uart4_dma_receive()
{
	SET_BIT((&huart4)->Instance->CR1, USART_CR1_IDLEIE);  //打开串口空闲中断
	HAL_UART_Receive_DMA(&huart4,uart4_buff,70);	 //打开DMA接收
}

volatile uint16_t io_flag;        //data is change
volatile uint16_t io_flag_clear;  //data use so clear
volatile uint16_t io;
void uart4_data_handle()
{
	uint16_t* new_io = (uint16_t* )&uart4_buff[3];
	uint16_t * crc = (uint16_t* )&uart4_buff[5];;
	if(crc16(0,uart4_buff,5) == *crc)
	{
		if(io_flag_clear == 0 && io != *new_io) //data has used  and  data changed
		{
			io_flag_clear = 1;
			io_flag = io ^ *new_io;
			io = *new_io;
		}
	}
}

uint32_t uart4_data_flush_times;
/**********************************************************************************************
***func:串口空闲中断回调函数
***     空闲中断用来判断一包数据的结束
***date: 2017/6/9
*** nome
***********************************************************************************************/
void uart_from_uart4_idle_callback()
{
	HAL_DMA_Abort((&huart4)->hdmarx);
	huart4.RxState = HAL_UART_STATE_READY;
	huart4.hdmarx->State = HAL_DMA_STATE_READY;
	uart4_data_flush_times++;
	HAL_UART_Receive_DMA(&huart4,uart4_buff,70);	 //打开DMA接收
	uart4_data_handle();
}

extern int insert_io_data(uint8_t io,uint8_t on_off);
extern int copy_string_to_double_buff(char *pstr);
extern char debug_send_buff[256];

void print_uart4_io_changed()
{
	uint8_t index;
	
	if(io_flag_clear == 1)
	{
		if(io_flag)
		{
			for(index=0;index<12;index++)
			{
				if((1<<index) & io_flag)
				{
					
					if((1<<index) & io)
					{
						insert_io_data(index,1);
						sprintf(debug_send_buff,"------[time=%d:%d io:%d->ON]\r\n",slot_count/128,slot_count%128*8,index);
					}
					else
					{
						insert_io_data(index,0);
						sprintf(debug_send_buff,"------[time=%d:%d io:%d->OFF]\r\n",slot_count/128,slot_count%128*8,index);
					}
					copy_string_to_double_buff(debug_send_buff);	
				}
			}
			io_flag = 0;
		}
		
		io_flag_clear = 0;
	}
}



