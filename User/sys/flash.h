#ifndef FLASH_H_
#define FLASH_H_


#include "stm32f4xx_hal.h"



#define AP_VERSION 0x19
//V:7  增加USB KEY   XOR校验  存储5次   读对任意一次即OK
//v:10 去掉USB KEY功能 增加串口设置sensor id  和远程设置sensor id 的功能
//v:0x11  增加指令使能rf1 rf2长发ack包    和收另一个rf的ack包  打印结果
//V:0X12 发现每日凌晨gps对时会导致日期少一天，修改为每天大于5点才会对时
//v:0x13 每日凌晨8点之前日期会少一天，是由于gps对时小时=8 溢出24的时候没有手动增加日期
//...... 修改跳频参数制作函数，IO6实际没有sesor id  如果参数有id会优先采用sensor id制作通道下发给sensor 而ap跳频没采用 导致通道不匹配收不到数据
//V:0X14 修改gps对时的bug 发现gps读取的时间一直是19:34:36 导致对时错误  更改为对比2次gps时间  相同的时候视为无效数据 增加gps时间先转换为秒再加8小时的秒数再对时
//V:0X15 增加了 rev_1000 0/1 syn_forever 0/1 send_wave 0/1 等命令  接收1000包  持续发送  单载波发送
//v:0x16 去掉4G模块查询 CREG的命令   返回0 3 注册被拒绝  实际上不查询后 也能正常入网 

//v:0x17 增加串口4接收io检测版程序，输出打印和4g回传
//V:0X18 IO ON OFF AND SHUNXU
//V:0X19 add test rf   timeout send test result








#define FLASH_AP_PARAM_BEGIN_1          0x08100000                     //AP参数起始地址1
#define FLASH_AP_PARAM_BEGIN_2          0x08104000                     //AP参数起始地址2


#define FLASH_RP_FIRMWARE_BEGIN       0x08120000         //RP固件起始地址
#define FLASH_SENSOR_FIRMWARE_BEGIN   0x08140000     //sensor固件起始地址
#define FLASH_AP_FIRMWARE_BEGIN       0x08160008     //ap固件起始地址
#define FLASH_AP_FIRMWARE_HEAD       0x08160000     //ap固件起始地址


typedef struct _flash_head_crc{

	unsigned int head;
	unsigned int crc;

}struct_flash_head_crc;





void read_ap_param_flash(void);
void write_ap_param_flash(void);
int32_t write_bin_flash(uint32_t address,uint8_t *pdata,uint32_t size);







#endif


