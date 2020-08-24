/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include <string.h>
// #include "main.h"
// #include "hw.h"
#include "sx1280.h"
#include "sx1280-hal.h"
#include "radio.h"
#include "main.h"


#define DATA_LEN 15

volatile PacketStatus_t pktStatus ;
volatile uint8_t radio_switch;
volatile int8_t now_can_read_rssi = -1;
extern unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);


uint32_t rf_ch[32] = {2405000000, 2407500000, 2410000000, 2412500000, 2415000000, 2417500000, 2420000000, 2422500000, 2425000000, 2427500000, 
	2430000000, 2432500000, 2435000000, 2437500000, 2440000000, 2442500000, 2445000000, 2447500000, 2450000000, 2452500000, 2455000000,
2457500000, 2460000000, 2462500000, 2465000000, 2467500000, 2470000000, 2472500000, 2475000000, 2477500000, 2480000000, 2482500000};


/*!
 * \brief Used to display firmware version UART flow
 */
#define FIRMWARE_VERSION    ( ( char* )"Firmware Version: 170919A" )

/*!
 * Select mode of operation for the Ping Ping application
 */
//#define MODE_BLE
#define MODE_LORA
//#define MODE_GFSK
//#define MODE_FLRC


#define RF_BL_ADV_CHANNEL_38                        2426000000u // Hz

/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                RF_BL_ADV_CHANNEL_38 // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                             0

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#define BUFFER_SIZE                                 20

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            10 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            50000 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                                4


#define printf(s)       
#define delay_ms(s)  do  {uint32_t delay=s*50*1000;while(delay--);}while(0);

extern uint8_t now_slot;
extern void etc_rev_rf_data(void *p_rf_data,uint8_t slot);

/*!
 * \brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;



strcut_radio_status radio_status[2];

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t Callbacks[2] =
{
    {&OnTxDone1,        // txDone
    &OnRxDone1,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout1,     // txTimeout
    &OnRxTimeout1,     // rxTimeout
    &OnRxError1,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone
		},
    {&OnTxDone2,        // txDone
    &OnRxDone2,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout2,     // txTimeout
    &OnRxTimeout2,     // rxTimeout
    &OnRxError2,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone	
		}
};

/*!
 * \brief The size of the buffer
 */
uint8_t BufferSize = BUFFER_SIZE;

/*!
 * \brief The buffer
 */
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief The State of the application
 */
AppStates_t AppState = APP_LOWPOWER;

#if defined( MODE_BLE )
/*!
 * \brief In case of BLE, the payload must contain the header
 */
typedef union
{
    struct BleAdvHeaderField_s
    {
        uint8_t pduType: 4;
        uint8_t rfu1:2;
        uint8_t txAddr:1;
        uint8_t rxAddr:1;
        uint8_t length:6;
        uint8_t rfu2:2;
    } Fields;
    uint8_t Serial[ 2 ];
}BleAdvHeaders_t;
BleAdvHeaders_t ble_header_adv;
#endif // MODE_BLE

PacketParams_t packetParams;

PacketStatus_t packetStatus;

const TickTime_t _RX_TIMEOUT = { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } ;
const TickTime_t _TX_TIMEOUT = { RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } ;

RadioStatus_t RadioStatus;

void radio_init(void)
{
    ModulationParams_t modulationParams;
		
		HAL_GPIO_WritePin(RF1_PWR_EN_GPIO_Port,RF1_PWR_EN_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(RF2_PWR_EN_GPIO_Port,RF2_PWR_EN_Pin,GPIO_PIN_SET);
	
    delay_ms( 50 );
		radio_switch = RF1;
    Radio.Init( &Callbacks[0], &Callbacks[1]) ;
    Radio.SetRegulatorMode( USE_DCDC ) ;

		
		radio_switch = RF2;
    Radio.Init( &Callbacks[0], &Callbacks[1]) ;
    Radio.SetRegulatorMode( USE_DCDC ) ;
	
    //配置参数
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF6;
    modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;
    modulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 12;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = DATA_LEN;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_OFF;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
		
/******************************/
		radio_switch = RF1;
    Radio.SetStandby( STDBY_RC );
    Radio.SetPacketType( modulationParams.PacketType );
    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &packetParams );
    Radio.SetRfFrequency( 2450000000 );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_02_US );

    //设置接收模式
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
		
/******************************/
		radio_switch = RF2;
    Radio.SetStandby( STDBY_RC );
		RadioStatus = Radio.GetStatus();
    Radio.SetPacketType( modulationParams.PacketType );
    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &packetParams );
    Radio.SetRfFrequency( 2450000000 );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_02_US );

    //设置接收模式
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );		
		
}


#define RADIO_RECEIVE_BUFFER_LENGTH 64
uint8_t _radio_receive_buffer[RADIO_RECEIVE_BUFFER_LENGTH] ;
uint8_t _radio_receive_bytes = 0 ;  //接收字节数大于0,即表示收到新数据
int8_t nodata_rssi[2];



void get_nodata_rssi()
{
	static int32_t flag = 0;
	uint32_t ret;
	
	ret = radio_switch;
	if(flag)
	{
		flag = 0;
		radio_switch = RF1;
		nodata_rssi[0] = Radio.GetRssiInst();
	}
	else
	{
		flag = 1;
		radio_switch = RF2;
		nodata_rssi[1] = Radio.GetRssiInst();
	}		
	radio_switch = ret;
}

void poll_get_nodata_rssi()
{
	if(now_can_read_rssi==RF1)
	{
		now_can_read_rssi = -1;
		 __disable_irq();
		nodata_rssi[0] = Radio.GetRssiInst();	
		__enable_irq();		
	}
	else if(now_can_read_rssi==RF2)
	{
		now_can_read_rssi = -1;
		 __disable_irq();
		nodata_rssi[1] = Radio.GetRssiInst();
		__enable_irq();			
	}
}

//发送完成
void OnTxDone1( void )
{
	uint32_t delay = 500;
    AppState = APP_LOWPOWER ;
		radio_switch = RF1;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT );   

		HAL_GPIO_WritePin(RF1_LED_1_GPIO_Port,RF1_LED_1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);	
//		while(delay--);
//		nodata_rssi[0] = Radio.GetRssiInst();
//		nodata_rssi[0] = Radio.GetRssiInst();
		radio_status[radio_switch].rf_send_error_times--;

}
uint32_t rf1_crc_error;


uint8_t rx_fun;



//接收完成
void OnRxDone1( void )
{
	  uint8_t ret;
		uint16_t crc;	
    AppState = APP_LOWPOWER ;
		radio_switch = RF1;
		rx_fun = 1;
		memset(_radio_receive_buffer,0,RADIO_RECEIVE_BUFFER_LENGTH);
    
		ret = Radio.GetPayload( _radio_receive_buffer, &_radio_receive_bytes, RADIO_RECEIVE_BUFFER_LENGTH );
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		if(0 == ret && _radio_receive_bytes>2)
		{
			HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);		
			radio_status[radio_switch].rf_rev_nothing_time = 0;
			Radio.GetPacketStatus(&pktStatus);	
			//nodata_rssi[0] = Radio.GetRssiInst();
			crc = crc16(0, &_radio_receive_buffer[2], _radio_receive_bytes-2);
			if(crc == (_radio_receive_buffer[0] | _radio_receive_buffer[1]<<8) )
				etc_rev_rf_data(_radio_receive_buffer,now_slot);
			else
				rf1_crc_error++;
			now_can_read_rssi = RF1;
		}
		//nodata_rssi[0] = Radio.GetRssiInst();
		rx_fun = 0;
}

//发送超时
void OnTxTimeout1( void )
{
		radio_status[radio_switch].rf_send_timeout_times++;
    AppState = APP_TX_TIMEOUT;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT );
		HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);		
}

//接收超时
//接收超时属于正常情况,只需要设置继续接收
void OnRxTimeout1( void )
{
    AppState = APP_LOWPOWER;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);		
}

//接收出错
//这里可以做接收失败的异常处理
void OnRxError1( IrqErrorCode_t errorCode )
{
    AppState = APP_LOWPOWER;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);		
}

//发送完成
void OnTxDone2( void )
{
	uint32_t delay = 500;
    AppState = APP_LOWPOWER ;
		radio_switch = RF2;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		HAL_GPIO_WritePin(RF2_LED_2_GPIO_Port,RF2_LED_2_Pin,GPIO_PIN_SET);	

		HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);
//		while(delay--);
//		nodata_rssi[1] = Radio.GetRssiInst();
//		nodata_rssi[1] = Radio.GetRssiInst();
		radio_status[radio_switch].rf_send_error_times--;
}

uint32_t rf2_crc_error;
//接收完成
void OnRxDone2( void )
{
	 uint8_t ret;
		uint16_t crc;	
    AppState = APP_LOWPOWER ;
		radio_switch = RF2;
		rx_fun = 2;
		memset(_radio_receive_buffer,0,RADIO_RECEIVE_BUFFER_LENGTH);
    
		 ret = Radio.GetPayload( _radio_receive_buffer, &_radio_receive_bytes, RADIO_RECEIVE_BUFFER_LENGTH );
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
	
		if(0== ret && _radio_receive_bytes>2)
		{
			HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);
			radio_status[radio_switch].rf_rev_nothing_time = 0;
			Radio.GetPacketStatus(&pktStatus);	
			//nodata_rssi[1] = Radio.GetRssiInst();
			crc = crc16(0, &_radio_receive_buffer[2], _radio_receive_bytes-2);
			if(crc == (_radio_receive_buffer[0] | _radio_receive_buffer[1]<<8))
				etc_rev_rf_data(_radio_receive_buffer,now_slot);
			else
				rf2_crc_error++;
			now_can_read_rssi = RF2;
			//nodata_rssi[1] = Radio.GetRssiInst();
		}
		rx_fun = 0;
}

//发送超时
void OnTxTimeout2( void )
{
		radio_status[radio_switch].rf_send_timeout_times++;
    AppState = APP_TX_TIMEOUT;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT );
		HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);	
}

//接收超时
//接收超时属于正常情况,只需要设置继续接收
void OnRxTimeout2( void )
{
    AppState = APP_LOWPOWER;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);	
}

//接收出错
//这里可以做接收失败的异常处理
void OnRxError2( IrqErrorCode_t errorCode )
{
    AppState = APP_LOWPOWER;
    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SetRx( _RX_TIMEOUT ); 
		HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);	
}



void Radio_write_data( uint8_t *data, uint8_t rf ,uint8_t len)
{
	uint16_t crc;
	radio_switch = rf;
  Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE ) ; //设置DIO1的中断	
	crc = crc16(0, &data[2], len-2);
	data[0] = crc;
	data[1] = crc>>8;
  Radio.SetPayload( data, len) ;
	packetParams.Params.LoRa.PayloadLength = len;
	Radio.SetPacketParams( &packetParams );	
}
void Radio_start_send( uint8_t rf )
{
	radio_switch = rf;


  Radio.SetTx(_TX_TIMEOUT) ;
	radio_status[rf].rf_send_error_times++;
}


void Radio_set_rf_ch(uint8_t rf,uint8_t ch)
{
	radio_switch = rf;
	Radio.SetRfFrequency(rf_ch[ch%32]);

  Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
  Radio.SetRx( _RX_TIMEOUT );
	if(radio_switch == RF1)
	{

		HAL_GPIO_WritePin(RF1_TX_EN_GPIO_Port,RF1_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF1_RX_EN_GPIO_Port,RF1_RX_EN_Pin,GPIO_PIN_SET);
	}
	else if(radio_switch == RF2)
	{

		HAL_GPIO_WritePin(RF2_TX_EN_GPIO_Port,RF2_TX_EN_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RF2_RX_EN_GPIO_Port,RF2_RX_EN_Pin,GPIO_PIN_SET);
	}			
}



void etc_reboot_rf(uint8_t rf)
{
    ModulationParams_t modulationParams;
		uint32_t delay = 10000;
	
    //配置参数
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF6;
    modulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;
    modulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = 12;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = DATA_LEN;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_OFF;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;	
	
		if(radio_switch == RF1)
		{
			HAL_GPIO_WritePin(RF1_RESET_GPIO_Port,RF1_RESET_Pin,GPIO_PIN_RESET);
			while(delay--);
			HAL_GPIO_WritePin(RF1_RESET_GPIO_Port,RF1_RESET_Pin,GPIO_PIN_SET);
		}
		else if(radio_switch == RF2)
		{
			HAL_GPIO_WritePin(RF2_RESET_GPIO_Port,RF2_RESET_Pin,GPIO_PIN_RESET);
			while(delay--);
			HAL_GPIO_WritePin(RF2_RESET_GPIO_Port,RF2_RESET_Pin,GPIO_PIN_SET);			
		}
		delay = 100000;
		while(delay--);
		Radio.SetStandby( STDBY_RC );
		Radio.SetPacketType( modulationParams.PacketType );
		Radio.SetModulationParams( &modulationParams );
		Radio.SetPacketParams( &packetParams );
		Radio.SetBufferBaseAddresses( 0x00, 0x00 );
		Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_02_US );

		//设置接收模式
		Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
		Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );			
	
}



