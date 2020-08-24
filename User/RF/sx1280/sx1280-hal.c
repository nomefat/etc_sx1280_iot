/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/

//#include "stm32f10x.h"
//#include "stm32f10x_conf.h"
#include "main.h"
#include "sx1280.h"
#include "sx1280-hal.h"
#include "radio.h"
#include <string.h>

/*
#define RADIO_NSS_PIN       GPIO_Pin_12     //PB12
#define RADIO_NSS_PORT      GPIOB

#define RADIO_MOSI_PIN      GPIO_Pin_15      //PB15
#define RADIO_MOSI_PORT     GPIOB

#define RADIO_MISO_PIN      GPIO_Pin_14     //PB14
#define RADIO_MISO_PORT     GPIOB

#define RADIO_SCK_PIN       GPIO_Pin_13     //PB13
#define RADIO_SCK_PORT      GPIOB

#define RADIO_nRESET_PIN    GPIO_Pin_11      //PB11
#define RADIO_nRESET_PORT   GPIOB

#define RADIO_BUSY_PIN      GPIO_Pin_10      //PB10
#define RADIO_BUSY_PORT     GPIOB

#define RADIO_DIOx_PIN      GPIO_Pin_7      //PA7
#define RADIO_DIOx_PORT     GPIOA
*/

extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

/*!
 * \brief Define the size of tx and rx hal buffers
 *
 * The Tx and Rx hal buffers are used for SPI communication to
 * store data to be sent/receive to/from the chip.
 *
 * \warning The application must ensure the maximal useful size to be much lower
 *          than the MAX_HAL_BUFFER_SIZE
 */
#define MAX_HAL_BUFFER_SIZE   0x400

#define IRQ_HIGH_PRIORITY  0
#define delay_ms(s)  do  {uint32_t delay=s*168*1000;while(delay--);}while(0);


extern volatile uint8_t radio_switch;
/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1280Init,
    SX1280HalReset,
    SX1280GetStatus,
    SX1280HalWriteCommand,
    SX1280HalReadCommand,
    SX1280HalWriteRegisters,
    SX1280HalWriteRegister,
    SX1280HalReadRegisters,
    SX1280HalReadRegister,
    SX1280HalWriteBuffer,
    SX1280HalReadBuffer,
    SX1280HalGetDioStatus,
    SX1280GetFirmwareVersion,
    SX1280SetRegulatorMode,
    SX1280SetStandby,
    SX1280SetPacketType,
    SX1280SetModulationParams,
    SX1280SetPacketParams,
    SX1280SetRfFrequency,
    SX1280SetBufferBaseAddresses,
    SX1280SetTxParams,
    SX1280SetDioIrqParams,
    SX1280SetSyncWord,
    SX1280SetRx,
    SX1280GetPayload,
    SX1280SendPayload,
    SX1280SetRangingRole,
    SX1280SetPollingMode,
    SX1280SetInterruptMode,
    SX1280SetRegistersDefault,
    SX1280GetOpMode,
    SX1280SetSleep,
    SX1280SetFs,
    SX1280SetTx,
    SX1280SetRxDutyCycle,
    SX1280SetCad,
    SX1280SetTxContinuousWave,
    SX1280SetTxContinuousPreamble,
    SX1280GetPacketType,
    SX1280SetCadParams,
    SX1280GetRxBufferStatus,
    SX1280GetPacketStatus,
    SX1280GetRssiInst,
    SX1280GetIrqStatus,
    SX1280ClearIrqStatus,
    SX1280Calibrate,
    SX1280SetSaveContext,
    SX1280SetAutoTx,
    SX1280SetAutoFS,
    SX1280SetLongPreamble,
    SX1280SetPayload,
    SX1280SetSyncWordErrorTolerance,
    SX1280SetCrcSeed,
    SX1280SetBleAccessAddress,
    SX1280SetBleAdvertizerAccessAddress,
    SX1280SetCrcPolynomial,
    SX1280SetWhiteningSeed,
    SX1280SetRangingIdLength,
    SX1280SetDeviceRangingAddress,
    SX1280SetRangingRequestAddress,
    SX1280GetRangingResult,
    SX1280SetRangingCalibration,
    SX1280RangingClearFilterResult,
    SX1280RangingSetFilterNumSamples,
    SX1280GetFrequencyError,
};

static uint8_t halTxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};
static uint8_t halRxBuffer[MAX_HAL_BUFFER_SIZE] = {0x00};

/*
void SX1280_HalInit(void)
{
	GPIO_InitTypeDef GPIO_initStruct;
	
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_initStruct.GPIO_Pin = RADIO_NSS_PIN | RADIO_MOSI_PIN  | RADIO_SCK_PIN ;  // CS/MOSI/SCK
	GPIO_initStruct.GPIO_Mode = GPIO_Mode_OUT;        //????
	GPIO_initStruct.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_initStruct.GPIO_OType = GPIO_OType_PP;      //??????
	GPIO_Init(RADIO_NSS_PORT, &GPIO_initStruct); 

    GPIO_initStruct.GPIO_Pin = RADIO_MISO_PIN ;  // MISO
		GPIO_initStruct.GPIO_Mode = GPIO_Mode_IN;
   	GPIO_initStruct.GPIO_PuPd = GPIO_PuPd_UP;  //上拉输入
    GPIO_Init(RADIO_MISO_PORT, &GPIO_initStruct) ;

    GPIO_initStruct.GPIO_Pin = RADIO_DIOx_PIN | RADIO_BUSY_PIN ;  //DIOx
    GPIO_initStruct.GPIO_Mode = GPIO_Mode_IN;
   	GPIO_initStruct.GPIO_PuPd = GPIO_PuPd_UP;   //上拉输入
    GPIO_Init(RADIO_DIOx_PORT, &GPIO_initStruct) ;
	
	GPIO_initStruct.GPIO_Pin = RADIO_nRESET_PIN ;  // RESET 
	GPIO_initStruct.GPIO_Mode = GPIO_Mode_OUT;        //tuiwanshuchu
	GPIO_initStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_initStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RADIO_BUSY_PORT, &GPIO_initStruct); 


	EXTI_ClearITPendingBit(EXTI_Line7); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	
 //   GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);//PA7 连接到中断线0
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);	
	
	//EXTI1 按键中断设为最高优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     

    GPIO_ResetBits( RADIO_SCK_PORT, RADIO_SCK_PIN ) ;
    GPIO_SetBits( RADIO_NSS_PORT, RADIO_NSS_PIN ) ;
}

 

uint8_t SX1280_SimSendByte( uint8_t output )
{
    uint8_t i;
    uint8_t temp;
    uint8_t input = 0;

    for (i=0; i<8; i++)  //8位
    {
        if (output & 0x80u)  // 1
        {
            GPIO_SetBits( RADIO_MOSI_PORT, RADIO_MOSI_PIN ) ;
        }
        else  //0
        {
            GPIO_ResetBits( RADIO_MOSI_PORT, RADIO_MOSI_PIN ) ;
        }
        output <<= 1 ;

        delay_us(1) ;

        GPIO_SetBits( RADIO_SCK_PORT, RADIO_SCK_PIN ) ;
        delay_us(1) ;

        input <<= 1 ;
        temp = GPIO_ReadInputDataBit( RADIO_MISO_PORT, RADIO_MISO_PIN ) ;
        if(temp)
        {
            input = input | 0x01 ;
        }

        GPIO_ResetBits( RADIO_SCK_PORT, RADIO_SCK_PIN ) ;
    }

    return input ;
}


void SpiIn( uint8_t *txBuffer, uint16_t size )
{
    uint16_t i ;

    for( i = 0 ; i < size ; i ++ )
    {
        SX1280_SimSendByte( txBuffer[i] ) ;
        // SPIWrite8bit( txBuffer[i] ) ;
    }
}


void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
    uint16_t i ;

    for ( i = 0 ; i < size ; i ++ )
    {
        rxBuffer[i] = SX1280_SimSendByte( txBuffer[i] ) ;
        // rxBuffer[i] = SPIRead8bit( txBuffer[i] ) ;
    }
}

*/


/*!
 * \brief Used to block execution waiting for low state on radio busy pin.
 *        Essentially used in SPI communications
 */
void SX1280HalWaitOnBusy( void )
{
	uint16_t delay;
	
	delay = 300;
	while(delay--);
//	if(radio_switch == RF1)
//	{
//    while( HAL_GPIO_ReadPin( RF1_BUSY_GPIO_Port, RF1_BUSY_Pin ) == GPIO_PIN_SET );
//	}
//	else if(radio_switch == RF2)
//	{
//    while( HAL_GPIO_ReadPin( RF2_BUSY_GPIO_Port, RF2_BUSY_Pin ) == GPIO_PIN_SET );		
//	}
}

void SX1280HalInit( DioIrqHandler **irqHandlers )
{
    SX1280HalReset( );
    SX1280HalIoIrqInit( irqHandlers );
}

void SX1280HalIoIrqInit( DioIrqHandler **irqHandlers )
{
    //GpioSetIrq( RADIO_DIOx_PORT, RADIO_DIOx_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
}

void SX1280HalReset( void )
{
	if(radio_switch == RF1)
	{
    //delay_ms(50) ;
    HAL_GPIO_WritePin( RF1_RESET_GPIO_Port, RF1_RESET_Pin ,GPIO_PIN_RESET) ;
    delay_ms(10) ;
    HAL_GPIO_WritePin( RF1_RESET_GPIO_Port, RF1_RESET_Pin ,GPIO_PIN_SET) ;
    delay_ms(10) ;		
	}
	else if(radio_switch == RF2)
	{
    //delay_ms(50) ;
    HAL_GPIO_WritePin( RF2_RESET_GPIO_Port, RF2_RESET_Pin,GPIO_PIN_RESET) ;
    delay_ms(10) ;
    HAL_GPIO_WritePin( RF2_RESET_GPIO_Port, RF2_RESET_Pin ,GPIO_PIN_SET) ;
    delay_ms(10) ;		
	}


}

void SX1280HalClearInstructionRam( void )
{
    // Clearing the instruction RAM is writing 0x00s on every bytes of the
    // instruction RAM
    uint16_t halSize = 3 + IRAM_SIZE;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( IRAM_START_ADDRESS >> 8 ) & 0x00FF;
    halTxBuffer[2] = IRAM_START_ADDRESS & 0x00FF;
    for( uint16_t index = 0; index < IRAM_SIZE; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }
		
	if(radio_switch == RF1)
	{
    SX1280HalWaitOnBusy( );

    HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;

    HAL_SPI_Transmit(&hspi4, halTxBuffer, halSize, 1);

		HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

    SX1280HalWaitOnBusy( );
	}
	else if(radio_switch == RF2)
	{
    SX1280HalWaitOnBusy( );

    HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;

    HAL_SPI_Transmit(&hspi5, halTxBuffer, halSize, 1);

		HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

    SX1280HalWaitOnBusy( );
	}	
}

void SX1280HalWakeup( void )
{
    __disable_irq( );

	if(radio_switch == RF1)
	{	
    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
   HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;

    uint16_t halSize = 2;
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    HAL_SPI_Transmit(&hspi4, halTxBuffer, halSize, 1);

    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
    HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );

    __enable_irq( );
	}
	else if(radio_switch == RF2)
	{	
    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
   HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;

    uint16_t halSize = 2;
    halTxBuffer[0] = RADIO_GET_STATUS;
    halTxBuffer[1] = 0x00;
    HAL_SPI_Transmit(&hspi5, halTxBuffer, halSize, 1);

    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
    HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

    // Wait for chip to be ready.
    SX1280HalWaitOnBusy( );

    __enable_irq( );
	}
}

void SX1280HalWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize  = size + 1;
	
	if(radio_switch == RF1)
	{	
    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
		HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;
	
    SX1280HalWaitOnBusy( );

    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );

		HAL_SPI_Transmit(&hspi4, halTxBuffer, halSize, 1);
		HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( );
    }
	}
	else 	if(radio_switch == RF2)
	{	
    // GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
		HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;
	
    SX1280HalWaitOnBusy( );

    halTxBuffer[0] = command;
    memcpy( halTxBuffer + 1, ( uint8_t * )buffer, size * sizeof( uint8_t ) );

		HAL_SPI_Transmit(&hspi5, halTxBuffer, halSize, 1);
		HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

    if( command != RADIO_SET_SLEEP )
    {
        SX1280HalWaitOnBusy( );
    }
	}
}

void SX1280HalReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 2 + size;
    halTxBuffer[0] = command;
    halTxBuffer[1] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[2+index] = 0x00;
    }

		if(radio_switch == RF1)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi4, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 2, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}
		else 		if(radio_switch == RF2)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi5, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 2, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}
}

void SX1280HalWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_WRITE_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    memcpy( halTxBuffer + 3, buffer, size );

	
		if(radio_switch == RF1)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;
		
			SX1280HalWaitOnBusy( );

			HAL_SPI_Transmit(&hspi4, halTxBuffer, halSize, 1);
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

		}
		else 	if(radio_switch == RF2)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;
		
			SX1280HalWaitOnBusy( );

			HAL_SPI_Transmit(&hspi5, halTxBuffer, halSize, 1);
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

		}	
	
}

void SX1280HalWriteRegister( uint16_t address, uint8_t value )
{
    SX1280HalWriteRegisters( address, &value, 1 );
}

void SX1280HalReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint16_t halSize = 4 + size;
    halTxBuffer[0] = RADIO_READ_REGISTER;
    halTxBuffer[1] = ( address & 0xFF00 ) >> 8;
    halTxBuffer[2] = address & 0x00FF;
    halTxBuffer[3] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[4+index] = 0x00;
    }

		if(radio_switch == RF1)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi4, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 4, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}
		else 		if(radio_switch == RF2)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi5, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 4, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}	
		
}

uint8_t SX1280HalReadRegister( uint16_t address )
{
    uint8_t data;

    SX1280HalReadRegisters( address, &data, 1 );

    return data;
}

void SX1280HalWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 2;
    halTxBuffer[0] = RADIO_WRITE_BUFFER;
    halTxBuffer[1] = ( offset ) >> 8;
    memcpy( halTxBuffer + 2, buffer, size );

		if(radio_switch == RF1)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;
		
			SX1280HalWaitOnBusy( );

			HAL_SPI_Transmit(&hspi4, halTxBuffer, halSize, 1);
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

		}
		else 	if(radio_switch == RF2)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;
		
			SX1280HalWaitOnBusy( );

			HAL_SPI_Transmit(&hspi5, halTxBuffer, halSize, 1);
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

		}	
}

void SX1280HalReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint16_t halSize = size + 3;
    halTxBuffer[0] = RADIO_READ_BUFFER;
    halTxBuffer[1] = offset;
    halTxBuffer[2] = 0x00;
    for( uint16_t index = 0; index < size; index++ )
    {
        halTxBuffer[3+index] = 0x00;
    }

		if(radio_switch == RF1)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi4, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 3, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF1_NSS_GPIO_Port, RF1_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}
		else 		if(radio_switch == RF2)
		{	
			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_RESET) ;		
			SX1280HalWaitOnBusy( );

			HAL_SPI_TransmitReceive(&hspi5, halTxBuffer, halRxBuffer,halSize, 1);
			//SpiInOut( halTxBuffer, halRxBuffer, halSize );

			memcpy( buffer, halRxBuffer + 3, size );

			// GpioWrite( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );
			HAL_GPIO_WritePin( RF2_NSS_GPIO_Port, RF2_NSS_Pin,GPIO_PIN_SET) ;

			SX1280HalWaitOnBusy( );
		}	
		
}

uint8_t SX1280HalGetDioStatus( void )
{
		if(radio_switch == RF1)
		{	
			
			return ( HAL_GPIO_ReadPin( RF1_DIO1_EXTI9_GPIO_Port, RF1_DIO1_EXTI9_Pin ) << 1 ) | ( HAL_GPIO_ReadPin( RF1_BUSY_GPIO_Port, RF1_BUSY_Pin ) << 0 );
		}
		else if(radio_switch == RF2)
		{
			return ( HAL_GPIO_ReadPin( RF2_DIO1_EXTI3_GPIO_Port, RF1_DIO1_EXTI9_Pin ) << 1 ) | ( HAL_GPIO_ReadPin( RF2_BUSY_GPIO_Port, RF2_BUSY_Pin ) << 0 );
			
		}
		
		
		return 0;
		
		
}










