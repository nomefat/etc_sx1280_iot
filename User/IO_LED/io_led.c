
#include "io_led.h"
#include "etc_rf.h"

extern struct_sensor_list etc_sensor_list[IO_COUNT];
__IO uint16_t ErrTimeout[7];
uint8_t Channel_State_Bm; // 1- on  0- off

static CH_LED ch_led_list[7] =
{
	{S1_G_GPIO_Port,S1_G_Pin},
	{S2_G_GPIO_Port,S2_G_Pin},
	{S3_G_GPIO_Port,S3_G_Pin},
	{S4_G_GPIO_Port,S4_G_Pin},
	{S5_G_GPIO_Port,S5_G_Pin},
	{S6_G_GPIO_Port,S6_G_Pin},
	//{S7_G_GPIO_Port,S7_G_Pin},
//	{SYS_R_GPIO_Port,SYS_R_Pin,SYS_G_GPIO_Port,SYS_G_Pin}
};

static TEST_LED test_led_list[10] =
{
	{LED1_GPIO_Port,LED1_Pin},
	{LED2_GPIO_Port,LED2_Pin},
	{LED3_GPIO_Port,LED3_Pin},
	{LED4_GPIO_Port,LED4_Pin},
	{LED5_GPIO_Port,LED5_Pin},
	{LED6_GPIO_Port,LED6_Pin},
	{LED7_GPIO_Port,LED7_Pin},
	{LED8_GPIO_Port,LED8_Pin},
	{LED9_GPIO_Port,LED9_Pin},
	{LED10_GPIO_Port,LED10_Pin},
//	{SYS_R_GPIO_Port,SYS_R_Pin,SYS_G_GPIO_Port,SYS_G_Pin}
};

uint8_t IO_Stauts;



void test_led_switch(uint8_t led,uint8_t on_off)
{
	if(on_off)
	{
		HAL_GPIO_WritePin(test_led_list[led].GPIOx,test_led_list[led].GPIO_Pin,GPIO_PIN_RESET);
	}
	else
		HAL_GPIO_WritePin(test_led_list[led].GPIOx,test_led_list[led].GPIO_Pin,GPIO_PIN_SET);		
}



/** Func:IO通道指示灯操作
  * Param: ch-通道号(0~6)  led_st-Led状态(NORM-正常 ERR-异常)
	* Ret: 无
	*/
void Channel_Led_Op(uint8_t ch, LED_ST led_st)
{
	if(ch > 6)
		ch = 6;
	if(led_st == RED)
	{
		//绿灯亮 红灯灭
//		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_R, ch_led_list[ch].GPIO_Pin_R, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_G, ch_led_list[ch].GPIO_Pin_G, GPIO_PIN_RESET);
	}
	else if(led_st == GREEN)
	{
		//红灯亮 绿灯灭
//		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_R, ch_led_list[ch].GPIO_Pin_R, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_G, ch_led_list[ch].GPIO_Pin_G, GPIO_PIN_SET);
	}
	else if(led_st == LEDOFF)
	{
		//红灯亮 绿灯灭
//		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_R, ch_led_list[ch].GPIO_Pin_R, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_G, ch_led_list[ch].GPIO_Pin_G, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_R, ch_led_list[ch].GPIO_Pin_R, GPIO_PIN_SET);
	}	
}

void All_Channel_Led_Off()
{
	uint8_t ch;
	
	for(ch=0; ch<7; ch++)
	{
//		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_R, ch_led_list[ch].GPIO_Pin_R, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch_led_list[ch].GPIOx_G, ch_led_list[ch].GPIO_Pin_G, GPIO_PIN_SET);
	}
}

//检测器接收RF正常处理
void Sensor_Norm_Proc(uint8_t ch)
{
	ErrTimeout[ch] = 0;
	//Channel_Led_Op(ch, NORM);
}

//检测器接收RF超时处理,在主循环while(1)中调用
void Sensor_RfTimeout_Proc(uint16_t timeout_count)
{
	uint8_t i;
	
	for(i=0; i<7; i++)
	{
		if(ErrTimeout[i] < timeout_count)
				ErrTimeout[i]++;
		else
		{
			if(etc_sensor_list[i].sensor_id != 0x0000)
						Channel_Led_Op(i, RED);
		}
	}
}

uint8_t Test_Flag = 0;
uint8_t ch_n = 0;
uint8_t ch_e = 0;
//走马灯测试
void Led_Test()
{
  
	if(Test_Flag == 1)
	{
		if(ch_n < 7)
		{
			Channel_Led_Op(ch_n, RED);
			IO_Op(ch_n, SON);
			ch_n++;
		}
		else
		{
			Channel_Led_Op(ch_e, RED);
			ch_e++;
			if(ch_e >= 8) //处理最后的led亮灯时间，此处改为8
			{
				Test_Flag = 0;
				ch_e = 0;
				ch_n = 0;
				All_Channel_Led_Off();
				ALL_Channel_IO_Off();
			}
		}		
	}
}

static IO_PIN_STRUCT  Io_Pin[7] = 
{
	{DATA1_GPIO_Port, DATA1_Pin},   //IO1
	{DATA2_GPIO_Port, DATA2_Pin},
	{DATA3_GPIO_Port, DATA3_Pin},
	{DATA4_GPIO_Port, DATA4_Pin},
	{DATA5_GPIO_Port, DATA5_Pin},
	{DATA6_GPIO_Port, DATA6_Pin}

};
/** Func:IO输出操作
  * Param: ch-通道号(0~6)  led_st-IO状态(SON  SOFF)
	* Ret: 无
	*/
void IO_Op(uint8_t ch, IO_ST io_st)
{
	uint8_t i;
	
	if(ch > 6)
		ch = 6;
	if(io_st == SON)
	{
		IO_Stauts |= (1<<ch);
	}
	else if(io_st == SOFF)
	{
		IO_Stauts &= ~(1<<ch);
	}
	
	if((1<<ch) & IO_Stauts)
	{
		HAL_GPIO_WritePin(Io_Pin[ch].GPIOx, Io_Pin[ch].GPIO_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(Io_Pin[ch].GPIOx, Io_Pin[ch].GPIO_Pin, GPIO_PIN_SET);
	}
	
	HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, GPIO_PIN_RESET);
}

void ALL_Channel_IO_Off()
{
	uint8_t i;
	
	for(i=0; i<7; i++)
	{
		IO_Op(i, SOFF);
	}
}

void Channel_IO_Proc()
{
	uint8_t ch;
	
	for(ch=0; ch<7; ch++)
	{
		if(etc_sensor_list[ch].now_on_off_status == NONE)
			continue;
		
		if((etc_sensor_list[ch].now_on_off_status == ON) && ((Channel_State_Bm &(1<<ch)) == 0))
		{
			IO_Op(ch, SON);
			Channel_State_Bm |= (1<<ch);
		}
		else if((etc_sensor_list[ch].now_on_off_status == OFF) && ((Channel_State_Bm &(1<<ch)) != 0))
		{
			IO_Op(ch, SOFF);
			Channel_State_Bm &= ~(1<<ch);
		}
	}
}

void Watch_Dog_Reset()
{
	HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
}


int8_t time_tick[6];
extern TIM_HandleTypeDef htim2;

void lift_bar(int8_t io)
{
		time_tick[io] = 3;
		HAL_TIM_Base_Start_IT(&htim2);	
}





void drop_rod(int8_t io)
{
	time_tick[io] = 5;
	HAL_TIM_Base_Start_IT(&htim2);	
}



void timer_500us_handle(void)
{
	int8_t i = 0;
	int8_t if_has_task = 0;

	for(i=0;i<6;i++)
	{
		if(time_tick[i] > 0)
		{
			if_has_task = 1;
			time_tick[i]--;
			HAL_GPIO_WritePin(Io_Pin[i].GPIOx, Io_Pin[i].GPIO_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(Io_Pin[i].GPIOx, Io_Pin[i].GPIO_Pin, GPIO_PIN_RESET);
		}
				
	}
	if(if_has_task == 0)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timer_500us_handle();
}