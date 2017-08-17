
 /**
  * @TotenBoard
  * 
  * 	Board Support Package
  *
  *	To use this BSP, you will need:
  *	
  *	- Call "TotenInitHW.TotenPeripInit()" to initialize the peripherals of the board; 
  *	- After that, you can use the object your prefer: "TotenObjs[LED_BLUE]",
  *											"TotenObjs[LED_GREEN]", etc;
  *
  *
  *
  */
  

#include "TotenBoard.h"
#include "string.h"



 /**
  * @Variables
  */
ADC_HandleTypeDef 	hadc;  
UART_HandleTypeDef 	huart2;
uint32_t 			prescaler_ms;
uint32_t 			prescaler_us;



/* 8 user defined characters to be loaded into CGRAM (used for bargraph)*/
const char UserFont[8][8] = {  
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    { 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 },
    { 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 },
    { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C },
    { 0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E },
    { 0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F },
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
    { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
};



/**
  * Bluetooth variables
  */
#ifdef INIT_BLUETOOTH_CONFIG

uint8_t CmdAt1[] = {"AT\r\n"};
uint8_t CmdAt2[] = {"AT+ORGL\r\n"};
uint8_t CmdAt3[] = {"AT+UART=38400,0,0\r\n"};	
uint8_t CmdAt4[] = {"AT+NAME=BTMASTER\r\n"};
uint8_t CmdAt5[] = {"AT+PSWD=1234\r\n"};
uint8_t CmdAt6[] = {"AT+ROLE=1\r\n"};
uint8_t CmdAt7[] = {"AT+CMODE=1\r\n"};
uint8_t CmdAt8[] = {"AT+INIT\r\n"};

uint8_t 	 RxBuffer[RX_BUFFER_SIZE] = {0};
pFunc	 pInitBt;
BT_TypeDef BtData;

#endif



 /**
  * Instantiating  the GPIO out object
  */  
Toten_GPIO_OutTypeDef TotenGpio[NUM_OBJS] = {
	
	{SetPin, 		ResetPin, 	TogglePin}, //LED_BLUE
	{SetPin, 		ResetPin, 	TogglePin}, //LED_GREEN
	{SetPin, 		ResetPin, 	TogglePin}, //USB
	{SetPin, 		ResetPin, 	TogglePin}, //WiFi
	{SetPin, 		ResetPin, 	TogglePin}, //BT
};


Toten_GPIO_InTypeDef TotenGpioUSER = {
	ReadPin,
	GPIO_PIN_RESET
};
	
 /**
  * Instantiating  the Hardware Init. object
  */
Toten_PeriphTypeDef  TotenInitHW = {
	TotenInitHWs
};



 /**
  * Instantiating  the UART object
  */
Toten_UART_InitTypeDef TotenUart = {
	SendReceiveDataUart,
	HAL_OK
};



 /**
  * Instantiating  the LCD object
  */
LCD_TypeDef TotenLCD = {
	LCD_Init,
	LCD_GoTo,
	LCD_Clear,
	LCD_SendText,
	LCD_Num
	
};



 /**
  * Instantiating the ADC object
  */
AD_TypeDef TotenADC = {
	GetValue,
	0
	
};


 /**
  * @brief: Configuring the UART
  * @param  none
  * @retval none
  */
void MX_USART2_UART_Init(void)
{
	huart2.Instance 			= USART2;
	huart2.Init.BaudRate 		= 38400;
	huart2.Init.WordLength 		= UART_WORDLENGTH_8B;
	huart2.Init.StopBits 		= UART_STOPBITS_1;
	huart2.Init.Parity 			= UART_PARITY_NONE;
	huart2.Init.Mode 			= UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart2.Init.OverSampling 	= UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling 	= UART_ONEBIT_SAMPLING_DISABLED ;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart2);

}



 /**
  * @brief: initialize all GPIOs used in this board
  * @param  none
  * @retval none
  */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  
  GPIO_InitStruct.Pin 	= USER_BUTTON_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin 	= RELE_1_Pin |
					  RELE_2_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  
  GPIO_InitStruct.Pin 	= BLUE_LED_Pin	|
					  GREEN_LED_Pin|
					  CTL_WIFI_Pin	|
					  CTL_USB_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

 
  GPIO_InitStruct.Pin 	= CTL_BT_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
}



 /**
  * @brief: Initialize ADC.
  * @param  none 
  * @retval none
  */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler 	= ADC_CLOCK_ASYNC;
  hadc.Init.Resolution 		= ADC_RESOLUTION12b;
  hadc.Init.DataAlign 		= ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode 	= ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection 	= EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait 	= DISABLE;
  hadc.Init.LowPowerAutoPowerOff 	= DISABLE;
  hadc.Init.ContinuousConvMode 	= ENABLE;
  hadc.Init.DiscontinuousConvMode 	= DISABLE;
  hadc.Init.ExternalTrigConvEdge 	= ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests 	= DISABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel 	= ADC_CHANNEL_4;
  sConfig.Rank 	= ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  
  HAL_ADC_Start(&hadc);

}



 /**
  * @brief: initialize all hardwares used in this board
  * @param  none
  * @retval none
  */
void TotenInitHWs(void)
{	
	/* Insert your hardware configuration here! */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC_Init();
}



 /**
  * @brief: set a specific pin
  * @param  Pin: pin to be setted.
  * @retval none
  */
void SetPin(uint16_t Pin)
{
	if(Pin == CTL_BT_Pin)
	{
		/* Insert your HAL here! */
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, Pin, GPIO_PIN_SET);
	}
}



 /**
  * @brief: reset a specific pin
  * @param  Pin: pin to be reseted.
  * @retval none
  */
void ResetPin(uint16_t Pin)
{
	if(Pin == CTL_BT_Pin)
	{
		/* Insert your HAL here! */
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, Pin, GPIO_PIN_RESET);
	}	
}



 /**
  * @brief: toggle a specific pin
  * @param  Pin: pin to be toggled.
  * @retval none
  */
void TogglePin(uint16_t Pin)
{
	if(Pin == CTL_BT_Pin)
	{
		/* Insert your HAL here! */
		HAL_GPIO_TogglePin(GPIOD, Pin);
	}
	else
	{
		HAL_GPIO_TogglePin(GPIOC, Pin);
	}	
}



 /**
  * @brief: read a specific pin
  * @param  Pin: pin to be read.
  * @retval GPIO_PinState: the state of the pin (GPIO_PIN_RESET or GPIO_PIN_SET)
  */
void ReadPin(uint16_t Pin)
{
	/* Initial value */	
	TotenGpioUSER.PinState = GPIO_PIN_RESET;
	
	/* Actual value */
	TotenGpioUSER.PinState = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, Pin);
	
}



 /**
  * @brief: this function has two roles - send and receive data by serial.
  * @param device	: choose the module to send or receive data (USB, WiFi or Bluetooth).
  * @param operation: send or receive (RECEIVE_UART or SEND_UART).
  * @param Size	: amount of data.
  * @retval none
  */
void SendReceiveDataUart(const unsigned char device, unsigned char operation, unsigned char *p, unsigned int size)
{
	if(p != NULL)
	{
		/* Select which module will send the data */
		switch(device)
		{
			case USB:		/* Enable the channel */
						TotenGpio[USB].SetPin  (CTL_USB_Pin);
						TotenGpio[BT].ResePin  (CTL_BT_Pin);
						TotenGpio[WiFi].ResePin(CTL_WIFI_Pin); 						
						break;
			
			case WiFi:	/* Enable the channel */
						TotenGpio[USB].ResePin (CTL_USB_Pin);
						TotenGpio[BT].ResePin  (CTL_BT_Pin);
						TotenGpio[WiFi].SetPin (CTL_WIFI_Pin); 
						break;
			
			case BT:		/* Enable the channel */
						TotenGpio[USB].ResePin (CTL_USB_Pin);
						TotenGpio[BT].SetPin   (CTL_BT_Pin);
						TotenGpio[WiFi].ResePin(CTL_WIFI_Pin); 
						break;
			
			default:		/* Enable all channels */
						TotenGpio[USB].ResePin (CTL_USB_Pin);
						TotenGpio[BT].ResePin  (CTL_BT_Pin);
						TotenGpio[WiFi].ResePin(CTL_WIFI_Pin);
						break;		
			
		}	

		/* Operation: send or receive */
		if(operation == SEND_UART)
		{
			TotenUart.StatusUart = HAL_UART_Transmit(&huart2, p, size, 10);
		}
		else	if(operation == RECEIVE_UART)
		{
			TotenUart.StatusUart = HAL_UART_Receive(&huart2, p, size, 10);
		}		
	}
}



#ifdef INIT_BLUETOOTH_CONFIG

 /**
  * @brief: send AT command to the module
  * @param  none
  * @retval none
  */
void vSendCmdBt(void)
{
	HAL_UART_Transmit(&huart2, BtData.pData[BtData.Index], BtData.LengData[BtData.Index], 10);
	
	/* If the all command were sent and returning "OK", then FINISH the Bluetooth init.. */
	if(BtData.Index >= INIT_CPLT_BT)
	{
		BtData.State = FinishBt;
		pInitBt 	   = BtData.pFuncBt[BtData.State];
	}
	else
	{
		/* Keep sendig AT commands */
		BtData.State 	= WaitCpltBt;
		pInitBt 		= BtData.pFuncBt[BtData.State];
	}
}



 /**
  * @brief: receive the confirmation "OK" of the Bluetooth module
  * @param  none
  * @retval none
  */
void vWaitCpltBt(void)
{
	BtData.TimeOut = HAL_GetTick();
	
	HAL_UART_Receive(&huart2, RxBuffer, RX_BUFFER_SIZE, 10);
	
	/* The return command is "OK", indicating that sent command was right */
	if(RxBuffer[0] == 'O' && RxBuffer[1] == 'K')
	{
		BtData.TimeOut = 0; 
		BtData.State 	= NextBt;
		pInitBt 		= BtData.pFuncBt[BtData.State];
	}
	else
	{	
		/* Timeout procedure */
		if(BtData.TimeOut >= _4s) 
		{
			TotenLCD.Clear();
			TotenLCD.XY(0,1);
			TotenLCD.Text("BlueTooth Fail!");
			TotenLCD.XY(0,2); 
			TotenLCD.Text("Reset the board!");  			
			for(;;){}		
		} /* Send another command to try stablish the comunication */
		else if(BtData.TimeOut >= _2s) 
		{
			BtData.State = SendCmdBt;
			pInitBt 	   = BtData.pFuncBt[BtData.State];		
		}
	}
}



 /**
  * @brief: Finish the Bluetooth configuration with success!
  * @param  none
  * @retval none
  */
void vFinishBt(void)
{
	return; /* Back to main */
}


 /**
  * @brief: Send the next AT command
  * @param  none
  * @retval none
  */
void vNextCommandBt(void)
{		
	uint8_t i;
	BtData.Index++;
	
	/* Clean the Buffer */
	for(i = 0; i < sizeof(RxBuffer) / sizeof(uint8_t); i++)
	{
		RxBuffer[i] = 0;
	}	
	BtData.State 	= SendCmdBt;
	pInitBt 	   	= BtData.pFuncBt[BtData.State];
}	



 /**
  * @brief: init. Bluetooth.
  * @param  none
  * @retval none
  */
void vInitBlueTooth(BT_TypeDef* Bt)
{
	if(Bt != NULL)
	{	
		/* Load the pointer vector with the AT comand adresses */
		Bt->pData[SendAt] 		= CmdAt1;
		Bt->pData[SetConfOrgl] 	= CmdAt2;
		Bt->pData[SetUART] 		= CmdAt3;
		Bt->pData[SetName] 		= CmdAt4;
		Bt->pData[SetPswd] 		= CmdAt5;
		Bt->pData[SetRole] 		= CmdAt6;
		Bt->pData[SetCmode] 	= CmdAt7;
		Bt->pData[InitBluetooth] = CmdAt8;
		
		/* inform the size of the contents pointed */
		Bt->LengData[SendAt] 	 	= (sizeof(CmdAt1)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetConfOrgl] 	= (sizeof(CmdAt2)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetUART] 		= (sizeof(CmdAt3)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetName] 		= (sizeof(CmdAt4)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetPswd] 		= (sizeof(CmdAt5)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetRole] 		= (sizeof(CmdAt6)/sizeof(uint8_t)) - 1;
		Bt->LengData[SetCmode] 		= (sizeof(CmdAt7)/sizeof(uint8_t)) - 1;
		Bt->LengData[InitBluetooth] 	= (sizeof(CmdAt8)/sizeof(uint8_t)) - 1;
		
		/* Load the function pointer vector */			
		Bt->pFuncBt[SendCmdBt] 	= vSendCmdBt;
		Bt->pFuncBt[WaitCpltBt] 	= vWaitCpltBt;
		Bt->pFuncBt[NextBt	]	= vNextCommandBt;
		Bt->pFuncBt[FinishBt]	= vFinishBt;		
		
		/* Load the first routine to run and set the state */		
		Bt->Index 	 = 0;
		Bt->TimeOut	 = 0;		
		Bt->State = SendCmdBt;
		pInitBt 	= Bt->pFuncBt[Bt->State];

		for(;;) 
		{
			/* Function pointer of application */
			(*pInitBt)();	
			
			if(pInitBt == vFinishBt)
			{
				break;
			}
		}
	}
	else
	{
		return;
	}
}

#endif



 /**
  * @brief: Milliseconds delay
  * @param  value of delay
  * @retval none
  */
void TIM6delay_ms(uint16_t value)
{
	TIM6->PSC = prescaler_ms;
	TIM6->ARR = value;
	TIM6->CNT = 0;
	TIM6->CR1 |= TIM_CR1_CEN;
	while((TIM6->SR & TIM_SR_UIF)==0){}
	TIM6->SR &=~ TIM_SR_UIF;
}


 /**
  * @brief: Milliseconds delay
  * @param  value of delay
  * @retval none
  */
void TIM6delay_us(uint16_t value)
{
	TIM6->PSC = prescaler_us;
	TIM6->ARR = value;
	TIM6->CNT = 0;
	TIM6->CR1 |= TIM_CR1_CEN;
	while((TIM6->SR & TIM_SR_UIF)==0){}
	TIM6->SR &=~ TIM_SR_UIF;
}



 /**
  * @brief: Microseconds delay
  * @param  value of delay 
  * @retval none
  */
void InitDelayTIM6(void)
{
	prescaler_ms =  SystemCoreClock / 1000-1; 
	prescaler_us =  SystemCoreClock / 1000000-1;
	RCC->APB1ENR |=  RCC_APB1ENR_TIM6EN;
}



 /**
  * @brief: Read a byte from LCD display
  * @param  none 
  * @retval uint8_t
  */
uint8_t LCD_ReadByte(void)
{
	uint8_t ReadedData=0;
	GPIO_InitTypeDef GPIO_Conf;
	
	HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_SET);

	GPIO_Conf.Pin 		= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;	
	GPIO_Conf.Mode 	= GPIO_MODE_INPUT;
	GPIO_Conf.Pull 	= GPIO_NOPULL;
	GPIO_Conf.Speed 	= GPIO_SPEED_LOW;
	HAL_GPIO_Init(LCD_PORT, &GPIO_Conf);

	HAL_GPIO_WritePin(LCD_PORT, LCD_RW, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);

	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D4))
		ReadedData |= 0x80;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D5))
		ReadedData |= 0x40;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D6))
		ReadedData |= 0x20;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D7))
		ReadedData |= 0x10;


	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
	TIM6delay_us(50);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
	
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D4))
		ReadedData |= 0x08;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D5))
		ReadedData |= 0x04;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D6))
		ReadedData |= 0x02;
	if(HAL_GPIO_ReadPin(LCD_PORT, LCD_D7))
		ReadedData |= 0x01;
	
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);

	GPIO_Conf.Pin = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIO_Conf.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Conf.Pull = GPIO_NOPULL;
	GPIO_Conf.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(LCD_PORT, &GPIO_Conf);

	return ReadedData;
}



 /**
  * @brief: Send a byte to LCD display
  * @param  value to be sent 
  * @retval none
  */
void LCD_SendByte(uint8_t cmd)
{
	uint8_t tcmd = 0;

	HAL_GPIO_WritePin(LCD_PORT, LCD_RW, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);

	tcmd = cmd >> 4;	
	if( tcmd & 0x01 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_SET);
	if( tcmd & 0x02 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
	if( tcmd & 0x04 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
	if( tcmd & 0x08 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_SET);
	

	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
	TIM6delay_us(50);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_RESET);

	cmd &= 0x0F;	
	if( cmd & 0x01 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D7, GPIO_PIN_SET);
	if( cmd & 0x02 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
	if( cmd & 0x04 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D5, GPIO_PIN_SET);
	if( cmd & 0x08 )
		HAL_GPIO_WritePin(LCD_PORT, LCD_D4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_D_ALL, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
	while(LCD_ReadByte() & 0x80);
}



 /**
  * @brief: Send a command to LCD display
  * @param  command to be sent 
  * @retval none
  */
void LCD_SendCmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
	TIM6delay_us(50);
	LCD_SendByte(cmd);
}



 /**
  * @brief: Send a data to LCD display
  * @param  data to be sent 
  * @retval none
  */
void LCD_SendData(uint32_t data)
{
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_SET);
	TIM6delay_us(50);
	LCD_SendByte(data);
}



 /**
  * @brief: Print a number between 0 and 9999.
  * @param  value to be printed 
  * @retval none
  */
void LCD_Num(uint32_t x)
{
	LCD_SendData((x/1000) + 0x30);
	LCD_SendData((x/100)%10 + 0x30);
	LCD_SendData((x%100)/10 + 0x30);
	LCD_SendData((x%10) + 0x30);
} 



 /**
  * @brief: Print a string on LCD display
  * @param  String to be printed 
  * @retval none
  */
void LCD_SendText(char *text)
{
	while(*text)
	{
		LCD_SendData(*text);
		text++;
	}
}



 /**
  * @brief: Clear the display.
  * @param  value to be printed 
  * @retval none
  */
void LCD_Clear(void)
{
	LCD_SendCmd(0x01);
}



 /**
  * @brief: Set the cursor position
  * @param  column and line 
  * @retval none
  */
void LCD_GoTo(unsigned char column, unsigned char line)
{
	uint8_t position = 0;	
	
	switch(line)
	{
		case 0:	position = 0x00;
				break;
		case 1:	position = 0x40;
				break;
		case 2:	position = 0x10;
				break;
		case 3:	position = 0x50;
				break;		
		default: 	position = 0x00;
				break;
	}
	LCD_SendCmd(0x80 | (position + column));
}



 /**
  * @brief: Initialize the LCD display.
  * @param  none 
  * @retval none
  */
void LCD_Init(void)
{
	uint8_t i;
	char const *p;

	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* GPIO Port Clock Enable */
	__GPIOC_CLK_ENABLE();
	InitDelayTIM6();
	
	/*Configure GPIO pins */
	GPIO_InitStructure.Pin 	= LCD_RS	|
						  LCD_RW	|
						  LCD_EN	|
						  LCD_D4	|
						  LCD_D5	|
						  LCD_D6	|
						  LCD_D7;
	GPIO_InitStructure.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull 	= GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(LCD_PORT, &GPIO_InitStructure);

	TIM6delay_ms(30);
	HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
	
	for(i = 0; i<3; i++)
	{
		HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(LCD_PORT, LCD_D6 | LCD_D7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_PORT, LCD_D5 | LCD_D4, GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_SET);
	TIM6delay_ms(50);
	
	HAL_GPIO_WritePin(LCD_PORT, LCD_D6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_PORT,LCD_D4 | LCD_D5 | LCD_D7, GPIO_PIN_RESET);	
	
	
	TIM6delay_ms(50);
	HAL_GPIO_WritePin(LCD_PORT, LCD_EN, GPIO_PIN_RESET);
  
	LCD_SendCmd(0x28); /* 2 lines, 5x8 character matrix      	*/
	LCD_SendCmd(0x08); /* Turn off display					*/
	LCD_SendCmd(0x01); /* Clean display with home cursor 		*/
	LCD_SendCmd(0x06); /* Entry mode: Move right, no shift   	*/
	LCD_SendCmd(0x0C); /* Display ctrl:Disp=ON,Curs/Blnk=OFF 	*/
	
	TIM6delay_ms(5);

	/* Load user-specific characters into CGRAM                               */
	LCD_SendCmd(0x40);                  /* Set CGRAM address counter to 0     */
	p = &UserFont[0][0];
	for (i = 0; i < sizeof(UserFont); i++, p++)
		LCD_SendData (*p);
	LCD_SendCmd(0x80);                 /* Set DDRAM address counter to 0     */
}



 /**
  * @brief: Initialize the LCD display.
  * @param  none 
  * @retval none
  */
void GetValue(void)
{
	TotenADC.ADCValue = HAL_ADC_GetValue(&hadc);	
}



