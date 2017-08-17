
#ifndef __TOTENBOARD_H
#define __TOTENBOARD_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

/**
  * @defines
  */
  
/* GPIO defines */
#define NUM_OBJS  	(unsigned char)6	

/* UART defines */
#define SIZE_BUFFER	 (unsigned char)64
#define RECEIVE_UART (unsigned char)0
#define SEND_UART	 (unsigned char)1


/* Bluetooth defines */
#define NUM_FUNC_BT		(unsigned char)4
#define INIT_CPLT_BT	(unsigned char)7	
#define RX_BUFFER_SIZE 	(unsigned char)10
	
#define _2s	(unsigned int)2000
#define _4s	(unsigned int)4000

/* If you want to use the routines to configure your HC-05, then discomment this line */
//#define INIT_BLUETOOTH_CONFIG
	

/**
  * @enum
  */	
enum
{
	LED_BLUE = 0,
	LED_GREEN	,
	USB		,
	WiFi		,
	BT		,
	LCD		,
	
};



#ifdef INIT_BLUETOOTH_CONFIG
enum
{
	SendAt		= 0,	
	SetConfOrgl	,
	SetUART		,
	SetName		,
	SetPswd		,
	SetRole		,
	SetCmode		,
	InitBluetooth	
};

enum
{
	SendCmdBt = 0,
	WaitCpltBt,
	NextBt	,
	FinishBt	
};
#endif


 /**
  * @Pointers
  */
#ifdef INIT_BLUETOOTH_CONFIG
typedef void(*pFunc)			(void);
#endif

/* GPIO */
typedef void(*pFuncInitHW)	(void);
typedef void(*pFuncSetPin)	(uint16_t Pin);
typedef void(*pFuncResetPin)	(uint16_t Pin);
typedef void(*pFuncTogglePin)	(uint16_t Pin);
typedef void(*pFuncReadPin)	(uint16_t Pin);

/* UART */
typedef void(*pUart)		(const 	unsigned char device,
								unsigned char operation,
								unsigned char *p, 
								unsigned int size);										  
/* LCD */
typedef void(*pFuncLCD_Init)	(void);
typedef void(*pFuncLCD_XY)	(unsigned char column, unsigned char line);
typedef void(*pFuncLCD_Clear)	(void);
typedef void(*pFuncLCD_Text)	(char *text);
typedef void(*pFuncLCD_Num)	(uint32_t x);										  

/* ADC */										  
typedef void(*pFuncADC)		(void);
										  
										  

/**
  * @struct
  */
  
/* GPIO struct - out */  
typedef struct
{
	pFuncSetPin	SetPin;	
	pFuncResetPin 	ResePin;	
	pFuncTogglePin TogglePin;
	
}Toten_GPIO_OutTypeDef;



/* GPIO struct - in */
typedef struct
{	
	pFuncReadPin	ReadPin;	
	GPIO_PinState	PinState;
		
}Toten_GPIO_InTypeDef;



/* Peripherals struct */
typedef struct
{
	pFuncInitHW 	TotenPeripInit;
	
}Toten_PeriphTypeDef;



/* UART struct */
typedef struct
{
	pUart			 SendReceiveData;	
	HAL_StatusTypeDef	 StatusUart;
	
}Toten_UART_InitTypeDef;



/* Bluetooth struct */
#ifdef INIT_BLUETOOTH_CONFIG

typedef struct
{
	uint8_t 	*pData[8];
	uint8_t 	LengData[8];
	uint8_t	State;
	uint8_t 	Index;
	uint32_t 	TimeOut;
	pFunc	pFuncBt[NUM_FUNC_BT];	
}BT_TypeDef;

#endif



/* LCD struct */
typedef struct
{
	pFuncLCD_Init	Init;
	pFuncLCD_XY	XY;   
	pFuncLCD_Clear	Clear;
	pFuncLCD_Text	Text;
	pFuncLCD_Num	Num;
	
}LCD_TypeDef;



/* ADC struct */
typedef struct
{
	pFuncADC GetValue;
	uint32_t ADCValue;
	
}AD_TypeDef;


/**
  * @Extern
  */
								
extern Toten_GPIO_OutTypeDef 	TotenGpio[NUM_OBJS];
extern Toten_GPIO_InTypeDef 	TotenGpioUSER;
extern Toten_PeriphTypeDef	TotenInitHW;
extern Toten_UART_InitTypeDef TotenUart;
extern UART_HandleTypeDef 	huart2;
extern LCD_TypeDef 			TotenLCD;
extern AD_TypeDef 			TotenADC;

#ifdef INIT_BLUETOOTH_CONFIG
	extern pFunc 			pInitBt;
	extern BT_TypeDef 		BtData;								
#endif


/**
  * @Prototypes
  */

void 			TotenInitHWs	(void);
void 			SetPin		(uint16_t Pin);
void 			ResetPin		(uint16_t Pin);
void 			TogglePin		(uint16_t Pin);
void		 		ReadPin		(uint16_t Pin);

void 			SendReceiveDataUart(const unsigned char device, 
								unsigned char operation, 
								unsigned char *p, 
								unsigned int size);

unsigned char 		LCD_ReadByte	(void);
void 			LCD_SendByte	(unsigned char cmd); 
void 			LCD_SendCmd 	(unsigned char cmd); 
void 			LCD_SendData	(uint32_t data);
void 			LCD_SendText	(char *text);
void 			LCD_GoTo 		(unsigned char line, unsigned char column);
void 			LCD_Clear 	(void);
void 			LCD_Init 		(void);
void 			LCD_Num		(uint32_t x);
								
void 			InitDelayTIM6	(void);
void 			TIM6delay_ms	(uint16_t value);
void 			TIM6delay_us	(uint16_t value);
								
void 			GetValue		(void);							
								
								
#ifdef INIT_BLUETOOTH_CONFIG
								
void 			vSendCmdBt	(void);
void 			vWaitCpltBt	(void);
void 			vFinishBt		(void);
void 			vNextCommandBt	(void);
void 			vInitBlueTooth	(BT_TypeDef* Bt);
								
#endif		

#endif //#ifndef __TOTENBOARD_H
