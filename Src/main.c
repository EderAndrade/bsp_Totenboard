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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "TotenBoard.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
  /* These are the objects available to use. Look below
	to see some the use of them.
	
	TotenGpio[LED_BLUE].
	TotenGpio[LED_GREEN].
	TotenInitHW
	TotenUart
	TotenLCD
 	TotenADC	
	TotenLCD
	TotenGpioUSER
	
  */
  
	/* Init. all peripherals used by BSP */
	TotenInitHW.TotenPeripInit();  
	
	/* LCD init. */
	TotenLCD.Init();

	/* Print on LCD */
	TotenLCD.Clear();
	TotenLCD.XY(0,0);
	TotenLCD.Text("***Totenboard***");
	TotenLCD.XY(1,2); 
	TotenLCD.Text("*BSP running!*");  
	TotenLCD.XY(6,3); 
	TotenLCD.Text("2017"); 

#ifdef INIT_BLUETOOTH_CONFIG
	
	/* Set the message to send */
	uint8_t msg[] = {"TotenBoard!\r\n"};
	
	/* Enable Bluetooth channel before do use it */
	TotenGpio[USB].ResePin (CTL_USB_Pin);
	TotenGpio[BT].SetPin   (CTL_BT_Pin);
	TotenGpio[WiFi].ResePin(CTL_WIFI_Pin); 
	
	vInitBlueTooth(&BtData);
  
#endif
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
	for(;;)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		#ifdef INIT_BLUETOOTH_CONFIG
		
			/* Sending a message if the init. config. is enabled */
			TotenUart.SendReceiveData(BT, SEND_UART, msg, (sizeof(msg) / sizeof(uint8_t) - 1));
		#endif
			
		HAL_Delay(1000);	

		/* Blink LED @Cucu */
		TotenGpio[LED_BLUE].TogglePin(BLUE_LED_Pin);
		TotenGpio[LED_GREEN].TogglePin(GREEN_LED_Pin);	
		
		/* Just read the ADC channel */
		TotenADC.GetValue();
		
		/* Read the state os USER button - without debouncing */
		TotenGpioUSER.ReadPin(USER_BUTTON_Pin);
		
	}
  /* USER CODE END 3 */

}


/* USER CODE BEGIN 4 */
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType 	= RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState 			= RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 		= RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL 		= RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV 		= RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType 		= RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider 		= RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider 	= RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void HAL_SYSTICK_Callback(void)
{
	
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
