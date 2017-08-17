/*** GPIO A ***/
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USER_BUTTON_Pin 		GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA


/*** GPIO B ***/
#define RELE_1_Pin 			GPIO_PIN_9
#define RELE_1_GPIO_Port 	GPIOB
#define RELE_2_Pin 			GPIO_PIN_8
#define RELE_2_GPIO_Port 	GPIOB


/*** GPIO C ***/
#define BLUE_LED_Pin 		GPIO_PIN_8
#define BLUE_LED_GPIO_Port 	GPIOC
#define GREEN_LED_Pin 		GPIO_PIN_9
#define GREEN_LED_GPIO_Port 	GPIOC
#define CTL_WIFI_Pin 		GPIO_PIN_10
#define CTL_WIFI_GPIO_Port 	GPIOC
#define CTL_USB_Pin 		GPIO_PIN_11
#define CTL_USB_GPIO_Port 	GPIOC

/*** LCD pins ***/
#define LCD_RS          GPIO_PIN_13
#define LCD_RW          GPIO_PIN_14
#define LCD_EN          GPIO_PIN_15 
#define LCD_D4          GPIO_PIN_3
#define LCD_D5          GPIO_PIN_2
#define LCD_D6		    GPIO_PIN_1
#define LCD_D7          GPIO_PIN_0
#define LCD_PORT        GPIOC        
#define LCD_D_ALL	    (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)


/*** GPIO D ***/
#define CTL_BT_Pin 			GPIO_PIN_2
#define CTL_BT_GPIO_Port 	GPIOD
