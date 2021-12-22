/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define W25Q_CS_Pin GPIO_PIN_13
#define W25Q_CS_GPIO_Port GPIOC
#define LCD_LED_Pin GPIO_PIN_14
#define LCD_LED_GPIO_Port GPIOC
#define RES_SENS_Pin GPIO_PIN_0
#define RES_SENS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define BAT_VOLT_Pin GPIO_PIN_2
#define BAT_VOLT_GPIO_Port GPIOA
#define LCD_RESET_Pin GPIO_PIN_3
#define LCD_RESET_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define SENS_VOLT_Pin GPIO_PIN_1
#define SENS_VOLT_GPIO_Port GPIOB
#define SENS_SUPPLY_Pin GPIO_PIN_2
#define SENS_SUPPLY_GPIO_Port GPIOB
#define ONE_WIRE_Pin GPIO_PIN_10
#define ONE_WIRE_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOB
#define CONV_OUTPUT_Pin GPIO_PIN_12
#define CONV_OUTPUT_GPIO_Port GPIOB
#define PUMP_OUTPUT_Pin GPIO_PIN_13
#define PUMP_OUTPUT_GPIO_Port GPIOB
#define DOOR_INPUT_Pin GPIO_PIN_14
#define DOOR_INPUT_GPIO_Port GPIOB
#define PRX_SENS_INPUT_Pin GPIO_PIN_15
#define PRX_SENS_INPUT_GPIO_Port GPIOB
#define IGN_INPUT_Pin GPIO_PIN_8
#define IGN_INPUT_GPIO_Port GPIOA
#define BUTTON_DOWN_Pin GPIO_PIN_15
#define BUTTON_DOWN_GPIO_Port GPIOA
#define BUTTON_ENTER_Pin GPIO_PIN_3
#define BUTTON_ENTER_GPIO_Port GPIOB
#define BUTTON_ESC_Pin GPIO_PIN_4
#define BUTTON_ESC_GPIO_Port GPIOB
#define BUTTON_UP_Pin GPIO_PIN_5
#define BUTTON_UP_GPIO_Port GPIOB
#define MAIN_SUPPLY_Pin GPIO_PIN_8
#define MAIN_SUPPLY_GPIO_Port GPIOB
#define FRIDGE_OUTPUT_Pin GPIO_PIN_9
#define FRIDGE_OUTPUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LED_BLINK_DELAY 			500
#define PRX_SENSOR_FEEDBACK_DELAY	10
#define MESSAGE_VIEW_TIME			1000

//MODES
#define OFF_MODE               	0
#define AUTO_MODE              	1
#define ON_MODE                	2

//MENU
#define MENU_SETPOINTS_NUM_ITEMS        25    // кол-во пунктов меню настроек
#define MENU_PARAM_VIEW_NUM_ITEMS       15    // кол-во пунктов меню просмотра параметров

//LCD
#define LCD_LINE_SPACER                 4  // промежуток между строками
#define LCD_FONT_HIGHT                  8   // высота шрифта
#define ITEM_MAX_CHARS                  17  // макс. количество символов в названии пункта
#define BLINK_INTERVAL					500

//BUTTONS
#define BTN_NOTHING  			0b00000000
#define BTN_UP    				0b00000001
#define BTN_DOWN				0b00000010
#define BTN_UP_LONG_PRESS    	0b00000100
#define BTN_DOWN_LONG_PRESS		0b00001000
#define BTN_ENTER				0b00010000
#define BTN_ESC  				0b00100000
#define BTN_ENTER_LONG_PRESS    0b01000000
#define BTN_ESC_LONG_PRESS		0b10000000

//MEMORY
#define FLASH_SECTOR_SIZE				4096
#define SETPOINTS_FLASH_SECTOR			10
#define DS18B20_IDS_FLASH_SECTOR		11
#define IMAGE_BUFFER_SIZE				1024
#define IMAGE_LOGO_FK					FLASH_SECTOR_SIZE*1		//64x55
#define IMAGE_LOGO_2					FLASH_SECTOR_SIZE*2		//64x55
#define IMAGE_LOGO_3					FLASH_SECTOR_SIZE*3		//64x55
#define IMAGE_LOGO_4					FLASH_SECTOR_SIZE*4		//64x55
#define IMAGE_WATER_LEVEL				FLASH_SECTOR_SIZE*5		//50x50

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//SETPOINTS RANGES
#define PUMP_T_OFF_MIN				3		//sec
#define PUMP_T_OFF_MAX				20

#define PUMP_OUT_MODE_MIN			0
#define PUMP_OUT_MODE_MAX			2

#define CONV_U_OFF_MIN				80
#define	CONV_U_OFF_MAX				150 	// V*10

#define CONV_T_U_OFF_MIN			1
#define	CONV_T_U_OFF_MAX			240 	//sec

#define CONV_U_ON_MIN				100
#define	CONV_U_ON_MAX				180 	// V*10

#define CONV_T_IGN_OFF_MIN			1
#define	CONV_T_IGN_OFF_MAX			240 	//min

#define CONV_OUT_MODE_MIN			0
#define CONV_OUT_MODE_MAX			2

#define FRIDGE_U_OFF_MIN			80
#define	FRIDGE_U_OFF_MAX			150 	// V*10

#define FRIDGE_T_U_OFF_MIN			1
#define	FRIDGE_T_U_OFF_MAX			240 	//sec

#define FRIDGE_U_ON_MIN				100
#define	FRIDGE_U_ON_MAX				180 	// V*10

#define FRIDGE_T_IGN_OFF_MIN		1
#define	FRIDGE_T_IGN_OFF_MAX		240 	//min

#define FRIDGE_TEMP_ON_MIN			5
#define	FRIDGE_TEMP_ON_MAX			30		// C

#define FRIDGE_TEMP_OFF_MIN			1
#define	FRIDGE_TEMP_OFF_MAX			28		// C

#define FRIDGE_OUT_MODE_MIN			0
#define FRIDGE_OUT_MODE_MAX			3

#define	WATER_SENS_MIN_MIN			0
#define WATER_SENS_MIN_MAX			240		// R

#define	WATER_SENS_MAX_MIN			0
#define WATER_SENS_MAX_MAX			240		// R

#define	WATER_SENS_CORR_MIN			0
#define WATER_SENS_CORR_MAX			255

#define	WATER_TANK_CAP_MIN			5
#define WATER_TANK_CAP_MAX			100  	// L

#define	SHUTDOWN_DELAY_MIN			1
#define	SHUTDOWN_DELAY_MAX			8		// h

#define	U_CORR_MIN					0
#define	U_CORR_MAX					255

#define	LIGHT_T_OFF_MIN				0
#define	LIGHT_T_OFF_MAX				60	//sec

#define	LOGO_MIN					0
#define	LOGO_MAX					2

#define	INSIDE_SENS_ID_MIN			0
#define	INSIDE_SENS_ID_MAX			2

#define	OUTSIDE_SENS_ID_MIN			0
#define	OUTSIDE_SENS_ID_MAX			2

#define	FRIDGE_SENS_ID_MIN			0
#define	FRIDGE_SENS_ID_MAX			2

//ADC
#define ADC_CHANELS					3
#define ADC_REFERENCE			 	0.0008      //3.3/4095=0.0008; 18.46*0.0008=0.01487
#define ADC_BAT_VOLT_DIVIDER		11			//R1=100k, R2=10k; Vin=36 Vout=3.27; 36/3.27=11;
#define ADC_SENS_VOLT_DIVIDER		1//3.25		//R1=27k, R2=12k; Vin=10 Vout=3.08; 10/3.08=3.246;
#define	ADC_RES_SENS_DIVIDER		100

#define EMA_FILTER_K				0.1  		//коэффициент сглаживания от 0 до 1 чем меньше, тем плавнее фильтр

#define TEMP_SENS_UPDATE_PERIOD		1000		//ms

//ERRORS
#define ERROR_NOTHING  			0b00000000
#define ERROR_TEM_SENS  		0b00000001
#define ERROR_SENS_SUPPLY  		0b00000010
#define ERROR_RES_SENS  		0b00000100
#define ERROR_FLASH_READ  		0b00001000

#define MAGIC_KEY				0x1234

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
