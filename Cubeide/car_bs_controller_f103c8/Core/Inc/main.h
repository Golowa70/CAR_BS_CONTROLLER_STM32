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
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define LCD_RESET_Pin GPIO_PIN_3
#define LCD_RESET_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define BUTTON_ENTER_Pin GPIO_PIN_3
#define BUTTON_ENTER_GPIO_Port GPIOB
#define BUTTON_ESC_Pin GPIO_PIN_4
#define BUTTON_ESC_GPIO_Port GPIOB
#define BUTTON_UP_Pin GPIO_PIN_5
#define BUTTON_UP_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_6
#define BUTTON_DOWN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LED_BLINK_DELAY 		500


//modes
#define OFF_MODE               0
#define AUTO_MODE              1
#define ON_MODE                2

//resistive sensor
#define MIN_RESISTANCE          10
#define MAX_RESISTANCE          240

//MENU
#define MENU_SETPOINTS_NUM_ITEMS        25    // кол-во пунктов меню настроек
#define MENU_PARAM_VIEW_NUM_ITEMS       15    // кол-во пунктов меню просмотра параметров

#define LCD_LINE_SPACER                 4  // промежуток между строками
#define LCD_FONT_HIGHT                  8   // высота шрифта
#define ITEM_MAX_CHARS                  17  // макс. количество символов в названии пункта

#define BLINK_INTERVAL					500

#define BTN_NOTHING  			0b00000000
#define BTN_UP    				0b00000001
#define BTN_DOWN				0b00000010
#define BTN_UP_LONG_PRESS    	0b00000100
#define BTN_DOWN_LONG_PRESS		0b00001000
#define BTN_ENTER				0b00010000
#define BTN_ESC  				0b00100000
#define BTN_ENTER_LONG_PRESS    0b01000000
#define BTN_ESC_LONG_PRESS		0b10000000


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

#define PUMP_T_OFF_MIN			3
#define PUMP_T_OFF_MAX			20

#define PUMP_OUT_MODE_MIN		0
#define PUMP_OUT_MODE_MAX		2


/*
#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define

#define
#define
*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
