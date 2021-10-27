/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define PRX_SENSOR_DIN_Pin GPIO_PIN_13
#define PRX_SENSOR_DIN_GPIO_Port GPIOC
#define SUPPLY_VOLTAGE_AIN_Pin GPIO_PIN_0
#define SUPPLY_VOLTAGE_AIN_GPIO_Port GPIOC
#define SENSORS_VOLTAGE_AIN_Pin GPIO_PIN_1
#define SENSORS_VOLTAGE_AIN_GPIO_Port GPIOC
#define RESIST_SENSOR_AIN_Pin GPIO_PIN_2
#define RESIST_SENSOR_AIN_GPIO_Port GPIOC
#define SENSORS_CURRENT_AIN_Pin GPIO_PIN_3
#define SENSORS_CURRENT_AIN_GPIO_Port GPIOC
#define WKUP_DIN_Pin GPIO_PIN_0
#define WKUP_DIN_GPIO_Port GPIOA
#define DOOR_SWITCH_DIN_Pin GPIO_PIN_1
#define DOOR_SWITCH_DIN_GPIO_Port GPIOA
#define PUMP_CURRENT_AIN_Pin GPIO_PIN_2
#define PUMP_CURRENT_AIN_GPIO_Port GPIOA
#define FRIDGE_CURRENT_AIN_Pin GPIO_PIN_3
#define FRIDGE_CURRENT_AIN_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define SI4421_CS_Pin GPIO_PIN_4
#define SI4421_CS_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_5
#define LCD_RST_GPIO_Port GPIOC
#define LCD_BLA_Pin GPIO_PIN_0
#define LCD_BLA_GPIO_Port GPIOB
#define SI4421_IRQ_Pin GPIO_PIN_1
#define SI4421_IRQ_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOB
#define ONE_WIRE_Pin GPIO_PIN_10
#define ONE_WIRE_GPIO_Port GPIOB
#define EXP_MODULE_CS_Pin GPIO_PIN_11
#define EXP_MODULE_CS_GPIO_Port GPIOB
#define W25Q32_CS_Pin GPIO_PIN_12
#define W25Q32_CS_GPIO_Port GPIOB
#define BUTTON_ENTER_Pin GPIO_PIN_6
#define BUTTON_ENTER_GPIO_Port GPIOC
#define BUTTON_DOWN_Pin GPIO_PIN_7
#define BUTTON_DOWN_GPIO_Port GPIOC
#define BUTTON_UP_Pin GPIO_PIN_8
#define BUTTON_UP_GPIO_Port GPIOC
#define BUTTON_ESC_Pin GPIO_PIN_9
#define BUTTON_ESC_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_8
#define STATUS_LED_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define IGNITION_DIN_Pin GPIO_PIN_10
#define IGNITION_DIN_GPIO_Port GPIOC
#define SENS_SUPPLY_EN_OUT_Pin GPIO_PIN_11
#define SENS_SUPPLY_EN_OUT_GPIO_Port GPIOC
#define MAIN_SUPPLY_EN_OUT_Pin GPIO_PIN_12
#define MAIN_SUPPLY_EN_OUT_GPIO_Port GPIOC
#define CONVERTER_OUT_Pin GPIO_PIN_2
#define CONVERTER_OUT_GPIO_Port GPIOD
#define FRIDGE_OUT_Pin GPIO_PIN_4
#define FRIDGE_OUT_GPIO_Port GPIOB
#define WATER_PUMP_OUT_Pin GPIO_PIN_5
#define WATER_PUMP_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
