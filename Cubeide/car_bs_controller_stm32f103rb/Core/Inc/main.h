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
#define PRX_SENSOR_IN_Pin GPIO_PIN_13
#define PRX_SENSOR_IN_GPIO_Port GPIOC
#define SUPPLY_VOLTAGE_IN_Pin GPIO_PIN_0
#define SUPPLY_VOLTAGE_IN_GPIO_Port GPIOC
#define SENSORS_VOLTAGE_IN_Pin GPIO_PIN_1
#define SENSORS_VOLTAGE_IN_GPIO_Port GPIOC
#define RESIST_SENSOR_IN_Pin GPIO_PIN_2
#define RESIST_SENSOR_IN_GPIO_Port GPIOC
#define SENSORS_CURRENT_IN_Pin GPIO_PIN_3
#define SENSORS_CURRENT_IN_GPIO_Port GPIOC
#define IGNITION_IN_Pin GPIO_PIN_0
#define IGNITION_IN_GPIO_Port GPIOA
#define DOOR_SWITCH_IN_Pin GPIO_PIN_1
#define DOOR_SWITCH_IN_GPIO_Port GPIOA
#define PUMP_CURRENT_IN_Pin GPIO_PIN_2
#define PUMP_CURRENT_IN_GPIO_Port GPIOA
#define FRIDGE_CURRENT_IN_Pin GPIO_PIN_3
#define FRIDGE_CURRENT_IN_GPIO_Port GPIOA
#define CS_LCD_Pin GPIO_PIN_4
#define CS_LCD_GPIO_Port GPIOA
#define CS_SI4421_Pin GPIO_PIN_4
#define CS_SI4421_GPIO_Port GPIOC
#define RST_LCD_Pin GPIO_PIN_5
#define RST_LCD_GPIO_Port GPIOC
#define BLA_LCD_Pin GPIO_PIN_0
#define BLA_LCD_GPIO_Port GPIOB
#define IRQ_SI4421_Pin GPIO_PIN_1
#define IRQ_SI4421_GPIO_Port GPIOB
#define W_WATER_LEVEL_IN_Pin GPIO_PIN_2
#define W_WATER_LEVEL_IN_GPIO_Port GPIOB
#define ONE_WIRE_Pin GPIO_PIN_10
#define ONE_WIRE_GPIO_Port GPIOB
#define CS_EXP_MODULE_Pin GPIO_PIN_11
#define CS_EXP_MODULE_GPIO_Port GPIOB
#define CS_W25Q32_Pin GPIO_PIN_12
#define CS_W25Q32_GPIO_Port GPIOB
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
#define SENS_SUPPLY_OUT_Pin GPIO_PIN_11
#define SENS_SUPPLY_OUT_GPIO_Port GPIOC
#define MAIN_SUPPLY_OUT_Pin GPIO_PIN_12
#define MAIN_SUPPLY_OUT_GPIO_Port GPIOC
#define CONVERTER_OUT_Pin GPIO_PIN_2
#define CONVERTER_OUT_GPIO_Port GPIOD
#define FRIDGE_OUT_Pin GPIO_PIN_4
#define FRIDGE_OUT_GPIO_Port GPIOB
#define PUMP_OUT_Pin GPIO_PIN_5
#define PUMP_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
