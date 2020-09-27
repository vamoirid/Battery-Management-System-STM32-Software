/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define LTC6811_CS_Pin GPIO_PIN_13
#define LTC6811_CS_GPIO_Port GPIOC
#define AIRS__Pin GPIO_PIN_0
#define AIRS__GPIO_Port GPIOC
#define AIRS_C1_Pin GPIO_PIN_1
#define AIRS_C1_GPIO_Port GPIOC
#define AIRS_Out_Pin GPIO_PIN_2
#define AIRS_Out_GPIO_Port GPIOC
#define Idc_Pin GPIO_PIN_3
#define Idc_GPIO_Port GPIOC
#define DCDC_Thermistor_Pin GPIO_PIN_0
#define DCDC_Thermistor_GPIO_Port GPIOA
#define Thermistor1_Pin GPIO_PIN_1
#define Thermistor1_GPIO_Port GPIOA
#define Thermistor2_Pin GPIO_PIN_2
#define Thermistor2_GPIO_Port GPIOA
#define Thermistor3_Pin GPIO_PIN_3
#define Thermistor3_GPIO_Port GPIOA
#define HV_Out_Pin GPIO_PIN_5
#define HV_Out_GPIO_Port GPIOC
#define HV__Pin GPIO_PIN_0
#define HV__GPIO_Port GPIOB
#define HV_B1_Pin GPIO_PIN_1
#define HV_B1_GPIO_Port GPIOB
#define Precharge_State_Pin GPIO_PIN_12
#define Precharge_State_GPIO_Port GPIOB
#define AdAdct_State_Pin GPIO_PIN_13
#define AdAdct_State_GPIO_Port GPIOB
#define IMD_State_Pin GPIO_PIN_14
#define IMD_State_GPIO_Port GPIOB
#define BMS_State_Pin GPIO_PIN_15
#define BMS_State_GPIO_Port GPIOB
#define Digital_Input2_Pin GPIO_PIN_6
#define Digital_Input2_GPIO_Port GPIOC
#define Digital_Input1_Pin GPIO_PIN_7
#define Digital_Input1_GPIO_Port GPIOC
#define BMS_State_Out_Pin GPIO_PIN_12
#define BMS_State_Out_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
