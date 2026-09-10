/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_LED_Pin GPIO_PIN_3
#define MCU_LED_GPIO_Port GPIOC
#define LM_ENC_CHA_Pin GPIO_PIN_0
#define LM_ENC_CHA_GPIO_Port GPIOA
#define LM_ENC_CHB_Pin GPIO_PIN_1
#define LM_ENC_CHB_GPIO_Port GPIOA
#define IMU_NCS_Pin GPIO_PIN_4
#define IMU_NCS_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define DRV_STBY_Pin GPIO_PIN_14
#define DRV_STBY_GPIO_Port GPIOB
#define RM_PWM_INA_Pin GPIO_PIN_6
#define RM_PWM_INA_GPIO_Port GPIOC
#define RM_PWM_INB_Pin GPIO_PIN_7
#define RM_PWM_INB_GPIO_Port GPIOC
#define LM_PWM_INA_Pin GPIO_PIN_8
#define LM_PWM_INA_GPIO_Port GPIOC
#define LM_PWM_INB_Pin GPIO_PIN_9
#define LM_PWM_INB_GPIO_Port GPIOC
#define RM_ENC_CHA_Pin GPIO_PIN_8
#define RM_ENC_CHA_GPIO_Port GPIOA
#define RM_ENC_CHB_Pin GPIO_PIN_9
#define RM_ENC_CHB_GPIO_Port GPIOA
#define MUX_SCL_Pin GPIO_PIN_6
#define MUX_SCL_GPIO_Port GPIOB
#define MUX_SDA_Pin GPIO_PIN_7
#define MUX_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
