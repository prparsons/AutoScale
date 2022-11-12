/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
typedef uint8_t bool;
#define nullptr 0

enum LED_STATUS {
   LED_INIT_SUCCESS,
   LED_SAMPLE_SUCCESS,
   LED_SAMPLE_FAIL,
   LED_I2C_FAIL
};



void timerDelay(uint16_t u16ths);
void statusBlink(enum LED_STATUS code);
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
#define ST0_M2_Pin GPIO_PIN_13
#define ST0_M2_GPIO_Port GPIOC
#define ST0_M3_Pin GPIO_PIN_2
#define ST0_M3_GPIO_Port GPIOC
#define ST0_STBY_Pin GPIO_PIN_0
#define ST0_STBY_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_5
#define RST_GPIO_Port GPIOC
#define DC_Pin GPIO_PIN_0
#define DC_GPIO_Port GPIOB
#define ST0_M0_Pin GPIO_PIN_8
#define ST0_M0_GPIO_Port GPIOC
#define ST0_M1_Pin GPIO_PIN_9
#define ST0_M1_GPIO_Port GPIOC
#define ST0_EN_Pin GPIO_PIN_8
#define ST0_EN_GPIO_Port GPIOA
#define ST1_EN_Pin GPIO_PIN_4
#define ST1_EN_GPIO_Port GPIOB
#define ST1_STBY_Pin GPIO_PIN_5
#define ST1_STBY_GPIO_Port GPIOB
#define ST1_M0_Pin GPIO_PIN_6
#define ST1_M0_GPIO_Port GPIOB
#define ST1_M1_Pin GPIO_PIN_7
#define ST1_M1_GPIO_Port GPIOB
#define ST1_M2_Pin GPIO_PIN_8
#define ST1_M2_GPIO_Port GPIOB
#define ST1_M3_Pin GPIO_PIN_9
#define ST1_M3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
