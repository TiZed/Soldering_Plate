/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BP_LED_Pin GPIO_PIN_13
#define BP_LED_GPIO_Port GPIOC
#define Thermistor_Pin GPIO_PIN_0
#define Thermistor_GPIO_Port GPIOA
#define Contrast_Pin GPIO_PIN_2
#define Contrast_GPIO_Port GPIOA
#define SSR_Pin GPIO_PIN_3
#define SSR_GPIO_Port GPIOA
#define Fan_Pin GPIO_PIN_4
#define Fan_GPIO_Port GPIOA
#define DispD6_Pin GPIO_PIN_10
#define DispD6_GPIO_Port GPIOB
#define DispD7_Pin GPIO_PIN_11
#define DispD7_GPIO_Port GPIOB
#define DispE_Pin GPIO_PIN_12
#define DispE_GPIO_Port GPIOB
#define DispRW_Pin GPIO_PIN_13
#define DispRW_GPIO_Port GPIOB
#define DispRS_Pin GPIO_PIN_14
#define DispRS_GPIO_Port GPIOB
#define EncA_Pin GPIO_PIN_8
#define EncA_GPIO_Port GPIOA
#define EncB_Pin GPIO_PIN_9
#define EncB_GPIO_Port GPIOA
#define EncButton_Pin GPIO_PIN_10
#define EncButton_GPIO_Port GPIOA
#define EncButton_EXTI_IRQn EXTI15_10_IRQn
#define EncLedOrange_Pin GPIO_PIN_15
#define EncLedOrange_GPIO_Port GPIOA
#define EncLedBlue_Pin GPIO_PIN_3
#define EncLedBlue_GPIO_Port GPIOB
#define DispD0_Pin GPIO_PIN_4
#define DispD0_GPIO_Port GPIOB
#define DispD1_Pin GPIO_PIN_5
#define DispD1_GPIO_Port GPIOB
#define DispD2_Pin GPIO_PIN_6
#define DispD2_GPIO_Port GPIOB
#define DispD3_Pin GPIO_PIN_7
#define DispD3_GPIO_Port GPIOB
#define DispD4_Pin GPIO_PIN_8
#define DispD4_GPIO_Port GPIOB
#define DispD5_Pin GPIO_PIN_9
#define DispD5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Disp_Port  GPIOB

typedef enum {DEV_ERROR = 0, SAFE_TEMP, HOT_TEMP} device_state_t ;

void USB_CDC_RxHandler(uint8_t *, uint32_t) ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
