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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define K1_Pin GPIO_PIN_0
#define K1_GPIO_Port GPIOA
#define K1_EXTI_IRQn EXTI0_IRQn
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOA
#define TFT_RS_Pin GPIO_PIN_5
#define TFT_RS_GPIO_Port GPIOC
#define TFT_BLK_Pin GPIO_PIN_1
#define TFT_BLK_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define TFT_SCK_Pin GPIO_PIN_13
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_SDO_Pin GPIO_PIN_14
#define TFT_SDO_GPIO_Port GPIOB
#define TFT_SDI_Pin GPIO_PIN_15
#define TFT_SDI_GPIO_Port GPIOB
#define MiniSD_DAT0_Pin GPIO_PIN_8
#define MiniSD_DAT0_GPIO_Port GPIOC
#define MiniSD_DAT1_Pin GPIO_PIN_9
#define MiniSD_DAT1_GPIO_Port GPIOC
#define MicroUSB_DM_Pin GPIO_PIN_11
#define MicroUSB_DM_GPIO_Port GPIOA
#define MicroUSB_DP_Pin GPIO_PIN_12
#define MicroUSB_DP_GPIO_Port GPIOA
#define W25Q16_CS_Pin GPIO_PIN_15
#define W25Q16_CS_GPIO_Port GPIOA
#define MiniSD_DAT2_Pin GPIO_PIN_10
#define MiniSD_DAT2_GPIO_Port GPIOC
#define MiniSD_CD_DAT3_Pin GPIO_PIN_11
#define MiniSD_CD_DAT3_GPIO_Port GPIOC
#define MiniSD_CLK_Pin GPIO_PIN_12
#define MiniSD_CLK_GPIO_Port GPIOC
#define MiniSD_CMD_Pin GPIO_PIN_2
#define MiniSD_CMD_GPIO_Port GPIOD
#define W25Q16_CLK_Pin GPIO_PIN_3
#define W25Q16_CLK_GPIO_Port GPIOB
#define W25Q16_SO_Pin GPIO_PIN_4
#define W25Q16_SO_GPIO_Port GPIOB
#define W25Q16_SI_Pin GPIO_PIN_5
#define W25Q16_SI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
