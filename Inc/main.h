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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32l4xx.h"
#include "ff.h"

#include "stm32_adafruit_sd.h"
#include "stm32_adafruit_lcd.h"
#include <stdio.h>
#include <stdlib.h>

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "fatfs_storage.h"
#include "waveplayer.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define  MAX_BMP_FILES  25
#define  MAX_BMP_FILE_NAME 11
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define NUCLEO_SPIx_SCK_Pin GPIO_PIN_5
#define NUCLEO_SPIx_SCK_GPIO_Port GPIOA
#define NUCLEO_SPIx_MISO_Pin GPIO_PIN_6
#define NUCLEO_SPIx_MISO_GPIO_Port GPIOA
#define NUCLEO_SPIx_MOSI_Pin GPIO_PIN_7
#define NUCLEO_SPIx_MOSI_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_9
#define SD_CS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define AMP_Pin GPIO_PIN_4
#define AMP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80
/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)
#define AMP_ENABLE() 	  HAL_GPIO_WritePin(  AMP_GPIO_Port,   AMP_Pin, GPIO_PIN_RESET)
#define AMP_DISABLE() 	  HAL_GPIO_WritePin(  AMP_GPIO_Port,   AMP_Pin, GPIO_PIN_SET)
/**
  * @brief  SD Control Interface pins
  */

#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
