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

#include "stm32f4xx_nucleo.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define TMC_CS0_Pin GPIO_PIN_0
#define TMC_CS0_GPIO_Port GPIOC
#define TMC_STEP0_Pin GPIO_PIN_1
#define TMC_STEP0_GPIO_Port GPIOC
#define TMC_DIR0_Pin GPIO_PIN_2
#define TMC_DIR0_GPIO_Port GPIOC
#define TMC_CS1_Pin GPIO_PIN_3
#define TMC_CS1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMC_STEP1_Pin GPIO_PIN_4
#define TMC_STEP1_GPIO_Port GPIOC
#define TMC_DIR1_Pin GPIO_PIN_5
#define TMC_DIR1_GPIO_Port GPIOC
#define TMC_ENN0_Pin GPIO_PIN_6
#define TMC_ENN0_GPIO_Port GPIOC
#define TMC_ENN1_Pin GPIO_PIN_7
#define TMC_ENN1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Motor 0 (Z) : TMC_CS0/STEP0/DIR0/ENN0, TIM2, AS5600 on I2C3 (PA8 SCL, PC9 SDA)
 * Motor 1 (X) : TMC_CS1/STEP1/DIR1/ENN1, TIM5, AS5600 on I2C1 (PB8 SCL, PB9 SDA)
 * Shared SPI2 : PB13 SCK, PB14 MISO, PB15 MOSI (mode 3). See wiring.md. */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
