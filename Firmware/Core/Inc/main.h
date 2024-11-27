/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_lptim.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define ANKLE_ENCODER_CSn_Pin LL_GPIO_PIN_13
#define ANKLE_ENCODER_CSn_GPIO_Port GPIOC
#define LC_TOP_IN_Pin LL_GPIO_PIN_2
#define LC_TOP_IN_GPIO_Port GPIOC
#define LC_BOT_IN_Pin LL_GPIO_PIN_3
#define LC_BOT_IN_GPIO_Port GPIOC
#define USART_TX_Pin LL_GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define IMU_SCK_Pin LL_GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin LL_GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin LL_GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define ANKLE_CAN_CONTROLLER_CS_Pin LL_GPIO_PIN_4
#define ANKLE_CAN_CONTROLLER_CS_GPIO_Port GPIOC
#define OSCOPE_Pin LL_GPIO_PIN_2
#define OSCOPE_GPIO_Port GPIOB
#define KNEE_CAN_CONTROLLER_CS_Pin LL_GPIO_PIN_12
#define KNEE_CAN_CONTROLLER_CS_GPIO_Port GPIOB
#define KNEE_CAN_CONTROLLER_SCK_Pin LL_GPIO_PIN_13
#define KNEE_CAN_CONTROLLER_SCK_GPIO_Port GPIOB
#define KNEE_CAN_CONTROLLER_MISO_Pin LL_GPIO_PIN_14
#define KNEE_CAN_CONTROLLER_MISO_GPIO_Port GPIOB
#define KNEE_CAN_CONTROLLER_MOSI_Pin LL_GPIO_PIN_15
#define KNEE_CAN_CONTROLLER_MOSI_GPIO_Port GPIOB
#define KNEE_ENCODER_CSn_Pin LL_GPIO_PIN_6
#define KNEE_ENCODER_CSn_GPIO_Port GPIOC
#define ENCODER_DO_Pin LL_GPIO_PIN_7
#define ENCODER_DO_GPIO_Port GPIOC
#define IMU_CS_Pin LL_GPIO_PIN_8
#define IMU_CS_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ANKLE_CAN_CONTROLLER_SCK_Pin LL_GPIO_PIN_3
#define ANKLE_CAN_CONTROLLER_SCK_GPIO_Port GPIOB
#define ANKLE_CAN_CONTROLLER_MISO_Pin LL_GPIO_PIN_4
#define ANKLE_CAN_CONTROLLER_MISO_GPIO_Port GPIOB
#define ANKLE_CAN_CONTROLLER_MOSI_Pin LL_GPIO_PIN_5
#define ANKLE_CAN_CONTROLLER_MOSI_GPIO_Port GPIOB
#define ENCODER_CLK_Pin LL_GPIO_PIN_7
#define ENCODER_CLK_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
