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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdint.h"
#define MAX_HW_SLOTS_SDI12    4                                              /**< @brief Maximum number of SDI12 slots present and supported by Hardware */
#define MAX_RESPONSE_SIZE     96                                             /**< @brief maximum sdi12 response size - as per standard max size is 83 for aD! command with CRC */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum
{
    UNREAD = 0,   /**< @brief Sensor address has not been read */
    READ,         /**< @brief Sensor address has been read */
}addrRead;

typedef enum
{
    DOWN = 0,     /**< @brief Address decrement request */
    UP,           /**< @brief Address increment request */
}addrChangeType;

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

/**
 * @brief Function called when a button interrupt is received to increment or decrement sensor address
 *
 * @param[in] changeAddress Whether address will be incremented or decremented
 */
void triggerAddressChange(addrChangeType changeAddress);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ButtonUp_Pin            GPIO_PIN_13
#define ButtonUp_GPIO_Port      GPIOC
#define ButtonUp_EXTI_IRQn      EXTI4_15_IRQn
#define ButtonDown_Pin          GPIO_PIN_1
#define ButtonDown_GPIO_Port    GPIOD
#define ButtonDown_EXTI_IRQn    EXTI0_1_IRQn
#define SDI_TX_ENB_Pin          GPIO_PIN_5
#define SDI_TX_ENB_GPIO_Port    GPIOB
#define SDI_TX_Pin              GPIO_PIN_6
#define SDI_TX_GPIO_Port        GPIOB
#define SDI_RX_Pin              GPIO_PIN_7
#define SDI_RX_GPIO_Port        GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
