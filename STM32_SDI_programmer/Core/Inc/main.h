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
#define MAX_HW_SLOTS_SDI12               4                                   /**< @brief Maximum number of SDI12 slots present and supported by Hardware */
#define MAX_RESPONSE_SIZE                96                                  /**< @brief maximum sdi12 response size - as per standard max size is 83 for aD! command with CRC */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  typedef enum
  {
      DOWN = 0,
      UP,
  }addrChangeType;

  /**
   * @brief Enum type for possible SDI12 return code / status code
   */
  typedef enum
  {
      SDI12RetCode_BUSY = -10,         /**< @brief SDI12 bus busy */
      SDI12RetCode_PWR_ERROR,          /**< @brief SDI12 power related error */
      SDI12RetCode_TX_ERROR,           /**< @brief SDI12 transmission error */
      SDI12RetCode_RX_ERROR,           /**< @brief SDI12 reception error */
      SDI12RetCode_ADDRESS_IN_USE,     /**< @brief SDI12 address already in use */
      SDI12RetCode_ADDRESS_DUPLICATE,  /**< @brief SDI12 address is duplicate - mainly used for address change */
      SDI12RetCode_ADDRESS_INVALID,    /**< @brief SDI12 address invalid */
      SDI12RetCode_CRC_ERROR,          /**< @brief SDI12 command response CRC does not match */
      SDI12RetCode_INVALID,            /**< @brief SDI12 unknown / invalid behavior */
      SDI12RetCode_ERROR,              /**< @brief SDI12 general failure or error */
      SDI12RetCode_OK,                 /**< @brief SDI12 communication OK */
  }SDI12RetCode;

  /**
   * @brief To specify port and pin mapping for sdi12
   */
  typedef struct __Sdi12PortPinMap_t
  {
      GPIO_TypeDef *pGpioPort; /**< @brief gpio port */
      uint16_t      gpioPin;   /**< @brief gpio pin number */
  }
  Sdi12PortPinMap;

  /**
   * @brief sdi12 pin configuration
   */
  typedef struct __Sdi12PinConfig_t
  {
      Sdi12PortPinMap sdiPwrMain;                      /**< @brief sdi12 5V power pin */
      Sdi12PortPinMap sdiTxEnb;                        /**< @brief sdi12 transmit enable pin */
      Sdi12PortPinMap sdiTx;                           /**< @brief sdi12 transmission pin */
  }
  Sdi12PinConfig;

  /**
   * @brief structure for sdi12 handle
   */
  typedef struct __Sdi12Handle_t
  {
      UART_HandleTypeDef *hUART;                  /**< @brief UART handle */
      Sdi12PinConfig      Sdi12PinCfg;            /**< @brief sdi12 bus related pin configuration */
      char                sdi12Address;           /**< @brief SDI12 sensor address */
      char                sdi12IdNewOrQuery;      /**< @brief This field is used for query and address change command only */
  }
  Sdi12Handle;

  typedef struct __Sdi12Receive_t
  {
      char   *recBuf;  /**< @brief buffer to receive data */
      uint8_t recSize; /**< @brief size of data received */
  }Sdi12Receive;

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
void triggerAddressChange(addrChangeType changeAddress);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ButtonUp_Pin GPIO_PIN_13
#define ButtonUp_GPIO_Port GPIOC
#define ButtonUp_EXTI_IRQn EXTI4_15_IRQn
#define ButtonDown_Pin GPIO_PIN_1
#define ButtonDown_GPIO_Port GPIOD
#define ButtonDown_EXTI_IRQn EXTI0_1_IRQn
#define SDI_TX_ENB_Pin GPIO_PIN_5
#define SDI_TX_ENB_GPIO_Port GPIOB
#define SDI_TX_Pin GPIO_PIN_6
#define SDI_TX_GPIO_Port GPIOB
#define SDI_RX_Pin GPIO_PIN_7
#define SDI_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
