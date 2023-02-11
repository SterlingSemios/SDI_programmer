/* USER CODE BEGIN Header */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sdi_logging.h"
#include "led_driver.h"
#include "sdi12Bus_communication.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STD_WAIT_SENRESP_MS     (150) /**< @brief SDI12 wait time for response as per standard*/
#define SDI_STANDARD_WAIT_MS    (750) /**< @brief SDI12 standard wait time for Address change and Identification command response as per standard*/
#define MAX_INIT_TIME           (6)   /**< @brief Max time in ms from uart init to address read command*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char         addressArr[]        = {'0', '1', '2', '3'};
int          buttonInterruptFlag = -1;
Sdi12Handle *interruptSdiHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    char         log[250]         = {0};
    char         address_buf      = {0};
    char         rxbuf[250]       = {0};
    int          rxSize           = 250;
    SDI12RetCode retStat          = 0;
    int          startTime        = HAL_GetTick();
    int          initCompleteTime = 0;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    // Init SDI handles
    Sdi12PortPinMap sdiTxEnb =
    {
        .pGpioPort = SDI_TX_ENB_GPIO_Port,
        .gpioPin   = SDI_TX_ENB_Pin
    };

    Sdi12PortPinMap sdiTx =
    {
        .pGpioPort = SDI_TX_GPIO_Port,
        .gpioPin   = SDI_TX_Pin
    };

    Sdi12PinConfig pinConfig =
    {
        .sdiTx    = sdiTx,
        .sdiTxEnb = sdiTxEnb
    };

    Sdi12Handle sensorHandle =
    {
        .Sdi12PinCfg       = pinConfig,
        .hUART             = &huart1,
        .sdi12IdNewOrQuery = address_buf
    };

    Sdi12Receive Sdi12Resp =
    {
        .recBuf  = rxbuf,
        .recSize = rxSize
    };

    interruptSdiHandle = &sensorHandle;

    printLog("          ---");
    printLog("SDI programmer initialized");
    printLog("          ---");
    initCompleteTime = HAL_GetTick();

    // Request SDI-12 sensor address
    retStat = sdi12_BusCommunication(&sensorHandle, Sdi12Resp.recBuf,
                                     &Sdi12Resp.recSize, STD_WAIT_SENRESP_MS,
                                     SDI12_QUERY_ADDR);
    sensorHandle.sdi12IdNewOrQuery = Sdi12Resp.recBuf[0];

    // SDI responses are very sensitive to power on -> first query time
    if (initCompleteTime - startTime > MAX_INIT_TIME)
    {
        sprintf(log, "Warning, init time of %d is too long, SDI may not respond",
                initCompleteTime - startTime);
        printLog("log");
    }

    if (retStat != SDI12RetCode_OK)
    {
        printLog("Error: Failed to read sensor address");
    }
    else
    {
        sprintf(log, "Current sensor address: %c", sensorHandle.sdi12IdNewOrQuery);
        printLog(log);
        setDigit(sensorHandle.sdi12IdNewOrQuery);
    }

    // Let sensor settle before first possible address change
    HAL_Delay(STD_WAIT_SENRESP_MS);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (buttonInterruptFlag == -1)
        {
            HAL_Delay(50);
        }
        else
        {
            sensorHandle.sdi12Address      = sensorHandle.sdi12IdNewOrQuery;
            sensorHandle.sdi12IdNewOrQuery = addressArr[buttonInterruptFlag];

            // Request SDI-12 sensor address change
            retStat = sdi12_BusCommunication(&sensorHandle, Sdi12Resp.recBuf,
                                             &Sdi12Resp.recSize, SDI_STANDARD_WAIT_MS,
                                             SDI12_CHANGE_ADDR);
            sensorHandle.sdi12IdNewOrQuery = Sdi12Resp.recBuf[0];

            if (retStat != SDI12RetCode_OK)
            {
                printLog("Error: Failed to change sensor address");
            }
            else
            {
                setDigit(sensorHandle.sdi12IdNewOrQuery);
                sprintf(log, "New sensor address: %c", sensorHandle.sdi12IdNewOrQuery);
                printLog(log);
            }

            buttonInterruptFlag = -1;
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 1200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | SDI_TX_ENB_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pin : ButtonUp_Pin */
    GPIO_InitStruct.Pin  = ButtonUp_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ButtonUp_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PB11 PB12 PB13 PB14
     *                       PB15 */
    GPIO_InitStruct.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA9 */
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ButtonDown_Pin */
    GPIO_InitStruct.Pin  = ButtonDown_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ButtonDown_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SDI_TX_ENB_Pin */
    GPIO_InitStruct.Pin   = SDI_TX_ENB_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SDI_TX_ENB_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */
void triggerAddressChange(addrChangeType changeAddress)
{
    int maxAddress = 3;
    int newAddress = 0;

    for (int address = 0; address <= maxAddress; address++)
    {
        if (strncmp(&interruptSdiHandle->sdi12IdNewOrQuery, &addressArr[address], 1) == 0)
        {
            if (changeAddress == UP)
            {
                if (address != 3)
                {
                    newAddress = address + 1;
                }
                else
                {
                    newAddress = 0;
                }
            }
            else if (changeAddress == DOWN)
            {
                if (address != 0)
                {
                    newAddress = address - 1;
                }
                else
                {
                    newAddress = 3;
                }
            }
        }
    }

    buttonInterruptFlag = newAddress;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */

    /* User can add his own implementation to report the file name and line number,
     * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
