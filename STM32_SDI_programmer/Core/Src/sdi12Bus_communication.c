/**
 * Copyright (C) 2021 SemiosBIO Technologies Inc.
 * All Rights Reserved.
 *
 * Unauthorized copying of these files via any medium is strictly prohibited.
 * Proprietary & Confidential
 */

/**
 * @file sdi12Bus_communication.c
 * @brief SDI12 bus communication driver (This is a stripped down version of the NodeY v3 SDI12 Driver)
 *        Refer: "https://semios.atlassian.net/wiki/spaces/EM/pages/1509097647/SDI12+Driver+Design+v2.0"
 */

#include "sdi12Bus_communication.h"
#include "sdi_logging.h"

#define SDI12_BREAK_MS         (35)                 /**< @brief SDI12 space - break */
#define SDI12_MARKING_MS       (25)                 /**< @brief SDI12 mark */
#define SDI12_TX_TIMEOUT_MS    (40)                 /**< @brief SDI12 transmission timeout */
#define SDI12_STD_RESP_WAIT    (150)

#define SDI_GPIO_MODE          0                    /**< @brief GPIO mode for sdi_tx line */
#define SDI_UART_MODE          1                    /**< @brief UART mode for sdi_tx line */

#define TRANSMISSION_MODE      0                    /**< @brief sdi12 transmission mode */
#define RECEPTION_MODE         1                    /**< @brief sdi12 reception mode */

#define TX_BUFFER_SIZE         16                   /**< @brief Transmission buffer size in bytes */
#define CMD_BUFFER_SIZE        9                    /**< @brief Command buffer size */

static uint8_t          flagEnterInSDI12RxMode = 0; /**< @brief Flag to check if driver is in reception mode or not */
static char             tx_buf[TX_BUFFER_SIZE];     /**< @brief transmission buffer */
static char             rx_buf[MAX_RESPONSE_SIZE];  /**< @brief Reception buffer */
static volatile uint8_t rxCnt;                      /**< @brief received data counts */
volatile uint8_t        flagRxComplete;             /**< @brief Flag indication for response ready/ RX complete */

static Sdi12Transmit sdi12Tx;

static const char strQueryAddr[]  = "?!";                     /**< @brief Query address command */
static const char strChangeAddr[] = "%cA%c!";                 /**< @brief Change address command */

/**
 * @brief: Function to check parity of data
 *
 * @param[in] byte  data for parity check
 *
 * @retval: 1 - Success 0 - Error
 */
static uint8_t checkParity(char byte);

/**
 * @brief Function to calculates even parity
 *
 * @param[in] byte  data for parity calculation
 *
 * @return parity bit
 */
static uint8_t calculateEvenParity(char byte);

/**
 * @brief Configure given pin in desired mode
 *
 * @param[in] sdiPin SDI12 pinmap configuration
 * @param[in] mode type of sdiPin mode - GPIO / UART
 */
static void sdi12_configurePinMode(Sdi12PortPinMap sdiPin, uint8_t mode);

/**
 * @brief Function to read received data with proper parity
 *
 * @param[out] rxSize Number of bytes received
 * @param[out] sdiRxData fill up received data
 * @param[in] responseDelay response wait time
 *
 * @return SDI12RetCode type
 *         SDI12RetCode_RX_ERROR - RX related error
 *         SDI12RetCode_OK - reception and data reading is successful
 */
static SDI12RetCode sdi12_readResponse(uint8_t *rxSize, char *sdiRxData,
                                       uint16_t responseDelay);

/**
 * @brief Function to create desired command with appropriate addresses
 *
 * @param[in] hSdi12 SDI12 handle
 * @param[in] commandNum SDI12Cmd enum type for command to be created
 *
 * @return SDI12RetCode status
 *         SDI12RetCode_OK : command created properly
 *         SDI12RetCode_ERROR : in case of error
 *         SDI12RetCode_INVALID : in all other cases
 */
static SDI12RetCode sdi12_createCmdBuf(Sdi12Handle *hSdi12,
                                       SDI12Cmd commandNum);

/**
 * @brief This function issues Break - 'spacing' & 'Mark' signals necessary to wakeup sensors on SDI line
 *        There is a proper sequence of events and timings that we need to follow to make sure sensors wake up properly.
 *        This basically is a 'wakeup' signal to sensors on SDI line as in default state they are in low power mode(sleep)
 *
 * @param[in] hSdiUart UART handle
 * @param[in] sdiPinMap sdi12 pin map configuration
 *
 * @return SDI12RetCode status
 *         SDI12RetCode_OK: all good
 *         SDI12RetCode_INVALID : in other cases
 */
static SDI12RetCode sdi12_wakeUpSignal(UART_HandleTypeDef *hSdiUart,
                                       Sdi12PinConfig *sdiPinMap);

/**
 * @brief Function to start SDI12 transmission
 *
 * @param[in] hSdi12 SDI12 handle
 *
 * @return SDI12RetCode_OK if Success
 *         SDI12RetCode_TX_ERROR if transmission timed out
 *         SDI12RetCode_INVALID in other cases
 */
static SDI12RetCode sdi12_sendCmd(Sdi12Handle *hSdi12);

void sdi12_receiveRxByteFromIsr(char sdi_rx_byte)
{
    // See if driver has entered in Rx mode
    if (flagEnterInSDI12RxMode == 1)
    {
        if (sdi_rx_byte == '\n')
        {
            rx_buf[rxCnt - 1] = '\0'; //NULL terminate received string
            flagRxComplete    = 1;
        }
        else
        {
            rx_buf[rxCnt++] = sdi_rx_byte;
        }
    } //collect actual valid response
    else
    {
        flagRxComplete = 0;
        rxCnt          = 0;
    } //neglect echoed back data
}

static SDI12RetCode sdi12_createCmdBuf(Sdi12Handle *hSdi12, SDI12Cmd commandNum)
{
    NULL_PTR_CHECK(hSdi12, SDI12RetCode_INVALID);

    char buf[CMD_BUFFER_SIZE] = {'\0'};

    memset(tx_buf, '\0', strlen(tx_buf));
    switch (commandNum)
    {
    case SDI12_QUERY_ADDR:
        memcpy(&buf[0], strQueryAddr, sizeof(strQueryAddr));
        sdi12Tx.size = sprintf(tx_buf, &buf[0]);
        break;

    case SDI12_CHANGE_ADDR:
        memcpy(&buf[0], strChangeAddr, sizeof(strChangeAddr));
        sdi12Tx.size = sprintf(tx_buf, &buf[0], hSdi12->sdi12Address,
                               hSdi12->sdi12IdNewOrQuery);
        break;

    default:
        break;
    }

    // Start Transmission mode
    sdi12_WritePin(&hSdi12->Sdi12PinCfg.sdiTxEnb, TRANSMISSION_MODE);
    flagEnterInSDI12RxMode = 0;

    printDebug("txBuf:");
    printDebug(tx_buf);

    sdi12Tx.pTxBuf = &tx_buf[0];
    ++sdi12Tx.size;

    return SDI12RetCode_OK; // all good
}

static SDI12RetCode sdi12_sendCmd(Sdi12Handle *hSdi12)
{
    NULL_PTR_CHECK(hSdi12, SDI12RetCode_INVALID);

    HAL_StatusTypeDef txRet;
    char byte;

    //clean up previous state, if any
    flagRxComplete = 0;
    memset(rx_buf, '\0', strlen(rx_buf));
    rxCnt = 0;

    while (sdi12Tx.size)
    {
        if (1 < sdi12Tx.size)
        {
            byte  = *(sdi12Tx.pTxBuf++);
            byte |= (calculateEvenParity(byte) << 7);

            printDebug("Sending byte:");
            printDebug(&byte);
            txRet = HAL_UART_Transmit((hSdi12->hUART), (uint8_t *)&byte, 1,
                                      SDI12_TX_TIMEOUT_MS);
            HAL_Delay(1); // to finish having Tx echoed back data on Rx properly
            if (txRet != HAL_OK)
            {
                break;
            }
        }
        --sdi12Tx.size;
    }

    if (txRet != HAL_OK)
    {
        return (SDI12RetCode_TX_ERROR);
    }
    else
    {
        printDebug("Entering RX reception mode");
        sdi12_WritePin(&hSdi12->Sdi12PinCfg.sdiTxEnb, RECEPTION_MODE);
        flagEnterInSDI12RxMode = 1;

        return (SDI12RetCode_OK);
    }
}

static SDI12RetCode sdi12_readResponse(uint8_t *rxSize, char *sdiRxData,
                                       uint16_t responseDelay)
{
    NULL_PTR_CHECK(sdiRxData, SDI12RetCode_ERROR);

    //
    HAL_Delay(responseDelay);

    //clean previous responses, if any
    memset(sdiRxData, '\0', strlen(sdiRxData));
    *rxSize = 0;

    if (flagRxComplete != 1 || rxCnt == 0 || rxCnt > MAX_RESPONSE_SIZE - 1)
    {
        return (SDI12RetCode_RX_ERROR);
    }

    // get number of bytes received
    *rxSize = rxCnt;

    for (uint8_t symbol = 0; symbol <= rxCnt; symbol++)
    {
        if (checkParity(rx_buf[symbol]))
        {
            sdiRxData[symbol] = (rx_buf[symbol] & 0x7F);
        }
        else
        {
            sdiRxData[symbol] = rx_buf[symbol];
        }
        // SemiosLogInfo("SSMITH reading: %c", rx_buf[symbol]);
    }

    return SDI12RetCode_OK;
}

static SDI12RetCode sdi12_wakeUpSignal(UART_HandleTypeDef *hSdiUart,
                                       Sdi12PinConfig *sdiPinMap)
{
    NULL_PTR_CHECK(hSdiUart, SDI12RetCode_INVALID);
    NULL_PTR_CHECK(sdiPinMap, SDI12RetCode_INVALID);

    __HAL_UART_DISABLE(hSdiUart);
    HAL_Delay(1);
    sdi12_WritePin(&sdiPinMap->sdiTxEnb, 0);
    sdi12_configurePinMode((sdiPinMap->sdiTx), SDI_GPIO_MODE); // making TX 0 in this configuration
    HAL_Delay(SDI12_BREAK_MS);
    sdi12_WritePin(&(sdiPinMap->sdiTx), 1);
    HAL_Delay(SDI12_MARKING_MS);
    sdi12_WritePin(&(sdiPinMap->sdiTxEnb), 1);
    HAL_Delay(1);
    sdi12_configurePinMode((sdiPinMap->sdiTx), SDI_UART_MODE);
    __HAL_UART_ENABLE(hSdiUart);
    HAL_Delay(1);

    return SDI12RetCode_OK;
}

SDI12RetCode sdi12_BusCommunication(Sdi12Handle *hSdi12, char *rxBuf,
                                    uint8_t *rxSize, uint16_t rxTimeout,
                                    SDI12Cmd commandNum)
{
    NULL_PTR_CHECK(rxBuf, SDI12RetCode_ERROR);
    NULL_PTR_CHECK(hSdi12, SDI12RetCode_INVALID);

    SDI12RetCode rv;

    // wakeup signal for sensors on sdi12 bus
    rv = sdi12_wakeUpSignal(hSdi12->hUART, &(hSdi12->Sdi12PinCfg));
    if (rv != SDI12RetCode_OK)
    {
        return (SDI12RetCode_ERROR);
    }

    //Create respective command with proper address
    sdi12_createCmdBuf(hSdi12, commandNum);

    // Send command
    rv = sdi12_sendCmd(hSdi12);
    if (rv != SDI12RetCode_OK)
    {
        return (SDI12RetCode_TX_ERROR);
    }

    //read received data
    rv = sdi12_readResponse(rxSize, rxBuf, rxTimeout);
    if (rv != SDI12RetCode_OK)
    {
        return (SDI12RetCode_RX_ERROR);
    }

    return (SDI12RetCode_OK);
}

//------------------------------ helper functions ---------------------------


static void sdi12_configurePinMode(Sdi12PortPinMap sdiPin, uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (mode == SDI_GPIO_MODE)
    {
        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /*Configure GPIO pin Output Level - SDI_TX */
        sdi12_WritePin(&sdiPin, GPIO_PIN_RESET); // Reset condition to make Tx '0' to start break to SDI12

        /*Configure GPIO pins :  SDI_TX_Pin */
        GPIO_InitStruct.Pin   = sdiPin.gpioPin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(sdiPin.pGpioPort, &GPIO_InitStruct);
    }
    else
    {
        __HAL_RCC_GPIOB_CLK_DISABLE();
        HAL_GPIO_DeInit(sdiPin.pGpioPort, sdiPin.gpioPin);

        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = sdiPin.gpioPin;;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
        HAL_GPIO_Init(sdiPin.pGpioPort, &GPIO_InitStruct);
    }
}

static uint8_t calculateEvenParity(char byte)
{
    uint8_t cnt;
    uint8_t res_even = 0;

    for (cnt = 0; cnt < 7; cnt++)
    {
        res_even ^= (byte & 0x01);
        byte    >>= 1;
    }
    return (0 == res_even) ? 0 : 1;
}

static uint8_t checkParity(char byte)
{
    return ((byte >> 7) == calculateEvenParity(byte)) ? 1 : 0;
}

void sdi12_WritePin(const Sdi12PortPinMap *pin, const GPIO_PinState state)
{
    HAL_GPIO_WritePin(pin->pGpioPort, pin->gpioPin, state);
}
