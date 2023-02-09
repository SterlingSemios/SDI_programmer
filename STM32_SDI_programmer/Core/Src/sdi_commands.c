#include "sdi_commands.h"
#include "string.h"
#include "sdi_logging.h"

static char sdi_rx_buf[100];
static int  receive_buf_cnt = 0;

#define SDI12_BREAK_MS         (35)                 /**< @brief SDI12 space - break */
#define SDI12_MARKING_MS       (25)                 /**< @brief SDI12 mark */
#define SDI12_TX_TIMEOUT_MS    (40)                 /**< @brief SDI12 transmission timeout */

#define SDI_GPIO_MODE          0
#define SDI_UART_MODE          1

#define TRANSMISSION_MODE      0                    /**< @brief sdi12 transmission mode */
#define RECEPTION_MODE         1                    /**< @brief sdi12 reception mode */

extern UART_HandleTypeDef huart1;


static SDI12RetCode wakeUpSignal(UART_HandleTypeDef *hSdiUart,
                                 Sdi12PinConfig *sdiPinMap);
static void sdi12_configurePinMode(Sdi12PortPinMap sdiPin, uint8_t mode);
void sdi_WritePin(const Sdi12PortPinMap *pin, const GPIO_PinState state);
static SDI12RetCode sendCmd(Sdi12PinConfig *pinConfig);
static uint8_t calculateEvenParity(char byte);
static uint8_t checkParity(char byte);

static int      inc2     = 0;
static int      timewake = 0;
static int      timesend = 0;

char temp_rx_buf[250];


// int enbPinState = HAL_GPIO_ReadPin(SDI_TX_ENB_GPIO_Port, SDI_TX_ENB_Pin);
// int txPinState = HAL_GPIO_ReadPin(SDI_TX_GPIO_Port, SDI_TX_Pin);
// int rxPinState = HAL_GPIO_ReadPin(SDI_RX_GPIO_Port, SDI_RX_Pin);
//sprintf(log, "SEND state: Tx Enb: %d, Tx: %d, Rx: %d", enbPinState, txPinState, rxPinState);
//printLog(log);







Sdi12PortPinMap sdiTxEnb2 =
{
    .pGpioPort = SDI_TX_ENB_GPIO_Port,
    .gpioPin   = SDI_TX_ENB_Pin
};

Sdi12PortPinMap sdiTx2 =
{
    .pGpioPort = SDI_TX_GPIO_Port,
    .gpioPin   = SDI_TX_Pin
};

void enableSdi()
{
    char           log[350];
    Sdi12PinConfig pinConfig =
    {
        .sdiTx    = sdiTx2,
        .sdiTxEnb = sdiTxEnb2
    };


    wakeUpSignal(&huart1, &pinConfig);
    //HAL_Delay(50);

    sdi_WritePin(&pinConfig.sdiTxEnb, TRANSMISSION_MODE);

    SDI12RetCode ret = sendCmd(&pinConfig);

    if (ret != SDI12RetCode_OK)
    {
        printLog("Bad send");
    }
    HAL_Delay(150);

    readAddress();

    sprintf(log, "Wake - send time: %d", timesend - timewake);
    printLog(log);

    sprintf(log, "rxBuf %s", temp_rx_buf);
    printLog(log);
}

static SDI12RetCode sendCmd(Sdi12Handle *hSdi12)
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

            timesend = HAL_GetTick();
            printLog("SSMITH sending");
            printLog(&byte);
            txRet = HAL_UART_Transmit((hSdi12->hUART), (uint8_t *)&byte, 1,
                                      SDI12_TX_TIMEOUT_MS);
            //SemiosLogInfo("SSMITH sending: %c", byte);
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
        printLog("SSMITH failed send");
        return (SDI12RetCode_TX_ERROR);
    }
    else
    {
        printLog("SSMITH reception mode");
        //SemiosLogInfo("SSMITH Wake - send time: %d", timesend - timewake);
        //Reception mode
        sdi12_WritePin(&hSdi12->Sdi12PinCfg.sdiTxEnb, RECEPTION_MODE);
        //Get start time for wait
       // txEndRxStartTime       = xTaskGetTickCount();
       // flagEnterInSDI12RxMode = 1;

        return (SDI12RetCode_OK);
    }
}

void readAddress()
{
    char log[350];

    if (receive_buf_cnt > 0)
    {

        for (uint8_t symbol = 0; symbol <= receive_buf_cnt; symbol++)
        {
            if (checkParity(sdi_rx_buf[symbol]))
            {
                temp_rx_buf[symbol] = (sdi_rx_buf[symbol] & 0x7F);
            }
            else
            {
                temp_rx_buf[symbol] = sdi_rx_buf[symbol];
            }
        }
    }
    else
    {
        sprintf(log, "No sdi data: %d", inc2);
        inc2++;
    }
}

void receiveRx(char sdiByte)
{
    char log[500];

    //sdi_rx_buf[receive_buf_cnt] = sdiByte;
    //receive_buf_cnt++;

    if (sdiByte == '\n')
    {
        //printLog("SSMITH rx interrupt END MSG");
        sdi_rx_buf[receive_buf_cnt - 1] = '\0'; //NULL terminate received string
        //flagRxComplete    = pdTRUE;
    }
    else
    {
       // printLog("SSMITH rx interrupt");
        sdi_rx_buf[receive_buf_cnt++] = sdiByte;
    }

    // sprintf(log, "%c\'\\0\'", rxbyte);

    // printChar(sdiByte);

    // int enbPinState = HAL_GPIO_ReadPin(SDI_TX_ENB_GPIO_Port, SDI_TX_ENB_Pin);
    // int txPinState  = HAL_GPIO_ReadPin(SDI_TX_GPIO_Port, SDI_TX_Pin);
    // int rxPinState  = HAL_GPIO_ReadPin(SDI_RX_GPIO_Port, SDI_RX_Pin);

//     sprintf(log, "RX state: Tx Enb: %d, Tx: %d, Rx: %d", enbPinState, txPinState, rxPinState);
//     printLog(log);
 }

void sdi_BusComm(Sdi12Handle *hSdi12, char *rxBuf,
                uint8_t *rxSize, uint16_t rxTimeout)
{
    wakeUpSignal(hSdi12->hUART, &(hSdi12->Sdi12PinCfg));
}





static SDI12RetCode wakeUpSignal(UART_HandleTypeDef *hSdiUart,
                                 Sdi12PinConfig *sdiPinMap)
{
    NULL_PTR_CHECK(hSdiUart, SDI12RetCode_INVALID);
    NULL_PTR_CHECK(sdiPinMap, SDI12RetCode_INVALID);

    timewake = HAL_GetTick();
    __HAL_UART_DISABLE(hSdiUart);
    HAL_Delay(1);
    sdi_WritePin(&sdiPinMap->sdiTxEnb, 0);
    sdi12_configurePinMode((sdiPinMap->sdiTx), SDI_GPIO_MODE); // making TX 0 in this configuration
    HAL_Delay(SDI12_BREAK_MS);
    sdi_WritePin(&(sdiPinMap->sdiTx), 1);
    HAL_Delay(SDI12_MARKING_MS);
    sdi_WritePin(&(sdiPinMap->sdiTxEnb), 1);
    HAL_Delay(1);
    sdi12_configurePinMode((sdiPinMap->sdiTx), SDI_UART_MODE);
    __HAL_UART_ENABLE(hSdiUart);
    HAL_Delay(1);

    return SDI12RetCode_OK;
}

static void sdi12_configurePinMode(Sdi12PortPinMap sdiPin, uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (mode == SDI_GPIO_MODE)
    {
        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /*Configure GPIO pin Output Level - SDI_TX */
        sdi_WritePin(&sdiPin, GPIO_PIN_RESET); // Reset condition to make Tx '0' to start break to SDI12

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

void sdi_WritePin(const Sdi12PortPinMap *pin, const GPIO_PinState state)
{
    HAL_GPIO_WritePin(pin->pGpioPort, pin->gpioPin, state);
}

static uint8_t checkParity(char byte)
{
    return ((byte >> 7) == calculateEvenParity(byte)) ? 1 : 0;
}
