/**
 * @file sdi12_driver.c
 * @brief SDI12 driver
 *        Refer: "https://semios.atlassian.net/wiki/x/IQCPU"
 *               "https://semios.atlassian.net/wiki/spaces/EM/pages/1509097647/SDI12+Driver+Design+v2.0"
 */

//#include "FreeRTOS.h"
//#include "semphr.h"

#include "sdi12_driver.h"
#include "sdi12Bus_communication.h"

#include "sdi_logging.h"

#define SDI_MAIN_PWR_WAIT_MS           (2000) /**< @brief SDI12 main power line delay */
#define SDI_DISABLE_PWR_MS             (1800) /**< @brief SDI12 disable slot delay*/
#define SDI_SENSOR_PWR_WAIT_MS         (1200) /**< @brief SDI12 sensor power slot delay */

#define STD_WAIT_SENRESP_MS            (150)  /**< @brief SDI12 wait time for response as per standard*/
#define SDI_STANDARD_WAIT_MS           (750)  /**< @brief SDI12 standard wait time for Address change and Identification command response as per standard */
#define WAIT_DATA_READY_MS             (2000) /**< @brief data read wait time */

//Address related defines
#define ADDRESS_CHAR_IN_RESP           0   /**< @brief  character in response */
#define ALLOWED_ADDRESS_MIN            '0' /**< @brief lower limit of valid address range */
#define ALLOWED_ADDRESS_MAX            '3' /**< @brief upper limit of valid address range */

// Measurement response related defines
#define MEAS_READY_TIME_CHAR1          1 /**< @brief measurement ready in - character 1*/
#define MEAS_READY_TIME_CHAR2          2 /**< @brief measurement ready in - character 2*/
#define MEAS_READY_TIME_CHAR3          3 /**< @brief measurement ready in - character 3*/
#define MEAS_NUMBER_CHAR_COMMON        4 /**< @brief number of measurement character  common for M & C response */
#define MEAS_NUMBER_CHAR_CONCURRENT    5 /**< @brief number of measurement character for concurrent response*/

#define MIN_RESP_LEN_IDENTITY          20

extern UART_HandleTypeDef huart1;

char address_buf;

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

/**
 * @brief structure to hold SDI12 reception data and size
 */
typedef struct __Sdi12Receive_t
{
    char   *recBuf;  /**< @brief buffer to receive data */
    uint8_t recSize; /**< @brief size of data received */
}Sdi12Receive;

//Sdi12Receive *Sdi12Resp;

//static SemaphoreHandle_t sdi12CommMutex = NULL; /**< @brief SDI12 communication mutex */

static Sdi12Handle sensorHandle = {0};          /*SDI12 sensor handle*/

extern UART_HandleTypeDef huart6;               //SDI12 interface

/**
 * @brief Function queries address of sensor present on SDI12 bus
 *        NOTE: This function must be used when only one sensor is present on bus
 *              or only single sensor is powered on and able to communicate on bus.
 *
 * NOTE: handleSdi12.sdi12IdNewOrQuery gets updated with address received from query command
 *
 * @return SDI12RetCode_OK if queried address is valid
 *         SDI12RetCode_ADDRESS_INVALID if wrong address is read
 *         SDI12RetCode_ERROR if failed
 */
static SDI12RetCode sdi12QueryAddress();



int sdi12QuerySensorAddress(uint8_t slotNumber, char *sdiAddrFromResp)
{
    NULL_PTR_CHECK(sdiAddrFromResp, SDI12RetCode_INVALID);

    int rv = SDI12RetCode_OK;

    // rv = sdi12GeneralPreCheck();
    // if (rv != SDI12RetCode_OK)
    // {
    //     return rv;
    // }

    // if (slotNumber >= sensorHandle.requiredSlotSupportNum)
    // {
    //     return (SDI12RetCode_INVALID);
    // }

    // if (rv != SDI12RetCode_OK)
    // {
    //     return rv;
    // }

    rv = sdi12QueryAddress();
    if (rv == SDI12RetCode_OK || SDI12RetCode_ADDRESS_IN_USE)
    {
        //return parsed address from response
        //*sdiAddrFromResp = sensorHandle.sdi12IdNewOrQuery;
        *sdiAddrFromResp = address_buf;
    }

    return rv;
}

static SDI12RetCode sdi12QueryAddress()
{
    //?! : a<CR><LF>

    SDI12RetCode retStat = 0; //assume all good

    // if (xSemaphoreTake(sdi12CommMutex, portMAX_DELAY) == 0)
    // {
    //     return SDI12RetCode_ERROR;
    // }

    Sdi12PinConfig pinConfig =
    {
        .sdiTx    = sdiTx,
        .sdiTxEnb = sdiTxEnb
    };



    Sdi12Handle sensorHandle = {
        .Sdi12PinCfg = pinConfig,
        .hUART = &huart1,
        .sdi12IdNewOrQuery = address_buf
    };          /*SDI12 sensor handle*/

    char rxbuf[250];
    int rxSize = 250;

    Sdi12Receive Sdi12Resptemp= {
        .recBuf = rxbuf,
        .recSize = rxSize
    };

    Sdi12Receive* Sdi12Resp = &Sdi12Resptemp;

    retStat = sdi12_BusCommunication(&sensorHandle, Sdi12Resp->recBuf,
                                     &Sdi12Resp->recSize, STD_WAIT_SENRESP_MS,
                                     SDI12_QUERY_ADDR);

    if (retStat == SDI12RetCode_OK)
    {
        if (Sdi12Resp->recBuf[ADDRESS_CHAR_IN_RESP] >= ALLOWED_ADDRESS_MIN &&
            Sdi12Resp->recBuf[ADDRESS_CHAR_IN_RESP] <= ALLOWED_ADDRESS_MAX)
        {
            sensorHandle.sdi12IdNewOrQuery = Sdi12Resp->recBuf[ADDRESS_CHAR_IN_RESP];
            address_buf = Sdi12Resp->recBuf[ADDRESS_CHAR_IN_RESP];

            char ch = Sdi12Resp->recBuf[ADDRESS_CHAR_IN_RESP] - '0'; //Limitation: for now applicable only to range 0-9
            //see if address is already in use
            if (sensorHandle.bitmaskAddressInUse & (1 << ch))
            {
                retStat = SDI12RetCode_ADDRESS_IN_USE;
            }
            else
            {
                // now that we received active address, mark that as in-use
                sensorHandle.bitmaskAddressInUse |= (1 << ch);
                retStat = SDI12RetCode_OK;
            }
        }
        else
        {
            retStat = SDI12RetCode_ADDRESS_INVALID;
        }
    }

    //xSemaphoreGive(sdi12CommMutex);

    return retStat;
}
